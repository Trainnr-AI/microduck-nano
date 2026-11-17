// SPDX-License-Identifier: Apache-2.0
//! Running a learned policy: observation in, joint targets out.
//!
//! A policy is what trainnr's `export_deployment` writes: a small neural network in ONNX (`policy.onnx`) and a
//! manifest (`deploy.json`, schema `trainnr-deploy/1`) that says exactly what its inputs and outputs mean. The
//! manifest is checked against the model and the robot before anything runs: a policy trained for another joint
//! order or observation layout is refused, not run.
//!
//! Inference uses `tract`, a pure-Rust ONNX runtime, so the runtime stays one static binary.
//!
//! What a policy outputs is a *wish*. Its targets still pass through the rigrs guards in this runtime and through
//! the firmware's own safety layer before a servo sees them.

use std::fmt;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

use serde_json::Value;
use tract_onnx::prelude::*;

use crate::firmware::Sensed;
use crate::robot::{Robot, JOINT_NAMES};

/// Joints a policy drives: every joint except the beak, in the firmware's order.
pub const POLICY_JOINTS: usize = 14;

/// One block of the observation vector, in the order the manifest lists them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Term {
    /// Body angular velocity, rad/s (3 values): the gyro.
    AngularVelocity,
    /// Direction of gravity in the body frame, unit vector (3 values); (0, 0, -1) when upright.
    ProjectedGravity,
    /// Joint angles relative to the default pose, radians (one per policy joint).
    JointPositions,
    /// Joint speeds, rad/s (one per policy joint).
    JointVelocities,
    /// The action the policy output on the previous tick (one per policy joint).
    LastAction,
    /// The command: walk speed forward (m/s), sideways (m/s) and turn rate (rad/s).
    Command,
}

impl Term {
    pub fn size(self) -> usize {
        match self {
            Term::AngularVelocity | Term::ProjectedGravity | Term::Command => 3,
            Term::JointPositions | Term::JointVelocities | Term::LastAction => POLICY_JOINTS,
        }
    }
}

/// A closed interval `[low, high]`.
pub type Range = (f64, f64);

/// One observation term with what the trainer did to it: multiplied by `scale` (one per value), then clipped.
#[derive(Debug, Clone, PartialEq)]
pub struct Observation {
    pub term: Term,
    pub scale: Vec<f64>,
    pub clip: Option<Range>,
}

/// The deployment format this runtime reads: what trainnr's `export_deployment` writes beside `policy.onnx`,
/// specified in trainnr's docs/87 ("The deployment manifest").
pub const DEPLOY_SCHEMA: &str = "trainnr-deploy/1";
const ACTION_KIND: &str = "joint position target = default_pos + scale * action";
/// The keys `trainnr-deploy/1` defines, and those it requires. Any other key belongs to a schema this runtime does
/// not know, so it is refused rather than guessed at, as trainnr's own loader does.
const KEYS: [&str; 23] = [
    "schema", "stamp_of", "policy", "checkpoint", "run", "robot", "actuator", "task", "dr_basis", "fit", "fit_basis",
    "seed", "certificate", "control", "joints", "action", "observations", "onnx", "scene", "commands",
    "command_basis", "termination", "unitree",
];
const REQUIRED: [&str; 10] = ["schema", "run", "robot", "control", "joints", "action", "observations", "onnx", "scene", "commands"];

/// What a policy means, read from a trainnr deployment (`deploy.json`, schema `trainnr-deploy/1`). Everything this
/// runtime cannot honour exactly is refused when it loads, not approximated.
#[derive(Debug, Clone)]
pub struct Manifest {
    /// The policy's version, as trainnr stamps it.
    pub name: String,
    /// The rate the policy was trained at (one tick per `physics_timestep_s * decimation`); the loop must run at it.
    pub control_hz: f64,
    /// The joints the outputs drive, in output order.
    pub joints: Vec<String>,
    /// The pose an all-zero action means, radians, one per joint.
    pub default_pose: Vec<f64>,
    /// target = default pose + scale * action (radians), per joint.
    pub action_scale: Vec<f64>,
    /// Actions are clipped to this before scaling, if set.
    pub action_clip: Option<Range>,
    pub observation: Vec<Observation>,
    /// The network, beside the manifest.
    pub onnx: PathBuf,
    /// The command ranges the policy was trained on: forward, sideways, turn.
    pub command_ranges: [Range; 3],
}

/// `v` followed down `path`; JSON null where anything along it is missing.
fn at<'a>(v: &'a Value, path: &[&str]) -> &'a Value {
    path.iter().try_fold(v, |v, key| v.get(key)).unwrap_or(&Value::Null)
}

fn bad(why: impl Into<String>) -> PolicyError {
    PolicyError::Mismatch(why.into())
}

fn numbers(v: &Value, what: &str) -> Result<Vec<f64>, PolicyError> {
    v.as_array()
        .and_then(|a| a.iter().map(Value::as_f64).collect::<Option<Vec<f64>>>())
        .ok_or_else(|| bad(format!("{what} must be a list of numbers")))
}

fn strings(v: &Value, what: &str) -> Result<Vec<String>, PolicyError> {
    v.as_array()
        .and_then(|a| a.iter().map(|s| s.as_str().map(str::to_string)).collect::<Option<Vec<String>>>())
        .ok_or_else(|| bad(format!("{what} must be a list of names")))
}

fn range(v: &Value) -> Option<Range> {
    match v.as_array()?.as_slice() {
        [low, high] => Some((low.as_f64()?, high.as_f64()?)).filter(|(low, high)| low <= high),
        _ => None,
    }
}

/// A clip: `[low, high]`, or null for none.
fn clip(v: &Value, what: &str) -> Result<Option<Range>, PolicyError> {
    if v.is_null() {
        return Ok(None);
    }
    range(v).map(Some).ok_or_else(|| bad(format!("{what} must be [low, high] or null")))
}

impl Manifest {
    /// Read a deployment: its folder, or its `deploy.json`.
    pub fn load(path: impl AsRef<Path>) -> Result<Self, PolicyError> {
        let path = path.as_ref();
        let file = if path.is_dir() { path.join("deploy.json") } else { path.to_path_buf() };
        let load = |e: &dyn fmt::Display| PolicyError::Load(format!("{}: {e}", file.display()));
        let text = fs::read_to_string(&file).map_err(|e| load(&e))?;
        let d: Value = serde_json::from_str(&text).map_err(|e| load(&e))?;
        Self::parse(&d, file.parent().unwrap_or(Path::new(".")))
    }

    /// Interpret a parsed `deploy.json` whose folder is `dir`.
    pub fn parse(d: &Value, dir: &Path) -> Result<Self, PolicyError> {
        let schema = at(d, &["schema"]).as_str().unwrap_or("none");
        if schema != DEPLOY_SCHEMA {
            return Err(bad(format!("the deployment's schema is {schema:?}; this runtime reads {DEPLOY_SCHEMA:?}")));
        }
        let keys = d.as_object().ok_or_else(|| bad("deploy.json is not an object"))?;
        if let Some(key) = keys.keys().find(|k| !KEYS.contains(&k.as_str())) {
            return Err(bad(format!("unknown key {key:?}: not part of {DEPLOY_SCHEMA}")));
        }
        if let Some(key) = REQUIRED.iter().find(|k| !keys.contains_key(**k)) {
            return Err(bad(format!("{key} is missing")));
        }
        let joints = strings(at(d, &["joints", "policy_order"]), "joints.policy_order")?;
        let identity: Vec<f64> = (0..joints.len()).map(|i| i as f64).collect();
        if numbers(at(d, &["joints", "action_to_ctrl"]), "joints.action_to_ctrl")? != identity {
            return Err(bad("joints.action_to_ctrl is not the identity: the policy's outputs are not in its joint order"));
        }
        let kind = at(d, &["action", "kind"]).as_str().unwrap_or("none");
        if kind != ACTION_KIND {
            return Err(bad(format!("the action is {kind:?}; this runtime drives {ACTION_KIND:?}")));
        }
        let default_pose = numbers(at(d, &["joints", "default_pos"]), "joints.default_pos")?;
        if numbers(at(d, &["action", "offset"]), "action.offset")? != default_pose {
            return Err(bad(format!("action.offset differs from joints.default_pos; {DEPLOY_SCHEMA} has them equal")));
        }
        let period_s = at(d, &["control", "physics_timestep_s"]).as_f64().zip(at(d, &["control", "decimation"]).as_f64())
            .map(|(dt, decimation)| dt * decimation)
            .filter(|p| p.is_finite() && *p > 0.0)
            .ok_or_else(|| bad("control.physics_timestep_s and control.decimation must be positive numbers"))?;
        let mut observation = Vec::new();
        for o in at(d, &["observations"]).as_array().ok_or_else(|| bad("observations must be a list"))? {
            let name = at(o, &["name"]).as_str().unwrap_or("?");
            let source = at(o, &["source"]).as_str().unwrap_or("?");
            let width = at(o, &["width"]).as_u64().and_then(|w| usize::try_from(w).ok()).ok_or_else(|| bad(format!("observation {name}: no width")))?;
            let term = match source {
                s if s.starts_with("sensor ") && s.ends_with("/imu_ang_vel") => Term::AngularVelocity,
                "projected_gravity" => Term::ProjectedGravity,
                // `biased`: in training the encoder's calibration error is simulated; a real reading carries its own.
                "joint_pos_rel" => Term::JointPositions,
                "joint_vel_rel" => Term::JointVelocities,
                "last_action" => Term::LastAction,
                "command twist" => Term::Command,
                _ => return Err(bad(format!("observation {name} ({source}) is not one this runtime can measure"))),
            };
            if term.size() != width {
                return Err(bad(format!("observation {name} is {width} wide; this robot gives {}", term.size())));
            }
            if at(o, &["history_length"]).as_u64().unwrap_or(0) != 0 {
                return Err(bad(format!("observation {name} has a history; this runtime keeps none")));
            }
            let scale = at(o, &["scale"]);
            let scale = scale.as_f64().map(|s| vec![s; width])
                .or_else(|| numbers(scale, "").ok().filter(|s| s.len() == width))
                .ok_or_else(|| bad(format!("observation {name}: scale must be one number or a list of {width}")))?;
            let clip = clip(at(o, &["clip"]), &format!("observation {name}: clip"))?;
            observation.push(Observation { term, scale, clip });
        }
        let twist = at(d, &["commands", "twist"]);
        let axis = |name| range(at(twist, &[name]));
        let (Some(x), Some(y), Some(z)) = (axis("lin_vel_x"), axis("lin_vel_y"), axis("ang_vel_z")) else {
            return Err(bad("commands.twist must give the lin_vel_x, lin_vel_y and ang_vel_z ranges"));
        };
        let onnx = dir.join(at(d, &["onnx", "file"]).as_str().ok_or_else(|| bad("onnx.file is missing"))?);
        Ok(Manifest {
            name: at(d, &["policy"]).as_str().unwrap_or("unnamed policy").to_string(),
            control_hz: 1.0 / period_s,
            joints,
            default_pose,
            action_scale: numbers(at(d, &["action", "scale"]), "action.scale")?,
            action_clip: clip(at(d, &["action", "clip"]), "action.clip")?,
            observation,
            onnx,
            command_ranges: [x, y, z],
        })
    }

    pub fn observation_size(&self) -> usize {
        self.observation.iter().map(|o| o.term.size()).sum()
    }

    pub fn command_size(&self) -> usize {
        self.observation.iter().filter(|o| o.term == Term::Command).map(|o| o.term.size()).sum()
    }

    /// `command`, if the policy can take it: the right number of values, each inside the range it was trained on
    /// (forward, sideways, turn). A policy asked for a speed it never saw in training does something nobody has
    /// checked.
    pub fn check_command(&self, command: &[f64]) -> Result<(), PolicyError> {
        let refuse = |why: String| Err(PolicyError::Command(why));
        if command.len() != self.command_size() {
            return refuse(format!("{} values given, the policy takes {}", command.len(), self.command_size()));
        }
        for ((i, &v), &(low, high)) in command.iter().enumerate().zip(self.command_ranges.iter().cycle()) {
            if !(low..=high).contains(&v) {
                return refuse(format!("value {i} is {v}; the policy was trained on {low} to {high}"));
            }
        }
        Ok(())
    }
}

#[derive(Debug)]
pub enum PolicyError {
    Load(String),
    /// The manifest does not fit this robot or this model. Nothing is run.
    Mismatch(String),
    /// A command the policy cannot take (wrong size, or outside what it was trained on). Nothing is run.
    Command(String),
    /// An input the policy needs was not measured this tick. Nothing is run.
    Missing(&'static str),
    Inference(String),
}

impl fmt::Display for PolicyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PolicyError::Load(why) => write!(f, "cannot load the policy: {why}"),
            PolicyError::Mismatch(why) => write!(f, "the policy does not fit this robot: {why}"),
            PolicyError::Command(why) => write!(f, "command refused: {why}"),
            PolicyError::Missing(what) => write!(f, "not measured this tick: {what}"),
            PolicyError::Inference(why) => write!(f, "inference failed: {why}"),
        }
    }
}

impl std::error::Error for PolicyError {}

/// Direction of gravity in the body frame (x forward, y left, z up) for a trunk pitched and rolled by these angles.
/// Upright gives (0, 0, -1). The sign convention must match the simulation the policy was trained in; it is
/// unverified against a real IMU.
pub fn projected_gravity(pitch: f64, roll: f64) -> [f64; 3] {
    [pitch.sin(), -roll.sin() * pitch.cos(), -roll.cos() * pitch.cos()]
}

type Model = Arc<TypedRunnableModel>;
type PerJoint = [f64; POLICY_JOINTS];

/// One finite value per policy joint, or why not.
fn per_joint(values: &[f64], what: &str) -> Result<PerJoint, PolicyError> {
    values
        .try_into()
        .ok()
        .filter(|v: &PerJoint| v.iter().all(|x| x.is_finite()))
        .ok_or_else(|| bad(format!("{what} has {} values, expected {POLICY_JOINTS} finite numbers", values.len())))
}

/// A loaded policy, checked against the robot, with the memory it needs between ticks.
pub struct Policy {
    manifest: Manifest,
    model: Model,
    default_pose: PerJoint,
    scale: PerJoint,
    last_action: [f64; POLICY_JOINTS],
    last_positions: Option<[f64; POLICY_JOINTS]>,
    /// How long the last inference took, in microseconds.
    pub last_inference_us: u64,
}

impl Policy {
    /// Load the deployment's network and check it and its manifest against `robot`.
    pub fn load(manifest: Manifest, robot: &Robot) -> Result<Self, PolicyError> {
        let robot_joints: Vec<&str> = JOINT_NAMES.iter().take(POLICY_JOINTS).copied().collect();
        if manifest.joints != robot_joints {
            return Err(bad(format!("the manifest's joints must be the robot's first {POLICY_JOINTS}, in order: {robot_joints:?}")));
        }
        let default_pose = per_joint(&manifest.default_pose, "joints.default_pos")?;
        let scale = per_joint(&manifest.action_scale, "action.scale")?;
        if !(manifest.control_hz.is_finite() && manifest.control_hz > 0.0) {
            return Err(bad("control.control_hz must be a positive number"));
        }
        // The default pose must be a pose the robot can hold.
        let limits: Vec<(f64, f64)> =
            robot.chains.iter().flat_map(|c| c.spec.joints.iter().map(|j| j.travel_radians)).collect();
        for ((&q, name), (low, high)) in default_pose.iter().zip(robot_joints).zip(limits.into_iter().chain(std::iter::repeat((f64::NAN, f64::NAN)))) {
            if !(q >= low - 1e-9 && q <= high + 1e-9) {
                return Err(bad(format!("default pose of `{name}` is outside its limits")));
            }
        }

        let inference = |e: TractError| PolicyError::Load(e.to_string());
        let graph = tract_onnx::onnx().model_for_path(&manifest.onnx).map_err(inference)?;
        let model = graph.into_optimized().map_err(inference)?.into_runnable().map_err(inference)?;
        let shape = |fact: &TypedFact| fact.shape.as_concrete().map(|dims| dims.to_vec());
        let input = model.model().input_fact(0).map_err(inference)?;
        let output = model.model().output_fact(0).map_err(inference)?;
        if shape(input) != Some(vec![1, manifest.observation_size()]) {
            return Err(bad(format!(
                "the model's input is {:?}, the manifest's observation is [1, {}]",
                shape(input),
                manifest.observation_size()
            )));
        }
        if shape(output) != Some(vec![1, POLICY_JOINTS]) {
            return Err(bad(format!("the model's output is {:?}, expected [1, {POLICY_JOINTS}]", shape(output))));
        }
        let last_action = [0.0; POLICY_JOINTS];
        Ok(Policy { manifest, model, default_pose, scale, last_action, last_positions: None, last_inference_us: 0 })
    }

    pub fn manifest(&self) -> &Manifest {
        &self.manifest
    }

    /// Forget the previous tick (after a pause, a fault or a hand-over), so stale speeds and actions are not used.
    pub fn reset(&mut self) {
        self.last_action = [0.0; POLICY_JOINTS];
        self.last_positions = None;
    }

    /// Build the observation vector for this tick. Fails if anything the manifest asks for was not measured.
    pub fn observe(&mut self, sensed: &Sensed, command: &[f64], period_seconds: f64) -> Result<Vec<f32>, PolicyError> {
        self.manifest.check_command(command)?;
        let mut positions = [0.0; POLICY_JOINTS];
        for (i, slot) in positions.iter_mut().enumerate() {
            *slot = sensed.joints.get(i).copied().flatten().ok_or(PolicyError::Missing("a joint angle"))?;
        }
        // Each joint's speed as its servo reported it. Without that report (an exchange that carries none), by
        // difference from the previous tick, zero on the first tick after a reset.
        let mut velocities = [0.0; POLICY_JOINTS];
        for (i, v) in velocities.iter_mut().enumerate() {
            let differenced = self.last_positions.and_then(|previous| Some((positions.get(i)? - previous.get(i)?) / period_seconds));
            *v = sensed.joint_speeds.get(i).copied().flatten().or(differenced).unwrap_or(0.0);
        }
        let mut obs: Vec<f32> = Vec::with_capacity(self.manifest.observation_size());
        let mut command = command.iter();
        for o in &self.manifest.observation {
            let values: Vec<f64> = match o.term {
                Term::AngularVelocity => sensed.angular_velocity.ok_or(PolicyError::Missing("angular velocity"))?.to_vec(),
                Term::ProjectedGravity => {
                    let (pitch, roll) = sensed.pitch_roll.ok_or(PolicyError::Missing("trunk orientation"))?;
                    projected_gravity(pitch, roll).to_vec()
                }
                Term::JointPositions => positions.iter().zip(&self.default_pose).map(|(p, d)| p - d).collect(),
                Term::JointVelocities => velocities.to_vec(),
                Term::LastAction => self.last_action.to_vec(),
                Term::Command => command.by_ref().take(o.term.size()).copied().collect(),
            };
            // As the trainer did: scale, then clip.
            obs.extend(values.iter().zip(&o.scale).map(|(&x, &s)| o.clip.map_or(x * s, |(low, high)| (x * s).clamp(low, high)) as f32));
        }
        if obs.iter().any(|x| !x.is_finite()) {
            return Err(PolicyError::Missing("a finite value (an input was not a number)"));
        }
        self.last_positions = Some(positions);
        Ok(obs)
    }

    /// Run the network on an observation and return the raw action.
    pub fn infer(&mut self, observation: Vec<f32>) -> Result<[f64; POLICY_JOINTS], PolicyError> {
        let inference = |e: TractError| PolicyError::Inference(e.to_string());
        let size = observation.len();
        let start = Instant::now();
        let input = Tensor::from_shape(&[1, size], &observation).map_err(inference)?;
        let outputs = self.model.run(tvec!(input.into())).map_err(inference)?;
        let first = outputs.first().ok_or_else(|| PolicyError::Inference("the model returned nothing".into()))?;
        let plain = first.try_as_plain_ram().map_err(inference)?;
        let values = plain.as_slice::<f32>().map_err(inference)?;
        self.last_inference_us = u64::try_from(start.elapsed().as_micros()).unwrap_or(u64::MAX);
        let action: [f32; POLICY_JOINTS] = values
            .try_into()
            .ok()
            .filter(|a: &[f32; POLICY_JOINTS]| a.iter().all(|x| x.is_finite()))
            .ok_or_else(|| PolicyError::Inference(format!("the action is not {POLICY_JOINTS} finite numbers")))?;
        Ok(action.map(f64::from))
    }

    /// One tick: observe, infer, and turn the action into a target for each of the robot's fifteen joints
    /// (radians, firmware order). The beak is not driven by the policy and keeps its measured angle.
    pub fn act(&mut self, sensed: &Sensed, command: &[f64], period_seconds: f64) -> Result<Vec<f64>, PolicyError> {
        let observation = self.observe(sensed, command, period_seconds)?;
        let action = self.infer(observation)?;
        let action = action.map(|a| self.manifest.action_clip.map_or(a, |(low, high)| a.clamp(low, high)));
        let mut targets: Vec<f64> = action.iter().zip(self.default_pose).zip(self.scale).map(|((a, d), s)| d + s * a).collect();
        for extra in sensed.joints.iter().skip(POLICY_JOINTS) {
            targets.push(extra.ok_or(PolicyError::Missing("a joint angle"))?);
        }
        self.last_action = action;
        Ok(targets)
    }
}
