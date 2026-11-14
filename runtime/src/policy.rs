// SPDX-License-Identifier: Apache-2.0
//! Running a learned policy: observation in, joint targets out.
//!
//! A policy is what trainnr's `export_deployment` writes: a small neural network in ONNX (`policy.onnx`) and a
//! manifest (`deploy.json`, schema `trainnr-deploy/1`) that says exactly what its inputs and outputs mean. The manifest is checked against the model and the robot before anything runs: a policy trained
//! for another joint order or observation layout is refused, not run.
//!
//! Inference uses `tract`, a pure-Rust ONNX runtime, so the runtime stays one static binary.
//!
//! What a policy outputs is a *wish*. Its targets still pass through the rigrs guards in this runtime and through
//! the firmware's own safety layer before a servo sees them.

use std::fmt;
use std::fs;
use std::path::Path;
use std::time::Instant;

use std::path::PathBuf;

use serde_json::Value;
use tract_onnx::prelude::*;

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
    /// The command: walk speed forward, sideways and turn rate for a walk; `size` values.
    Command { size: usize },
}

impl Term {
    pub fn size(self) -> usize {
        match self {
            Term::AngularVelocity | Term::ProjectedGravity => 3,
            Term::JointPositions | Term::JointVelocities | Term::LastAction => POLICY_JOINTS,
            Term::Command { size } => size,
        }
    }
}

/// One observation term with what the trainer did to it: clipped to +- `clip`, then multiplied by `scale`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Observation {
    pub term: Term,
    pub scale: f64,
    pub clip: Option<f64>,
}

/// The deployment format this runtime reads: what trainnr's `export_deployment` writes beside `policy.onnx`.
pub const DEPLOY_SCHEMA: &str = "trainnr-deploy/1";
const ACTION_KIND: &str = "joint position target = default_pos + scale * action";

/// What a policy means, read from a trainnr deployment (`deploy.json`, schema `trainnr-deploy/1`). Everything this
/// runtime cannot honour exactly is refused when it loads, not approximated.
#[derive(Debug, Clone)]
pub struct Manifest {
    /// The policy's version, as trainnr stamps it.
    pub name: String,
    /// The rate the policy was trained at; the loop must run at this rate.
    pub control_hz: f64,
    /// The joints the outputs drive, in output order.
    pub joints: Vec<String>,
    /// The pose an all-zero action means, radians, one per joint.
    pub default_pose: Vec<f64>,
    /// target = default pose + offset + action * scale (radians), per joint.
    pub action_scale: Vec<f64>,
    pub action_offset: Vec<f64>,
    /// Actions are clipped to +- this before scaling, if set.
    pub action_clip: Option<f64>,
    pub observation: Vec<Observation>,
    /// The network, beside the manifest.
    pub onnx: PathBuf,
    /// The command ranges the policy was trained on (forward, sideways, turn), if the deployment states them.
    pub command_ranges: Option<[(f64, f64); 3]>,
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

fn range(v: &Value) -> Option<(f64, f64)> {
    let a = v.as_array()?;
    Some((a.first()?.as_f64()?, a.get(1)?.as_f64()?))
}

impl Manifest {
    /// Read a deployment: its folder, or its `deploy.json`.
    pub fn load(path: impl AsRef<Path>) -> Result<Self, PolicyError> {
        let path = path.as_ref();
        let file = if path.is_dir() { path.join("deploy.json") } else { path.to_path_buf() };
        let text = fs::read_to_string(&file).map_err(|e| PolicyError::Load(format!("{}: {e}", file.display())))?;
        let d: Value = serde_json::from_str(&text).map_err(|e| PolicyError::Load(format!("{}: {e}", file.display())))?;
        Self::parse(&d, file.parent().unwrap_or(Path::new(".")))
    }

    /// Interpret a parsed `deploy.json` whose folder is `dir`.
    pub fn parse(d: &Value, dir: &Path) -> Result<Self, PolicyError> {
        let schema = at(d, &["schema"]).as_str().unwrap_or("none");
        if schema != DEPLOY_SCHEMA {
            return Err(bad(format!("the deployment's schema is {schema:?}; this runtime reads {DEPLOY_SCHEMA:?}")));
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
        let action_clip = match at(d, &["action", "clip"]) {
            Value::Null => None,
            v => Some(v.as_f64().filter(|c| *c > 0.0).ok_or_else(|| bad("action.clip must be a positive number or null"))?),
        };
        let mut observation = Vec::new();
        for o in at(d, &["observations"]).as_array().ok_or_else(|| bad("observations must be a list"))? {
            let name = at(o, &["name"]).as_str().unwrap_or("?");
            let source = at(o, &["source"]).as_str().unwrap_or("?");
            let width = at(o, &["width"]).as_u64().and_then(|w| usize::try_from(w).ok()).ok_or_else(|| bad(format!("observation {name}: no width")))?;
            let term = match (name, source) {
                ("base_ang_vel", s) if s.starts_with("sensor ") => Term::AngularVelocity,
                ("projected_gravity", "projected_gravity") => Term::ProjectedGravity,
                // `biased`: in training the encoder's calibration error is simulated; a real reading carries its own.
                ("joint_pos", "joint_pos_rel") => Term::JointPositions,
                ("joint_vel", "joint_vel_rel") => Term::JointVelocities,
                ("actions", "last_action") => Term::LastAction,
                ("command", s) if s.starts_with("command ") => Term::Command { size: width },
                _ => return Err(bad(format!("observation {name} ({source}) is not one this runtime can measure"))),
            };
            if term.size() != width {
                return Err(bad(format!("observation {name} is {width} wide; this robot gives {}", term.size())));
            }
            if at(o, &["history_length"]).as_u64().unwrap_or(0) != 0 {
                return Err(bad(format!("observation {name} has a history; this runtime keeps none")));
            }
            let scale = at(o, &["scale"]).as_f64().ok_or_else(|| bad(format!("observation {name}: scale must be one number")))?;
            let clip = match at(o, &["clip"]) {
                Value::Null => None,
                v => Some(v.as_f64().ok_or_else(|| bad(format!("observation {name}: clip must be one number or null")))?),
            };
            observation.push(Observation { term, scale, clip });
        }
        let twist = at(d, &["commands", "twist"]);
        let command_ranges = match (range(at(twist, &["lin_vel_x"])), range(at(twist, &["lin_vel_y"])), range(at(twist, &["ang_vel_z"]))) {
            (Some(x), Some(y), Some(z)) => Some([x, y, z]),
            _ => None,
        };
        let onnx = dir.join(at(d, &["onnx", "file"]).as_str().ok_or_else(|| bad("onnx.file is missing"))?);
        Ok(Manifest {
            name: at(d, &["policy"]).as_str().unwrap_or("unnamed policy").to_string(),
            control_hz: at(d, &["control", "control_hz"]).as_f64().ok_or_else(|| bad("control.control_hz is missing"))?,
            joints,
            default_pose: numbers(at(d, &["joints", "default_pos"]), "joints.default_pos")?,
            action_scale: numbers(at(d, &["action", "scale"]), "action.scale")?,
            action_offset: numbers(at(d, &["action", "offset"]), "action.offset")?,
            action_clip,
            observation,
            onnx,
            command_ranges,
        })
    }

    pub fn observation_size(&self) -> usize {
        self.observation.iter().map(|o| o.term.size()).sum()
    }

    pub fn command_size(&self) -> usize {
        self.observation.iter().map(|o| if let Term::Command { size } = o.term { size } else { 0 }).sum()
    }
}

#[derive(Debug)]
pub enum PolicyError {
    Load(String),
    /// The manifest does not fit this robot or this model. Nothing is run.
    Mismatch(String),
    /// An input the policy needs was not measured this tick. Nothing is run.
    Missing(&'static str),
    Inference(String),
}

impl fmt::Display for PolicyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PolicyError::Load(why) => write!(f, "cannot load the policy: {why}"),
            PolicyError::Mismatch(why) => write!(f, "the policy does not fit this robot: {why}"),
            PolicyError::Missing(what) => write!(f, "not measured this tick: {what}"),
            PolicyError::Inference(why) => write!(f, "inference failed: {why}"),
        }
    }
}

impl std::error::Error for PolicyError {}

/// What the robot measured on one tick, in the firmware's joint order. `None` means "not measured", never zero.
#[derive(Debug, Clone, Default)]
pub struct Sensed {
    /// Joint angles in radians, all fifteen.
    pub joints: Vec<Option<f64>>,
    /// Trunk pitch (nose up positive) and roll (left side up positive) in radians, from the IMU.
    pub pitch_roll: Option<(f64, f64)>,
    /// Body angular velocity in rad/s.
    pub angular_velocity: Option<[f64; 3]>,
}

/// Direction of gravity in the body frame (x forward, y left, z up) for a trunk pitched and rolled by these angles.
/// Upright gives (0, 0, -1). The sign convention must match the simulation the policy was trained in; it is
/// unverified against a real IMU.
pub fn projected_gravity(pitch: f64, roll: f64) -> [f64; 3] {
    [pitch.sin(), -roll.sin() * pitch.cos(), -roll.cos() * pitch.cos()]
}

type Model = Arc<TypedRunnableModel>;

/// A loaded policy, checked against the robot, with the memory it needs between ticks.
pub struct Policy {
    manifest: Manifest,
    model: Model,
    default_pose: [f64; POLICY_JOINTS],
    last_action: [f64; POLICY_JOINTS],
    last_positions: Option<[f64; POLICY_JOINTS]>,
    /// How long the last inference took, in microseconds.
    pub last_inference_us: u64,
}

impl Policy {
    /// Load the deployment's network and check it and its manifest against `robot`.
    pub fn load(manifest: Manifest, robot: &Robot) -> Result<Self, PolicyError> {
        let mismatch = |why: String| Err(PolicyError::Mismatch(why));
        if manifest.joints.len() != POLICY_JOINTS
            || manifest.joints.iter().zip(JOINT_NAMES.iter()).any(|(a, b)| a != b)
        {
            return mismatch(format!(
                "the manifest's joints must be the robot's first {POLICY_JOINTS}, in order: {:?}",
                JOINT_NAMES.get(..POLICY_JOINTS)
            ));
        }
        for (what, values) in [
            ("joints.default_pos", &manifest.default_pose),
            ("action.scale", &manifest.action_scale),
            ("action.offset", &manifest.action_offset),
        ] {
            if values.len() != POLICY_JOINTS || values.iter().any(|v| !v.is_finite()) {
                return mismatch(format!("{what} has {} values, expected {POLICY_JOINTS} finite numbers", values.len()));
            }
        }
        if !(manifest.control_hz.is_finite() && manifest.control_hz > 0.0) {
            return mismatch("control.control_hz must be a positive number".into());
        }
        // The default pose must be a pose the robot can hold.
        let limits: Vec<(f64, f64)> =
            robot.chains.iter().flat_map(|c| c.spec.joints.iter().map(|j| j.travel_radians)).collect();
        let mut default_pose = [0.0; POLICY_JOINTS];
        for (index, (slot, &radians)) in default_pose.iter_mut().zip(&manifest.default_pose).enumerate() {
            let Some(&(low, high)) = limits.get(index) else {
                return mismatch("the robot has fewer joints than the policy drives".into());
            };
            if !radians.is_finite() || radians < low - 1e-9 || radians > high + 1e-9 {
                return mismatch(format!("default pose of `{}` is outside its limits", JOINT_NAMES.get(index).copied().unwrap_or("?")));
            }
            *slot = radians;
        }

        let inference = |e: TractError| PolicyError::Load(e.to_string());
        let graph = tract_onnx::onnx().model_for_path(&manifest.onnx).map_err(inference)?;
        let model = graph.into_optimized().map_err(inference)?.into_runnable().map_err(inference)?;
        let shape = |fact: &TypedFact| fact.shape.as_concrete().map(|dims| dims.to_vec());
        let input = model.model().input_fact(0).map_err(inference)?;
        let output = model.model().output_fact(0).map_err(inference)?;
        if shape(input) != Some(vec![1, manifest.observation_size()]) {
            return mismatch(format!(
                "the model's input is {:?}, the manifest's observation is [1, {}]",
                shape(input),
                manifest.observation_size()
            ));
        }
        if shape(output) != Some(vec![1, POLICY_JOINTS]) {
            return mismatch(format!("the model's output is {:?}, expected [1, {POLICY_JOINTS}]", shape(output)));
        }
        Ok(Policy { manifest, model, default_pose, last_action: [0.0; POLICY_JOINTS], last_positions: None, last_inference_us: 0 })
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
        if command.len() != self.manifest.command_size() {
            return Err(PolicyError::Mismatch(format!(
                "the command has {} values, the policy takes {}",
                command.len(),
                self.manifest.command_size()
            )));
        }
        let mut positions = [0.0; POLICY_JOINTS];
        for (slot, measured) in positions.iter_mut().zip(&sensed.joints) {
            *slot = measured.ok_or(PolicyError::Missing("a joint angle"))?;
        }
        if sensed.joints.len() < POLICY_JOINTS {
            return Err(PolicyError::Missing("a joint angle"));
        }
        // Speeds by difference from the previous tick; zero on the first tick after a reset.
        let mut velocities = [0.0; POLICY_JOINTS];
        if let Some(previous) = self.last_positions {
            for ((v, now), before) in velocities.iter_mut().zip(&positions).zip(&previous) {
                *v = (now - before) / period_seconds;
            }
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
                Term::Command { size } => command.by_ref().take(size).copied().collect(),
            };
            // As the trainer did: clip, then scale.
            obs.extend(values.iter().map(|&x| (o.clip.map_or(x, |c| x.clamp(-c, c)) * o.scale) as f32));
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
        let mut action = [0.0; POLICY_JOINTS];
        if values.len() != POLICY_JOINTS || values.iter().any(|x| !x.is_finite()) {
            return Err(PolicyError::Inference("the action is not fourteen finite numbers".into()));
        }
        for (slot, value) in action.iter_mut().zip(values) {
            *slot = f64::from(*value);
        }
        Ok(action)
    }

    /// One tick: observe, infer, and turn the action into a target for each of the robot's fifteen joints
    /// (radians, firmware order). The beak is not driven by the policy and keeps its measured angle.
    pub fn act(&mut self, sensed: &Sensed, command: &[f64], period_seconds: f64) -> Result<Vec<f64>, PolicyError> {
        let observation = self.observe(sensed, command, period_seconds)?;
        let action = self.infer(observation)?;
        let action = action.map(|a| self.manifest.action_clip.map_or(a, |c| a.clamp(-c, c)));
        let m = &self.manifest;
        let mut targets = Vec::with_capacity(JOINT_NAMES.len());
        for (i, (a, default)) in action.iter().zip(&self.default_pose).enumerate() {
            let scale = m.action_scale.get(i).copied().unwrap_or(0.0);
            let offset = m.action_offset.get(i).copied().unwrap_or(0.0);
            targets.push(default + offset + a * scale);
        }
        for extra in sensed.joints.iter().skip(POLICY_JOINTS) {
            targets.push(extra.ok_or(PolicyError::Missing("a joint angle"))?);
        }
        self.last_action = action;
        Ok(targets)
    }
}
