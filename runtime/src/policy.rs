// SPDX-License-Identifier: Apache-2.0
//! Running a learned policy: observation in, joint targets out.
//!
//! A policy is a small neural network exported to ONNX, plus a manifest that says exactly what its inputs and
//! outputs mean. The manifest is checked against the model and the robot before anything runs: a policy trained
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

use serde::Deserialize;
use tract_onnx::prelude::*;

use crate::robot::{Robot, JOINT_NAMES};

/// Joints a policy drives: every joint except the beak, in the firmware's order.
pub const POLICY_JOINTS: usize = 14;

/// One block of the observation vector, in the order the manifest lists them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(tag = "term", rename_all = "snake_case")]
pub enum Term {
    /// Body angular velocity, rad/s (3 values).
    AngularVelocity,
    /// Direction of gravity in the body frame, unit vector (3 values); (0, 0, -1) when upright.
    ProjectedGravity,
    /// Joint angles relative to the default pose, radians (one per policy joint).
    JointPositions,
    /// Joint speeds, rad/s (one per policy joint).
    JointVelocities,
    /// The action the policy output on the previous tick (one per policy joint).
    LastAction,
    /// The command: whatever the task defines (walk speed, turn rate, head pose ...), `size` values.
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

/// What a policy file means. Lives beside the `.onnx` as JSON.
#[derive(Debug, Clone, Deserialize)]
pub struct Manifest {
    pub name: String,
    /// The rate the policy was trained at; the loop must run at this rate.
    pub control_hz: f64,
    /// The joints the outputs drive, in output order.
    pub joints: Vec<String>,
    /// The pose an all-zero action means, degrees, one per joint.
    pub default_pose_deg: Vec<f64>,
    /// target = default pose + action * this many degrees.
    pub action_scale_deg: f64,
    /// Actions are clipped to +- this before scaling.
    pub action_clip: f64,
    pub observation: Vec<Term>,
}

impl Manifest {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, PolicyError> {
        let text = fs::read_to_string(path).map_err(|e| PolicyError::Load(e.to_string()))?;
        serde_json::from_str(&text).map_err(|e| PolicyError::Load(e.to_string()))
    }

    pub fn observation_size(&self) -> usize {
        self.observation.iter().map(|t| t.size()).sum()
    }

    pub fn command_size(&self) -> usize {
        self.observation.iter().map(|t| if let Term::Command { size } = t { *size } else { 0 }).sum()
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
    /// Load `model_path` and check it and `manifest` against `robot`.
    pub fn load(model_path: impl AsRef<Path>, manifest: Manifest, robot: &Robot) -> Result<Self, PolicyError> {
        let mismatch = |why: String| Err(PolicyError::Mismatch(why));
        if manifest.joints.len() != POLICY_JOINTS
            || manifest.joints.iter().zip(JOINT_NAMES.iter()).any(|(a, b)| a != b)
        {
            return mismatch(format!(
                "the manifest's joints must be the robot's first {POLICY_JOINTS}, in order: {:?}",
                JOINT_NAMES.get(..POLICY_JOINTS)
            ));
        }
        if manifest.default_pose_deg.len() != POLICY_JOINTS {
            return mismatch(format!("default_pose_deg has {} values, expected {POLICY_JOINTS}", manifest.default_pose_deg.len()));
        }
        let finite_positive = |x: f64| x.is_finite() && x > 0.0;
        if !finite_positive(manifest.control_hz) || !finite_positive(manifest.action_scale_deg) || !finite_positive(manifest.action_clip) {
            return mismatch("control_hz, action_scale_deg and action_clip must be positive numbers".into());
        }
        // The default pose must be a pose the robot can hold.
        let limits: Vec<(f64, f64)> =
            robot.chains.iter().flat_map(|c| c.spec.joints.iter().map(|j| j.travel_radians)).collect();
        let mut default_pose = [0.0; POLICY_JOINTS];
        for (index, (slot, degrees)) in default_pose.iter_mut().zip(&manifest.default_pose_deg).enumerate() {
            let radians = degrees.to_radians();
            let Some(&(low, high)) = limits.get(index) else {
                return mismatch("the robot has fewer joints than the policy drives".into());
            };
            if !radians.is_finite() || radians < low - 1e-9 || radians > high + 1e-9 {
                return mismatch(format!("default pose of `{}` is outside its limits", JOINT_NAMES.get(index).copied().unwrap_or("?")));
            }
            *slot = radians;
        }

        let inference = |e: TractError| PolicyError::Load(e.to_string());
        let graph = tract_onnx::onnx().model_for_path(model_path).map_err(inference)?;
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
        for term in &self.manifest.observation {
            match term {
                Term::AngularVelocity => {
                    let w = sensed.angular_velocity.ok_or(PolicyError::Missing("angular velocity"))?;
                    obs.extend(w.iter().map(|&x| x as f32));
                }
                Term::ProjectedGravity => {
                    let (pitch, roll) = sensed.pitch_roll.ok_or(PolicyError::Missing("trunk orientation"))?;
                    obs.extend(projected_gravity(pitch, roll).iter().map(|&x| x as f32));
                }
                Term::JointPositions => {
                    obs.extend(positions.iter().zip(&self.default_pose).map(|(p, d)| (p - d) as f32));
                }
                Term::JointVelocities => obs.extend(velocities.iter().map(|&x| x as f32)),
                Term::LastAction => obs.extend(self.last_action.iter().map(|&x| x as f32)),
                Term::Command { size } => obs.extend(command.by_ref().take(*size).map(|&x| x as f32)),
            }
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
        let clip = self.manifest.action_clip;
        let scale = self.manifest.action_scale_deg.to_radians();
        let mut targets = Vec::with_capacity(JOINT_NAMES.len());
        for (a, default) in action.iter().zip(&self.default_pose) {
            targets.push(default + a.clamp(-clip, clip) * scale);
        }
        for extra in sensed.joints.iter().skip(POLICY_JOINTS) {
            targets.push(extra.ok_or(PolicyError::Missing("a joint angle"))?);
        }
        self.last_action = action.map(|a| a.clamp(-clip, clip));
        Ok(targets)
    }
}
