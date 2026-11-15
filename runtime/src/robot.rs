// SPDX-License-Identifier: Apache-2.0
//! The robot's description, read from the firmware's limits file (`firmware/config/robot.json`) and turned into
//! rigrs joint specifications.
//!
//! rigrs's `arm` crate carries at most six joints per `ArmSpec`, so the duck is described as four chains, each
//! with its own guard. See docs/project/rigrs-for-general-use.md, item 1.

use std::fmt;
use std::fs;
use std::path::Path;

use arm::{collect_joints, ArmSpec, JointSpec};
use serde::Deserialize;

/// Joint order used by the firmware on the wire.
pub const JOINT_NAMES: [&str; 15] = [
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
    "beak",
];

/// The chains, as (name, first joint index, joint count) in the firmware's order.
pub const CHAINS: [(&str, usize, usize); 4] =
    [("left leg", 0, 5), ("neck and head", 5, 4), ("right leg", 9, 5), ("beak", 14, 1)];

/// The SCS0009 reports 1024 steps over 300 degrees of travel. rigrs describes an encoder by its counts per full
/// turn, so this is 1024 * 360 / 300, rounded.
const COUNTS_PER_TURN: u32 = 1229;

#[derive(Debug, Deserialize)]
struct JointEntry {
    name: String,
    servo_id: Option<u8>,
    min_deg: f64,
    max_deg: f64,
    max_speed_dps: f64,
}

#[derive(Debug, Deserialize)]
struct RobotFile {
    joints: Vec<JointEntry>,
}

#[derive(Debug)]
pub enum RobotError {
    Read(std::io::Error),
    Parse(serde_json::Error),
    /// The file's joints are not the firmware's fifteen, in order.
    JointOrder { index: usize, expected: &'static str, found: String },
    JointCount(usize),
    Limits { joint: &'static str },
}

impl fmt::Display for RobotError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RobotError::Read(e) => write!(f, "cannot read the robot file: {e}"),
            RobotError::Parse(e) => write!(f, "cannot parse the robot file: {e}"),
            RobotError::JointOrder { index, expected, found } => {
                write!(f, "joint {index} is `{found}`, expected `{expected}`")
            }
            RobotError::JointCount(n) => write!(f, "the robot file has {n} joints, expected {}", JOINT_NAMES.len()),
            RobotError::Limits { joint } => write!(f, "joint `{joint}` has limits or a speed that are not usable"),
        }
    }
}

impl std::error::Error for RobotError {}

/// One chain of the robot: its joints as a rigrs `ArmSpec` and where they sit in the firmware's joint order.
#[derive(Debug, Clone)]
pub struct Chain {
    pub name: &'static str,
    pub first: usize,
    pub spec: ArmSpec,
}

#[derive(Debug, Clone)]
pub struct Robot {
    pub chains: Vec<Chain>,
    /// How many joints have a servo ID assigned in the file.
    pub ids_assigned: usize,
}

impl Robot {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, RobotError> {
        let text = fs::read_to_string(path).map_err(RobotError::Read)?;
        Self::parse(&text)
    }

    pub fn parse(text: &str) -> Result<Self, RobotError> {
        let file: RobotFile = serde_json::from_str(text).map_err(RobotError::Parse)?;
        if file.joints.len() != JOINT_NAMES.len() {
            return Err(RobotError::JointCount(file.joints.len()));
        }
        let mut specs = Vec::with_capacity(JOINT_NAMES.len());
        for (index, (entry, &expected)) in file.joints.iter().zip(JOINT_NAMES.iter()).enumerate() {
            if entry.name != expected {
                return Err(RobotError::JointOrder { index, expected, found: entry.name.clone() });
            }
            let usable = entry.min_deg.is_finite()
                && entry.max_deg.is_finite()
                && entry.min_deg < entry.max_deg
                && entry.max_speed_dps.is_finite()
                && entry.max_speed_dps > 0.0;
            if !usable {
                return Err(RobotError::Limits { joint: expected });
            }
            specs.push(JointSpec {
                name: expected,
                // 0 is a placeholder: the runtime never addresses a servo itself, the firmware does.
                bus_id: entry.servo_id.unwrap_or(0),
                travel_radians: (entry.min_deg.to_radians(), entry.max_deg.to_radians()),
                max_speed_radians_per_second: entry.max_speed_dps.to_radians(),
                encoder_counts: COUNTS_PER_TURN,
            });
        }
        let ids_assigned = file.joints.iter().filter(|j| j.servo_id.is_some()).count();
        let mut chains = Vec::with_capacity(CHAINS.len());
        for (name, first, count) in CHAINS {
            let joints = specs.get(first..first + count).ok_or(RobotError::JointCount(specs.len()))?;
            let joints = collect_joints(joints.iter().cloned()).ok_or(RobotError::JointCount(count))?;
            chains.push(Chain { name, first, spec: ArmSpec { joints } });
        }
        Ok(Robot { chains, ids_assigned })
    }

    pub fn joints(&self) -> usize {
        self.chains.iter().map(|c| c.spec.joints()).sum()
    }
}
