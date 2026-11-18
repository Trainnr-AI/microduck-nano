// SPDX-License-Identifier: Apache-2.0
//! The limits over several joints, read from the firmware's limits file and applied to the targets the loop sends.
//!
//! The rules themselves are the `duck-limits` crate (`no_std`, shared with a future microcontroller build). This
//! module owns their data (`hip_map`, `knee_ankle`, `both_legs` in `firmware/config/robot.json`) and knows where
//! the legs sit in the firmware's joint order. The firmware enforces the same rules on whatever it is sent; applying
//! them here first means the runtime knows what will actually be commanded, and a policy's recording says so.

use std::fmt;

use duck_limits::{BothLegs, HipAxes, HipMap, KneeAnkle, LegJoints, LegRules, MapError, Settled, LEG_JOINTS};
use serde::Deserialize;

/// Where each leg's five joints start in the firmware's joint order.
pub const LEFT_LEG_FIRST: usize = 0;
pub const RIGHT_LEG_FIRST: usize = 9;

#[derive(Debug, Deserialize)]
struct GridFile {
    hip_pitch: Vec<f32>,
    hip_roll: Vec<f32>,
    hip_yaw: Vec<f32>,
    knee: Vec<f32>,
    ankle: Vec<f32>,
}

#[derive(Debug, Deserialize)]
struct RowFile {
    hip_pitch: f32,
    hip_yaw: f32,
    knee: f32,
    left: String,
    right: String,
}

#[derive(Debug, Deserialize)]
pub(crate) struct HipMapFile {
    grid: GridFile,
    rows: Vec<RowFile>,
}

#[derive(Debug, Deserialize)]
pub(crate) struct KneeAnkleFile {
    knee_over_deg: f32,
    ankle_min_by_knee_deg: Vec<(f32, f32)>,
}

#[derive(Debug, Deserialize)]
pub(crate) struct BothLegsFile {
    yaw_out_over_deg: f32,
    pitch_forward_of_deg: f32,
    band_pitch_forward_of_deg: f32,
    band_roll_in_past_deg: f32,
}

#[derive(Debug, Clone, PartialEq)]
pub enum LimitsError {
    /// A row names a hip pitch, hip yaw or knee that is not on the grid.
    RowOffGrid { row: usize },
    /// A row's `left` or `right` is not one group of 0 / 1 per ankle value, one character per hip roll value.
    RowLayers { row: usize },
    /// A cell of the grid has no row, or two.
    RowCoverage,
    Map(MapError),
}

impl fmt::Display for LimitsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LimitsError::RowOffGrid { row } => write!(f, "hip map row {row} is not on the grid"),
            LimitsError::RowLayers { row } => write!(f, "hip map row {row}: `left` / `right` do not match the grid"),
            LimitsError::RowCoverage => write!(f, "the hip map's rows do not cover its grid exactly once"),
            LimitsError::Map(e) => write!(f, "the leg limits are not usable: {e:?}"),
        }
    }
}

impl std::error::Error for LimitsError {}

/// The leg limits' data, checked when loaded. [`LegLimits::rules`] lends it to the rules.
#[derive(Debug, Clone)]
pub struct LegLimits {
    pitch: Vec<f32>,
    roll: Vec<f32>,
    yaw: Vec<f32>,
    knee: Vec<f32>,
    ankle: Vec<f32>,
    left: Vec<u8>,
    right: Vec<u8>,
    knee_over: f32,
    knee_ankle_rows: Vec<(f32, f32)>,
    both_legs: BothLegs,
}

fn index_of(grid: &[f32], value: f32) -> Option<usize> {
    grid.iter().position(|g| (g - value).abs() < 1e-3)
}

/// One row's `left` or `right`: a group per ankle value, a character per hip roll value, `1` = not allowed.
fn layers(text: &str, ankles: usize, rolls: usize) -> Option<Vec<u8>> {
    let groups: Vec<&str> = text.split(' ').collect();
    if groups.len() != ankles {
        return None;
    }
    groups
        .iter()
        .map(|group| {
            if group.len() != rolls {
                return None;
            }
            group.bytes().enumerate().try_fold(0u8, |bits, (i, c)| match c {
                b'0' => Some(bits),
                b'1' => Some(bits | 1u8.checked_shl(u32::try_from(i).ok()?)?),
                _ => None,
            })
        })
        .collect()
}

impl LegLimits {
    pub(crate) fn from_file(hip: HipMapFile, knee_ankle: KneeAnkleFile, both: BothLegsFile) -> Result<Self, LimitsError> {
        let GridFile { hip_pitch: pitch, hip_roll: roll, hip_yaw: yaw, knee, ankle } = hip.grid;
        let per_ankle = pitch.len() * yaw.len() * knee.len();
        let count = ankle.len() * per_ankle;
        let mut left: Vec<Option<u8>> = vec![None; count];
        let mut right: Vec<Option<u8>> = vec![None; count];
        for (row, entry) in hip.rows.iter().enumerate() {
            let off_grid = LimitsError::RowOffGrid { row };
            let p = index_of(&pitch, entry.hip_pitch).ok_or(off_grid.clone())?;
            let y = index_of(&yaw, entry.hip_yaw).ok_or(off_grid.clone())?;
            let k = index_of(&knee, entry.knee).ok_or(off_grid)?;
            for (text, cells) in [(&entry.left, &mut left), (&entry.right, &mut right)] {
                let bits = layers(text, ankle.len(), roll.len()).ok_or(LimitsError::RowLayers { row })?;
                for (a, bits) in bits.into_iter().enumerate() {
                    let cell = cells
                        .get_mut(((a * pitch.len() + p) * yaw.len() + y) * knee.len() + k)
                        .ok_or(LimitsError::RowCoverage)?;
                    if cell.replace(bits).is_some() {
                        return Err(LimitsError::RowCoverage);
                    }
                }
            }
        }
        let filled = |cells: Vec<Option<u8>>| cells.into_iter().collect::<Option<Vec<u8>>>().ok_or(LimitsError::RowCoverage);
        let limits = LegLimits {
            pitch,
            roll,
            yaw,
            knee,
            ankle,
            left: filled(left)?,
            right: filled(right)?,
            knee_over: knee_ankle.knee_over_deg,
            knee_ankle_rows: knee_ankle.ankle_min_by_knee_deg,
            both_legs: BothLegs {
                yaw_out_over: both.yaw_out_over_deg,
                pitch_forward_of: both.pitch_forward_of_deg,
                band_pitch_forward_of: both.band_pitch_forward_of_deg,
                band_roll_in_past: both.band_roll_in_past_deg,
            },
        };
        limits.rules().map_err(LimitsError::Map)?;
        Ok(limits)
    }

    /// The rules over this data. Checked once when the data was loaded, so this does not fail afterwards.
    pub fn rules(&self) -> Result<LegRules<'_>, MapError> {
        let axes = HipAxes { pitch: &self.pitch, roll: &self.roll, yaw: &self.yaw, knee: &self.knee, ankle: &self.ankle };
        Ok(LegRules {
            hip: HipMap::new(axes, &self.left, &self.right)?,
            knee_ankle: KneeAnkle::new(self.knee_over, &self.knee_ankle_rows)?,
            both_legs: self.both_legs,
        })
    }
}

/// One leg's joints out of a full joint vector in radians, as degrees.
fn leg_degrees(joints: &[f64], first: usize) -> Option<LegJoints> {
    let leg: &[f64; LEG_JOINTS] = joints.get(first..first + LEG_JOINTS)?.try_into().ok()?;
    Some(leg.map(|radians| radians.to_degrees() as f32))
}

/// Pass the legs of `targets` (radians, firmware joint order) through the leg rules, given the pose `previous`
/// the robot was last sent to. A joint the rules hold gets its `previous` angle back, exactly; every other joint
/// keeps its target untouched. `None` if either slice is too short to hold both legs.
pub fn settle_targets(rules: &LegRules<'_>, previous: &[f64], targets: &mut [f64]) -> Option<Settled> {
    let previous_legs = (leg_degrees(previous, LEFT_LEG_FIRST)?, leg_degrees(previous, RIGHT_LEG_FIRST)?);
    let wanted = (leg_degrees(targets, LEFT_LEG_FIRST)?, leg_degrees(targets, RIGHT_LEG_FIRST)?);
    let (mut left, mut right) = wanted;
    let settled = rules.settle((&previous_legs.0, &previous_legs.1), (&mut left, &mut right));
    if !settled.unchanged() {
        for (first, wanted, got) in [(LEFT_LEG_FIRST, wanted.0, left), (RIGHT_LEG_FIRST, wanted.1, right)] {
            for (offset, (wanted, got)) in wanted.iter().zip(got.iter()).enumerate() {
                if wanted != got {
                    *targets.get_mut(first + offset)? = *previous.get(first + offset)?;
                }
            }
        }
    }
    Some(settled)
}
