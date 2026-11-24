// SPDX-License-Identifier: Apache-2.0
//! Going after a thing: where the camera sees it, turned into what the walking policy is asked to do.
//!
//! The policy takes a command (forward speed, sideways speed, turn rate) and knows nothing of toys. Chasing is
//! choosing that command from the thing's direction and distance: turn to face it, walk faster the further away
//! it is, stop when it is within reach. The command never leaves the ranges the policy was trained on.

use crate::policy::Range;

/// How a thing is chased.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Chase {
    /// Within this distance it is caught: stop.
    pub reach_m: f64,
    /// From this distance on, full speed; nearer, slower in proportion.
    pub far_m: f64,
    /// Turn rate asked per radian the thing is off to the side.
    pub turn_per_rad: f64,
    /// Further off to the side than this, turn on the spot before walking.
    pub face_rad: f64,
}

impl Default for Chase {
    fn default() -> Self {
        Chase { reach_m: 0.12, far_m: 0.5, turn_per_rad: 2.5, face_rad: 0.6 }
    }
}

impl Chase {
    /// True when the thing is within reach.
    pub fn caught(&self, distance_m: Option<f64>) -> bool {
        distance_m.is_some_and(|d| d <= self.reach_m)
    }

    /// The command for a thing `bearing` radians to the left of straight ahead and `distance_m` away (`None`: its
    /// size is not known, so only its direction is), inside the policy's `ranges` (forward, sideways, turn).
    pub fn command(&self, bearing: f64, distance_m: Option<f64>, ranges: &[Range; 3]) -> [f64; 3] {
        let [(_, forward_max), _, (turn_min, turn_max)] = *ranges;
        let turn = (self.turn_per_rad * bearing).clamp(turn_min, turn_max);
        if self.caught(distance_m) {
            return [0.0, 0.0, 0.0];
        }
        // How fast for this distance: all the policy has when far, less when near; half when the distance is unknown.
        let pace = distance_m.map_or(0.5, |d| ((d - self.reach_m) / (self.far_m - self.reach_m).max(1e-6)).clamp(0.15, 1.0));
        // And slower the more it has to turn: none at all until it roughly faces the thing.
        let facing = (1.0 - bearing.abs() / self.face_rad).clamp(0.0, 1.0);
        [(forward_max * pace * facing).max(0.0), 0.0, turn]
    }
}
