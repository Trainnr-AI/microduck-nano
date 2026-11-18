// SPDX-License-Identifier: Apache-2.0
//! Limits over several joints, for a biped's legs.
//!
//! A range per joint cannot express most collisions inside a robot: they are combinations. This crate holds the
//! three forms Microduck Nano needs, as data plus a small amount of code:
//!
//! - [`HipMap`]: a grid over hip pitch, hip yaw, knee, ankle and hip roll, one map per leg, saying which poses
//!   were measured free. Between grid values every surrounding grid point must be free (the "stricter
//!   neighbour"); beyond an axis's end the end value is used.
//! - [`KneeAnkle`]: a table over two joints. Past a knee angle, the ankle has a minimum that rises with the knee.
//! - [`BothLegs`]: a rule over the two legs together.
//!
//! [`LegRules::settle`] turns a wanted pose into an allowed one, keeping as much of the move as the rules permit.
//! It is the same decision, branch for branch, as the microcontroller firmware's safety layer
//! (`firmware/app/microduck/sketch/src/duck/safety.cpp`); the two are checked against each other with shared test
//! vectors.
//!
//! Every rule is written in the LEFT leg's signs (hip pitch + = leg back, roll + = out, yaw + = toe out). A right
//! leg's angles have their signs flipped before a rule is applied. Angles are degrees, `f32`.
//!
//! `no_std`, no allocation, no `unsafe`: the maps borrow their data, so the same code serves a table loaded from
//! a file on Linux and a `static` table on a microcontroller. Written to move into rigrs
//! (docs/project/rigrs-for-general-use.md, item 3).
#![no_std]

/// Which leg.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
}

impl Side {
    const fn index(self) -> usize {
        match self {
            Side::Left => 0,
            Side::Right => 1,
        }
    }
}

/// The joints of one leg, in the firmware's order.
pub const LEG_JOINTS: usize = 5;
const YAW: usize = 0;
const ROLL: usize = 1;
const PITCH: usize = 2;
const KNEE: usize = 3;
const ANKLE: usize = 4;

/// One leg's joint angles in degrees and in that leg's own signs: hip yaw, hip roll, hip pitch, knee, ankle.
pub type LegJoints = [f32; LEG_JOINTS];

/// One leg's pose in the left leg's signs, which every rule is written in.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LegPose {
    pub yaw: f32,
    pub roll: f32,
    pub pitch: f32,
    pub knee: f32,
    pub ankle: f32,
}

impl LegPose {
    /// From a leg's joints in its own signs.
    pub fn of(side: Side, joints: &LegJoints) -> Self {
        let sign = match side {
            Side::Left => 1.0,
            Side::Right => -1.0,
        };
        let [yaw, roll, pitch, knee, ankle] = *joints;
        LegPose { yaw: sign * yaw, roll: sign * roll, pitch: sign * pitch, knee: sign * knee, ankle: sign * ankle }
    }

    fn is_finite(&self) -> bool {
        self.yaw.is_finite()
            && self.roll.is_finite()
            && self.pitch.is_finite()
            && self.knee.is_finite()
            && self.ankle.is_finite()
    }
}

/// Why a map's data was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapError {
    /// An axis has no values, or its values do not rise.
    Axis(&'static str),
    /// More hip roll values than a cell has bits (8).
    TooManyRolls,
    /// A leg's cells are not `ankle x pitch x yaw x knee` long.
    CellCount,
    /// A knee x ankle table whose knee values do not rise or whose ankle minimums fall.
    Table,
}

/// Two values are the same grid value within this many degrees.
const GRID_TOLERANCE: f32 = 1e-3;

/// The grid indices around `v`: one index if `v` is on a grid value or beyond the grid's ends, else the two
/// neighbours. `None` for an empty grid.
fn bracket(grid: &[f32], v: f32) -> Option<(usize, usize)> {
    let last = grid.len().checked_sub(1)?;
    if v <= *grid.first()? + GRID_TOLERANCE {
        return Some((0, 0));
    }
    if v >= *grid.get(last)? - GRID_TOLERANCE {
        return Some((last, last));
    }
    for (i, &g) in grid.iter().enumerate() {
        if (v - g).abs() <= GRID_TOLERANCE {
            return Some((i, i));
        }
        if v < g {
            return Some((i.checked_sub(1)?, i));
        }
    }
    Some((last, last))
}

fn rises(grid: &[f32]) -> bool {
    !grid.is_empty() && grid.windows(2).all(|w| matches!(w, [a, b] if a < b))
}

/// Which hip poses are free, for each leg, by hip pitch, hip yaw, knee, ankle and hip roll.
#[derive(Debug, Clone, Copy)]
pub struct HipMap<'a> {
    pitch: &'a [f32],
    roll: &'a [f32],
    yaw: &'a [f32],
    knee: &'a [f32],
    ankle: &'a [f32],
    /// Per leg, `[ankle][pitch][yaw][knee]` flattened; bit `i` set = not allowed at `roll[i]`.
    cells: [&'a [u8]; 2],
}

/// The axes of a [`HipMap`], each rising.
#[derive(Debug, Clone, Copy)]
pub struct HipAxes<'a> {
    pub pitch: &'a [f32],
    pub roll: &'a [f32],
    pub yaw: &'a [f32],
    pub knee: &'a [f32],
    pub ankle: &'a [f32],
}

impl<'a> HipMap<'a> {
    /// `left` and `right` are `[ankle][pitch][yaw][knee]` flattened, one byte per cell: bit `i` set = not allowed
    /// at `roll[i]`.
    pub fn new(axes: HipAxes<'a>, left: &'a [u8], right: &'a [u8]) -> Result<Self, MapError> {
        let named = [
            ("hip pitch", axes.pitch),
            ("hip roll", axes.roll),
            ("hip yaw", axes.yaw),
            ("knee", axes.knee),
            ("ankle", axes.ankle),
        ];
        for (name, grid) in named {
            if !rises(grid) {
                return Err(MapError::Axis(name));
            }
        }
        if axes.roll.len() > 8 {
            return Err(MapError::TooManyRolls);
        }
        let count = axes.ankle.len() * axes.pitch.len() * axes.yaw.len() * axes.knee.len();
        if left.len() != count || right.len() != count {
            return Err(MapError::CellCount);
        }
        Ok(HipMap {
            pitch: axes.pitch,
            roll: axes.roll,
            yaw: axes.yaw,
            knee: axes.knee,
            ankle: axes.ankle,
            cells: [left, right],
        })
    }

    /// Is this pose inside that leg's measured free set? A pose that is not finite is not.
    pub fn allows(&self, side: Side, pose: &LegPose) -> bool {
        self.lookup(side, pose).unwrap_or(false)
    }

    fn lookup(&self, side: Side, pose: &LegPose) -> Option<bool> {
        if !pose.is_finite() {
            return Some(false);
        }
        let (p0, p1) = bracket(self.pitch, pose.pitch)?;
        let (r0, r1) = bracket(self.roll, pose.roll)?;
        let (y0, y1) = bracket(self.yaw, pose.yaw)?;
        let (k0, k1) = bracket(self.knee, pose.knee)?;
        let (a0, a1) = bracket(self.ankle, pose.ankle)?;
        let cells = self.cells.get(side.index())?;
        let (pitches, yaws, knees) = (self.pitch.len(), self.yaw.len(), self.knee.len());
        let mut rolls: u8 = 0;
        for r in r0..=r1 {
            rolls |= 1u8.checked_shl(u32::try_from(r).ok()?)?;
        }
        for a in a0..=a1 {
            for p in p0..=p1 {
                for y in y0..=y1 {
                    for k in k0..=k1 {
                        let cell = *cells.get(((a * pitches + p) * yaws + y) * knees + k)?;
                        if cell & rolls != 0 {
                            return Some(false);
                        }
                    }
                }
            }
        }
        Some(true)
    }
}

/// Knee x ankle: with the knee bent past `knee_over`, the ankle must be at least the minimum of the first row
/// whose knee value is at or above the knee angle.
#[derive(Debug, Clone, Copy)]
pub struct KneeAnkle<'a> {
    knee_over: f32,
    /// (knee, least ankle), knee rising, least ankle not falling.
    rows: &'a [(f32, f32)],
}

impl<'a> KneeAnkle<'a> {
    pub fn new(knee_over: f32, rows: &'a [(f32, f32)]) -> Result<Self, MapError> {
        let ordered = rows.windows(2).all(|w| matches!(w, [(k0, a0), (k1, a1)] if k0 < k1 && a0 <= a1));
        if rows.is_empty() || !ordered || !knee_over.is_finite() {
            return Err(MapError::Table);
        }
        Ok(KneeAnkle { knee_over, rows })
    }

    pub fn allows(&self, knee: f32, ankle: f32) -> bool {
        if !(knee.is_finite() && ankle.is_finite()) {
            return false;
        }
        if knee <= self.knee_over {
            return true;
        }
        let row = self.rows.iter().find(|(k, _)| knee <= *k).or(self.rows.last());
        row.is_some_and(|(_, least)| ankle >= *least)
    }
}

/// Both legs together: with both hips toed out past `yaw_out_over`, the legs must not both be forward of
/// `pitch_forward_of`, nor both forward of `band_pitch_forward_of` with a hip rolled in past
/// `band_roll_in_past`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BothLegs {
    pub yaw_out_over: f32,
    pub pitch_forward_of: f32,
    pub band_pitch_forward_of: f32,
    pub band_roll_in_past: f32,
}

impl BothLegs {
    pub fn allows(&self, left: &LegPose, right: &LegPose) -> bool {
        let finite = [left.yaw, right.yaw, left.pitch, right.pitch, left.roll, right.roll].iter().all(|v| v.is_finite());
        if !finite {
            return false;
        }
        if !(left.yaw > self.yaw_out_over && right.yaw > self.yaw_out_over) {
            return true;
        }
        if left.pitch < self.pitch_forward_of && right.pitch < self.pitch_forward_of {
            return false;
        }
        let roll_in = if left.roll < right.roll { left.roll } else { right.roll };
        !(left.pitch < self.band_pitch_forward_of
            && right.pitch < self.band_pitch_forward_of
            && roll_in < self.band_roll_in_past)
    }
}

/// What [`LegRules::settle`] did with one leg's wanted pose.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LegVerdict {
    /// The wanted pose is allowed and was kept whole.
    Move,
    /// The wanted pose is not allowed; with the ankle held where it was, the rest is.
    AnkleHeld,
    /// Only the ankle moves; the hip and the knee stay where they were.
    HipAndKneeHeld,
    /// Nothing of the wanted pose is allowed; the leg stays where it was.
    Held,
}

/// What [`LegRules::settle`] did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Settled {
    pub left: LegVerdict,
    pub right: LegVerdict,
    /// The knee x ankle rule was one of the reasons, on either leg.
    pub knee_ankle: bool,
    /// The two legs together broke the both-legs rule, and both stay where they were.
    pub both_legs: bool,
}

impl Settled {
    /// Did every joint get the angle that was wanted?
    pub fn unchanged(&self) -> bool {
        self.left == LegVerdict::Move && self.right == LegVerdict::Move && !self.both_legs
    }

    /// The same bits as the firmware's `SafetyFlag` (left leg, right leg, knee x ankle, both legs), so the two
    /// implementations can be compared.
    pub fn firmware_flags(&self) -> u16 {
        let mut flags = 0;
        if self.left != LegVerdict::Move {
            flags |= 1 << 4;
        }
        if self.right != LegVerdict::Move {
            flags |= 1 << 5;
        }
        if self.knee_ankle {
            flags |= 1 << 6;
        }
        if self.both_legs {
            flags |= 1 << 7;
        }
        flags
    }
}

/// All the rules over a biped's legs.
#[derive(Debug, Clone, Copy)]
pub struct LegRules<'a> {
    pub hip: HipMap<'a>,
    pub knee_ankle: KneeAnkle<'a>,
    pub both_legs: BothLegs,
}

impl LegRules<'_> {
    /// Does this leg's pose keep its own rules (hip map and knee x ankle)?
    pub fn leg_allowed(&self, side: Side, joints: &LegJoints) -> bool {
        let pose = LegPose::of(side, joints);
        self.knee_ankle.allows(pose.knee, pose.ankle) && self.hip.allows(side, &pose)
    }

    /// Do the two legs together keep the both-legs rule?
    pub fn both_legs_allowed(&self, left: &LegJoints, right: &LegJoints) -> bool {
        self.both_legs.allows(&LegPose::of(Side::Left, left), &LegPose::of(Side::Right, right))
    }

    /// Is the whole pose allowed?
    pub fn allowed(&self, left: &LegJoints, right: &LegJoints) -> bool {
        self.leg_allowed(Side::Left, left)
            && self.leg_allowed(Side::Right, right)
            && self.both_legs_allowed(left, right)
    }

    /// Make one leg's wanted pose an allowed one, changing as little as possible.
    fn settle_leg(&self, side: Side, previous: &LegJoints, wanted: &mut LegJoints) -> (LegVerdict, bool) {
        if self.leg_allowed(side, wanted) {
            return (LegVerdict::Move, false);
        }
        let pose = LegPose::of(side, wanted);
        let knee_ankle = !self.knee_ankle.allows(pose.knee, pose.ankle);

        let [.., previous_ankle] = *previous;
        let [yaw, roll, pitch, knee, ankle] = *wanted;
        let ankle_held = [yaw, roll, pitch, knee, previous_ankle];
        let [previous_yaw, previous_roll, previous_pitch, previous_knee, _] = *previous;
        let hip_and_knee_held = [previous_yaw, previous_roll, previous_pitch, previous_knee, ankle];
        let verdict = if self.leg_allowed(side, &ankle_held) {
            *wanted = ankle_held;
            LegVerdict::AnkleHeld
        } else if self.leg_allowed(side, &hip_and_knee_held) {
            *wanted = hip_and_knee_held;
            LegVerdict::HipAndKneeHeld
        } else {
            *wanted = *previous;
            LegVerdict::Held
        };
        (verdict, knee_ankle)
    }

    /// Turn the wanted pose of the two legs into an allowed one. Each leg keeps as much of its move as its own
    /// rules allow: all of it; else with the ankle held; else the ankle alone; else it stays where it was. If the
    /// two legs together then break the both-legs rule, both stay where they were.
    ///
    /// The result is an allowed pose whenever `previous` is one. From a pose that is not allowed the legs can only
    /// stay or move into an allowed pose.
    pub fn settle(
        &self,
        previous: (&LegJoints, &LegJoints),
        wanted: (&mut LegJoints, &mut LegJoints),
    ) -> Settled {
        let (left_verdict, left_knee_ankle) = self.settle_leg(Side::Left, previous.0, wanted.0);
        let (right_verdict, right_knee_ankle) = self.settle_leg(Side::Right, previous.1, wanted.1);
        let both_legs = !self.both_legs_allowed(wanted.0, wanted.1);
        if both_legs {
            *wanted.0 = *previous.0;
            *wanted.1 = *previous.1;
        }
        Settled { left: left_verdict, right: right_verdict, knee_ankle: left_knee_ankle || right_knee_ankle, both_legs }
    }
}

// Joint positions inside a `LegJoints`, for callers that index it.
pub const JOINT_HIP_YAW: usize = YAW;
pub const JOINT_HIP_ROLL: usize = ROLL;
pub const JOINT_HIP_PITCH: usize = PITCH;
pub const JOINT_KNEE: usize = KNEE;
pub const JOINT_ANKLE: usize = ANKLE;

#[cfg(test)]
#[allow(clippy::expect_used, clippy::indexing_slicing)]
mod tests {
    use super::*;

    // A small map: pitch -30 / 0 / 30, roll -10 / 0 / 10, yaw 0 / 10, knee 0 / 60, ankle 0 / 20.
    const PITCH_AXIS: [f32; 3] = [-30.0, 0.0, 30.0];
    const ROLL_AXIS: [f32; 3] = [-10.0, 0.0, 10.0];
    const YAW_AXIS: [f32; 2] = [0.0, 10.0];
    const KNEE_AXIS: [f32; 2] = [0.0, 60.0];
    const ANKLE_AXIS: [f32; 2] = [0.0, 20.0];
    const CELLS: usize = 2 * 3 * 2 * 2;

    fn cell(ankle: usize, pitch: usize, yaw: usize, knee: usize) -> usize {
        ((ankle * 3 + pitch) * 2 + yaw) * 2 + knee
    }

    fn axes() -> HipAxes<'static> {
        HipAxes { pitch: &PITCH_AXIS, roll: &ROLL_AXIS, yaw: &YAW_AXIS, knee: &KNEE_AXIS, ankle: &ANKLE_AXIS }
    }

    fn pose(pitch: f32, roll: f32, yaw: f32, knee: f32, ankle: f32) -> LegPose {
        LegPose { yaw, roll, pitch, knee, ankle }
    }

    #[test]
    fn a_grid_value_is_looked_up_alone_and_between_values_the_stricter_neighbour_decides() {
        let mut left = [0u8; CELLS];
        // Left leg: at pitch 30, yaw 0, knee 0, ankle 0 the roll +10 is not allowed.
        left[cell(0, 2, 0, 0)] = 0b100;
        let right = [0u8; CELLS];
        let map = HipMap::new(axes(), &left, &right).expect("a valid map");

        assert!(!map.allows(Side::Left, &pose(30.0, 10.0, 0.0, 0.0, 0.0)));
        assert!(map.allows(Side::Left, &pose(30.0, 0.0, 0.0, 0.0, 0.0)));
        assert!(map.allows(Side::Left, &pose(0.0, 10.0, 0.0, 0.0, 0.0)));
        // Between pitch 0 (free) and 30 (not), between roll 0 (free) and 10 (not): not allowed.
        assert!(!map.allows(Side::Left, &pose(15.0, 10.0, 0.0, 0.0, 0.0)));
        assert!(!map.allows(Side::Left, &pose(30.0, 5.0, 0.0, 0.0, 0.0)));
        assert!(map.allows(Side::Left, &pose(15.0, 0.0, 0.0, 0.0, 0.0)));
        // The other axes too: a little yaw, knee or ankle away from the cell still brackets it.
        assert!(!map.allows(Side::Left, &pose(30.0, 10.0, 5.0, 0.0, 0.0)));
        assert!(!map.allows(Side::Left, &pose(30.0, 10.0, 0.0, 30.0, 0.0)));
        assert!(!map.allows(Side::Left, &pose(30.0, 10.0, 0.0, 0.0, 10.0)));
        // On the far grid value of another axis the cell is no longer a neighbour.
        assert!(map.allows(Side::Left, &pose(30.0, 10.0, 10.0, 0.0, 0.0)));
        assert!(map.allows(Side::Left, &pose(30.0, 10.0, 0.0, 0.0, 20.0)));
        // Beyond an axis's end the end value is used.
        assert!(!map.allows(Side::Left, &pose(45.0, 25.0, -5.0, -20.0, -30.0)));
        // Each leg has its own map.
        assert!(map.allows(Side::Right, &pose(30.0, 10.0, 0.0, 0.0, 0.0)));
        // A pose that is not a number is never allowed.
        assert!(!map.allows(Side::Left, &pose(f32::NAN, 0.0, 0.0, 0.0, 0.0)));
    }

    #[test]
    fn bad_map_data_is_refused() {
        let cells = [0u8; CELLS];
        let falling = [0.0, -1.0];
        let mut bad = axes();
        bad.yaw = &falling;
        assert_eq!(HipMap::new(bad, &cells, &cells).err(), Some(MapError::Axis("hip yaw")));
        assert_eq!(HipMap::new(axes(), &cells[..CELLS - 1], &cells).err(), Some(MapError::CellCount));
        let nine = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        let mut wide = axes();
        wide.roll = &nine;
        assert_eq!(HipMap::new(wide, &cells, &cells).err(), Some(MapError::TooManyRolls));
        assert_eq!(KneeAnkle::new(30.0, &[(40.0, -60.0), (35.0, -45.0)]).err(), Some(MapError::Table));
        assert_eq!(KneeAnkle::new(30.0, &[]).err(), Some(MapError::Table));
    }

    #[test]
    fn the_knee_ankle_table_uses_the_next_row_up() {
        let rows = [(40.0, -60.0), (50.0, -45.0), (68.0, 0.0)];
        let rule = KneeAnkle::new(30.0, &rows).expect("a valid table");
        assert!(rule.allows(30.0, -85.0));
        assert!(rule.allows(35.0, -60.0) && !rule.allows(35.0, -61.0));
        assert!(rule.allows(41.0, -45.0) && !rule.allows(41.0, -50.0));
        assert!(rule.allows(68.0, 0.0) && !rule.allows(68.0, -1.0));
        assert!(rule.allows(75.0, 0.0) && !rule.allows(75.0, -1.0));
        assert!(!rule.allows(f32::NAN, 0.0));
    }

    #[test]
    fn settle_keeps_as_much_of_a_move_as_the_rules_allow() {
        let free = [0u8; CELLS];
        let mut left = [0u8; CELLS];
        // With the ankle at 20, pitch 30 is not allowed at roll 0 (left leg).
        left[cell(1, 2, 0, 0)] = 0b010;
        let rows = [(60.0, 0.0)];
        let rules = LegRules {
            hip: HipMap::new(axes(), &left, &free).expect("a valid map"),
            knee_ankle: KneeAnkle::new(30.0, &rows).expect("a valid table"),
            both_legs: BothLegs {
                yaw_out_over: 0.0,
                pitch_forward_of: -20.0,
                band_pitch_forward_of: -20.0,
                band_roll_in_past: -999.0,
            },
        };
        let rest: LegJoints = [0.0; LEG_JOINTS];

        // Allowed as wanted.
        let (mut l, mut r) = ([0.0, 0.0, 10.0, 0.0, 0.0], rest);
        let done = rules.settle((&rest, &rest), (&mut l, &mut r));
        assert!(done.unchanged());
        assert_eq!(l, [0.0, 0.0, 10.0, 0.0, 0.0]);

        // Pitch 30 with the ankle going to 20: the ankle is held, the hip moves.
        let (mut l, mut r) = ([0.0, 0.0, 30.0, 0.0, 20.0], rest);
        let done = rules.settle((&rest, &rest), (&mut l, &mut r));
        assert_eq!((done.left, done.right, done.knee_ankle), (LegVerdict::AnkleHeld, LegVerdict::Move, false));
        assert_eq!(l, [0.0, 0.0, 30.0, 0.0, 0.0]);

        // Already at ankle 20 and asked for pitch 30 with the ankle back at 0: only the ankle moves this time.
        let at = [0.0, 0.0, 0.0, 0.0, 20.0];
        let (mut l, mut r) = ([0.0, 0.0, 30.0, 0.0, 12.0], rest);
        let done = rules.settle((&at, &rest), (&mut l, &mut r));
        assert_eq!(done.left, LegVerdict::HipAndKneeHeld);
        assert_eq!(l, [0.0, 0.0, 0.0, 0.0, 12.0]);

        // Knee past 30 with the ankle below its minimum, from a pose that already breaks the rule: held whole.
        let bent = [0.0, 0.0, 0.0, 50.0, -10.0];
        let (mut l, mut r) = ([0.0, 0.0, 0.0, 55.0, -12.0], rest);
        let done = rules.settle((&bent, &rest), (&mut l, &mut r));
        assert_eq!((done.left, done.knee_ankle), (LegVerdict::Held, true));
        assert_eq!(l, bent);

        // The right leg in its own signs: knee -50 is its bend, ankle +10 its "down".
        let (mut l, mut r) = (rest, [0.0, 0.0, 0.0, -50.0, 10.0]);
        let done = rules.settle((&rest, &rest), (&mut l, &mut r));
        assert_eq!((done.right, done.knee_ankle), (LegVerdict::AnkleHeld, true));
        assert_eq!(r, [0.0, 0.0, 0.0, -50.0, 0.0]);

        // Both legs forward and toed out: each leg alone is fine, together they stay.
        let (mut l, mut r) = ([5.0, 0.0, -25.0, 0.0, 0.0], [-5.0, 0.0, 25.0, 0.0, 0.0]);
        let done = rules.settle((&rest, &rest), (&mut l, &mut r));
        assert!(done.both_legs && done.left == LegVerdict::Move && done.right == LegVerdict::Move);
        assert_eq!((l, r), (rest, rest));
        assert_eq!(done.firmware_flags(), 1 << 7);
        // One toed in: allowed.
        let (mut l, mut r) = ([5.0, 0.0, -25.0, 0.0, 0.0], [5.0, 0.0, 25.0, 0.0, 0.0]);
        assert!(rules.settle((&rest, &rest), (&mut l, &mut r)).unchanged());
    }
}
