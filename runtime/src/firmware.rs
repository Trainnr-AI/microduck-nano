// SPDX-License-Identifier: Apache-2.0
//! Typed calls to the Microduck Nano microcontroller firmware (`firmware/app/microduck/sketch`).
//!
//! Every call here is read-only except [`step`], which sends joint targets, and the `bench_*` calls, which move one
//! servo on its own. The firmware passes targets through its own safety layer and ignores them unless it was built
//! with motion enabled (and, for the bench calls, as a bench build) and has been armed.

use std::io::{Read, Write};

use rmpv::Value;

use crate::bridge::{Bridge, BridgeError};
use crate::robot::JOINT_NAMES;

/// The firmware protocol this runtime speaks: 3 added the gyro to the state message, 4 each joint's speed.
pub const PROTOCOL: i64 = 4;
/// The arm key the firmware expects (`duck::kArmKey`).
pub const ARM_KEY: u32 = 0xD0C5_A4B3;

/// The firmware's value for "not measured" (a servo that did not answer, a gyro that is not reporting).
const MISSING: i64 = -32768;

const MODES: [&str; 3] = ["DISARMED", "ARMED", "FAULT"];
const BENCH_MODES: [&str; 3] = ["OFF", "ARMED", "FAULT"];
const POSTURES: [&str; 6] =
    ["UNKNOWN", "STANDING", "SITTING", "MOVING_TO_SIT", "MOVING_TO_STAND", "RETURNING_TO_STAND"];
const FAULTS: [&str; 8] = [
    "none", "overload", "over-temperature", "under-voltage", "servo status", "servo not answering", "fall",
    "bus write",
];
/// Result codes of a command, as the firmware reports them (`duck::Result`).
const RESULTS: [&str; 14] = [
    "ok", "firmware built without motion", "bad key", "joint table not calibrated", "servo missing", "IMU required",
    "fault latched", "not armed", "already armed", "wrong posture", "not at the STAND keyframe",
    "facing uphill: sit refused", "busy", "bus error",
];

fn name(table: &[&'static str], code: i64) -> &'static str {
    usize::try_from(code).ok().and_then(|i| table.get(i).copied()).unwrap_or("unknown code")
}

fn field(v: &[i64], i: usize) -> Result<i64, BridgeError> {
    v.get(i).copied().ok_or(BridgeError::Shape("reply is shorter than the firmware's protocol says"))
}

/// An angle or a speed on the wire (tenths of a degree, or of a degree per second; [`MISSING`] for none) in radians
/// (per second).
fn radians(tenths: i64) -> Option<f64> {
    (tenths != MISSING).then(|| (tenths as f64 / 10.0).to_radians())
}

/// An angle in radians as the wire carries it: tenths of a degree.
fn tenths(radians: f64) -> i64 {
    (radians.to_degrees() * 10.0).round() as i64
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Info {
    pub protocol: i64,
    pub motion_build: bool,
    pub eeprom_build: bool,
    pub joint_table_ready: bool,
    pub imu_present: bool,
    pub servo_uart_is_serial1: bool,
    /// Built with DUCK_BENCH_MOTION: one servo can be armed on its own (absent from older firmware: false).
    pub bench_build: bool,
}

pub fn info<S: Read + Write>(bridge: &mut Bridge<S>) -> Result<Info, BridgeError> {
    let v = bridge.call_ints("duck_info", vec![])?;
    let flag = |i| field(&v, i).map(|b| b != 0);
    Ok(Info {
        protocol: field(&v, 0)?,
        motion_build: flag(1)?,
        eeprom_build: flag(2)?,
        joint_table_ready: flag(3)?,
        imu_present: flag(4)?,
        servo_uart_is_serial1: flag(5)?,
        bench_build: flag(6).unwrap_or(false),
    })
}

#[derive(Debug, Clone, PartialEq)]
pub struct Status {
    pub mode: &'static str,
    pub posture: &'static str,
    pub fault: &'static str,
    pub flags: i64,
    pub imu_ok: bool,
    pub pitch_deg: f64,
    pub roll_deg: f64,
    pub worst_tick_us: i64,
}

impl Status {
    /// Read from the words at `at`: mode, posture, fault, flags, IMU ok, pitch x10, roll x10, worst tick.
    /// (`duck_status` and the state message lay them out differently.)
    fn from_words(v: &[i64], at: [usize; 8]) -> Result<Self, BridgeError> {
        let [mode, posture, fault, flags, imu_ok, pitch, roll, worst] = at.map(|i| field(v, i));
        Ok(Status {
            mode: name(&MODES, mode?),
            posture: name(&POSTURES, posture?),
            fault: name(&FAULTS, fault?),
            flags: flags?,
            imu_ok: imu_ok? != 0,
            pitch_deg: pitch? as f64 / 10.0,
            roll_deg: roll? as f64 / 10.0,
            worst_tick_us: worst?,
        })
    }

    /// Trunk pitch and roll in radians, if the IMU is usable.
    pub fn pitch_roll(&self) -> Option<(f64, f64)> {
        self.imu_ok.then(|| (self.pitch_deg.to_radians(), self.roll_deg.to_radians()))
    }
}

pub fn status<S: Read + Write>(bridge: &mut Bridge<S>) -> Result<Status, BridgeError> {
    Status::from_words(&bridge.call_ints("duck_status", vec![])?, [0, 1, 2, 4, 5, 6, 7, 9])
}

/// What the robot measured on one tick, in the firmware's joint order. `None` means "not measured", never zero.
#[derive(Debug, Clone, Default)]
pub struct Sensed {
    /// Joint angles in radians, all fifteen.
    pub joints: Vec<Option<f64>>,
    /// Trunk pitch (nose up positive) and roll (left side up positive) in radians, from the IMU.
    pub pitch_roll: Option<(f64, f64)>,
    /// Body angular velocity in rad/s.
    pub angular_velocity: Option<[f64; 3]>,
    /// Joint speeds in rad/s as each servo reported them in the same read as the angle; empty when the exchange
    /// carries none (firmware protocol 1 reads).
    pub joint_speeds: Vec<Option<f64>>,
}

/// Everything a control tick needs, from one reply (firmware protocol 4: `duck_state` and `duck_step`).
#[derive(Debug, Clone, PartialEq)]
pub struct State {
    /// What the firmware did with the command this reply answers; "ok" for a plain read.
    pub result: &'static str,
    pub status: Status,
    /// Measured joint angles in radians, firmware joint order; `None` where the servo did not answer.
    pub joints: Vec<Option<f64>>,
    /// Body angular velocity in rad/s about x (forward), y (left), z (up); `None` when the gyro is not measured
    /// (no IMU, a stale report, or the IMU's mounting not calibrated in `robot.json`).
    pub angular_velocity: Option<[f64; 3]>,
    /// Joint speeds in rad/s as each servo reported them in the same read as the angle (the register's unit is
    /// unconfirmed on a real SCS0009: bench checklist 8b); `None` where the servo did not answer.
    pub joint_speeds: Vec<Option<f64>>,
}

impl State {
    pub fn sensed(self) -> Sensed {
        let State { status, joints, angular_velocity, joint_speeds, .. } = self;
        Sensed { pitch_roll: status.pitch_roll(), joints, angular_velocity, joint_speeds }
    }
}

fn parse_state(v: &[i64]) -> Result<State, BridgeError> {
    let joints = JOINT_NAMES.len();
    if v.len() != 10 + joints + 3 + joints {
        return Err(BridgeError::Shape(
            "state reply is not 10 words, one angle per joint, 3 gyro words and one speed per joint: the firmware \
             speaks an older protocol (see duckd status); this runtime needs 4",
        ));
    }
    let gyro = [field(v, 10 + joints)?, field(v, 11 + joints)?, field(v, 12 + joints)?];
    Ok(State {
        result: name(&RESULTS, field(v, 0)?),
        status: Status::from_words(v, [1, 2, 3, 5, 6, 7, 8, 9])?,
        joints: v.iter().skip(10).take(joints).map(|&w| radians(w)).collect(),
        angular_velocity: (!gyro.contains(&MISSING)).then(|| gyro.map(|w| (w as f64 / 10.0).to_radians())),
        joint_speeds: v.iter().skip(13 + joints).map(|&w| radians(w)).collect(),
    })
}

/// Read the whole state in one round trip.
pub fn state<S: Read + Write>(bridge: &mut Bridge<S>) -> Result<State, BridgeError> {
    parse_state(&bridge.call_ints("duck_state", vec![])?)
}

/// Send one target per joint (radians, firmware joint order) and get the state back, in one round trip.
///
/// The firmware clamps, rate-limits and interlocks the targets itself, and ignores them unless armed; the reply's
/// `result` says which. Targets travel as tenths of a degree.
pub fn step<S: Read + Write>(bridge: &mut Bridge<S>, targets: &[f64]) -> Result<State, BridgeError> {
    if targets.len() != JOINT_NAMES.len() || targets.iter().any(|t| !t.is_finite()) {
        return Err(BridgeError::Shape("step needs one finite target per joint"));
    }
    let words = targets.iter().map(|&t| Value::from(tenths(t))).collect();
    parse_state(&bridge.call_ints("duck_step", vec![Value::Array(words)])?)
}

/// Measured joint angles in radians, in the firmware's joint order. `None` where the servo did not answer.
pub fn joints<S: Read + Write>(bridge: &mut Bridge<S>) -> Result<Vec<Option<f64>>, BridgeError> {
    let v = bridge.call_ints("duck_joints", vec![])?;
    if v.len() != JOINT_NAMES.len() {
        return Err(BridgeError::Shape("duck_joints did not return one value per joint"));
    }
    Ok(v.into_iter().map(radians).collect())
}

/// Put `text` on the board's LED matrix (firmware protocol 3). Up to 32 characters; letters, digits and a few
/// signs (`+ - / . : ? ! % ( ) = # _ '`), lower case shown as upper case. It stays up for 3 s after the last call
/// (longer if it scrolls), then the firmware's status ticker returns; a fault always shows instead. An empty string
/// clears it. Returns how many characters the firmware kept.
pub fn say<S: Read + Write>(bridge: &mut Bridge<S>, text: &str) -> Result<i64, BridgeError> {
    bridge.call_int("duck_say", vec![Value::from(text)])
}

/// What the firmware's bench servo did and read (bench build only; `duck_bench_step`).
#[derive(Debug, Clone, PartialEq)]
pub struct Bench {
    /// What the firmware did with the target this reply answers.
    pub result: &'static str,
    /// "OFF", "ARMED" or "FAULT".
    pub mode: &'static str,
    pub fault: &'static str,
    /// The joint armed (or last armed), firmware order.
    pub joint: Option<usize>,
    /// The servo's last read, if it answered.
    pub reading: Option<BenchReading>,
    /// The goal written last, after the firmware's clamp and rate limit, radians.
    pub setpoint: f64,
}

impl Bench {
    pub fn armed(&self) -> bool {
        self.mode == BENCH_MODES[1]
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BenchReading {
    pub position: f64,
    /// The servo's own speed report, rad/s.
    pub velocity: f64,
    /// Load as the servo reports it: a duty cycle in percent, signed in the joint's direction. Not a torque.
    pub load_percent: f64,
    pub voltage: f64,
    pub temperature_c: i64,
    /// When the microcontroller read it, on its own millisecond clock (wraps after 49 days).
    pub read_ms: u32,
}

/// Arm one joint's servo on its own (bench build). `Ok(())`, or the firmware's reason for refusing.
pub fn bench_arm<S: Read + Write>(bridge: &mut Bridge<S>, joint: usize) -> Result<Result<(), &'static str>, BridgeError> {
    let joint = i64::try_from(joint).map_err(|_| BridgeError::Shape("joint index"))?;
    let result = bridge.call_int("duck_bench_arm", vec![Value::from(ARM_KEY), Value::from(joint)])?;
    Ok(if result == 0 { Ok(()) } else { Err(name(&RESULTS, result)) })
}

/// Send the bench servo's target (radians; `None`: no new target, just read) and get its state back.
pub fn bench_step<S: Read + Write>(bridge: &mut Bridge<S>, target: Option<f64>) -> Result<Bench, BridgeError> {
    let word = match target {
        Some(t) if t.is_finite() => tenths(t),
        Some(_) => return Err(BridgeError::Shape("the bench target must be a finite number")),
        None => MISSING,
    };
    let v = bridge.call_ints("duck_bench_step", vec![Value::from(word)])?;
    if v.len() != 12 {
        return Err(BridgeError::Shape("bench reply is not 12 words"));
    }
    let radians = |i| field(&v, i).map(|w| (w as f64 / 10.0).to_radians());
    let reading = if field(&v, 4)? != 0 {
        Some(BenchReading {
            position: radians(5)?,
            velocity: radians(6)?,
            load_percent: field(&v, 7)? as f64 / 10.0,
            voltage: field(&v, 8)? as f64 / 10.0,
            temperature_c: field(&v, 9)?,
            read_ms: field(&v, 10)? as u32,
        })
    } else {
        None
    };
    Ok(Bench {
        result: name(&RESULTS, field(&v, 0)?),
        mode: name(&BENCH_MODES, field(&v, 1)?),
        fault: name(&FAULTS, field(&v, 2)?),
        joint: usize::try_from(field(&v, 3)?).ok(),
        reading,
        setpoint: radians(11)?,
    })
}

/// Torque off on the bench servo and forget a latched bench fault.
pub fn bench_disarm<S: Read + Write>(bridge: &mut Bridge<S>) -> Result<(), BridgeError> {
    bridge.call_int("duck_bench_disarm", vec![]).map(drop)
}
