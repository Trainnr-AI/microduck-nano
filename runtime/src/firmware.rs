// SPDX-License-Identifier: Apache-2.0
//! Typed calls to the Microduck Nano microcontroller firmware (`firmware/app/microduck/sketch`).
//!
//! Every call here is read-only except [`step`], which sends joint targets, and the `bench_*` calls, which move one
//! servo on its own. The firmware passes targets through its own safety layer and ignores them unless it was built
//! with motion enabled (and, for the bench calls, as a bench build) and has been armed.

use std::io::{Read, Write};

use crate::bridge::{Bridge, BridgeError};
use crate::robot::JOINT_NAMES;

/// The firmware's value for "not measured" (a servo that did not answer, a gyro that is not reporting).
const MISSING: i64 = -32768;

/// The firmware protocol this runtime speaks: 3 added the gyro to the state message.
pub const PROTOCOL: i64 = 3;

const MODES: [&str; 3] = ["DISARMED", "ARMED", "FAULT"];
const POSTURES: [&str; 6] =
    ["UNKNOWN", "STANDING", "SITTING", "MOVING_TO_SIT", "MOVING_TO_STAND", "RETURNING_TO_STAND"];
const FAULTS: [&str; 8] = [
    "none", "overload", "over-temperature", "under-voltage", "servo status", "servo not answering", "fall",
    "bus write",
];

fn name(table: &[&'static str], code: i64) -> &'static str {
    usize::try_from(code).ok().and_then(|i| table.get(i).copied()).unwrap_or("unknown code")
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

fn field(v: &[i64], i: usize) -> Result<i64, BridgeError> {
    v.get(i).copied().ok_or(BridgeError::Shape("reply is shorter than the firmware's protocol says"))
}

pub fn info<S: Read + Write>(bridge: &mut Bridge<S>) -> Result<Info, BridgeError> {
    let v = bridge.call_ints("duck_info", vec![])?;
    Ok(Info {
        protocol: field(&v, 0)?,
        motion_build: field(&v, 1)? != 0,
        eeprom_build: field(&v, 2)? != 0,
        joint_table_ready: field(&v, 3)? != 0,
        imu_present: field(&v, 4)? != 0,
        servo_uart_is_serial1: field(&v, 5)? != 0,
        bench_build: v.get(6).is_some_and(|&b| b != 0),
    })
}

pub fn status<S: Read + Write>(bridge: &mut Bridge<S>) -> Result<Status, BridgeError> {
    let v = bridge.call_ints("duck_status", vec![])?;
    Ok(Status {
        mode: name(&MODES, field(&v, 0)?),
        posture: name(&POSTURES, field(&v, 1)?),
        fault: name(&FAULTS, field(&v, 2)?),
        flags: field(&v, 4)?,
        imu_ok: field(&v, 5)? != 0,
        pitch_deg: field(&v, 6)? as f64 / 10.0,
        roll_deg: field(&v, 7)? as f64 / 10.0,
        worst_tick_us: field(&v, 9)?,
    })
}

/// Result codes of a command, as the firmware reports them (`duck::Result`).
const RESULTS: [&str; 14] = [
    "ok", "firmware built without motion", "bad key", "joint table not calibrated", "servo missing", "IMU required",
    "fault latched", "not armed", "already armed", "wrong posture", "not at the STAND keyframe",
    "facing uphill: sit refused", "busy", "bus error",
];

/// Everything a control tick needs, from one reply (firmware protocol 3: `duck_state` and `duck_step`).
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
}

fn parse_state(v: &[i64]) -> Result<State, BridgeError> {
    let joints = JOINT_NAMES.len();
    if v.len() == 10 + joints {
        return Err(BridgeError::Shape("state reply has no gyro: the firmware speaks protocol 2, this runtime needs 3"));
    }
    if v.len() != 10 + joints + 3 {
        return Err(BridgeError::Shape("state reply is not 10 words, one per joint, then 3 gyro words"));
    }
    let status = Status {
        mode: name(&MODES, field(v, 1)?),
        posture: name(&POSTURES, field(v, 2)?),
        fault: name(&FAULTS, field(v, 3)?),
        flags: field(v, 5)?,
        imu_ok: field(v, 6)? != 0,
        pitch_deg: field(v, 7)? as f64 / 10.0,
        roll_deg: field(v, 8)? as f64 / 10.0,
        worst_tick_us: field(v, 9)?,
    };
    // The length was checked above: three gyro words follow the joints.
    let gyro = [field(v, 10 + joints)?, field(v, 11 + joints)?, field(v, 12 + joints)?];
    let angular_velocity = (!gyro.contains(&MISSING)).then(|| gyro.map(|w| (w as f64 / 10.0).to_radians()));
    let joints =
        v.iter().skip(10).take(joints).map(|&t| (t != MISSING).then(|| (t as f64 / 10.0).to_radians())).collect();
    Ok(State { result: name(&RESULTS, field(v, 0)?), status, joints, angular_velocity })
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
    let tenths = targets.iter().map(|t| rmpv::Value::from((t.to_degrees() * 10.0).round() as i64)).collect();
    parse_state(&bridge.call_ints("duck_step", vec![rmpv::Value::Array(tenths)])?)
}

/// Put `text` on the board's LED matrix (firmware protocol 3). Up to 32 characters; letters, digits and a few
/// signs (`+ - / . : ? ! % ( ) = # _ '`), lower case shown as upper case. It stays up for 3 s after the last call
/// (longer if it scrolls), then the firmware's status ticker returns; a fault always shows instead. An empty string
/// clears it. Returns how many characters the firmware kept.
pub fn say<S: Read + Write>(bridge: &mut Bridge<S>, text: &str) -> Result<i64, BridgeError> {
    call_int(bridge, "duck_say", vec![rmpv::Value::from(text)])
}

/// The arm key the firmware expects (`duck::kArmKey`).
pub const ARM_KEY: u32 = 0xD0C5_A4B3;

fn call_int<S: Read + Write>(bridge: &mut Bridge<S>, method: &str, params: Vec<rmpv::Value>) -> Result<i64, BridgeError> {
    match bridge.call(method, params)? {
        rmpv::Value::Integer(n) => n.as_i64().ok_or(BridgeError::Shape("expected a whole number")),
        _ => Err(BridgeError::Shape("expected a whole number")),
    }
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

#[derive(Debug, Clone, PartialEq)]
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

const BENCH_MODES: [&str; 3] = ["OFF", "ARMED", "FAULT"];

/// Arm one joint's servo on its own (bench build). Returns the firmware's result: "ok" or why not.
pub fn bench_arm<S: Read + Write>(bridge: &mut Bridge<S>, joint: usize) -> Result<&'static str, BridgeError> {
    let joint = i64::try_from(joint).map_err(|_| BridgeError::Shape("joint index"))?;
    let result = call_int(bridge, "duck_bench_arm", vec![rmpv::Value::from(ARM_KEY), rmpv::Value::from(joint)])?;
    Ok(name(&RESULTS, result))
}

/// Send the bench servo's target (radians; `None`: no new target, just read) and get its state back.
pub fn bench_step<S: Read + Write>(bridge: &mut Bridge<S>, target: Option<f64>) -> Result<Bench, BridgeError> {
    let tenths = match target {
        Some(t) if t.is_finite() => (t.to_degrees() * 10.0).round() as i64,
        Some(_) => return Err(BridgeError::Shape("the bench target must be a finite number")),
        None => MISSING,
    };
    let v = bridge.call_ints("duck_bench_step", vec![rmpv::Value::from(tenths)])?;
    if v.len() != 12 {
        return Err(BridgeError::Shape("bench reply is not 12 words"));
    }
    let deg10 = |i: usize| field(&v, i).map(|t| (t as f64 / 10.0).to_radians());
    let reading = if field(&v, 4)? != 0 {
        Some(BenchReading {
            position: deg10(5)?,
            velocity: deg10(6)?,
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
        setpoint: deg10(11)?,
    })
}

/// Torque off on the bench servo and forget a latched bench fault.
pub fn bench_disarm<S: Read + Write>(bridge: &mut Bridge<S>) -> Result<(), BridgeError> {
    call_int(bridge, "duck_bench_disarm", vec![]).map(|_| ())
}

/// Measured joint angles in radians, in the firmware's joint order. `None` where the servo did not answer.
pub fn joints<S: Read + Write>(bridge: &mut Bridge<S>) -> Result<Vec<Option<f64>>, BridgeError> {
    let v = bridge.call_ints("duck_joints", vec![])?;
    if v.len() != JOINT_NAMES.len() {
        return Err(BridgeError::Shape("duck_joints did not return one value per joint"));
    }
    Ok(v.iter().map(|&tenths| (tenths != MISSING).then(|| (tenths as f64 / 10.0).to_radians())).collect())
}
