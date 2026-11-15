// SPDX-License-Identifier: Apache-2.0
//! Recording what the loop measured and sent, for system identification in trainnr.
//!
//! The file is an MCAP with ROS 2 messages (profile `ros2`, CDR encoding), which trainnr's `mcap` adapter reads
//! without ROS (`ingest_recording`):
//!
//! | Topic | Message | What |
//! |---|---|---|
//! | `/joint_states` | `sensor_msgs/msg/JointState` | measured joint angles, radians, all fifteen joints; NaN where the servo did not answer |
//! | `/joint_commands` | `sensor_msgs/msg/JointState` | the targets sent to the firmware that tick (only with `--exchange step`); from `duckd bench`, the goal the firmware wrote after its clamp and speed limit |
//! | `/joint_targets` | `sensor_msgs/msg/JointState` | `duckd bench` only: the target asked for, before the firmware's clamp and speed limit |
//! | `/imu` | `sensor_msgs/msg/Imu` | trunk orientation from the firmware's pitch and roll (yaw unknown), body angular velocity |
//!
//! Velocity and effort are left empty: the firmware reports neither (the servo's "load" is a duty cycle, not a
//! torque). Linear acceleration is NaN (no accelerometer report yet), as is any IMU part not measured that tick.
//! A loop tick is stamped with the time `duckd` received the state; a bench read with the time the servo was read.
//! The IMU sign conventions are unchecked against a real sensor; see `docs/robot/bench-checklist.md`.

use std::collections::BTreeMap;
use std::fmt;
use std::fs::File;
use std::io::BufWriter;
use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};

use crate::bench::BenchSample;
use crate::firmware::Sensed;
use crate::robot::JOINT_NAMES;

const HEADER_DEFS: &str = "\
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
";

const JOINT_STATE_SCHEMA: &str = "\
std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
";

const IMU_SCHEMA: &str = "\
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
";

const IMU_EXTRA_DEFS: &str = "\
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
";

/// ROS's "this field is not provided": -1 as the first element of its covariance.
const UNKNOWN: [f64; 9] = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
/// Pitch and roll known, yaw not: a large variance on the rotation about z.
const YAW_UNKNOWN: [f64; 9] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e9];

#[derive(Debug)]
pub struct RecordError(String);

impl fmt::Display for RecordError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "recording failed: {}", self.0)
    }
}

impl std::error::Error for RecordError {}

impl From<mcap::McapError> for RecordError {
    fn from(e: mcap::McapError) -> Self {
        RecordError(e.to_string())
    }
}

/// Little-endian CDR, as ROS 2 puts it on the wire: a 4-byte encapsulation header, then every primitive aligned to
/// its own size counted from the end of that header.
struct Cdr(Vec<u8>);

impl Cdr {
    fn new() -> Self {
        Cdr(vec![0x00, 0x01, 0x00, 0x00])
    }
    fn align(&mut self, n: usize) {
        while !(self.0.len() - 4).is_multiple_of(n) {
            self.0.push(0);
        }
    }
    /// A primitive's little-endian bytes, aligned to its size.
    fn put<const N: usize>(&mut self, bytes: [u8; N]) {
        self.align(N);
        self.0.extend_from_slice(&bytes);
    }
    fn f64s(&mut self, values: &[f64]) {
        for v in values {
            self.put(v.to_le_bytes());
        }
    }
    fn len(&mut self, n: usize) {
        self.put(u32::try_from(n).unwrap_or(u32::MAX).to_le_bytes());
    }
    fn string(&mut self, s: &str) {
        self.len(s.len() + 1);
        self.0.extend_from_slice(s.as_bytes());
        self.0.push(0);
    }
    fn header(&mut self, t_ns: u64, frame: &str) {
        self.put(i32::try_from(t_ns / 1_000_000_000).unwrap_or(i32::MAX).to_le_bytes());
        self.put(u32::try_from(t_ns % 1_000_000_000).unwrap_or(0).to_le_bytes());
        self.string(frame);
    }
    fn f64_seq(&mut self, values: &[f64]) {
        self.len(values.len());
        self.f64s(values);
    }
}

fn joint_state(t_ns: u64, names: &[&str], positions: &[f64], velocities: &[f64]) -> Vec<u8> {
    let mut c = Cdr::new();
    c.header(t_ns, "");
    c.len(names.len());
    for name in names {
        c.string(name);
    }
    c.f64_seq(positions);
    c.f64_seq(velocities);
    c.f64_seq(&[]);
    c.0
}

/// The trunk's orientation for a pitch (nose up positive) and roll (left side up positive), yaw 0, as a ROS
/// quaternion (x, y, z, w) in REP-103's body frame (x forward, y left, z up). Nose up is a negative rotation about y.
pub fn orientation(pitch: f64, roll: f64) -> [f64; 4] {
    let (sr, cr) = (roll / 2.0).sin_cos();
    let (sp, cp) = (-pitch / 2.0).sin_cos();
    [sr * cp, cr * sp, -sr * sp, cr * cp]
}

fn imu(t_ns: u64, pitch_roll: Option<(f64, f64)>, angular_velocity: Option<[f64; 3]>) -> Vec<u8> {
    let mut c = Cdr::new();
    c.header(t_ns, "imu");
    // What is not measured is NaN as well as flagged: trainnr's adapter reads the values and not the -1 flag, and a
    // zero would pass for a measurement.
    c.f64s(&pitch_roll.map_or([f64::NAN; 4], |(p, r)| orientation(p, r)));
    c.f64s(if pitch_roll.is_some() { &YAW_UNKNOWN } else { &UNKNOWN });
    c.f64s(&angular_velocity.unwrap_or([f64::NAN; 3]));
    c.f64s(if angular_velocity.is_some() { &[0.0; 9] } else { &UNKNOWN });
    c.f64s(&[f64::NAN; 3]); // no accelerometer report yet
    c.f64s(&UNKNOWN);
    c.0
}

/// Nanoseconds after the Unix epoch, now.
pub fn now_ns() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| u64::try_from(d.as_nanos()).unwrap_or(u64::MAX))
}

/// An open recording.
pub struct Recorder {
    writer: mcap::Writer<BufWriter<File>>,
    joint_states: u16,
    joint_commands: u16,
    joint_targets: u16,
    imu: u16,
    sequence: u32,
    /// Ticks recorded so far.
    pub ticks: u32,
}

impl Recorder {
    /// Start a recording at `path` (overwrites it).
    pub fn create(path: impl AsRef<Path>) -> Result<Self, RecordError> {
        let file = File::create(path.as_ref()).map_err(|e| RecordError(format!("{}: {e}", path.as_ref().display())))?;
        let options = mcap::WriteOptions::new().profile("ros2").library("duckd").compression(None);
        let mut writer = mcap::Writer::with_options(BufWriter::new(file), options)?;
        let joint_schema = format!("{JOINT_STATE_SCHEMA}{HEADER_DEFS}");
        let imu_schema = format!("{IMU_SCHEMA}{HEADER_DEFS}{IMU_EXTRA_DEFS}");
        let joint = writer.add_schema("sensor_msgs/msg/JointState", "ros2msg", joint_schema.as_bytes())?;
        let imu_id = writer.add_schema("sensor_msgs/msg/Imu", "ros2msg", imu_schema.as_bytes())?;
        let none = BTreeMap::new();
        let joint_states = writer.add_channel(joint, "/joint_states", "cdr", &none)?;
        let joint_commands = writer.add_channel(joint, "/joint_commands", "cdr", &none)?;
        let joint_targets = writer.add_channel(joint, "/joint_targets", "cdr", &none)?;
        let imu = writer.add_channel(imu_id, "/imu", "cdr", &none)?;
        Ok(Recorder { writer, joint_states, joint_commands, joint_targets, imu, sequence: 0, ticks: 0 })
    }

    fn write(&mut self, channel_id: u16, t_ns: u64, data: &[u8]) -> Result<(), RecordError> {
        let header = mcap::records::MessageHeader { channel_id, sequence: self.sequence, log_time: t_ns, publish_time: t_ns };
        self.sequence = self.sequence.wrapping_add(1);
        Ok(self.writer.write_to_known_channel(&header, data)?)
    }

    /// Record one tick: what was measured and, if targets went out with it, what was sent (radians, firmware order).
    pub fn tick(&mut self, sensed: &Sensed, sent: Option<&[f64]>) -> Result<(), RecordError> {
        let t_ns = now_ns();
        let positions: Vec<f64> = sensed.joints.iter().map(|j| j.unwrap_or(f64::NAN)).collect();
        self.write(self.joint_states, t_ns, &joint_state(t_ns, &JOINT_NAMES, &positions, &[]))?;
        if let Some(sent) = sent {
            self.write(self.joint_commands, t_ns, &joint_state(t_ns, &JOINT_NAMES, sent, &[]))?;
        }
        if sensed.pitch_roll.is_some() || sensed.angular_velocity.is_some() {
            self.write(self.imu, t_ns, &imu(t_ns, sensed.pitch_roll, sensed.angular_velocity))?;
        }
        self.ticks += 1;
        Ok(())
    }

    /// Record a bench run of one joint (`duckd bench`): each target asked for as `/joint_targets` at the time it was
    /// sent; for each distinct read of the servo, `/joint_states` (position and the servo's own speed) and
    /// `/joint_commands` (the goal the firmware wrote in that same tick, after its clamp and speed limit: what the
    /// servo was actually told) at the time the microcontroller made it. Its clock is mapped onto this machine's by
    /// the reply that came back fastest, so the link's delay and jitter stay out of the measured timing. Returns the
    /// number of distinct reads.
    pub fn bench(&mut self, joint: &str, samples: &[BenchSample]) -> Result<usize, RecordError> {
        // Every read was made before its reply arrived: the smallest gap is the best estimate of the clock offset.
        let offset = samples
            .iter()
            .filter_map(|s| s.read.map(|r| i128::from(s.received_ns) - i128::from(r.read_ms) * 1_000_000))
            .min()
            .unwrap_or(0);
        let mut last_read: Option<u32> = None;
        let mut reads = 0;
        for s in samples {
            self.write(self.joint_targets, s.sent_ns, &joint_state(s.sent_ns, &[joint], &[s.target], &[]))?;
            if let Some(r) = s.read.filter(|r| Some(r.read_ms) != last_read) {
                let t = u64::try_from(i128::from(r.read_ms) * 1_000_000 + offset).unwrap_or(0);
                self.write(self.joint_states, t, &joint_state(t, &[joint], &[r.position], &[r.velocity]))?;
                self.write(self.joint_commands, t, &joint_state(t, &[joint], &[s.setpoint], &[]))?;
                last_read = Some(r.read_ms);
                reads += 1;
            }
        }
        self.ticks += u32::try_from(samples.len()).unwrap_or(u32::MAX);
        Ok(reads)
    }

    /// Write the summary and close the file. A recording that is not finished cannot be read.
    pub fn finish(mut self) -> Result<u32, RecordError> {
        self.writer.finish()?;
        Ok(self.ticks)
    }
}
