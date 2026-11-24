// SPDX-License-Identifier: Apache-2.0
//! Host tests: no board needed. The router is a thread on the other end of a socket pair.
#![allow(clippy::expect_used, clippy::indexing_slicing)] // tests may panic: that is how they fail

use std::os::unix::net::UnixStream;
use std::sync::atomic::{AtomicU32, Ordering};
use std::thread;
use std::time::Duration;

use arm::Verdict;
use duck_runtime::bench::{self, Excitation};
use duck_runtime::bridge::{Bridge, BridgeError};
use duck_runtime::chase::Chase;
use duck_runtime::control::{self, ChainOutcome, Direction, Exchange, Guards, Sinks};
use duck_runtime::firmware;
use duck_runtime::firmware::Sensed;
use duck_runtime::hear::{self, Heard};
use duck_runtime::pet::{self, Mood, Pet};
use duck_runtime::policy::{projected_gravity, Manifest, Policy, PolicyError, Term};
use duck_runtime::record::{orientation, Recorder};
use duck_runtime::robot::{Robot, JOINT_NAMES};
use duck_runtime::see::{self, Chroma, Lens};
use duck_runtime::status_led::{Colour, StatusLed};
use rmpv::Value;

const ROBOT_JSON: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../firmware/config/robot.json");

fn robot() -> Robot {
    Robot::load(ROBOT_JSON).expect("robot.json")
}

/// A fake router: answers each request with `answer(method)`; `None` means "method not available".
fn fake_router(answer: fn(&str) -> Option<Value>) -> Bridge<UnixStream> {
    let (client, mut server) = UnixStream::pair().expect("socket pair");
    thread::spawn(move || {
        while let Ok(Value::Array(req)) = rmpv::decode::read_value(&mut server) {
            let id = req.get(1).cloned().unwrap_or(Value::Nil);
            let method = req.get(2).and_then(Value::as_str).unwrap_or("").to_string();
            let reply = match answer(&method) {
                Some(result) => Value::Array(vec![Value::from(1), id, Value::Nil, result]),
                None => Value::Array(vec![Value::from(1), id, Value::from("method not available"), Value::Nil]),
            };
            if rmpv::encode::write_value(&mut server, &reply).is_err() {
                break;
            }
        }
    });
    client.set_read_timeout(Some(Duration::from_secs(2))).expect("timeout");
    Bridge::new(client)
}

#[test]
fn a_late_reply_to_an_earlier_call_is_dropped_not_taken_as_this_ones() {
    // Before each answer the router sends the answer to the previous request, as a reply that missed its timeout does.
    let (client, mut server) = UnixStream::pair().expect("socket pair");
    thread::spawn(move || {
        while let Ok(Value::Array(req)) = rmpv::decode::read_value(&mut server) {
            let id = req.get(1).and_then(Value::as_u64).unwrap_or(0);
            for (reply_to, answer) in [(id - 1, "stale"), (id, "fresh")] {
                let reply = Value::Array(vec![Value::from(1), Value::from(reply_to), Value::Nil, Value::from(answer)]);
                rmpv::encode::write_value(&mut server, &reply).expect("write");
            }
        }
    });
    client.set_read_timeout(Some(Duration::from_secs(2))).expect("timeout");
    let mut bridge = Bridge::new(client);
    for _ in 0..3 {
        assert_eq!(bridge.call("duck_info", vec![]).expect("a reply").as_str(), Some("fresh"));
    }

    // A reply to a request not yet sent is still an error.
    let (client, mut server) = UnixStream::pair().expect("socket pair");
    thread::spawn(move || {
        let _ = rmpv::decode::read_value(&mut server);
        rmpv::encode::write_value(&mut server, &Value::Array(vec![Value::from(1), Value::from(9), Value::Nil, Value::Nil])).expect("write");
    });
    client.set_read_timeout(Some(Duration::from_secs(2))).expect("timeout");
    assert!(matches!(Bridge::new(client).call("duck_info", vec![]), Err(BridgeError::WrongId { sent: 1, got: 9 })));
}

#[test]
fn a_lost_reply_costs_the_loop_one_tick() {
    // The router never answers the third request, as when the link drops a reply's bytes.
    let (client, mut server) = UnixStream::pair().expect("socket pair");
    thread::spawn(move || {
        let mut n = 0;
        while let Ok(Value::Array(req)) = rmpv::decode::read_value(&mut server) {
            n += 1;
            if n == 3 {
                continue;
            }
            let method = req.get(2).and_then(Value::as_str).unwrap_or("").to_string();
            let reply = Value::Array(vec![Value::from(1), req[1].clone(), Value::Nil, bare_board(&method).unwrap_or(Value::Nil)]);
            rmpv::encode::write_value(&mut server, &reply).expect("write");
        }
    });
    let mut bridge = Bridge::new(client);
    let period = Duration::from_millis(10);
    bridge.set_timeout(period).expect("timeout");
    let stats = control::run(&mut bridge, &robot(), period, Duration::from_millis(300), Exchange::Step, None, &mut Sinks::none());
    assert_eq!(stats.errors, 1, "{:?}", stats.last_error);
    assert!(stats.ticks >= 25, "the loop kept its rate: {} ticks in 300 ms", stats.ticks);
    assert!(stats.missed_deadlines <= 1);
}

fn ints(v: &[i64]) -> Value {
    Value::Array(v.iter().map(|&x| Value::from(x)).collect())
}

/// A protocol 4 state reply: result, mode 0, IMU ok or not, level, every joint at `joint` tenths of a degree, the gyro,
/// then every joint's speed: 2.0 deg/s where the joint answered.
fn state(result: i64, imu_ok: i64, joint: i64, gyro: [i64; 3]) -> Option<Value> {
    let speed = if joint == -32768 { -32768 } else { 20 };
    let head = [result, 0, 0, 0, -1, if imu_ok == 1 { 0 } else { 1024 }, imu_ok, 0, 0, 2];
    Some(ints(&[&head[..], &[joint; 15][..], &gyro[..], &[speed; 15][..]].concat()))
}

/// The firmware as it answers on a bare board: bench-safe build, no servos, no IMU.
fn bare_board(method: &str) -> Option<Value> {
    match method {
        "duck_info" => Some(ints(&[4, 0, 0, 0, 0, 1])),
        "duck_status" => Some(ints(&[0, 0, 0, -1, 1024, 0, 0, 0, 0, 2])),
        "duck_joints" => Some(ints(&[-32768; 15])),
        "duck_say" => Some(Value::from(5)),
        "duck_state" => state(0, 0, -32768, [-32768; 3]),
        // not armed (result 7): the targets are ignored, the state still comes back
        "duck_step" => state(7, 0, -32768, [-32768; 3]),
        _ => None,
    }
}

/// Every servo answering, each joint at 0.5 degrees; IMU ok, level, turning left at 12.5 deg/s and rolling at -3.
fn standing_robot(method: &str) -> Option<Value> {
    match method {
        "duck_joints" => Some(ints(&[5; 15])),
        "duck_state" => state(0, 1, 5, [-30, 0, 125]),
        "duck_step" => state(7, 1, 5, [-30, 0, 125]),
        other => bare_board(other),
    }
}

/// As `standing_robot`, but the gyro is not reporting (stale, or its mounting is not calibrated).
fn standing_robot_without_gyro(method: &str) -> Option<Value> {
    match method {
        "duck_state" => state(0, 1, 5, [-32768; 3]),
        "duck_step" => state(7, 1, 5, [-32768; 3]),
        other => standing_robot(other),
    }
}

/// The firmware before joint speeds: a 28-word state message (protocol 3).
fn protocol_3_firmware(method: &str) -> Option<Value> {
    match method {
        "duck_state" => Some(ints(&[&[0, 0, 0, 0, -1, 0, 1, 0, 0, 2][..], &[5; 15][..], &[0; 3][..]].concat())),
        other => standing_robot(other),
    }
}

#[test]
fn the_gyro_and_joint_speeds_arrive_in_radians_per_second_or_not_at_all() {
    let state = firmware::state(&mut fake_router(standing_robot)).expect("state");
    let w = state.angular_velocity.expect("gyro");
    let expect = [-3.0f64.to_radians(), 0.0, 12.5f64.to_radians()];
    assert!(w.iter().zip(expect).all(|(a, b)| (a - b).abs() < 1e-12), "{w:?}");
    assert_eq!(state.joints.len(), 15);
    assert!(state.joints.iter().all(|j| j.is_some_and(|r| (r - 0.5f64.to_radians()).abs() < 1e-12)));
    assert_eq!(state.joint_speeds.len(), 15);
    assert!(state.joint_speeds.iter().all(|s| s.is_some_and(|w| (w - 2f64.to_radians()).abs() < 1e-12)));

    let state = firmware::state(&mut fake_router(standing_robot_without_gyro)).expect("state");
    assert_eq!(state.angular_velocity, None, "a missing gyro is not a zero");
    let state = firmware::state(&mut fake_router(bare_board)).expect("state");
    assert!(state.joint_speeds.iter().all(Option::is_none), "a servo that did not answer has no speed, not zero");

    let old = firmware::state(&mut fake_router(protocol_3_firmware));
    assert!(matches!(old, Err(BridgeError::Shape(why)) if why.contains("needs 4")));
}

#[test]
fn reads_info_and_status_from_a_bare_board() {
    let mut bridge = fake_router(bare_board);
    let info = firmware::info(&mut bridge).expect("info");
    assert!(!info.motion_build && !info.joint_table_ready && !info.imu_present && info.servo_uart_is_serial1);
    let status = firmware::status(&mut bridge).expect("status");
    assert_eq!((status.mode, status.posture, status.fault), ("DISARMED", "UNKNOWN", "none"));
    let joints = firmware::joints(&mut bridge).expect("joints");
    assert_eq!(joints.len(), 15);
    assert!(joints.iter().all(Option::is_none));
}

#[test]
fn a_refused_call_is_an_error_not_a_value() {
    let mut bridge = fake_router(bare_board);
    match bridge.call("duck_does_not_exist", vec![]) {
        Err(BridgeError::Remote(why)) => assert!(why.contains("not available")),
        other => panic!("expected a refusal, got {other:?}"),
    }
    // the connection is still usable afterwards
    assert!(firmware::info(&mut bridge).is_ok());
}

#[test]
fn the_robot_file_gives_four_chains_within_rigrs_limits() {
    let robot = robot();
    assert_eq!(robot.joints(), JOINT_NAMES.len());
    let sizes: Vec<usize> = robot.chains.iter().map(|c| c.spec.joints()).collect();
    assert_eq!(sizes, [5, 4, 5, 1]);
    assert_eq!(robot.ids_assigned, 0, "servo IDs are placeholders until a real robot is provisioned");
    // left hip yaw: -6.3 .. +19 degrees in the file
    let hip_yaw = &robot.chains[0].spec.joints[0];
    assert_eq!(hip_yaw.name, "left_hip_yaw");
    assert!((hip_yaw.travel_radians.0.to_degrees() + 6.3).abs() < 1e-9);
    assert!((hip_yaw.travel_radians.1.to_degrees() - 19.0).abs() < 1e-9);
    // beak: 0 .. +35
    let beak = &robot.chains[3].spec.joints[0];
    assert!((beak.travel_radians.1.to_degrees() - 35.0).abs() < 1e-9 && beak.travel_radians.0 == 0.0);
}

#[test]
fn a_wrong_joint_order_is_refused() {
    let text = std::fs::read_to_string(ROBOT_JSON).expect("robot.json");
    let swapped = text.replacen("left_hip_yaw", "left_hip_roll", 1);
    assert!(Robot::parse(&swapped).is_err());
}

#[test]
fn the_guard_clamps_to_the_joint_limit_and_limits_the_step() {
    let robot = robot();
    let period = Duration::from_millis(20);
    let mut guards = Guards::new(&robot, period);
    let measured = vec![Some(0.0); 15];
    let mut wanted = vec![0.0; 15];
    wanted[0] = 90f64.to_radians(); // left hip yaw far beyond its +19 degree limit
    guards.fed(0);
    let verdicts = guards.authorise(0, &measured, &wanted);
    let (name, outcome) = &verdicts[0];
    assert_eq!(*name, "left leg");
    let ChainOutcome::Guarded(Verdict::Move { radians }) = outcome else {
        panic!("expected a move, got {outcome:?}");
    };
    let limit = robot.chains[0].spec.joints[0].travel_radians.1;
    let max_step = robot.chains[0].spec.joints[0].max_speed_radians_per_second * period.as_secs_f64();
    assert!(radians[0] <= limit + 1e-12, "never beyond the joint limit");
    assert!(radians[0] <= max_step + 1e-12, "and no further than one tick at the joint's speed limit");
    assert!(radians[0] > 0.0);
}

#[test]
fn the_guard_holds_when_commands_go_stale_and_when_a_servo_is_silent() {
    let robot = robot();
    let mut guards = Guards::new(&robot, Duration::from_millis(20));
    let measured = vec![Some(0.0); 15];
    let wanted = vec![0.0; 15];
    // never fed
    assert!(matches!(guards.authorise(0, &measured, &wanted)[0].1, ChainOutcome::Guarded(Verdict::HoldUncommanded)));
    guards.fed(0);
    assert!(matches!(guards.authorise(10, &measured, &wanted)[0].1, ChainOutcome::Guarded(Verdict::Move { .. })));
    // no new command while time passes tick by tick: stale after rigrs's 500 ms timeout
    assert!(matches!(guards.authorise(300, &measured, &wanted)[0].1, ChainOutcome::Guarded(Verdict::Move { .. })));
    assert!(matches!(guards.authorise(600, &measured, &wanted)[0].1, ChainOutcome::Guarded(Verdict::HoldStale { .. })));
    // the clock leaping by more than the timeout between two looks is reported as a clock fault, and also holds
    assert!(matches!(guards.authorise(5000, &measured, &wanted)[0].1, ChainOutcome::Guarded(Verdict::HoldClockFault { .. })));
    // one silent servo in the left leg: that chain has nothing to authorise against, the others still do
    let mut partial = measured.clone();
    partial[3] = None;
    guards.fed(5000);
    let verdicts = guards.authorise(5001, &partial, &wanted);
    assert_eq!(verdicts[0].1, ChainOutcome::Unmeasured);
    assert!(matches!(verdicts[1].1, ChainOutcome::Guarded(Verdict::Move { .. })));
}

#[test]
fn the_dry_run_loop_keeps_time_against_a_fake_router() {
    let robot = robot();
    let mut bridge = fake_router(standing_robot);
    let stats = control::run(&mut bridge, &robot, Duration::from_millis(10), Duration::from_millis(300), Exchange::TwoReads, None, &mut Sinks::none());
    assert!(stats.ticks >= 25 && stats.ticks <= 32, "about 30 ticks in 300 ms at 100 Hz, got {}", stats.ticks);
    assert_eq!(stats.errors, 0);
    assert_eq!(stats.unmeasured_chain_ticks, 0);
    assert_eq!(stats.moves, stats.ticks * 4, "every chain authorised on every tick");
    assert!(stats.percentile(0.5).is_some());

    let mut bare = fake_router(bare_board);
    let stats = control::run(&mut bare, &robot, Duration::from_millis(10), Duration::from_millis(100), Exchange::State, None, &mut Sinks::none());
    assert_eq!(stats.moves, 0, "with no servo answering nothing is authorised");
    assert_eq!(stats.unmeasured_chain_ticks, stats.ticks * 4);
}

#[test]
fn one_exchange_per_tick_reads_state_and_reports_what_the_firmware_did_with_targets() {
    let mut bridge = fake_router(standing_robot);
    let state = firmware::state(&mut bridge).expect("state");
    assert_eq!(state.result, "ok");
    assert_eq!(state.status.mode, "DISARMED");
    assert_eq!(state.joints.len(), 15);
    assert!(state.joints.iter().all(|j| (j.expect("measured") - 0.5f64.to_radians()).abs() < 1e-9));

    let after = firmware::step(&mut bridge, &[0.0; 15]).expect("step");
    assert_eq!(after.result, "not armed", "a disarmed firmware says so and moves nothing");

    // a target that is not a number never reaches the wire
    let mut bad = [0.0; 15];
    bad[2] = f64::NAN;
    assert!(firmware::step(&mut bridge, &bad).is_err());
    assert!(firmware::step(&mut bridge, &[0.0; 14]).is_err());

    let robot = robot();
    let stats = control::run(&mut bridge, &robot, Duration::from_millis(10), Duration::from_millis(200), Exchange::Step, None, &mut Sinks::none());
    assert_eq!(stats.errors, 0);
    assert_eq!(stats.moves, stats.ticks * 4);
}

// ---------------------------------------------------------------- policy

const TINY: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/tiny_policy");
const TRAINNR_EXPORT: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/trainnr_export_example.json");

/// The tiny policy's deployment manifest, optionally edited first.
fn manifest(edit: fn(&mut serde_json::Value)) -> Manifest {
    let path = std::path::Path::new(TINY).join("deploy.json");
    let mut d: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(path).expect("manifest")).expect("json");
    edit(&mut d);
    Manifest::parse(&d, std::path::Path::new(TINY)).expect("a manifest")
}

/// The same network, described without a gyro: its three gyro slots read gravity again instead.
fn no_gyro(d: &mut serde_json::Value) {
    d["observations"][4]["source"] = "projected_gravity".into();
}

fn level(joints: [f64; 15]) -> Sensed {
    Sensed { joints: joints.iter().map(|&j| Some(j)).collect(), pitch_roll: Some((0.0, 0.0)), angular_velocity: Some([0.0; 3]), ..Sensed::default() }
}

#[test]
fn gravity_points_down_when_upright_and_forward_when_nose_up() {
    let g = projected_gravity(0.0, 0.0);
    assert!(g[0].abs() < 1e-12 && g[1].abs() < 1e-12 && (g[2] + 1.0).abs() < 1e-12);
    let g = projected_gravity(std::f64::consts::FRAC_PI_2, 0.0);
    assert!((g[0] - 1.0).abs() < 1e-12 && g[2].abs() < 1e-12);
    let g = projected_gravity(0.2, -0.3);
    assert!((g.iter().map(|x| x * x).sum::<f64>() - 1.0).abs() < 1e-12, "always a unit vector");
}

#[test]
fn a_policy_that_does_not_fit_the_robot_or_the_model_is_refused() {
    let robot = robot();
    assert!(Policy::load(manifest(|_| ()), &robot).is_ok());
    // joints in another order
    let swapped = manifest(|d| d["joints"]["policy_order"].as_array_mut().expect("joints").swap(0, 1));
    assert!(matches!(Policy::load(swapped, &robot), Err(PolicyError::Mismatch(_))));
    // an observation layout the network was not built for (48 values, the model takes 51)
    let short = manifest(|d| {
        d["observations"].as_array_mut().expect("observations").remove(4);
    });
    assert!(matches!(Policy::load(short, &robot), Err(PolicyError::Mismatch(_))));
    // a default pose the robot cannot hold: left hip yaw at -30 degrees, its limit is -6.3
    let outside = manifest(|d| {
        d["joints"]["default_pos"][0] = (-30f64).to_radians().into();
        d["action"]["offset"][0] = d["joints"]["default_pos"][0].clone();
    });
    assert!(matches!(Policy::load(outside, &robot), Err(PolicyError::Mismatch(_))));
    // a model file that is not there
    assert!(matches!(Policy::load(manifest(|d| d["onnx"]["file"] = "/nonexistent.onnx".into()), &robot), Err(PolicyError::Load(_))));
}

#[test]
fn the_policy_turns_an_observation_into_targets_we_can_compute_by_hand() {
    // The tiny network: action = 0.5 * ELU(joint position relative to the default pose) + 0.1, per joint.
    let robot = robot();
    let mut policy = Policy::load(manifest(|_| ()), &robot).expect("policy");
    let mut joints = [0.0; 15];
    joints[0] = 0.2;
    joints[1] = -0.3;
    joints[14] = 0.4; // the beak: not the policy's to move
    let targets = policy.act(&level(joints), &[0.0; 3], 0.02).expect("act");
    assert_eq!(targets.len(), 15);
    let scale = 10f64.to_radians();
    let expect = |action: f64| action * scale;
    assert!((targets[0] - expect(0.5 * 0.2 + 0.1)).abs() < 1e-6);
    assert!((targets[1] - expect(0.5 * ((-0.3f64).exp() - 1.0) + 0.1)).abs() < 1e-6);
    assert!((targets[2] - expect(0.1)).abs() < 1e-6);
    assert!((targets[14] - 0.4).abs() < 1e-12, "the beak keeps its measured angle");

    // the second tick sees joint speeds and the previous action
    joints[0] = 0.21;
    let obs = policy.observe(&level(joints), &[0.0; 3], 0.02).expect("observe");
    assert_eq!(obs.len(), 51);
    assert!((f64::from(obs[14]) - 0.5).abs() < 1e-4, "speed of joint 0: 0.01 rad in 0.02 s");
    assert!((f64::from(obs[28]) - 0.2).abs() < 1e-6, "last action of joint 0");
    assert!((f64::from(obs[44]) + 1.0).abs() < 1e-6, "gravity z when level");

    // after a reset there is no previous tick to difference against
    policy.reset();
    let obs = policy.observe(&level(joints), &[0.0; 3], 0.02).expect("observe");
    assert_eq!(obs[14], 0.0);
    assert_eq!(obs[28], 0.0);
}

#[test]
fn the_policy_takes_joint_speeds_from_the_servos_when_they_report_them() {
    let robot = robot();
    let mut policy = Policy::load(manifest(|_| ()), &robot).expect("policy");
    let mut sensed = level([0.0; 15]);
    sensed.joint_speeds = (0..15).map(|j| Some(0.1 * j as f64)).collect();
    // Even on the first tick, when there is nothing to difference against.
    let obs = policy.observe(&sensed, &[0.0; 3], 0.02).expect("observe");
    assert!((0..14).all(|j| (f64::from(obs[14 + j]) - 0.1 * j as f64).abs() < 1e-6), "{:?}", &obs[14..28]);
    // A joint whose servo gave no speed falls back to the difference from the previous tick.
    sensed.joint_speeds[3] = None;
    sensed.joints[3] = Some(0.01);
    let obs = policy.observe(&sensed, &[0.0; 3], 0.02).expect("observe");
    assert!((f64::from(obs[14 + 3]) - 0.5).abs() < 1e-5, "0.01 rad in 0.02 s: {}", obs[17]);
}

#[test]
fn the_action_is_clipped_before_it_is_scaled() {
    let robot = robot();
    let mut policy = Policy::load(manifest(|d| d["action"]["clip"] = serde_json::json!([0.12, 0.15])), &robot).expect("policy");
    let mut joints = [0.0; 15];
    joints[0] = 0.2; // raw action 0.2: above the clip
    let targets = policy.act(&level(joints), &[0.0; 3], 0.02).expect("act");
    assert!((targets[0] - 0.15 * 10f64.to_radians()).abs() < 1e-9);
    assert!((targets[2] - 0.12 * 10f64.to_radians()).abs() < 1e-9, "raw action 0.1: below the clip");
}

#[test]
fn the_default_pose_is_counted_once() {
    // trainnr-deploy/1: target = default_pos + scale * action, and action.offset repeats default_pos.
    let robot = robot();
    let home = |d: &mut serde_json::Value| {
        d["joints"]["default_pos"][2] = 0.3.into();
        d["action"]["offset"][2] = 0.3.into();
    };
    let mut policy = Policy::load(manifest(home), &robot).expect("policy");
    let mut joints = [0.0; 15];
    joints[2] = 0.3; // at home: the network sees 0 and outputs 0.1
    let targets = policy.act(&level(joints), &[0.0; 3], 0.02).expect("act");
    assert!((targets[2] - (0.3 + 0.1 * 10f64.to_radians())).abs() < 1e-6, "{}", targets[2]);
}

#[test]
fn a_missing_measurement_stops_the_policy_instead_of_becoming_a_zero() {
    let robot = robot();
    let mut policy = Policy::load(manifest(|_| ()), &robot).expect("policy");
    let command = [0.0; 3];
    let mut silent = level([0.0; 15]);
    silent.joints[3] = None;
    assert!(matches!(policy.act(&silent, &command, 0.02), Err(PolicyError::Missing(_))));
    let mut no_imu = level([0.0; 15]);
    no_imu.pitch_roll = None;
    assert!(matches!(policy.act(&no_imu, &command, 0.02), Err(PolicyError::Missing(_))));
    let mut no_gyro_reading = level([0.0; 15]);
    no_gyro_reading.angular_velocity = None;
    assert!(matches!(policy.act(&no_gyro_reading, &command, 0.02), Err(PolicyError::Missing("angular velocity"))));
    // and a command of the wrong length is refused
    assert!(matches!(policy.act(&level([0.0; 15]), &[0.0; 5], 0.02), Err(PolicyError::Command(_))));
}

#[test]
fn the_loop_runs_a_policy_and_holds_when_the_policy_cannot_run() {
    let robot = robot();
    let period = Duration::from_millis(20);

    // A policy that needs no gyro runs on every tick, and its targets pass the guards.
    let mut policy = Policy::load(manifest(no_gyro), &robot).expect("policy");
    let mut bridge = fake_router(standing_robot);
    let command = [0.0; 3];
    let stats = control::run(&mut bridge, &robot, period, Duration::from_millis(300), Exchange::Step, Some((&mut policy, &command)), &mut Sinks::none());
    assert_eq!(stats.errors, 0);
    assert_eq!(stats.policy_ran, stats.ticks);
    assert_eq!(stats.policy_skipped, 0);
    assert_eq!(stats.moves, stats.ticks * 4);
    assert!(stats.inference_percentile(0.5).is_some());

    // A policy that needs the gyro runs when the firmware reports it ...
    let mut needs_gyro = Policy::load(manifest(|_| ()), &robot).expect("policy");
    let mut bridge = fake_router(standing_robot);
    let command = [0.0; 3];
    let stats = control::run(&mut bridge, &robot, period, Duration::from_millis(200), Exchange::Step, Some((&mut needs_gyro, &command)), &mut Sinks::none());
    assert_eq!(stats.errors, 0);
    assert_eq!(stats.policy_ran, stats.ticks);

    // ... and is skipped, holding in place, on a tick when it does not.
    let mut bridge = fake_router(standing_robot_without_gyro);
    let stats = control::run(&mut bridge, &robot, period, Duration::from_millis(200), Exchange::State, Some((&mut needs_gyro, &command)), &mut Sinks::none());
    assert_eq!(stats.policy_ran, 0);
    assert_eq!(stats.policy_skipped, stats.ticks);
    assert!(stats.last_policy_skip.as_deref().unwrap_or("").contains("angular velocity"));
    assert_eq!(stats.moves, stats.ticks * 4, "holding in place is still authorised");
}

// ---------------------------------------------------------------- status light

/// A stand-in for `/sys/class/leds` with LED1 as this board names it.
fn fake_leds(name: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("duck-leds-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    for colour in ["red", "green", "blue"] {
        let led = dir.join(format!("unoq:user-{colour}1"));
        std::fs::create_dir_all(&led).expect("led dir");
        std::fs::write(led.join("brightness"), "0").expect("brightness");
        std::fs::write(led.join("trigger"), "none [heartbeat] timer").expect("trigger");
    }
    dir
}

fn lit(dir: &std::path::Path) -> [bool; 3] {
    ["red", "green", "blue"].map(|c| std::fs::read_to_string(dir.join(format!("unoq:user-{c}1/brightness"))).is_ok_and(|v| v.trim() == "1"))
}

#[test]
fn the_status_light_shows_what_the_loop_did_and_goes_out_with_duckd() {
    let robot = robot();
    let leds = fake_leds("loop");
    {
        let mut sinks = Sinks { led: StatusLed::find_in(&leds), recorder: None, record_error: None };
        assert!(sinks.led.present());
        // holding in place, no policy: green
        control::run(&mut fake_router(standing_robot), &robot, Duration::from_millis(10), Duration::from_millis(50), Exchange::State, None, &mut sinks);
        assert_eq!(lit(&leds), [false, true, false]);
        // a policy that needs the gyro, which is not reporting: yellow
        let mut needs_gyro = Policy::load(manifest(|_| ()), &robot).expect("policy");
        let command = [0.0; 3];
        control::run(&mut fake_router(standing_robot_without_gyro), &robot, Duration::from_millis(10), Duration::from_millis(50), Exchange::State, Some((&mut needs_gyro, &command)), &mut sinks);
        assert_eq!(lit(&leds), [true, true, false]);
        // the policy running: blue
        control::run(&mut fake_router(standing_robot), &robot, Duration::from_millis(10), Duration::from_millis(50), Exchange::State, Some((&mut needs_gyro, &command)), &mut sinks);
        assert_eq!(lit(&leds), [false, false, true]);
        // the firmware not answering: red
        control::run(&mut fake_router(|_| None), &robot, Duration::from_millis(10), Duration::from_millis(30), Exchange::State, None, &mut sinks);
        assert_eq!(lit(&leds), [true, false, false]);
        sinks.led.show(Colour::White);
        assert_eq!(lit(&leds), [true, true, true]);
    }
    assert_eq!(lit(&leds), [false, false, false], "dropped: off");
    let trigger = |c: &str| std::fs::read_to_string(leds.join(format!("unoq:user-{c}1/trigger"))).expect("trigger");
    assert_eq!(trigger("red"), "heartbeat", "the trigger it found is given back");

    // While held, nothing else may drive it; a write that fails part-way leaves it off and given back.
    std::fs::write(leds.join("unoq:user-green1/trigger"), "none [timer]").expect("trigger");
    let mut led = StatusLed::find_in(&leds);
    assert_eq!(trigger("green"), "none");
    led.show(Colour::White);
    std::fs::remove_dir_all(leds.join("unoq:user-blue1")).expect("remove blue");
    led.show(Colour::Red);
    assert!(!led.present());
    assert_eq!(lit(&leds)[..2], [false, false]);
    assert_eq!(trigger("green"), "timer");
    drop(led);

    // One it cannot write is not claimed at all.
    let readonly = fake_leds("readonly");
    let brightness = readonly.join("unoq:user-red1/brightness");
    let mut perms = std::fs::metadata(&brightness).expect("meta").permissions();
    perms.set_readonly(true);
    std::fs::set_permissions(&brightness, perms).expect("chmod");
    assert!(!StatusLed::find_in(&readonly).present());
    let _ = std::fs::remove_dir_all(&leds);
    let _ = std::fs::remove_dir_all(&readonly);
    // a machine without the LED: nothing happens, nothing fails
    let mut absent = StatusLed::find_in(std::path::Path::new("/nonexistent"));
    assert!(!absent.present());
    absent.show(Colour::Red);
}

#[test]
fn text_for_the_matrix_reaches_the_firmware() {
    assert_eq!(firmware::say(&mut fake_router(bare_board), "hello").expect("say"), 5);
}

// ---------------------------------------------------------------- recording

/// Every message of a recording: topic, log time, CDR bytes. Checks each is CDR.
fn read_mcap(path: &std::path::Path) -> Vec<(String, u64, Vec<u8>)> {
    let bytes = std::fs::read(path).expect("read back");
    mcap::MessageStream::new(&bytes)
        .expect("an MCAP")
        .map(|m| {
            let m = m.expect("a message");
            assert_eq!(m.channel.message_encoding, "cdr");
            (m.channel.topic.clone(), m.log_time, m.data.to_vec())
        })
        .collect()
}

#[test]
fn a_recording_holds_every_tick_as_ros2_messages() {
    let robot = robot();
    let path = std::env::temp_dir().join(format!("duck-record-{}.mcap", std::process::id()));
    let mut sinks = Sinks { led: StatusLed::none(), recorder: Some(Recorder::create(&path).expect("create")), record_error: None };
    let stats = control::run(&mut fake_router(standing_robot), &robot, Duration::from_millis(10), Duration::from_millis(100), Exchange::Step, None, &mut sinks);
    assert_eq!(sinks.record_error, None);
    let recorder = sinks.recorder.take().expect("still recording");
    assert_eq!(recorder.finish().expect("finish"), stats.ticks);

    let messages = read_mcap(&path);
    let count = |topic: &str| messages.iter().filter(|m| m.0 == topic).count();
    let ticks = stats.ticks as usize;
    assert_eq!(count("/joint_states"), ticks);
    assert_eq!(count("/imu"), ticks, "the IMU reports on every tick of this robot");
    // targets go out from the second tick on (the first has nothing authorised yet)
    assert_eq!(count("/joint_commands"), ticks - 1);
    let first_state = messages.iter().find(|m| m.0 == "/joint_states").map(|m| m.2.clone());

    // The first joint state: CDR header, stamp, empty frame id, fifteen names, fifteen positions of 0.5 degrees.
    let data = first_state.expect("a joint state");
    assert_eq!(&data[..4], &[0, 1, 0, 0]);
    let u32_at = |i: usize| u32::from_le_bytes(data[i..i + 4].try_into().expect("4 bytes"));
    assert_eq!(u32_at(12), 1, "frame_id is the empty string");
    assert_eq!(u32_at(20), 15, "one name per joint");
    let name_len = u32_at(24) as usize;
    assert_eq!(&data[28..28 + name_len - 1], b"left_hip_yaw");
    // From the end: effort (empty), fifteen velocities, their length and padding to 8 bytes, fifteen positions.
    let f64s = |bytes: &[u8]| bytes.chunks(8).map(|b| f64::from_le_bytes(b.try_into().expect("8 bytes"))).collect::<Vec<f64>>();
    let end = data.len() - 4;
    assert_eq!(&data[end..], &[0u8; 4], "effort is empty");
    let velocities = f64s(&data[end - 8 * 15..end]);
    assert!(velocities.iter().all(|v| (v - 2f64.to_radians()).abs() < 1e-12), "the servos' own speeds: {velocities:?}");
    assert_eq!(u32_at(end - 8 * 15 - 8), 15, "one velocity per joint");
    let positions = f64s(&data[end - 8 * 15 - 8 - 8 * 15..end - 8 * 15 - 8]);
    assert!(positions.iter().all(|p| (p - 0.5f64.to_radians()).abs() < 1e-12), "{positions:?}");
    // DUCK_KEEP_RECORDING=<path> keeps a copy, to check the file with another MCAP / ROS 2 reader.
    if let Ok(keep) = std::env::var("DUCK_KEEP_RECORDING") {
        std::fs::copy(&path, keep).expect("keep a copy");
    }
    let _ = std::fs::remove_file(&path);
}

#[test]
fn nose_up_is_a_negative_rotation_about_y() {
    let q = orientation(0.2, 0.0);
    assert!(q[0].abs() < 1e-12 && q[2].abs() < 1e-12);
    assert!((q[1] + (0.1f64).sin()).abs() < 1e-12 && (q[3] - (0.1f64).cos()).abs() < 1e-12);
    let q = orientation(0.0, 0.3); // left side up: positive about x
    assert!((q[0] - (0.15f64).sin()).abs() < 1e-12);
}

// ---------------------------------------------------------------- bench


/// A bench build with the left knee armed: each step reply reads the servo 10 ms later than the last, at 2 degrees.
static BENCH_READ_MS: AtomicU32 = AtomicU32::new(5000);
fn bench_firmware(method: &str) -> Option<Value> {
    match method {
        "duck_info" => Some(ints(&[3, 1, 0, 0, 0, 1, 1])),
        "duck_bench_arm" | "duck_bench_disarm" => Some(Value::from(0)),
        "duck_bench_step" => {
            let read_ms = BENCH_READ_MS.fetch_add(10, Ordering::Relaxed);
            Some(ints(&[0, 1, 0, 3, 1, 20, -15, 125, 50, 31, i64::from(read_ms), 20]))
        }
        other => bare_board(other),
    }
}

/// The same, but the servo overheats on the third tick and the firmware switches it off.
static FAULT_CALLS: AtomicU32 = AtomicU32::new(0);
fn overheating_bench(method: &str) -> Option<Value> {
    // its own replies: the tests run in parallel, and BENCH_READ_MS belongs to the other one
    match method {
        "duck_bench_step" if FAULT_CALLS.fetch_add(1, Ordering::Relaxed) >= 2 => Some(ints(&[6, 2, 2, 3, 1, 20, 0, 0, 50, 90, 0, 20])),
        "duck_bench_step" => Some(ints(&[0, 1, 0, 3, 1, 20, 0, 0, 50, 60, 0, 20])),
        other => bare_board(other),
    }
}

#[test]
fn the_excitation_starts_and_ends_at_rest() {
    for excitation in [Excitation::Chirp { from_hz: 0.1, to_hz: 2.0 }, Excitation::Steps { period_s: 2.0 }] {
        assert_eq!(excitation.at(0.0, 20.0), 0.0);
        assert!(excitation.at(20.0, 20.0).abs() < 1e-12);
        assert!((0..2000).all(|i| excitation.at(f64::from(i) * 0.01, 20.0).abs() <= 1.0));
    }
    assert_eq!(Excitation::Steps { period_s: 2.0 }.at(1.5, 20.0), -1.0);
    assert_eq!(Excitation::Steps { period_s: 2.0 }.at(2.5, 20.0), 1.0);
    assert_eq!(Excitation::Chirp { from_hz: 1.0, to_hz: 1.0 }.at(-1.0, 20.0), 0.0, "outside the run: rest");
}

#[test]
fn a_bench_run_is_recorded_with_the_servos_own_read_times() {
    let mut bridge = fake_router(bench_firmware);
    assert!(firmware::info(&mut bridge).expect("info").bench_build);
    assert_eq!(firmware::bench_arm(&mut bridge, 3).expect("arm"), Ok(()));
    let first = firmware::bench_step(&mut bridge, None).expect("read");
    assert_eq!((first.mode, first.joint), ("ARMED", Some(3)));
    let r = first.reading.expect("a read");
    assert!((r.position - 2f64.to_radians()).abs() < 1e-12 && (r.velocity + 1.5f64.to_radians()).abs() < 1e-12);
    assert!((r.load_percent - 12.5).abs() < 1e-12 && (r.voltage - 5.0).abs() < 1e-12 && r.temperature_c == 31);

    let run = bench::run(&mut bridge, 0.0, 0.1, Excitation::Chirp { from_hz: 0.5, to_hz: 2.0 }, Duration::from_millis(300), Duration::from_millis(20));
    assert_eq!(run.stopped, None);
    assert!(run.samples.len() >= 10, "{} ticks", run.samples.len());

    let path = std::env::temp_dir().join(format!("duck-bench-{}.mcap", std::process::id()));
    let mut recorder = Recorder::create(&path).expect("create");
    let reads = recorder.bench("left_knee", &run.samples).expect("bench");
    recorder.finish().expect("finish");
    assert_eq!(reads, run.samples.len(), "every reply carried a new read");
    let messages = read_mcap(&path);
    let on = |topic: &'static str| messages.iter().filter(move |m| m.0 == topic);
    assert!(messages.iter().all(|m| ["/joint_states", "/joint_commands", "/joint_targets"].contains(&m.0.as_str())));
    let state_times: Vec<u64> = on("/joint_states").map(|m| m.1).collect();
    let commands: Vec<(u64, Vec<u8>)> = on("/joint_commands").map(|m| (m.1, m.2.clone())).collect();
    let targets = on("/joint_targets").count();
    assert_eq!(targets, run.samples.len(), "every target asked for");
    assert_eq!(commands.len(), reads, "one written goal per read");
    // The command is the firmware's setpoint (2 degrees in every reply here), stamped with its read.
    let (t, data) = &commands[0];
    assert_eq!(*t, state_times[0]);
    let last = f64::from_le_bytes(data[data.len() - 16..data.len() - 8].try_into().expect("8 bytes"));
    assert!((last - 2f64.to_radians()).abs() < 1e-12, "{last}");
    // Stamped by the microcontroller's clock: exactly 10 ms apart, whatever the link did.
    assert!(state_times.windows(2).all(|w| w[1] - w[0] == 10_000_000), "{state_times:?}");
    if let Ok(keep) = std::env::var("DUCK_KEEP_BENCH") {
        std::fs::copy(&path, keep).expect("keep a copy");
    }
    let _ = std::fs::remove_file(&path);
}

#[test]
fn a_bench_run_stops_when_the_firmware_switches_the_servo_off() {
    let mut bridge = fake_router(overheating_bench);
    let run = bench::run(&mut bridge, 0.0, 0.1, Excitation::Steps { period_s: 1.0 }, Duration::from_secs(5), Duration::from_millis(10));
    assert_eq!(run.samples.len(), 3);
    let why = run.stopped.expect("stopped");
    assert!(why.contains("FAULT") && why.contains("over-temperature"), "{why}");
    assert_eq!(run.last.map(|b| b.result), Some("fault latched"));
}

/// The link garbles the third and the tenth call (the method's name arrives wrong, so the sketch does not know
/// it), then every call from the twentieth on.
static GARBLED_CALLS: AtomicU32 = AtomicU32::new(0);
fn garbling_bench(method: &str) -> Option<Value> {
    match method {
        "duck_bench_step" => match GARBLED_CALLS.fetch_add(1, Ordering::Relaxed) {
            2 | 9 | 19.. => None,
            _ => Some(ints(&[0, 1, 0, 3, 1, 20, 0, 0, 50, 60, 0, 20])),
        },
        other => bare_board(other),
    }
}

#[test]
fn a_bench_run_passes_over_one_garbled_call_and_stops_when_the_link_stays_down() {
    let mut bridge = fake_router(garbling_bench);
    let run = bench::run(&mut bridge, 0.0, 0.1, Excitation::Steps { period_s: 1.0 }, Duration::from_secs(5), Duration::from_millis(10));
    assert_eq!(run.samples.len(), 17, "the calls that were answered before the link went down");
    assert!(run.lost_exchanges > 2, "two passed over, then the ones that ended it: {}", run.lost_exchanges);
    let why = run.stopped.expect("stopped");
    assert!(why.contains("the link failed"), "{why}");
    // well inside the firmware's own 0.5 s of silence
    assert!(run.lost_exchanges <= 2 + 30, "{} lost: it gave up after {:?}", run.lost_exchanges, bench::LINK_PATIENCE);
}

// ---------------------------------------------------------------- hearing

/// Made-up sound, 16 kHz: `noise` is the room's hiss (peak amplitude), claps are 30 ms bursts that die away.
struct Sound {
    samples: Vec<i16>,
    seed: u32,
}

impl Sound {
    fn new() -> Self {
        Sound { samples: Vec::new(), seed: 12345 }
    }
    fn random(&mut self) -> f64 {
        self.seed = self.seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        f64::from(self.seed >> 8) / f64::from(1u32 << 24) * 2.0 - 1.0
    }
    fn noise(mut self, seconds: f64, amplitude: f64) -> Self {
        for _ in 0..(seconds * f64::from(hear::RATE)) as usize {
            let v = self.random() * amplitude;
            self.samples.push(v as i16);
        }
        self
    }
    fn clap(mut self, amplitude: f64) -> Self {
        let n = (0.030 * f64::from(hear::RATE)) as usize;
        for i in 0..n {
            let v = self.random() * amplitude * (-(i as f64) / (n as f64) * 5.0).exp();
            self.samples.push(v as i16);
        }
        self
    }
    /// Everything `Ears` reports for this sound, with the frame it was reported on.
    fn heard(&self) -> Vec<(usize, Heard)> {
        let bytes: Vec<u8> = self.samples.iter().flat_map(|s| s.to_le_bytes()).collect();
        let (mut out, mut frame) = (Vec::new(), 0);
        hear::listen(&mut bytes.as_slice(), |_, _, heard| {
            frame += 1;
            out.extend(heard.map(|h| (frame, h)));
            true
        })
        .expect("listen");
        out
    }
}

#[test]
fn loudness_is_measured_against_full_scale() {
    assert_eq!(hear::level_db(&[0; hear::FRAME]), hear::SILENCE_DB);
    assert_eq!(hear::level_db(&[]), hear::SILENCE_DB);
    assert!(hear::level_db(&[i16::MAX; hear::FRAME]).abs() < 1e-9, "a full-scale square wave is 0 dB");
    let half = hear::level_db(&[i16::MAX / 2; hear::FRAME]);
    assert!((half + 6.02).abs() < 0.01, "half the amplitude is 6 dB down: {half}");
}

#[test]
fn claps_are_counted_in_groups() {
    let quiet = 30.0; // about -63 dB
    for n in 1..=3u32 {
        let mut sound = Sound::new().noise(1.0, quiet);
        for _ in 0..n {
            sound = sound.clap(12_000.0).noise(0.3, quiet);
        }
        let heard = sound.noise(1.0, quiet).heard();
        assert_eq!(heard.iter().map(|h| h.1).collect::<Vec<_>>(), vec![Heard::Claps(n)], "{n} clap(s)");
    }
    // Two pairs far apart are two groups, each reported about GROUP_GAP after its last clap.
    let pairs = Sound::new().noise(1.0, quiet).clap(12_000.0).noise(0.3, quiet).clap(12_000.0).noise(2.0, quiet)
        .clap(12_000.0).noise(0.3, quiet).clap(12_000.0).noise(1.0, quiet);
    let heard = pairs.heard();
    assert_eq!(heard.iter().map(|h| h.1).collect::<Vec<_>>(), vec![Heard::Claps(2), Heard::Claps(2)]);
    let reported_s = heard[0].0 as f64 * hear::FRAME_TIME.as_secs_f64();
    assert!((1.9..2.1).contains(&reported_s), "the first pair ended at 1.36 s and was reported at {reported_s} s");
}

#[test]
fn a_long_loud_sound_is_not_a_clap_and_a_clap_is_still_heard_over_it() {
    let quiet = 30.0;
    // A second of something loud (a servo, speech): no clap.
    let long = Sound::new().noise(1.0, quiet).noise(1.0, 12_000.0).noise(1.0, quiet);
    assert_eq!(long.heard(), vec![]);
    // A steady hum 30 dB over the quiet room becomes the room's noise; two loud claps over it are still two claps.
    let over_hum = Sound::new().noise(1.0, quiet).noise(4.0, 1_000.0).clap(30_000.0).noise(0.3, 1_000.0).clap(30_000.0).noise(1.0, 1_000.0);
    assert_eq!(over_hum.heard().iter().map(|h| h.1).collect::<Vec<_>>(), vec![Heard::Claps(2)]);
    // A quiet tap, 10 dB over the room, is not a clap.
    let tap = Sound::new().noise(1.0, quiet).clap(300.0).noise(1.0, quiet);
    assert_eq!(tap.heard(), vec![]);
}

#[test]
fn a_command_from_outside_the_loop_replaces_the_fixed_one_and_runs_out() {
    let robot = robot();
    let period = Duration::from_millis(20);
    let mut policy = Policy::load(manifest(no_gyro), &robot).expect("policy");
    // The fixed command is one the policy cannot take: used, it is refused every tick. With a direction it is not
    // used at all.
    let fixed = [1e6, 0.0, 0.0];
    let run = |policy: &mut Policy, direction: Option<&Direction>| {
        control::run_directed(&mut fake_router(standing_robot), &robot, period, Duration::from_millis(200), Exchange::Step, Some((policy, &fixed)), direction, &mut Sinks::none())
    };
    let stats = run(&mut policy, None);
    assert_eq!((stats.policy_ran, stats.policy_skipped), (0, stats.ticks));
    // Nothing asked: the policy runs on a command of zeros.
    let direction = Direction::new();
    assert_eq!(direction.now(), None);
    let stats = run(&mut policy, Some(&direction));
    assert_eq!((stats.policy_ran, stats.policy_skipped), (stats.ticks, 0));
    // Something asked that the policy cannot take: refused, as the fixed one was. Cleared: it runs again.
    direction.set(&[0.0, 1e6, 0.0]);
    let stats = run(&mut policy, Some(&direction));
    assert_eq!((stats.policy_ran, stats.policy_skipped), (0, stats.ticks));
    direction.clear();
    assert_eq!(run(&mut policy, Some(&direction)).policy_skipped, 0);
    // A command of the wrong length is nothing asked; one given for a time is gone after it.
    direction.set(&[0.0; 5]);
    assert_eq!(run(&mut policy, Some(&direction)).policy_skipped, 0);
    direction.set_for(&[0.1, 0.0, 0.0], Duration::from_millis(30));
    assert_eq!(direction.now(), Some(vec![0.1, 0.0, 0.0]));
    thread::sleep(Duration::from_millis(40));
    assert_eq!(direction.now(), None, "a source that stops leaves nothing asked");
}

#[test]
fn chasing_turns_to_face_the_thing_then_walks_to_it_and_stops_within_reach() {
    let chase = Chase::default();
    let ranges = [(-0.2, 0.4), (-0.3, 0.3), (-1.0, 1.0)];
    // Far and dead ahead: all the speed the policy was trained for, no turn.
    assert_eq!(chase.command(0.0, Some(2.0), &ranges), [0.4, 0.0, 0.0]);
    // Off to the left: turn left, walk slower; far off to the side: turn on the spot, as fast as the policy may.
    let [forward, sideways, turn] = chase.command(0.3, Some(2.0), &ranges);
    assert!(forward > 0.1 && forward < 0.4 && sideways == 0.0 && turn > 0.5, "{forward} {turn}");
    assert_eq!(chase.command(-1.2, Some(2.0), &ranges), [0.0, 0.0, -1.0]);
    // Nearer is slower, never backwards; within reach it stops and has caught it.
    let near = chase.command(0.0, Some(0.2), &ranges)[0];
    assert!(near > 0.0 && near < 0.2, "{near}");
    assert_eq!(chase.command(0.2, Some(0.10), &ranges), [0.0, 0.0, 0.0]);
    assert!(chase.caught(Some(0.10)) && !chase.caught(Some(0.2)) && !chase.caught(None));
    // A thing of unknown size: its direction is all there is, so it is approached at half speed.
    assert_eq!(chase.command(0.0, None, &ranges), [0.2, 0.0, 0.0]);
    // Every command is one the policy can take.
    for bearing in [-3.0, -0.4, 0.0, 0.4, 3.0] {
        for distance in [None, Some(0.05), Some(0.3), Some(5.0)] {
            let command = chase.command(bearing, distance, &ranges);
            assert!(command.iter().zip(&ranges).all(|(c, (low, high))| low <= c && c <= high), "{command:?}");
        }
    }
}

// ---------------------------------------------------------------- seeing

/// A made-up YUYV frame: a grey room with discs of colour, each (centre x, centre y, radius, U, V).
fn picture(discs: &[(f64, f64, f64, u8, u8)]) -> Vec<u8> {
    let mut frame = Vec::with_capacity(see::FRAME_BYTES);
    for y in 0..see::HEIGHT {
        for pair in 0..see::WIDTH / 2 {
            let x = (pair * 2) as f64 + 1.0;
            let disc = discs.iter().find(|d| (x - d.0).hypot(y as f64 + 0.5 - d.1) <= d.2);
            let (u, v) = disc.map_or((128, 128), |d| (d.3, d.4));
            frame.extend([120, u, 120, v]);
        }
    }
    frame
}

#[test]
fn a_coloured_ball_is_found_where_it_is() {
    let orange = Chroma::new(90, 190);
    // A ball 40 px across, left of and above the middle.
    let frame = picture(&[(100.0, 80.0, 20.0, 92, 186)]);
    let blob = see::find(&frame, &orange, 12).expect("the ball");
    assert!((blob.x - 100.0).abs() <= 2.0 && (blob.y - 80.0).abs() <= 2.0, "{blob:?}");
    assert!((blob.width - 40.0).abs() <= 4.0 && (blob.height - 40.0).abs() <= 4.0, "{blob:?}");
    assert!((0.7..0.9).contains(&blob.fill()), "a disc fills pi/4 of its box: {}", blob.fill());

    // The camera sees 60 degrees from left to right: the ball is to the left and up, and 0.28 m away if it is 40 mm.
    let lens = Lens::from_fov(60.0);
    assert!((lens.focal_px - 277.1).abs() < 0.1);
    let (left, up) = (lens.bearing(&blob).to_degrees(), lens.elevation(&blob).to_degrees());
    assert!((left - 12.2).abs() < 0.6 && (up - 8.2).abs() < 0.6, "{left} left, {up} up");
    assert!((lens.distance(&blob, 0.040) - 0.277).abs() < 0.03);
    // The same ball held at a known distance measures the lens back.
    assert!((Lens::from_ball(&blob, 0.040, 0.277).focal_px - lens.focal_px).abs() < 30.0);

    // Dead ahead: no angle either way.
    let ahead = see::find(&picture(&[(160.0, 120.0, 10.0, 90, 190)]), &orange, 12).expect("the ball");
    assert!(lens.bearing(&ahead).abs() < 0.01 && lens.elevation(&ahead).abs() < 0.01);
}

#[test]
fn only_the_largest_patch_of_the_right_colour_counts() {
    let orange = Chroma::new(90, 190);
    // Nothing of that colour: a grey room, a blue ball, a dark patch of the right colour's place.
    assert_eq!(see::find(&picture(&[]), &orange, 12), None);
    assert_eq!(see::find(&picture(&[(100.0, 80.0, 20.0, 200, 100)]), &orange, 12), None);
    // Specks of the colour are not the ball; a small ball and a large one: the large one.
    assert_eq!(see::find(&picture(&[(40.0, 40.0, 2.0, 90, 190), (300.0, 200.0, 2.0, 90, 190)]), &orange, 12), None);
    let two = picture(&[(60.0, 60.0, 8.0, 90, 190), (220.0, 150.0, 25.0, 90, 190)]);
    let blob = see::find(&two, &orange, 12).expect("the large ball");
    assert!((blob.x - 220.0).abs() <= 2.0 && (blob.y - 150.0).abs() <= 2.0, "{blob:?}");
    // A frame of another size is not looked at.
    assert_eq!(see::find(&two[..two.len() - 4], &orange, 12), None);
    // A ball cut off by the picture's edge still gives its distance from its larger side.
    let cut = see::find(&picture(&[(0.0, 120.0, 20.0, 90, 190)]), &orange, 12).expect("half a ball");
    assert!(cut.width < 24.0 && (cut.height - 40.0).abs() <= 4.0, "{cut:?}");
    assert!((Lens::from_fov(60.0).distance(&cut, 0.040) - 0.277).abs() < 0.03);
}

#[test]
fn the_colour_in_the_middle_of_the_picture_is_learnt() {
    let held = picture(&[(160.0, 120.0, 60.0, 77, 201)]);
    let colour = see::learn(&held).expect("a colour");
    assert_eq!((colour.u, colour.v), (77, 201));
    assert!(colour.strength() > 80.0);
    assert!(see::learn(&picture(&[])).expect("grey").strength() < 1.0, "a grey wall has no colour to look for");
    assert!(see::find(&held, &colour, 12).is_some());
    // Frames come off a stream one after another, and the stream's end is not an error.
    let two = [held.clone(), picture(&[])].concat();
    let (mut stream, mut frame) = (&two[..two.len() - 10], Vec::new());
    assert!(see::read_frame(&mut stream, &mut frame).expect("first"));
    assert_eq!(frame, held);
    assert!(!see::read_frame(&mut stream, &mut frame).expect("a cut second frame is the end"));
}

// ---------------------------------------------------------------- one joint as a pet

/// A made-up servo for the pet: it follows its target with a short lag, unless a hand holds it at `held_at`.
/// Runs `seconds` at 50 Hz and returns every tick's (time, target, position, mood).
fn pet_run(seconds: f64, look: impl Fn(f64) -> Option<f64>, held_at: impl Fn(f64) -> Option<f64>) -> Vec<(f64, f64, f64, Mood)> {
    let dt = 0.02;
    let (mut pet, mut position, mut target) = (Pet::new(0.0), 0.0f64, 0.0f64);
    (0..(seconds / dt) as usize)
        .map(|i| {
            let t = i as f64 * dt;
            position = held_at(t).unwrap_or(position + (target - position) * (dt / 0.05_f64).min(1.0));
            let (next, mood) = pet.tick(dt, position, look(t));
            target = next;
            (t, next, position, mood)
        })
        .collect()
}

#[test]
fn a_free_joint_looks_and_glances_and_never_struggles() {
    // Nothing to look at: it glances about and comes home, inside its glance.
    let idle = pet_run(20.0, |_| None, |_| None);
    assert!(idle.iter().all(|r| r.3 == Mood::Idle), "{:?}", idle.iter().find(|r| r.3 != Mood::Idle));
    assert!(idle.iter().any(|r| r.1 >= pet::GLANCE_DEG - 0.1) && idle.iter().any(|r| r.1 <= -pet::GLANCE_DEG + 0.1));
    assert!(idle.iter().all(|r| r.1.abs() <= pet::GLANCE_DEG + 1e-9));

    // Something moves across in front of it: it follows, at its own pace, and ends up pointing at it.
    let watching = pet_run(6.0, |t| Some((40.0 * (t - 1.0)).clamp(-30.0, 45.0)), |_| None);
    assert!(watching.iter().all(|r| r.3 == Mood::Watching));
    let last = watching.last().expect("ticks");
    assert!((last.2 - 45.0).abs() < 1.0, "pointing at it: {last:?}");
    assert!(watching.windows(2).all(|w| (w[1].1 - w[0].1).abs() <= pet::GAZE_SPEED_DPS * 0.02 + 1e-9), "never faster than its gaze speed");

    // Something far off to the side: it turns as far as its range and no further.
    let far = pet_run(3.0, |_| Some(170.0), |_| None);
    assert!(far.iter().all(|r| r.1 <= pet::RANGE_DEG + 1e-9) && (far.last().expect("ticks").1 - pet::RANGE_DEG).abs() < 1e-9);
}

#[test]
fn a_held_joint_struggles_in_bursts_then_gives_up_and_is_calm_again_when_let_go() {
    // A hand holds it at 0 from the start to 14 s. Its first glance (2.5 s) finds it cannot move.
    let run = pet_run(22.0, |_| None, |t| (t < 14.0).then_some(0.0));
    let moods: Vec<(f64, Mood)> = run.iter().map(|r| (r.0, r.3)).fold(Vec::new(), |mut seen, (t, m)| {
        if seen.last().map(|l: &(f64, Mood)| l.1) != Some(m) {
            seen.push((t, m));
        }
        seen
    });
    let order: Vec<Mood> = moods.iter().map(|m| m.1).collect();
    use Mood::{Checking, GivenUp, Idle, Struggling};
    assert_eq!(order[..10], [Idle, Struggling, Checking, Struggling, Checking, Struggling, Checking, Struggling, Checking, GivenUp], "{moods:?}");
    let noticed = moods[1].0;
    assert!((2.5..3.0).contains(&noticed), "held is noticed on the first glance, {noticed} s");
    // A burst goes both ways about where it was held, and no further than a struggle's size.
    let burst: Vec<f64> = run.iter().filter(|r| r.3 == Struggling).map(|r| r.1).collect();
    assert!(burst.iter().any(|t| *t > 5.0) && burst.iter().any(|t| *t < -5.0));
    assert!(burst.iter().all(|t| t.abs() <= pet::STRUGGLE_DEG + 1e-9));
    // Given up: it is told to be wherever the hand has it, so it does not push.
    assert!(run.iter().filter(|r| r.3 == GivenUp).all(|r| (r.1 - r.2).abs() < 1e-9));
    // Still held after giving up: it tries again. Let go at 14 s: calm by the end, and it stays calm.
    assert!(order[10..].contains(&Struggling), "{moods:?}");
    assert!(run.iter().filter(|r| r.0 > 19.0).all(|r| r.3 == Idle), "{moods:?}");

    // Let go during the first burst: one check, then calm.
    let brief = pet_run(8.0, |_| None, |t| (t < 2.8).then_some(0.0));
    let order: Vec<Mood> = brief.iter().map(|r| r.3).fold(Vec::new(), |mut seen, m| {
        if seen.last() != Some(&m) {
            seen.push(m);
        }
        seen
    });
    assert_eq!(order, [Idle, Struggling, Checking, Idle]);
}

#[test]
fn a_trainnr_export_is_read_as_trainnr_wrote_it() {
    // deploy.json exactly as trainnr's export_deployment wrote it (a 50-iteration pipeline check on Pollen's microduck,
    // whose joints are named and ordered as ours; the network itself is not kept here).
    let m = Manifest::load(TRAINNR_EXPORT).expect("trainnr's manifest");
    assert!((m.control_hz - 50.0).abs() < 1e-9, "one tick per physics_timestep_s * decimation");
    assert_eq!(m.joints, JOINT_NAMES[..14].iter().map(|s| s.to_string()).collect::<Vec<_>>());
    assert_eq!(m.observation_size(), 51);
    assert_eq!(m.command_size(), 3);
    let terms: Vec<_> = m.observation.iter().map(|o| o.term).collect();
    assert_eq!(
        terms,
        [Term::AngularVelocity, Term::ProjectedGravity, Term::JointPositions, Term::JointVelocities, Term::LastAction, Term::Command]
    );
    assert_eq!(m.action_clip, None);
    assert_eq!(m.command_ranges, [(-0.4, 0.4), (-0.3, 0.3), (-1.0, 1.0)]);
    assert!(m.onnx.ends_with("policy.onnx"));
}

#[test]
fn a_deployment_this_runtime_cannot_honour_is_refused() {
    let base: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(TRAINNR_EXPORT).expect("read")).expect("json");
    let refused = |edit: fn(&mut serde_json::Value)| {
        let mut d = base.clone();
        edit(&mut d);
        Manifest::parse(&d, std::path::Path::new(".")).err()
    };
    assert!(refused(|_| ()).is_none());
    assert!(matches!(refused(|d| d["schema"] = "trainnr-deploy/2".into()), Some(PolicyError::Mismatch(_))));
    assert!(matches!(refused(|d| d["observations"][0]["history_length"] = 3.into()), Some(PolicyError::Mismatch(_))));
    assert!(matches!(refused(|d| d["observations"][0]["source"] = "base_lin_vel".into()), Some(PolicyError::Mismatch(_))));
    assert!(matches!(refused(|d| d["observations"][2]["width"] = 15.into()), Some(PolicyError::Mismatch(_))));
    assert!(matches!(refused(|d| d["joints"]["action_to_ctrl"][0] = 1.into()), Some(PolicyError::Mismatch(_))));
    assert!(matches!(refused(|d| d["action"]["kind"] = "joint velocity target".into()), Some(PolicyError::Mismatch(_))));
    // what trainnr's own loader refuses: a key or a source it does not know, a missing required key
    assert!(matches!(refused(|d| d["gains"] = 1.into()), Some(PolicyError::Mismatch(_))));
    assert!(matches!(refused(|d| d["observations"][0]["source"] = "sensor robot/imu_lin_vel".into()), Some(PolicyError::Mismatch(_))));
    assert!(matches!(refused(|d| d["observations"][5]["source"] = "gait_phase".into()), Some(PolicyError::Mismatch(_))));
    for key in ["run", "robot", "scene", "commands", "control"] {
        let mut d = base.clone();
        d.as_object_mut().expect("object").remove(key);
        let why = Manifest::parse(&d, std::path::Path::new(".")).expect_err("refused").to_string();
        assert!(why.contains(key), "{why}");
    }
    // rules the format states, broken
    assert!(matches!(refused(|d| d["action"]["offset"][0] = 0.1.into()), Some(PolicyError::Mismatch(_))));
    assert!(matches!(refused(|d| d["action"]["clip"] = 1.0.into()), Some(PolicyError::Mismatch(_))));
    assert!(matches!(refused(|d| d["action"]["clip"] = serde_json::json!([1.0, -1.0])), Some(PolicyError::Mismatch(_))));
    assert!(matches!(refused(|d| d["observations"][0]["scale"] = serde_json::json!([1.0, 2.0])), Some(PolicyError::Mismatch(_))));
    assert!(matches!(refused(|d| d["commands"]["twist"]["lin_vel_y"] = serde_json::Value::Null), Some(PolicyError::Mismatch(_))));
}

#[test]
fn observation_terms_are_scaled_then_clipped_as_trainnr_specifies() {
    let robot = robot();
    let mut policy = Policy::load(
        manifest(|d| {
            d["observations"][0]["scale"] = 2.0.into();
            d["observations"][0]["clip"] = serde_json::json!([-0.1, 0.5]);
            d["observations"][3]["scale"] = serde_json::json!([1.0, 1.0, 3.0]);
        }),
        &robot,
    )
    .expect("policy");
    let mut joints = [0.0; 15];
    joints[0] = 0.3; // doubled to 0.6, clipped to 0.5
    joints[1] = -0.04; // doubled to -0.08: inside the clip
    joints[2] = -0.3; // doubled to -0.6, clipped to -0.1
    let obs = policy.observe(&level(joints), &[0.0; 3], 0.02).expect("observe");
    let near = |i: usize, v: f64| (f64::from(obs[i]) - v).abs() < 1e-6;
    assert!(near(0, 0.5) && near(1, -0.08) && near(2, -0.1), "{:?}", &obs[..3]);
    assert!(near(42, 0.0) && near(44, -3.0), "gravity, scaled per value: {:?}", &obs[42..45]);
}

const TINY_LEGS: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/tiny_policy_legs");
const TRAINNR_EXPORT_HEAD_HELD: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/trainnr_export_head_held.json");

#[test]
fn a_trainnr_export_with_the_head_held_is_read_as_trainnr_wrote_it() {
    // deploy.json exactly as trainnr's export_deployment wrote it for a walk declared on a copy of this robot's
    // model with neck and head pinned (trainnr's own test of its legged-walk family, 10 iterations; the network is
    // not kept here). The policy observes fourteen joints and drives the ten leg joints.
    let m = Manifest::load(TRAINNR_EXPORT_HEAD_HELD).expect("trainnr's manifest");
    assert_eq!(m.joints, JOINT_NAMES[..14].iter().map(|s| s.to_string()).collect::<Vec<_>>());
    assert_eq!(m.acting, [0, 1, 2, 3, 4, 9, 10, 11, 12, 13], "the legs, in the action's order");
    assert_eq!(m.action_scale.len(), 10);
    assert_eq!(m.observation_size(), 47, "3 + 3 + 14 + 14 + 10 (the last action) + 3");
    assert_eq!(m.limits.as_deref(), Some("limits@3ac96bead5fa (terminate)"));
    let all = Manifest::load(TRAINNR_EXPORT).expect("trainnr's manifest");
    assert_eq!(all.acting, (0..14).collect::<Vec<_>>(), "without action_joints the action drives every joint");
    assert_eq!(all.limits, None);

    let base: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(TRAINNR_EXPORT_HEAD_HELD).expect("read")).expect("json");
    let refused = |edit: fn(&mut serde_json::Value)| {
        let mut d = base.clone();
        edit(&mut d);
        Manifest::parse(&d, std::path::Path::new(".")).err().map(|e| e.to_string())
    };
    assert_eq!(refused(|_| ()), None);
    let unknown = refused(|d| d["joints"]["action_joints"][0] = "tail".into()).expect("refused");
    assert!(unknown.contains("tail"), "{unknown}");
    let twice = refused(|d| d["joints"]["action_joints"][1] = "left_hip_yaw".into()).expect("refused");
    assert!(twice.contains("twice"), "{twice}");
    assert!(refused(|d| d["joints"]["action_joints"] = serde_json::json!([])).is_some(), "a policy that drives nothing");
    // one scale per acting joint, and the last action as wide as the action
    assert!(refused(|d| d["action"]["scale"] = serde_json::json!(vec![1.0; 14])).is_some());
    assert!(refused(|d| d["observations"][4]["width"] = 14.into()).is_some());
    assert!(refused(|d| d["limits"] = serde_json::json!({"file": "limits.json"})).is_some(), "limits without their stamp");
}

#[test]
fn a_policy_that_drives_only_the_legs_holds_the_head_at_its_default_pose() {
    // The tiny leg network: action = 0.5 * ELU(position of its own joint relative to the default pose) + 0.1.
    let robot = robot();
    let stand_neck = 20f64.to_radians();
    let legs = |edit: fn(&mut serde_json::Value)| {
        let path = std::path::Path::new(TINY_LEGS).join("deploy.json");
        let mut d: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(path).expect("manifest")).expect("json");
        d["joints"]["default_pos"][5] = 20f64.to_radians().into(); // neck pitch, as in the stand pose
        d["action"]["offset"][5] = d["joints"]["default_pos"][5].clone();
        edit(&mut d);
        Manifest::parse(&d, std::path::Path::new(TINY_LEGS)).expect("a manifest")
    };
    let mut policy = Policy::load(legs(|_| ()), &robot).expect("policy");
    let mut joints = [0.0; 15];
    joints[0] = 0.2; // left hip yaw: the action's first joint
    joints[5] = -0.5; // the neck, pushed away from where the policy holds it
    joints[9] = 0.1; // right hip yaw: the action's sixth joint, the robot's tenth
    joints[14] = 0.4; // the beak
    let targets = policy.act(&level(joints), &[0.0; 3], 0.02).expect("act");
    assert_eq!(targets.len(), 15);
    let scale = 10f64.to_radians();
    assert!((targets[0] - scale * (0.5 * 0.2 + 0.1)).abs() < 1e-6);
    assert!((targets[9] - scale * (0.5 * 0.1 + 0.1)).abs() < 1e-6, "an action lands on its own joint, not the next in line");
    assert!((targets[13] - scale * 0.1).abs() < 1e-6);
    assert!((targets[5] - stand_neck).abs() < 1e-12, "the neck is held at the default pose, whatever it reads");
    for (held, target) in targets.iter().enumerate().take(9).skip(6) {
        assert_eq!(*target, 0.0, "head joint {held} is held at the default pose");
    }
    assert!((targets[14] - 0.4).abs() < 1e-12, "the beak keeps its measured angle");

    // the next observation: fourteen positions and speeds, ten last actions
    let obs = policy.observe(&level(joints), &[0.0; 3], 0.02).expect("observe");
    assert_eq!(obs.len(), 47);
    assert!((f64::from(obs[5]) - (-0.5 - stand_neck)).abs() < 1e-6, "the neck is still observed, relative to its default");
    assert!((f64::from(obs[28]) - 0.2).abs() < 1e-6, "last action of the first acting joint");
    assert!((f64::from(obs[28 + 5]) - 0.15).abs() < 1e-6, "last action of the sixth acting joint, right hip yaw");
    assert!((f64::from(obs[40]) + 1.0).abs() < 1e-6, "gravity z when level, after ten last actions");
    policy.reset();
    assert_eq!(policy.observe(&level(joints), &[0.0; 3], 0.02).expect("observe")[28], 0.0);

    // a network that outputs ten values cannot be described as driving all fourteen joints
    let all = legs(|d| {
        d["joints"].as_object_mut().expect("joints").remove("action_joints");
        d["action"]["scale"] = serde_json::json!(vec![0.1; 14]);
        d["observations"][2]["width"] = 14.into();
    });
    assert!(matches!(Policy::load(all, &robot), Err(PolicyError::Mismatch(_))));
}

#[test]
fn a_command_outside_what_the_policy_was_trained_on_is_refused() {
    let m = Manifest::load(TRAINNR_EXPORT).expect("trainnr's manifest");
    assert!(m.check_command(&[0.2, 0.0, -0.5]).is_ok());
    assert!(m.check_command(&[0.4, -0.3, 1.0]).is_ok(), "the ends of the ranges are inside");
    let faster = m.check_command(&[0.5, 0.0, 0.0]).expect_err("faster than trained");
    assert_eq!(faster.to_string(), "command refused: value 0 is 0.5; the policy was trained on -0.4 to 0.4");
    assert!(matches!(m.check_command(&[0.0, 0.0, f64::NAN]), Err(PolicyError::Command(_))));
    assert!(matches!(m.check_command(&[0.1, 0.0]), Err(PolicyError::Command(_))), "too few values");
}

// ---------------------------------------------------------------- limits over several joints

const LEG_VECTORS: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/leg_rule_vectors.txt");

#[test]
fn the_leg_rules_decide_as_the_firmware_does_on_its_vectors() {
    // Written by the firmware's own implementation (`make -C firmware/tests leg-vectors`): per line the ten leg
    // joints before and wanted in tenths of a degree, the firmware's flags, and which joints got the wanted angle.
    let robot = robot();
    let rules = robot.leg_limits.rules().expect("leg rules");
    let text = std::fs::read_to_string(LEG_VECTORS).expect("leg_rule_vectors.txt");
    let mut cases = 0;
    let mut held = 0;
    for (number, line) in text.lines().enumerate().filter(|(_, l)| !l.starts_with('#')) {
        let words: Vec<i32> = line.split_whitespace().map(|w| w.parse().expect("an integer")).collect();
        assert_eq!(words.len(), 22, "line {}", number + 1);
        let leg = |at: usize| -> [f32; 5] { std::array::from_fn(|i| words[at + i] as f32 / 10.0) };
        let previous = (leg(0), leg(5));
        let wanted = (leg(10), leg(15));
        let (mut left, mut right) = wanted;
        let settled = rules.settle((&previous.0, &previous.1), (&mut left, &mut right));
        let mut mask = 0;
        for (i, (got, wanted)) in left.iter().chain(&right).zip(wanted.0.iter().chain(&wanted.1)).enumerate() {
            if got == wanted {
                mask |= 1 << i;
            }
        }
        assert_eq!(
            (i32::from(settled.firmware_flags()), mask),
            (words[20], words[21]),
            "line {}: previous {previous:?}, wanted {wanted:?}",
            number + 1
        );
        // Whatever was decided, a held joint is where it was before.
        for (got, (wanted, previous)) in
            left.iter().chain(&right).zip(wanted.0.iter().chain(&wanted.1).zip(previous.0.iter().chain(&previous.1)))
        {
            assert!(got == wanted || got == previous);
        }
        cases += 1;
        held += usize::from(!settled.unchanged());
    }
    assert_eq!(cases, 4000);
    assert!(held > 1000, "the vectors must exercise the rules, {held} did");
}

#[test]
fn the_stand_pose_is_inside_the_leg_rules_and_the_sit_pose_is_not() {
    // SIT is reached only along the firmware's checked sit / stand path, which is why it may lie outside the map.
    let text = std::fs::read_to_string(ROBOT_JSON).expect("robot.json");
    let file: serde_json::Value = serde_json::from_str(&text).expect("json");
    let robot = robot();
    let rules = robot.leg_limits.rules().expect("leg rules");
    let leg = |pose: &str, side: &str| -> [f32; 5] {
        ["hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle"]
            .map(|joint| file["poses"][pose][format!("{side}_{joint}")].as_f64().expect("a pose angle") as f32)
    };
    assert!(rules.allowed(&leg("STAND", "left"), &leg("STAND", "right")));
    assert!(!rules.leg_allowed(duck_limits::Side::Left, &leg("SIT", "left")));
    assert!(!rules.leg_allowed(duck_limits::Side::Right, &leg("SIT", "right")));
}

#[test]
fn targets_the_leg_rules_refuse_go_back_to_the_last_pose_and_the_rest_are_untouched() {
    let robot = robot();
    let rules = robot.leg_limits.rules().expect("leg rules");
    let previous = vec![0.0f64; 15];
    // Left knee to its 68 degree limit with the ankle dropping to -20: the knee x ankle rule holds the ankle.
    // The head and the right leg carry values that must come through bit for bit.
    let mut targets = vec![0.0f64; 15];
    targets[3] = 68f64.to_radians();
    targets[4] = (-20f64).to_radians();
    targets[7] = 0.123_456_789_012_345;
    targets[11] = 10f64.to_radians();
    let wanted = targets.clone();
    let settled = duck_runtime::limits::settle_targets(&rules, &previous, &mut targets).expect("fifteen joints");
    assert_eq!(settled.left, duck_limits::LegVerdict::AnkleHeld);
    assert!(settled.knee_ankle && !settled.both_legs);
    assert_eq!(targets[4], 0.0);
    for i in (0..15).filter(|&i| i != 4) {
        assert_eq!(targets[i].to_bits(), wanted[i].to_bits(), "joint {i}");
    }
    // Both legs forward and toed out: both legs stay, the head still moves.
    let mut targets = vec![0.0f64; 15];
    for (first, sign) in [(0, 1.0), (9, -1.0)] {
        targets[first] = sign * 10f64.to_radians();
        targets[first + 2] = sign * (-45f64).to_radians();
    }
    targets[5] = 0.2;
    let settled = duck_runtime::limits::settle_targets(&rules, &previous, &mut targets).expect("fifteen joints");
    assert!(settled.both_legs);
    assert!(targets[..5].iter().chain(&targets[9..14]).all(|&t| t == 0.0));
    assert_eq!(targets[5], 0.2);
    // Too few joints: nothing is decided.
    assert!(duck_runtime::limits::settle_targets(&rules, &previous[..10], &mut targets).is_none());
}
