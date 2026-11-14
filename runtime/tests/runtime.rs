// SPDX-License-Identifier: Apache-2.0
//! Host tests: no board needed. The router is a thread on the other end of a socket pair.
#![allow(clippy::expect_used, clippy::indexing_slicing)] // tests may panic: that is how they fail

use std::os::unix::net::UnixStream;
use std::thread;
use std::time::Duration;

use arm::Verdict;
use duck_runtime::bridge::{Bridge, BridgeError};
use duck_runtime::control::{self, ChainOutcome, Exchange, Guards};
use duck_runtime::firmware;
use duck_runtime::robot::{Robot, JOINT_NAMES};
use rmpv::Value;

const ROBOT_JSON: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../firmware/config/robot.json");

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

fn ints(v: &[i64]) -> Value {
    Value::Array(v.iter().map(|&x| Value::from(x)).collect())
}

/// The firmware as it answers on a bare board: bench-safe build, no servos, no IMU.
fn bare_board(method: &str) -> Option<Value> {
    match method {
        "duck_info" => Some(ints(&[1, 0, 0, 0, 0, 1])),
        "duck_status" => Some(ints(&[0, 0, 0, -1, 1024, 0, 0, 0, 0, 2])),
        "duck_joints" => Some(ints(&[-32768; 15])),
        "duck_state" => Some(ints(&[&[0, 0, 0, 0, -1, 1024, 0, 0, 0, 2][..], &[-32768; 15][..]].concat())),
        // not armed (result 7): the targets are ignored, the state still comes back
        "duck_step" => Some(ints(&[&[7, 0, 0, 0, -1, 1024, 0, 0, 0, 2][..], &[-32768; 15][..]].concat())),
        _ => None,
    }
}

/// Every servo answering, each joint at 0.5 degrees.
fn standing_robot(method: &str) -> Option<Value> {
    match method {
        "duck_joints" => Some(ints(&[5; 15])),
        "duck_state" => Some(ints(&[&[0, 0, 0, 0, -1, 0, 0, 0, 0, 2][..], &[5; 15][..]].concat())),
        "duck_step" => Some(ints(&[&[7, 0, 0, 0, -1, 0, 0, 0, 0, 2][..], &[5; 15][..]].concat())),
        other => bare_board(other),
    }
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
    let robot = Robot::load(ROBOT_JSON).expect("robot.json");
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
    let robot = Robot::load(ROBOT_JSON).expect("robot.json");
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
    let robot = Robot::load(ROBOT_JSON).expect("robot.json");
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
    let robot = Robot::load(ROBOT_JSON).expect("robot.json");
    let mut bridge = fake_router(standing_robot);
    let stats = control::run(&mut bridge, &robot, Duration::from_millis(10), Duration::from_millis(300), Exchange::TwoReads);
    assert!(stats.ticks >= 25 && stats.ticks <= 32, "about 30 ticks in 300 ms at 100 Hz, got {}", stats.ticks);
    assert_eq!(stats.errors, 0);
    assert_eq!(stats.unmeasured_chain_ticks, 0);
    assert_eq!(stats.moves, stats.ticks * 4, "every chain authorised on every tick");
    assert!(stats.percentile(0.5).is_some());

    let mut bare = fake_router(bare_board);
    let stats = control::run(&mut bare, &robot, Duration::from_millis(10), Duration::from_millis(100), Exchange::State);
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

    let robot = Robot::load(ROBOT_JSON).expect("robot.json");
    let stats = control::run(&mut bridge, &robot, Duration::from_millis(10), Duration::from_millis(200), Exchange::Step);
    assert_eq!(stats.errors, 0);
    assert_eq!(stats.moves, stats.ticks * 4);
}
