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
use duck_runtime::policy::{projected_gravity, Manifest, Policy, PolicyError, Sensed};
use duck_runtime::robot::{Robot, JOINT_NAMES};
use duck_runtime::status_led::{Colour, StatusLed};
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
        "duck_info" => Some(ints(&[3, 0, 0, 0, 0, 1])),
        "duck_status" => Some(ints(&[0, 0, 0, -1, 1024, 0, 0, 0, 0, 2])),
        "duck_joints" => Some(ints(&[-32768; 15])),
        "duck_say" => Some(Value::from(5)),
        "duck_state" => Some(ints(&[&[0, 0, 0, 0, -1, 1024, 0, 0, 0, 2][..], &[-32768; 15][..], &[-32768; 3][..]].concat())),
        // not armed (result 7): the targets are ignored, the state still comes back
        "duck_step" => Some(ints(&[&[7, 0, 0, 0, -1, 1024, 0, 0, 0, 2][..], &[-32768; 15][..], &[-32768; 3][..]].concat())),
        _ => None,
    }
}

/// Every servo answering, each joint at 0.5 degrees; IMU ok, level, turning left at 12.5 deg/s and rolling at -3.
fn standing_robot(method: &str) -> Option<Value> {
    match method {
        "duck_joints" => Some(ints(&[5; 15])),
        "duck_state" => Some(ints(&[&[0, 0, 0, 0, -1, 0, 1, 0, 0, 2][..], &[5; 15][..], &[-30, 0, 125][..]].concat())),
        "duck_step" => Some(ints(&[&[7, 0, 0, 0, -1, 0, 1, 0, 0, 2][..], &[5; 15][..], &[-30, 0, 125][..]].concat())),
        other => bare_board(other),
    }
}

/// As `standing_robot`, but the gyro is not reporting (stale, or its mounting is not calibrated).
fn standing_robot_without_gyro(method: &str) -> Option<Value> {
    match method {
        "duck_state" => Some(ints(&[&[0, 0, 0, 0, -1, 0, 1, 0, 0, 2][..], &[5; 15][..], &[-32768; 3][..]].concat())),
        "duck_step" => Some(ints(&[&[7, 0, 0, 0, -1, 0, 1, 0, 0, 2][..], &[5; 15][..], &[-32768; 3][..]].concat())),
        other => standing_robot(other),
    }
}

/// The firmware before the gyro: a 25-word state message.
fn protocol_2_firmware(method: &str) -> Option<Value> {
    match method {
        "duck_state" => Some(ints(&[&[0, 0, 0, 0, -1, 0, 1, 0, 0, 2][..], &[5; 15][..]].concat())),
        other => standing_robot(other),
    }
}

#[test]
fn the_gyro_arrives_in_radians_per_second_or_not_at_all() {
    let state = firmware::state(&mut fake_router(standing_robot)).expect("state");
    let w = state.angular_velocity.expect("gyro");
    let expect = [-3.0f64.to_radians(), 0.0, 12.5f64.to_radians()];
    assert!(w.iter().zip(expect).all(|(a, b)| (a - b).abs() < 1e-12), "{w:?}");
    assert_eq!(state.joints.len(), 15);
    assert!(state.joints.iter().all(|j| j.is_some_and(|r| (r - 0.5f64.to_radians()).abs() < 1e-12)));

    let state = firmware::state(&mut fake_router(standing_robot_without_gyro)).expect("state");
    assert_eq!(state.angular_velocity, None, "a missing gyro is not a zero");

    let old = firmware::state(&mut fake_router(protocol_2_firmware));
    assert!(matches!(old, Err(BridgeError::Shape(why)) if why.contains("protocol 2")));
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
    let stats = control::run(&mut bridge, &robot, Duration::from_millis(10), Duration::from_millis(300), Exchange::TwoReads, None, &mut StatusLed::none());
    assert!(stats.ticks >= 25 && stats.ticks <= 32, "about 30 ticks in 300 ms at 100 Hz, got {}", stats.ticks);
    assert_eq!(stats.errors, 0);
    assert_eq!(stats.unmeasured_chain_ticks, 0);
    assert_eq!(stats.moves, stats.ticks * 4, "every chain authorised on every tick");
    assert!(stats.percentile(0.5).is_some());

    let mut bare = fake_router(bare_board);
    let stats = control::run(&mut bare, &robot, Duration::from_millis(10), Duration::from_millis(100), Exchange::State, None, &mut StatusLed::none());
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
    let stats = control::run(&mut bridge, &robot, Duration::from_millis(10), Duration::from_millis(200), Exchange::Step, None, &mut StatusLed::none());
    assert_eq!(stats.errors, 0);
    assert_eq!(stats.moves, stats.ticks * 4);
}

// ---------------------------------------------------------------- policy

const TINY_ONNX: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/tiny_policy.onnx");
const TINY_JSON: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/tiny_policy.json");

/// The tiny policy's manifest, optionally edited as text first.
fn manifest(edit: fn(String) -> String) -> Manifest {
    let text = edit(std::fs::read_to_string(TINY_JSON).expect("manifest"));
    serde_json::from_str(&text).expect("manifest json")
}

/// The same network, described without a gyro: its three slots become part of the command.
fn no_gyro(text: String) -> String {
    text.replace(r#"{"term": "angular_velocity"},"#, "").replace(r#""size": 13"#, r#""size": 16"#)
}

fn level(joints: [f64; 15]) -> Sensed {
    Sensed { joints: joints.iter().map(|&j| Some(j)).collect(), pitch_roll: Some((0.0, 0.0)), angular_velocity: Some([0.0; 3]) }
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
    let robot = Robot::load(ROBOT_JSON).expect("robot.json");
    assert!(Policy::load(TINY_ONNX, manifest(|t| t), &robot).is_ok());
    // joints in another order
    let swapped = manifest(|t| t.replacen("left_hip_yaw", "X", 1).replacen("left_hip_roll", "left_hip_yaw", 1).replacen("X", "left_hip_roll", 1));
    assert!(matches!(Policy::load(TINY_ONNX, swapped, &robot), Err(PolicyError::Mismatch(_))));
    // an observation layout the network was not built for (60 values, the model takes 61)
    let short = manifest(|t| t.replace(r#""size": 13"#, r#""size": 12"#));
    assert!(matches!(Policy::load(TINY_ONNX, short, &robot), Err(PolicyError::Mismatch(_))));
    // a default pose the robot cannot hold: left hip yaw at -30 degrees, its limit is -6.3
    let outside = manifest(|t| t.replace("[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]", "[-30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]"));
    assert!(matches!(Policy::load(TINY_ONNX, outside, &robot), Err(PolicyError::Mismatch(_))));
    // a model file that is not there
    assert!(matches!(Policy::load("/nonexistent.onnx", manifest(|t| t), &robot), Err(PolicyError::Load(_))));
}

#[test]
fn the_policy_turns_an_observation_into_targets_we_can_compute_by_hand() {
    // The tiny network: action = 0.5 * ELU(joint position relative to the default pose) + 0.1, per joint.
    let robot = Robot::load(ROBOT_JSON).expect("robot.json");
    let mut policy = Policy::load(TINY_ONNX, manifest(|t| t), &robot).expect("policy");
    let mut joints = [0.0; 15];
    joints[0] = 0.2;
    joints[1] = -0.3;
    joints[14] = 0.4; // the beak: not the policy's to move
    let targets = policy.act(&level(joints), &[0.0; 13], 0.02).expect("act");
    assert_eq!(targets.len(), 15);
    let scale = 10f64.to_radians();
    let expect = |action: f64| action * scale;
    assert!((targets[0] - expect(0.5 * 0.2 + 0.1)).abs() < 1e-6);
    assert!((targets[1] - expect(0.5 * ((-0.3f64).exp() - 1.0) + 0.1)).abs() < 1e-6);
    assert!((targets[2] - expect(0.1)).abs() < 1e-6);
    assert!((targets[14] - 0.4).abs() < 1e-12, "the beak keeps its measured angle");

    // the second tick sees joint speeds and the previous action
    joints[0] = 0.21;
    let obs = policy.observe(&level(joints), &[0.0; 13], 0.02).expect("observe");
    assert_eq!(obs.len(), 61);
    assert!((f64::from(obs[14]) - 0.5).abs() < 1e-4, "speed of joint 0: 0.01 rad in 0.02 s");
    assert!((f64::from(obs[28]) - 0.2).abs() < 1e-6, "last action of joint 0");
    assert!((f64::from(obs[44]) + 1.0).abs() < 1e-6, "gravity z when level");

    // after a reset there is no previous tick to difference against
    policy.reset();
    let obs = policy.observe(&level(joints), &[0.0; 13], 0.02).expect("observe");
    assert_eq!(obs[14], 0.0);
    assert_eq!(obs[28], 0.0);
}

#[test]
fn the_action_is_clipped_before_it_is_scaled() {
    let robot = Robot::load(ROBOT_JSON).expect("robot.json");
    let mut policy = Policy::load(TINY_ONNX, manifest(|t| t.replace(r#""action_clip": 1.0"#, r#""action_clip": 0.15"#)), &robot).expect("policy");
    let mut joints = [0.0; 15];
    joints[0] = 0.2; // raw action 0.2
    let targets = policy.act(&level(joints), &[0.0; 13], 0.02).expect("act");
    assert!((targets[0] - 0.15 * 10f64.to_radians()).abs() < 1e-9);
}

#[test]
fn a_missing_measurement_stops_the_policy_instead_of_becoming_a_zero() {
    let robot = Robot::load(ROBOT_JSON).expect("robot.json");
    let mut policy = Policy::load(TINY_ONNX, manifest(|t| t), &robot).expect("policy");
    let command = [0.0; 13];
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
    assert!(matches!(policy.act(&level([0.0; 15]), &[0.0; 5], 0.02), Err(PolicyError::Mismatch(_))));
}

#[test]
fn the_loop_runs_a_policy_and_holds_when_the_policy_cannot_run() {
    let robot = Robot::load(ROBOT_JSON).expect("robot.json");
    let period = Duration::from_millis(20);

    // A policy that needs no gyro runs on every tick, and its targets pass the guards.
    let mut policy = Policy::load(TINY_ONNX, manifest(no_gyro), &robot).expect("policy");
    let mut bridge = fake_router(standing_robot);
    let command = [0.0; 16];
    let stats = control::run(&mut bridge, &robot, period, Duration::from_millis(300), Exchange::Step, Some((&mut policy, &command)), &mut StatusLed::none());
    assert_eq!(stats.errors, 0);
    assert_eq!(stats.policy_ran, stats.ticks);
    assert_eq!(stats.policy_skipped, 0);
    assert_eq!(stats.moves, stats.ticks * 4);
    assert!(stats.inference_percentile(0.5).is_some());

    // A policy that needs the gyro runs when the firmware reports it ...
    let mut needs_gyro = Policy::load(TINY_ONNX, manifest(|t| t), &robot).expect("policy");
    let mut bridge = fake_router(standing_robot);
    let command = [0.0; 13];
    let stats = control::run(&mut bridge, &robot, period, Duration::from_millis(200), Exchange::Step, Some((&mut needs_gyro, &command)), &mut StatusLed::none());
    assert_eq!(stats.errors, 0);
    assert_eq!(stats.policy_ran, stats.ticks);

    // ... and is skipped, holding in place, on a tick when it does not.
    let mut bridge = fake_router(standing_robot_without_gyro);
    let stats = control::run(&mut bridge, &robot, period, Duration::from_millis(200), Exchange::State, Some((&mut needs_gyro, &command)), &mut StatusLed::none());
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
    let robot = Robot::load(ROBOT_JSON).expect("robot.json");
    let leds = fake_leds("loop");
    {
        let mut led = StatusLed::find_in(&leds);
        assert!(led.present());
        // holding in place, no policy: green
        control::run(&mut fake_router(standing_robot), &robot, Duration::from_millis(10), Duration::from_millis(50), Exchange::State, None, &mut led);
        assert_eq!(lit(&leds), [false, true, false]);
        // a policy that needs the gyro, which is not reporting: yellow
        let mut needs_gyro = Policy::load(TINY_ONNX, manifest(|t| t), &robot).expect("policy");
        let command = [0.0; 13];
        control::run(&mut fake_router(standing_robot_without_gyro), &robot, Duration::from_millis(10), Duration::from_millis(50), Exchange::State, Some((&mut needs_gyro, &command)), &mut led);
        assert_eq!(lit(&leds), [true, true, false]);
        // the policy running: blue
        control::run(&mut fake_router(standing_robot), &robot, Duration::from_millis(10), Duration::from_millis(50), Exchange::State, Some((&mut needs_gyro, &command)), &mut led);
        assert_eq!(lit(&leds), [false, false, true]);
        // the firmware not answering: red
        control::run(&mut fake_router(|_| None), &robot, Duration::from_millis(10), Duration::from_millis(30), Exchange::State, None, &mut led);
        assert_eq!(lit(&leds), [true, false, false]);
        led.show(Colour::White);
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
