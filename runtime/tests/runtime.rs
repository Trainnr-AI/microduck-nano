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
use duck_runtime::control::{self, ChainOutcome, Exchange, Guards, Sinks};
use duck_runtime::firmware;
use duck_runtime::firmware::Sensed;
use duck_runtime::policy::{projected_gravity, Manifest, Policy, PolicyError, Term};
use duck_runtime::record::{orientation, Recorder};
use duck_runtime::robot::{Robot, JOINT_NAMES};
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

fn ints(v: &[i64]) -> Value {
    Value::Array(v.iter().map(|&x| Value::from(x)).collect())
}

/// A protocol 3 state reply: result, mode 0, IMU ok or not, level, every joint at `joint` tenths of a degree, the gyro.
fn state(result: i64, imu_ok: i64, joint: i64, gyro: [i64; 3]) -> Option<Value> {
    Some(ints(&[&[result, 0, 0, 0, -1, if imu_ok == 1 { 0 } else { 1024 }, imu_ok, 0, 0, 2][..], &[joint; 15][..], &gyro[..]].concat()))
}

/// The firmware as it answers on a bare board: bench-safe build, no servos, no IMU.
fn bare_board(method: &str) -> Option<Value> {
    match method {
        "duck_info" => Some(ints(&[3, 0, 0, 0, 0, 1])),
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

/// The same network, described without a gyro: its three slots become part of the command.
fn no_gyro(d: &mut serde_json::Value) {
    let obs = d["observations"].as_array_mut().expect("observations");
    obs.retain(|o| o["name"] != "base_ang_vel");
    if let Some(command) = obs.iter_mut().find(|o| o["name"] == "command") {
        command["width"] = 16.into();
    }
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
    let robot = robot();
    assert!(Policy::load(manifest(|_| ()), &robot).is_ok());
    // joints in another order
    let swapped = manifest(|d| d["joints"]["policy_order"].as_array_mut().expect("joints").swap(0, 1));
    assert!(matches!(Policy::load(swapped, &robot), Err(PolicyError::Mismatch(_))));
    // an observation layout the network was not built for (60 values, the model takes 61)
    let short = manifest(|d| d["observations"][5]["width"] = 12.into());
    assert!(matches!(Policy::load(short, &robot), Err(PolicyError::Mismatch(_))));
    // a default pose the robot cannot hold: left hip yaw at -30 degrees, its limit is -6.3
    let outside = manifest(|d| d["joints"]["default_pos"][0] = (-30f64).to_radians().into());
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
    let robot = robot();
    let mut policy = Policy::load(manifest(|d| d["action"]["clip"] = 0.15.into()), &robot).expect("policy");
    let mut joints = [0.0; 15];
    joints[0] = 0.2; // raw action 0.2
    let targets = policy.act(&level(joints), &[0.0; 13], 0.02).expect("act");
    assert!((targets[0] - 0.15 * 10f64.to_radians()).abs() < 1e-9);
}

#[test]
fn a_missing_measurement_stops_the_policy_instead_of_becoming_a_zero() {
    let robot = robot();
    let mut policy = Policy::load(manifest(|_| ()), &robot).expect("policy");
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
    assert!(matches!(policy.act(&level([0.0; 15]), &[0.0; 5], 0.02), Err(PolicyError::Command(_))));
}

#[test]
fn the_loop_runs_a_policy_and_holds_when_the_policy_cannot_run() {
    let robot = robot();
    let period = Duration::from_millis(20);

    // A policy that needs no gyro runs on every tick, and its targets pass the guards.
    let mut policy = Policy::load(manifest(no_gyro), &robot).expect("policy");
    let mut bridge = fake_router(standing_robot);
    let command = [0.0; 16];
    let stats = control::run(&mut bridge, &robot, period, Duration::from_millis(300), Exchange::Step, Some((&mut policy, &command)), &mut Sinks::none());
    assert_eq!(stats.errors, 0);
    assert_eq!(stats.policy_ran, stats.ticks);
    assert_eq!(stats.policy_skipped, 0);
    assert_eq!(stats.moves, stats.ticks * 4);
    assert!(stats.inference_percentile(0.5).is_some());

    // A policy that needs the gyro runs when the firmware reports it ...
    let mut needs_gyro = Policy::load(manifest(|_| ()), &robot).expect("policy");
    let mut bridge = fake_router(standing_robot);
    let command = [0.0; 13];
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
        let command = [0.0; 13];
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
    let tail = &data[data.len() - 8 * 15 - 8..];
    let positions: Vec<f64> = tail[..8 * 15].chunks(8).map(|b| f64::from_le_bytes(b.try_into().expect("8 bytes"))).collect();
    assert!(positions.iter().all(|p| (p - 0.5f64.to_radians()).abs() < 1e-12), "{positions:?}");
    assert_eq!(&tail[8 * 15..], &[0u8; 8], "velocity and effort are empty");
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

#[test]
fn a_trainnr_export_is_read_as_trainnr_wrote_it() {
    // deploy.json exactly as trainnr's export_deployment wrote it (a 50-iteration pipeline check on Pollen's microduck,
    // whose joints are named and ordered as ours; the network itself is not kept here).
    let m = Manifest::load(TRAINNR_EXPORT).expect("trainnr's manifest");
    assert_eq!(m.control_hz, 50.0);
    assert_eq!(m.joints, JOINT_NAMES[..14].iter().map(|s| s.to_string()).collect::<Vec<_>>());
    assert_eq!(m.observation_size(), 51);
    assert_eq!(m.command_size(), 3);
    let terms: Vec<_> = m.observation.iter().map(|o| o.term).collect();
    assert_eq!(
        terms,
        [Term::AngularVelocity, Term::ProjectedGravity, Term::JointPositions, Term::JointVelocities, Term::LastAction, Term::Command { size: 3 }]
    );
    assert_eq!(m.action_clip, None);
    assert_eq!(m.command_ranges, Some([(-0.4, 0.4), (-0.3, 0.3), (-1.0, 1.0)]));
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
}

#[test]
fn observation_terms_are_clipped_then_scaled_as_in_training() {
    let robot = robot();
    let mut policy = Policy::load(
        manifest(|d| {
            d["observations"][0]["clip"] = 0.1.into();
            d["observations"][0]["scale"] = 2.0.into();
        }),
        &robot,
    )
    .expect("policy");
    let mut joints = [0.0; 15];
    joints[0] = 0.3; // clipped to 0.1, then doubled
    joints[1] = 0.05; // inside the clip: doubled
    let obs = policy.observe(&level(joints), &[0.0; 13], 0.02).expect("observe");
    assert!((f64::from(obs[0]) - 0.2).abs() < 1e-6 && (f64::from(obs[1]) - 0.1).abs() < 1e-6);
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
    // without stated ranges only the count and finiteness are checked
    assert!(manifest(|_| ()).check_command(&[9.0; 13]).is_ok());
}
