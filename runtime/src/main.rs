// SPDX-License-Identifier: Apache-2.0
//! `duckd`: the runtime's command-line entry point. Run it on the board.

use std::error::Error;
use std::os::unix::net::UnixStream;
use std::process::ExitCode;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use duck_runtime::bench::{self, Excitation};
use duck_runtime::bridge::{Bridge, ROUTER_SOCKET};
use duck_runtime::control::{self, Exchange, Sinks, STOP};
use duck_runtime::firmware::{self, Sensed};
use duck_runtime::policy::{Manifest, Policy, POLICY_JOINTS};
use duck_runtime::record::Recorder;
use duck_runtime::robot::{Robot, JOINT_NAMES};
use duck_runtime::status_led::StatusLed;

const USAGE: &str = "\
duckd: Microduck Nano runtime (nothing here arms the robot; only `bench`, against a bench build, moves one servo)

usage:
  duckd status [--socket PATH]
      Ask the firmware what it is and what state it is in.
  duckd say TEXT... [--socket PATH]
      Put TEXT on the board's LED matrix for a few seconds (up to 32 characters; the words are joined).
  duckd time [--count 200] [--socket PATH]
      Time single read-only calls to the microcontroller, by reply size, to see what the link costs.
  duckd policy-time --robot robot.json --policy DEPLOYMENT [--count 2000]
      Load a policy (a trainnr deployment: its folder or deploy.json), check it against the robot, and time
      inference alone on made-up observations. Needs no firmware.
  duckd run --robot robot.json [--hz 50] [--seconds 10] [--exchange state|step|two-reads] [--policy DEPLOYMENT]
            [--command FORWARD,SIDEWAYS,TURN] [--record FILE.mcap] [--socket PATH]
      Run the control loop (nothing here arms the robot): read the state every tick, pass a target through the rigrs
      guards (the policy's, or hold-in-place), and report round-trip time to the microcontroller and missed deadlines.
        state      one read per tick (default; sends nothing)
        step       one exchange per tick: the guarded hold-in-place targets go out, the state comes back.
                   The firmware ignores targets unless it is armed.
        two-reads  status and joints as two calls (firmware protocol 1; no gyro)
      With --policy the targets come from the policy instead of hold-in-place; the loop runs at the policy's rate.
      A tick on which an input the policy needs was not measured holds instead. --command is what the policy is
      asked to do (for a walk: m/s forward, m/s to the left, rad/s turning left), zero if not given; it is refused
      outside the ranges the policy was trained on.
      The board's LED1 shows each tick: green = holding in place, blue = the policy drove the targets,
      yellow = the policy could not run (an input was missing), red = the exchange failed; off when duckd exits.
      With --record, every tick's measured joints, sent targets and IMU go into an MCAP file (ROS 2 messages) that
      trainnr's ingest_recording reads.
  duckd bench --joint NAME --supervised [--excite chirp|steps] [--amplitude 10] [--seconds 20] [--hz 100]
              [--record FILE.mcap] [--socket PATH]
      MOVES ONE SERVO, for measuring it. Needs a bench build of the firmware (DUCK_BENCH_MOTION 1), the joint's
      entry calibrated in robot.json, the servo free to turn, and someone watching with the power at hand; say so
      with --supervised. Arms that joint alone, moves it around where it is (+- amplitude degrees, faded in and
      out; chirp 0.1 to 2 Hz, or steps every second), then switches it off. The firmware keeps it inside the joint's
      limits and speed limit and switches it off on any fault, or when duckd stops talking. --record writes the
      targets asked for, the goals the firmware wrote after its clamp and speed limit, and the servo's reads
      (position and its own speed report) stamped when it was read; at the default 100 Hz, every read.
";

type Res<T = ()> = Result<T, Box<dyn Error>>;

/// How long one call to the firmware may take outside the control loop.
const CALL_TIMEOUT: Duration = Duration::from_millis(500);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Command {
    Status,
    Say,
    Time,
    PolicyTime,
    Run,
    Bench,
}

struct Args {
    command: Command,
    socket: String,
    robot: Option<String>,
    /// Loop rate; `run` defaults to 50 Hz (or the policy's), `bench` to the firmware's 100 Hz tick.
    hz: Option<f64>,
    seconds: f64,
    count: u32,
    exchange: Exchange,
    policy: Option<String>,
    record: Option<String>,
    joint: Option<String>,
    excite: (&'static str, Excitation),
    amplitude_deg: f64,
    supervised: bool,
    /// `duckd say`'s text.
    text: Option<String>,
    /// `duckd run --command`: what the policy is asked to do.
    walk: Option<Vec<f64>>,
}

const CHIRP: (&str, Excitation) = ("chirp", Excitation::Chirp { from_hz: 0.1, to_hz: 2.0 });

fn number<T: std::str::FromStr>(flag: &str, value: String) -> Result<T, String> {
    value.parse().map_err(|_| format!("{flag} needs a number, not {value:?}"))
}

fn parse(mut argv: impl Iterator<Item = String>) -> Result<Args, String> {
    let command = match argv.next().ok_or("missing command")?.as_str() {
        "status" => Command::Status,
        "say" => Command::Say,
        "time" => Command::Time,
        "policy-time" => Command::PolicyTime,
        "run" => Command::Run,
        "bench" => Command::Bench,
        other => return Err(format!("unknown command {other}")),
    };
    let mut args = Args { command, socket: ROUTER_SOCKET.into(), robot: None, hz: None, seconds: 10.0, count: 200,
        exchange: Exchange::State, policy: None, record: None, joint: None, excite: CHIRP, amplitude_deg: 10.0,
        supervised: false, text: None, walk: None };
    while let Some(flag) = argv.next() {
        if flag == "--supervised" {
            args.supervised = true;
            continue;
        }
        if command == Command::Say && !flag.starts_with("--") {
            let text = args.text.get_or_insert_with(String::new);
            if !text.is_empty() {
                text.push(' ');
            }
            text.push_str(&flag);
            continue;
        }
        let value = argv.next().ok_or(format!("{flag} needs a value"))?;
        match flag.as_str() {
            "--socket" => args.socket = value,
            "--robot" => args.robot = Some(value),
            "--policy" => args.policy = Some(value),
            "--record" => args.record = Some(value),
            "--joint" => args.joint = Some(value),
            "--hz" => args.hz = Some(number(&flag, value)?),
            "--seconds" => args.seconds = number(&flag, value)?,
            "--count" => args.count = number(&flag, value)?,
            "--amplitude" => args.amplitude_deg = number(&flag, value)?,
            "--command" => {
                let values = value.split(',').map(|v| number(&flag, v.trim().to_string())).collect::<Result<_, _>>()?;
                args.walk = Some(values);
            }
            "--exchange" => {
                args.exchange = match value.as_str() {
                    "state" => Exchange::State,
                    "step" => Exchange::Step,
                    "two-reads" => Exchange::TwoReads,
                    _ => return Err("--exchange must be state, step or two-reads".into()),
                }
            }
            "--excite" => {
                args.excite = match value.as_str() {
                    "chirp" => CHIRP,
                    "steps" => ("steps", Excitation::Steps { period_s: 2.0 }),
                    _ => return Err("--excite must be chirp or steps".into()),
                }
            }
            other => return Err(format!("unknown option {other}")),
        }
    }
    if args.hz.is_some_and(|hz| !(hz.is_finite() && hz > 0.0 && hz <= 1000.0)) {
        return Err("--hz must be between 0 and 1000".into());
    }
    if !(args.seconds.is_finite() && args.seconds > 0.0) {
        return Err("--seconds must be positive".into());
    }
    if !(args.amplitude_deg > 0.0 && args.amplitude_deg <= 30.0) {
        return Err("--amplitude must be more than 0 and at most 30 degrees".into());
    }
    Ok(args)
}

fn main() -> ExitCode {
    let args = match parse(std::env::args().skip(1)) {
        Ok(args) => args,
        Err(why) => {
            eprintln!("{why}\n\n{USAGE}");
            return ExitCode::from(2);
        }
    };
    let outcome = match args.command {
        Command::PolicyTime => policy_time(&args),
        Command::Status => with_bridge(&args, status),
        Command::Say => with_bridge(&args, |b| say(&args, b)),
        Command::Time => with_bridge(&args, |b| time(&args, b)),
        Command::Run => with_bridge(&args, |b| run(&args, b)),
        Command::Bench => with_bridge(&args, |b| bench_command(&args, b)),
    };
    match outcome {
        Ok(()) => ExitCode::SUCCESS,
        Err(why) => {
            eprintln!("duckd: {why}");
            ExitCode::FAILURE
        }
    }
}

/// Connect to the firmware through the board's router and run `command` with it.
fn with_bridge(args: &Args, command: impl FnOnce(&mut Bridge<UnixStream>) -> Res) -> Res {
    command(&mut Bridge::connect(&args.socket, CALL_TIMEOUT)?)
}

/// "median / 95th / max" of sorted microsecond times, in milliseconds with `decimals` places ("-" when empty).
fn summary(sorted_us: &[u64], decimals: usize) -> String {
    let ms = |p| control::percentile(sorted_us, p).map_or("-".into(), |us| format!("{:.decimals$}", us as f64 / 1000.0));
    format!("median {} ms   95th {} ms   max {} ms", ms(0.5), ms(0.95), ms(1.0))
}

/// Ctrl-C or SIGTERM sets `control::STOP`, which ends a loop after its tick. With `second_exits`, a second one
/// exits at once; without, it cannot skip what follows the loop (the bench's disarm).
fn stop_on_signal(second_exits: bool) -> Result<(), ctrlc::Error> {
    ctrlc::set_handler(move || {
        if STOP.swap(true, Ordering::Relaxed) && second_exits {
            std::process::exit(130);
        }
    })
}

fn load_policy(args: &Args, robot: &Robot) -> Res<Option<Policy>> {
    let Some(path) = args.policy.as_deref() else { return Ok(None) };
    Ok(Some(Policy::load(Manifest::load(path)?, robot)?))
}

fn robot(args: &Args) -> Res<Robot> {
    Ok(Robot::load(args.robot.as_deref().ok_or("needs --robot <path to firmware/config/robot.json>")?)?)
}

fn status(bridge: &mut Bridge<UnixStream>) -> Res {
    let info = firmware::info(bridge)?;
    let status = firmware::status(bridge)?;
    println!("firmware protocol {} (this runtime speaks {})", info.protocol, firmware::PROTOCOL);
    println!("  motion build: {}   permanent-memory writes: {}", info.motion_build, info.eeprom_build);
    println!("  joint table ready: {}   IMU present: {}", info.joint_table_ready, info.imu_present);
    println!("  servo port is Serial1: {}", info.servo_uart_is_serial1);
    println!("mode {}   posture {}   fault {}", status.mode, status.posture, status.fault);
    println!("  IMU ok: {}   pitch {:.1} deg   roll {:.1} deg", status.imu_ok, status.pitch_deg, status.roll_deg);
    println!("  flags 0x{:x}   worst firmware tick {} us", status.flags, status.worst_tick_us);
    let joints = firmware::joints(bridge)?;
    println!("servos answering: {}/{}", joints.iter().flatten().count(), joints.len());
    Ok(())
}

fn say(args: &Args, bridge: &mut Bridge<UnixStream>) -> Res {
    let kept = firmware::say(bridge, args.text.as_deref().ok_or("say needs the text")?)?;
    println!("on the matrix: {kept} characters");
    Ok(())
}

/// Time read-only calls with replies of different sizes (values in the reply shown in brackets).
fn time(args: &Args, bridge: &mut Bridge<UnixStream>) -> Res {
    let zeros = || vec![rmpv::Value::Array(vec![rmpv::Value::from(0); JOINT_NAMES.len()])];
    for (method, values, params) in [
        ("duck_info", 7, vec![]),
        ("duck_status", 10, vec![]),
        ("duck_joints", 15, vec![]),
        ("duck_state", 43, vec![]),
        ("duck_step", 43, zeros()),
        ("duck_health", 45, vec![]),
    ] {
        // A failed call is counted, not fatal: how often the link drops a reply is part of what this measures.
        let mut times = Vec::with_capacity(args.count as usize);
        let mut failed = 0;
        for _ in 0..args.count {
            let start = Instant::now();
            match bridge.call(method, params.clone()) {
                Ok(_) => times.push(u64::try_from(start.elapsed().as_micros()).unwrap_or(u64::MAX)),
                Err(_) => failed += 1,
            }
        }
        times.sort_unstable();
        println!("{method:12} [{values:2} values]  {}   ({} calls, {failed} failed)", summary(&times, 2), times.len());
    }
    Ok(())
}

/// Inference alone, on made-up but changing inputs: every joint swinging a little, the trunk tilting a little.
fn policy_time(args: &Args) -> Res {
    let robot = robot(args)?;
    let mut policy = load_policy(args, &robot)?.ok_or("policy-time needs --policy")?;
    let m = policy.manifest();
    println!("policy `{}`: {} observations -> {} actions, trained at {} Hz", m.name, m.observation_size(), m.acting.len(), m.control_hz);
    let (command, period) = (vec![0.0; m.command_size()], 1.0 / m.control_hz);
    let mut times = Vec::with_capacity(args.count as usize);
    for i in 0..args.count {
        let phase = f64::from(i) * 0.1;
        let sensed = Sensed {
            joints: (0..JOINT_NAMES.len()).map(|j| Some(0.05 * (phase + j as f64).sin())).collect(),
            pitch_roll: Some((0.02 * phase.sin(), 0.02 * phase.cos())),
            angular_velocity: Some([0.1 * phase.sin(), 0.1 * phase.cos(), 0.0]),
            joint_speeds: (0..JOINT_NAMES.len()).map(|j| Some(0.25 * (phase + j as f64).cos())).collect(),
        };
        policy.act(&sensed, &command, period)?;
        times.push(policy.last_inference_us);
    }
    times.sort_unstable();
    println!("inference: {}   ({} runs)", summary(&times, 3), times.len());
    Ok(())
}

fn run(args: &Args, bridge: &mut Bridge<UnixStream>) -> Res {
    let robot = robot(args)?;
    let mut policy = load_policy(args, &robot)?;
    let protocol = firmware::info(bridge)?.protocol;
    if args.exchange != Exchange::TwoReads && protocol < firmware::PROTOCOL {
        return Err(format!(
            "the firmware speaks protocol {protocol}; --exchange state and step need {} (the state message with the \
             gyro). Push the app from this repository, or use --exchange two-reads",
            firmware::PROTOCOL
        )
        .into());
    }
    let hz = policy.as_ref().map_or(args.hz.unwrap_or(50.0), |p| p.manifest().control_hz);
    let command = match (&policy, &args.walk) {
        (Some(p), walk) => {
            let command = walk.clone().unwrap_or_else(|| vec![0.0; p.manifest().command_size()]);
            p.manifest().check_command(&command)?;
            command
        }
        (None, Some(_)) => return Err("--command needs --policy".into()),
        (None, None) => Vec::new(),
    };
    let period = Duration::from_secs_f64(1.0 / hz);
    println!(
        "robot: {} joints in {} chains, {} with a servo ID; loop {hz} Hz for {} s",
        robot.joints(),
        robot.chains.len(),
        robot.ids_assigned,
        args.seconds
    );
    if let Some(p) = &policy {
        let m = p.manifest();
        println!("policy `{}`, command {command:?}", m.name);
        let held: Vec<&str> = (0..POLICY_JOINTS).filter(|i| !m.acting.contains(i)).filter_map(|i| JOINT_NAMES.get(i).copied()).collect();
        if !held.is_empty() {
            println!("  it drives {} joints and holds the others at its stand pose: {}", m.acting.len(), held.join(", "));
        }
        if let Some(limits) = &m.limits {
            println!("  trained against the limits over several joints {limits}; here the firmware holds its own");
        }
    }
    stop_on_signal(true)?;
    let led = StatusLed::find();
    println!("status light (LED1): {}", if led.present() { "yes" } else { "not found" });
    let recorder = args.record.as_deref().map(Recorder::create).transpose()?;
    let mut sinks = Sinks { led, recorder, record_error: None };
    // Announce on the matrix; an older firmware without duck_say just does not show it.
    let _ = firmware::say(bridge, if policy.is_some() { "DUCKD POLICY" } else { "DUCKD RUN" });
    let duration = Duration::from_secs_f64(args.seconds);
    // In the loop a reply is waited for one tick at most: a lost one costs that tick, not half a second of them.
    bridge.set_timeout(period)?;
    let stats = control::run(bridge, &robot, period, duration, args.exchange, policy.as_mut().map(|p| (p, command.as_slice())), &mut sinks);
    bridge.set_timeout(CALL_TIMEOUT)?;
    drop(sinks.led); // the light goes out with the loop
    if let (Some(recorder), Some(path)) = (sinks.recorder, &args.record) {
        println!("recorded {} ticks to {path}", recorder.finish()?);
    }
    if let Some(why) = &sinks.record_error {
        println!("{why}; the recording stopped there and is not readable");
    }
    let _ = firmware::say(bridge, "DUCKD DONE");
    println!("ticks {}   errors {}   missed deadlines {}", stats.ticks, stats.errors, stats.missed_deadlines);
    println!("exchange per tick ({:?}): {}   (period {:.2} ms)", args.exchange, summary(&stats.round_trip_us, 2), period.as_secs_f64() * 1000.0);
    println!("guard verdicts: move {}   hold {}   chain unmeasured {}", stats.moves, stats.holds, stats.unmeasured_chain_ticks);
    println!("leg rules (hip maps, knee x ankle, both legs): held a leg joint on {} ticks", stats.leg_rule_ticks);
    if policy.is_some() {
        println!("policy: ran {} ticks, skipped {}   inference {}", stats.policy_ran, stats.policy_skipped, summary(&stats.inference_us, 2));
        if let Some(why) = &stats.last_policy_skip {
            println!("last skip: {why}");
        }
    }
    if let Some(why) = stats.last_error {
        println!("last error: {why}");
    }
    Ok(())
}

/// `duckd bench`: one servo, armed alone, moved and switched off again (whatever happens in between).
fn bench_command(args: &Args, bridge: &mut Bridge<UnixStream>) -> Res {
    let name = args.joint.as_deref().ok_or("bench needs --joint NAME")?;
    let joint = JOINT_NAMES.iter().position(|n| *n == name).ok_or_else(|| format!("no joint `{name}`; the joints are {JOINT_NAMES:?}"))?;
    if !args.supervised {
        return Err("bench moves a servo. Run it only with the servo free to turn and someone watching with the power \
                    at hand, and say so with --supervised"
            .into());
    }
    if !firmware::info(bridge)?.bench_build {
        return Err("the firmware is not a bench build: set DUCK_ENABLE_MOTION 1 and DUCK_BENCH_MOTION 1 in \
                    duck_build_config.h, push the app, and go back to the default build afterwards"
            .into());
    }
    // The file first: a path that cannot be written is found before anything moves.
    let mut recorder = args.record.as_deref().map(Recorder::create).transpose()?;
    // Ctrl-C ends the run after its tick, and then the servo is switched off; a second Ctrl-C does not skip that.
    stop_on_signal(false)?;
    let hz = args.hz.unwrap_or(100.0);
    let (excite, excitation) = args.excite;
    // From the arm request on, every way out goes through the disarm below: even a lost reply may have armed it.
    let outcome = (|| -> Res<bench::BenchRun> {
        firmware::bench_arm(bridge, joint)?.map_err(|why| format!("the firmware refused to arm `{name}`: {why}"))?;
        let start = firmware::bench_step(bridge, None)?.setpoint;
        println!("`{name}` armed at {:.1} deg; {excite} +- {} deg for {} s at {hz} Hz", start.to_degrees(), args.amplitude_deg, args.seconds);
        let (duration, period) = (Duration::from_secs_f64(args.seconds), Duration::from_secs_f64(1.0 / hz));
        Ok(bench::run(bridge, start, args.amplitude_deg.to_radians(), excitation, duration, period))
    })();
    let disarmed = firmware::bench_disarm(bridge);
    let run = outcome?;
    disarmed?;
    println!("switched off. {} ticks", run.samples.len());
    if let Some(why) = &run.stopped {
        println!("ended early: {why}");
    }
    if let Some(r) = run.last.as_ref().and_then(|b| b.reading.as_ref()) {
        println!("last read: {:.1} V, {} C, load {:.1} %", r.voltage, r.temperature_c, r.load_percent);
    }
    if let (Some(mut recorder), Some(path)) = (recorder.take(), &args.record) {
        let reads = recorder.bench(name, &run.samples)?;
        recorder.finish()?;
        println!("recorded {} targets and {reads} servo reads to {path}", run.samples.len());
    }
    Ok(())
}
