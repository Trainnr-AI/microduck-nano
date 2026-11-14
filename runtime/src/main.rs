// SPDX-License-Identifier: Apache-2.0
//! `duckd`: the runtime's command-line entry point. Run it on the board.

use std::process::ExitCode;
use std::time::Duration;

use duck_runtime::bench::{self, Excitation};
use duck_runtime::bridge::{Bridge, ROUTER_SOCKET};
use duck_runtime::control;
use duck_runtime::firmware;
use duck_runtime::policy::{Manifest, Policy, Sensed};
use duck_runtime::robot::{Robot, JOINT_NAMES};
use duck_runtime::record::Recorder;
use duck_runtime::status_led::StatusLed;

const USAGE: &str = "\
duckd: Microduck Nano runtime (a dry run at this stage; the firmware it talks to must be armed to move a servo)

usage:
  duckd status [--socket PATH]
      Ask the firmware what it is and what state it is in.
  duckd say TEXT... [--socket PATH]
      Put TEXT on the board's LED matrix for a few seconds (up to 32 characters; the words are joined).
  duckd time [--count 200] [--socket PATH]
      Time single read-only calls to the microcontroller, by reply size, to see what the link costs.
  duckd policy-time --robot robot.json --policy DEPLOYMENT [--count 2000]
      Load a policy (a trainnr deployment: its folder or deploy.json), check it against the robot, and time inference alone
      on made-up observations. Needs no firmware.
  duckd run --robot robot.json [--hz 50] [--seconds 10] [--exchange state|step|two-reads] [--policy DEPLOYMENT]
            [--record FILE.mcap] [--socket PATH]
      Dry-run the control loop: read state every tick, pass a hold-in-place target through the rigrs guards,
      and report round-trip time to the microcontroller and missed deadlines.
        state      one read per tick (default; sends nothing)
        step       one exchange per tick: the guarded hold-in-place targets go out, the state comes back.
                   The firmware ignores targets unless it is armed.
        two-reads  status and joints as two calls (firmware protocol 1; no gyro)
      With --policy the targets come from the policy (zero command) instead of hold-in-place; the loop runs at the
      policy's rate. A tick on which an input the policy needs was not measured holds instead.
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

struct Args {
    command: String,
    socket: String,
    robot: Option<String>,
    /// Loop rate; `run` defaults to 50 Hz, `bench` to the firmware's 100 Hz tick.
    hz: Option<f64>,
    seconds: f64,
    count: u32,
    exchange: control::Exchange,
    policy: Option<String>,
    record: Option<String>,
    joint: Option<String>,
    excite: String,
    amplitude_deg: f64,
    supervised: bool,
    /// `duckd say`'s text.
    text: Option<String>,
}

fn parse(mut argv: impl Iterator<Item = String>) -> Result<Args, String> {
    let command = argv.next().ok_or("missing command")?;
    let mut args = Args { command, socket: ROUTER_SOCKET.to_string(), robot: None, hz: None, seconds: 10.0, count: 200, exchange: control::Exchange::State, policy: None, record: None, text: None, joint: None, excite: "chirp".into(), amplitude_deg: 10.0, supervised: false };
    while let Some(flag) = argv.next() {
        if flag == "--supervised" {
            args.supervised = true;
            continue;
        }
        let mut value = || argv.next().ok_or(format!("{flag} needs a value"));
        match flag.as_str() {
            "--joint" => args.joint = Some(value()?),
            "--excite" => args.excite = value()?,
            "--amplitude" => args.amplitude_deg = value()?.parse().map_err(|_| "--amplitude needs a number (degrees)")?,
            "--socket" => args.socket = value()?,
            "--robot" => args.robot = Some(value()?),
            "--hz" => args.hz = Some(value()?.parse().map_err(|_| "--hz needs a number")?),
            "--seconds" => args.seconds = value()?.parse().map_err(|_| "--seconds needs a number")?,
            "--exchange" => {
                args.exchange = match value()?.as_str() {
                    "state" => control::Exchange::State,
                    "step" => control::Exchange::Step,
                    "two-reads" => control::Exchange::TwoReads,
                    _ => return Err("--exchange must be state, step or two-reads".into()),
                }
            }
            "--policy" => args.policy = Some(value()?),
            "--record" => args.record = Some(value()?),
            "--count" => args.count = value()?.parse().map_err(|_| "--count needs a whole number")?,
            word if args.command == "say" && !word.starts_with("--") => {
                let text = args.text.get_or_insert_with(String::new);
                if !text.is_empty() {
                    text.push(' ');
                }
                text.push_str(word);
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
    match run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(why) => {
            eprintln!("duckd: {why}");
            ExitCode::FAILURE
        }
    }
}

/// Ctrl-C or SIGTERM sets `control::STOP`, which ends a loop after its tick; a second one exits at once.
fn stop_on_signal() -> Result<(), ctrlc::Error> {
    ctrlc::set_handler(|| {
        if control::STOP.swap(true, std::sync::atomic::Ordering::Relaxed) {
            std::process::exit(130);
        }
    })
}

/// `duckd bench`: one servo, armed alone, moved and switched off again (whatever happens in between).
fn bench_command(args: &Args, bridge: &mut Bridge<std::os::unix::net::UnixStream>) -> Result<(), Box<dyn std::error::Error>> {
    let name = args.joint.as_deref().ok_or("bench needs --joint NAME")?;
    let joint = JOINT_NAMES.iter().position(|n| *n == name).ok_or_else(|| format!("no joint `{name}`; the joints are {JOINT_NAMES:?}"))?;
    let excitation = match args.excite.as_str() {
        "chirp" => Excitation::Chirp { from_hz: 0.1, to_hz: 2.0 },
        "steps" => Excitation::Steps { period_s: 2.0 },
        _ => return Err("--excite must be chirp or steps".into()),
    };
    if !(args.amplitude_deg > 0.0 && args.amplitude_deg <= 30.0) {
        return Err("--amplitude must be more than 0 and at most 30 degrees".into());
    }
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
    // Ctrl-C ends the run after its tick (at most 10 ms), and then the servo is switched off; a second Ctrl-C does
    // not skip that.
    ctrlc::set_handler(|| control::STOP.store(true, std::sync::atomic::Ordering::Relaxed))?;
    let hz = args.hz.unwrap_or(100.0);
    // From the arm request on, every way out goes through the disarm below: even a lost reply may have armed it.
    let outcome = (|| -> Result<bench::BenchRun, Box<dyn std::error::Error>> {
        let armed = firmware::bench_arm(bridge, joint)?;
        if armed != "ok" {
            return Err(format!("the firmware refused to arm `{name}`: {armed}").into());
        }
        let start = firmware::bench_step(bridge, None)?.setpoint;
        println!("`{name}` armed at {:.1} deg; {} +- {} deg for {} s at {hz} Hz", start.to_degrees(), args.excite, args.amplitude_deg, args.seconds);
        Ok(bench::run(
            bridge,
            start,
            args.amplitude_deg.to_radians(),
            excitation,
            Duration::from_secs_f64(args.seconds),
            Duration::from_secs_f64(1.0 / hz),
        ))
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
    if let (Some(recorder), Some(path)) = (recorder.as_mut(), &args.record) {
        let reads = recorder.bench(name, &run.samples)?;
        println!("recorded {} targets and {reads} servo reads to {path}", run.samples.len());
    }
    if let Some(recorder) = recorder {
        recorder.finish()?;
    }
    Ok(())
}

fn load_policy(args: &Args, robot: &Robot) -> Result<Option<Policy>, Box<dyn std::error::Error>> {
    let Some(path) = args.policy.as_deref() else { return Ok(None) };
    let manifest = Manifest::load(path)?;
    Ok(Some(Policy::load(manifest, robot)?))
}

fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    if args.command == "policy-time" {
        let path = args.robot.as_deref().ok_or("policy-time needs --robot")?;
        let robot = Robot::load(path)?;
        let mut policy = load_policy(args, &robot)?.ok_or("policy-time needs --policy")?;
        let m = policy.manifest();
        println!("policy `{}`: {} observations -> 14 actions, trained at {} Hz", m.name, m.observation_size(), m.control_hz);
        let command = vec![0.0; m.command_size()];
        let period = 1.0 / m.control_hz;
        let mut times = Vec::with_capacity(args.count as usize);
        for i in 0..args.count {
            // made-up but changing inputs: every joint swinging a little, the trunk tilting a little
            let phase = f64::from(i) * 0.1;
            let sensed = Sensed {
                joints: (0..15).map(|j| Some(0.05 * (phase + f64::from(j)).sin())).collect(),
                pitch_roll: Some((0.02 * phase.sin(), 0.02 * phase.cos())),
                angular_velocity: Some([0.1 * phase.sin(), 0.1 * phase.cos(), 0.0]),
            };
            policy.act(&sensed, &command, period)?;
            times.push(policy.last_inference_us);
        }
        times.sort_unstable();
        let at = |p: f64| times.get((times.len().saturating_sub(1) as f64 * p) as usize).copied().unwrap_or(0) as f64 / 1000.0;
        println!("inference: median {:.3} ms   95th {:.3} ms   max {:.3} ms   ({} runs)", at(0.5), at(0.95), at(1.0), times.len());
        return Ok(());
    }
    let mut bridge = Bridge::connect(&args.socket, Duration::from_millis(500))?;
    match args.command.as_str() {
        "status" => {
            let info = firmware::info(&mut bridge)?;
            let status = firmware::status(&mut bridge)?;
            println!("firmware protocol {} (this runtime speaks {})", info.protocol, firmware::PROTOCOL);
            println!("  motion build: {}   permanent-memory writes: {}", info.motion_build, info.eeprom_build);
            println!("  joint table ready: {}   IMU present: {}", info.joint_table_ready, info.imu_present);
            println!("  servo port is Serial1: {}", info.servo_uart_is_serial1);
            println!("mode {}   posture {}   fault {}", status.mode, status.posture, status.fault);
            println!("  IMU ok: {}   pitch {:.1} deg   roll {:.1} deg", status.imu_ok, status.pitch_deg, status.roll_deg);
            println!("  flags 0x{:x}   worst firmware tick {} us", status.flags, status.worst_tick_us);
            let joints = firmware::joints(&mut bridge)?;
            let answering = joints.iter().filter(|j| j.is_some()).count();
            println!("servos answering: {answering}/{}", joints.len());
            Ok(())
        }
        "say" => {
            let text = args.text.as_deref().ok_or("say needs the text")?;
            let kept = firmware::say(&mut bridge, text)?;
            println!("on the matrix: {kept} characters");
            Ok(())
        }
        "bench" => bench_command(args, &mut bridge),
        "time" => {
            // Read-only calls with replies of different sizes (values in the reply shown in brackets).
            let zeros = || vec![rmpv::Value::Array(vec![rmpv::Value::from(0); 15])];
            for (method, values, params) in [
                ("duck_info", 6, vec![]),
                ("duck_status", 10, vec![]),
                ("duck_joints", 15, vec![]),
                ("duck_state", 28, vec![]),
                ("duck_step", 28, zeros()),
                ("duck_health", 45, vec![]),
            ] {
                let mut times = Vec::with_capacity(args.count as usize);
                for _ in 0..args.count {
                    let start = std::time::Instant::now();
                    bridge.call(method, params.clone())?;
                    times.push(start.elapsed().as_micros());
                }
                times.sort_unstable();
                let at = |p: f64| times.get(((times.len().saturating_sub(1)) as f64 * p) as usize).copied().unwrap_or(0) as f64 / 1000.0;
                println!(
                    "{method:12} [{values:2} values]  median {:.2} ms   95th {:.2} ms   max {:.2} ms   ({} calls)",
                    at(0.5), at(0.95), at(1.0), times.len()
                );
            }
            Ok(())
        }
        "run" => {
            let path = args.robot.as_deref().ok_or("run needs --robot <path to firmware/config/robot.json>")?;
            let robot = Robot::load(path)?;
            let mut policy = load_policy(args, &robot)?;
            let protocol = firmware::info(&mut bridge)?.protocol;
            if args.exchange != control::Exchange::TwoReads && protocol < firmware::PROTOCOL {
                return Err(format!(
                    "the firmware speaks protocol {protocol}; --exchange state and step need {} (the state message \
                     with the gyro). Push the app from this repository, or use --exchange two-reads",
                    firmware::PROTOCOL
                )
                .into());
            }
            let hz = policy.as_ref().map_or(args.hz.unwrap_or(50.0), |p| p.manifest().control_hz);
            let command = vec![0.0; policy.as_ref().map_or(0, |p| p.manifest().command_size())];
            let period = Duration::from_secs_f64(1.0 / hz);
            println!(
                "robot: {} joints in {} chains, {} with a servo ID; loop {} Hz for {} s (dry run)",
                robot.joints(),
                robot.chains.len(),
                robot.ids_assigned,
                hz,
                args.seconds
            );
            if let Some(p) = &policy {
                println!("policy `{}`, zero command", p.manifest().name);
            }
            // Ctrl-C or SIGTERM: finish the tick, report, and put the light out. A second one exits at once.
            stop_on_signal()?;
            let led = StatusLed::find();
            println!("status light (LED1): {}", if led.present() { "yes" } else { "not found" });
            let recorder = args.record.as_deref().map(Recorder::create).transpose()?;
            let mut sinks = control::Sinks { led, recorder, record_error: None };
            // Announce on the matrix; an older firmware without duck_say just does not show it.
            let _ = firmware::say(&mut bridge, if policy.is_some() { "DUCKD POLICY" } else { "DUCKD RUN" });
            let stats = control::run(
                &mut bridge,
                &robot,
                period,
                Duration::from_secs_f64(args.seconds),
                args.exchange,
                policy.as_mut().map(|p| (p, command.as_slice())),
                &mut sinks,
            );
            drop(sinks.led); // the light goes out with the loop
            if let (Some(recorder), Some(path)) = (sinks.recorder, &args.record) {
                println!("recorded {} ticks to {path}", recorder.finish()?);
            }
            if let Some(why) = &sinks.record_error {
                println!("{why}; the recording stopped there and is not readable");
            }
            let _ = firmware::say(&mut bridge, "DUCKD DONE");
            let ms = |us: Option<u64>| us.map_or("-".to_string(), |us| format!("{:.2}", us as f64 / 1000.0));
            println!("ticks {}   errors {}   missed deadlines {}", stats.ticks, stats.errors, stats.missed_deadlines);
            println!(
                "exchange per tick ({:?}): median {} ms   95th {} ms   max {} ms   (period {:.2} ms)",
                args.exchange,
                ms(stats.percentile(0.5)),
                ms(stats.percentile(0.95)),
                ms(stats.percentile(1.0)),
                period.as_secs_f64() * 1000.0
            );
            println!(
                "guard verdicts: move {}   hold {}   chain unmeasured {}",
                stats.moves, stats.holds, stats.unmeasured_chain_ticks
            );
            if policy.is_some() {
                println!(
                    "policy: ran {} ticks, skipped {}   inference median {} ms   max {} ms",
                    stats.policy_ran,
                    stats.policy_skipped,
                    ms(stats.inference_percentile(0.5)),
                    ms(stats.inference_percentile(1.0))
                );
                if let Some(why) = &stats.last_policy_skip {
                    println!("last skip: {why}");
                }
            }
            if let Some(why) = stats.last_error {
                println!("last error: {why}");
            }
            Ok(())
        }
        other => Err(format!("unknown command {other}\n\n{USAGE}").into()),
    }
}
