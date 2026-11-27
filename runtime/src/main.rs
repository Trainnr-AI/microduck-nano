// SPDX-License-Identifier: Apache-2.0
//! `duckd`: the runtime's command-line entry point. Run it on the board.

use std::error::Error;
use std::os::unix::net::UnixStream;
use std::process::ExitCode;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use duck_runtime::bench::{self, Excitation};
use duck_runtime::bridge::{Bridge, ROUTER_SOCKET};
use duck_runtime::chase::Chase;
use duck_runtime::control::{self, Direction, Exchange, Sinks, STOP};
use duck_runtime::firmware::{self, Sensed};
use duck_runtime::hear::{self, Heard};
use duck_runtime::pet::{Mood, Pet};
use duck_runtime::policy::{Manifest, Policy, POLICY_JOINTS};
use duck_runtime::record::Recorder;
use duck_runtime::robot::{Robot, JOINT_NAMES};
use duck_runtime::see::{self, Chroma, Lens};
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
            [--command FORWARD,SIDEWAYS,TURN] [--listen DEVICE] [--chase U,V [--ball-mm 40] [--fov 60]
            [--device auto]] [--record FILE.mcap] [--socket PATH]
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
      With --listen (and --policy) a microphone switches the command: the run starts with the policy asked for
      nothing, two claps switch to --command, two more switch back. If the microphone stops, so does the walk.
      DEVICE as for `duckd listen`. Not tried with a microphone yet.
      With --chase (and --policy) the camera chooses the command: turn to face the thing of that colour (as
      `duckd see` prints it), walk to it as fast as the policy was trained to, stop within reach. With --ball-mm
      it slows as it gets near; out of sight for half a second, the policy is asked for nothing. The camera must
      look straight ahead of the body. Not tried on a robot: none walks yet.
      With --record, every tick's measured joints, sent targets and IMU go into an MCAP file (ROS 2 messages) that
      trainnr's ingest_recording reads.
  duckd bench-set --joints NAME,NAME,... --supervised [--excite steps] [--period S] [--amplitude 10] [--seconds 10]
                  [--hz 25] [--record FILE.mcap] [--socket PATH]
      MOVES SEVERAL SERVOS TOGETHER, each a little around where it is (+- amplitude degrees: a sine every
      --period seconds, 2 if not given, faded in and out). Needs what `duckd bench` needs, for every joint named.
      The firmware arms them all or none, keeps each inside its own limits and speed limit, and switches them all
      off when any one faults or duckd stops talking. Prints the lowest voltage and the highest load each of them
      read: how the supply holds with several servos working.
      With --excite steps they all jump from one end to the other at the same instant (there and back every
      --period seconds, 1 if not given): the most the set can ask of the supply at once without a load on it.
      With --amplitude 0 they hold still, for loading them by hand.
  duckd listen --device DEVICE [--seconds 10]
      Listen to a microphone and print what is heard: the level and the room's noise twice a second, and each
      group of claps. DEVICE is an ALSA capture device as `arecord -L` names it (for example plughw:1,0). Needs
      `arecord` on the board and no firmware. Not tried with a microphone yet.
  duckd pet --joint NAME --supervised [--colour U,V] [--device auto] [--fov 60] [--seconds 30]
              [--record FILE.mcap] [--socket PATH]
      MOVES ONE SERVO, as a small animal would. Needs what `duckd bench` needs: a bench build, the joint
      calibrated, the horn free, someone watching; say so with --supervised. With --colour the joint turns to
      point at the coloured thing the camera sees (the camera standing still, pointing where the joint's start
      position points); with nothing in sight it glances about. Hold the horn and it struggles in bursts; keep
      holding and it gives up and goes where the hand takes it for a while. The matrix shows its mood. The
      firmware's limits and faults stay in charge: fought hard enough, its overload check switches the servo off.
  duckd see [--device auto] [--seconds 10] [--colour U,V] [--ball-mm 40] [--fov 60 | --at-mm 300]
      Look for one coloured thing with the camera and print where it is twice a second: frames a second, its
      direction (degrees to the left of and above where the camera points), its size and, with --ball-mm, how
      far away it is. Without --colour the colour in the middle of the first frames is taken: hold the toy in
      front of the camera, and use the --colour it prints from then on. --fov is the angle the picture covers
      from left to right; with --at-mm (the ball held that far away) it is measured instead and printed.
      Needs `v4l2-ctl` on the board and no firmware.
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
    BenchSet,
    Listen,
    See,
    Pet,
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
    /// `bench-set --excite steps`: seconds for one whole there-and-back.
    period_s: Option<f64>,
    amplitude_deg: f64,
    supervised: bool,
    /// `duckd say`'s text.
    text: Option<String>,
    /// `duckd run --command`: what the policy is asked to do.
    walk: Option<Vec<f64>>,
    /// The microphone: `duckd listen --device`, `duckd run --listen`.
    microphone: Option<String>,
    /// `duckd run --chase`: the colour to go after.
    chase: Option<see::Chroma>,
    /// `duckd see`: the video device, the colour looked for, the ball's size, the lens.
    camera: String,
    colour: Option<Chroma>,
    ball_mm: Option<f64>,
    fov_deg: f64,
    at_mm: Option<f64>,
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
        "bench-set" => Command::BenchSet,
        "listen" => Command::Listen,
        "see" => Command::See,
        "pet" => Command::Pet,
        other => return Err(format!("unknown command {other}")),
    };
    let mut args = Args { command, socket: ROUTER_SOCKET.into(), robot: None, hz: None, seconds: 10.0, count: 200,
        exchange: Exchange::State, policy: None, record: None, joint: None, excite: CHIRP, period_s: None,
        amplitude_deg: 10.0, supervised: false, text: None, walk: None, microphone: None, camera: see::AUTO.into(),
        colour: None, chase: None, ball_mm: None, fov_deg: 60.0, at_mm: None };
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
            "--joint" | "--joints" => args.joint = Some(value),
            "--listen" if command == Command::Run => args.microphone = Some(value),
            "--device" if command == Command::Listen => args.microphone = Some(value),
            "--device" if matches!(command, Command::See | Command::Pet | Command::Run) => args.camera = value,
            "--colour" | "--chase" => {
                let uv: Vec<u8> = value.split(',').map(|v| number(&flag, v.trim().to_string())).collect::<Result<_, _>>()?;
                let [u, v] = <[u8; 2]>::try_from(uv).map_err(|_| format!("{flag} needs two numbers from 0 to 255: U,V"))?;
                *(if flag == "--chase" { &mut args.chase } else { &mut args.colour }) = Some(Chroma::new(u, v));
            }
            "--ball-mm" => args.ball_mm = Some(number(&flag, value)?),
            "--fov" => args.fov_deg = number(&flag, value)?,
            "--at-mm" => args.at_mm = Some(number(&flag, value)?),
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
            "--period" => args.period_s = Some(number(&flag, value)?),
            other => return Err(format!("unknown option {other}")),
        }
    }
    if args.hz.is_some_and(|hz| !(hz.is_finite() && hz > 0.0 && hz <= 1000.0)) {
        return Err("--hz must be between 0 and 1000".into());
    }
    if !(args.seconds.is_finite() && args.seconds > 0.0) {
        return Err("--seconds must be positive".into());
    }
    if !(args.fov_deg > 1.0 && args.fov_deg < 179.0) || args.ball_mm.is_some_and(|d| d <= 0.0) || args.at_mm.is_some_and(|d| d <= 0.0) {
        return Err("--fov must be between 1 and 179 degrees, --ball-mm and --at-mm more than 0".into());
    }
    if args.at_mm.is_some() && args.ball_mm.is_none() {
        return Err("--at-mm needs --ball-mm: the lens is measured from a ball of known size at a known distance".into());
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
        Command::BenchSet => with_bridge(&args, |b| bench_set_command(&args, b)),
        Command::Listen => listen(&args),
        Command::See => see_command(&args),
        Command::Pet => with_bridge(&args, |b| pet_command(&args, b)),
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
    // With a microphone or the camera in charge the policy is at times asked for nothing, which it must be able
    // to take too.
    let direction = match (&args.microphone, &args.chase, &policy) {
        (None, None, _) => None,
        (_, _, None) => return Err("--listen and --chase need --policy: they decide what the policy is asked to do".into()),
        (Some(_), Some(_), _) => return Err("--listen or --chase, not both: one thing decides the command".into()),
        (Some(device), None, Some(p)) => {
            p.manifest().check_command(&vec![0.0; command.len()])?;
            Some(walk_on_two_claps(device, command.clone())?)
        }
        (None, Some(colour), Some(p)) => {
            p.manifest().check_command(&vec![0.0; command.len()])?;
            Some(chase_colour(args, *colour, p.manifest().command_ranges)?)
        }
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
    let stats = control::run_directed(
        bridge,
        &robot,
        period,
        duration,
        args.exchange,
        policy.as_mut().map(|p| (p, command.as_slice())),
        direction.as_deref(),
        &mut sinks,
    );
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
/// Start listening to `device` on a thread of its own. The policy starts asked for nothing; each group of
/// exactly two claps switches between that and `command`. When the microphone stops, nothing is asked from then on.
fn walk_on_two_claps(device: &str, command: Vec<f64>) -> Res<Arc<Direction>> {
    let mut microphone = hear::arecord(device)?;
    let direction = Arc::new(Direction::new());
    let shared = Arc::clone(&direction);
    println!("listening on {device}: two claps start the walk, two more stop it");
    std::thread::spawn(move || {
        let mut walking = false;
        let ended = hear::listen(&mut microphone, |_, _, heard| {
            if heard == Some(Heard::Claps(2)) {
                walking = !walking;
                if walking {
                    shared.set(&command);
                } else {
                    shared.clear();
                }
                println!("two claps: {}", if walking { "walking" } else { "stopped" });
            }
            true
        });
        shared.clear();
        match ended {
            Ok(()) => println!("the microphone stopped: the policy is asked for nothing from here on"),
            Err(e) => println!("the microphone failed ({e}): the policy is asked for nothing from here on"),
        }
    });
    Ok(direction)
}

/// Start watching the camera on a thread of its own: every frame with the thing in it asks the policy to go to
/// it, for the next half second only, so losing sight of it (or the camera) leaves the policy asked for nothing.
fn chase_colour(args: &Args, colour: see::Chroma, ranges: [duck_runtime::policy::Range; 3]) -> Res<Arc<Direction>> {
    const SIGHT_LASTS: Duration = Duration::from_millis(500);
    let device = see::find_camera(&args.camera)?;
    let mut camera = see::camera(&device)?;
    let (lens, ball_m, chase) = (Lens::from_fov(args.fov_deg), args.ball_mm.map(|mm| mm / 1000.0), Chase::default());
    let direction = Arc::new(Direction::new());
    let shared = Arc::clone(&direction);
    println!("chasing the colour {},{} seen through {}", colour.u, colour.v, device);
    std::thread::spawn(move || {
        let (mut frame, mut caught) = (Vec::new(), false);
        while matches!(see::read_frame(&mut camera, &mut frame), Ok(true)) {
            let Some(blob) = see::find(&frame, &colour, 12) else { continue };
            let distance = ball_m.map(|d| lens.distance(&blob, d));
            shared.set_for(&chase.command(lens.bearing(&blob), distance, &ranges), SIGHT_LASTS);
            if chase.caught(distance) != caught {
                caught = !caught;
                println!("{}", if caught { "caught it" } else { "it got away" });
            }
        }
        shared.clear();
        println!("the camera stopped: the policy is asked for nothing from here on");
    });
    Ok(direction)
}

/// `duckd listen`: print what a microphone hears.
fn listen(args: &Args) -> Res {
    let device = args.microphone.as_deref().ok_or("listen needs --device (see `arecord -L`)")?;
    let mut microphone = hear::arecord(device)?;
    stop_on_signal(false)?;
    println!("listening on {device} for {} s", args.seconds);
    let frames = (args.seconds / hear::FRAME_TIME.as_secs_f64()).ceil() as u64;
    let report_every = 50; // frames: twice a second
    let (mut heard_frames, mut loudest, mut groups) = (0u64, hear::SILENCE_DB, 0u32);
    hear::listen(&mut microphone, |level, ears, heard| {
        heard_frames += 1;
        loudest = loudest.max(level);
        if let Some(Heard::Claps(n)) = heard {
            groups += 1;
            println!("  {:6.2} s  {n} clap{}", heard_frames as f64 * hear::FRAME_TIME.as_secs_f64(), if n == 1 { "" } else { "s" });
        }
        if heard_frames % report_every == 0 {
            let noise = ears.noise_db().unwrap_or(hear::SILENCE_DB);
            println!("  {:6.2} s  loudest {loudest:6.1} dB   room noise {noise:6.1} dB", heard_frames as f64 * hear::FRAME_TIME.as_secs_f64());
            loudest = hear::SILENCE_DB;
        }
        heard_frames < frames && !STOP.load(Ordering::Relaxed)
    })?;
    if heard_frames == 0 {
        return Err(format!("no sound came from {device}: check the device name with `arecord -L`").into());
    }
    println!("heard {:.1} s, {groups} group(s) of claps", heard_frames as f64 * hear::FRAME_TIME.as_secs_f64());
    Ok(())
}

/// `duckd see`: print where the coloured thing is.
fn see_command(args: &Args) -> Res {
    /// The smallest patch taken for the thing: 12 cells, about 7 x 7 pixels.
    const MIN_CELLS: usize = 12;
    let device = see::find_camera(&args.camera)?;
    let mut camera = see::camera(&device)?;
    stop_on_signal(false)?;
    println!("looking through {} for {} s", device, args.seconds);
    let mut frame = Vec::new();
    let (mut colour, mut lens) = (args.colour, Lens::from_fov(args.fov_deg));
    let ball_m = args.ball_mm.map(|mm| mm / 1000.0);
    let (begin, mut frames, mut found, mut since_report) = (Instant::now(), 0u32, 0u32, Instant::now());
    let mut frames_at_report = 0u32;
    while begin.elapsed().as_secs_f64() < args.seconds && !STOP.load(Ordering::Relaxed) {
        if !see::read_frame(&mut camera, &mut frame)? {
            break;
        }
        frames += 1;
        // The camera's first frames are dark while its exposure settles: learn the colour from the tenth.
        let Some(looked_for) = colour else {
            if frames == 10 {
                let learnt = see::learn(&frame).ok_or("the frame is not the size asked for")?;
                println!("the colour in the middle of the picture: --colour {},{}", learnt.u, learnt.v);
                if learnt.strength() < 25.0 {
                    println!("  it is close to grey ({:.0} from it; under 25): walls and floors will match. Use something with a strong colour", learnt.strength());
                }
                colour = Some(learnt);
            }
            continue;
        };
        let blob = see::find(&frame, &looked_for, MIN_CELLS);
        found += u32::from(blob.is_some());
        if since_report.elapsed() >= Duration::from_millis(500) {
            let rate = f64::from(frames - frames_at_report) / since_report.elapsed().as_secs_f64();
            (since_report, frames_at_report) = (Instant::now(), frames);
            match blob {
                Some(b) => {
                    if let (Some(d), Some(at)) = (ball_m, args.at_mm) {
                        lens = Lens::from_ball(&b, d, at / 1000.0);
                    }
                    let distance = ball_m.map_or(String::new(), |d| format!("   {:.2} m away", lens.distance(&b, d)));
                    println!(
                        "  {rate:4.1} frames/s   {:+6.1} deg left  {:+6.1} deg up   {:.0} x {:.0} px, {:.0} % full{distance}",
                        lens.bearing(&b).to_degrees(),
                        lens.elevation(&b).to_degrees(),
                        b.width,
                        b.height,
                        b.fill() * 100.0
                    );
                }
                None => println!("  {rate:4.1} frames/s   not in sight"),
            }
        }
    }
    if frames == 0 {
        return Err(format!("no frames came from {}: is it the camera? (`v4l2-ctl --list-devices`)", device).into());
    }
    println!("{frames} frames in {:.1} s; the colour was in sight in {found}", begin.elapsed().as_secs_f64());
    if args.at_mm.is_some() {
        let fov = 2.0 * (see::WIDTH as f64 / 2.0 / lens.focal_px).atan().to_degrees();
        println!("measured lens: the picture covers {fov:.1} degrees from left to right (--fov {fov:.1})");
    }
    Ok(())
}

/// Where the coloured thing was last seen (degrees to the left of where the camera points) and when.
type Sighting = Arc<Mutex<Option<(Instant, f64)>>>;

/// Watch the camera on a thread of its own.
fn watch_colour(args: &Args, colour: see::Chroma) -> Res<Sighting> {
    let device = see::find_camera(&args.camera)?;
    let mut camera = see::camera(&device)?;
    let lens = Lens::from_fov(args.fov_deg);
    let seen = Arc::new(Mutex::new(None));
    let report = Arc::clone(&seen);
    std::thread::spawn(move || {
        let mut frame = Vec::new();
        while matches!(see::read_frame(&mut camera, &mut frame), Ok(true)) {
            if let Some(blob) = see::find(&frame, &colour, 12) {
                if let Ok(mut seen) = report.lock() {
                    *seen = Some((Instant::now(), lens.bearing(&blob).to_degrees()));
                }
            }
        }
        println!("the camera stopped: nothing more will be seen");
    });
    Ok(seen)
}

/// `duckd pet`: one bench servo that looks at things and struggles when held.
fn pet_command(args: &Args, bridge: &mut Bridge<UnixStream>) -> Res {
    /// A sighting older than this is no sighting: the thing has gone.
    const IN_SIGHT: Duration = Duration::from_millis(400);
    let name = args.joint.as_deref().ok_or("pet needs --joint NAME")?;
    let joint = JOINT_NAMES.iter().position(|n| *n == name).ok_or_else(|| format!("no joint named `{name}`"))?;
    if !args.supervised {
        return Err("pet MOVES A SERVO. Run it only with the horn free, someone watching and the power at hand, and say \
                    so with --supervised"
            .into());
    }
    if !firmware::info(bridge)?.bench_build {
        return Err("the firmware is not a bench build (DUCK_ENABLE_MOTION 1 and DUCK_BENCH_MOTION 1)".into());
    }
    let mut recorder = args.record.as_deref().map(Recorder::create).transpose()?;
    let seen = args.colour.map(|colour| watch_colour(args, colour)).transpose()?;
    stop_on_signal(false)?;
    let period = Duration::from_secs_f64(1.0 / args.hz.unwrap_or(50.0));
    let mut samples = Vec::new();
    // From the arm request on, every way out goes through the disarm below.
    let outcome = (|| -> Res<Option<String>> {
        firmware::bench_arm(bridge, joint)?.map_err(|why| format!("the firmware refused to arm `{name}`: {why}"))?;
        let start = firmware::bench_step(bridge, None)?.setpoint.to_degrees();
        println!("`{name}` armed at {start:.1} deg for {} s{}", args.seconds, if seen.is_some() { "; watching the camera" } else { "" });
        let (mut pet, mut position, mut mood) = (Pet::new(start), start, Mood::Idle);
        let (begin, mut ticker, mut failing_since) = (Instant::now(), control::Ticker::new(period), None::<Instant>);
        let mut target = start;
        while begin.elapsed().as_secs_f64() < args.seconds {
            if STOP.load(Ordering::Relaxed) {
                return Ok(Some("stopped by Ctrl-C".into()));
            }
            let look = seen.as_ref().and_then(|s| s.lock().ok().and_then(|s| *s)).filter(|(at, _)| at.elapsed() < IN_SIGHT).map(|(_, left)| start + left);
            let sent_ns = duck_runtime::record::now_ns();
            match firmware::bench_step(bridge, Some(target.to_radians())) {
                Ok(b) => {
                    failing_since = None;
                    if !b.armed() {
                        return Ok(Some(format!("the firmware switched the servo off: {} ({})", b.mode, b.fault)));
                    }
                    if let Some(r) = &b.reading {
                        position = r.position.to_degrees();
                    }
                    samples.push(bench::BenchSample { sent_ns, received_ns: duck_runtime::record::now_ns(), target: target.to_radians(), setpoint: b.setpoint, read: b.reading });
                }
                Err(e) => {
                    if failing_since.get_or_insert_with(Instant::now).elapsed() >= bench::LINK_PATIENCE {
                        return Ok(Some(format!("the link failed: {e}")));
                    }
                }
            }
            let (next, now) = pet.tick(period.as_secs_f64(), position, look);
            target = next;
            if now != mood {
                mood = now;
                let text = match mood {
                    Mood::Watching => "LOOK",
                    Mood::Idle => "HM",
                    Mood::Struggling => "GRR",
                    Mood::Checking => "?",
                    Mood::GivenUp => "FINE",
                };
                println!("  {:6.2} s  {mood:?}", begin.elapsed().as_secs_f64());
                // The pause between bursts is too short to be worth a call; an older firmware has no matrix text.
                if mood != Mood::Checking {
                    let _ = firmware::say(bridge, text);
                }
            }
            ticker.wait();
        }
        Ok(None)
    })();
    let disarmed = firmware::bench_disarm(bridge);
    let stopped = outcome?;
    disarmed?;
    println!("switched off. {} ticks", samples.len());
    if let Some(why) = stopped {
        println!("ended early: {why}");
    }
    if let (Some(mut recorder), Some(path)) = (recorder.take(), &args.record) {
        let reads = recorder.bench(name, &samples)?;
        recorder.finish()?;
        println!("recorded {} targets and {reads} servo reads to {path}", samples.len());
    }
    Ok(())
}

/// `duckd bench-set`: several servos armed together, each swinging a little around where it is.
fn bench_set_command(args: &Args, bridge: &mut Bridge<UnixStream>) -> Res {
    let wanted = args
        .joint
        .as_deref()
        .ok_or("bench-set needs --joints NAME,NAME,...")?;
    let mut mask = 0u32;
    for name in wanted.split(',').map(str::trim) {
        let joint = JOINT_NAMES
            .iter()
            .position(|n| *n == name)
            .ok_or_else(|| format!("no joint `{name}`; the joints are {JOINT_NAMES:?}"))?;
        mask |= 1 << joint;
    }
    if !args.supervised {
        return Err("bench-set moves servos. Run it only with every one of them free to turn and someone watching \
                    with the power at hand, and say so with --supervised"
            .into());
    }
    if !firmware::info(bridge)?.bench_build {
        return Err(
            "the firmware is not a bench build (DUCK_ENABLE_MOTION 1 and DUCK_BENCH_MOTION 1)"
                .into(),
        );
    }
    let mut recorder = args.record.as_deref().map(Recorder::create).transpose()?;
    stop_on_signal(false)?;
    let hz = args.hz.unwrap_or(25.0);
    let amplitude = args.amplitude_deg.to_radians();
    let in_set = |j: usize| mask & (1 << j) != 0;
    // Steps: every servo jumps between the two ends at the same instant, which is the most the set can ask of the
    // supply at once (the firmware still holds each to its speed limit). Otherwise a slow sine.
    let steps = args.excite.0 == "steps";
    let cycle_s = args.period_s.unwrap_or(if steps { 1.0 } else { 2.0 });
    if cycle_s.is_nan() || cycle_s <= 0.0 {
        return Err("--period must be more than zero".into());
    }
    struct Seen {
        ticks: u32,
        lost: u32,
        lowest_v: f64,
        peak_load: f64,
        /// Per joint: the lowest voltage and the highest load it read.
        worst: Vec<(f64, f64)>,
        stopped: Option<String>,
        last: Option<firmware::BenchSet>,
    }
    // From the arm request on, every way out goes through the disarm below: even a lost reply may have armed it.
    let outcome = (|| -> Res<Seen> {
        firmware::benchset_arm(bridge, mask)?
            .map_err(|why| format!("the firmware refused to arm the set: {why}"))?;
        let first = firmware::benchset_step(bridge, &[None; JOINT_NAMES.len()])?;
        let start: Vec<Option<f64>> = first
            .joints
            .iter()
            .enumerate()
            .map(|(j, s)| s.position.filter(|_| in_set(j)))
            .collect();
        println!(
            "{} servos armed; each +- {} deg around where it is, {} every {cycle_s} s, for {} s at {hz} Hz",
            first.armed(),
            args.amplitude_deg,
            if steps { "all stepping together" } else { "a sine" },
            args.seconds
        );
        let mut seen = Seen {
            ticks: 0,
            lost: 0,
            lowest_v: f64::INFINITY,
            peak_load: 0.0,
            worst: vec![(f64::INFINITY, 0.0); JOINT_NAMES.len()],
            stopped: None,
            last: None,
        };
        let (begin, period) = (std::time::Instant::now(), Duration::from_secs_f64(1.0 / hz));
        let mut lost_in_a_row = 0;
        while !STOP.load(Ordering::Relaxed) {
            let t = begin.elapsed().as_secs_f64();
            if t >= args.seconds {
                break;
            }
            // Faded in and out over a second, so nothing starts or ends with a jerk.
            let fade = t.min(args.seconds - t).clamp(0.0, 1.0);
            let wave = (std::f64::consts::TAU * t / cycle_s).sin();
            // A step run ends back where it began: the last second is spent there.
            let swing = if steps {
                if args.seconds - t < 1.0 {
                    0.0
                } else {
                    amplitude * wave.signum()
                }
            } else {
                amplitude * fade * wave
            };
            let targets: Vec<Option<f64>> = start.iter().map(|s| s.map(|s| s + swing)).collect();
            match firmware::benchset_step(bridge, &targets) {
                Ok(set) => {
                    lost_in_a_row = 0;
                    seen.ticks += 1;
                    for ((j, s), joint_name) in set
                        .joints
                        .iter()
                        .enumerate()
                        .zip(JOINT_NAMES)
                        .filter(|((j, _), _)| in_set(*j))
                    {
                        if s.answered {
                            seen.lowest_v = seen.lowest_v.min(s.voltage);
                            seen.peak_load = seen.peak_load.max(s.load_percent.abs());
                            if let Some(w) = seen.worst.get_mut(j) {
                                *w = (w.0.min(s.voltage), w.1.max(s.load_percent.abs()));
                            }
                        }
                        if s.mode != "ARMED" && seen.stopped.is_none() {
                            seen.stopped = Some(format!(
                                "`{joint_name}` went {} ({}), joint {j}",
                                s.mode, s.fault
                            ));
                        }
                    }
                    if let Some(recorder) = recorder.as_mut() {
                        let sensed = firmware::Sensed {
                            joints: set
                                .joints
                                .iter()
                                .map(|s| s.position.filter(|_| s.answered))
                                .collect(),
                            pitch_roll: None,
                            angular_velocity: None,
                            joint_speeds: vec![],
                        };
                        let sent: Vec<f64> =
                            targets.iter().map(|t| t.unwrap_or(f64::NAN)).collect();
                        recorder.tick(&sensed, Some(&sent))?;
                    }
                    let over = seen.stopped.is_some();
                    seen.last = Some(set);
                    if over {
                        break;
                    }
                }
                Err(_) if lost_in_a_row < 3 => {
                    lost_in_a_row += 1;
                    seen.lost += 1;
                }
                Err(e) => return Err(e.into()),
            }
            std::thread::sleep(period.saturating_sub(begin.elapsed() - Duration::from_secs_f64(t)));
        }
        Ok(seen)
    })();
    let disarmed = firmware::bench_disarm(bridge);
    let seen = outcome?;
    disarmed?;
    println!("switched off. {} ticks", seen.ticks);
    if seen.lost > 0 {
        println!(
            "{} exchange(s) with the firmware failed and were passed over",
            seen.lost
        );
    }
    if let Some(why) = &seen.stopped {
        println!("ended early: {why}; the others were switched off with it");
    }
    if seen.lowest_v.is_finite() {
        println!(
            "lowest voltage any servo read: {:.1} V; highest load: {:.1} %",
            seen.lowest_v, seen.peak_load
        );
    }
    if let Some(set) = &seen.last {
        for (((j, s), joint_name), (low_v, top_load)) in set
            .joints
            .iter()
            .enumerate()
            .zip(JOINT_NAMES)
            .zip(&seen.worst)
            .filter(|(((j, _), _), _)| in_set(*j))
        {
            let at = s
                .position
                .map_or("?".into(), |p| format!("{:.1}", p.to_degrees()));
            println!("  {j:>2} {joint_name:<16} {:<5} {:<20} at {at} deg  lowest {low_v:.1} V  highest load {top_load:.1} %", s.mode, s.fault);
        }
    }
    if let (Some(recorder), Some(path)) = (recorder.take(), &args.record) {
        println!("recorded {} ticks to {path}", recorder.finish()?);
    }
    Ok(())
}

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
    if run.lost_exchanges > 0 {
        println!("{} exchange(s) with the firmware failed and were passed over", run.lost_exchanges);
    }
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
