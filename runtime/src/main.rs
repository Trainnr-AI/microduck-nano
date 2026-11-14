// SPDX-License-Identifier: Apache-2.0
//! `duckd`: the runtime's command-line entry point. Run it on the board.

use std::process::ExitCode;
use std::time::Duration;

use duck_runtime::bridge::{Bridge, ROUTER_SOCKET};
use duck_runtime::control;
use duck_runtime::firmware;
use duck_runtime::robot::Robot;

const USAGE: &str = "\
duckd: Microduck Nano runtime (a dry run at this stage; the firmware it talks to must be armed to move a servo)

usage:
  duckd status [--socket PATH]
      Ask the firmware what it is and what state it is in.
  duckd time [--count 200] [--socket PATH]
      Time single read-only calls to the microcontroller, by reply size, to see what the link costs.
  duckd run --robot robot.json [--hz 50] [--seconds 10] [--exchange state|step|two-reads] [--socket PATH]
      Dry-run the control loop: read state every tick, pass a hold-in-place target through the rigrs guards,
      and report round-trip time to the microcontroller and missed deadlines.
        state      one read per tick (default; sends nothing)
        step       one exchange per tick: the guarded hold-in-place targets go out, the state comes back.
                   The firmware ignores targets unless it is armed.
        two-reads  status and joints as two calls (firmware protocol 1)
";

struct Args {
    command: String,
    socket: String,
    robot: Option<String>,
    hz: f64,
    seconds: f64,
    count: u32,
    exchange: control::Exchange,
}

fn parse(mut argv: impl Iterator<Item = String>) -> Result<Args, String> {
    let command = argv.next().ok_or("missing command")?;
    let mut args = Args { command, socket: ROUTER_SOCKET.to_string(), robot: None, hz: 50.0, seconds: 10.0, count: 200, exchange: control::Exchange::State };
    while let Some(flag) = argv.next() {
        let mut value = || argv.next().ok_or(format!("{flag} needs a value"));
        match flag.as_str() {
            "--socket" => args.socket = value()?,
            "--robot" => args.robot = Some(value()?),
            "--hz" => args.hz = value()?.parse().map_err(|_| "--hz needs a number")?,
            "--seconds" => args.seconds = value()?.parse().map_err(|_| "--seconds needs a number")?,
            "--exchange" => {
                args.exchange = match value()?.as_str() {
                    "state" => control::Exchange::State,
                    "step" => control::Exchange::Step,
                    "two-reads" => control::Exchange::TwoReads,
                    _ => return Err("--exchange must be state, step or two-reads".into()),
                }
            }
            "--count" => args.count = value()?.parse().map_err(|_| "--count needs a whole number")?,
            other => return Err(format!("unknown option {other}")),
        }
    }
    if !(args.hz.is_finite() && args.hz > 0.0 && args.hz <= 1000.0) {
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

fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let mut bridge = Bridge::connect(&args.socket, Duration::from_millis(500))?;
    match args.command.as_str() {
        "status" => {
            let info = firmware::info(&mut bridge)?;
            let status = firmware::status(&mut bridge)?;
            println!("firmware protocol {}", info.protocol);
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
        "time" => {
            // Read-only calls with replies of different sizes (values in the reply shown in brackets).
            let zeros = || vec![rmpv::Value::Array(vec![rmpv::Value::from(0); 15])];
            for (method, values, params) in [
                ("duck_info", 6, vec![]),
                ("duck_status", 10, vec![]),
                ("duck_joints", 15, vec![]),
                ("duck_state", 25, vec![]),
                ("duck_step", 25, zeros()),
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
            let period = Duration::from_secs_f64(1.0 / args.hz);
            println!(
                "robot: {} joints in {} chains, {} with a servo ID; loop {} Hz for {} s (dry run)",
                robot.joints(),
                robot.chains.len(),
                robot.ids_assigned,
                args.hz,
                args.seconds
            );
            let stats = control::run(&mut bridge, &robot, period, Duration::from_secs_f64(args.seconds), args.exchange);
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
            if let Some(why) = stats.last_error {
                println!("last error: {why}");
            }
            Ok(())
        }
        other => Err(format!("unknown command {other}\n\n{USAGE}").into()),
    }
}
