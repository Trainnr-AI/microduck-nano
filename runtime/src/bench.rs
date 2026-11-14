// SPDX-License-Identifier: Apache-2.0
//! Moving one servo on the bench to measure it, for system identification (`duckd bench`).
//!
//! Needs a bench build of the firmware (`DUCK_BENCH_MOTION 1`), which arms one joint on its own and keeps it inside
//! that joint's limits and speed limit with the fault checks running (`firmware/app/microduck/sketch/src/duck/bench.h`).
//! This side only chooses the targets: an excitation around wherever the servo was when armed, faded in over the
//! first second and out over the last, so a run starts and ends at rest where it began.

use std::f64::consts::TAU;
use std::io::{Read, Write};
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use crate::bridge::Bridge;
use crate::control::STOP;
use crate::firmware::{self, Bench};
use crate::record::{now_ns, BenchRead, BenchSample};

/// The shape of the motion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Excitation {
    /// A sine whose frequency rises linearly from `from_hz` to `to_hz` over the run.
    Chirp { from_hz: f64, to_hz: f64 },
    /// A square wave: the target jumps between +1 and -1 every half `period_s`.
    Steps { period_s: f64 },
}

impl Excitation {
    /// The excitation at `t` seconds of a run of `duration` seconds, in -1..1, faded in and out over one second each.
    pub fn at(&self, t: f64, duration: f64) -> f64 {
        if !(0.0..=duration).contains(&t) {
            return 0.0;
        }
        let fade = t.min(duration - t).clamp(0.0, 1.0);
        let raw = match *self {
            Excitation::Chirp { from_hz, to_hz } => {
                let rate = (to_hz - from_hz) / duration.max(1e-9);
                (TAU * (from_hz * t + 0.5 * rate * t * t)).sin()
            }
            Excitation::Steps { period_s } => {
                if (t / period_s.max(1e-9)).fract() < 0.5 {
                    1.0
                } else {
                    -1.0
                }
            }
        };
        fade * raw
    }
}

/// How a bench run went.
#[derive(Debug, Default)]
pub struct BenchRun {
    pub samples: Vec<BenchSample>,
    /// Why it ended before its time, if it did (a fault, the servo switched off, the link, Ctrl-C).
    pub stopped: Option<String>,
    /// The last state the firmware reported.
    pub last: Option<Bench>,
}

/// Drive the armed bench servo: `start + amplitude * excitation(t)` (radians) every `period` for `duration`.
/// Stops at once if the firmware reports anything but ARMED. Does not disarm: the caller does, whatever happened.
pub fn run<S: Read + Write>(
    bridge: &mut Bridge<S>,
    start: f64,
    amplitude: f64,
    excitation: Excitation,
    duration: Duration,
    period: Duration,
) -> BenchRun {
    let mut out = BenchRun::default();
    let begin = Instant::now();
    let mut next = begin;
    let seconds = duration.as_secs_f64();
    loop {
        let t = begin.elapsed().as_secs_f64();
        if t > seconds {
            break;
        }
        if STOP.load(Ordering::Relaxed) {
            out.stopped = Some("stopped by Ctrl-C".into());
            break;
        }
        let target = start + amplitude * excitation.at(t, seconds);
        let sent_ns = now_ns();
        let reply = firmware::bench_step(bridge, Some(target));
        let received_ns = now_ns();
        let bench = match reply {
            Ok(b) => b,
            Err(e) => {
                out.stopped = Some(format!("the link failed: {e}"));
                break;
            }
        };
        let read = bench.reading.as_ref().map(|r| BenchRead { read_ms: r.read_ms, position: r.position, velocity: r.velocity });
        out.samples.push(BenchSample { sent_ns, received_ns, target, setpoint: bench.setpoint, read });
        let armed = bench.mode == "ARMED";
        if !armed {
            out.stopped = Some(format!("the firmware switched the servo off: {} ({})", bench.mode, bench.fault));
        }
        out.last = Some(bench);
        if !armed {
            break;
        }
        next += period;
        match next.checked_duration_since(Instant::now()) {
            Some(wait) => std::thread::sleep(wait),
            None => next = Instant::now(),
        }
    }
    out
}
