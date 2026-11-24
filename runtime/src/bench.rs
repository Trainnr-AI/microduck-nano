// SPDX-License-Identifier: Apache-2.0
//! Moving one servo on the bench to measure it, for system identification (`duckd bench`).
//!
//! Needs a bench build of the firmware (`DUCK_BENCH_MOTION 1`), which arms one joint on its own and keeps it inside
//! that joint's limits and speed limit with the fault checks running
//! (`firmware/app/microduck/sketch/src/duck/bench.h`). This side only chooses the targets: an excitation around
//! wherever the servo was when armed, faded in over the first second and out over the last, so a run starts and
//! ends at rest where it began.

use std::f64::consts::TAU;
use std::io::{Read, Write};
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use crate::bridge::Bridge;
use crate::control::{Ticker, STOP};
use crate::firmware::{self, Bench, BenchReading};
use crate::record::now_ns;

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

/// One tick of a bench run: the target sent and the servo's state that came back.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BenchSample {
    /// When the target was sent and the reply received, nanoseconds after the Unix epoch.
    pub sent_ns: u64,
    pub received_ns: u64,
    /// The target asked for, radians.
    pub target: f64,
    /// The goal the firmware wrote last (after its clamp and speed limit), radians, from the same reply.
    pub setpoint: f64,
    /// The servo's last read in that reply, if it answered.
    pub read: Option<BenchReading>,
}

/// How long exchanges may fail in a row before a run gives up. The board's internal link garbles a call now and
/// then while a servo has power; the firmware itself switches the servo off after its command timeout (0.5 s) of
/// silence, so giving up well before that leaves the stop to this side.
pub const LINK_PATIENCE: Duration = Duration::from_millis(200);

/// How a bench run went.
#[derive(Debug, Default)]
pub struct BenchRun {
    pub samples: Vec<BenchSample>,
    /// Exchanges that failed and were passed over (no sample for those ticks).
    pub lost_exchanges: usize,
    /// Why it ended before its time, if it did (a fault, the servo switched off, the link, Ctrl-C).
    pub stopped: Option<String>,
    /// The last state the firmware reported.
    pub last: Option<Bench>,
}

/// Drive the armed bench servo: `start + amplitude * excitation(t)` (radians) every `period` for `duration`.
/// Stops at once if the firmware reports anything but ARMED. A failed exchange costs its tick and no more, until
/// they have failed for `LINK_PATIENCE` in a row. Does not disarm: the caller does, whatever happened.
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
    let mut ticker = Ticker::new(period);
    let seconds = duration.as_secs_f64();
    let mut failing_since: Option<Instant> = None;
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
        let bench = match firmware::bench_step(bridge, Some(target)) {
            Ok(b) => {
                failing_since = None;
                b
            }
            Err(e) => {
                out.lost_exchanges += 1;
                if failing_since.get_or_insert_with(Instant::now).elapsed() >= LINK_PATIENCE {
                    out.stopped = Some(format!("the link failed: {e}"));
                    break;
                }
                ticker.wait();
                continue;
            }
        };
        out.samples.push(BenchSample { sent_ns, received_ns: now_ns(), target, setpoint: bench.setpoint, read: bench.reading });
        let armed = bench.armed();
        if !armed {
            out.stopped = Some(format!("the firmware switched the servo off: {} ({})", bench.mode, bench.fault));
        }
        out.last = Some(bench);
        if !armed {
            break;
        }
        ticker.wait();
    }
    out
}
