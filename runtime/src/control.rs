// SPDX-License-Identifier: Apache-2.0
//! The fixed-rate loop.
//!
//! Each tick reads the robot's state from the firmware and passes a target for every chain through its rigrs
//! guard: a policy's targets, or "stay where you are" without one (or on a tick it cannot run). By default nothing
//! is sent back to the firmware; with [`Exchange::Step`] the guarded targets are sent, which a firmware that is not
//! armed ignores. It also measures how long a round trip to the microcontroller takes and how many ticks miss their
//! deadline, which decides the control rate a policy can have.

use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use arm::{Guard, Verdict};

use crate::bridge::{Bridge, BridgeError};
use crate::firmware::{self, Sensed};
use crate::limits::settle_targets;
use crate::policy::Policy;
use crate::record::Recorder;
use crate::robot::Robot;
use crate::status_led::{Colour, StatusLed};

/// Set to end a running loop early (Ctrl-C in `duckd`); the loop finishes its tick and returns its statistics.
pub static STOP: AtomicBool = AtomicBool::new(false);

/// Commands older than this are stale and the guard holds (rigrs `Guard`).
const SOURCE_TIMEOUT_MS: u64 = 500;

/// Where the loop reports each tick besides its statistics: the status light, and a recording if one is open.
pub struct Sinks {
    pub led: StatusLed,
    pub recorder: Option<Recorder>,
    /// Why the recording stopped early, if it did; the loop itself carries on.
    pub record_error: Option<String>,
}

impl Sinks {
    /// No light, no recording (tests, timing runs).
    pub fn none() -> Self {
        Sinks { led: StatusLed::none(), recorder: None, record_error: None }
    }

    /// Record a tick if a recording is open; on the first failure keep why and close it (the loop carries on).
    fn record(&mut self, sensed: &Sensed, sent: Option<&[f64]>) {
        if let Some(Err(e)) = self.recorder.as_mut().map(|r| r.tick(sensed, sent)) {
            self.record_error = Some(e.to_string());
            self.recorder = None;
        }
    }
}

/// A fixed-rate clock that never tries to catch up: a burst of late ticks is worse than a slow one.
pub struct Ticker {
    next: Instant,
    period: Duration,
}

impl Ticker {
    pub fn new(period: Duration) -> Self {
        Ticker { next: Instant::now(), period }
    }

    /// Sleep until the next tick. `false` if it was already late (the clock then restarts from now).
    pub fn wait(&mut self) -> bool {
        self.next += self.period;
        match self.next.checked_duration_since(Instant::now()) {
            Some(wait) => {
                std::thread::sleep(wait);
                true
            }
            None => {
                self.next = Instant::now();
                false
            }
        }
    }
}

/// What one chain's guard said on one tick.
#[derive(Debug, Clone, PartialEq)]
pub enum ChainOutcome {
    /// At least one servo in the chain did not report a position, so there is nothing to authorise against.
    Unmeasured,
    Guarded(Verdict),
}

struct ChainGuard {
    name: &'static str,
    first: usize,
    joints: usize,
    guard: Guard,
}

/// The guards for every chain of the robot, at one control period.
pub struct Guards {
    chains: Vec<ChainGuard>,
}

impl Guards {
    pub fn new(robot: &Robot, period: Duration) -> Self {
        let chains = robot
            .chains
            .iter()
            .map(|chain| ChainGuard {
                name: chain.name,
                first: chain.first,
                joints: chain.spec.joints(),
                guard: Guard::new(chain.spec.clone(), period.as_secs_f64(), SOURCE_TIMEOUT_MS),
            })
            .collect();
        Guards { chains }
    }

    /// Tell every guard a fresh command arrived at `now_ms`.
    pub fn fed(&mut self, now_ms: u64) {
        for chain in &mut self.chains {
            chain.guard.fed(now_ms);
        }
    }

    /// Pass `wanted` (radians, firmware joint order) through each chain's guard against `measured`.
    pub fn authorise(&mut self, now_ms: u64, measured: &[Option<f64>], wanted: &[f64]) -> Vec<(&'static str, ChainOutcome)> {
        self.chains
            .iter_mut()
            .map(|chain| {
                let range = chain.first..chain.first + chain.joints;
                let outcome = match (measured.get(range.clone()), wanted.get(range)) {
                    (Some(m), Some(w)) => match m.iter().copied().collect::<Option<Vec<f64>>>() {
                        Some(m) => ChainOutcome::Guarded(chain.guard.authorise(now_ms, &m, w)),
                        None => ChainOutcome::Unmeasured,
                    },
                    _ => ChainOutcome::Unmeasured,
                };
                (chain.name, outcome)
            })
            .collect()
    }
}

/// How a tick talks to the firmware.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Exchange {
    /// Two reads, `duck_status` then `duck_joints` (firmware protocol 1).
    TwoReads,
    /// One read, `duck_state` (protocol 4). With `Step`, the only exchanges that carry the gyro and joint speeds.
    State,
    /// One exchange, `duck_step`: send the previous tick's guarded targets, get the state back (protocol 4).
    Step,
}

/// Timing of a run of the loop.
#[derive(Debug, Clone, Default)]
pub struct LoopStats {
    pub ticks: u32,
    pub errors: u32,
    pub missed_deadlines: u32,
    pub unmeasured_chain_ticks: u32,
    pub moves: u32,
    pub holds: u32,
    /// Ticks on which the leg rules (hip maps, knee x ankle, both legs) held at least one leg joint back.
    pub leg_rule_ticks: u32,
    /// Round-trip times of the calls made in each tick, in microseconds, sorted.
    pub round_trip_us: Vec<u64>,
    pub last_error: Option<String>,
    /// Ticks on which the policy produced targets, and ticks on which it could not (an input was missing).
    pub policy_ran: u32,
    pub policy_skipped: u32,
    pub last_policy_skip: Option<String>,
    /// Inference times in microseconds, sorted.
    pub inference_us: Vec<u64>,
}

impl LoopStats {
    pub fn percentile(&self, p: f64) -> Option<u64> {
        percentile(&self.round_trip_us, p)
    }

    pub fn inference_percentile(&self, p: f64) -> Option<u64> {
        percentile(&self.inference_us, p)
    }
}

/// The value at fraction `p` of a sorted list (nearest rank), if it has any.
pub fn percentile(sorted: &[u64], p: f64) -> Option<u64> {
    let last = sorted.len().checked_sub(1)?;
    let index = ((last as f64) * p).round() as usize;
    sorted.get(index.min(last)).copied()
}

/// Run the loop for `duration` at `period`. Without a policy the target is "stay where you are"; with one, the
/// policy's targets (for `command`) go through the guards. Either way nothing moves unless the firmware is armed.
///
/// `sinks.led` shows each tick: green = holding in place (no policy), blue = the policy produced the targets,
/// yellow = the policy could not run this tick (an input was missing), red = the exchange with the firmware failed.
/// `sinks.recorder`, if open, gets every tick that reached the firmware: what was measured and what was sent.
pub fn run<S: Read + Write>(
    bridge: &mut Bridge<S>,
    robot: &Robot,
    period: Duration,
    duration: Duration,
    exchange: Exchange,
    mut policy: Option<(&mut Policy, &[f64])>,
    sinks: &mut Sinks,
) -> LoopStats {
    let mut guards = Guards::new(robot, period);
    // Checked when the robot file was loaded; if it somehow cannot be built, nothing is ever sent.
    let leg_rules = robot.leg_limits.rules().ok();
    let mut stats = LoopStats::default();
    let start = Instant::now();
    let mut ticker = Ticker::new(period);
    // What the guards allowed on the previous tick, to send with this one. None until every chain has moved once.
    let mut allowed: Option<Vec<f64>> = None;
    while start.elapsed() < duration && !STOP.load(Ordering::Relaxed) {
        let tick_start = Instant::now();
        let now_ms = u64::try_from(start.elapsed().as_millis()).unwrap_or(u64::MAX);
        // The targets that go out with this tick's exchange (also what the recording says was sent).
        let sending = allowed.as_deref().filter(|_| exchange == Exchange::Step);
        match read_state(bridge, exchange, sending) {
            Ok(sensed) => {
                sinks.record(&sensed, sending);
                let measured = &sensed.joints;
                // Without a policy (or when it cannot run this tick) the command is: hold every joint where it is.
                let hold = || measured.iter().map(|m| m.unwrap_or(0.0)).collect::<Vec<f64>>();
                let wanted = match policy.as_mut() {
                    None => {
                        sinks.led.show(Colour::Green);
                        hold()
                    }
                    Some((policy, command)) => match policy.act(&sensed, command, period.as_secs_f64()) {
                        Ok(targets) => {
                            stats.policy_ran += 1;
                            stats.inference_us.push(policy.last_inference_us);
                            sinks.led.show(Colour::Blue);
                            targets
                        }
                        Err(why) => {
                            stats.policy_skipped += 1;
                            stats.last_policy_skip = Some(why.to_string());
                            policy.reset();
                            sinks.led.show(Colour::Yellow);
                            hold()
                        }
                    },
                };
                guards.fed(now_ms);
                let mut next_targets = Vec::with_capacity(wanted.len());
                for (_, outcome) in guards.authorise(now_ms, measured, &wanted) {
                    match outcome {
                        ChainOutcome::Unmeasured => stats.unmeasured_chain_ticks += 1,
                        ChainOutcome::Guarded(Verdict::Move { radians }) => {
                            stats.moves += 1;
                            next_targets.extend(radians.iter().copied());
                        }
                        ChainOutcome::Guarded(_) => stats.holds += 1,
                    }
                }
                // Only a full set of authorised targets is ever sent; a partial one is dropped. A full set then
                // goes through the limits over several joints, against the pose last sent (the measured one on the
                // first tick): a leg joint those rules hold stays at that pose.
                allowed = if next_targets.len() == wanted.len() {
                    let previous = allowed.take().unwrap_or_else(hold);
                    leg_rules.as_ref().and_then(|rules| settle_targets(rules, &previous, &mut next_targets)).map(
                        |settled| {
                            if !settled.unchanged() {
                                stats.leg_rule_ticks += 1;
                            }
                            next_targets
                        },
                    )
                } else {
                    None
                };
            }
            Err(e) => {
                stats.errors += 1;
                stats.last_error = Some(e.to_string());
                sinks.led.show(Colour::Red);
            }
        }
        stats.round_trip_us.push(u64::try_from(tick_start.elapsed().as_micros()).unwrap_or(u64::MAX));
        stats.ticks += 1;
        if !ticker.wait() {
            stats.missed_deadlines += 1;
        }
    }
    stats.round_trip_us.sort_unstable();
    stats.inference_us.sort_unstable();
    stats
}

/// One tick's exchange with the firmware: `sending`, if any, goes out with it. Returns what was measured.
fn read_state<S: Read + Write>(
    bridge: &mut Bridge<S>,
    exchange: Exchange,
    sending: Option<&[f64]>,
) -> Result<Sensed, BridgeError> {
    Ok(match (exchange, sending) {
        // The protocol 1 reads have no gyro and no joint speeds.
        (Exchange::TwoReads, _) => {
            let pitch_roll = firmware::status(bridge)?.pitch_roll();
            Sensed { joints: firmware::joints(bridge)?, pitch_roll, ..Sensed::default() }
        }
        (_, Some(targets)) => firmware::step(bridge, targets)?.sensed(),
        // Nothing authorised yet (or a plain read was asked for): read only.
        (_, None) => firmware::state(bridge)?.sensed(),
    })
}
