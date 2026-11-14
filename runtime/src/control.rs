// SPDX-License-Identifier: Apache-2.0
//! The fixed-rate loop.
//!
//! Each tick reads the robot's state from the firmware and passes a target for every chain through its rigrs
//! guard. This stage is a dry run: the target is "stay where you are". By default nothing is sent back to the
//! firmware; with [`Exchange::Step`] the guarded targets are sent, which a firmware that is not armed ignores.
//! What it measures is real: how long a round trip to the microcontroller takes and how many ticks miss their
//! deadline, which decides the control rate a policy can have.

use std::io::{Read, Write};
use std::time::{Duration, Instant};

use arm::{Guard, Verdict};

use crate::bridge::{Bridge, BridgeError};
use crate::firmware;
use crate::robot::Robot;

/// Commands older than this are stale and the guard holds (rigrs `Guard`).
const SOURCE_TIMEOUT_MS: u64 = 500;

/// What one chain's guard said on one tick.
#[derive(Debug, Clone, PartialEq)]
pub enum ChainOutcome {
    /// At least one servo in the chain did not report a position, so there is nothing to authorise against.
    Unmeasured,
    Guarded(Verdict),
}

pub struct ChainGuard {
    pub name: &'static str,
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
    /// One read, `duck_state` (protocol 2).
    State,
    /// One exchange, `duck_step`: send the previous tick's guarded targets, get the state back (protocol 2).
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
    /// Round-trip times of the calls made in each tick, in microseconds, sorted.
    pub round_trip_us: Vec<u64>,
    pub last_error: Option<String>,
}

impl LoopStats {
    pub fn percentile(&self, p: f64) -> Option<u64> {
        if self.round_trip_us.is_empty() {
            return None;
        }
        let last = self.round_trip_us.len() - 1;
        let index = ((last as f64) * p).round() as usize;
        self.round_trip_us.get(index.min(last)).copied()
    }
}

/// Run the dry-run loop for `duration` at `period`.
pub fn run<S: Read + Write>(
    bridge: &mut Bridge<S>,
    robot: &Robot,
    period: Duration,
    duration: Duration,
    exchange: Exchange,
) -> LoopStats {
    let mut guards = Guards::new(robot, period);
    let mut stats = LoopStats::default();
    let start = Instant::now();
    let mut next = start;
    // What the guards allowed on the previous tick, to send with this one. None until every chain has moved once.
    let mut allowed: Option<Vec<f64>> = None;
    while start.elapsed() < duration {
        let tick_start = Instant::now();
        let now_ms = u64::try_from(start.elapsed().as_millis()).unwrap_or(u64::MAX);
        match read_state(bridge, exchange, allowed.as_deref()) {
            Ok(measured) => {
                // The dry-run command: hold every joint where it is. A policy will supply this later.
                let wanted: Vec<f64> = measured.iter().map(|m| m.unwrap_or(0.0)).collect();
                guards.fed(now_ms);
                let mut next_targets = Vec::with_capacity(wanted.len());
                for (_, outcome) in guards.authorise(now_ms, &measured, &wanted) {
                    match outcome {
                        ChainOutcome::Unmeasured => stats.unmeasured_chain_ticks += 1,
                        ChainOutcome::Guarded(Verdict::Move { radians }) => {
                            stats.moves += 1;
                            next_targets.extend(radians.iter().copied());
                        }
                        ChainOutcome::Guarded(_) => stats.holds += 1,
                    }
                }
                // Only a full set of authorised targets is ever sent; a partial one is dropped.
                allowed = (next_targets.len() == wanted.len()).then_some(next_targets);
            }
            Err(e) => {
                stats.errors += 1;
                stats.last_error = Some(e.to_string());
            }
        }
        stats.round_trip_us.push(u64::try_from(tick_start.elapsed().as_micros()).unwrap_or(u64::MAX));
        stats.ticks += 1;
        next += period;
        match next.checked_duration_since(Instant::now()) {
            Some(wait) => std::thread::sleep(wait),
            None => {
                stats.missed_deadlines += 1;
                next = Instant::now(); // do not try to catch up: a burst of late ticks is worse than a slow one
            }
        }
    }
    stats.round_trip_us.sort_unstable();
    stats
}

/// One tick's exchange with the firmware; returns the measured joint angles.
fn read_state<S: Read + Write>(
    bridge: &mut Bridge<S>,
    exchange: Exchange,
    allowed: Option<&[f64]>,
) -> Result<Vec<Option<f64>>, BridgeError> {
    match (exchange, allowed) {
        (Exchange::TwoReads, _) => {
            firmware::status(bridge)?;
            firmware::joints(bridge)
        }
        (Exchange::Step, Some(targets)) => Ok(firmware::step(bridge, targets)?.joints),
        // Nothing authorised yet (or a plain read was asked for): read only.
        (Exchange::State, _) | (Exchange::Step, None) => Ok(firmware::state(bridge)?.joints),
    }
}
