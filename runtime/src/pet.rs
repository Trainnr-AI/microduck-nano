// SPDX-License-Identifier: Apache-2.0
//! One joint behaving like part of an animal: it looks at what the camera sees, glances about when there is
//! nothing to look at, and struggles when it is held.
//!
//! This is the robot's interaction in miniature, for one servo on the bench (`duckd pet`). Two things here are
//! meant to be kept when there is a whole robot:
//!
//! - **Being held is felt through the servo itself.** A free joint is where it was told to be, give or take its
//!   lag; a joint that stays away from its target is being held or pushed. No extra sensor is involved.
//! - **What it does about it is a small state machine**, not a policy: calm, struggling in bursts, checking
//!   whether it is still held, giving up and going along with the hand, then calm again.
//!
//! Everything is in degrees of the joint. [`Pet::tick`] is pure: the same inputs give the same outputs, so the
//! behaviour is tested without a servo.

/// A free joint is within this of its target once it has had time to get there.
pub const FREE_ERROR_DEG: f64 = 4.0;
/// A moving target is followed about this late (measured on a bare SCS0009 on 2026-11-23: some 80 ms).
pub const LAG_S: f64 = 0.10;
/// Away from the target for this long: held.
pub const HELD_AFTER_S: f64 = 0.15;
/// How fast it turns to look at something, and how far from where it started it will go.
pub const GAZE_SPEED_DPS: f64 = 150.0;
pub const RANGE_DEG: f64 = 60.0;
/// With nothing to look at for this long it glances about, by up to this much: which is also how a hand that
/// holds it still is noticed.
pub const BORED_AFTER_S: f64 = 2.5;
pub const GLANCE_DEG: f64 = 10.0;
/// A struggle: this far each way, changing side this often, for this long; then it stops to feel whether it is
/// free. After so many bursts it gives up for a while and goes where the hand takes it. The target sweeps from
/// side to side at a set speed and does not jump: a servo thrown into reverse at full drive pulled four AA cells
/// under the firmware's voltage limit on the first try (2026-11-24, 12 degrees each way, jumping).
pub const STRUGGLE_DEG: f64 = 8.0;
pub const STRUGGLE_HALF_S: f64 = 0.12;
pub const STRUGGLE_SPEED_DPS: f64 = 240.0;
pub const BURST_S: f64 = 0.7;
pub const CHECK_S: f64 = 0.35;
/// Feeling whether it is free is trying a small move: a joint held still is exactly where it was left, which
/// looks the same as free until it tries to go somewhere.
pub const CHECK_DEG: f64 = 8.0;
pub const BURSTS_BEFORE_GIVING_UP: u32 = 4;
pub const GIVEN_UP_S: f64 = 3.0;

/// What it is doing, for whoever shows it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mood {
    /// Looking at the thing the camera sees.
    Watching,
    /// Nothing to look at.
    Idle,
    /// Held, and fighting it.
    Struggling,
    /// Between bursts, trying a small move to feel whether it is free.
    Checking,
    /// Held for too long: it goes along with the hand.
    GivenUp,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum State {
    Calm,
    Struggling { since: f64, bursts: u32 },
    Checking { since: f64, bursts: u32 },
    GivenUp { since: f64 },
}

/// One joint's behaviour. Make it with the angle the joint is at; call [`Pet::tick`] every control tick.
#[derive(Debug)]
pub struct Pet {
    state: State,
    /// Where it started: the middle of its range.
    home_deg: f64,
    /// The target sent last.
    target_deg: f64,
    /// The middle of a struggle: where it was when it noticed the hold.
    centre_deg: f64,
    /// How long it has been away from its target, and how long with nothing to look at.
    away_s: f64,
    bored_s: f64,
    /// Where it is glancing, and a counter that picks the next glance.
    glance_deg: f64,
    glances: u32,
}

impl Pet {
    /// Where it tries to go to feel whether it is free: a little to one side of where it is held, the other side
    /// after the next burst, and towards home if that side is the end of its range.
    fn check_target(&self, bursts: u32, low: f64, high: f64) -> f64 {
        let side = if bursts % 2 == 1 { 1.0 } else { -1.0 };
        let there = self.centre_deg + side * CHECK_DEG;
        if (low..=high).contains(&there) {
            there
        } else {
            self.centre_deg - side * CHECK_DEG
        }
    }

    pub fn new(position_deg: f64) -> Self {
        Pet {
            state: State::Calm,
            home_deg: position_deg,
            target_deg: position_deg,
            centre_deg: position_deg,
            away_s: 0.0,
            bored_s: 0.0,
            glance_deg: 0.0,
            glances: 0,
        }
    }

    /// One tick of `dt` seconds: the joint is at `position_deg`; `look_deg` is the joint angle that would point at
    /// the thing the camera sees, if it sees it. Returns the target for the joint and what it is doing.
    pub fn tick(&mut self, dt: f64, position_deg: f64, look_deg: Option<f64>) -> (f64, Mood) {
        let (low, high) = (self.home_deg - RANGE_DEG, self.home_deg + RANGE_DEG);
        let error = (position_deg - self.target_deg).abs();
        let (next, mood) = match self.state {
            State::Calm => {
                // Where it wants to look: at the thing, or a glance away from home now and then.
                let wanted = match look_deg {
                    Some(at) => {
                        self.bored_s = 0.0;
                        at
                    }
                    None => {
                        self.bored_s += dt;
                        if self.bored_s >= BORED_AFTER_S {
                            self.bored_s = 0.0;
                            self.glances += 1;
                            // left, right, further left, home: no randomness, so a run can be repeated
                            self.glance_deg = [1.0, -1.0, 0.6, 0.0].get(self.glances as usize % 4).copied().unwrap_or(0.0) * GLANCE_DEG;
                        }
                        self.home_deg + self.glance_deg
                    }
                };
                let step = (wanted.clamp(low, high) - self.target_deg).clamp(-GAZE_SPEED_DPS * dt, GAZE_SPEED_DPS * dt);
                let target = self.target_deg + step;
                // Away from the target by more than a free joint's lag allows: someone has it.
                let allowed = FREE_ERROR_DEG + LAG_S * (step.abs() / dt.max(1e-6));
                self.away_s = if error > allowed { self.away_s + dt } else { 0.0 };
                if self.away_s >= HELD_AFTER_S {
                    self.away_s = 0.0;
                    self.centre_deg = position_deg.clamp(low, high);
                    self.state = State::Struggling { since: 0.0, bursts: 1 };
                    (self.centre_deg, Mood::Struggling)
                } else {
                    (target, if look_deg.is_some() { Mood::Watching } else { Mood::Idle })
                }
            }
            State::Struggling { since, bursts } => {
                let since = since + dt;
                if since >= BURST_S {
                    self.state = State::Checking { since: 0.0, bursts };
                    (self.check_target(bursts, low, high), Mood::Checking)
                } else {
                    self.state = State::Struggling { since, bursts };
                    let side = if ((since / STRUGGLE_HALF_S) as u64).is_multiple_of(2) { 1.0 } else { -1.0 };
                    let wanted = (self.centre_deg + side * STRUGGLE_DEG).clamp(low, high);
                    let step = STRUGGLE_SPEED_DPS * dt;
                    (self.target_deg + (wanted - self.target_deg).clamp(-step, step), Mood::Struggling)
                }
            }
            State::Checking { since, bursts } => {
                let since = since + dt;
                if since < CHECK_S {
                    self.state = State::Checking { since, bursts };
                    (self.check_target(bursts, low, high), Mood::Checking)
                } else if error <= FREE_ERROR_DEG {
                    // It got where it tried to go: free.
                    self.state = State::Calm;
                    self.bored_s = 0.0;
                    (self.target_deg, Mood::Idle)
                } else if bursts >= BURSTS_BEFORE_GIVING_UP {
                    self.state = State::GivenUp { since: 0.0 };
                    (position_deg.clamp(low, high), Mood::GivenUp)
                } else {
                    self.state = State::Struggling { since: 0.0, bursts: bursts + 1 };
                    (self.centre_deg, Mood::Struggling)
                }
            }
            State::GivenUp { since } => {
                let since = since + dt;
                if since >= GIVEN_UP_S {
                    self.state = State::Calm;
                    self.bored_s = 0.0;
                    self.glance_deg = 0.0;
                } else {
                    self.state = State::GivenUp { since };
                }
                // Wherever the hand has put it is where it is told to be: it does not push back.
                (position_deg.clamp(low, high), Mood::GivenUp)
            }
        };
        self.target_deg = next;
        (next, mood)
    }
}
