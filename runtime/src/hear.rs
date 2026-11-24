// SPDX-License-Identifier: Apache-2.0
//! Hearing: a microphone's samples turned into the few things the robot acts on.
//!
//! The walking policy does not hear: it was trained in a simulation with no sound. Sound picks what the policy is
//! asked to do instead (`duckd run --listen`: two claps start and stop the walk), so this module ends in events,
//! not in policy inputs.
//!
//! What is here: loudness per 10 ms frame, a running estimate of the room's noise, claps (a sudden loud burst
//! that is over within [`CLAP_LONGEST`]) and groups of claps. The samples come from ALSA's `arecord` as a child
//! process (16-bit mono at 16 kHz, [`arecord`]), so nothing here links a sound library and any `Read` of such
//! samples can be listened to in a test.
//!
//! NOT TRIED WITH A MICROPHONE: the robot has none yet (its camera turned out to be video only). Every threshold
//! below is a starting value chosen on synthetic sound; the servos' own noise next to the microphone is the real
//! test and has not been made.

use std::io::{self, Read};
use std::process::{Child, ChildStdout, Command, Stdio};
use std::time::Duration;

/// Samples a second asked of the microphone.
pub const RATE: u32 = 16_000;
/// Samples in one frame: 10 ms.
pub const FRAME: usize = 160;
/// One frame's length.
pub const FRAME_TIME: Duration = Duration::from_millis(10);
/// The quietest level reported (16 bits hold about 96 dB).
pub const SILENCE_DB: f64 = -96.0;

/// A burst must rise this far above the room's noise to start a clap, and be at least this loud outright.
pub const CLAP_OVER_NOISE_DB: f64 = 20.0;
pub const CLAP_LOUDEST_BELOW_DB: f64 = -30.0;
/// It ends once it has fallen this far below its own peak.
pub const CLAP_DECAY_DB: f64 = 12.0;
/// A burst that lasts longer is not a clap (speech, a servo, something dragged).
pub const CLAP_LONGEST: Duration = Duration::from_millis(120);
/// After a burst, this long is ignored: a clap's echo is not a second clap.
pub const CLAP_ECHO: Duration = Duration::from_millis(80);
/// Claps closer together than this are one group; the group is reported once this long has passed without another.
pub const GROUP_GAP: Duration = Duration::from_millis(600);

/// The loudness of a frame: its RMS against full scale, in decibels (0 is the loudest a sample can be).
pub fn level_db(frame: &[i16]) -> f64 {
    if frame.is_empty() {
        return SILENCE_DB;
    }
    let power = frame.iter().map(|&s| f64::from(s) * f64::from(s)).sum::<f64>() / frame.len() as f64;
    let rms = power.sqrt() / f64::from(i16::MAX);
    if rms <= 0.0 {
        SILENCE_DB
    } else {
        (20.0 * rms.log10()).max(SILENCE_DB)
    }
}

/// What was heard, reported on the frame that settles it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Heard {
    /// A group of this many claps ended ([`GROUP_GAP`] of no further clap).
    Claps(u32),
}

/// Frames in a duration, at least one.
fn frames(d: Duration) -> u32 {
    u32::try_from((d.as_millis() / FRAME_TIME.as_millis()).max(1)).unwrap_or(u32::MAX)
}

/// Finds claps in a stream of frame levels and groups them.
#[derive(Debug)]
pub struct Ears {
    /// The room's noise: follows a falling level quickly and a rising one slowly, so a clap does not lift it.
    noise_db: Option<f64>,
    /// A burst in progress: its peak and how many frames it has lasted.
    burst: Option<(f64, u32)>,
    /// Frames still ignored after a burst.
    deaf: u32,
    /// Claps in the group being collected and frames since its last one.
    claps: u32,
    since_clap: u32,
}

impl Default for Ears {
    fn default() -> Self {
        Self::new()
    }
}

impl Ears {
    pub fn new() -> Self {
        Ears { noise_db: None, burst: None, deaf: 0, claps: 0, since_clap: 0 }
    }

    /// The room's noise as estimated so far, if a frame has been heard.
    pub fn noise_db(&self) -> Option<f64> {
        self.noise_db
    }

    /// Take the next frame's level ([`level_db`]). Call once per frame, in order.
    pub fn frame(&mut self, level: f64) -> Option<Heard> {
        let noise = *self.noise_db.get_or_insert(level);
        if self.deaf > 0 {
            self.deaf -= 1;
        } else if let Some((peak, lasted)) = self.burst {
            let peak = peak.max(level);
            if level <= peak - CLAP_DECAY_DB || level <= noise + CLAP_OVER_NOISE_DB / 2.0 {
                // Over. Short enough: a clap.
                if lasted <= frames(CLAP_LONGEST) {
                    self.claps += 1;
                    self.since_clap = 0;
                }
                self.burst = None;
                self.deaf = frames(CLAP_ECHO);
            } else {
                self.burst = Some((peak, lasted.saturating_add(1)));
            }
        } else if level >= noise + CLAP_OVER_NOISE_DB && level >= CLAP_LOUDEST_BELOW_DB {
            self.burst = Some((level, 1));
        }
        // The noise estimate does not learn from a burst while it could still be a clap; a sound that outlasts one
        // is the room's noise now (a servo running, someone talking) and lifts it.
        let learning = match self.burst {
            Some((_, lasted)) => lasted > frames(CLAP_LONGEST),
            None => self.deaf == 0,
        };
        if learning {
            let rate = if level < noise { 0.2 } else { 0.01 };
            self.noise_db = Some(noise + rate * (level - noise));
        }
        if self.claps > 0 {
            self.since_clap = self.since_clap.saturating_add(1);
            if self.since_clap > frames(GROUP_GAP) && self.burst.is_none() {
                let n = std::mem::take(&mut self.claps);
                return Some(Heard::Claps(n));
            }
        }
        None
    }
}

/// A microphone being recorded by `arecord`. Dropping it stops the recording.
pub struct Microphone {
    child: Child,
    samples: ChildStdout,
}

/// Start recording from the ALSA capture device `device` (as `arecord -L` names it, for example `plughw:1,0`).
pub fn arecord(device: &str) -> io::Result<Microphone> {
    let mut child = Command::new("arecord")
        .args(["-q", "-D", device, "-f", "S16_LE", "-c", "1", "-t", "raw", "-r"])
        .arg(RATE.to_string())
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .map_err(|e| io::Error::new(e.kind(), format!("could not start arecord (ALSA's recorder): {e}")))?;
    match child.stdout.take() {
        Some(samples) => Ok(Microphone { child, samples }),
        None => {
            let _ = child.kill();
            Err(io::Error::other("arecord gave no output to read"))
        }
    }
}

impl Read for Microphone {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.samples.read(buf)
    }
}

impl Drop for Microphone {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

/// Read the next frame of 16-bit little-endian samples. `Ok(false)`: the stream ended.
pub fn read_frame(from: &mut impl Read, frame: &mut [i16; FRAME]) -> io::Result<bool> {
    let mut bytes = [0u8; FRAME * 2];
    match from.read_exact(&mut bytes) {
        Ok(()) => {}
        Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(false),
        Err(e) => return Err(e),
    }
    for (sample, pair) in frame.iter_mut().zip(bytes.as_chunks::<2>().0) {
        *sample = i16::from_le_bytes(*pair);
    }
    Ok(true)
}

/// Listen to `from` until it ends or `each` returns false. `each` gets every frame's level and what it settled.
pub fn listen(from: &mut impl Read, mut each: impl FnMut(f64, &Ears, Option<Heard>) -> bool) -> io::Result<()> {
    let mut ears = Ears::new();
    let mut frame = [0i16; FRAME];
    while read_frame(from, &mut frame)? {
        let level = level_db(&frame);
        let heard = ears.frame(level);
        if !each(level, &ears, heard) {
            break;
        }
    }
    Ok(())
}
