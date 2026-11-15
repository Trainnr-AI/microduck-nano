// SPDX-License-Identifier: Apache-2.0
//! `duckd`'s own status light: the RGB LED the board's Linux side drives (LED1, the "user" LED).
//!
//! The microcontroller shows the robot's state on its own LEDs (`docs/robot/status-lights.md`); this one says what
//! `duckd` is doing. The LED is three on/off kernel LEDs under `/sys/class/leds`, writable by the `arduino` user (it
//! is in the `gpiod` group). On any other machine, or if the files cannot be written, the light is simply absent:
//! a status light never stops the loop.
//!
//! While `duckd` holds the light, each colour's kernel trigger is set to `none` so nothing else blinks it; when it
//! lets go (dropped, or a write failed) the light is switched off and the triggers it found are put back. Arduino App
//! Lab also writes this LED when an app starts or stops; the last writer wins.

use std::fs;
use std::path::{Path, PathBuf};

/// Where the kernel's LEDs are.
pub const LEDS: &str = "/sys/class/leds";
/// The names LED1's three colours have had on UNO Q images: the board's (`unoq:user-red1`) and Arduino's
/// documentation's (`red:user`).
const NAMES: [[&str; 3]; 2] = [["unoq:user-red1", "unoq:user-green1", "unoq:user-blue1"], ["red:user", "green:user", "blue:user"]];

/// What the light shows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Colour {
    Off,
    Red,
    Green,
    Yellow,
    Blue,
    Magenta,
    Cyan,
    White,
}

impl Colour {
    fn rgb(self) -> [bool; 3] {
        match self {
            Colour::Off => [false, false, false],
            Colour::Red => [true, false, false],
            Colour::Green => [false, true, false],
            Colour::Yellow => [true, true, false],
            Colour::Blue => [false, false, true],
            Colour::Magenta => [true, false, true],
            Colour::Cyan => [false, true, true],
            Colour::White => [true, true, true],
        }
    }
}

/// One colour of the LED: its directory and the trigger it had before `duckd` took it.
struct Channel {
    dir: PathBuf,
    trigger: Option<String>,
}

impl Channel {
    fn set(&self, on: bool) -> bool {
        fs::write(self.dir.join("brightness"), if on { "1" } else { "0" }).is_ok()
    }
}

/// The active trigger in a `trigger` file's text: the word in brackets (`none [heartbeat] timer`).
fn active_trigger(text: &str) -> Option<String> {
    text.split_whitespace().find_map(|w| w.strip_prefix('[')?.strip_suffix(']').map(str::to_string))
}

/// The light, or nothing if this machine has none.
pub struct StatusLed {
    channels: Option<[Channel; 3]>,
    shown: Option<Colour>,
}

impl StatusLed {
    /// The board's LED1, if it is there and can be written.
    pub fn find() -> Self {
        Self::find_in(Path::new(LEDS))
    }

    /// LED1 under `leds` (a copy of `/sys/class/leds` in tests). It is taken (triggers off) and switched off; if any
    /// of that cannot be written, it is given back and the light is absent.
    pub fn find_in(leds: &Path) -> Self {
        let found = NAMES.iter().map(|names| names.map(|name| leds.join(name))).find(|dirs| dirs.iter().all(|d| d.join("brightness").exists()));
        let Some(dirs) = found else { return Self::none() };
        let channels = dirs.map(|dir| {
            let trigger = fs::read_to_string(dir.join("trigger")).ok().as_deref().and_then(active_trigger);
            Channel { dir, trigger }
        });
        let mut led = StatusLed { channels: Some(channels), shown: None };
        let taken = led.channels.as_ref().is_some_and(|channels| {
            channels.iter().all(|c| c.trigger.is_none() || fs::write(c.dir.join("trigger"), "none").is_ok())
                && channels.iter().all(|c| c.set(false))
        });
        if taken {
            led.shown = Some(Colour::Off);
        } else {
            led.release();
        }
        led
    }

    /// No light at all (tests, `duckd` off the board).
    pub fn none() -> Self {
        StatusLed { channels: None, shown: None }
    }

    pub fn present(&self) -> bool {
        self.channels.is_some()
    }

    /// Show `colour`. Writes only when it changes, so calling it every tick costs nothing. If a write fails, the light
    /// is switched off as far as it can be and given back, rather than left half lit or retried every tick.
    pub fn show(&mut self, colour: Colour) {
        if self.shown == Some(colour) {
            return;
        }
        let Some(channels) = &self.channels else { return };
        if channels.iter().zip(colour.rgb()).all(|(c, on)| c.set(on)) {
            self.shown = Some(colour);
        } else {
            self.release();
        }
    }

    /// Off, the triggers put back, and no more writes.
    fn release(&mut self) {
        if let Some(channels) = self.channels.take() {
            for c in &channels {
                let _ = c.set(false);
                if let Some(trigger) = &c.trigger {
                    let _ = fs::write(c.dir.join("trigger"), trigger);
                }
            }
        }
        self.shown = None;
    }
}

impl Drop for StatusLed {
    /// `duckd` is not running a loop any more: the light goes out and is given back.
    fn drop(&mut self) {
        self.release();
    }
}
