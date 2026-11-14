// SPDX-License-Identifier: Apache-2.0
//! `duckd`'s own status light: the RGB LED the board's Linux side drives (LED1, the "user" LED).
//!
//! The microcontroller shows the robot's state on its own LEDs (`docs/robot/status-lights.md`); this one says what
//! `duckd` is doing. The LED is three on/off kernel LEDs under `/sys/class/leds`, writable by the `arduino` user (it
//! is in the `gpiod` group). On any other machine, or if the files cannot be written, the light is simply absent:
//! a status light never stops the loop.
//!
//! Arduino App Lab also writes these LEDs when an app starts or stops; the last writer wins.

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

/// The light, or nothing if this machine has none.
pub struct StatusLed {
    brightness: Option<[PathBuf; 3]>,
    shown: Option<Colour>,
}

impl StatusLed {
    /// The board's LED1, if it is there.
    pub fn find() -> Self {
        Self::find_in(Path::new(LEDS))
    }

    /// LED1 under `leds` (a copy of `/sys/class/leds` in tests).
    pub fn find_in(leds: &Path) -> Self {
        let brightness = NAMES
            .iter()
            .map(|names| names.map(|name| leds.join(name).join("brightness")))
            .find(|paths| paths.iter().all(|p| p.exists()));
        if let Some(paths) = &brightness {
            // Nothing else should be blinking it (a trigger would fight our writes).
            for path in paths {
                if let Some(dir) = path.parent() {
                    let _ = fs::write(dir.join("trigger"), "none");
                }
            }
        }
        StatusLed { brightness, shown: None }
    }

    /// No light at all (tests, `duckd` off the board).
    pub fn none() -> Self {
        StatusLed { brightness: None, shown: None }
    }

    pub fn present(&self) -> bool {
        self.brightness.is_some()
    }

    /// Show `colour`. Writes only when it changes, so calling it every tick costs nothing. If a write fails, the
    /// light is given up on rather than retried every tick.
    pub fn show(&mut self, colour: Colour) {
        if self.shown == Some(colour) {
            return;
        }
        let Some(paths) = &self.brightness else { return };
        let ok = paths.iter().zip(colour.rgb()).all(|(path, on)| fs::write(path, if on { "1" } else { "0" }).is_ok());
        if ok {
            self.shown = Some(colour);
        } else {
            self.brightness = None;
        }
    }
}

impl Drop for StatusLed {
    /// `duckd` is not running any more: the light goes out.
    fn drop(&mut self) {
        self.show(Colour::Off);
    }
}
