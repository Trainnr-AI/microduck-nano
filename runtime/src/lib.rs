// SPDX-License-Identifier: Apache-2.0
//! Linux-side runtime for Microduck Nano on the Arduino UNO Q.
//!
//! The microcontroller owns the servo bus, the limits and the faults (`firmware/`). This crate is the other half:
//! it talks to that firmware through the board's router, describes the robot to rigrs, and runs the fixed-rate
//! loop a walking policy will live in. The control loop is a dry run: the firmware ignores its targets unless armed,
//! and nothing here arms the robot. The one exception is `duckd bench` ([`bench`]), which moves one servo on its own
//! against a bench build of the firmware, only when asked with `--supervised`.

pub mod bench;
pub mod bridge;
pub mod control;
pub mod firmware;
pub mod limits;
pub mod policy;
pub mod record;
pub mod robot;
pub mod status_led;
