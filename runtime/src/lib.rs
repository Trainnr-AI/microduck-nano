// SPDX-License-Identifier: Apache-2.0
//! Linux-side runtime for Microduck Nano on the Arduino UNO Q.
//!
//! The microcontroller owns the servo bus, the limits and the faults (`firmware/`). This crate is the other half:
//! it talks to that firmware through the board's router, describes the robot to rigrs, and runs the fixed-rate
//! loop a walking policy will live in. At this stage it only reads; it has no call that can move a servo.

pub mod bridge;
pub mod control;
pub mod firmware;
pub mod policy;
pub mod record;
pub mod robot;
pub mod status_led;
