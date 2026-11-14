// SPDX-License-Identifier: Apache-2.0
// Build switches. The defaults make a firmware that CANNOT move a servo.
#pragma once

// 0 = bench-safe build: torque is never enabled and no goal position is ever
//     sent, whatever the Linux side asks. Reads, pings and telemetry work.
// 1 = motion build: arming becomes possible (it still needs the arm key, a
//     calibrated joint table in config/robot.json, every servo answering and
//     a calibrated IMU). Set this only for a supervised bench session.
#ifndef DUCK_ENABLE_MOTION
#define DUCK_ENABLE_MOTION 0
#endif

// 1 = provisioning build: adds the commands that write servo EEPROM (angle
//     limits, protection settings, servo ID). One servo on the bus at a time
//     for ID changes. Never leave this on in the robot's normal firmware.
#ifndef DUCK_ENABLE_EEPROM_WRITE
#define DUCK_ENABLE_EEPROM_WRITE 0
#endif

// 1 = read the BNO086 over Qwiic (Wire1, address 0x4B) with the SparkFun
//     BNO08x library ("SparkFun BNO08x Cortex Based IMU"; add it to
//     sketch.yaml). UNTESTED on hardware. 0 = no IMU: arming is refused
//     unless config/robot.json says the IMU is not required.
#ifndef DUCK_IMU_BNO08X
#define DUCK_IMU_BNO08X 0
#endif
#define DUCK_IMU_I2C_ADDRESS 0x4B

// Speed of the link between the microcontroller and the Linux side (the Bridge), in baud. The board's router
// service uses 115200 unless it has been changed with tools/set-link-speed.sh; the two must match or the
// firmware's functions never appear on the Linux side.
#ifndef DUCK_BRIDGE_BAUD
#define DUCK_BRIDGE_BAUD 115200
#endif

// Control loop period.
#define DUCK_TICK_MS 10
