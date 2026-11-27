// SPDX-License-Identifier: Apache-2.0
// The sketch's shared state, defined in sketch.ino.
#pragma once
#include <stdint.h>

#include "src/duck/bench.h"
#include "src/duck/controller.h"
#include "src/duck/robot_config.h"
#include "src/duck/scs_bus.h"

extern const duck::RobotConfig& g_config;  // duck::kRobotConfig (one copy: robot_config_data.h is included once)
extern duck::ScsBus g_bus;
extern duck::Controller g_ctl;
extern duck::BenchServo g_bench;   // one servo on its own; it can only ever be armed in a bench build
extern duck::BenchServo g_bench2;  // a second one beside it, for a bench with two servos; when either stops, both do
extern duck::BenchSet g_bench_set;  // several at once, one slot per joint, under the same rules (bench build)
// A bench servo has torque.
inline bool bench_armed() { return g_bench.armed() || g_bench2.armed() || g_bench_set.armed(); }
// No bench servo has torque and none has a fault latched.
inline bool bench_idle() { return g_bench.idle() && g_bench2.idle() && g_bench_set.idle(); }
// Torque off and latched faults forgotten, all of them.
inline void bench_disarm() {
  g_bench.disarm();
  g_bench2.disarm();
  g_bench_set.disarm();
}
extern duck::ImuSample g_imu;     // the newest IMU samples
extern bool g_imu_present;        // the IMU answered at boot
extern uint32_t g_tick_us_max;    // the longest control tick, in us, since the Linux side last read it
