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
extern duck::BenchServo g_bench;  // one servo on its own; it can only ever be armed in a bench build
extern duck::ImuSample g_imu;     // the newest IMU samples
extern bool g_imu_present;        // the IMU answered at boot
extern uint32_t g_tick_us_max;    // the longest control tick, in us, since the Linux side last read it
