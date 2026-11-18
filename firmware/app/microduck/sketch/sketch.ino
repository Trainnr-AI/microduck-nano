// SPDX-License-Identifier: Apache-2.0
// Microduck Nano - microcontroller (STM32) side.
//
// Owns everything time-critical and everything safety-related: the servo bus,
// the IMU, joint limits, interlocks, the balance law, fault handling. The
// Linux side only sends requests over the Bridge; every one of them goes
// through the same safety layer, and none of them can move a servo unless the
// firmware was built with DUCK_ENABLE_MOTION and then armed.
//
// Wiring assumed (firmware/README.md): servo adapter on D0 (RX) / D1 (TX),
// 3.3 V logic, 1 Mbaud; BNO086 on the Qwiic connector.
//
// This file holds the shared state, setup() and the control loop. The Bridge functions are in rpc*.cpp, the LEDs in
// status_display.cpp, the UART in uart_transport.h.

#include <Arduino.h>
#include <Arduino_RouterBridge.h>

#include "duck_build_config.h"
#ifndef DUCK_TICK_MS
#error "duck_build_config.h must be included before its switches are tested"
#endif
#include "duck_globals.h"
#include "imu.h"
#include "rpc.h"
#include "src/duck/robot_config_data.h"
#include "status_display.h"
#include "uart_transport.h"

namespace {
UartTransport g_uart;
uint32_t g_next_tick_ms = 0;
}  // namespace

const duck::RobotConfig& g_config = duck::kRobotConfig;
duck::ScsBus g_bus(g_uart);
duck::Controller g_ctl(g_config, g_bus, DUCK_ENABLE_MOTION != 0);
duck::BenchServo g_bench(g_config, g_bus, DUCK_ENABLE_MOTION != 0 && DUCK_BENCH_MOTION != 0);
duck::ImuSample g_imu = {{0.0f, 0.0f, 1.0f}, false, 0, {0.0f, 0.0f, 0.0f}, false, 0};
bool g_imu_present = false;
uint32_t g_tick_us_max = 0;

void setup() {
  status_display_begin();

  DUCK_SERVO_SERIAL.begin(g_config.servo.baud);

  Bridge.begin(DUCK_BRIDGE_BAUD);
  rpc_register();
  // The IMU starts after the Bridge. The IMU library prints through `Serial` when it finds the sensor, and on this
  // board `Serial` is the Bridge's monitor: printed before Bridge.begin(), the call never returned and the firmware's
  // functions never appeared on the Linux side (seen on 2026-11-18, the first start with a BNO086 attached).
  g_imu_present = imu_begin();
  g_next_tick_ms = millis();
}

void loop() {
  const uint32_t now = millis();

  // imu_poll() leaves a vector that is not fresh untouched.
  const int fresh = imu_poll(g_imu.up, g_imu.gyro_dps);
  if (fresh & kImuFreshUp) {
    g_imu.valid = true;
    g_imu.t_ms = now;
  }
  if (fresh & kImuFreshGyro) {
    g_imu.gyro_valid = true;
    g_imu.gyro_t_ms = now;
  }

  if (static_cast<int32_t>(now - g_next_tick_ms) >= 0) {
    g_next_tick_ms += DUCK_TICK_MS;
    if (static_cast<int32_t>(now - g_next_tick_ms) > 5 * DUCK_TICK_MS) g_next_tick_ms = now + DUCK_TICK_MS;
    const uint32_t t0 = micros();
    g_ctl.tick(now, g_imu);
    g_bench.tick(now, g_ctl.status().mode == duck::Mode::kArmed);
    status_display_update(now);  // counted in the worst tick: the display must not cost the control loop
    const uint32_t dt = micros() - t0;
    if (dt > g_tick_us_max) g_tick_us_max = dt;
  }
}
