// SPDX-License-Identifier: Apache-2.0
// BNO086 over Qwiic. UNTESTED on hardware: written against the SparkFun
// library's API, compiled, never run with a sensor attached.
// Things to confirm on the bench (README, bench checklist):
//   - the gravity report's sign (assumed: reads about +9.8 on the axis that
//     points up, like an accelerometer at rest);
//   - the report rate actually delivered at 100 Hz on a 400 kHz bus.
#include "imu.h"

#include <Arduino.h>

#include "duck_build_config.h"

#if DUCK_IMU_BNO08X

#include <Wire.h>

#include "SparkFun_BNO08x_Arduino_Library.h"
#include "src/duck/fmath.h"

static BNO08x g_imu;
static bool g_started = false;

static bool enable_reports() { return g_imu.enableGravity(DUCK_TICK_MS); }

bool imu_begin() {
  Wire1.begin();  // the Qwiic connector is Wire1 on the UNO Q (3.3 V)
  Wire1.setClock(400000);
  g_started = g_imu.begin(DUCK_IMU_I2C_ADDRESS, Wire1) && enable_reports();
  return g_started;
}

bool imu_poll(float up[3]) {
  if (!g_started) return false;
  if (g_imu.wasReset()) enable_reports();  // the sensor forgets its reports after a reset
  bool fresh = false;
  // Drain what is waiting; keep the newest gravity sample.
  for (int i = 0; i < 4 && g_imu.getSensorEvent(); ++i) {
    if (g_imu.getSensorEventID() != SENSOR_REPORTID_GRAVITY) continue;
    const float x = g_imu.getGravityX(), y = g_imu.getGravityY(), z = g_imu.getGravityZ();
    const float n = duck::sqrt_f(x * x + y * y + z * z);
    if (n < 1.0f) continue;  // not a plausible gravity vector (m/s^2)
    up[0] = x / n;
    up[1] = y / n;
    up[2] = z / n;
    fresh = true;
  }
  return fresh;
}

#else  // no IMU in this build

bool imu_begin() { return false; }
bool imu_poll(float up[3]) {
  (void)up;
  return false;
}

#endif
