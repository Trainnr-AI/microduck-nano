// SPDX-License-Identifier: Apache-2.0
// BNO086 over Qwiic. UNTESTED on hardware: written against the SparkFun
// library's API, compiled, never run with a sensor attached.
// Things to confirm on the bench (README, bench checklist):
//   - the gravity report's sign (assumed: reads about +9.8 on the axis that
//     points up, like an accelerometer at rest);
//   - the report rate actually delivered at 100 Hz on a 400 kHz bus, with two reports;
//   - the gyro's sign: right-handed about the sensor's axes, as the datasheet says.
#include "imu.h"

#include <Arduino.h>

#include "duck_build_config.h"

#if DUCK_IMU_BNO08X

#include <Wire.h>

#include "SparkFun_BNO08x_Arduino_Library.h"
#include "src/duck/fmath.h"

static BNO08x g_imu;
static bool g_started = false;

// Gravity and the calibrated gyro (bias removed by the sensor), both at the control rate. Gravity decides whether the
// IMU counts as present: the firmware's own safety needs only it. A gyro that will not start just never reports, and
// the state message says "not measured".
static bool enable_reports() {
  const bool gravity = g_imu.enableGravity(DUCK_TICK_MS);
  g_imu.enableGyro(DUCK_TICK_MS);
  return gravity;
}

bool imu_begin() {
  Wire1.begin();  // the Qwiic connector is Wire1 on the UNO Q (3.3 V)
  Wire1.setClock(400000);
  g_started = g_imu.begin(DUCK_IMU_I2C_ADDRESS, Wire1) && enable_reports();
  return g_started;
}

int imu_poll(float up[3], float gyro_dps[3]) {
  if (!g_started) return 0;
  if (g_imu.wasReset()) enable_reports();  // the sensor forgets its reports after a reset
  int fresh = 0;
  // Drain what is waiting; keep the newest sample of each report.
  for (int i = 0; i < 8 && g_imu.getSensorEvent(); ++i) {
    const uint8_t id = g_imu.getSensorEventID();
    if (id == SENSOR_REPORTID_GRAVITY) {
      const float x = g_imu.getGravityX(), y = g_imu.getGravityY(), z = g_imu.getGravityZ();
      const float n = duck::sqrt_f(x * x + y * y + z * z);
      if (n < 1.0f) continue;  // not a plausible gravity vector (m/s^2)
      up[0] = x / n;
      up[1] = y / n;
      up[2] = z / n;
      fresh |= kImuFreshUp;
    } else if (id == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
      const float k = 57.2957795f;  // rad/s -> deg/s
      const float x = g_imu.getGyroX() * k, y = g_imu.getGyroY() * k, z = g_imu.getGyroZ() * k;
      if (!duck::finitef(x) || !duck::finitef(y) || !duck::finitef(z)) continue;
      gyro_dps[0] = x;
      gyro_dps[1] = y;
      gyro_dps[2] = z;
      fresh |= kImuFreshGyro;
    }
  }
  return fresh;
}

#else  // no IMU in this build

bool imu_begin() { return false; }
int imu_poll(float up[3], float gyro_dps[3]) {
  (void)up;
  (void)gyro_dps;
  return 0;
}

#endif
