// SPDX-License-Identifier: Apache-2.0
// IMU glue: the firmware needs "which way is up"; the Linux side's policy also wants the gyro.
#pragma once

// Starts the IMU. Returns false if it is absent or the build has no IMU support.
bool imu_begin();

enum ImuFresh { kImuFreshUp = 1, kImuFreshGyro = 2 };
// Fetches what is waiting, both in the sensor's own frame:
//   up[]       unit vector pointing away from gravity;
//   gyro_dps[] angular velocity, degrees per second.
// Returns kImuFresh* bits saying which of the two is new; the other is left untouched.
int imu_poll(float up[3], float gyro_dps[3]);
