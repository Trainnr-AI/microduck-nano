// SPDX-License-Identifier: Apache-2.0
// IMU glue: the rest of the firmware only needs "which way is up".
#pragma once

// Starts the IMU. Returns false if it is absent or the build has no IMU support.
bool imu_begin();
// Fetches a new gravity sample if one is waiting: up[] = unit vector pointing
// away from gravity, in the sensor's own frame. Returns true when up[] is new.
bool imu_poll(float up[3]);
