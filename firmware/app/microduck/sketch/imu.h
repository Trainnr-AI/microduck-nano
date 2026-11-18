// SPDX-License-Identifier: Apache-2.0
// IMU glue: the firmware needs "which way is up"; the Linux side's policy also wants the gyro.
#pragma once
#include <stdint.h>

// Starts the IMU. Returns false if it is absent or the build has no IMU support.
bool imu_begin();
// For an IMU that was not there (or not ready) at start-up: tries again, at most every two seconds, and only if
// something now answers at its address. A try that finds the sensor blocks for about half a second, so call it only
// while nothing is armed. Returns true once the IMU has started.
bool imu_retry(uint32_t now_ms);

enum ImuFresh { kImuFreshUp = 1, kImuFreshGyro = 2 };
// Fetches what is waiting, both in the sensor's own frame:
//   up[]       unit vector pointing away from gravity;
//   gyro_dps[] angular velocity, degrees per second.
// Returns kImuFresh* bits saying which of the two is new; the other is left untouched.
int imu_poll(float up[3], float gyro_dps[3]);

// For the bench: what the IMU's start found and what has arrived since.
struct ImuDiag {
  int built;          // 1 = this build has IMU support
  int ack_0x4a;       // 1 = something answers at that I2C address now, 0 = nothing, -1 = not probed (no IMU support)
  int ack_0x4b;
  int started;        // the library's start succeeded and the gravity report was accepted
  int gyro_on;        // the gyro report was accepted
  int gravity_reports;  // reports received since start
  int gyro_reports;
  int other_reports;
  int resets;         // times the sensor reported a reset after start
  int start_stage;    // where the last start attempt stopped: 0 not tried, 1 the library's begin() failed,
                      // 2 the gravity report was refused, 3 started
  int start_attempts;
  int bus_freed;      // times the firmware found the Qwiic bus held by a device and clocked it free
  int bus_last;       // the last such attempt: 0 = the lines were free, 1 = freed, 2 = still held
};
ImuDiag imu_diag();

// For the bench, when the IMU answers at its address but will not start: soft-resets the sensor and reads what it
// sends the way the library does (a 4-byte header, then the packet in 32-byte reads).
// out = [reset write accepted, reads made, milliseconds they took, then per read: the header's length word as sent
// (bit 15 = continuation; -1 = the read failed) and its channel]. Takes about 0.4 s. All zero once the IMU has
// started.
constexpr int kImuProbeReads = 20;
constexpr int kImuProbeWords = 3 + 2 * kImuProbeReads;
void imu_probe(int out[kImuProbeWords]);
