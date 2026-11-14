// SPDX-License-Identifier: Apache-2.0
// Sit / stand trajectory, the planned trunk pitch that goes with it, the IMU
// ankle balance law, and tilt from the IMU's gravity vector.
// No gait lives here: none exists yet.
#pragma once
#include <stdint.h>

#include "robot_config.h"

namespace duck {

// (1 - cos(pi u)) / 2 for u in 0..1: the easing the sit/stand was simulated with.
float ease_cos(float u);

// Straight line in joint space between two poses, eased in time.
class PoseMove {
 public:
  void start(const float from[kNumPoseJoints], const float to[kNumPoseJoints], float duration_s);
  bool active() const { return active_; }
  // Advances by dt and writes the pose; returns the eased progress 0..1.
  // active() turns false once the end is reached.
  float advance(float dt_s, float out[kNumPoseJoints]);
  void cancel() { active_ = false; }

 private:
  float from_[kNumPoseJoints], to_[kNumPoseJoints];
  float duration_ = 1.0f, t_ = 0.0f;
  bool active_ = false;
};

// Planned trunk pitch (deg, + = nose up) at sit_fraction 0 (STAND) .. 1 (SIT).
float planned_pitch_deg(const MotionConfig& cfg, float sit_fraction);

struct Tilt {
  float pitch_deg;  // + = nose up (= facing uphill)
  float roll_deg;
};
// up_sensor: unit vector pointing away from gravity, in the IMU's own frame.
Tilt tilt_from_up(const ImuConfig& cfg, const float up_sensor[3]);

// A sensor-frame vector (the gyro's rates) turned into the body frame (x forward, y left, z up) by the mounting
// matrix. Unlike tilt_from_up nothing is normalised, so the matrix must be a rotation (gen_config.py checks it).
void sensor_to_body(const ImuConfig& cfg, const float v_sensor[3], float v_body[3]);

// ankle correction = gain x dead_zone(low_pass(pitch - planned pitch)).
// Add the result to the left ankle and subtract it from the right ankle.
class BalanceLaw {
 public:
  explicit BalanceLaw(const BalanceConfig& cfg) : cfg_(cfg) {}
  void reset() { filtered_ = 0.0f; correction_ = 0.0f; }
  float update(float pitch_error_deg, float dt_s);
  float correction() const { return correction_; }

 private:
  const BalanceConfig& cfg_;
  float filtered_ = 0.0f, correction_ = 0.0f;
};

}  // namespace duck
