// SPDX-License-Identifier: Apache-2.0
#include "robot_config.h"

#include "fmath.h"

namespace duck {

bool config_ready_for_motion(const RobotConfig& cfg) {
  for (int i = 0; i < kNumJoints; ++i) {
    const JointConfig& j = cfg.joints[i];
    if (!j.calibrated || j.servo_id == kServoIdUnassigned || j.servo_id > 253) return false;
    if (j.direction != 1 && j.direction != -1) return false;
    if (!(j.min_deg <= j.max_deg)) return false;
    for (int k = 0; k < i; ++k) {
      if (cfg.joints[k].servo_id == j.servo_id) return false;
    }
  }
  return cfg.servo.steps_per_deg > 0.0f;
}

int16_t deg_to_step(const RobotConfig& cfg, int joint, float deg) {
  const JointConfig& j = cfg.joints[joint];
  if (!(deg == deg)) deg = 0.0f;  // NaN
  if (deg < j.min_deg) deg = j.min_deg;
  if (deg > j.max_deg) deg = j.max_deg;
  long step = j.zero_step + j.direction * round_to_long(deg * cfg.servo.steps_per_deg);
  if (step < cfg.servo.step_min) step = cfg.servo.step_min;
  if (step > cfg.servo.step_max) step = cfg.servo.step_max;
  return static_cast<int16_t>(step);
}

float step_to_deg(const RobotConfig& cfg, int joint, int16_t step) {
  const JointConfig& j = cfg.joints[joint];
  return static_cast<float>(j.direction * (step - j.zero_step)) / cfg.servo.steps_per_deg;
}

}  // namespace duck
