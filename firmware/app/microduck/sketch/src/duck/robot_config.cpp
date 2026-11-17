// SPDX-License-Identifier: Apache-2.0
#include "robot_config.h"

#include "fmath.h"

namespace duck {

bool joint_ready(const RobotConfig& cfg, int joint) {
  if (joint < 0 || joint >= kNumJoints) return false;
  const JointConfig& j = cfg.joints[joint];
  return j.calibrated && j.servo_id <= 253 && (j.direction == 1 || j.direction == -1) && j.min_deg <= j.max_deg &&
         cfg.servo.steps_per_deg > 0.0f;
}

bool config_ready_for_motion(const RobotConfig& cfg) {
  for (int i = 0; i < kNumJoints; ++i) {
    if (!joint_ready(cfg, i)) return false;
    for (int k = 0; k < i; ++k) {
      if (cfg.joints[k].servo_id == cfg.joints[i].servo_id) return false;
    }
  }
  return true;
}

int16_t deg_to_step(const RobotConfig& cfg, int joint, float deg) {
  const JointConfig& j = cfg.joints[joint];
  if (!(deg == deg)) deg = 0.0f;  // NaN
  deg = clampf(deg, j.min_deg, j.max_deg);  // min <= max: gen_config.py and joint_ready() check it
  long step = j.zero_step + j.direction * round_to_long(deg * cfg.servo.steps_per_deg);
  if (step < cfg.servo.step_min) step = cfg.servo.step_min;
  if (step > cfg.servo.step_max) step = cfg.servo.step_max;
  return static_cast<int16_t>(step);
}

float step_to_deg(const RobotConfig& cfg, int joint, int16_t step) {
  const JointConfig& j = cfg.joints[joint];
  return static_cast<float>(j.direction * (step - j.zero_step)) / cfg.servo.steps_per_deg;
}

float speed_to_dps(const RobotConfig& cfg, int joint, int16_t speed) {
  return static_cast<float>(cfg.joints[joint].direction * speed) / cfg.servo.steps_per_deg;
}

}  // namespace duck
