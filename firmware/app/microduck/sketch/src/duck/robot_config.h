// SPDX-License-Identifier: Apache-2.0
// Robot description used by every other module. The values live in
// firmware/config/robot.json and reach the firmware through the generated
// robot_config_data.h (tools/gen_config.py). Nothing here depends on Arduino.
#pragma once
#include <stdint.h>

namespace duck {

constexpr int kNumJoints = 15;      // 14 joints + beak
constexpr int kNumPoseJoints = 14;  // joints that belong to a pose (no beak)

// Joint order = MJCF actuator order, beak last.
enum JointIndex : uint8_t {
  kLHipYaw = 0, kLHipRoll, kLHipPitch, kLKnee, kLAnkle,
  kNeckPitch, kHeadPitch, kHeadYaw, kHeadRoll,
  kRHipYaw, kRHipRoll, kRHipPitch, kRKnee, kRAnkle,
  kBeak
};

constexpr uint8_t kServoIdUnassigned = 0xFF;

struct ServoConfig {
  float steps_per_deg;      // UNCONFIRMED on a real servo
  int16_t step_min, step_max;
  uint32_t baud;
  uint16_t goal_speed;      // steps/s written with every goal; meaning of 0 UNCONFIRMED
};

// Joint names are not stored here: they live in config/robot.json, and the status display has its own short ones
// (display.cpp).
struct JointConfig {
  uint8_t servo_id;         // kServoIdUnassigned until decided
  int16_t zero_step;        // servo step at the joint's zero
  int8_t direction;         // +1 / -1
  bool calibrated;          // false = placeholder zero / direction: arming is refused
  float min_deg, max_deg;
  float max_speed_dps;
};

struct HeadInterlockConfig {
  float yaw_window_deg, yaw_margin_deg;
  float roll_max_deg, roll_margin_deg;
  float roll_free_deg;
};

constexpr int kHipPitchN = 13, kHipRollN = 8, kHipYawN = 3, kHipKneeN = 4;
struct HipMapConfig {       // left-leg signs; the right leg is the mirror image
  float pitch[kHipPitchN], roll[kHipRollN], yaw[kHipYawN], knee[kHipKneeN];
  uint8_t collides[kHipPitchN][kHipYawN][kHipKneeN];  // bit i set = collides at roll[i]
};

constexpr int kPlannedPitchN = 9;
struct MotionConfig {
  float sit_stand_s, return_to_stand_s;
  float pose_tol_deg, posture_detect_tol_deg;
  float planned_pitch_deg[kPlannedPitchN];  // STAND (first) .. SIT (last)
  float sit_uphill_limit_deg;
  bool head_motion_while_sitting;
};

struct BalanceConfig {
  float ankle_gain, lowpass_ms, dead_zone_deg, max_correction_deg;
};

struct ImuConfig {
  float sensor_to_body[3][3];
  bool calibrated;
  uint32_t stale_ms;
  bool require_to_arm;
};

struct FaultConfig {
  float overload_load_pct, overload_s;
  float over_temp_c, under_voltage_v;
  uint8_t comm_fail_count;
  float fall_pitch_error_deg, fall_roll_deg, fall_s;
};

struct ProtectionConfig {   // servo EEPROM values, written only by a provisioning build
  uint8_t unload_condition, max_temperature_c, overload_torque_pct, protection_time_10ms, holding_torque_pct;
};

struct RobotConfig {
  ServoConfig servo;
  JointConfig joints[kNumJoints];
  float stand[kNumPoseJoints];
  float sit[kNumPoseJoints];
  HeadInterlockConfig head;
  HipMapConfig hip;
  MotionConfig motion;
  BalanceConfig balance;
  ImuConfig imu;
  FaultConfig faults;
  uint32_t cmd_timeout_ms;
  ProtectionConfig protection;
};

// True when this joint can be driven: marked calibrated, a servo ID (0..253), a direction of +-1, min <= max, and a
// servo step size above zero.
bool joint_ready(const RobotConfig& cfg, int joint);
// True when every joint is ready and no two share a servo ID.
bool config_ready_for_motion(const RobotConfig& cfg);

// Joint angle <-> servo step. deg_to_step clamps to the joint's limits and to
// the servo's step range.
int16_t deg_to_step(const RobotConfig& cfg, int joint, float deg);
float step_to_deg(const RobotConfig& cfg, int joint, int16_t step);

}  // namespace duck
