// SPDX-License-Identifier: Apache-2.0
// The safety layer. Every joint command, whatever its source, goes through
// SafetyFilter::step() before it is turned into servo steps:
//   1. non-finite values are dropped (the joint keeps its last setpoint),
//   2. per-joint clamp to the servo angle limits (beak 0..+35 included),
//   3. rate limit (max_speed_dps per joint),
//   4. head roll x head yaw interlock,
//   5. hip map per leg (hip pitch x roll x yaw x knee), stricter neighbour.
// FaultMonitor watches servo feedback and trunk tilt and latches a fault.
#pragma once
#include <stdint.h>

#include "robot_config.h"
#include "scs_bus.h"

namespace duck {

enum SafetyFlag : uint16_t {
  kFlagNonFinite = 1 << 0,
  kFlagJointClamp = 1 << 1,
  kFlagRateLimit = 1 << 2,
  kFlagHeadInterlock = 1 << 3,
  kFlagHipMapLeft = 1 << 4,
  kFlagHipMapRight = 1 << 5,
};

// Is this left-leg pose inside the measured collision-free set? Between grid
// values every surrounding grid point must be free ("stricter neighbour").
bool hip_pose_allowed(const HipMapConfig& map, float pitch, float roll, float yaw, float knee);
// Same for one leg of a full joint vector; the right leg is mirrored.
bool leg_allowed(const HipMapConfig& map, const float q[kNumJoints], bool right);

// Head interlock. Allowed states: abs(yaw) <= window and abs(roll) <= roll max,
// or abs(roll) <= roll_free at any yaw; window and roll max are reduced by
// their margins. (prev_yaw, prev_roll) is the last setpoint, (*yaw, *roll) the
// wanted one, changed in place if needed. Returns true if it changed anything.
bool head_interlock(const HeadInterlockConfig& cfg, float prev_yaw, float prev_roll, float* yaw, float* roll);

class SafetyFilter {
 public:
  explicit SafetyFilter(const RobotConfig& cfg) : cfg_(cfg) {}

  // Start from the pose the robot is actually in (measured, degrees).
  void reset(const float current[kNumJoints]);

  // checked_path = true only for the sit/stand trajectory and the SIT hold:
  // that path was checked on the exact parts on its own and lies outside the
  // general hip map, so the hip map is skipped for it.
  uint16_t step(const float target[kNumJoints], float dt_s, bool checked_path, float out[kNumJoints]);

  const float* setpoint() const { return q_; }

 private:
  const RobotConfig& cfg_;
  float q_[kNumJoints] = {0};
};

enum class Fault : uint8_t {
  kNone = 0,
  kOverload,       // load above the threshold for too long
  kOverTemp,
  kUnderVoltage,
  kServoStatus,    // the servo itself reports overload / overheat
  kComm,           // a servo stopped answering
  kFall,           // trunk tilt far from the plan
  kBusWrite,       // a goal write was refused or failed
};

class FaultMonitor {
 public:
  explicit FaultMonitor(const FaultConfig& cfg) : cfg_(cfg) { clear(); }
  void clear();
  void on_feedback(int joint, const Feedback& fb, uint32_t now_ms);
  void on_comm_fail(int joint);
  void on_tilt(float pitch_error_deg, float roll_deg, uint32_t now_ms);
  void tilt_unknown() { tilt_bad_since_valid_ = false; }
  void raise(Fault f, int joint) { if (fault_ == Fault::kNone) { fault_ = f; joint_ = joint; } }
  Fault fault() const { return fault_; }
  int fault_joint() const { return joint_; }  // -1 if not tied to a joint

 private:
  const FaultConfig& cfg_;
  Fault fault_;
  int joint_;
  uint32_t overload_since_[kNumJoints];
  bool overloaded_[kNumJoints];
  uint8_t comm_fails_[kNumJoints];
  uint32_t tilt_bad_since_;
  bool tilt_bad_since_valid_;
};

}  // namespace duck
