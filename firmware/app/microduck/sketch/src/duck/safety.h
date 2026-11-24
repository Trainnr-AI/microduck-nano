// SPDX-License-Identifier: Apache-2.0
// The safety layer. Every joint command, whatever its source, goes through
// SafetyFilter::step() before it is turned into servo steps:
//   1. non-finite values are dropped (the joint keeps its last setpoint),
//   2. per-joint clamp to the servo angle limits (beak 0..+35 included),
//   3. rate limit (max_speed_dps per joint),
//   4. head roll x head yaw interlock,
//   5. each leg's own hip map (hip pitch x roll x yaw x knee x ankle), stricter neighbour, and the knee x ankle rule,
//   6. the both-legs rule.
// 5 and 6 are skipped on the checked sit / stand path.
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
  kFlagLegLeft = 1 << 4,    // a rule of the left leg (hip map or knee x ankle) held joints of that leg
  kFlagLegRight = 1 << 5,
  kFlagKneeAnkle = 1 << 6,  // ...and the knee x ankle rule was one of the reasons
  kFlagBothLegs = 1 << 7,   // the both-legs rule held both legs
};

// Steps 2 and 3 for one joint, each setting its flag in *flags when it changes something. They are separate so the
// bench can clamp a target when it arrives and rate-limit toward it every tick; limit_joint() is both at once.
float clamp_joint(const JointConfig& j, float deg, uint16_t* flags);
// setpoint moved toward target by at most max_speed_dps x dt_s.
float rate_limit(const JointConfig& j, float setpoint, float target, float dt_s, uint16_t* flags);
inline float limit_joint(const JointConfig& j, float setpoint, float target, float dt_s, uint16_t* flags) {
  return rate_limit(j, setpoint, clamp_joint(j, target, flags), dt_s, flags);
}

// One leg's joints in the LEFT leg's signs, which every leg rule is written in.
struct LegPose { float yaw, roll, pitch, knee, ankle; };
LegPose leg_pose(const float q[kNumJoints], bool right);

// Is this hip pose (left signs) inside that leg's measured collision-free set? Between grid values every
// surrounding grid point must be free ("stricter neighbour"); beyond an axis's end the end value is used.
bool hip_pose_allowed(const HipMapConfig& map, bool right, float pitch, float roll, float yaw, float knee,
                      float ankle);
// Knee x ankle (left signs): with the knee bent past the threshold the ankle must not drop below the row's value.
bool knee_ankle_allowed(const KneeAnkleConfig& c, float knee, float ankle);
// Both of the above for one leg of a full joint vector.
bool leg_allowed(const RobotConfig& cfg, const float q[kNumJoints], bool right);
// Both legs together: not both forward while both hips are toed out.
bool both_legs_allowed(const BothLegsConfig& c, const float q[kNumJoints]);
// Rules 5 and 6 on a full joint vector: turns `next` into an allowed pose, given the allowed pose `prev` the robot
// is in. Each leg keeps as much of its move as its own rules allow (all of it; else with the ankle held; else the
// ankle alone; else it stays); if the two legs together then break the both-legs rule, both stay. Returns the
// kFlagLeg* / kFlagKneeAnkle / kFlagBothLegs bits. The Rust runtime has the same function (runtime/limits) and
// the two are compared on shared vectors (tests: --leg-vectors).
uint16_t settle_legs(const RobotConfig& cfg, const float prev[kNumJoints], float next[kNumJoints]);

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
  // general hip map, so the leg rules (hip maps, knee x ankle, both legs) are skipped for it.
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
  kPositionJump,   // a servo's position changed faster than the servo can move
  kImuLost,        // the IMU stopped reporting while the robot was balancing on it
};
// How many there are, for the tables that name them. Move it with the last one.
constexpr int kFaultCount = static_cast<int>(Fault::kImuLost) + 1;

// "This condition has been true for `seconds`". The first true sample starts the clock and never trips on its own,
// even with a zero duration; a false sample stops it.
class Held {
 public:
  bool update(bool condition, uint32_t now_ms, float seconds) {
    if (!condition) {
      on_ = false;
      return false;
    }
    if (!on_) {
      on_ = true;
      since_ms_ = now_ms;
      return false;
    }
    return static_cast<float>(now_ms - since_ms_) >= seconds * 1000.0f;
  }
  void reset() { on_ = false; }

 private:
  uint32_t since_ms_ = 0;
  bool on_ = false;
};

class FaultMonitor {
 public:
  explicit FaultMonitor(const FaultConfig& cfg) : cfg_(cfg) { clear(); }
  void clear();
  void on_feedback(int joint, const Feedback& fb, uint32_t now_ms);
  void on_comm_fail(int joint);
  void on_tilt(float pitch_error_deg, float roll_deg, uint32_t now_ms);
  void tilt_unknown() { tilt_bad_.reset(); }
  void raise(Fault f, int joint) { if (fault_ == Fault::kNone) { fault_ = f; joint_ = joint; } }
  Fault fault() const { return fault_; }
  int fault_joint() const { return joint_; }  // -1 if not tied to a joint

 private:
  const FaultConfig& cfg_;
  Fault fault_;
  int joint_;
  Held overloaded_[kNumJoints];
  uint8_t comm_fails_[kNumJoints];
  // The last position read from each joint's servo and when, for the position-jump check.
  int16_t last_position_[kNumJoints];
  uint32_t last_position_ms_[kNumJoints];
  bool have_position_[kNumJoints];
  Held tilt_bad_;
};

}  // namespace duck
