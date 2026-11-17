// SPDX-License-Identifier: Apache-2.0
// The robot's state machine. Hardware-independent: it talks to the servos
// through ScsBus and is handed IMU samples, so the whole thing runs in host
// tests against simulated servos.
//
// Modes
//   DISARMED  after power-up, after disarm(). No torque, no goal writes; only
//             reads. The bus's motion gate is closed.
//   ARMED     torque on; every tick sends the safety-filtered setpoints.
//   FAULT     torque off on every servo, latched until clear_fault().
//
// Arming needs ALL of: a firmware built with motion enabled, the arm key, a
// complete and calibrated joint table, every servo answering, and (unless the
// config says otherwise) a calibrated, live IMU.
#pragma once
#include <stdint.h>

#include "motion.h"
#include "robot_config.h"
#include "safety.h"
#include "scs_bus.h"

namespace duck {

constexpr uint32_t kArmKey = 0xD0C5A4B3u;

// After torque goes off, "torque off" is broadcast again in case a write was lost on a failing bus: on each of the
// first kOffRepeatTicks ticks, then every kOffRepeatPeriod ticks (once a second) while a fault is latched.
constexpr uint32_t kOffRepeatTicks = 10;
constexpr uint32_t kOffRepeatPeriod = 100;
inline bool off_repeat_due(uint32_t ticks_since_off, bool fault_latched) {
  return ticks_since_off < kOffRepeatTicks || (fault_latched && ticks_since_off % kOffRepeatPeriod == 0);
}

enum class Mode : uint8_t { kDisarmed = 0, kArmed, kFault };

enum class Posture : uint8_t {
  kUnknown = 0,      // legs hold; only a supported stand is accepted
  kStanding,
  kSitting,
  kMovingToSit,
  kMovingToStand,
  kReturningToStand  // back to the STAND keyframe (also the supported stand)
};

enum class Result : uint8_t {
  kOk = 0,
  kMotionBuildDisabled,  // firmware built without DUCK_ENABLE_MOTION
  kBadKey,
  kNotCalibrated,        // joint table has placeholders
  kServoMissing,
  kImuRequired,
  kFaultLatched,
  kNotArmed,
  kAlreadyArmed,
  kWrongPosture,
  kNotAtStand,           // sit() needs every joint at the STAND keyframe first
  kUphill,               // facing uphill more than the limit: sit refused
  kBusy,                 // a sit/stand move is running
  kBusError,
};

struct ImuSample {
  float up[3];          // unit vector away from gravity, sensor frame
  bool valid;
  uint32_t t_ms;        // when it was measured
  float gyro_dps[3];    // angular velocity, deg/s, sensor frame (a separate report, with its own age)
  bool gyro_valid;
  uint32_t gyro_t_ms;
};

enum StatusFlag : uint16_t {
  kStatCmdTimeout = 1 << 8,     // Linux side silent: holding pose
  kStatLegCmdIgnored = 1 << 9,  // leg targets ignored in this posture
  kStatImuStale = 1 << 10,
  kStatSupported = 1 << 11,     // supported stand: balance and fall detection off
};

struct Status {
  Mode mode;
  Posture posture;
  Fault fault;
  int8_t fault_joint;
  uint16_t flags;        // SafetyFlag | StatusFlag from the last tick
  bool imu_ok;
  float pitch_deg, roll_deg;
  float balance_deg;     // current ankle correction
  bool gyro_ok;          // calibrated mounting, a gyro sample no older than imu.stale_ms
  float gyro_dps[3];     // body angular velocity, deg/s, body frame (x forward, y left, z up); 0 unless gyro_ok
};

class Controller {
 public:
  Controller(const RobotConfig& cfg, ScsBus& bus, bool motion_build_enabled);

  Result arm(uint32_t key, uint32_t now_ms, const ImuSample& imu);
  void disarm();
  Result clear_fault();

  // Joint targets in degrees, joint order as JointIndex. Leg targets are used
  // only while STANDING; head targets while STANDING or UNKNOWN (and SITTING
  // if the config allows); the beak always.
  Result set_targets(const float deg[kNumJoints], uint32_t now_ms);
  Result set_head(float neck_pitch, float head_pitch, float head_yaw, float head_roll, float beak, uint32_t now_ms);
  void heartbeat(uint32_t now_ms) { last_cmd_ms_ = now_ms; }

  // STANDING -> back to the STAND keyframe; SITTING -> stand up (3 s);
  // UNKNOWN -> only with supported = true (robot held off the floor).
  Result stand(bool supported, uint32_t now_ms);
  Result sit(uint32_t now_ms, const ImuSample& imu);

  // Call at a fixed rate (100 Hz intended).
  void tick(uint32_t now_ms, const ImuSample& imu);

  Status status() const;
  const float* setpoint_deg() const { return filter_.setpoint(); }
  const float* measured_deg() const { return measured_; }
  const Feedback& feedback(int joint) const { return servo_[joint].fb; }
  bool feedback_valid(int joint) const { return servo_[joint].ok; }

 private:
  void release();  // torque off, motion gate closed, move cancelled, posture unknown; the caller sets the mode
  void enter_fault();
  BusError write_goals(const uint16_t steps[kNumJoints]);
  bool imu_usable(const ImuSample& imu, uint32_t now_ms) const;
  bool gyro_usable(const ImuSample& imu, uint32_t now_ms) const;
  void poll_servos(uint32_t now_ms, int count);  // up to `count` assigned servos, round-robin
  bool pose_near(const float q[kNumJoints], const float pose[kNumPoseJoints], float tol, bool legs_only) const;
  float sit_fraction() const;

  const RobotConfig& cfg_;
  ScsBus& bus_;
  const bool motion_build_;
  SafetyFilter filter_;
  FaultMonitor faults_;
  BalanceLaw balance_;
  PoseMove move_;

  Mode mode_ = Mode::kDisarmed;
  Posture posture_ = Posture::kUnknown;
  bool supported_ = false;
  bool timed_out_ = false;
  float move_progress_ = 0.0f;
  float user_[kNumJoints] = {};     // last targets from the Linux side
  float nominal_[kNumJoints] = {};  // what was asked of the filter, before balance
  float measured_[kNumJoints] = {};
  ServoReading servo_[kNumJoints] = {};
  uint16_t flags_ = 0;
  Tilt tilt_ = {0.0f, 0.0f};
  bool imu_ok_ = false;
  bool gyro_ok_ = false;
  float gyro_dps_[3] = {0.0f, 0.0f, 0.0f};
  uint32_t last_tick_ms_ = 0;
  bool have_tick_ = false;
  uint32_t last_cmd_ms_ = 0;
  uint32_t fault_ticks_ = 0;
  uint32_t idle_poll_ms_ = 0;
  int poll_joint_ = 0;
};

}  // namespace duck
