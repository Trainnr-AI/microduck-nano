// SPDX-License-Identifier: Apache-2.0
// One servo moved on its own, to measure it on a bench (system identification). Only in a bench build:
// DUCK_BENCH_MOTION 1, which also needs DUCK_ENABLE_MOTION 1; both default to 0.
//
// It arms ONE joint without the rest of the robot, so it is narrower than arming the robot, never looser where it
// matters:
//   - only that joint's servo is ever written; the bus's motion gate is open only while it is armed;
//   - that joint's own table entry must be calibrated (ID, zero, direction, limits); the others need not be;
//   - targets are clamped to the joint's limits and rate-limited to its max_speed_dps;
//   - the robot's fault checks run on that servo every tick (overload, temperature, voltage, the servo's own
//     status, no answer): any of them switches its torque off and latches;
//   - if the Linux side sends no target for the command timeout, torque goes OFF (not hold): a bench run ends by
//     going limp;
//   - after it switches off, torque-off is repeated (ten ticks, then once a second while a fault is latched), as
//     the robot's fault mode does, in case a write was lost on a bus that is failing;
//   - the robot cannot be armed while it is armed or faulted (the sketch checks), and it cannot be armed while the
//     robot is; nor do the raw bus reads run while it is armed.
#pragma once
#include <stdint.h>

#include "controller.h"
#include "robot_config.h"
#include "safety.h"
#include "scs_bus.h"

namespace duck {

enum class BenchMode : uint8_t { kOff = 0, kArmed, kFault };

struct BenchState {
  BenchMode mode;
  Fault fault;          // why it is in kFault
  int8_t joint;         // the joint armed (or last armed); -1 never
  bool read_ok;         // the last read of the servo answered
  Feedback feedback;    // that read
  uint32_t read_ms;     // when it was made (the microcontroller's clock)
  float setpoint_deg;   // the goal written last (after clamp and rate limit)
};

class BenchServo {
 public:
  BenchServo(const RobotConfig& cfg, ScsBus& bus, bool enabled);

  // Arm `joint` with the arm key. The robot must be disarmed (the caller checks).
  Result arm(uint32_t key, int joint, uint32_t now_ms);
  // Torque off and forget a latched fault (an operator's explicit act). Does nothing on the bus if it is already
  // off, so it cannot disturb an armed robot.
  void disarm();
  // The joint's target in degrees; ignored unless armed. Clamped and rate-limited in tick().
  Result set_target(float deg, uint32_t now_ms);
  // Every control tick: read the servo, check faults, move the setpoint toward the target, write it. Once off,
  // repeat the torque-off for a while, unless `robot_armed` (the bus is then the robot's).
  void tick(uint32_t now_ms, bool robot_armed);

  bool armed() const { return mode_ == BenchMode::kArmed; }
  // Off, with no fault latched: the robot may be armed.
  bool idle() const { return mode_ == BenchMode::kOff; }
  BenchState state() const;

 private:
  void stop(BenchMode next);
  bool joint_ready(int joint) const;

  const RobotConfig& cfg_;
  ScsBus& bus_;
  const bool enabled_;
  FaultMonitor faults_;
  BenchMode mode_ = BenchMode::kOff;
  int joint_ = -1;
  float target_deg_ = 0.0f;
  float setpoint_deg_ = 0.0f;
  uint32_t last_cmd_ms_ = 0;
  uint32_t last_tick_ms_ = 0;
  bool read_ok_ = false;
  Feedback fb_ = Feedback();
  uint32_t read_ms_ = 0;
  uint32_t off_ticks_ = kOffRepeatTicks;  // ticks since it switched off; nothing to repeat before the first arm
  static constexpr uint32_t kOffRepeatTicks = 10;
};

}  // namespace duck
