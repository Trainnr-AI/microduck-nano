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
  ServoReading reading;  // the last read of the servo (the microcontroller's clock)
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
  // The joint's target in degrees; ignored unless armed. Clamped here, rate-limited toward in tick().
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

  const RobotConfig& cfg_;
  ScsBus& bus_;
  const bool enabled_;
  FaultMonitor faults_;
  BenchMode mode_ = BenchMode::kOff;
  int joint_ = -1;
  float target_deg_ = 0.0f;
  float setpoint_deg_ = 0.0f;
  uint8_t write_fails_ = 0;  // goal writes in a row that got no answer
  uint32_t last_cmd_ms_ = 0;
  uint32_t last_tick_ms_ = 0;
  ServoReading reading_ = ServoReading();
  uint32_t off_ticks_ = kOffRepeatTicks;  // ticks since it switched off; nothing to repeat before the first arm
};

// Several servos moved together on a bench, each under the rules of BenchServo: one slot per joint, each with its
// own limits, speed limit, fault checks and command timeout. They share the bus, so a stop of any one (a fault, the
// Linux side going quiet) switches every servo off and closes the bus's motion gate; the others' next goal write is
// refused and they stop too. A set is armed whole or not at all.
class BenchSet {
 public:
  BenchSet(const RobotConfig& cfg, ScsBus& bus, bool enabled);

  // Arm every joint whose bit is set in `mask` (bit j = joint j). All of them, or none: the first refusal switches
  // off the ones already armed and is returned. Needs every slot off with no fault latched.
  Result arm(uint32_t key, uint32_t mask, uint32_t now_ms);
  // Torque off, every slot, and forget latched faults.
  void disarm();
  // One target per joint, degrees; a slot that is not armed ignores its own. A non-number moves nothing but counts
  // as the Linux side talking. The worst refusal among the armed-or-faulted slots is returned.
  Result set_targets(const float* deg, uint32_t now_ms);
  // Every control tick. `bus_in_use`: the robot or another bench servo is armed.
  void tick(uint32_t now_ms, bool bus_in_use);

  bool armed() const;  // any slot armed
  bool idle() const;   // every slot off with no fault latched
  BenchState state(int joint) const { return slot_[joint].state(); }
  // The one slot worth showing: a real fault before a stop that followed another's, before an armed one.
  BenchState shown() const;

 private:
  BenchServo slot_[kNumJoints];
};

}  // namespace duck
