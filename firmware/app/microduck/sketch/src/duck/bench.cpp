// SPDX-License-Identifier: Apache-2.0
#include "bench.h"

#include "fmath.h"

namespace duck {

BenchServo::BenchServo(const RobotConfig& cfg, ScsBus& bus, bool enabled)
    : cfg_(cfg), bus_(bus), enabled_(enabled), faults_(cfg.faults) {}

bool BenchServo::joint_ready(int joint) const {
  if (joint < 0 || joint >= kNumJoints) return false;
  const JointConfig& j = cfg_.joints[joint];
  return j.calibrated && j.servo_id != kServoIdUnassigned && j.servo_id <= 253 &&
         (j.direction == 1 || j.direction == -1) && j.min_deg <= j.max_deg && j.max_speed_dps > 0.0f &&
         cfg_.servo.steps_per_deg > 0.0f;
}

Result BenchServo::arm(uint32_t key, int joint, uint32_t now_ms) {
  if (mode_ == BenchMode::kFault) return Result::kFaultLatched;
  if (mode_ == BenchMode::kArmed) return Result::kAlreadyArmed;
  if (!enabled_) return Result::kMotionBuildDisabled;
  if (key != kArmKey) return Result::kBadKey;
  if (!joint_ready(joint)) return Result::kNotCalibrated;
  const uint8_t id = cfg_.joints[joint].servo_id;

  faults_.clear();
  Feedback fb;
  if (bus_.read_feedback(id, &fb) != BusError::kOk) return Result::kServoMissing;
  faults_.on_feedback(joint, fb, now_ms);
  joint_ = joint;
  fb_ = fb;
  read_ok_ = true;
  read_ms_ = now_ms;
  if (faults_.fault() != Fault::kNone) {
    stop(BenchMode::kFault);
    return Result::kFaultLatched;
  }

  // Start where the servo is, so nothing jumps; then torque on.
  setpoint_deg_ = target_deg_ = step_to_deg(cfg_, joint, fb.position);
  bus_.allow_motion(true);
  BusError e = bus_.write_goal(id, static_cast<uint16_t>(fb.position), 0, cfg_.servo.goal_speed);
  if (e == BusError::kOk) e = bus_.torque_on(id);
  if (e != BusError::kOk) {
    stop(BenchMode::kOff);
    return Result::kBusError;
  }
  mode_ = BenchMode::kArmed;
  last_cmd_ms_ = last_tick_ms_ = now_ms;
  return Result::kOk;
}

void BenchServo::stop(BenchMode next) {
  if (joint_ >= 0) bus_.torque_off(cfg_.joints[joint_].servo_id);
  bus_.torque_off_all();  // and the broadcast, in case the addressed one was lost
  bus_.allow_motion(false);
  mode_ = next;
  off_ticks_ = 0;  // tick() repeats the broadcast
}

void BenchServo::disarm() {
  if (mode_ != BenchMode::kOff) stop(BenchMode::kOff);
  faults_.clear();
}

Result BenchServo::set_target(float deg, uint32_t now_ms) {
  if (mode_ != BenchMode::kArmed) return mode_ == BenchMode::kFault ? Result::kFaultLatched : Result::kNotArmed;
  if (!finitef(deg)) return Result::kOk;  // a non-number moves nothing, but still counts as the Linux side talking
  const JointConfig& j = cfg_.joints[joint_];
  target_deg_ = deg < j.min_deg ? j.min_deg : (deg > j.max_deg ? j.max_deg : deg);
  last_cmd_ms_ = now_ms;
  return Result::kOk;
}

void BenchServo::tick(uint32_t now_ms, bool robot_armed) {
  if (mode_ != BenchMode::kArmed) {
    if (!robot_armed && (off_ticks_ < kOffRepeatTicks || (mode_ == BenchMode::kFault && off_ticks_ % 100 == 0))) {
      bus_.torque_off_all();
    }
    ++off_ticks_;
    return;
  }
  const JointConfig& j = cfg_.joints[joint_];
  float dt = static_cast<float>(now_ms - last_tick_ms_) * 0.001f;
  if (dt > 0.05f) dt = 0.05f;
  last_tick_ms_ = now_ms;

  Feedback fb;
  if (bus_.read_feedback(j.servo_id, &fb) == BusError::kOk) {
    fb_ = fb;
    read_ok_ = true;
    read_ms_ = now_ms;
    faults_.on_feedback(joint_, fb, now_ms);
  } else {
    read_ok_ = false;
    faults_.on_comm_fail(joint_);
  }
  if (faults_.fault() != Fault::kNone) {
    stop(BenchMode::kFault);
    return;
  }
  if (now_ms - last_cmd_ms_ > cfg_.cmd_timeout_ms) {  // the run is over or the Linux side died: go limp
    stop(BenchMode::kOff);
    return;
  }

  const float step = j.max_speed_dps * dt;
  const float err = target_deg_ - setpoint_deg_;
  setpoint_deg_ += err > step ? step : (err < -step ? -step : err);
  const int16_t goal = deg_to_step(cfg_, joint_, setpoint_deg_);
  if (bus_.write_goal(j.servo_id, static_cast<uint16_t>(goal), 0, cfg_.servo.goal_speed) != BusError::kOk) {
    faults_.raise(Fault::kBusWrite, joint_);
    stop(BenchMode::kFault);
  }
}

BenchState BenchServo::state() const {
  BenchState s;
  s.mode = mode_;
  s.fault = faults_.fault();
  s.joint = static_cast<int8_t>(joint_);
  s.read_ok = read_ok_;
  s.feedback = fb_;
  s.read_ms = read_ms_;
  s.setpoint_deg = setpoint_deg_;
  return s;
}

}  // namespace duck
