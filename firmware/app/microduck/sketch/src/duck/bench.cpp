// SPDX-License-Identifier: Apache-2.0
#include "bench.h"

#include "fmath.h"

namespace duck {

BenchServo::BenchServo(const RobotConfig& cfg, ScsBus& bus, bool enabled)
    : cfg_(cfg), bus_(bus), enabled_(enabled), faults_(cfg.faults) {}

Result BenchServo::arm(uint32_t key, int joint, uint32_t now_ms) {
  if (mode_ == BenchMode::kFault) return Result::kFaultLatched;
  if (mode_ == BenchMode::kArmed) return Result::kAlreadyArmed;
  if (!enabled_) return Result::kMotionBuildDisabled;
  if (key != kArmKey) return Result::kBadKey;
  // The robot's per-joint check, plus a speed limit: the bench's only motion is rate-limited by it.
  if (!joint_ready(cfg_, joint) || !(cfg_.joints[joint].max_speed_dps > 0.0f)) return Result::kNotCalibrated;
  const uint8_t id = cfg_.joints[joint].servo_id;

  faults_.clear();
  Feedback fb;
  if (bus_.read_feedback(id, &fb) != BusError::kOk) return Result::kServoMissing;
  faults_.on_feedback(joint, fb, now_ms);
  joint_ = joint;
  reading_.got(fb, now_ms);
  if (faults_.fault() != Fault::kNone) {
    stop(BenchMode::kFault);
    return Result::kFaultLatched;
  }

  // Start where the servo is, so nothing jumps; then torque on. The target starts there too, unclamped: until the
  // Linux side sends one, the servo holds where it is, even outside its limits.
  setpoint_deg_ = target_deg_ = step_to_deg(cfg_, joint, fb.position);
  bus_.allow_motion(true);
  BusError e = bus_.write_goal(id, static_cast<uint16_t>(fb.position), 0, cfg_.servo.goal_speed);
  if (e == BusError::kOk) e = bus_.torque_on(id);
  if (e != BusError::kOk) {
    stop(BenchMode::kOff);
    return Result::kBusError;
  }
  mode_ = BenchMode::kArmed;
  write_fails_ = 0;
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
  last_cmd_ms_ = now_ms;
  if (!finitef(deg)) return Result::kOk;  // a non-number moves nothing, but still counts as the Linux side talking
  uint16_t flags = 0;  // the bench reports no safety flags
  target_deg_ = clamp_joint(cfg_.joints[joint_], deg, &flags);
  return Result::kOk;
}

void BenchServo::tick(uint32_t now_ms, bool robot_armed) {
  if (mode_ != BenchMode::kArmed) {
    if (!robot_armed && off_repeat_due(off_ticks_, mode_ == BenchMode::kFault)) bus_.torque_off_all();
    ++off_ticks_;
    return;
  }
  const JointConfig& j = cfg_.joints[joint_];
  float dt = static_cast<float>(now_ms - last_tick_ms_) * 0.001f;
  if (dt > 0.05f) dt = 0.05f;
  last_tick_ms_ = now_ms;

  Feedback fb;
  if (bus_.read_feedback(j.servo_id, &fb) == BusError::kOk) {
    reading_.got(fb, now_ms);
    faults_.on_feedback(joint_, fb, now_ms);
  } else {
    reading_.ok = false;
    faults_.on_comm_fail(joint_);
  }
  if (faults_.fault() != Fault::kNone) {
    stop(BenchMode::kFault);
    return;
  }
  if (elapsed_over(now_ms, last_cmd_ms_, cfg_.cmd_timeout_ms)) {  // the run is over or the Linux side died: go limp
    stop(BenchMode::kOff);
    return;
  }

  uint16_t flags = 0;
  setpoint_deg_ = rate_limit(j, setpoint_deg_, target_deg_, dt, &flags);
  // The setpoint, not its clamp: armed outside its limits the servo comes in at the speed limit (set_target clamps
  // every target, so the setpoint only ever moves toward the limits).
  const int16_t goal = setpoint_to_step(cfg_, joint_, setpoint_deg_);
  // One unanswered goal write is not a fault: on a real bus a reply is lost now and then (about one in a thousand
  // here, 2026-11-19), and the next tick writes the goal again. A write the bus's gate refused is: something else
  // closed it. So is a run of failures as long as the one that makes a servo "not answering".
  const BusError written = bus_.write_goal(j.servo_id, static_cast<uint16_t>(goal), 0, cfg_.servo.goal_speed);
  if (written == BusError::kOk) {
    write_fails_ = 0;
  } else if (written == BusError::kGated || ++write_fails_ >= cfg_.faults.comm_fail_count) {
    faults_.raise(Fault::kBusWrite, joint_);
    stop(BenchMode::kFault);
  }
}

BenchState BenchServo::state() const {
  BenchState s;
  s.mode = mode_;
  s.fault = faults_.fault();
  s.joint = static_cast<int8_t>(joint_);
  s.reading = reading_;
  s.setpoint_deg = setpoint_deg_;
  return s;
}

#define DUCK_BENCH_SLOT {cfg, bus, enabled}
BenchSet::BenchSet(const RobotConfig& cfg, ScsBus& bus, bool enabled)
    : slot_{DUCK_BENCH_SLOT, DUCK_BENCH_SLOT, DUCK_BENCH_SLOT, DUCK_BENCH_SLOT, DUCK_BENCH_SLOT,
            DUCK_BENCH_SLOT, DUCK_BENCH_SLOT, DUCK_BENCH_SLOT, DUCK_BENCH_SLOT, DUCK_BENCH_SLOT,
            DUCK_BENCH_SLOT, DUCK_BENCH_SLOT, DUCK_BENCH_SLOT, DUCK_BENCH_SLOT, DUCK_BENCH_SLOT} {
  static_assert(kNumJoints == 15, "one DUCK_BENCH_SLOT per joint");
}
#undef DUCK_BENCH_SLOT

Result BenchSet::arm(uint32_t key, uint32_t mask, uint32_t now_ms) {
  if (!idle()) return armed() ? Result::kAlreadyArmed : Result::kFaultLatched;
  if (mask == 0 || (mask >> kNumJoints) != 0) return Result::kNotCalibrated;  // no joint, or one the table has not
  for (int j = 0; j < kNumJoints; ++j) {
    if (!(mask & (1u << j))) continue;
    const Result r = slot_[j].arm(key, j, now_ms);
    if (r != Result::kOk) {
      disarm();
      return r;
    }
  }
  return Result::kOk;
}

void BenchSet::disarm() {
  for (BenchServo& s : slot_) s.disarm();
}

Result BenchSet::set_targets(const float* deg, uint32_t now_ms) {
  Result worst = Result::kOk;
  for (int j = 0; j < kNumJoints; ++j) {
    if (slot_[j].idle()) continue;
    const Result r = slot_[j].set_target(deg[j], now_ms);
    if (r != Result::kOk) worst = r;
  }
  return worst;
}

void BenchSet::tick(uint32_t now_ms, bool bus_in_use) {
  int armed_now = 0;
  for (const BenchServo& s : slot_) armed_now += s.armed() ? 1 : 0;
  for (BenchServo& s : slot_) s.tick(now_ms, bus_in_use || armed_now > (s.armed() ? 1 : 0));
}

bool BenchSet::armed() const {
  for (const BenchServo& s : slot_) {
    if (s.armed()) return true;
  }
  return false;
}

bool BenchSet::idle() const {
  for (const BenchServo& s : slot_) {
    if (!s.idle()) return false;
  }
  return true;
}

BenchState BenchSet::shown() const {
  BenchState best = slot_[0].state();
  int rank = -1;
  for (const BenchServo& s : slot_) {
    const BenchState b = s.state();
    const int r = b.mode == BenchMode::kFault ? (b.fault != Fault::kBusWrite ? 3 : 2) : b.mode == BenchMode::kArmed ? 1 : 0;
    if (r > rank) {
      rank = r;
      best = b;
    }
  }
  return best;
}

}  // namespace duck
