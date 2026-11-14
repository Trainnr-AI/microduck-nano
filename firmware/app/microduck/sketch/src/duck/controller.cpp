// SPDX-License-Identifier: Apache-2.0
#include "controller.h"

#include "fmath.h"

namespace duck {

namespace {
const int kLegJoints[10] = {kLHipYaw, kLHipRoll, kLHipPitch, kLKnee, kLAnkle,
                            kRHipYaw, kRHipRoll, kRHipPitch, kRKnee, kRAnkle};
const int kHeadJoints[4] = {kNeckPitch, kHeadPitch, kHeadYaw, kHeadRoll};
}  // namespace

Controller::Controller(const RobotConfig& cfg, ScsBus& bus, bool motion_build_enabled)
    : cfg_(cfg), bus_(bus), motion_build_(motion_build_enabled), filter_(cfg), faults_(cfg.faults),
      balance_(cfg.balance) {
  for (int i = 0; i < kNumJoints; ++i) {
    user_[i] = nominal_[i] = measured_[i] = 0.0f;
    fb_valid_[i] = false;
    fb_[i] = Feedback();
  }
  bus_.allow_motion(false);
}

bool Controller::imu_usable(const ImuSample& imu, uint32_t now_ms) const {
  return cfg_.imu.calibrated && imu.valid && (now_ms - imu.t_ms) <= cfg_.imu.stale_ms;
}

bool Controller::gyro_usable(const ImuSample& imu, uint32_t now_ms) const {
  return cfg_.imu.calibrated && imu.gyro_valid && (now_ms - imu.gyro_t_ms) <= cfg_.imu.stale_ms;
}

bool Controller::pose_near(const float q[kNumJoints], const float pose[kNumPoseJoints], float tol,
                           bool legs_only) const {
  for (int k = 0; k < 10; ++k) {
    const int j = kLegJoints[k];
    if (absf(q[j] - pose[j]) > tol) return false;
  }
  if (!legs_only) {
    for (int k = 0; k < 4; ++k) {
      const int j = kHeadJoints[k];
      if (absf(q[j] - pose[j]) > tol) return false;
    }
  }
  return true;
}

float Controller::sit_fraction() const {
  switch (posture_) {
    case Posture::kSitting: return 1.0f;
    case Posture::kMovingToSit: return move_progress_;
    case Posture::kMovingToStand: return 1.0f - move_progress_;
    default: return 0.0f;
  }
}

Result Controller::arm(uint32_t key, uint32_t now_ms, const ImuSample& imu) {
  if (mode_ == Mode::kFault) return Result::kFaultLatched;
  if (mode_ == Mode::kArmed) return Result::kAlreadyArmed;
  if (!motion_build_) return Result::kMotionBuildDisabled;
  if (key != kArmKey) return Result::kBadKey;
  if (!config_ready_for_motion(cfg_)) return Result::kNotCalibrated;
  const bool imu_ok = imu_usable(imu, now_ms);
  if (cfg_.imu.require_to_arm && !imu_ok) return Result::kImuRequired;

  // Every servo must answer and report a healthy state before any torque.
  float q[kNumJoints];
  faults_.clear();
  for (int j = 0; j < kNumJoints; ++j) {
    Feedback fb;
    if (bus_.read_feedback(cfg_.joints[j].servo_id, &fb) != BusError::kOk) return Result::kServoMissing;
    fb_[j] = fb;
    fb_valid_[j] = true;
    q[j] = measured_[j] = step_to_deg(cfg_, j, fb.position);
    faults_.on_feedback(j, fb, now_ms);
  }
  if (faults_.fault() != Fault::kNone) {
    mode_ = Mode::kFault;
    enter_fault();
    return Result::kFaultLatched;
  }

  // Start every setpoint at the measured position so nothing jumps, send
  // those goals first, then switch torque on one servo at a time.
  filter_.reset(q);
  uint8_t ids[kNumJoints];
  uint16_t steps[kNumJoints];
  for (int j = 0; j < kNumJoints; ++j) {
    ids[j] = cfg_.joints[j].servo_id;
    steps[j] = static_cast<uint16_t>(fb_[j].position);
    user_[j] = nominal_[j] = q[j];
  }
  bus_.allow_motion(true);
  BusError e = bus_.sync_write_goal(ids, steps, kNumJoints, 0, cfg_.servo.goal_speed);
  for (int j = 0; j < kNumJoints && e == BusError::kOk; ++j) e = bus_.torque_on(ids[j]);
  if (e != BusError::kOk) {
    bus_.torque_off_all();
    bus_.allow_motion(false);
    return Result::kBusError;
  }

  // Which posture is the robot in? Legs near a keyframe (and, if the IMU is
  // usable, the trunk pitch that belongs to it); otherwise unknown.
  posture_ = Posture::kUnknown;
  const float tol = cfg_.motion.posture_detect_tol_deg;
  Tilt t = {0.0f, 0.0f};
  if (imu_ok) t = tilt_from_up(cfg_.imu, imu.up);
  if (pose_near(q, cfg_.stand, tol, true) &&
      (!imu_ok || absf(t.pitch_deg - planned_pitch_deg(cfg_.motion, 0.0f)) < 15.0f)) {
    posture_ = Posture::kStanding;
  } else if (pose_near(q, cfg_.sit, tol, true) &&
             (!imu_ok || absf(t.pitch_deg - planned_pitch_deg(cfg_.motion, 1.0f)) < 15.0f)) {
    posture_ = Posture::kSitting;
  }
  supported_ = false;
  move_.cancel();
  balance_.reset();
  mode_ = Mode::kArmed;
  last_cmd_ms_ = now_ms;
  have_tick_ = false;
  return Result::kOk;
}

void Controller::disarm() {
  bus_.torque_off_all();
  bus_.torque_off_all();
  bus_.allow_motion(false);
  move_.cancel();
  if (mode_ == Mode::kArmed) mode_ = Mode::kDisarmed;
  posture_ = Posture::kUnknown;
}

void Controller::enter_fault() {
  bus_.torque_off_all();
  bus_.torque_off_all();
  bus_.allow_motion(false);
  move_.cancel();
  mode_ = Mode::kFault;
  posture_ = Posture::kUnknown;
  fault_ticks_ = 0;
}

Result Controller::clear_fault() {
  if (mode_ != Mode::kFault) return Result::kOk;
  faults_.clear();
  mode_ = Mode::kDisarmed;  // never straight back to armed
  return Result::kOk;
}

Result Controller::set_targets(const float deg[kNumJoints], uint32_t now_ms) {
  if (mode_ != Mode::kArmed) return Result::kNotArmed;
  for (int i = 0; i < kNumJoints; ++i) {
    if (finitef(deg[i])) user_[i] = deg[i];
  }
  last_cmd_ms_ = now_ms;
  return Result::kOk;
}

Result Controller::set_head(float neck_pitch, float head_pitch, float head_yaw, float head_roll, float beak,
                            uint32_t now_ms) {
  if (mode_ != Mode::kArmed) return Result::kNotArmed;
  const float v[5] = {neck_pitch, head_pitch, head_yaw, head_roll, beak};
  const int idx[5] = {kNeckPitch, kHeadPitch, kHeadYaw, kHeadRoll, kBeak};
  for (int i = 0; i < 5; ++i) {
    if (finitef(v[i])) user_[idx[i]] = v[i];
  }
  last_cmd_ms_ = now_ms;
  return Result::kOk;
}

Result Controller::stand(bool supported, uint32_t now_ms) {
  if (mode_ != Mode::kArmed) return Result::kNotArmed;
  if (move_.active()) return Result::kBusy;
  last_cmd_ms_ = now_ms;
  switch (posture_) {
    case Posture::kSitting:
      move_.start(cfg_.sit, cfg_.stand, cfg_.motion.sit_stand_s);
      posture_ = Posture::kMovingToStand;
      break;
    case Posture::kStanding:
      move_.start(nominal_, cfg_.stand, cfg_.motion.return_to_stand_s);
      posture_ = Posture::kReturningToStand;
      break;
    case Posture::kUnknown:
      if (!supported) return Result::kWrongPosture;
      move_.start(nominal_, cfg_.stand, cfg_.motion.sit_stand_s);
      posture_ = Posture::kReturningToStand;
      supported_ = true;
      break;
    default:
      return Result::kBusy;
  }
  move_progress_ = 0.0f;
  return Result::kOk;
}

Result Controller::sit(uint32_t now_ms, const ImuSample& imu) {
  if (mode_ != Mode::kArmed) return Result::kNotArmed;
  if (move_.active()) return Result::kBusy;
  if (posture_ != Posture::kStanding) return Result::kWrongPosture;
  // The checked path starts at the STAND keyframe, head included.
  if (!pose_near(nominal_, cfg_.stand, cfg_.motion.pose_tol_deg, false)) return Result::kNotAtStand;
  // The sit needs the balance law, and the slope check needs the IMU.
  if (!imu_usable(imu, now_ms)) return Result::kImuRequired;
  const Tilt t = tilt_from_up(cfg_.imu, imu.up);
  if (t.pitch_deg - planned_pitch_deg(cfg_.motion, 0.0f) > cfg_.motion.sit_uphill_limit_deg) return Result::kUphill;
  last_cmd_ms_ = now_ms;
  move_.start(cfg_.stand, cfg_.sit, cfg_.motion.sit_stand_s);
  posture_ = Posture::kMovingToSit;
  move_progress_ = 0.0f;
  return Result::kOk;
}

void Controller::poll_one_servo(uint32_t now_ms) {
  // The next joint that has a servo: on a bench with a few servos assigned, each is read every turn instead of
  // waiting behind the joints that have none.
  int j = -1;
  for (int k = 0; k < kNumJoints && j < 0; ++k) {
    const int candidate = (poll_joint_ + k) % kNumJoints;
    if (cfg_.joints[candidate].servo_id != kServoIdUnassigned) j = candidate;
  }
  if (j < 0) return;  // no servo assigned at all
  poll_joint_ = (j + 1) % kNumJoints;
  const uint8_t id = cfg_.joints[j].servo_id;
  Feedback fb;
  if (bus_.read_feedback(id, &fb) == BusError::kOk) {
    fb_[j] = fb;
    fb_valid_[j] = true;
    measured_[j] = step_to_deg(cfg_, j, fb.position);
    if (mode_ == Mode::kArmed) faults_.on_feedback(j, fb, now_ms);
  } else {
    fb_valid_[j] = false;
    if (mode_ == Mode::kArmed) faults_.on_comm_fail(j);
  }
}

void Controller::tick(uint32_t now_ms, const ImuSample& imu) {
  float dt = have_tick_ ? static_cast<float>(now_ms - last_tick_ms_) * 0.001f : 0.01f;
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.05f) dt = 0.05f;
  last_tick_ms_ = now_ms;
  have_tick_ = true;

  imu_ok_ = imu_usable(imu, now_ms);
  if (imu_ok_) tilt_ = tilt_from_up(cfg_.imu, imu.up);
  // Reported for the Linux side's policy only; nothing in the firmware acts on it. A stale gyro reads as
  // "not measured", never as the last value or zero.
  gyro_ok_ = gyro_usable(imu, now_ms);
  if (gyro_ok_) {
    sensor_to_body(cfg_.imu, imu.gyro_dps, gyro_dps_);
  } else {
    gyro_dps_[0] = gyro_dps_[1] = gyro_dps_[2] = 0.0f;
  }

  if (mode_ == Mode::kFault) {
    // Keep saying "torque off" for a while, then once a second.
    if (fault_ticks_ < 10 || fault_ticks_ % 100 == 0) bus_.torque_off_all();
    ++fault_ticks_;
    flags_ = 0;
    return;
  }

  if (mode_ == Mode::kDisarmed) {
    // Telemetry only, at a low rate so a bench without servos costs little.
    flags_ = imu_ok_ ? 0 : kStatImuStale;
    if (now_ms - idle_poll_ms_ >= 100) {
      idle_poll_ms_ = now_ms;
      poll_one_servo(now_ms);
    }
    return;
  }

  // ---- ARMED ----
  uint16_t flags = 0;
  poll_one_servo(now_ms);

  // Linux side silent: stop following its targets, hold what we have.
  if (now_ms - last_cmd_ms_ > cfg_.cmd_timeout_ms) {
    flags |= kStatCmdTimeout;
    if (!timed_out_) {
      // Freeze at the setpoints of this moment (ankles without the balance
      // correction, which keeps running on top).
      timed_out_ = true;
      const float* sp = filter_.setpoint();
      for (int i = 0; i < kNumJoints; ++i) user_[i] = sp[i];
      if (posture_ != Posture::kUnknown && !supported_) {
        user_[kLAnkle] -= balance_.correction();
        user_[kRAnkle] += balance_.correction();
      }
    }
  } else {
    timed_out_ = false;
  }

  float want[kNumJoints];
  for (int i = 0; i < kNumJoints; ++i) want[i] = nominal_[i];

  if (move_.active()) {
    move_progress_ = move_.advance(dt, want);  // fills the 14 pose joints
    if (!move_.active()) {
      if (posture_ == Posture::kMovingToSit) {
        posture_ = Posture::kSitting;
      } else {
        posture_ = Posture::kStanding;
        supported_ = false;
      }
      for (int i = 0; i < kNumPoseJoints; ++i) user_[i] = want[i];
    }
  } else {
    switch (posture_) {
      case Posture::kStanding:
        for (int i = 0; i < kNumPoseJoints; ++i) want[i] = user_[i];
        break;
      case Posture::kSitting:
        for (int i = 0; i < kNumPoseJoints; ++i) want[i] = cfg_.sit[i];
        if (cfg_.motion.head_motion_while_sitting) {
          for (int k = 0; k < 4; ++k) want[kHeadJoints[k]] = user_[kHeadJoints[k]];
        }
        flags |= kStatLegCmdIgnored;
        break;
      default:  // unknown posture: legs hold, head follows
        for (int k = 0; k < 4; ++k) want[kHeadJoints[k]] = user_[kHeadJoints[k]];
        flags |= kStatLegCmdIgnored;
        break;
    }
  }
  want[kBeak] = user_[kBeak];
  for (int i = 0; i < kNumJoints; ++i) nominal_[i] = want[i];

  // IMU ankle balance and fall detection, against the planned trunk pitch.
  const bool balancing = posture_ != Posture::kUnknown && !supported_;
  if (supported_) flags |= kStatSupported;
  if (!imu_ok_) flags |= kStatImuStale;
  if (balancing && imu_ok_) {
    const float err = tilt_.pitch_deg - planned_pitch_deg(cfg_.motion, sit_fraction());
    balance_.update(err, dt);
    faults_.on_tilt(err, tilt_.roll_deg, now_ms);
  } else {
    faults_.tilt_unknown();
  }
  if (balancing) {  // with a stale IMU the last correction is kept
    want[kLAnkle] += balance_.correction();
    want[kRAnkle] -= balance_.correction();
  }

  const bool checked_path = posture_ == Posture::kSitting || posture_ == Posture::kMovingToSit ||
                            posture_ == Posture::kMovingToStand;
  float out[kNumJoints];
  flags |= filter_.step(want, dt, checked_path, out);

  uint8_t ids[kNumJoints];
  uint16_t steps[kNumJoints];
  for (int j = 0; j < kNumJoints; ++j) {
    ids[j] = cfg_.joints[j].servo_id;
    steps[j] = static_cast<uint16_t>(deg_to_step(cfg_, j, out[j]));
  }
  if (bus_.sync_write_goal(ids, steps, kNumJoints, 0, cfg_.servo.goal_speed) != BusError::kOk) {
    faults_.raise(Fault::kBusWrite, -1);
  }

  flags_ = flags;
  if (faults_.fault() != Fault::kNone) enter_fault();
}

Status Controller::status() const {
  Status s;
  s.mode = mode_;
  s.posture = posture_;
  s.fault = faults_.fault();
  s.fault_joint = static_cast<int8_t>(faults_.fault_joint());
  s.flags = flags_;
  s.imu_ok = imu_ok_;
  s.pitch_deg = tilt_.pitch_deg;
  s.roll_deg = tilt_.roll_deg;
  s.balance_deg = balance_.correction();
  s.gyro_ok = gyro_ok_;
  for (int i = 0; i < 3; ++i) s.gyro_dps[i] = gyro_dps_[i];
  return s;
}

}  // namespace duck
