// SPDX-License-Identifier: Apache-2.0
#include "safety.h"

#include "fmath.h"

namespace duck {

namespace {

// Grid points around v: one index if v sits on a grid value (or beyond the
// ends), otherwise the two neighbours.
void bracket(const float* grid, int n, float v, int* lo, int* hi) {
  const float eps = 1e-3f;
  if (v <= grid[0] + eps) { *lo = *hi = 0; return; }
  if (v >= grid[n - 1] - eps) { *lo = *hi = n - 1; return; }
  for (int i = 0; i < n; ++i) {
    if (absf(v - grid[i]) <= eps) { *lo = *hi = i; return; }
    if (v < grid[i]) { *lo = i - 1; *hi = i; return; }
  }
  *lo = *hi = n - 1;
}

}  // namespace

LegPose leg_pose(const float q[kNumJoints], bool right) {
  if (right) return {-q[kRHipYaw], -q[kRHipRoll], -q[kRHipPitch], -q[kRKnee], -q[kRAnkle]};
  return {q[kLHipYaw], q[kLHipRoll], q[kLHipPitch], q[kLKnee], q[kLAnkle]};
}

bool hip_pose_allowed(const HipMapConfig& m, bool right, float pitch, float roll, float yaw, float knee,
                      float ankle) {
  if (!(pitch == pitch && roll == roll && yaw == yaw && knee == knee && ankle == ankle)) return false;
  int p0, p1, r0, r1, y0, y1, k0, k1, a0, a1;
  bracket(m.pitch, kHipPitchN, pitch, &p0, &p1);
  bracket(m.roll, kHipRollN, roll, &r0, &r1);
  bracket(m.yaw, kHipYawN, yaw, &y0, &y1);
  bracket(m.knee, kHipKneeN, knee, &k0, &k1);
  bracket(m.ankle, kHipAnkleN, ankle, &a0, &a1);
  const auto& leg = m.collides[right ? 1 : 0];
  for (int a = a0; a <= a1; ++a)
    for (int p = p0; p <= p1; ++p)
      for (int y = y0; y <= y1; ++y)
        for (int k = k0; k <= k1; ++k)
          for (int r = r0; r <= r1; ++r)
            if (leg[a][p][y][k] & (1u << r)) return false;
  return true;
}

bool knee_ankle_allowed(const KneeAnkleConfig& c, float knee, float ankle) {
  if (!(knee == knee && ankle == ankle)) return false;
  if (knee <= c.knee_over_deg) return true;
  for (int i = 0; i < kKneeAnkleN; ++i) {
    if (knee <= c.knee[i]) return ankle >= c.ankle_min[i];
  }
  return ankle >= c.ankle_min[kKneeAnkleN - 1];
}

bool leg_allowed(const RobotConfig& cfg, const float q[kNumJoints], bool right) {
  const LegPose l = leg_pose(q, right);
  return knee_ankle_allowed(cfg.knee_ankle, l.knee, l.ankle) &&
         hip_pose_allowed(cfg.hip, right, l.pitch, l.roll, l.yaw, l.knee, l.ankle);
}

bool both_legs_allowed(const BothLegsConfig& c, const float q[kNumJoints]) {
  const LegPose l = leg_pose(q, false), r = leg_pose(q, true);
  if (!(l.yaw == l.yaw && r.yaw == r.yaw && l.pitch == l.pitch && r.pitch == r.pitch && l.roll == l.roll &&
        r.roll == r.roll)) {
    return false;
  }
  if (!(l.yaw > c.yaw_out_over_deg && r.yaw > c.yaw_out_over_deg)) return true;
  if (l.pitch < c.pitch_forward_of_deg && r.pitch < c.pitch_forward_of_deg) return false;
  const float roll_in = l.roll < r.roll ? l.roll : r.roll;
  return !(l.pitch < c.band_pitch_forward_of_deg && r.pitch < c.band_pitch_forward_of_deg &&
           roll_in < c.band_roll_in_past_deg);
}

namespace {

// The five joints of a leg follow each other in JointIndex, hip yaw first.
static_assert(kLHipRoll == kLHipYaw + 1 && kLHipPitch == kLHipYaw + 2 && kLKnee == kLHipYaw + 3 &&
                  kLAnkle == kLHipYaw + 4 && kRHipRoll == kRHipYaw + 1 && kRHipPitch == kRHipYaw + 2 &&
                  kRKnee == kRHipYaw + 3 && kRAnkle == kRHipYaw + 4,
              "JointIndex order changed: check the leg rules");
constexpr int kLegJoints = 5;
constexpr int kAnkleOffset = 4;

void hold(float next[kNumJoints], const float prev[kNumJoints], int first, int count) {
  for (int i = first; i < first + count; ++i) next[i] = prev[i];
}

// Make one leg's next pose an allowed one, changing as little as possible: all of it; else the ankle held; else
// the hip and knee held while the ankle moves; else the whole leg where it is. Returns the flags to raise.
uint16_t settle_leg(const RobotConfig& cfg, const float prev[kNumJoints], float next[kNumJoints], bool right) {
  if (leg_allowed(cfg, next, right)) return 0;
  const int first = right ? kRHipYaw : kLHipYaw;
  const LegPose wanted = leg_pose(next, right);
  uint16_t flags = right ? kFlagLegRight : kFlagLegLeft;
  if (!knee_ankle_allowed(cfg.knee_ankle, wanted.knee, wanted.ankle)) flags |= kFlagKneeAnkle;

  float trial[kNumJoints];
  for (int i = 0; i < kNumJoints; ++i) trial[i] = next[i];
  trial[first + kAnkleOffset] = prev[first + kAnkleOffset];
  if (!leg_allowed(cfg, trial, right)) {
    trial[first + kAnkleOffset] = next[first + kAnkleOffset];
    hold(trial, prev, first, kAnkleOffset);
    if (!leg_allowed(cfg, trial, right)) hold(trial, prev, first, kLegJoints);
  }
  for (int i = first; i < first + kLegJoints; ++i) next[i] = trial[i];
  return flags;
}

}  // namespace

bool head_interlock(const HeadInterlockConfig& c, float prev_yaw, float prev_roll, float* yaw, float* roll) {
  float win = c.yaw_window_deg - c.yaw_margin_deg;
  float rmax = c.roll_max_deg - c.roll_margin_deg;
  if (win < 0.0f) win = 0.0f;
  if (rmax < c.roll_free_deg) rmax = c.roll_free_deg;
  const float free_roll = c.roll_free_deg;
  const float y_in = *yaw, r_in = *roll;
  float y = y_in;
  float r = clampf(r_in, -rmax, rmax);

  const bool prev_rolled = absf(prev_roll) > free_roll;
  const bool prev_yawed = absf(prev_yaw) > win;
  if (prev_rolled) {
    // Rolled: yaw must stay inside the window. If it is already outside (a
    // state the interlock never produces itself), hold yaw where it is.
    y = prev_yawed ? prev_yaw : clampf(y, -win, win);
  }
  if (absf(y) > win) {
    // Yawed: roll must stay near zero. If it is already rolled, it may only
    // move toward zero.
    if (prev_rolled) {
      r = prev_roll > 0.0f ? clampf(r, -free_roll, prev_roll) : clampf(r, prev_roll, free_roll);
    } else {
      r = clampf(r, -free_roll, free_roll);
    }
  }
  *yaw = y;
  *roll = r;
  return y != y_in || r != r_in;
}

float clamp_joint(const JointConfig& j, float deg, uint16_t* flags) {
  const float c = clampf(deg, j.min_deg, j.max_deg);
  if (c != deg) *flags |= kFlagJointClamp;
  return c;
}

float rate_limit(const JointConfig& j, float setpoint, float target, float dt_s, uint16_t* flags) {
  const float max_step = j.max_speed_dps * dt_s;
  const float d = clampf(target - setpoint, -max_step, max_step);
  if (d != target - setpoint) *flags |= kFlagRateLimit;
  return setpoint + d;
}

uint16_t settle_legs(const RobotConfig& cfg, const float prev[kNumJoints], float next[kNumJoints]) {
  uint16_t flags = settle_leg(cfg, prev, next, false);
  flags |= settle_leg(cfg, prev, next, true);
  // Each leg is now where its own rules allow. If the two together are not allowed, both stay where they are.
  if (!both_legs_allowed(cfg.both_legs, next)) {
    hold(next, prev, kLHipYaw, kLegJoints);
    hold(next, prev, kRHipYaw, kLegJoints);
    flags |= kFlagBothLegs;
  }
  return flags;
}

void SafetyFilter::reset(const float current[kNumJoints]) {
  for (int i = 0; i < kNumJoints; ++i) q_[i] = (current[i] == current[i]) ? current[i] : 0.0f;
}

uint16_t SafetyFilter::step(const float target[kNumJoints], float dt_s, bool checked_path, float out[kNumJoints]) {
  uint16_t flags = 0;
  float next[kNumJoints];
  if (!(dt_s > 0.0f)) dt_s = 0.0f;
  if (dt_s > 0.05f) dt_s = 0.05f;

  for (int i = 0; i < kNumJoints; ++i) {
    float t = target[i];
    if (!finitef(t)) {
      t = q_[i];
      flags |= kFlagNonFinite;
    }
    next[i] = limit_joint(cfg_.joints[i], q_[i], t, dt_s, &flags);
  }

  if (head_interlock(cfg_.head, q_[kHeadYaw], q_[kHeadRoll], &next[kHeadYaw], &next[kHeadRoll])) {
    flags |= kFlagHeadInterlock;
  }

  if (!checked_path) flags |= settle_legs(cfg_, q_, next);

  for (int i = 0; i < kNumJoints; ++i) {
    q_[i] = next[i];
    out[i] = next[i];
  }
  return flags;
}

void FaultMonitor::clear() {
  fault_ = Fault::kNone;
  joint_ = -1;
  for (int i = 0; i < kNumJoints; ++i) {
    overloaded_[i].reset();
    comm_fails_[i] = 0;
  }
  tilt_bad_.reset();
}

void FaultMonitor::on_feedback(int joint, const Feedback& fb, uint32_t now_ms) {
  comm_fails_[joint] = 0;
  if (fb.error & (scs::kErrOverload | scs::kErrOverheat)) raise(Fault::kServoStatus, joint);
  if (static_cast<float>(fb.temperature_c) >= cfg_.over_temp_c) raise(Fault::kOverTemp, joint);
  if (static_cast<float>(fb.voltage_dv) * 0.1f < cfg_.under_voltage_v) raise(Fault::kUnderVoltage, joint);
  const float load_pct = absf(static_cast<float>(fb.load)) * 0.1f;
  if (overloaded_[joint].update(load_pct >= cfg_.overload_load_pct, now_ms, cfg_.overload_s)) {
    raise(Fault::kOverload, joint);
  }
}

void FaultMonitor::on_comm_fail(int joint) {
  if (comm_fails_[joint] < 255) ++comm_fails_[joint];
  if (comm_fails_[joint] >= cfg_.comm_fail_count) raise(Fault::kComm, joint);
}

void FaultMonitor::on_tilt(float pitch_error_deg, float roll_deg, uint32_t now_ms) {
  const bool bad = absf(pitch_error_deg) > cfg_.fall_pitch_error_deg || absf(roll_deg) > cfg_.fall_roll_deg;
  if (tilt_bad_.update(bad, now_ms, cfg_.fall_s)) raise(Fault::kFall, -1);
}

}  // namespace duck
