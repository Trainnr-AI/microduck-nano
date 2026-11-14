// SPDX-License-Identifier: Apache-2.0
#include "motion.h"

#include "fmath.h"

namespace duck {

namespace {
float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }
}  // namespace

float ease_cos(float u) { return (1.0f - cos_f(kPi * clamp01(u))) * 0.5f; }

void PoseMove::start(const float from[kNumPoseJoints], const float to[kNumPoseJoints], float duration_s) {
  for (int i = 0; i < kNumPoseJoints; ++i) {
    from_[i] = from[i];
    to_[i] = to[i];
  }
  duration_ = duration_s > 0.05f ? duration_s : 0.05f;
  t_ = 0.0f;
  active_ = true;
}

float PoseMove::advance(float dt_s, float out[kNumPoseJoints]) {
  if (dt_s > 0.0f) t_ += dt_s;
  const float u = clamp01(t_ / duration_);
  const float e = ease_cos(u);
  for (int i = 0; i < kNumPoseJoints; ++i) out[i] = from_[i] + (to_[i] - from_[i]) * e;
  if (u >= 1.0f) active_ = false;
  return e;
}

float planned_pitch_deg(const MotionConfig& cfg, float sit_fraction) {
  const float x = clamp01(sit_fraction) * (kPlannedPitchN - 1);
  int i = static_cast<int>(x);
  if (i >= kPlannedPitchN - 1) return cfg.planned_pitch_deg[kPlannedPitchN - 1];
  const float f = x - static_cast<float>(i);
  return cfg.planned_pitch_deg[i] + (cfg.planned_pitch_deg[i + 1] - cfg.planned_pitch_deg[i]) * f;
}

void sensor_to_body(const ImuConfig& cfg, const float v[3], float out[3]) {
  for (int r = 0; r < 3; ++r) {
    out[r] = cfg.sensor_to_body[r][0] * v[0] + cfg.sensor_to_body[r][1] * v[1] + cfg.sensor_to_body[r][2] * v[2];
  }
}

Tilt tilt_from_up(const ImuConfig& cfg, const float up[3]) {
  float b[3];
  sensor_to_body(cfg, up, b);
  const float n = sqrt_f(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
  Tilt t = {0.0f, 0.0f};
  if (n < 1e-6f) return t;
  const float x = b[0] / n, y = b[1] / n;
  // Same definition as the simulation (tools/sim/sit_stand_robust.py imu_tilt):
  // the body's forward (x) and left (y) axes dotted with "up".
  t.pitch_deg = asin_deg(x);
  t.roll_deg = asin_deg(y);
  return t;
}

float BalanceLaw::update(float pitch_error_deg, float dt_s) {
  if (!finitef(pitch_error_deg)) return correction_;
  const float tau = cfg_.lowpass_ms * 0.001f;
  float a = 1.0f;
  if (tau > 0.0f) {
    a = dt_s / tau;
    if (a > 1.0f) a = 1.0f;
    if (a < 0.0f) a = 0.0f;
  }
  filtered_ += (pitch_error_deg - filtered_) * a;
  float mag = absf(filtered_) - cfg_.dead_zone_deg;
  if (mag < 0.0f) mag = 0.0f;
  float c = cfg_.ankle_gain * (filtered_ < 0.0f ? -mag : mag);
  if (c > cfg_.max_correction_deg) c = cfg_.max_correction_deg;
  if (c < -cfg_.max_correction_deg) c = -cfg_.max_correction_deg;
  correction_ = c;
  return c;
}

}  // namespace duck
