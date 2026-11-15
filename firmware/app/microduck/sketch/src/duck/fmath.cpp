// SPDX-License-Identifier: Apache-2.0
#include "fmath.h"

namespace duck {

float sqrt_f(float v) {
  if (!(v > 0.0f)) return 0.0f;
  // Scale into 0.25..4 by powers of 4 (exact), then Newton's method.
  float scale = 1.0f;
  while (v > 4.0f) { v *= 0.25f; scale *= 2.0f; }
  while (v < 0.25f) { v *= 4.0f; scale *= 0.5f; }
  float x = 0.5f * (v + 1.0f);
  for (int i = 0; i < 6; ++i) x = 0.5f * (x + v / x);
  return x * scale;
}

float sin_f(float x) {
  if (!finitef(x)) return 0.0f;
  // Reduce to -pi..pi, then to -pi/2..pi/2 by symmetry.
  const float two_pi = 2.0f * kPi;
  if (x > kPi || x < -kPi) {
    const float turns = x / two_pi;
    x -= two_pi * static_cast<float>(round_to_long(turns));
  }
  if (x > 0.5f * kPi) x = kPi - x;
  if (x < -0.5f * kPi) x = -kPi - x;
  const float x2 = x * x;
  // Taylor series to x^11: error below 6e-8 on -pi/2..pi/2.
  return x * (1.0f + x2 * (-1.0f / 6.0f + x2 * (1.0f / 120.0f + x2 * (-1.0f / 5040.0f +
              x2 * (1.0f / 362880.0f + x2 * (-1.0f / 39916800.0f))))));
}

float cos_f(float x) { return sin_f(x + 0.5f * kPi); }

namespace {
// asin for |v| <= 0.5, radians: series to v^13, error below 2e-6.
float asin_small(float v) {
  const float v2 = v * v;
  return v * (1.0f + v2 * (1.0f / 6.0f + v2 * (3.0f / 40.0f + v2 * (15.0f / 336.0f +
              v2 * (105.0f / 3456.0f + v2 * (945.0f / 42240.0f + v2 * (10395.0f / 599040.0f)))))));
}
}  // namespace

float asin_deg(float v) {
  if (!(v == v)) return 0.0f;
  v = clampf(v, -1.0f, 1.0f);
  const float a = absf(v);
  float r;
  if (a <= 0.5f) {
    r = asin_small(a);
  } else {
    // asin(a) = pi/2 - 2 asin(sqrt((1 - a) / 2))
    r = 0.5f * kPi - 2.0f * asin_small(sqrt_f(0.5f * (1.0f - a)));
  }
  r *= 180.0f / kPi;
  return v < 0.0f ? -r : r;
}

}  // namespace duck
