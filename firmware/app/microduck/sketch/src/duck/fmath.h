// SPDX-License-Identifier: Apache-2.0
// The few math functions the firmware needs, written out here.
//
// Why not <math.h>: a UNO Q sketch is linked against its loader at start-up,
// and the loader on a board can be older than the installed core and lack
// some libm functions. The build still succeeds, but the sketch then silently
// never starts (seen with cosf / asinf / sqrtf on a board whose loader was
// older than its core 0.52.0). These are accurate to better than 0.01 degree,
// which is far below a servo step (0.29 degree).
#pragma once

namespace duck {

constexpr float kPi = 3.14159265358979f;

inline float absf(float v) { return v < 0.0f ? -v : v; }
inline bool finitef(float v) { return (v - v) == 0.0f; }  // false for NaN and +-infinity

float sqrt_f(float v);          // v < 0 gives 0
float sin_f(float radians);     // any angle
float cos_f(float radians);
float asin_deg(float v);        // v clamped to -1..1, result in degrees
// Round to nearest, halves away from zero.
inline long round_to_long(float v) { return static_cast<long>(v >= 0.0f ? v + 0.5f : v - 0.5f); }

}  // namespace duck
