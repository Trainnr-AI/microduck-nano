// SPDX-License-Identifier: Apache-2.0
// What the Bridge functions (rpc*.cpp) share: how replies are packed, and when the bus is not theirs to use.
#pragma once
#include <stdint.h>

#include <type_traits>

#include "duck_globals.h"
#include "src/duck/fmath.h"

namespace rpc {

// "Not measured" in a reply (a servo that did not answer, a gyro that is not reporting), and "no new target" in a
// request.
constexpr int32_t kMissing = -32768;

// Degrees (or anything else) x10, rounded half away from zero.
inline int32_t tenths(float v) { return static_cast<int32_t>(duck::round_to_long(v * 10.0f)); }

// An enum's code on the wire.
template <class E, class = typename std::enable_if<std::is_enum<E>::value>::type>
constexpr int32_t code(E e) {
  return static_cast<int32_t>(e);
}

// One joint's measured angle x10, or kMissing where the servo did not answer.
inline int32_t joint_word(int j) { return g_ctl.feedback_valid(j) ? tenths(g_ctl.measured_deg()[j]) : kMissing; }

// One joint's speed as its servo reported it in the same read, degrees per second x10, or kMissing.
inline int32_t speed_word(int j) {
  return g_ctl.feedback_valid(j) ? tenths(duck::speed_to_dps(g_config, j, g_ctl.feedback(j).speed)) : kMissing;
}

// The longest control tick since the last call, in us; the next call starts over.
inline int32_t take_worst_tick() {
  const int32_t worst = static_cast<int32_t>(g_tick_us_max);
  g_tick_us_max = 0;
  return worst;
}

// The status words duck_status and the state message share, in this order:
// mode, posture, fault, fault joint, flags, imu ok, pitch x10, roll x10.
constexpr int kStatusWords = 8;
inline void put_status(const duck::Status& s, int32_t* out) {
  out[0] = code(s.mode);
  out[1] = code(s.posture);
  out[2] = code(s.fault);
  out[3] = s.fault_joint;
  out[4] = s.flags;
  out[5] = s.imu_ok ? 1 : 0;
  out[6] = tenths(s.pitch_deg);
  out[7] = tenths(s.roll_deg);
}

// A control loop owns the bus (the robot or a bench servo is armed): the raw bus tools are refused.
inline bool bus_owned() { return g_ctl.status().mode == duck::Mode::kArmed || g_bench.armed(); }

// A servo ID that is not the broadcast.
inline bool valid_id(int32_t id) { return id >= 0 && id <= 253; }

// Registration of the functions only some builds have (rpc_bench.cpp, rpc_provision.cpp).
void register_bench();
void register_provisioning();

}  // namespace rpc
