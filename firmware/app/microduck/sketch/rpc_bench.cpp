// SPDX-License-Identifier: Apache-2.0
// The Bridge functions of a bench build (DUCK_BENCH_MOTION 1): one servo moved on its own (src/duck/bench.h), and
// a second one beside it (duck_bench2_*) for a bench with two servos. The two share the bus: when either stops, for
// any reason, every servo's torque goes off and the other stops with it.
#include <Arduino.h>
#include <Arduino_RouterBridge.h>

#include <array>

#include "duck_build_config.h"
#ifndef DUCK_TICK_MS
#error "duck_build_config.h must be included before its switches are tested"
#endif

#if DUCK_BENCH_MOTION

#include "duck_globals.h"
#include "rpc_util.h"
#include "status_display.h"

using rpc::code;
using rpc::tenths;

namespace {

typedef std::array<int32_t, 12> BenchWords;  // see rpc_bench_step()

// Arm one joint's servo on its own. Refused unless the robot is disarmed.
int32_t bench_arm(duck::BenchServo& bench, const duck::BenchServo& other, uint32_t key, int32_t joint) {
  note_command();
  if (g_ctl.status().mode != duck::Mode::kDisarmed) return code(duck::Result::kBusy);
  // Not the joint the other one holds, and not while the other has a fault latched (its servo-off repeats).
  const duck::BenchState o = other.state();
  if (o.mode == duck::BenchMode::kFault || (o.mode == duck::BenchMode::kArmed && o.joint == joint)) {
    return code(duck::Result::kBusy);
  }
  return code(bench.arm(key, joint, millis()));
}
int32_t rpc_bench_arm(uint32_t key, int32_t joint) { return bench_arm(g_bench, g_bench2, key, joint); }
int32_t rpc_bench2_arm(uint32_t key, int32_t joint) { return bench_arm(g_bench2, g_bench, key, joint); }

// One bench tick in one round trip: the target in degrees x10 out (kMissing: no new target, just read), the servo's
// state back:
// [result, mode (0 off, 1 armed, 2 fault), fault, joint, read ok, position deg x10, speed deg/s x10,
//  load 0.1 % (signed, in the joint's direction), voltage 0.1 V, temperature C, read time ms, setpoint deg x10].
// Position, speed and load are as of the servo's last read, made at `read time` on the microcontroller's clock.
BenchWords bench_step(duck::BenchServo& bench, int32_t deg10) {
  note_command();
  const duck::Result r =
      deg10 == rpc::kMissing ? duck::Result::kOk : bench.set_target(static_cast<float>(deg10) * 0.1f, millis());
  const duck::BenchState s = bench.state();
  const duck::Feedback& fb = s.reading.fb;
  BenchWords out;
  out.fill(0);
  out[0] = code(r);
  out[1] = code(s.mode);
  out[2] = code(s.fault);
  out[3] = s.joint;
  out[4] = s.reading.ok ? 1 : 0;
  if (s.joint >= 0) {
    const duck::JointConfig& j = g_config.joints[s.joint];
    out[5] = tenths(duck::step_to_deg(g_config, s.joint, fb.position));
    out[6] = tenths(duck::speed_to_dps(g_config, s.joint, fb.speed));
    out[7] = fb.load * j.direction;
  }
  out[8] = fb.voltage_dv;
  out[9] = fb.temperature_c;
  out[10] = static_cast<int32_t>(s.reading.ms);
  out[11] = tenths(s.setpoint_deg);
  return out;
}

BenchWords rpc_bench_step(int32_t deg10) { return bench_step(g_bench, deg10); }
BenchWords rpc_bench2_step(int32_t deg10) { return bench_step(g_bench2, deg10); }

// Either call switches both off: the two share the bus.
int32_t rpc_bench_disarm() {
  note_command();
  g_bench.disarm();
  g_bench2.disarm();
  return 0;
}

}  // namespace

void rpc::register_bench() {
  Bridge.provide_safe("duck_bench_arm", rpc_bench_arm);
  Bridge.provide_safe("duck_bench_step", rpc_bench_step);
  Bridge.provide_safe("duck_bench_disarm", rpc_bench_disarm);
  Bridge.provide_safe("duck_bench2_arm", rpc_bench2_arm);
  Bridge.provide_safe("duck_bench2_step", rpc_bench2_step);
  Bridge.provide_safe("duck_bench2_disarm", rpc_bench_disarm);
}

#endif  // DUCK_BENCH_MOTION
