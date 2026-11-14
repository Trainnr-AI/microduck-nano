// SPDX-License-Identifier: Apache-2.0
// The Bridge functions every build has: reads, the bench's read-only bus tools, and the robot's commands.
#include "rpc.h"

#include <Arduino.h>
#include <Arduino_RouterBridge.h>

#include <array>

#include "duck_build_config.h"
#ifndef DUCK_TICK_MS
#error "duck_build_config.h must be included before its switches are tested"
#endif
#include "duck_globals.h"
#include "rpc_util.h"
#include "status_display.h"
#include "uart_transport.h"

using rpc::code;
using rpc::joint_word;
using rpc::kMissing;
using rpc::take_worst_tick;
using rpc::tenths;

namespace {

constexpr int32_t kProtocolVersion = 3;  // 3: the state message carries the gyro

// Fixed-size arrays only: on older UNO Q cores a sketch cannot link the parts of the C++ library that std::vector's
// size checks pull in.
typedef std::array<int32_t, 7> Info;
typedef std::array<int32_t, 10> StatusWords;
typedef std::array<int32_t, duck::kNumJoints> JointWords;
typedef std::array<int32_t, duck::kNumJoints * 3> HealthWords;
typedef std::array<int32_t, 33> ScanResult;  // [count, id, id, ...]
typedef std::array<int32_t, 10> ReadResult;  // [bus error, count, byte, ...]
typedef std::array<float, duck::kNumJoints> Targets;
typedef std::array<int32_t, duck::kNumJoints> TargetWords;          // degrees x10
typedef std::array<int32_t, 10 + duck::kNumJoints + 3> StateWords;  // see state_words()

// ----------------------------------------------------------------- reads

// [protocol version, motion build, eeprom build, joint table ready, imu present, servo UART is Serial1, bench build]
Info rpc_info() {
  note_call();
  Info out = {{kProtocolVersion, DUCK_ENABLE_MOTION, DUCK_ENABLE_EEPROM_WRITE,
               duck::config_ready_for_motion(g_config) ? 1 : 0, g_imu_present ? 1 : 0,
               DUCK_SERVO_SERIAL_IS_SERIAL1, DUCK_BENCH_MOTION}};
  return out;
}

// [mode, posture, fault, fault joint, flags, imu ok, pitch x10, roll x10, balance x10, worst tick in us]
StatusWords rpc_status() {
  note_call();
  const duck::Status s = g_ctl.status();
  StatusWords out;
  rpc::put_status(s, out.data());
  out[8] = tenths(s.balance_deg);
  out[9] = take_worst_tick();
  return out;
}

// Measured joint angles, degrees x10; kMissing where the servo did not answer.
JointWords rpc_joints() {
  note_call();
  JointWords out;
  for (int j = 0; j < duck::kNumJoints; ++j) out[j] = joint_word(j);
  return out;
}

// The whole state a control loop needs, in one reply, so a tick costs one round trip on the link instead of several:
// [result, mode, posture, fault, fault joint, flags, imu ok, pitch x10, roll x10, worst tick in us,
//  then one measured joint angle per joint, degrees x10, kMissing where the servo did not answer,
//  then the body angular velocity about x (forward), y (left), z (up), degrees per second x10, kMissing each when the
//  gyro is not measured (absent, stale, or the IMU's mounting not calibrated)].
// `result` is the duck::Result of the command this reply answers (0 for a plain read). Protocol 3; protocol 2 was
// the same without the last three words.
StateWords state_words(int32_t result) {
  const duck::Status s = g_ctl.status();
  StateWords out;
  out[0] = result;
  rpc::put_status(s, out.data() + 1);
  out[1 + rpc::kStatusWords] = take_worst_tick();
  int32_t* joints = out.data() + 2 + rpc::kStatusWords;
  for (int j = 0; j < duck::kNumJoints; ++j) joints[j] = joint_word(j);
  for (int i = 0; i < 3; ++i) joints[duck::kNumJoints + i] = s.gyro_ok ? tenths(s.gyro_dps[i]) : kMissing;
  return out;
}
static_assert(2 + rpc::kStatusWords == 10, "the state message's joints start at word 10");

StateWords rpc_state() {
  note_call();
  return state_words(0);
}

// Commanded joint angles after the safety layer, degrees x10.
JointWords rpc_setpoints() {
  note_call();
  JointWords out;
  for (int j = 0; j < duck::kNumJoints; ++j) out[j] = tenths(g_ctl.setpoint_deg()[j]);
  return out;
}

// Per joint: load (0.1 %, signed), voltage (0.1 V), temperature (deg C); 45 numbers.
HealthWords rpc_health() {
  note_call();
  HealthWords out;
  for (int j = 0; j < duck::kNumJoints; ++j) {
    const duck::Feedback& fb = g_ctl.feedback(j);
    const bool ok = g_ctl.feedback_valid(j);
    out[j * 3] = ok ? fb.load : kMissing;
    out[j * 3 + 1] = ok ? fb.voltage_dv : -1;
    out[j * 3 + 2] = ok ? fb.temperature_c : -1;
  }
  return out;
}

// --------------------------------------------- bench reads (no motion)

// 0 = the servo answered; otherwise a BusError code (1 = no reply).
int32_t rpc_ping(int32_t id) {
  note_call();
  if (!rpc::valid_id(id)) return code(duck::BusError::kBadArg);
  if (rpc::bus_owned()) return code(duck::BusError::kGated);
  return code(g_bus.ping(static_cast<uint8_t>(id)));
}

// [count, id, id, ...]: IDs that answer a ping in first..last (at most 32 per
// call: a silent ID costs a timeout).
ScanResult rpc_scan(int32_t first, int32_t last) {
  note_call();
  ScanResult found;
  found.fill(0);
  if (rpc::bus_owned()) return found;  // not while a loop owns the bus
  if (first < 0) first = 0;
  if (last > 253) last = 253;
  if (last > first + 31) last = first + 31;
  for (int32_t id = first; id <= last; ++id) {
    if (g_bus.ping(static_cast<uint8_t>(id)) == duck::BusError::kOk) found[1 + found[0]++] = id;
  }
  return found;
}

// [bus error, count, byte, byte, ...] - raw register read, up to 8 bytes.
ReadResult rpc_read(int32_t id, int32_t addr, int32_t n) {
  note_call();
  ReadResult out;
  out.fill(0);
  uint8_t data[8];
  if (!rpc::valid_id(id) || addr < 0 || addr > 79 || n < 1 || n > 8) {
    out[0] = code(duck::BusError::kBadArg);
    return out;
  }
  if (rpc::bus_owned()) {
    out[0] = code(duck::BusError::kGated);
    return out;
  }
  const duck::BusError e = g_bus.read(static_cast<uint8_t>(id), static_cast<uint8_t>(addr), data,
                                      static_cast<uint8_t>(n));
  out[0] = code(e);
  if (e == duck::BusError::kOk) {
    out[1] = n;
    for (int i = 0; i < n; ++i) out[2 + i] = data[i];
  }
  return out;
}

// For adapters that echo transmitted bytes (bench checklist).
bool rpc_set_echo(bool on) {
  note_call();
  g_bus.set_tx_echo(on);
  return on;
}

// --------------------------------------------- the LED matrix

// Text for the LED matrix: up to 32 characters. The display draws letters, digits and + - / . : ? ! % ( ) = # _ ',
// lower case as upper case, anything else as '?'. It stays up for 3 s after it was last sent (two passes if it
// scrolls), then the status ticker returns; send it again to keep it. A fault always shows instead. An empty string
// clears it. Returns how many characters were kept.
int32_t rpc_say(String text) {
  note_call();
  return status_display_say(text);
}

// --------------------------------------------- commands
// All return a duck::Result code (0 = ok).

int32_t rpc_heartbeat() {
  note_command();
  g_ctl.heartbeat(millis());
  return code(g_ctl.status().mode);
}

int32_t rpc_arm(uint32_t key) {
  note_command();
  // A bench servo has torque, or a bench fault is latched: clear it with duck_disarm first.
  if (!g_bench.idle()) return code(duck::Result::kBusy);
  return code(g_ctl.arm(key, millis(), g_imu));
}

int32_t rpc_disarm() {
  note_command();
  g_bench.disarm();
  g_ctl.disarm();
  return 0;
}

int32_t rpc_clear_fault() {
  note_command();
  return code(g_ctl.clear_fault());
}

int32_t rpc_stand(bool supported) {
  note_command();
  return code(g_ctl.stand(supported, millis()));
}

int32_t rpc_sit() {
  note_command();
  return code(g_ctl.sit(millis(), g_imu));
}

// 15 joint targets in degrees, joint order as in config/robot.json.
int32_t rpc_set_targets(Targets deg) {
  note_command();
  return code(g_ctl.set_targets(deg.data(), millis()));
}

// One control tick in one round trip: 15 joint targets in degrees x10 (integers are a third of the size of floats
// on the wire), answered with the state. The targets take exactly the path duck_set_targets takes: through the
// safety layer, and ignored unless the firmware is armed.
StateWords rpc_step(TargetWords deg10) {
  note_command();
  float deg[duck::kNumJoints];
  for (int j = 0; j < duck::kNumJoints; ++j) deg[j] = static_cast<float>(deg10[j]) * 0.1f;
  return state_words(code(g_ctl.set_targets(deg, millis())));
}

int32_t rpc_set_head(float neck_pitch, float head_pitch, float head_yaw, float head_roll, float beak) {
  note_command();
  return code(g_ctl.set_head(neck_pitch, head_pitch, head_yaw, head_roll, beak, millis()));
}

}  // namespace

void rpc_register() {
  // provide_safe: the handlers run from loop(), never in the middle of a control tick.
  Bridge.provide_safe("duck_info", rpc_info);
  Bridge.provide_safe("duck_status", rpc_status);
  Bridge.provide_safe("duck_joints", rpc_joints);
  Bridge.provide_safe("duck_state", rpc_state);
  Bridge.provide_safe("duck_setpoints", rpc_setpoints);
  Bridge.provide_safe("duck_health", rpc_health);
  Bridge.provide_safe("duck_ping", rpc_ping);
  Bridge.provide_safe("duck_scan", rpc_scan);
  Bridge.provide_safe("duck_read", rpc_read);
  Bridge.provide_safe("duck_set_echo", rpc_set_echo);
  Bridge.provide_safe("duck_say", rpc_say);
  Bridge.provide_safe("duck_heartbeat", rpc_heartbeat);
  Bridge.provide_safe("duck_arm", rpc_arm);
  Bridge.provide_safe("duck_disarm", rpc_disarm);
  Bridge.provide_safe("duck_clear_fault", rpc_clear_fault);
  Bridge.provide_safe("duck_stand", rpc_stand);
  Bridge.provide_safe("duck_sit", rpc_sit);
  Bridge.provide_safe("duck_set_targets", rpc_set_targets);
  Bridge.provide_safe("duck_step", rpc_step);
  Bridge.provide_safe("duck_set_head", rpc_set_head);
#if DUCK_BENCH_MOTION
  rpc::register_bench();
#endif
#if DUCK_ENABLE_EEPROM_WRITE
  rpc::register_provisioning();
#endif
}
