// SPDX-License-Identifier: Apache-2.0
// The Bridge functions of a provisioning build (DUCK_ENABLE_EEPROM_WRITE 1): servo EEPROM writes.
// Only in a provisioning build, only while disarmed, only with the arm key.
#include <Arduino.h>
#include <Arduino_RouterBridge.h>

#include <array>

#include "duck_build_config.h"
#ifndef DUCK_TICK_MS
#error "duck_build_config.h must be included before its switches are tested"
#endif

#if DUCK_ENABLE_EEPROM_WRITE

#include "duck_globals.h"
#include "rpc_util.h"
#include "status_display.h"

using rpc::code;

namespace {

// The result word when the request was refused before anything was sent (wrong key, not disarmed, bad argument).
constexpr int32_t kRefused = 255;

bool provisioning_ok(uint32_t key) {
  return key == duck::kArmKey && g_ctl.status().mode == duck::Mode::kDisarmed && g_bench.idle();
}

// Writes one joint's angle limits (from the joint table) into its servo and reads them back.
// [result, min step read back, max step read back]
std::array<int32_t, 3> rpc_write_limits(uint32_t key, int32_t joint) {
  note_command();
  std::array<int32_t, 3> out = {{kRefused, -1, -1}};
  if (!provisioning_ok(key) || joint < 0 || joint >= duck::kNumJoints) return out;
  const duck::RobotConfig& c = g_config;
  if (!duck::config_ready_for_motion(c)) return out;
  const int16_t a = duck::deg_to_step(c, joint, c.joints[joint].min_deg);
  const int16_t b = duck::deg_to_step(c, joint, c.joints[joint].max_deg);
  const uint16_t lo = static_cast<uint16_t>(a < b ? a : b), hi = static_cast<uint16_t>(a < b ? b : a);
  g_bus.allow_eeprom(true);
  const duck::BusError e = g_bus.write_angle_limits(c.joints[joint].servo_id, lo, hi);
  g_bus.allow_eeprom(false);
  uint16_t rlo = 0, rhi = 0;
  out[0] = code(e);
  if (g_bus.read_angle_limits(c.joints[joint].servo_id, &rlo, &rhi) == duck::BusError::kOk) {
    out[1] = rlo;
    out[2] = rhi;
  }
  return out;
}

// Writes the protection settings from config/robot.json into one servo and reads them back.
// [result, max temperature, unload condition, holding torque, protection time, overload torque]
std::array<int32_t, 6> rpc_write_protection(uint32_t key, int32_t id) {
  note_command();
  std::array<int32_t, 6> out = {{kRefused, -1, -1, -1, -1, -1}};
  if (!provisioning_ok(key) || !rpc::valid_id(id)) return out;
  const duck::ProtectionConfig& pc = g_config.protection;
  duck::Protection p;
  p.max_temperature_c = pc.max_temperature_c;
  p.unload_condition = pc.unload_condition;
  p.holding_torque_pct = pc.holding_torque_pct;
  p.protection_time_10ms = pc.protection_time_10ms;
  p.overload_torque_pct = pc.overload_torque_pct;
  g_bus.allow_eeprom(true);
  out[0] = code(g_bus.write_protection(static_cast<uint8_t>(id), p));
  g_bus.allow_eeprom(false);
  duck::Protection r;
  if (g_bus.read_protection(static_cast<uint8_t>(id), &r) == duck::BusError::kOk) {
    out[1] = r.max_temperature_c;
    out[2] = r.unload_condition;
    out[3] = r.holding_torque_pct;
    out[4] = r.protection_time_10ms;
    out[5] = r.overload_torque_pct;
  }
  return out;
}

// Changes a servo's ID. Have ONLY that servo on the bus.
int32_t rpc_set_id(uint32_t key, int32_t old_id, int32_t new_id) {
  note_command();
  if (!provisioning_ok(key) || !rpc::valid_id(old_id) || !rpc::valid_id(new_id)) return kRefused;
  g_bus.allow_eeprom(true);
  const duck::BusError e = g_bus.write_id(static_cast<uint8_t>(old_id), static_cast<uint8_t>(new_id));
  g_bus.allow_eeprom(false);
  return code(e);
}

}  // namespace

void rpc::register_provisioning() {
  Bridge.provide_safe("duck_write_limits", rpc_write_limits);
  Bridge.provide_safe("duck_write_protection", rpc_write_protection);
  Bridge.provide_safe("duck_set_id", rpc_set_id);
}

#endif  // DUCK_ENABLE_EEPROM_WRITE
