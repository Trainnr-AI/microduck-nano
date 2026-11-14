// SPDX-License-Identifier: Apache-2.0
#include "scs_bus.h"

namespace duck {

using namespace scs;

bool ScsBus::write_permitted(uint8_t addr, const uint8_t* data, uint8_t n) const {
  // Look at every byte the write would touch.
  for (uint8_t i = 0; i < n; ++i) {
    const uint8_t a = static_cast<uint8_t>(addr + i);
    const uint8_t v = data[i];
    if (a < reg::kFirstRamRegister) {            // EEPROM area
      if (!eeprom_ok_) return false;
    } else if (a == reg::kTorqueEnable) {        // off is always fine
      if (v != 0 && !motion_ok_) return false;
    } else if (a >= reg::kGoalPosition && a < reg::kLock) {  // goal position / time / speed
      if (!motion_ok_) return false;
    } else if (a == reg::kLock) {                // unlocking the EEPROM
      if (v == 0 && !eeprom_ok_) return false;
    } else if (a == 41) {
      return false;                              // undefined register: never written
    }
  }
  return true;
}

void ScsBus::send(const uint8_t* pkt, size_t n) {
  io_.discard_input();
  io_.write(pkt, n);
  if (tx_echo_) {
    uint8_t echo[kMaxPacket];
    io_.read(echo, n, timeout_ms_);
  }
}

BusError ScsBus::receive(uint8_t id, Reply* reply) {
  uint8_t buf[48];
  size_t have = 0;
  for (;;) {
    // Ask for the smallest reply first, then for whatever the length byte says.
    size_t want = 6;
    if (have >= 4) {
      size_t i = 0;
      while (i + 1 < have && !(buf[i] == 0xFF && buf[i + 1] == 0xFF)) ++i;
      while (i + 2 < have && buf[i + 2] == 0xFF) ++i;
      if (i + 3 < have) want = i + 4 + buf[i + 3];
    }
    if (want > sizeof(buf)) return BusError::kBadReply;
    if (want <= have) want = have + 1;
    if (want > sizeof(buf)) return BusError::kBadReply;
    const size_t got = io_.read(buf + have, want - have, timeout_ms_);
    have += got;
    const Parse r = parse_reply(buf, have, reply);
    if (r == Parse::kOk) return reply->id == id ? BusError::kOk : BusError::kWrongId;
    if (r != Parse::kIncomplete) return BusError::kBadReply;
    if (got == 0) return have == 0 ? BusError::kNoReply : BusError::kBadReply;
  }
}

BusError ScsBus::ping(uint8_t id, uint8_t* error) {
  if (id >= kBroadcastId) return BusError::kBadArg;
  uint8_t pkt[8];
  send(pkt, build_ping(id, pkt, sizeof(pkt)));
  Reply r;
  const BusError e = receive(id, &r);
  if (e == BusError::kOk && error) *error = r.error;
  return e;
}

BusError ScsBus::read(uint8_t id, uint8_t addr, uint8_t* data, uint8_t n, uint8_t* error) {
  if (id >= kBroadcastId || n == 0 || n > sizeof(Reply().params)) return BusError::kBadArg;
  uint8_t pkt[8];
  send(pkt, build_read(id, addr, n, pkt, sizeof(pkt)));
  Reply r;
  const BusError e = receive(id, &r);
  if (e != BusError::kOk) return e;
  if (r.n != n) return BusError::kBadReply;
  for (uint8_t i = 0; i < n; ++i) data[i] = r.params[i];
  if (error) *error = r.error;
  return BusError::kOk;
}

BusError ScsBus::read_u8(uint8_t id, uint8_t addr, uint8_t* value) { return read(id, addr, value, 1); }

BusError ScsBus::read_u16(uint8_t id, uint8_t addr, uint16_t* value) {
  uint8_t b[2];
  const BusError e = read(id, addr, b, 2);
  if (e == BusError::kOk) *value = get_u16(b);
  return e;
}

BusError ScsBus::read_feedback(uint8_t id, Feedback* fb) {
  uint8_t b[8];  // 56..63
  uint8_t err = 0;
  const BusError e = read(id, reg::kPresentPosition, b, 8, &err);
  if (e != BusError::kOk) return e;
  fb->position = static_cast<int16_t>(get_u16(b));
  fb->speed = decode_signed(get_u16(b + 2), 15);
  fb->load = decode_signed(get_u16(b + 4), 10);
  fb->voltage_dv = b[6];
  fb->temperature_c = b[7];
  fb->error = err;
  return BusError::kOk;
}

BusError ScsBus::read_angle_limits(uint8_t id, uint16_t* min_step, uint16_t* max_step) {
  uint8_t b[4];
  const BusError e = read(id, reg::kMinAngleLimit, b, 4);
  if (e == BusError::kOk) {
    *min_step = get_u16(b);
    *max_step = get_u16(b + 2);
  }
  return e;
}

BusError ScsBus::read_protection(uint8_t id, Protection* p) {
  BusError e = read_u8(id, reg::kMaxTemperature, &p->max_temperature_c);
  if (e == BusError::kOk) e = read_u8(id, reg::kUnloadCondition, &p->unload_condition);
  uint8_t b[3];
  if (e == BusError::kOk) e = read(id, reg::kHoldingTorque, b, 3);
  if (e == BusError::kOk) {
    p->holding_torque_pct = b[0];
    p->protection_time_10ms = b[1];
    p->overload_torque_pct = b[2];
  }
  return e;
}

BusError ScsBus::write(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t n) {
  if (n == 0 || n > 16 || id == 0xFF) return BusError::kBadArg;
  if (!write_permitted(addr, data, n)) return BusError::kGated;
  uint8_t pkt[32];
  send(pkt, build_write(id, addr, data, n, pkt, sizeof(pkt)));
  if (id == kBroadcastId) return BusError::kOk;
  Reply r;
  return receive(id, &r);
}

BusError ScsBus::torque_off(uint8_t id) {
  const uint8_t off = 0;
  return write(id, reg::kTorqueEnable, &off, 1);
}

void ScsBus::torque_off_all() {
  const uint8_t off = 0;
  write(kBroadcastId, reg::kTorqueEnable, &off, 1);
}

BusError ScsBus::torque_on(uint8_t id) {
  if (id >= kBroadcastId) return BusError::kBadArg;  // never broadcast "torque on"
  const uint8_t on = 1;
  return write(id, reg::kTorqueEnable, &on, 1);
}

BusError ScsBus::write_goal(uint8_t id, uint16_t position, uint16_t time_ms, uint16_t speed) {
  if (id >= kBroadcastId || position > 1023) return BusError::kBadArg;
  uint8_t b[6];
  put_u16(b, position);
  put_u16(b + 2, time_ms);
  put_u16(b + 4, speed);
  return write(id, reg::kGoalPosition, b, 6);
}

BusError ScsBus::sync_write_goal(const uint8_t* ids, const uint16_t* positions, size_t count, uint16_t time_ms,
                                 uint16_t speed) {
  if (count == 0 || count > 16) return BusError::kBadArg;
  if (!motion_ok_) return BusError::kGated;
  uint8_t data[16 * 6];
  for (size_t i = 0; i < count; ++i) {
    if (ids[i] >= kBroadcastId || positions[i] > 1023) return BusError::kBadArg;
    put_u16(data + i * 6, positions[i]);
    put_u16(data + i * 6 + 2, time_ms);
    put_u16(data + i * 6 + 4, speed);
  }
  uint8_t pkt[kMaxPacket];
  const size_t n = build_sync_write(reg::kGoalPosition, 6, ids, data, count, pkt, sizeof(pkt));
  if (n == 0) return BusError::kBadArg;
  send(pkt, n);
  return BusError::kOk;  // sync write is a broadcast: no reply
}

// EEPROM writes: unlock (48 = 0), write, lock (48 = 1), as Feetech's library does.
BusError ScsBus::write_angle_limits(uint8_t id, uint16_t min_step, uint16_t max_step) {
  if (id >= kBroadcastId || min_step >= max_step || max_step > 1023) return BusError::kBadArg;
  if (!eeprom_ok_) return BusError::kGated;
  const uint8_t unlock = 0, lock = 1;
  uint8_t b[4];
  put_u16(b, min_step);
  put_u16(b + 2, max_step);
  BusError e = write(id, reg::kLock, &unlock, 1);
  if (e == BusError::kOk) e = write(id, reg::kMinAngleLimit, b, 4);
  const BusError e2 = write(id, reg::kLock, &lock, 1);
  return e != BusError::kOk ? e : e2;
}

BusError ScsBus::write_protection(uint8_t id, const Protection& p) {
  if (id >= kBroadcastId) return BusError::kBadArg;
  if (!eeprom_ok_) return BusError::kGated;
  const uint8_t unlock = 0, lock = 1;
  const uint8_t tail[3] = {p.holding_torque_pct, p.protection_time_10ms, p.overload_torque_pct};
  BusError e = write(id, reg::kLock, &unlock, 1);
  if (e == BusError::kOk) e = write(id, reg::kMaxTemperature, &p.max_temperature_c, 1);
  if (e == BusError::kOk) e = write(id, reg::kUnloadCondition, &p.unload_condition, 1);
  if (e == BusError::kOk) e = write(id, reg::kHoldingTorque, tail, 3);
  const BusError e2 = write(id, reg::kLock, &lock, 1);
  return e != BusError::kOk ? e : e2;
}

BusError ScsBus::write_id(uint8_t id, uint8_t new_id) {
  if (id >= kBroadcastId || new_id >= 254) return BusError::kBadArg;
  if (!eeprom_ok_) return BusError::kGated;
  const uint8_t unlock = 0, lock = 1;
  BusError e = write(id, reg::kLock, &unlock, 1);
  if (e == BusError::kOk) e = write(id, reg::kId, &new_id, 1);
  // The servo answers the lock under its new ID.
  const BusError e2 = write(new_id, reg::kLock, &lock, 1);
  return e != BusError::kOk ? e : e2;
}

}  // namespace duck
