// SPDX-License-Identifier: Apache-2.0
// Simulated SCS servos behind the Transport interface, for host tests.
// Each servo has an 80-byte memory table and answers ping / read / write /
// sync write like the protocol documents say. A servo with torque on jumps to
// its goal position at once. Every write is logged so tests can prove that a
// given kind of write never left the firmware.
#pragma once
#include <stdint.h>
#include <string.h>

#include <vector>

#include "duck/scs_bus.h"

struct FakeServo {
  bool present = true;
  bool mute = false;  // powered but not answering (for comms-loss tests)
  uint8_t mem[80];
  FakeServo() {
    memset(mem, 0, sizeof(mem));
    mem[duck::scs::reg::kLock] = 1;
    set16(duck::scs::reg::kMinAngleLimit, 20);
    set16(duck::scs::reg::kMaxAngleLimit, 1003);
    mem[duck::scs::reg::kMaxTemperature] = 70;
    mem[duck::scs::reg::kHoldingTorque] = 20;
    mem[duck::scs::reg::kProtectionTime] = 200;
    mem[duck::scs::reg::kOverloadTorque] = 80;
    set16(duck::scs::reg::kPresentPosition, 512);
    mem[duck::scs::reg::kPresentVoltage] = 50;
    mem[duck::scs::reg::kPresentTemperature] = 30;
  }
  void set16(uint8_t addr, uint16_t v) { mem[addr] = v >> 8; mem[addr + 1] = v & 0xFF; }
  uint16_t get16(uint8_t addr) const { return (mem[addr] << 8) | mem[addr + 1]; }
  bool torque() const { return mem[duck::scs::reg::kTorqueEnable] == 1; }
  uint16_t position() const { return get16(duck::scs::reg::kPresentPosition); }
  void set_position(uint16_t p) { set16(duck::scs::reg::kPresentPosition, p); }
  void set_load(int16_t l) { set16(duck::scs::reg::kPresentLoad, l < 0 ? (uint16_t)((-l) | (1 << 10)) : (uint16_t)l); }
};

struct WriteLog {
  uint8_t id, addr, n;
  uint8_t data[16];  // ScsBus::write sends at most 16 bytes
};

// Every ID a servo can have, 0..253 (254 is the broadcast ID).
constexpr int kServoIds = duck::scs::kBroadcastId;

class FakeBus : public duck::Transport {
 public:
  FakeServo servo[kServoIds];
  std::vector<WriteLog> writes;
  std::vector<uint8_t> reads;  // the ID of every read request, answered or not
  std::vector<uint8_t> rx;  // bytes waiting to be read by the host
  size_t tx_bytes = 0;
  bool echo = false;
  uint8_t status_error = 0;  // error byte put into every reply

  FakeBus() {
    for (int i = 0; i < kServoIds; ++i) servo[i].present = false;
  }
  void add(uint8_t id) { servo[id].present = true; }

  void write(const uint8_t* d, size_t n) override {
    tx_bytes += n;
    if (echo) rx.insert(rx.end(), d, d + n);
    handle(d, n);
  }
  size_t read(uint8_t* d, size_t n, uint32_t) override {
    size_t k = n < rx.size() ? n : rx.size();
    memcpy(d, rx.data(), k);
    rx.erase(rx.begin(), rx.begin() + k);
    return k;
  }
  void discard_input() override { rx.clear(); }

  // Is `hit(address, value)` true for any byte of any logged write?
  template <class Hit>
  bool any_written_byte(Hit hit) const {
    for (const WriteLog& w : writes) {
      for (uint8_t i = 0; i < w.n; ++i) {
        if (hit(static_cast<uint8_t>(w.addr + i), w.data[i])) return true;
      }
    }
    return false;
  }
  // Torque-enable set non-zero, or a goal register written.
  bool any_motion_write() const {
    return any_written_byte([](uint8_t a, uint8_t v) {
      return (a == duck::scs::reg::kTorqueEnable && v != 0) ||
             (a >= duck::scs::reg::kGoalPosition && a < duck::scs::reg::kLock);
    });
  }
  // An EEPROM register written, or the EEPROM unlocked.
  bool any_eeprom_write() const {
    return any_written_byte([](uint8_t a, uint8_t v) {
      return a < duck::scs::reg::kFirstRamRegister || (a == duck::scs::reg::kLock && v == 0);
    });
  }
  int torque_count() const {
    int c = 0;
    for (int i = 0; i < kServoIds; ++i) c += servo[i].present && servo[i].torque();
    return c;
  }

 private:
  void reply(uint8_t id, const uint8_t* params, size_t n) {
    uint8_t p[64];
    p[0] = 0xFF; p[1] = 0xFF; p[2] = id; p[3] = n + 2; p[4] = status_error;
    memcpy(p + 5, params, n);
    p[5 + n] = duck::scs::checksum(p + 2, n + 3);
    rx.insert(rx.end(), p, p + n + 6);
  }
  void apply(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t n) {
    WriteLog w;
    w.id = id; w.addr = addr; w.n = n < sizeof(w.data) ? n : sizeof(w.data);
    memcpy(w.data, data, w.n);
    writes.push_back(w);
    for (int s = 0; s < kServoIds; ++s) {
      if (!servo[s].present || (id != duck::scs::kBroadcastId && id != s)) continue;
      FakeServo& sv = servo[s];
      for (uint8_t i = 0; i < n && addr + i < 80; ++i) {
        uint8_t a = addr + i;
        bool eeprom = a < duck::scs::reg::kFirstRamRegister;
        if (eeprom && sv.mem[duck::scs::reg::kLock] == 1) continue;  // locked: not kept
        sv.mem[a] = data[i];
      }
      if (sv.torque()) sv.set_position(sv.get16(duck::scs::reg::kGoalPosition));
    }
  }
  void handle(const uint8_t* d, size_t n) {
    if (n < 6 || d[0] != 0xFF || d[1] != 0xFF) return;
    uint8_t id = d[2], len = d[3], inst = d[4];
    if ((size_t)len + 4 != n || duck::scs::checksum(d + 2, len + 1) != d[n - 1]) return;
    const uint8_t* p = d + 5;
    bool answers = id < kServoIds && servo[id].present && !servo[id].mute;
    switch (inst) {
      case duck::scs::kPing:
        if (answers) reply(id, nullptr, 0);
        break;
      case duck::scs::kRead:
        reads.push_back(id);
        if (answers) reply(id, servo[id].mem + p[0], p[1]);
        break;
      case duck::scs::kWrite:
        apply(id, p[0], p + 1, len - 3);
        if (answers) reply(id, nullptr, 0);
        break;
      case duck::scs::kSyncWrite: {
        uint8_t addr = p[0], each = p[1];
        for (size_t k = 2; k + each < (size_t)len - 2; k += each + 1) apply(p[k], addr, p + k + 1, each);
        break;
      }
      default: break;
    }
  }
};
