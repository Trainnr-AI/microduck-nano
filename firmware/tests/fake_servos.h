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
  uint8_t data[8];
};

class FakeBus : public duck::Transport {
 public:
  FakeServo servo[254];
  std::vector<WriteLog> writes;
  std::vector<uint8_t> rx;  // bytes waiting to be read by the host
  size_t tx_bytes = 0;
  bool echo = false;
  uint8_t status_error = 0;  // error byte put into every reply

  FakeBus() {
    for (int i = 0; i < 254; ++i) servo[i].present = false;
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

  // Did any logged write touch torque-enable with a non-zero value or a goal register?
  bool any_motion_write() const {
    for (const WriteLog& w : writes) {
      for (uint8_t i = 0; i < w.n; ++i) {
        uint8_t a = w.addr + i;
        if (a == duck::scs::reg::kTorqueEnable && w.data[i] != 0) return true;
        if (a >= duck::scs::reg::kGoalPosition && a < duck::scs::reg::kLock) return true;
      }
    }
    return false;
  }
  bool any_eeprom_write() const {
    for (const WriteLog& w : writes) {
      if (w.addr < duck::scs::reg::kFirstRamRegister) return true;
      if (w.addr == duck::scs::reg::kLock && w.data[0] == 0) return true;
    }
    return false;
  }
  int torque_count() const {
    int c = 0;
    for (int i = 0; i < 254; ++i) c += servo[i].present && servo[i].torque();
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
    w.id = id; w.addr = addr; w.n = n < 8 ? n : 8;
    memcpy(w.data, data, w.n);
    writes.push_back(w);
    for (int s = 0; s < 254; ++s) {
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
    bool answers = id < 254 && servo[id].present && !servo[id].mute;
    switch (inst) {
      case duck::scs::kPing:
        if (answers) reply(id, nullptr, 0);
        break;
      case duck::scs::kRead:
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
