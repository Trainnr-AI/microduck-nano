// SPDX-License-Identifier: Apache-2.0
// SCS0009 bus driver on top of an abstract byte transport, so the same code
// runs on the STM32 (UART) and in host tests (simulated servos).
//
// SAFETY GATES. Every write goes through one function that sorts it by
// register address:
//   - anything that can make a servo move or hold (torque enable = on, goal
//     position / time / speed) is refused unless allow_motion(true) was called;
//   - anything stored in the servo's EEPROM (ID, baud, angle limits,
//     protection settings, the EEPROM unlock) is refused unless
//     allow_eeprom(true) was called.
// Both gates are closed after construction. Reads, pings and "torque off" are
// always allowed.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "scs_protocol.h"

namespace duck {

class Transport {
 public:
  virtual ~Transport() {}
  virtual void write(const uint8_t* data, size_t n) = 0;
  // Reads up to n bytes; returns as soon as n bytes arrived or timeout_ms passed
  // without a new byte. Returns the number of bytes read.
  virtual size_t read(uint8_t* data, size_t n, uint32_t timeout_ms) = 0;
  virtual void discard_input() = 0;
};

enum class BusError : uint8_t { kOk = 0, kNoReply, kBadReply, kWrongId, kGated, kBadArg };

struct Feedback {
  int16_t position;    // steps
  int16_t speed;       // steps/s, signed
  int16_t load;        // 0.1 % of PWM, signed
  uint8_t voltage_dv;  // 0.1 V
  uint8_t temperature_c;
  uint8_t error;       // error byte of the reply (scs::kErr* bits)
};

// The last read of one servo. A failed read clears `ok` and keeps the feedback it had.
struct ServoReading {
  Feedback fb;
  bool ok;      // the last read answered
  uint32_t ms;  // when `fb` was read (the caller's clock)
  void got(const Feedback& f, uint32_t now_ms) {
    fb = f;
    ok = true;
    ms = now_ms;
  }
};

struct Protection {    // all [T] registers: unconfirmed addresses
  uint8_t max_temperature_c, unload_condition, holding_torque_pct, protection_time_10ms, overload_torque_pct;
};

class ScsBus {
 public:
  explicit ScsBus(Transport& transport) : io_(transport) {}

  void allow_motion(bool on) { motion_ok_ = on; }
  void allow_eeprom(bool on) { eeprom_ok_ = on; }
  bool motion_allowed() const { return motion_ok_; }
  bool eeprom_allowed() const { return eeprom_ok_; }

  // Some half-duplex adapters echo every transmitted byte back to the host.
  // Whether the Waveshare Bus Servo Adapter (A) does is UNCONFIRMED; set this
  // after the check in docs/robot/bench-checklist.md.
  void set_tx_echo(bool on) { tx_echo_ = on; }
  void set_reply_timeout_ms(uint32_t ms) { timeout_ms_ = ms; }

  // --- always allowed ---
  BusError ping(uint8_t id, uint8_t* error = nullptr);
  // Bench diagnostic: sends a PING to `id` and returns every byte that arrives within wait_ms, as it came (an echo
  // of the six bytes sent, a reply, noise, or nothing). Tells a dead wire from a silent servo.
  size_t raw_ping(uint8_t id, uint8_t* rx, size_t cap, uint32_t wait_ms);
  BusError read(uint8_t id, uint8_t addr, uint8_t* data, uint8_t n, uint8_t* error = nullptr);
  BusError read_feedback(uint8_t id, Feedback* fb);  // position .. temperature in one read
  BusError read_angle_limits(uint8_t id, uint16_t* min_step, uint16_t* max_step);
  BusError read_protection(uint8_t id, Protection* p);
  BusError torque_off(uint8_t id);
  void torque_off_all();  // broadcast, no reply expected

  // --- need allow_motion(true) ---
  BusError torque_on(uint8_t id);
  BusError write_goal(uint8_t id, uint16_t position, uint16_t time_ms, uint16_t speed);
  BusError sync_write_goal(const uint8_t* ids, const uint16_t* positions, size_t count, uint16_t time_ms,
                           uint16_t speed);

  // --- need allow_eeprom(true); each one unlocks, writes, locks again ---
  BusError write_angle_limits(uint8_t id, uint16_t min_step, uint16_t max_step);
  BusError write_protection(uint8_t id, const Protection& p);
  BusError write_id(uint8_t id, uint8_t new_id);

  // Generic gated write (used by the functions above).
  BusError write(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t n);

 private:
  struct EepromChunk {
    uint8_t addr;
    const uint8_t* data;
    uint8_t n;
  };
  BusError write_eeprom(uint8_t id, const EepromChunk* chunks, size_t count, uint8_t lock_id);
  bool write_permitted(uint8_t addr, const uint8_t* data, uint8_t n) const;
  void send(const uint8_t* pkt, size_t n);
  BusError receive(uint8_t id, scs::Reply* reply);

  Transport& io_;
  bool motion_ok_ = false;
  bool eeprom_ok_ = false;
  bool tx_echo_ = false;
  uint32_t timeout_ms_ = 3;
  static constexpr uint32_t kEepromReplyTimeoutMs = 60;  // for writes into a servo's permanent memory
};

}  // namespace duck
