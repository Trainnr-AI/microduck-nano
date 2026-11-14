// SPDX-License-Identifier: Apache-2.0
// Feetech SCS serial bus protocol (SCSCL memory table, as used by the SCS0009):
// packet building and reply parsing. Pure functions, no I/O.
//
// Packet:  0xFF 0xFF ID LEN INST PARAM... CHECKSUM
//   LEN = number of params + 2, CHECKSUM = ~(ID + LEN + INST + params) & 0xFF.
// Reply:   0xFF 0xFF ID LEN ERROR PARAM... CHECKSUM
// Two-byte values are sent HIGH BYTE FIRST on the SCS series (Feetech's own
// library sets End = 1 for SCSCL; Waveshare's table says the same). The STS /
// SMS series are the other way round - do not reuse this for them.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace duck {
namespace scs {

constexpr uint8_t kBroadcastId = 0xFE;

enum Instruction : uint8_t {
  kPing = 0x01, kRead = 0x02, kWrite = 0x03, kRegWrite = 0x04, kAction = 0x05, kSyncWrite = 0x83
};

// Register addresses (decimal).
//   [L] = present in Feetech's official library header (SCSCL.h): reliable.
//   [T] = only in the public memory tables (Waveshare "SC Servo - Memory Table
//         Explanation", aifitlab "SCSCL Memory Table Analysis"). Both tables
//         agree on the decimal address, but one of them prints a wrong hex
//         value for 39 and both call address 48 two different things, so
//         every [T] register is UNCONFIRMED until read back on a real SCS0009.
namespace reg {
constexpr uint8_t kId = 5;                 // [L] EEPROM
constexpr uint8_t kBaudRate = 6;           // [L] EEPROM, 0 = 1 Mbaud
constexpr uint8_t kMinAngleLimit = 9;      // [L] EEPROM, 2 bytes
constexpr uint8_t kMaxAngleLimit = 11;     // [L] EEPROM, 2 bytes
constexpr uint8_t kMaxTemperature = 13;    // [T] EEPROM, deg C, factory 70
constexpr uint8_t kMaxVoltage = 14;        // [T] EEPROM, 0.1 V
constexpr uint8_t kMinVoltage = 15;        // [T] EEPROM, 0.1 V
constexpr uint8_t kMaxTorque = 16;         // [T] EEPROM, 2 bytes, 0.1 %
constexpr uint8_t kUnloadCondition = 19;   // [T] EEPROM, bits: 32 overload, 4 overheat, 1 voltage
constexpr uint8_t kLedAlarm = 20;          // [T] EEPROM
constexpr uint8_t kHoldingTorque = 37;     // [T] EEPROM, %, factory 20
constexpr uint8_t kProtectionTime = 38;    // [T] EEPROM, 10 ms, factory 200
constexpr uint8_t kOverloadTorque = 39;    // [T] EEPROM, %, factory 80
constexpr uint8_t kTorqueEnable = 40;      // [L] RAM, 0 = off, 1 = on
constexpr uint8_t kGoalPosition = 42;      // [L] RAM, 2 bytes
constexpr uint8_t kGoalTime = 44;          // [L] RAM, 2 bytes, ms
constexpr uint8_t kGoalSpeed = 46;         // [L] RAM, 2 bytes, steps/s
constexpr uint8_t kLock = 48;              // [L] RAM, 1 = EEPROM locked, 0 = writes are kept
constexpr uint8_t kPresentPosition = 56;   // [L] RAM, 2 bytes
constexpr uint8_t kPresentSpeed = 58;      // [L] RAM, 2 bytes, bit 15 = direction
constexpr uint8_t kPresentLoad = 60;       // [L] RAM, 2 bytes, 0.1 % of PWM, bit 10 = direction
constexpr uint8_t kPresentVoltage = 62;    // [L] RAM, 0.1 V
constexpr uint8_t kPresentTemperature = 63;// [L] RAM, deg C
constexpr uint8_t kStatus = 65;            // [T] RAM, same bits as the reply's error byte
constexpr uint8_t kMoving = 66;            // [L] RAM
constexpr uint8_t kFirstRamRegister = 40;  // below this: EEPROM
}  // namespace reg

// Bits of the error byte in a reply / register 65 ([T], unconfirmed).
constexpr uint8_t kErrVoltage = 0x01;
constexpr uint8_t kErrOverheat = 0x04;
constexpr uint8_t kErrOverload = 0x20;

constexpr size_t kMaxPacket = 128;

uint8_t checksum(const uint8_t* id_len_inst_params, size_t n);

// Builders return the packet length, or 0 if it does not fit in `cap`.
size_t build_packet(uint8_t id, uint8_t inst, const uint8_t* params, size_t n, uint8_t* out, size_t cap);
size_t build_ping(uint8_t id, uint8_t* out, size_t cap);
size_t build_read(uint8_t id, uint8_t addr, uint8_t len, uint8_t* out, size_t cap);
size_t build_write(uint8_t id, uint8_t addr, const uint8_t* data, size_t n, uint8_t* out, size_t cap);
// data = count blocks of len_each bytes, one per id.
size_t build_sync_write(uint8_t addr, uint8_t len_each, const uint8_t* ids, const uint8_t* data, size_t count,
                        uint8_t* out, size_t cap);

enum class Parse : uint8_t { kOk, kIncomplete, kBadChecksum, kBadLength };

struct Reply {
  uint8_t id;
  uint8_t error;
  uint8_t n;            // number of parameter bytes
  uint8_t params[32];
};

// Finds the first 0xFF 0xFF header in buf and parses the reply after it.
// kIncomplete = more bytes are needed.
Parse parse_reply(const uint8_t* buf, size_t n, Reply* out);

inline void put_u16(uint8_t* p, uint16_t v) { p[0] = static_cast<uint8_t>(v >> 8); p[1] = static_cast<uint8_t>(v & 0xFF); }
inline uint16_t get_u16(const uint8_t* p) { return static_cast<uint16_t>((p[0] << 8) | p[1]); }
// Sign-and-magnitude values: `sign_bit` set means negative.
inline int16_t decode_signed(uint16_t raw, uint8_t sign_bit) {
  const uint16_t mask = static_cast<uint16_t>(1u << sign_bit);
  return (raw & mask) ? static_cast<int16_t>(-static_cast<int16_t>(raw & ~mask)) : static_cast<int16_t>(raw);
}

}  // namespace scs
}  // namespace duck
