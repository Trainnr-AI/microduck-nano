// SPDX-License-Identifier: Apache-2.0
// The servo bus's bytes over the UART on D0 (RX) / D1 (TX).
#pragma once
#include <Arduino.h>

#include "src/duck/scs_bus.h"

// On UNO Q core 0.55 and later `Serial` is the Bridge monitor and D0/D1 is `Serial1`; on older cores (0.52 ships on
// some boards) D0/D1 is `Serial` and `Serial1` is the Bridge's own link, which must not be touched. The core tells us
// which.
#if defined(ARDUINO_HARDWARE_SERIAL)
#define DUCK_SERVO_SERIAL ARDUINO_HARDWARE_SERIAL
#define DUCK_SERVO_SERIAL_IS_SERIAL1 1
#else
#define DUCK_SERVO_SERIAL Serial
#define DUCK_SERVO_SERIAL_IS_SERIAL1 0
#endif

class UartTransport : public duck::Transport {
 public:
  void write(const uint8_t* data, size_t n) override {
    DUCK_SERVO_SERIAL.write(data, n);
    DUCK_SERVO_SERIAL.flush();
  }
  size_t read(uint8_t* data, size_t n, uint32_t timeout_ms) override {
    size_t got = 0;
    uint32_t t0 = millis();
    while (got < n) {
      const int c = DUCK_SERVO_SERIAL.read();
      if (c >= 0) {
        data[got++] = static_cast<uint8_t>(c);
        t0 = millis();
      } else if (millis() - t0 > timeout_ms) {
        break;
      }
    }
    return got;
  }
  void discard_input() override {
    while (DUCK_SERVO_SERIAL.read() >= 0) {
    }
  }
};
