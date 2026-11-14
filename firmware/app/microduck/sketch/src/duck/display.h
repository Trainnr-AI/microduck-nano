// SPDX-License-Identifier: Apache-2.0
// What the board's own LEDs show: the 8 x 13 matrix and the two RGB LEDs the microcontroller drives (LED3, LED4).
// Hardware-independent: render_display() fills a frame and says which colours are on; the sketch copies that to the
// hardware. The legend is in docs/robot/status-lights.md.
//
//   col  0 1 2 3 4 5 6 7 8 9 10 11 12
//   row0 T T T T T T T T T T T  T  T     T: text, 3 x 5 letters, scrolling when it does not fit:
//   row1 T T T T T T T T T T T  T  T        a fault, else a message from the Linux side, else the status ticker
//   row2 T T T T T T T T T T T  T  T
//   row3 T T T T T T T T T T T  T  T
//   row4 T T T T T T T T T T T  T  T
//   row5 . . . . . . . . . . .  .  .
//   row6 l l l l l . K . r r r  r  r     l / r: left / right leg servos, hip yaw .. ankle   K: link with Linux
//   row7 G W . . h h h h h . .  .  A     h: neck pitch, head pitch, head yaw, head roll, beak
//                                        G: gravity  W: gyro  A: the firmware loop is running (blinks)
//   A servo dot: bright = answering, dim = silent, blinking = it caused the fault.
#pragma once
#include <stdint.h>

#include "bench.h"
#include "controller.h"

namespace duck {

constexpr int kMatrixRows = 8;
constexpr int kMatrixCols = 13;
// Grey levels of the matrix (3 bits). The driver shows 5..7 alike; 1..4 are 1/23, 1/15, 1/5 and 1/3 on.
constexpr uint8_t kLedFull = 7;
constexpr uint8_t kLedMid = 4;
constexpr uint8_t kLedDim = 2;

// Colour bits of an RGB LED.
enum Rgb : uint8_t {
  kRgbOff = 0,
  kRgbRed = 1,
  kRgbGreen = 2,
  kRgbYellow = 3,
  kRgbBlue = 4,
  kRgbMagenta = 5,
  kRgbCyan = 6,
  kRgbWhite = 7,
};

// How long a call from the Linux side counts as "the link is alive". The app's watcher reads once a second.
constexpr uint32_t kLinkReadFreshMs = 1500;
constexpr uint32_t kLinkCommandFreshMs = 500;
constexpr uint32_t kNever = 0xFFFFFFFFu;

// Text: one column every kScrollMs; a message from Linux stays up for kMessageHoldMs after it was last sent, or for
// two passes if it is long. The Linux side re-sends a message it wants to keep up.
constexpr uint32_t kScrollMs = 70;
constexpr uint32_t kMessageHoldMs = 3000;
constexpr int kMessageMax = 32;

struct DisplayInput {
  Status status;
  bool imu_present;                  // the IMU answered at boot
  bool joint_answering[kNumJoints];  // firmware joint order
  uint32_t ms_since_call;            // since the Linux side last called anything; kNever if it never has
  uint32_t ms_since_command;         // since it last sent a command (targets, step, heartbeat, arm ...); or kNever
  const char* message;               // text from the Linux side (duck_say), or nullptr
  uint32_t ms_since_message;         // since it was sent; kNever if never
  BenchState bench;                  // a bench build's single servo (mode kOff otherwise)
};

struct DisplayOutput {
  uint8_t matrix[kMatrixRows * kMatrixCols];  // row by row, grey levels 0..7
  uint8_t led3;                               // Rgb bits. LED3: the safety mode (magenta: one servo armed on a bench)
  uint8_t led4;                               // Rgb bits. LED4: the link with the Linux side
};

void render_display(const DisplayInput& in, uint32_t now_ms, DisplayOutput* out);

// The text render_display() puts up for this input (for tests and for duck_client's status line). Returns its
// length; `text` must hold at least kTextMax bytes.
constexpr int kTextMax = 96;
int display_text(const DisplayInput& in, char* text);

// Width in columns of `text` in the 3 x 5 font (4 columns per letter, the last spacing column not counted).
int text_width(const char* text);

}  // namespace duck
