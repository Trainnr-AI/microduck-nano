// SPDX-License-Identifier: Apache-2.0
#include "status_display.h"

#include <string.h>

#include "duck_build_config.h"
#ifndef DUCK_TICK_MS
#error "duck_build_config.h must be included before its switches are tested"
#endif
#if DUCK_STATUS_MATRIX
#include <Arduino_LED_Matrix.h>
#endif
#include "duck_globals.h"
#include "src/duck/display.h"

namespace {

#if DUCK_STATUS_MATRIX
Arduino_LED_Matrix g_matrix;
#endif
uint8_t g_frame[duck::kMatrixRows * duck::kMatrixCols];
uint8_t g_led3 = 0xFF, g_led4 = 0xFF;  // what the RGB LEDs show now; 0xFF = not yet written
// When the Linux side last called a function, and last sent a command; kNever until it does.
uint32_t g_last_call_ms = duck::kNever;
uint32_t g_last_command_ms = duck::kNever;
// Text from the Linux side for the matrix (duck_say), and when it arrived.
char g_message[duck::kMessageMax + 1] = "";
uint32_t g_message_ms = duck::kNever;

uint32_t ms_since(uint32_t then, uint32_t now) { return then == duck::kNever ? duck::kNever : now - then; }

// The RGB LEDs are wired active low.
void write_rgb(uint8_t colour, int pin_r, int pin_g, int pin_b) {
  digitalWrite(pin_r, (colour & duck::kRgbRed) ? LOW : HIGH);
  digitalWrite(pin_g, (colour & duck::kRgbGreen) ? LOW : HIGH);
  digitalWrite(pin_b, (colour & duck::kRgbBlue) ? LOW : HIGH);
}

}  // namespace

void note_call() { g_last_call_ms = millis(); }
void note_command() { g_last_call_ms = g_last_command_ms = millis(); }

void status_display_begin() {
  const int rgb_pins[6] = {LED3_R, LED3_G, LED3_B, LED4_R, LED4_G, LED4_B};
  for (int pin : rgb_pins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);  // off (active low)
  }
#if DUCK_STATUS_MATRIX
  g_matrix.begin();
  g_matrix.setGrayscaleBits(3);  // grey levels 0..7; set it: the boot animation leaves the driver at 8 bits
#endif
  memset(g_frame, 0xFF, sizeof(g_frame));  // differs from any real frame, so the first one is drawn
}

void status_display_update(uint32_t now) {
  duck::DisplayInput in;
  in.status = g_ctl.status();
  in.imu_present = g_imu_present;
  for (int j = 0; j < duck::kNumJoints; ++j) in.joint_answering[j] = g_ctl.feedback_valid(j);
  in.ms_since_call = ms_since(g_last_call_ms, now);
  in.ms_since_command = ms_since(g_last_command_ms, now);
  in.message = g_message;
  in.ms_since_message = ms_since(g_message_ms, now);
  in.bench = duck::shown_bench(g_bench.state(), g_bench2.state());
  duck::DisplayOutput out;
  duck::render_display(in, now, &out);
  if (memcmp(out.matrix, g_frame, sizeof(g_frame)) != 0) {
    memcpy(g_frame, out.matrix, sizeof(g_frame));
#if DUCK_STATUS_MATRIX
    g_matrix.draw(g_frame);
#endif
  }
  if (out.led3 != g_led3) {
    g_led3 = out.led3;
    write_rgb(g_led3, LED3_R, LED3_G, LED3_B);
  }
  if (out.led4 != g_led4) {
    g_led4 = out.led4;
    write_rgb(g_led4, LED4_R, LED4_G, LED4_B);
  }
}

int status_display_say(const String& text) {
  int n = 0;
  while (n < duck::kMessageMax && n < static_cast<int>(text.length()) && text[n] != '\0') {
    g_message[n] = text[n];
    ++n;
  }
  g_message[n] = '\0';
  g_message_ms = n ? millis() : duck::kNever;
  return n;
}
