// SPDX-License-Identifier: Apache-2.0
// The board's LEDs as a status display (src/duck/display.h, docs/robot/status-lights.md): the glue to the hardware,
// the message from the Linux side, and when the Linux side last called.
#pragma once
#include <Arduino.h>
#include <stdint.h>

// The LEDs off and the matrix started. Call once, first thing in setup().
void status_display_begin();
// Draws what changed. Call every control tick.
void status_display_update(uint32_t now_ms);

// Every Bridge function calls one of these first: the link LED shows that the Linux side is talking, and whether it
// is sending commands.
void note_call();
void note_command();

// Text from the Linux side for the matrix (duck_say): up to kMessageMax characters, an empty string clears it.
// Returns how many characters were kept.
int status_display_say(const String& text);
