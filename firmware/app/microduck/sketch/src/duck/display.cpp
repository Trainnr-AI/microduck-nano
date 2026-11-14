// SPDX-License-Identifier: Apache-2.0
#include "display.h"

#include "fmath.h"

namespace duck {

namespace {

// 3 x 5 font for ASCII 32..95 (lower case is drawn as upper case). Five rows of three bits, top row first, the
// leftmost column the high bit of each row. Characters without a glyph are drawn as '?'.
const uint16_t kFont[64] = {
    0x0000, 0x2482, 0x6282, 0x5F7D, 0x6282, 0x52A5, 0x6282, 0x2400,
    0x2922, 0x224A, 0x6282, 0x05D0, 0x0014, 0x01C0, 0x0002, 0x12A4,
    0x7B6F, 0x2C97, 0x73E7, 0x73CF, 0x5BC9, 0x79CF, 0x79EF, 0x7292,
    0x7BEF, 0x7BCF, 0x0410, 0x6282, 0x6282, 0x0E38, 0x6282, 0x6282,
    0x6282, 0x2BED, 0x6BAE, 0x3923, 0x6B6E, 0x79A7, 0x79A4, 0x396B,
    0x5BED, 0x7497, 0x126A, 0x5BAD, 0x4927, 0x5FED, 0x6B6D, 0x2B6A,
    0x6BA4, 0x2B73, 0x6BAD, 0x388E, 0x7492, 0x5B6F, 0x5B6A, 0x5BFD,
    0x5AAD, 0x5A92, 0x72A7, 0x6282, 0x6282, 0x6282, 0x6282, 0x0007,
};

uint16_t glyph(char c) {
  if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
  if (c < 32 || c > 95) c = '?';
  return kFont[c - 32];
}

const char* const kFaultNames[] = {"", "OVERLOAD", "HOT", "LOW VOLTS", "SERVO ERROR", "NO ANSWER", "FALL",
                                   "BUS WRITE"};
const char* const kJointNames[kNumJoints] = {
    "L HIP YAW", "L HIP ROLL", "L HIP PITCH", "L KNEE", "L ANKLE", "NECK", "HEAD PITCH", "HEAD YAW", "HEAD ROLL",
    "R HIP YAW", "R HIP ROLL", "R HIP PITCH", "R KNEE", "R ANKLE", "BEAK"};
const char* const kPostureNames[] = {"", "STAND", "SIT", "SITTING", "STANDING", "TO STAND"};

// Bounded text builder.
struct Text {
  char* buf;
  int n;
  void put(const char* s) {
    while (*s && n < kTextMax - 1) buf[n++] = *s++;
    buf[n] = '\0';
  }
  void put_int(long v, bool sign) {
    char digits[12];
    int k = 0;
    const bool neg = v < 0;
    unsigned long u = static_cast<unsigned long>(neg ? -v : v);
    do {
      digits[k++] = static_cast<char>('0' + u % 10);
      u /= 10;
    } while (u && k < 11);
    char s[14];
    int m = 0;
    if (neg) {
      s[m++] = '-';
    } else if (sign) {
      s[m++] = '+';
    }
    while (k) s[m++] = digits[--k];
    s[m] = '\0';
    put(s);
  }
};

int str_len(const char* s) {
  int n = 0;
  while (s && s[n]) ++n;
  return n;
}

// On for on_ms out of every period_ms.
bool blink(uint32_t now_ms, uint32_t period_ms, uint32_t on_ms) { return (now_ms % period_ms) < on_ms; }

void set(DisplayOutput* out, int row, int col, uint8_t level) {
  if (row < 0 || row >= kMatrixRows || col < 0 || col >= kMatrixCols) return;
  out->matrix[row * kMatrixCols + col] = level;
}

bool fresh(uint32_t ms_since, uint32_t limit) { return ms_since != kNever && ms_since <= limit; }

bool message_up(const DisplayInput& in) {
  if (in.message == nullptr || in.message[0] == '\0' || in.ms_since_message == kNever) return false;
  const uint32_t two_passes = 2u * static_cast<uint32_t>(text_width(in.message) + kMatrixCols) * kScrollMs;
  return in.ms_since_message <= (two_passes > kMessageHoldMs ? two_passes : kMessageHoldMs);
}

// Draws `text` in rows 0..4. Text that fits stands still, centred; longer text scrolls in from the right, `offset`
// columns along a loop of one blank screen followed by the text.
void draw_text(DisplayOutput* out, const char* text, uint32_t offset) {
  const int n = str_len(text);
  const int width = text_width(text);
  const bool scrolls = width > kMatrixCols;
  const int start = scrolls ? 0 : (kMatrixCols - width) / 2;  // where text that fits begins
  const uint32_t loop = static_cast<uint32_t>(kMatrixCols + n * 4);
  for (int x = 0; x < kMatrixCols; ++x) {
    // column within the text
    const int s = scrolls ? static_cast<int>((offset + static_cast<uint32_t>(x)) % loop) - kMatrixCols : x - start;
    if (s < 0) continue;
    const int ci = s / 4, cc = s % 4;
    if (ci >= n || cc == 3) continue;
    const uint16_t bits = glyph(text[ci]);
    for (int r = 0; r < 5; ++r) {
      if ((bits >> ((4 - r) * 3 + (2 - cc))) & 1u) set(out, r, x, kLedFull);
    }
  }
}

uint8_t mode_colour(const DisplayInput& in, uint32_t now_ms) {
  switch (in.status.mode) {
    case Mode::kFault:
      // Steady: a crashed sketch is the core blinking this same red LED, and must not look like a fault.
      return kRgbRed;
    case Mode::kArmed:
      // A supported stand (robot held off the floor) blinks slowly; otherwise steady.
      if (in.status.flags & kStatSupported) return blink(now_ms, 1000, 500) ? kRgbBlue : kRgbOff;
      return kRgbBlue;
    case Mode::kDisarmed:
    default:
      return blink(now_ms, 1000, 50) ? kRgbGreen : kRgbOff;
  }
}

uint8_t link_colour(const DisplayInput& in, uint32_t now_ms) {
  if (fresh(in.ms_since_command, kLinkCommandFreshMs)) return kRgbBlue;
  if (fresh(in.ms_since_call, kLinkReadFreshMs)) return kRgbGreen;
  if (in.ms_since_call != kNever) return blink(now_ms, 1000, 50) ? kRgbRed : kRgbOff;  // was talking, went silent
  return kRgbOff;                                                                    // nothing since power-up
}

uint8_t link_level(const DisplayInput& in, uint32_t now_ms) {
  if (fresh(in.ms_since_command, kLinkCommandFreshMs)) return kLedFull;
  if (fresh(in.ms_since_call, kLinkReadFreshMs)) return kLedMid;
  if (in.ms_since_call != kNever) return blink(now_ms, 1000, 50) ? kLedFull : 0;
  return 0;
}

uint8_t imu_level(bool present, bool ok) { return ok ? kLedFull : (present ? kLedDim : 0); }

// Where each joint's dot is, in firmware joint order: {row, col}.
const int8_t kJointDot[kNumJoints][2] = {
    {6, 0},  {6, 1},  {6, 2},  {6, 3},  {6, 4},  // left leg: hip yaw .. ankle
    {7, 4},  {7, 5},  {7, 6},  {7, 7},           // neck pitch, head pitch, head yaw, head roll
    {6, 8},  {6, 9},  {6, 10}, {6, 11}, {6, 12},  // right leg
    {7, 8},                                       // beak
};

}  // namespace

int text_width(const char* text) {
  const int n = str_len(text);
  return n ? n * 4 - 1 : 0;
}

int display_text(const DisplayInput& in, char* buf) {
  Text t = {buf, 0};
  buf[0] = '\0';
  const Status& s = in.status;
  if (s.mode == Mode::kFault) {
    t.put("FAULT ");
    const int f = static_cast<int>(s.fault);
    t.put(f > 0 && f < static_cast<int>(sizeof(kFaultNames) / sizeof(kFaultNames[0])) ? kFaultNames[f] : "?");
    if (s.fault_joint >= 0 && s.fault_joint < kNumJoints) {
      t.put(" ");
      t.put(kJointNames[s.fault_joint]);
    }
    return t.n;
  }
  if (message_up(in)) {
    t.put(in.message);
    return t.n;
  }
  // The status ticker.
  if (s.mode == Mode::kArmed) {
    t.put("ARMED ");
    const int p = static_cast<int>(s.posture);
    t.put(p > 0 && p < static_cast<int>(sizeof(kPostureNames) / sizeof(kPostureNames[0])) ? kPostureNames[p] : "?");
  } else {
    t.put("SAFE");
  }
  int answering = 0;
  for (int j = 0; j < kNumJoints; ++j) answering += in.joint_answering[j] ? 1 : 0;
  t.put("  SV ");
  t.put_int(answering, false);
  t.put("/");
  t.put_int(kNumJoints, false);
  if (!in.imu_present) {
    t.put("  NO IMU");
  } else if (!s.imu_ok) {
    t.put("  IMU ?");
  } else {
    t.put("  P");
    t.put_int(finitef(s.pitch_deg) ? round_to_long(s.pitch_deg) : 0, true);
    t.put(" R");
    t.put_int(finitef(s.roll_deg) ? round_to_long(s.roll_deg) : 0, true);
  }
  t.put(fresh(in.ms_since_call, kLinkReadFreshMs) ? "  LINUX OK" : "  NO LINUX");
  return t.n;
}

void render_display(const DisplayInput& in, uint32_t now_ms, DisplayOutput* out) {
  for (int i = 0; i < kMatrixRows * kMatrixCols; ++i) out->matrix[i] = 0;

  // Text. A message scrolls from its own start; the ticker and a fault scroll with the clock.
  char text[kTextMax];
  display_text(in, text);
  const bool is_message = in.status.mode != Mode::kFault && message_up(in);
  draw_text(out, text, (is_message ? in.ms_since_message : now_ms) / kScrollMs);

  // Servos.
  const bool fault_blink = blink(now_ms, 200, 100);
  for (int j = 0; j < kNumJoints; ++j) {
    uint8_t level = in.joint_answering[j] ? kLedFull : kLedDim;
    if (in.status.mode == Mode::kFault && in.status.fault_joint == j) level = fault_blink ? kLedFull : 0;
    set(out, kJointDot[j][0], kJointDot[j][1], level);
  }

  // Link, IMU (dim = the IMU answered but this reading is stale or its mounting is not calibrated; off = no IMU),
  // and the loop's own heartbeat.
  set(out, 6, 6, link_level(in, now_ms));
  set(out, 7, 0, imu_level(in.imu_present, in.status.imu_ok));
  set(out, 7, 1, imu_level(in.imu_present, in.status.gyro_ok));
  set(out, 7, 12, blink(now_ms, 1000, 500) ? kLedMid : 0);

  out->led3 = mode_colour(in, now_ms);
  out->led4 = link_colour(in, now_ms);
}

}  // namespace duck
