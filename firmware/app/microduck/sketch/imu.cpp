// SPDX-License-Identifier: Apache-2.0
// BNO086 over Qwiic, with the SparkFun library. First run with a sensor on 2026-11-18 (7Semi breakout, 0x4B):
// gravity and gyro both arrive at 100 reports a second, and the board lying flat reads "up" on the sensor's +z.
//
// The bus runs at 100 kHz, not 400. Over Qwiic there is no interrupt line, so nothing tells the host when the
// sensor has its next bytes ready. At 400 kHz this microcontroller reads back-to-back so fast (20 reads in 13 ms)
// that the sensor answers some of them with an empty header in the middle of a packet; the library takes that for
// data and its start-up handshake fails every time. At 100 kHz each read takes long enough and it starts first
// time. duck_imu_probe shows what the sensor sends, read by read.
//
// Checked by hand the same day (tools/imu_watch.py): on the 7Semi breakout the sensor's x runs along the board's
// long side, y along its short side, z out of the component side; tipping the board about x and about y turns
// gravity and the gyro the same, right-handed way, and a flat turn anticlockwise seen from above reads positive z.
// The gyro's scale was not checked against a known angle. Still to do (bench checklist, step 3): the mounting
// matrix, once the IMU is fixed in the robot.
#include "imu.h"

#include <Arduino.h>

#include "duck_build_config.h"

#if DUCK_IMU_BNO08X

#include <Wire.h>

#include "SparkFun_BNO08x_Arduino_Library.h"
#include "src/duck/fmath.h"

static BNO08x g_imu;
static bool g_started = false;
static bool g_gyro_on = false;
static int g_gravity_reports = 0, g_gyro_reports = 0, g_other_reports = 0, g_resets = 0;

static int count_up(int n) { return n < 1000000 ? n + 1 : n; }

// ---- Freeing a stuck bus.
// When the microcontroller restarts in the middle of a read (every firmware reload does that), the sensor is left
// holding the data line low, waiting for clock pulses that never come; every transfer then times out after half a
// second until the sensor loses power. The cure is the standard one: clock the bus by hand until the sensor lets
// go, then send a STOP. The Qwiic bus is I2C4 on PD12 (clock) and PD13 (data), which the Arduino core does not
// offer as digital pins, so the two pins are driven through the port's registers for a fraction of a millisecond
// and then given back to the I2C peripheral exactly as they were.
struct GpioPort {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR;
};
static GpioPort* const kQwiicPort = reinterpret_cast<GpioPort*>(0x42020C00u);  // GPIOD on the STM32U585
constexpr uint32_t kQwiicScl = 12, kQwiicSda = 13;
static int g_bus_freed = 0;       // times a held bus was freed
static int g_bus_last = 0;        // the last attempt: 0 = the lines were free, 1 = freed, 2 = still held

static int free_qwiic_bus() {
  GpioPort* const g = kQwiicPort;
  const uint32_t scl = 1u << kQwiicScl, sda = 1u << kQwiicSda;
  const uint32_t moder = g->MODER, otyper = g->OTYPER;
  g->BSRR = scl | sda;                // released (high) before they become outputs
  g->OTYPER = otyper | scl | sda;     // open drain: a line is only ever pulled low, never driven high
  g->MODER = (moder & ~((3u << (2 * kQwiicScl)) | (3u << (2 * kQwiicSda)))) | (1u << (2 * kQwiicScl)) |
             (1u << (2 * kQwiicSda));
  delayMicroseconds(20);
  int result = 0;
  if ((g->IDR & (scl | sda)) != (scl | sda)) {
    result = 2;
    for (int i = 0; i < 18 && result == 2; ++i) {  // up to two bytes of clock pulses
      g->BSRR = scl << 16;
      delayMicroseconds(10);
      g->BSRR = scl;
      delayMicroseconds(10);
      if ((g->IDR & (scl | sda)) == (scl | sda)) result = 1;
    }
  }
  // A STOP, so every device on the bus knows the transfer is over: data rises while the clock is high.
  g->BSRR = scl << 16;
  delayMicroseconds(10);
  g->BSRR = sda << 16;
  delayMicroseconds(10);
  g->BSRR = scl;
  delayMicroseconds(10);
  g->BSRR = sda;
  delayMicroseconds(10);
  g->OTYPER = otyper;
  g->MODER = moder;
  g_bus_last = result;
  if (result == 1 && g_bus_freed < 1000000) ++g_bus_freed;
  return result;
}

static int probe(uint8_t address) {
  Wire1.beginTransmission(address);
  return Wire1.endTransmission() == 0 ? 1 : 0;
}

// Gravity and the calibrated gyro (bias removed by the sensor), both at the control rate. Gravity decides whether the
// IMU counts as present: the firmware's own safety needs only it. A gyro that will not start just never reports, and
// the state message says "not measured".
static bool enable_reports() {
  const bool gravity = g_imu.enableGravity(DUCK_TICK_MS);
  g_gyro_on = g_imu.enableGyro(DUCK_TICK_MS);
  return gravity;
}

// Where the last start attempt stopped: 0 = not tried, 1 = the library's begin() failed (no answer, or the SHTP
// handshake did not complete), 2 = begin() worked but the gravity report was refused, 3 = started.
static int g_start_stage = 0;
static int g_start_attempts = 0;

bool imu_begin() {
  Wire1.begin();  // the Qwiic connector is Wire1 on the UNO Q (3.3 V)
  free_qwiic_bus();        // a sensor left mid-read by the restart: see above
  Wire1.setClock(100000);  // not 400000: see the top of this file. Also restarts the peripheral after the above
  for (int attempt = 0; attempt < 3 && !g_started; ++attempt) {
    ++g_start_attempts;
    if (!g_imu.begin(DUCK_IMU_I2C_ADDRESS, Wire1)) {
      g_start_stage = 1;
      if (probe(DUCK_IMU_I2C_ADDRESS) == 0) break;  // nothing there: no point in trying again
      delay(100);
      continue;
    }
    g_started = enable_reports();
    g_start_stage = g_started ? 3 : 2;
  }
  return g_started;
}

bool imu_retry(uint32_t now_ms) {
  static uint32_t last_try_ms = 0;
  if (g_started) return true;
  if (now_ms - last_try_ms < 2000) return false;
  last_try_ms = now_ms;
  if (probe(DUCK_IMU_I2C_ADDRESS) == 0) {
    // Nothing answers: unplugged (the probe was quick), or the bus is held (it took half a second). Try to free it;
    // the next try, two seconds on, finds out.
    free_qwiic_bus();
    Wire1.setClock(100000);
    return false;
  }
  ++g_start_attempts;
  if (!g_imu.begin(DUCK_IMU_I2C_ADDRESS, Wire1)) {
    g_start_stage = 1;
    return false;
  }
  g_started = enable_reports();
  g_start_stage = g_started ? 3 : 2;
  return g_started;
}

int imu_poll(float up[3], float gyro_dps[3]) {
  if (!g_started) return 0;
  if (g_imu.wasReset()) {  // the sensor forgets its reports after a reset
    g_resets = count_up(g_resets);
    enable_reports();
  }
  int fresh = 0;
  // Drain what is waiting; keep the newest sample of each report.
  for (int i = 0; i < 8 && g_imu.getSensorEvent(); ++i) {
    const uint8_t id = g_imu.getSensorEventID();
    if (id == SENSOR_REPORTID_GRAVITY) {
      const float x = g_imu.getGravityX(), y = g_imu.getGravityY(), z = g_imu.getGravityZ();
      const float n = duck::sqrt_f(x * x + y * y + z * z);
      if (n < 1.0f) continue;  // not a plausible gravity vector (m/s^2)
      up[0] = x / n;
      up[1] = y / n;
      up[2] = z / n;
      fresh |= kImuFreshUp;
      g_gravity_reports = count_up(g_gravity_reports);
    } else if (id == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
      const float k = 57.2957795f;  // rad/s -> deg/s
      const float x = g_imu.getGyroX() * k, y = g_imu.getGyroY() * k, z = g_imu.getGyroZ() * k;
      if (!duck::finitef(x) || !duck::finitef(y) || !duck::finitef(z)) continue;
      gyro_dps[0] = x;
      gyro_dps[1] = y;
      gyro_dps[2] = z;
      fresh |= kImuFreshGyro;
      g_gyro_reports = count_up(g_gyro_reports);
    } else {
      g_other_reports = count_up(g_other_reports);
    }
  }
  return fresh;
}

ImuDiag imu_diag() {
  return {1, probe(0x4A), probe(0x4B), g_started ? 1 : 0, g_gyro_on ? 1 : 0,
          g_gravity_reports, g_gyro_reports, g_other_reports, g_resets, g_start_stage, g_start_attempts,
          g_bus_freed, g_bus_last};
}

// One read of n bytes; fills h[] with the first four (the SHTP header). False if the read failed.
static bool read_packet_start(size_t n, uint8_t h[4]) {
  if (Wire1.requestFrom(static_cast<uint8_t>(DUCK_IMU_I2C_ADDRESS), n) != n) return false;
  for (size_t i = 0; i < n; ++i) {
    const int c = Wire1.read();
    if (i < 4) h[i] = static_cast<uint8_t>(c);
  }
  return true;
}

void imu_probe(int out[kImuProbeWords]) {
  for (int i = 0; i < kImuProbeWords; ++i) out[i] = 0;
  if (g_started) return;
  const uint8_t soft_reset[] = {5, 0, 1, 0, 1};  // SHTP: length 5, channel 1 (executable), sequence 0, "reset"
  Wire1.beginTransmission(static_cast<uint8_t>(DUCK_IMU_I2C_ADDRESS));
  Wire1.write(soft_reset, sizeof(soft_reset));
  out[0] = Wire1.endTransmission() == 0 ? 1 : 0;
  delay(300);
  // Read as the library does: a 4-byte header, then the packet in 32-byte reads, each of which starts with a
  // header again. One entry per read: the header's length word as sent (bit 15 = continuation) and its channel.
  const uint32_t t0 = micros();
  int n = 0;
  int remaining = 0;  // cargo bytes of the current packet still to read; 0 = at a packet boundary
  bool first = true;
  while (n < kImuProbeReads) {
    uint8_t h[4] = {0, 0, 0, 0};
    size_t want = 4;
    if (remaining > 0) want = static_cast<size_t>(first ? (remaining < 32 ? remaining : 32)
                                                        : (remaining + 4 < 32 ? remaining + 4 : 32));
    const bool ok = read_packet_start(want, h);
    const int word = h[0] | (h[1] << 8);
    out[3 + 2 * n] = ok ? word : -1;
    out[4 + 2 * n] = ok ? h[2] : -1;
    ++n;
    if (!ok) break;
    if (remaining == 0) {
      remaining = word & 0x7FFF;  // the whole packet, header included, is read again from its start
      first = true;
      if (remaining == 0) delay(5);  // nothing waiting
    } else {
      remaining -= first ? static_cast<int>(want) : static_cast<int>(want) - 4;
      first = false;
    }
  }
  out[1] = n;
  out[2] = static_cast<int>((micros() - t0) / 1000);
}

#else  // no IMU in this build

bool imu_begin() { return false; }
bool imu_retry(uint32_t now_ms) {
  (void)now_ms;
  return false;
}
ImuDiag imu_diag() { return {0, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; }
void imu_probe(int out[kImuProbeWords]) {
  for (int i = 0; i < kImuProbeWords; ++i) out[i] = 0;
}
int imu_poll(float up[3], float gyro_dps[3]) {
  (void)up;
  (void)gyro_dps;
  return 0;
}

#endif
