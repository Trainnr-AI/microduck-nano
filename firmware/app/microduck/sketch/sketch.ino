// SPDX-License-Identifier: Apache-2.0
// Microduck Nano - microcontroller (STM32) side.
//
// Owns everything time-critical and everything safety-related: the servo bus,
// the IMU, joint limits, interlocks, the balance law, fault handling. The
// Linux side only sends requests over the Bridge; every one of them goes
// through the same safety layer, and none of them can move a servo unless the
// firmware was built with DUCK_ENABLE_MOTION and then armed.
//
// Wiring assumed (firmware/README.md): servo adapter on D0 (RX) / D1 (TX),
// 3.3 V logic, 1 Mbaud; BNO086 on the Qwiic connector.

#include <Arduino.h>
#include <Arduino_RouterBridge.h>

#include <array>
#include <string.h>

#include "duck_build_config.h"
#if DUCK_STATUS_MATRIX
#include <Arduino_LED_Matrix.h>
#endif
#include "imu.h"
#include "src/duck/bench.h"
#include "src/duck/controller.h"
#include "src/duck/display.h"
#include "src/duck/fmath.h"
#include "src/duck/robot_config_data.h"

// The UART on D0/D1. On UNO Q core 0.55 and later `Serial` is the Bridge
// monitor and D0/D1 is `Serial1`; on older cores (0.52 ships on some boards)
// D0/D1 is `Serial` and `Serial1` is the Bridge's own link, which must not be
// touched. The core tells us which.
#if defined(ARDUINO_HARDWARE_SERIAL)
#define DUCK_SERVO_SERIAL ARDUINO_HARDWARE_SERIAL
#define DUCK_SERVO_SERIAL_IS_SERIAL1 1
#else
#define DUCK_SERVO_SERIAL Serial
#define DUCK_SERVO_SERIAL_IS_SERIAL1 0
#endif

#define DUCK_PROTOCOL_VERSION 3  // 3: the state message carries the gyro

namespace {

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

UartTransport g_uart;
duck::ScsBus g_bus(g_uart);
duck::Controller g_ctl(duck::kRobotConfig, g_bus, DUCK_ENABLE_MOTION != 0);
// One servo on its own, for the bench; it can only ever be armed in a bench build.
duck::BenchServo g_bench(duck::kRobotConfig, g_bus, DUCK_ENABLE_MOTION != 0 && DUCK_BENCH_MOTION != 0);
duck::ImuSample g_imu = {{0.0f, 0.0f, 1.0f}, false, 0, {0.0f, 0.0f, 0.0f}, false, 0};
bool g_imu_present = false;
uint32_t g_next_tick_ms = 0;
uint32_t g_tick_us_max = 0;

// The board's LEDs as a status display (src/duck/display.h, docs/robot/status-lights.md).
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

void note_call() { g_last_call_ms = millis(); }
void note_command() { g_last_call_ms = g_last_command_ms = millis(); }

uint32_t ms_since(uint32_t then, uint32_t now) { return then == duck::kNever ? duck::kNever : now - then; }

// The RGB LEDs are wired active low.
void write_rgb(uint8_t colour, int pin_r, int pin_g, int pin_b) {
  digitalWrite(pin_r, (colour & duck::kRgbRed) ? LOW : HIGH);
  digitalWrite(pin_g, (colour & duck::kRgbGreen) ? LOW : HIGH);
  digitalWrite(pin_b, (colour & duck::kRgbBlue) ? LOW : HIGH);
}

void update_display(uint32_t now) {
  duck::DisplayInput in;
  in.status = g_ctl.status();
  in.imu_present = g_imu_present;
  for (int j = 0; j < duck::kNumJoints; ++j) in.joint_answering[j] = g_ctl.feedback_valid(j);
  in.ms_since_call = ms_since(g_last_call_ms, now);
  in.ms_since_command = ms_since(g_last_command_ms, now);
  in.message = g_message;
  in.ms_since_message = ms_since(g_message_ms, now);
  in.bench = g_bench.state();
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

// Fixed-size arrays only: on older UNO Q cores a sketch cannot link the parts
// of the C++ library that std::vector's size checks pull in.
typedef std::array<int32_t, 7> Info;
typedef std::array<int32_t, 10> StatusWords;
typedef std::array<int32_t, duck::kNumJoints> JointWords;
typedef std::array<int32_t, duck::kNumJoints * 3> HealthWords;
typedef std::array<int32_t, 33> ScanResult;   // [count, id, id, ...]
typedef std::array<int32_t, 10> ReadResult;   // [bus error, count, byte, ...]
typedef std::array<float, duck::kNumJoints> Targets;
typedef std::array<int32_t, duck::kNumJoints> TargetWords;          // degrees x10
typedef std::array<int32_t, 10 + duck::kNumJoints + 3> StateWords;  // see fill_state()
typedef std::array<int32_t, 12> BenchWords;                         // see rpc_bench_step()

int32_t tenths(float v) { return static_cast<int32_t>(duck::round_to_long(v * 10.0f)); }

}  // namespace

// ----------------------------------------------------------------- RPC: info

// [protocol version, motion build, eeprom build, joint table ready, imu present, servo UART is Serial1, bench build]
Info rpc_info() {
  note_call();
  Info out = {{DUCK_PROTOCOL_VERSION, DUCK_ENABLE_MOTION, DUCK_ENABLE_EEPROM_WRITE,
               duck::config_ready_for_motion(duck::kRobotConfig) ? 1 : 0, g_imu_present ? 1 : 0,
               DUCK_SERVO_SERIAL_IS_SERIAL1, DUCK_BENCH_MOTION}};
  return out;
}

// [mode, posture, fault, fault joint, flags, imu ok, pitch x10, roll x10, balance x10, worst tick in us]
StatusWords rpc_status() {
  note_call();
  const duck::Status s = g_ctl.status();
  StatusWords out = {{static_cast<int32_t>(s.mode), static_cast<int32_t>(s.posture),
                      static_cast<int32_t>(s.fault), s.fault_joint, s.flags, s.imu_ok ? 1 : 0,
                      tenths(s.pitch_deg), tenths(s.roll_deg), tenths(s.balance_deg),
                      static_cast<int32_t>(g_tick_us_max)}};
  g_tick_us_max = 0;
  return out;
}

// Measured joint angles, degrees x10; -32768 where the servo did not answer.
JointWords rpc_joints() {
  note_call();
  JointWords out;
  for (int j = 0; j < duck::kNumJoints; ++j) {
    out[j] = g_ctl.feedback_valid(j) ? tenths(g_ctl.measured_deg()[j]) : -32768;
  }
  return out;
}

// The whole state a control loop needs, in one reply, so a tick costs one round trip on the link instead of several:
// [result, mode, posture, fault, fault joint, flags, imu ok, pitch x10, roll x10, worst tick in us,
//  then one measured joint angle per joint, degrees x10, -32768 where the servo did not answer,
//  then the body angular velocity about x (forward), y (left), z (up), degrees per second x10, -32768 each when the
//  gyro is not measured (absent, stale, or the IMU's mounting not calibrated)].
// `result` is the duck::Result of the command this reply answers (0 for a plain read). Protocol 3; protocol 2 was
// the same without the last three words.
static StateWords fill_state(int32_t result) {
  const duck::Status s = g_ctl.status();
  StateWords out;
  out[0] = result;
  out[1] = static_cast<int32_t>(s.mode);
  out[2] = static_cast<int32_t>(s.posture);
  out[3] = static_cast<int32_t>(s.fault);
  out[4] = s.fault_joint;
  out[5] = s.flags;
  out[6] = s.imu_ok ? 1 : 0;
  out[7] = tenths(s.pitch_deg);
  out[8] = tenths(s.roll_deg);
  out[9] = static_cast<int32_t>(g_tick_us_max);
  g_tick_us_max = 0;
  for (int j = 0; j < duck::kNumJoints; ++j) {
    out[10 + j] = g_ctl.feedback_valid(j) ? tenths(g_ctl.measured_deg()[j]) : -32768;
  }
  for (int i = 0; i < 3; ++i) out[10 + duck::kNumJoints + i] = s.gyro_ok ? tenths(s.gyro_dps[i]) : -32768;
  return out;
}

StateWords rpc_state() {
  note_call();
  return fill_state(0);
}

// Commanded joint angles after the safety layer, degrees x10.
JointWords rpc_setpoints() {
  note_call();
  JointWords out;
  for (int j = 0; j < duck::kNumJoints; ++j) out[j] = tenths(g_ctl.setpoint_deg()[j]);
  return out;
}

// Per joint: load (0.1 %, signed), voltage (0.1 V), temperature (deg C); 45 numbers.
HealthWords rpc_health() {
  note_call();
  HealthWords out;
  for (int j = 0; j < duck::kNumJoints; ++j) {
    const duck::Feedback& fb = g_ctl.feedback(j);
    const bool ok = g_ctl.feedback_valid(j);
    out[j * 3] = ok ? fb.load : -32768;
    out[j * 3 + 1] = ok ? fb.voltage_dv : -1;
    out[j * 3 + 2] = ok ? fb.temperature_c : -1;
  }
  return out;
}

// ------------------------------------------------- RPC: bench reads (no motion)

// 0 = the servo answered; otherwise a BusError code (1 = no reply).
int32_t rpc_ping(int32_t id) {
  note_call();
  if (id < 0 || id > 253) return static_cast<int32_t>(duck::BusError::kBadArg);
  if (g_ctl.status().mode == duck::Mode::kArmed || g_bench.armed()) return static_cast<int32_t>(duck::BusError::kGated);
  return static_cast<int32_t>(g_bus.ping(static_cast<uint8_t>(id)));
}

// [count, id, id, ...]: IDs that answer a ping in first..last (at most 32 per
// call: a silent ID costs a timeout).
ScanResult rpc_scan(int32_t first, int32_t last) {
  note_call();
  ScanResult found;
  found.fill(0);
  if (g_ctl.status().mode == duck::Mode::kArmed || g_bench.armed()) return found;  // not while a loop owns the bus
  if (first < 0) first = 0;
  if (last > 253) last = 253;
  if (last > first + 31) last = first + 31;
  for (int32_t id = first; id <= last; ++id) {
    if (g_bus.ping(static_cast<uint8_t>(id)) == duck::BusError::kOk) found[1 + found[0]++] = id;
  }
  return found;
}

// [bus error, count, byte, byte, ...] - raw register read, up to 8 bytes.
ReadResult rpc_read(int32_t id, int32_t addr, int32_t n) {
  note_call();
  ReadResult out;
  out.fill(0);
  uint8_t data[8];
  if (id < 0 || id > 253 || addr < 0 || addr > 79 || n < 1 || n > 8) {
    out[0] = static_cast<int32_t>(duck::BusError::kBadArg);
    return out;
  }
  if (g_ctl.status().mode == duck::Mode::kArmed || g_bench.armed()) {
    out[0] = static_cast<int32_t>(duck::BusError::kGated);
    return out;
  }
  const duck::BusError e = g_bus.read(static_cast<uint8_t>(id), static_cast<uint8_t>(addr), data,
                                      static_cast<uint8_t>(n));
  out[0] = static_cast<int32_t>(e);
  if (e == duck::BusError::kOk) {
    out[1] = n;
    for (int i = 0; i < n; ++i) out[2 + i] = data[i];
  }
  return out;
}

// Text for the LED matrix: up to 32 characters. The display draws letters, digits and + - / . : ? ! % ( ) = # _ ',
// lower case as upper case, anything else as '?'. It stays up for 3 s after it was last sent (two passes if it
// scrolls), then the status ticker returns; send it again to keep it. A fault always shows instead. An empty string
// clears it. Returns how many characters were kept.
int32_t rpc_say(String text) {
  note_call();
  int n = 0;
  while (n < duck::kMessageMax && n < static_cast<int>(text.length()) && text[n] != '\0') {
    g_message[n] = text[n];
    ++n;
  }
  g_message[n] = '\0';
  g_message_ms = n ? millis() : duck::kNever;
  return n;
}

// For adapters that echo transmitted bytes (bench checklist).
bool rpc_set_echo(bool on) {
  note_call();
  g_bus.set_tx_echo(on);
  return on;
}

// ------------------------------------------------------------- RPC: commands
// All return a duck::Result code (0 = ok).

int32_t rpc_heartbeat() {
  note_command();
  g_ctl.heartbeat(millis());
  return static_cast<int32_t>(g_ctl.status().mode);
}

int32_t rpc_arm(uint32_t key) {
  note_command();
  // A bench servo has torque, or a bench fault is latched: clear it with duck_disarm first.
  if (!g_bench.idle()) return static_cast<int32_t>(duck::Result::kBusy);
  return static_cast<int32_t>(g_ctl.arm(key, millis(), g_imu)); }

int32_t rpc_disarm() {
  note_command();
  g_bench.disarm();
  g_ctl.disarm();
  return 0;
}

int32_t rpc_clear_fault() {
  note_command();
  return static_cast<int32_t>(g_ctl.clear_fault()); }

int32_t rpc_stand(bool supported) {
  note_command();
  return static_cast<int32_t>(g_ctl.stand(supported, millis())); }

int32_t rpc_sit() {
  note_command();
  return static_cast<int32_t>(g_ctl.sit(millis(), g_imu)); }

// 15 joint targets in degrees, joint order as in config/robot.json.
int32_t rpc_set_targets(Targets deg) {
  note_command();
  return static_cast<int32_t>(g_ctl.set_targets(deg.data(), millis()));
}

// One control tick in one round trip: 15 joint targets in degrees x10 (integers are a third of the size of floats
// on the wire), answered with the state. The targets take exactly the path duck_set_targets takes: through the
// safety layer, and ignored unless the firmware is armed.
StateWords rpc_step(TargetWords deg10) {
  note_command();
  float deg[duck::kNumJoints];
  for (int j = 0; j < duck::kNumJoints; ++j) deg[j] = static_cast<float>(deg10[j]) * 0.1f;
  return fill_state(static_cast<int32_t>(g_ctl.set_targets(deg, millis())));
}

int32_t rpc_set_head(float neck_pitch, float head_pitch, float head_yaw, float head_roll, float beak) {
  note_command();
  return static_cast<int32_t>(g_ctl.set_head(neck_pitch, head_pitch, head_yaw, head_roll, beak, millis()));
}

#if DUCK_BENCH_MOTION
// ------------------------------------------- RPC: one servo on the bench (bench build)

// Arm one joint's servo on its own (src/duck/bench.h). Refused unless the robot is disarmed.
int32_t rpc_bench_arm(uint32_t key, int32_t joint) {
  note_command();
  if (g_ctl.status().mode != duck::Mode::kDisarmed) return static_cast<int32_t>(duck::Result::kBusy);
  return static_cast<int32_t>(g_bench.arm(key, joint, millis()));
}

// One bench tick in one round trip: the target in degrees x10 out (-32768: no new target, just read), the servo's
// state back:
// [result, mode (0 off, 1 armed, 2 fault), fault, joint, read ok, position deg x10, speed deg/s x10,
//  load 0.1 % (signed, in the joint's direction), voltage 0.1 V, temperature C, read time ms, setpoint deg x10].
// Position, speed and load are as of the servo's last read, made at `read time` on the microcontroller's clock.
BenchWords rpc_bench_step(int32_t deg10) {
  note_command();
  const duck::Result r = deg10 == -32768 ? duck::Result::kOk : g_bench.set_target(static_cast<float>(deg10) * 0.1f, millis());
  const duck::BenchState s = g_bench.state();
  BenchWords out;
  out.fill(0);
  out[0] = static_cast<int32_t>(r);
  out[1] = static_cast<int32_t>(s.mode);
  out[2] = static_cast<int32_t>(s.fault);
  out[3] = s.joint;
  out[4] = s.read_ok ? 1 : 0;
  if (s.joint >= 0) {
    const duck::JointConfig& j = duck::kRobotConfig.joints[s.joint];
    out[5] = tenths(duck::step_to_deg(duck::kRobotConfig, s.joint, s.feedback.position));
    out[6] = tenths(static_cast<float>(s.feedback.speed) * j.direction / duck::kRobotConfig.servo.steps_per_deg);
    out[7] = s.feedback.load * j.direction;
  }
  out[8] = s.feedback.voltage_dv;
  out[9] = s.feedback.temperature_c;
  out[10] = static_cast<int32_t>(s.read_ms);
  out[11] = tenths(s.setpoint_deg);
  return out;
}

int32_t rpc_bench_disarm() {
  note_command();
  g_bench.disarm();
  return 0;
}
#endif  // DUCK_BENCH_MOTION

#if DUCK_ENABLE_EEPROM_WRITE
// --------------------------------------- RPC: provisioning (servo EEPROM writes)
// Only in a provisioning build, only while disarmed, only with the arm key.

static bool provisioning_ok(uint32_t key) {
  return key == duck::kArmKey && g_ctl.status().mode == duck::Mode::kDisarmed && g_bench.idle();
}

// Writes one joint's angle limits (from the joint table) into its servo and reads them back.
// [result, min step read back, max step read back]
std::array<int32_t, 3> rpc_write_limits(uint32_t key, int32_t joint) {
  note_command();
  std::array<int32_t, 3> out = {{255, -1, -1}};
  if (!provisioning_ok(key) || joint < 0 || joint >= duck::kNumJoints) return out;
  const duck::RobotConfig& c = duck::kRobotConfig;
  if (!duck::config_ready_for_motion(c)) return out;
  const int16_t a = duck::deg_to_step(c, joint, c.joints[joint].min_deg);
  const int16_t b = duck::deg_to_step(c, joint, c.joints[joint].max_deg);
  const uint16_t lo = static_cast<uint16_t>(a < b ? a : b), hi = static_cast<uint16_t>(a < b ? b : a);
  g_bus.allow_eeprom(true);
  const duck::BusError e = g_bus.write_angle_limits(c.joints[joint].servo_id, lo, hi);
  g_bus.allow_eeprom(false);
  uint16_t rlo = 0, rhi = 0;
  out[0] = static_cast<int32_t>(e);
  if (g_bus.read_angle_limits(c.joints[joint].servo_id, &rlo, &rhi) == duck::BusError::kOk) {
    out[1] = rlo;
    out[2] = rhi;
  }
  return out;
}

// Writes the protection settings from config/robot.json into one servo and reads them back.
// [result, max temperature, unload condition, holding torque, protection time, overload torque]
std::array<int32_t, 6> rpc_write_protection(uint32_t key, int32_t id) {
  note_command();
  std::array<int32_t, 6> out = {{255, -1, -1, -1, -1, -1}};
  if (!provisioning_ok(key) || id < 0 || id > 253) return out;
  const duck::ProtectionConfig& pc = duck::kRobotConfig.protection;
  duck::Protection p;
  p.max_temperature_c = pc.max_temperature_c;
  p.unload_condition = pc.unload_condition;
  p.holding_torque_pct = pc.holding_torque_pct;
  p.protection_time_10ms = pc.protection_time_10ms;
  p.overload_torque_pct = pc.overload_torque_pct;
  g_bus.allow_eeprom(true);
  out[0] = static_cast<int32_t>(g_bus.write_protection(static_cast<uint8_t>(id), p));
  g_bus.allow_eeprom(false);
  duck::Protection r;
  if (g_bus.read_protection(static_cast<uint8_t>(id), &r) == duck::BusError::kOk) {
    out[1] = r.max_temperature_c;
    out[2] = r.unload_condition;
    out[3] = r.holding_torque_pct;
    out[4] = r.protection_time_10ms;
    out[5] = r.overload_torque_pct;
  }
  return out;
}

// Changes a servo's ID. Have ONLY that servo on the bus.
int32_t rpc_set_id(uint32_t key, int32_t old_id, int32_t new_id) {
  note_command();
  if (!provisioning_ok(key) || old_id < 0 || old_id > 253 || new_id < 0 || new_id > 253) return 255;
  g_bus.allow_eeprom(true);
  const duck::BusError e = g_bus.write_id(static_cast<uint8_t>(old_id), static_cast<uint8_t>(new_id));
  g_bus.allow_eeprom(false);
  return static_cast<int32_t>(e);
}
#endif  // DUCK_ENABLE_EEPROM_WRITE

// ---------------------------------------------------------------------------

void setup() {
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

  DUCK_SERVO_SERIAL.begin(duck::kRobotConfig.servo.baud);
  g_imu_present = imu_begin();

  Bridge.begin(DUCK_BRIDGE_BAUD);
  // provide_safe: the handlers run from loop(), never in the middle of a control tick.
  Bridge.provide_safe("duck_info", rpc_info);
  Bridge.provide_safe("duck_status", rpc_status);
  Bridge.provide_safe("duck_joints", rpc_joints);
  Bridge.provide_safe("duck_state", rpc_state);
  Bridge.provide_safe("duck_setpoints", rpc_setpoints);
  Bridge.provide_safe("duck_health", rpc_health);
  Bridge.provide_safe("duck_ping", rpc_ping);
  Bridge.provide_safe("duck_scan", rpc_scan);
  Bridge.provide_safe("duck_read", rpc_read);
  Bridge.provide_safe("duck_set_echo", rpc_set_echo);
  Bridge.provide_safe("duck_say", rpc_say);
  Bridge.provide_safe("duck_heartbeat", rpc_heartbeat);
  Bridge.provide_safe("duck_arm", rpc_arm);
  Bridge.provide_safe("duck_disarm", rpc_disarm);
  Bridge.provide_safe("duck_clear_fault", rpc_clear_fault);
  Bridge.provide_safe("duck_stand", rpc_stand);
  Bridge.provide_safe("duck_sit", rpc_sit);
  Bridge.provide_safe("duck_set_targets", rpc_set_targets);
  Bridge.provide_safe("duck_step", rpc_step);
  Bridge.provide_safe("duck_set_head", rpc_set_head);
#if DUCK_BENCH_MOTION
  Bridge.provide_safe("duck_bench_arm", rpc_bench_arm);
  Bridge.provide_safe("duck_bench_step", rpc_bench_step);
  Bridge.provide_safe("duck_bench_disarm", rpc_bench_disarm);
#endif
#if DUCK_ENABLE_EEPROM_WRITE
  Bridge.provide_safe("duck_write_limits", rpc_write_limits);
  Bridge.provide_safe("duck_write_protection", rpc_write_protection);
  Bridge.provide_safe("duck_set_id", rpc_set_id);
#endif
  g_next_tick_ms = millis();
}

void loop() {
  const uint32_t now = millis();

  float up[3], gyro[3];
  const int fresh = imu_poll(up, gyro);
  if (fresh & kImuFreshUp) {
    for (int i = 0; i < 3; ++i) g_imu.up[i] = up[i];
    g_imu.valid = true;
    g_imu.t_ms = now;
  }
  if (fresh & kImuFreshGyro) {
    for (int i = 0; i < 3; ++i) g_imu.gyro_dps[i] = gyro[i];
    g_imu.gyro_valid = true;
    g_imu.gyro_t_ms = now;
  }

  if (static_cast<int32_t>(now - g_next_tick_ms) >= 0) {
    g_next_tick_ms += DUCK_TICK_MS;
    if (static_cast<int32_t>(now - g_next_tick_ms) > 5 * DUCK_TICK_MS) g_next_tick_ms = now + DUCK_TICK_MS;
    const uint32_t t0 = micros();
    g_ctl.tick(now, g_imu);
    g_bench.tick(now, g_ctl.status().mode == duck::Mode::kArmed);
    update_display(now);  // counted in the worst tick: the display must not cost the control loop
    const uint32_t dt = micros() - t0;
    if (dt > g_tick_us_max) g_tick_us_max = dt;
  }
}
