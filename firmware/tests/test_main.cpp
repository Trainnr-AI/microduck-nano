// SPDX-License-Identifier: Apache-2.0
// Host unit tests for the portable firmware core. Build and run: make test
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "duck/bench.h"
#include "duck/controller.h"
#include "duck/display.h"
#include "duck/fmath.h"
#include "duck/motion.h"
#include "duck/robot_config.h"
#include "duck/robot_config_data.h"
#include "duck/safety.h"
#include "duck/scs_bus.h"
#include "duck/scs_protocol.h"
#include "fake_servos.h"

using namespace duck;

static int g_checks = 0, g_failed = 0, g_tests = 0;
#define CHECK(cond)                                                             \
  do {                                                                          \
    ++g_checks;                                                                 \
    if (!(cond)) { ++g_failed; printf("    FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)
#define CHECK_NEAR(a, b, tol)                                                   \
  do {                                                                          \
    ++g_checks;                                                                 \
    if (!(fabs((double)(a) - (double)(b)) <= (tol))) {                          \
      ++g_failed;                                                               \
      printf("    FAIL %s:%d  %s = %g, expected %g +- %g\n", __FILE__, __LINE__, #a, (double)(a), (double)(b), (double)(tol)); \
    }                                                                           \
  } while (0)
#define RUN(fn)                                                                 \
  do { int before = g_failed; ++g_tests; fn(); printf("%s %s\n", g_failed == before ? "ok  " : "FAIL", #fn); } while (0)

// The shipped config has placeholders (no servo IDs, nothing calibrated).
// Tests that need a robot that can arm use this filled-in copy.
static RobotConfig calibrated_config() {
  RobotConfig c = kRobotConfig;
  for (int i = 0; i < kNumJoints; ++i) {
    c.joints[i].servo_id = static_cast<uint8_t>(i + 1);
    c.joints[i].calibrated = true;
    c.joints[i].direction = (i % 2) ? -1 : 1;
  }
  c.imu.calibrated = true;
  return c;
}

static ImuSample imu_at(float pitch_deg, float roll_deg, uint32_t t_ms) {
  const float d2r = 3.14159265f / 180.0f;
  ImuSample s;
  s.up[0] = sinf(pitch_deg * d2r);
  s.up[1] = sinf(roll_deg * d2r);
  s.up[2] = sqrtf(fmaxf(0.0f, 1.0f - s.up[0] * s.up[0] - s.up[1] * s.up[1]));
  s.valid = true;
  s.t_ms = t_ms;
  s.gyro_dps[0] = s.gyro_dps[1] = s.gyro_dps[2] = 0.0f;
  s.gyro_valid = false;
  s.gyro_t_ms = 0;
  return s;
}

// A pose (14 joints) as a full joint vector, the beak at 0.
static void pose_to_joints(const float pose[kNumPoseJoints], float q[kNumJoints]) {
  for (int j = 0; j < kNumPoseJoints; ++j) q[j] = pose[j];
  q[kBeak] = 0.0f;
}

static void place_servos(FakeBus& bus, const RobotConfig& c, const float pose[kNumPoseJoints]) {
  for (int j = 0; j < kNumJoints; ++j) {
    bus.add(c.joints[j].servo_id);
    float deg = j < kNumPoseJoints ? pose[j] : 0.0f;
    bus.servo[c.joints[j].servo_id].set_position(static_cast<uint16_t>(deg_to_step(c, j, deg)));
  }
}

// ---------------------------------------------------------------- protocol

static void test_protocol_packets() {
  uint8_t b[64];
  // PING id 1: FF FF 01 02 01 FB (Feetech protocol manual example).
  size_t n = scs::build_ping(1, b, sizeof(b));
  const uint8_t ping[] = {0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};
  CHECK(n == 6 && memcmp(b, ping, 6) == 0);
  // READ id 1, 2 bytes at 56: checksum = ~(1 + 4 + 2 + 56 + 2).
  n = scs::build_read(1, 56, 2, b, sizeof(b));
  const uint8_t rd[] = {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE};
  CHECK(n == 8 && memcmp(b, rd, 8) == 0);
  // WRITE goal 512 (0x0200), high byte first, to id 3.
  uint8_t pos[2];
  scs::put_u16(pos, 512);
  CHECK(pos[0] == 0x02 && pos[1] == 0x00);
  n = scs::build_write(3, scs::reg::kGoalPosition, pos, 2, b, sizeof(b));
  const uint8_t wr[] = {0xFF, 0xFF, 0x03, 0x05, 0x03, 0x2A, 0x02, 0x00, 0xC8};
  CHECK(n == 9 && memcmp(b, wr, 9) == 0);
  // SYNC WRITE two servos, 2 bytes each.
  const uint8_t ids[] = {1, 2};
  const uint8_t data[] = {0x01, 0x00, 0x03, 0xFF};
  n = scs::build_sync_write(scs::reg::kGoalPosition, 2, ids, data, 2, b, sizeof(b));
  const uint8_t sw[] = {0xFF, 0xFF, 0xFE, 0x0A, 0x83, 0x2A, 0x02, 0x01, 0x01, 0x00, 0x02, 0x03, 0xFF, 0x42};
  CHECK(n == sizeof(sw) && memcmp(b, sw, sizeof(sw)) == 0);
  CHECK(scs::build_ping(1, b, 5) == 0);  // does not fit
  CHECK(scs::decode_signed(0x0400 | 300, 10) == -300);
  CHECK(scs::decode_signed(300, 10) == 300);
  CHECK(scs::decode_signed(0x8000 | 50, 15) == -50);
}

static void test_protocol_parse() {
  scs::Reply r;
  const uint8_t ok[] = {0xFF, 0xFF, 0x01, 0x04, 0x00, 0x02, 0x00, 0xF8};
  CHECK(scs::parse_reply(ok, sizeof(ok), &r) == scs::Parse::kOk);
  CHECK(r.id == 1 && r.error == 0 && r.n == 2 && scs::get_u16(r.params) == 512);
  CHECK(scs::parse_reply(ok, 5, &r) == scs::Parse::kIncomplete);
  CHECK(scs::parse_reply(ok, 0, &r) == scs::Parse::kIncomplete);
  uint8_t bad[sizeof(ok)];
  memcpy(bad, ok, sizeof(ok));
  bad[6] ^= 0x10;
  CHECK(scs::parse_reply(bad, sizeof(bad), &r) == scs::Parse::kBadChecksum);
  // Line noise before the header, and an extra 0xFF, are skipped.
  const uint8_t noisy[] = {0x00, 0x37, 0xFF, 0xFF, 0xFF, 0x05, 0x02, 0x20, 0xD8};
  CHECK(scs::parse_reply(noisy, sizeof(noisy), &r) == scs::Parse::kOk);
  CHECK(r.id == 5 && r.error == scs::kErrOverload && r.n == 0);
  const uint8_t badlen[] = {0xFF, 0xFF, 0x01, 0x01, 0x00, 0xFD};
  CHECK(scs::parse_reply(badlen, sizeof(badlen), &r) == scs::Parse::kBadLength);
}

// --------------------------------------------------------------------- bus

static void test_bus_reads() {
  FakeBus fake;
  fake.add(7);
  ScsBus bus(fake);
  CHECK(bus.ping(7) == BusError::kOk);
  CHECK(bus.ping(8) == BusError::kNoReply);
  CHECK(bus.ping(0xFE) == BusError::kBadArg);
  fake.servo[7].set_position(700);
  fake.servo[7].set_load(-420);
  fake.servo[7].mem[scs::reg::kPresentVoltage] = 49;
  fake.servo[7].mem[scs::reg::kPresentTemperature] = 41;
  Feedback fb;
  CHECK(bus.read_feedback(7, &fb) == BusError::kOk);
  CHECK(fb.position == 700 && fb.load == -420 && fb.voltage_dv == 49 && fb.temperature_c == 41);
  uint16_t lo = 0, hi = 0;
  CHECK(bus.read_angle_limits(7, &lo, &hi) == BusError::kOk && lo == 20 && hi == 1003);
  Protection p;
  CHECK(bus.read_protection(7, &p) == BusError::kOk);
  CHECK(p.max_temperature_c == 70 && p.overload_torque_pct == 80 && p.protection_time_10ms == 200 && p.holding_torque_pct == 20);
  CHECK(fake.writes.empty());  // reading writes nothing
  // Adapter that echoes transmitted bytes.
  fake.echo = true;
  CHECK(bus.read_feedback(7, &fb) != BusError::kOk);  // echo not expected: reply is misread
  bus.set_tx_echo(true);
  CHECK(bus.read_feedback(7, &fb) == BusError::kOk && fb.position == 700);
}

static void test_bus_gates() {
  FakeBus fake;
  fake.add(1);
  fake.add(2);
  ScsBus bus(fake);
  const uint8_t ids[] = {1, 2};
  const uint16_t pos[] = {600, 400};
  // Fresh bus: nothing that moves a servo or touches EEPROM leaves it.
  size_t before = fake.tx_bytes;
  CHECK(bus.torque_on(1) == BusError::kGated);
  CHECK(bus.write_goal(1, 600, 0, 0) == BusError::kGated);
  CHECK(bus.sync_write_goal(ids, pos, 2, 0, 0) == BusError::kGated);
  CHECK(bus.write_angle_limits(1, 100, 900) == BusError::kGated);
  Protection p = {70, 36, 20, 100, 80};
  CHECK(bus.write_protection(1, p) == BusError::kGated);
  CHECK(bus.write_id(1, 9) == BusError::kGated);
  const uint8_t one = 1, zero = 0;
  CHECK(bus.write(1, scs::reg::kTorqueEnable, &one, 1) == BusError::kGated);
  CHECK(bus.write(1, scs::reg::kLock, &zero, 1) == BusError::kGated);
  CHECK(bus.write(1, scs::reg::kId, &one, 1) == BusError::kGated);
  const uint8_t span[4] = {0, 0, 2, 0};  // starts at a harmless register, runs into the goal
  CHECK(bus.write(1, 40, span, 4) == BusError::kGated);
  CHECK(fake.tx_bytes == before);  // not one byte was sent
  CHECK(fake.writes.empty());
  // Torque off is always allowed.
  CHECK(bus.torque_off(1) == BusError::kOk);
  bus.torque_off_all();
  CHECK(!fake.any_motion_write());

  // Motion gate open: goals and torque work, EEPROM still closed.
  bus.allow_motion(true);
  CHECK(bus.torque_on(1) == BusError::kOk && bus.torque_on(2) == BusError::kOk);
  CHECK(bus.torque_on(scs::kBroadcastId) == BusError::kBadArg);
  CHECK(bus.sync_write_goal(ids, pos, 2, 0, 0) == BusError::kOk);
  CHECK(fake.servo[1].position() == 600 && fake.servo[2].position() == 400);
  CHECK(bus.write_goal(1, 1024, 0, 0) == BusError::kBadArg);
  CHECK(bus.write_angle_limits(1, 100, 900) == BusError::kGated);
  CHECK(!fake.any_eeprom_write());
  bus.torque_off_all();
  CHECK(fake.torque_count() == 0);

  // EEPROM gate open: limits and protection are written (unlock, write, lock) and read back.
  bus.allow_eeprom(true);
  CHECK(bus.write_angle_limits(1, 100, 900) == BusError::kOk);
  uint16_t lo = 0, hi = 0;
  CHECK(bus.read_angle_limits(1, &lo, &hi) == BusError::kOk && lo == 100 && hi == 900);
  CHECK(fake.servo[1].mem[scs::reg::kLock] == 1);
  CHECK(bus.write_angle_limits(1, 900, 100) == BusError::kBadArg);
  CHECK(bus.write_protection(2, p) == BusError::kOk);
  Protection q;
  CHECK(bus.read_protection(2, &q) == BusError::kOk);
  CHECK(q.unload_condition == 36 && q.protection_time_10ms == 100 && q.overload_torque_pct == 80 && q.holding_torque_pct == 20);
}

// ------------------------------------------------------------------ joints

static void test_joint_conversion() {
  RobotConfig c = calibrated_config();
  c.joints[kLHipYaw].direction = 1;
  c.joints[kLHipYaw].zero_step = 512;
  // JOINT_LIMITS.md example: zero 512, +19 deg -> 512 + 65 = 577.
  CHECK(deg_to_step(c, kLHipYaw, 19.0f) == 577);
  CHECK(deg_to_step(c, kLHipYaw, 40.0f) == 577);          // clamped to the joint limit
  CHECK(deg_to_step(c, kLHipYaw, -100.0f) == 512 - 21);   // -6.3 deg
  c.joints[kLHipYaw].direction = -1;
  CHECK(deg_to_step(c, kLHipYaw, 19.0f) == 512 - 65);
  CHECK_NEAR(step_to_deg(c, kLHipYaw, 512 - 65), 19.0f, 0.15f);
  for (int j = 0; j < kNumJoints; ++j) {
    for (float d = c.joints[j].min_deg; d <= c.joints[j].max_deg; d += 3.7f) {
      CHECK_NEAR(step_to_deg(c, j, deg_to_step(c, j, d)), d, 0.5f / c.servo.steps_per_deg + 0.16f);
    }
  }
  // Head yaw +-150 deg needs the whole 0..1023 travel: the step is clamped, never out of range.
  c.joints[kHeadYaw].direction = 1;
  c.joints[kHeadYaw].zero_step = 512;
  CHECK(deg_to_step(c, kHeadYaw, 150.0f) == 1023);
  CHECK(deg_to_step(c, kHeadYaw, -150.0f) == 0);
  CHECK(deg_to_step(c, kBeak, NAN) == deg_to_step(c, kBeak, 0.0f));
  // A servo's speed report (steps/s) as the joint's speed in deg/s, in the joint's direction.
  c.joints[kLKnee].direction = 1;
  c.joints[kRKnee].direction = -1;
  CHECK_NEAR(speed_to_dps(c, kLKnee, 341), 100.0f, 0.01f);
  CHECK_NEAR(speed_to_dps(c, kRKnee, 341), -100.0f, 0.01f);
  CHECK_NEAR(speed_to_dps(c, kLKnee, -34), -9.97f, 0.01f);
  CHECK(!config_ready_for_motion(kRobotConfig));           // shipped placeholders
  CHECK(config_ready_for_motion(c));
  c.joints[3].servo_id = c.joints[4].servo_id;
  CHECK(!config_ready_for_motion(c));                      // duplicate ID
}

// ------------------------------------------------------------------ safety

static void run_filter(SafetyFilter& f, const float target[kNumJoints], int ticks, bool checked, uint16_t* any_flags) {
  float out[kNumJoints];
  uint16_t flags = 0;
  for (int i = 0; i < ticks; ++i) flags |= f.step(target, 0.01f, checked, out);
  if (any_flags) *any_flags = flags;
}

static void test_safety_clamp_and_rate() {
  RobotConfig c = calibrated_config();
  SafetyFilter f(c);
  float q[kNumJoints] = {0};
  f.reset(q);
  float t[kNumJoints] = {0};
  t[kNeckPitch] = 90.0f;   // limit +30
  t[kBeak] = -20.0f;       // limit 0..35
  float out[kNumJoints];
  uint16_t flags = f.step(t, 0.01f, false, out);
  CHECK(flags & kFlagJointClamp);
  CHECK(flags & kFlagRateLimit);
  CHECK_NEAR(out[kNeckPitch], 1.8f, 1e-4);   // 180 deg/s x 10 ms
  CHECK(out[kBeak] == 0.0f);
  run_filter(f, t, 300, false, nullptr);
  CHECK_NEAR(f.setpoint()[kNeckPitch], 30.0f, 1e-4);
  t[kBeak] = 80.0f;
  run_filter(f, t, 300, false, nullptr);
  CHECK_NEAR(f.setpoint()[kBeak], 35.0f, 1e-4);
  // Every joint, driven far past both ends, stays inside its limits.
  for (int sign = -1; sign <= 1; sign += 2) {
    for (int j = 0; j < kNumJoints; ++j) t[j] = sign * 1000.0f;
    for (int i = 0; i < 400; ++i) {
      f.step(t, 0.01f, false, out);
      for (int j = 0; j < kNumJoints; ++j) CHECK(out[j] >= c.joints[j].min_deg && out[j] <= c.joints[j].max_deg);
    }
  }
  // NaN / infinity never reach a servo: the joint keeps its setpoint.
  f.reset(q);
  for (int j = 0; j < kNumJoints; ++j) t[j] = 0.0f;
  t[kLKnee] = NAN;
  t[kRKnee] = INFINITY;
  flags = f.step(t, 0.01f, false, out);
  CHECK((flags & kFlagNonFinite) && out[kLKnee] == 0.0f && out[kRKnee] == 0.0f);
}

static bool head_state_ok(const HeadInterlockConfig& h, float yaw, float roll) {
  const float win = h.yaw_window_deg - h.yaw_margin_deg, rmax = h.roll_max_deg - h.roll_margin_deg;
  const float e = 1e-4f;
  return (fabsf(yaw) <= win + e && fabsf(roll) <= rmax + e) || fabsf(roll) <= h.roll_free_deg + e;
}

static void test_head_interlock() {
  RobotConfig c = calibrated_config();
  const HeadInterlockConfig& h = c.head;
  SafetyFilter f(c);
  float q[kNumJoints] = {0};
  float t[kNumJoints] = {0};
  float out[kNumJoints];
  // Roll at yaw 0 goes to roll max minus the margin, not to the documented 15.
  f.reset(q);
  t[kHeadRoll] = 15.0f;
  run_filter(f, t, 100, false, nullptr);
  CHECK_NEAR(f.setpoint()[kHeadRoll], h.roll_max_deg - h.roll_margin_deg, 1e-4);
  // Now ask for yaw: it stops at the window minus the margin while rolled.
  t[kHeadYaw] = 60.0f;
  uint16_t flags = 0;
  run_filter(f, t, 100, false, &flags);
  CHECK(flags & kFlagHeadInterlock);
  CHECK_NEAR(f.setpoint()[kHeadYaw], h.yaw_window_deg - h.yaw_margin_deg, 1e-4);
  // The documented boundary (yaw 2.5 with roll 15), where the review found a collision, is unreachable.
  CHECK(!(fabsf(f.setpoint()[kHeadYaw]) >= 2.5f && fabsf(f.setpoint()[kHeadRoll]) >= 15.0f));
  // Roll back to 0: yaw is released.
  t[kHeadRoll] = 0.0f;
  run_filter(f, t, 200, false, nullptr);
  CHECK_NEAR(f.setpoint()[kHeadYaw], 60.0f, 1e-3);
  // Yawed: roll is held to the free band.
  t[kHeadRoll] = -15.0f;
  run_filter(f, t, 100, false, &flags);
  CHECK(flags & kFlagHeadInterlock);
  CHECK_NEAR(f.setpoint()[kHeadRoll], -h.roll_free_deg, 1e-4);
  // Random walk of commands: the allowed-state rule holds on every tick.
  f.reset(q);
  unsigned seed = 12345;
  for (int i = 0; i < 20000; ++i) {
    seed = seed * 1103515245u + 12345u;
    if (i % 37 == 0) t[kHeadYaw] = static_cast<float>((seed >> 8) % 3200) / 10.0f - 160.0f;
    seed = seed * 1103515245u + 12345u;
    if (i % 23 == 0) t[kHeadRoll] = static_cast<float>((seed >> 8) % 400) / 10.0f - 20.0f;
    f.step(t, 0.01f, false, out);
    CHECK(head_state_ok(h, out[kHeadYaw], out[kHeadRoll]));
  }
  // A larger margin shrinks the window; margin >= window forbids rolling while yawed at all.
  HeadInterlockConfig tight = h;
  tight.yaw_margin_deg = 5.0f;
  float yaw = 1.0f, roll = 10.0f;
  head_interlock(tight, 0.0f, 10.0f, &yaw, &roll);
  CHECK(yaw == 0.0f && roll == 10.0f);
  // A state the interlock never produces (yawed AND rolled): yaw holds, roll may only go toward zero.
  yaw = 30.0f; roll = 12.0f;
  head_interlock(h, 20.0f, 8.0f, &yaw, &roll);
  CHECK(yaw == 20.0f && roll == 8.0f);
  yaw = 30.0f; roll = 2.0f;
  head_interlock(h, 20.0f, 8.0f, &yaw, &roll);
  CHECK(yaw == 20.0f && roll == 2.0f);
}

// The table in JOINT_LIMITS.md is "with the knee in -45..+68": free at all three of those knee grid values.
static bool table_allowed(const HipMapConfig& m, float pitch, float roll, float yaw) {
  return hip_pose_allowed(m, pitch, roll, yaw, -45) && hip_pose_allowed(m, pitch, roll, yaw, 0) &&
         hip_pose_allowed(m, pitch, roll, yaw, 68);
}

static void test_hip_map() {
  const HipMapConfig& m = kRobotConfig.hip;
  // Cells of the table in JOINT_LIMITS.md section 2 (left leg).
  CHECK(table_allowed(m, 45, 10, 0));        // +45, yaw 0: -12..+10
  CHECK(!table_allowed(m, 45, 15, 0));
  CHECK(table_allowed(m, 45, -12, 0));
  CHECK(!table_allowed(m, 45, 0, -6.3f));    // +45, toe in: none
  CHECK(!table_allowed(m, 60, -12, -6.3f));
  CHECK(table_allowed(m, 60, 0, 0));         // +60, yaw 0: -12..0
  CHECK(!table_allowed(m, 60, 5, 0));
  CHECK(table_allowed(m, 75, -8, 0));        // +75, yaw 0: -12..-8
  CHECK(!table_allowed(m, 75, -4, 0));
  CHECK(table_allowed(m, 60, 15, 19));       // +60, toe out: -12..+15
  CHECK(!table_allowed(m, 60, 19.5f, 19));
  // -90 (leg fully forward): the design table allows these for every knee angle, but the review found the RIGHT
  // foot meeting the neck bearing plate / body frame with the knee at its 68 deg limit. One map serves both legs,
  // so the knee-68 cells are blocked for both (review_extra_collisions in robot.json).
  CHECK(hip_pose_allowed(m, -90, 15, -6.3f, 0));   // -90, toe in: -12..+15 with the knee straight
  CHECK(!table_allowed(m, -90, 15, -6.3f));        // ...but not over the whole knee range any more
  CHECK(!table_allowed(m, -90, 19.5f, -6.3f));
  CHECK(hip_pose_allowed(m, -90, 0, 19, 0));       // -90, toe out: -12..0 with the knee straight
  CHECK(!hip_pose_allowed(m, -90, 0, 19, 68));     // review: right foot vs neck bearing plate, 6.5 mm
  CHECK(!table_allowed(m, -90, 5, 19));
  CHECK(table_allowed(m, 15, 10, -6.3f));    // +15, toe in: -8..+10
  CHECK(!table_allowed(m, 15, -12, -6.3f));
  CHECK(!table_allowed(m, 15, 15, -6.3f));
  CHECK(table_allowed(m, 30, 5, -6.3f));     // +30, toe in: -8..+5
  CHECK(!table_allowed(m, 30, 10, -6.3f));
  CHECK(table_allowed(m, -30, 19.5f, 19));   // -45..0: all
  CHECK(table_allowed(m, -30, -12, -6.3f));
  CHECK(hip_pose_allowed(m, 0, 10, -6.3f, -90));   // folded at 0: +5..+19.5
  CHECK(!hip_pose_allowed(m, 0, 0, -6.3f, -90));
  CHECK(!hip_pose_allowed(m, 60, 0, 0, -90));      // the fold / sit-up collision: hip +60, knee -90
  // Stricter neighbour between grid values, in every axis.
  CHECK(hip_pose_allowed(m, 45, 5, 0, -45));       // pitch 45 allows +5 here, pitch 60 does not
  CHECK(!hip_pose_allowed(m, 60, 5, 0, -45));
  CHECK(!hip_pose_allowed(m, 50, 5, 0, -45));
  CHECK(hip_pose_allowed(m, 50, 0, 0, -45));
  CHECK(!hip_pose_allowed(m, 45, 12, 0, 0));       // roll between free 10 and colliding 15
  CHECK(!hip_pose_allowed(m, 45, 5, -3, 0));       // yaw between 0 (free) and -6.3 (collides)
  CHECK(!hip_pose_allowed(m, 30, 10, 0, -60));     // knee between -45 (free) and -90 (collides)
  CHECK(hip_pose_allowed(m, 30, 10, 0, -30));
  CHECK(!hip_pose_allowed(m, NAN, 0, 0, 0));
  // Right leg = mirror.
  float q[kNumJoints] = {0};
  q[kRHipPitch] = -45; q[kRHipRoll] = -10; q[kRHipYaw] = 0; q[kRKnee] = 0;
  CHECK(leg_allowed(m, q, true));
  q[kRHipRoll] = -15;
  CHECK(!leg_allowed(m, q, true));
  // STAND is inside the map. SIT is NOT (toe-in 3 deg with hip pitch 70): the sit path was
  // checked on its own, which is why the filter has a checked-path mode.
  float stand[kNumJoints], sit[kNumJoints];
  pose_to_joints(kRobotConfig.stand, stand);
  pose_to_joints(kRobotConfig.sit, sit);
  CHECK(leg_allowed(m, stand, false) && leg_allowed(m, stand, true));
  CHECK(!leg_allowed(m, sit, false) && !leg_allowed(m, sit, true));
}

static void test_filter_hip_map() {
  RobotConfig c = calibrated_config();
  SafetyFilter f(c);
  float q[kNumJoints] = {0};
  f.reset(q);
  float t[kNumJoints] = {0};
  float out[kNumJoints];
  // Swing the left leg back with the toe in: the leg stops before the map's edge, the right leg is free.
  t[kLHipYaw] = -6.3f;
  t[kLHipPitch] = 80.0f;
  t[kRHipPitch] = -30.0f;
  uint16_t flags = 0;
  for (int i = 0; i < 300; ++i) {
    flags |= f.step(t, 0.01f, false, out);
    CHECK(leg_allowed(c.hip, out, false));
    CHECK(leg_allowed(c.hip, out, true));
  }
  CHECK(flags & kFlagHipMapLeft);
  CHECK(!(flags & kFlagHipMapRight));
  CHECK(f.setpoint()[kLHipPitch] <= 30.001f);   // at +45 with the toe in nothing is free
  CHECK_NEAR(f.setpoint()[kRHipPitch], -30.0f, 1e-3);
  // In free mode the SIT pose cannot be reached; on the checked path it can.
  float stand[kNumJoints], sit[kNumJoints];
  pose_to_joints(c.stand, stand);
  pose_to_joints(c.sit, sit);
  f.reset(stand);
  run_filter(f, sit, 300, false, &flags);
  CHECK((flags & kFlagHipMapLeft) && (flags & kFlagHipMapRight));
  CHECK(f.setpoint()[kLHipPitch] <= 30.001f);
  f.reset(stand);
  run_filter(f, sit, 300, true, &flags);
  CHECK(!(flags & (kFlagHipMapLeft | kFlagHipMapRight)));
  CHECK_NEAR(f.setpoint()[kLHipPitch], 70.0f, 1e-3);
}

// -------------------------------------------------------------------- math

static void test_fmath_against_libm() {
  double worst_sin = 0, worst_asin = 0, worst_sqrt = 0;
  for (int i = -4000; i <= 4000; ++i) {
    const float x = static_cast<float>(i) * 0.005f;   // -20..20 rad
    worst_sin = fmax(worst_sin, fabs(duck::sin_f(x) - sin(static_cast<double>(x))));
    worst_sin = fmax(worst_sin, fabs(duck::cos_f(x) - cos(static_cast<double>(x))));
  }
  for (int i = -10000; i <= 10000; ++i) {
    const float v = static_cast<float>(i) * 1e-4f;
    worst_asin = fmax(worst_asin, fabs(duck::asin_deg(v) - asin(static_cast<double>(v)) * 180.0 / M_PI));
  }
  for (int i = 1; i <= 20000; ++i) {
    const float v = static_cast<float>(i) * static_cast<float>(i) * 1e-6f;   // 1e-6 .. 400
    worst_sqrt = fmax(worst_sqrt, fabs(duck::sqrt_f(v) / sqrt(static_cast<double>(v)) - 1.0));
  }
  printf("    worst error: sin/cos %.2e, asin %.2e deg, sqrt %.2e (relative)\n", worst_sin, worst_asin, worst_sqrt);
  CHECK(worst_sin < 2e-5);
  CHECK(worst_asin < 0.01);
  CHECK(worst_sqrt < 1e-6);
  CHECK(duck::asin_deg(2.0f) == 90.0f && duck::asin_deg(-2.0f) == -90.0f && duck::asin_deg(NAN) == 0.0f);
  CHECK(duck::sqrt_f(-1.0f) == 0.0f && duck::sqrt_f(0.0f) == 0.0f);
  CHECK(duck::finitef(1.0f) && !duck::finitef(NAN) && !duck::finitef(INFINITY) && !duck::finitef(-INFINITY));
  CHECK(duck::round_to_long(2.5f) == 3 && duck::round_to_long(-2.5f) == -3 && duck::round_to_long(0.49f) == 0);
}

// ------------------------------------------------------------------ motion

static void test_motion_and_balance() {
  CHECK_NEAR(ease_cos(0.0f), 0.0f, 1e-6);
  CHECK_NEAR(ease_cos(0.5f), 0.5f, 1e-6);
  CHECK_NEAR(ease_cos(1.0f), 1.0f, 1e-6);
  const RobotConfig& c = kRobotConfig;
  PoseMove mv;
  mv.start(c.stand, c.sit, 3.0f);
  float q[kNumPoseJoints];
  float prev = -1.0f;
  int ticks = 0;
  while (mv.active()) {
    float e = mv.advance(0.01f, q);
    ++ticks;
    CHECK(e >= prev);
    prev = e;
    // Straight line in joint space: every joint is at the same fraction of its way.
    for (int j = 0; j < kNumPoseJoints; ++j) CHECK_NEAR(q[j], c.stand[j] + (c.sit[j] - c.stand[j]) * e, 1e-3);
  }
  CHECK(ticks >= 299 && ticks <= 301);   // 3 s at 100 Hz
  for (int j = 0; j < kNumPoseJoints; ++j) CHECK_NEAR(q[j], c.sit[j], 1e-4);

  CHECK_NEAR(planned_pitch_deg(c.motion, 0.0f), c.motion.planned_pitch_deg[0], 1e-5);
  CHECK_NEAR(planned_pitch_deg(c.motion, 1.0f), c.motion.planned_pitch_deg[kPlannedPitchN - 1], 1e-5);
  CHECK_NEAR(planned_pitch_deg(c.motion, 0.0625f), 0.5f * (c.motion.planned_pitch_deg[0] + c.motion.planned_pitch_deg[1]), 1e-4);
  CHECK_NEAR(planned_pitch_deg(c.motion, 7.0f), c.motion.planned_pitch_deg[kPlannedPitchN - 1], 1e-5);

  // Tilt from gravity: + pitch = nose up; a mounting matrix is applied first.
  ImuConfig ic = c.imu;
  Tilt t = tilt_from_up(ic, imu_at(10.0f, -4.0f, 0).up);
  CHECK_NEAR(t.pitch_deg, 10.0f, 1e-3);
  CHECK_NEAR(t.roll_deg, -4.0f, 1e-3);
  const float rot[3][3] = {{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};  // sensor z = body x
  memcpy(ic.sensor_to_body, rot, sizeof(rot));
  const float up_sensor[3] = {-0.9848f, 0.0f, 0.1736f};
  CHECK_NEAR(tilt_from_up(ic, up_sensor).pitch_deg, 10.0f, 0.05);

  // Balance law: ankle correction = -1.5 x (error beyond the 0.2 deg dead zone), error low-passed 25 ms.
  BalanceLaw b(c.balance);
  for (int i = 0; i < 100; ++i) b.update(0.15f, 0.01f);
  CHECK(b.correction() == 0.0f);                               // inside the dead zone
  b.reset();
  for (int i = 0; i < 100; ++i) b.update(2.0f, 0.01f);
  CHECK_NEAR(b.correction(), -1.5f * (2.0f - 0.2f), 1e-3);     // leaning back -> ankle goes negative
  b.reset();
  for (int i = 0; i < 100; ++i) b.update(-2.0f, 0.01f);
  CHECK_NEAR(b.correction(), 1.5f * 1.8f, 1e-3);
  b.reset();
  for (int i = 0; i < 25; ++i) b.update(2.0f, 0.001f);         // one time constant
  CHECK_NEAR(b.correction() / (-1.5f), 2.0f * (1.0f - expf(-1.0f)) - 0.2f, 0.03f);
  b.reset();
  for (int i = 0; i < 100; ++i) b.update(60.0f, 0.01f);
  CHECK_NEAR(b.correction(), -c.balance.max_correction_deg, 1e-4);  // clamp
  b.update(NAN, 0.01f);
  CHECK_NEAR(b.correction(), -c.balance.max_correction_deg, 1e-4);
}

// -------------------------------------------------------------- controller

struct Rig {
  RobotConfig cfg;
  FakeBus fake;
  ScsBus bus;
  Controller ctl;
  uint32_t now = 1000;
  float pitch = 0.0f, roll = 0.0f;
  bool imu_valid = true;
  explicit Rig(const RobotConfig& c, bool motion_build = true) : cfg(c), bus(fake), ctl(cfg, bus, motion_build) {}
  ImuSample imu() {
    ImuSample s = imu_at(pitch, roll, now);
    s.valid = imu_valid;
    return s;
  }
  void run(int ticks, bool heartbeat = true) {
    for (int i = 0; i < ticks; ++i) {
      now += 10;
      if (heartbeat) ctl.heartbeat(now);
      ctl.tick(now, imu());
    }
  }
  float servo_deg(int j) { return step_to_deg(cfg, j, static_cast<int16_t>(fake.servo[cfg.joints[j].servo_id].position())); }
  // Every servo at `pose`, then armed: the arm must succeed.
  void arm_at(const float pose[kNumPoseJoints]) {
    place_servos(fake, cfg, pose);
    CHECK(ctl.arm(kArmKey, now, imu()) == Result::kOk);
  }
};

static void test_arming_is_refused_by_default() {
  // 1. The shipped configuration (placeholders) with a motion-enabled build.
  {
    Rig r(kRobotConfig, true);
    for (int id = 1; id <= 15; ++id) r.fake.add(id);
    CHECK(r.ctl.arm(kArmKey, r.now, r.imu()) == Result::kNotCalibrated);
    r.run(300);
    CHECK(r.ctl.status().mode == Mode::kDisarmed);
    CHECK(!r.fake.any_motion_write() && !r.fake.any_eeprom_write() && r.fake.torque_count() == 0);
  }
  // 2. A calibrated robot, but the default build (motion compiled out).
  {
    Rig r(calibrated_config(), false);
    place_servos(r.fake, r.cfg, r.cfg.stand);
    CHECK(r.ctl.arm(kArmKey, r.now, r.imu()) == Result::kMotionBuildDisabled);
    float t[kNumJoints] = {0};
    CHECK(r.ctl.set_targets(t, r.now) == Result::kNotArmed);
    CHECK(r.ctl.stand(true, r.now) == Result::kNotArmed);
    CHECK(r.ctl.sit(r.now, r.imu()) == Result::kNotArmed);
    r.run(300);
    CHECK(!r.fake.any_motion_write() && r.fake.torque_count() == 0);
    CHECK(r.ctl.feedback_valid(0));  // telemetry still works while disarmed
  }
  // 3. Wrong key, missing servo, missing / uncalibrated IMU.
  {
    Rig r(calibrated_config());
    place_servos(r.fake, r.cfg, r.cfg.stand);
    CHECK(r.ctl.arm(0, r.now, r.imu()) == Result::kBadKey);
    r.fake.servo[5].present = false;
    CHECK(r.ctl.arm(kArmKey, r.now, r.imu()) == Result::kServoMissing);
    r.fake.servo[5].present = true;
    r.imu_valid = false;
    CHECK(r.ctl.arm(kArmKey, r.now, r.imu()) == Result::kImuRequired);
    r.imu_valid = true;
    ImuSample old = r.imu();
    old.t_ms = r.now - 500;
    CHECK(r.ctl.arm(kArmKey, r.now, old) == Result::kImuRequired);
    r.cfg.imu.calibrated = false;
    CHECK(r.ctl.arm(kArmKey, r.now, r.imu()) == Result::kImuRequired);
    CHECK(!r.fake.any_motion_write() && r.fake.torque_count() == 0);
    // A servo that is already too hot: fault instead of torque.
    r.cfg.imu.calibrated = true;
    r.fake.servo[3].mem[scs::reg::kPresentTemperature] = 80;
    CHECK(r.ctl.arm(kArmKey, r.now, r.imu()) == Result::kFaultLatched);
    CHECK(r.ctl.status().fault == Fault::kOverTemp && r.fake.torque_count() == 0);
  }
}

static void test_arm_holds_position() {
  Rig r(calibrated_config());
  float odd[kNumPoseJoints] = {5, 3, -10, 20, 4, -30, 10, 40, 0, -5, -3, 10, -20, -4};
  place_servos(r.fake, r.cfg, odd);
  uint16_t before[kNumJoints];
  for (int j = 0; j < kNumJoints; ++j) before[j] = r.fake.servo[r.cfg.joints[j].servo_id].position();
  CHECK(r.ctl.arm(kArmKey, r.now, r.imu()) == Result::kOk);
  CHECK(r.ctl.status().mode == Mode::kArmed && r.ctl.status().posture == Posture::kUnknown);
  CHECK(r.fake.torque_count() == 15);
  r.run(100);
  // Nothing moved: goals were the measured positions.
  for (int j = 0; j < kNumJoints; ++j) CHECK(r.fake.servo[r.cfg.joints[j].servo_id].position() == before[j]);
  // Unknown posture: leg targets are ignored, a plain stand is refused, head and beak follow.
  float t[kNumJoints] = {0};
  t[kLHipPitch] = 40.0f;
  t[kHeadYaw] = -20.0f;
  t[kBeak] = 20.0f;
  CHECK(r.ctl.set_targets(t, r.now) == Result::kOk);
  r.run(200);
  CHECK(r.ctl.status().flags & kStatLegCmdIgnored);
  CHECK(r.fake.servo[r.cfg.joints[kLHipPitch].servo_id].position() == before[kLHipPitch]);
  CHECK_NEAR(r.servo_deg(kHeadYaw), -20.0f, 0.3f);
  CHECK_NEAR(r.servo_deg(kBeak), 20.0f, 0.3f);
  CHECK(r.ctl.stand(false, r.now) == Result::kWrongPosture);
  CHECK(r.ctl.sit(r.now, r.imu()) == Result::kWrongPosture);
  // Supported stand (robot held): 3 s to the STAND keyframe.
  CHECK(r.ctl.stand(true, r.now) == Result::kOk);
  r.run(310);
  CHECK(r.ctl.status().posture == Posture::kStanding);
  for (int j = 0; j < kNumPoseJoints; ++j) CHECK_NEAR(r.servo_deg(j), r.cfg.stand[j], 0.3f);
  r.ctl.disarm();
  CHECK(r.fake.torque_count() == 0 && r.ctl.status().mode == Mode::kDisarmed && !r.bus.motion_allowed());
}

static void test_sit_and_stand() {
  Rig r(calibrated_config());
  r.arm_at(r.cfg.stand);
  CHECK(r.ctl.status().posture == Posture::kStanding);
  r.run(20);
  // Uphill by more than the limit: refused. Just under: accepted.
  r.pitch = r.cfg.motion.sit_uphill_limit_deg + 0.5f;
  CHECK(r.ctl.sit(r.now, r.imu()) == Result::kUphill);
  r.pitch = 0.0f;
  r.imu_valid = false;
  CHECK(r.ctl.sit(r.now, r.imu()) == Result::kImuRequired);
  r.imu_valid = true;
  // Head turned away from the keyframe: refused until stand() brings it back.
  float t[kNumJoints];
  pose_to_joints(r.cfg.stand, t);
  t[kHeadYaw] = 50.0f;
  r.ctl.set_targets(t, r.now);
  r.run(100);
  CHECK(r.ctl.sit(r.now, r.imu()) == Result::kNotAtStand);
  CHECK(r.ctl.stand(false, r.now) == Result::kOk);
  r.run(160);
  CHECK(r.ctl.status().posture == Posture::kStanding);
  // Sit: 3 s, the trunk pitch follows the plan, and every servo step stays inside its limits.
  CHECK(r.ctl.sit(r.now, r.imu()) == Result::kOk);
  CHECK(r.ctl.sit(r.now, r.imu()) == Result::kBusy);
  int ticks = 0;
  float max_dev = 0.0f;
  while (r.ctl.status().posture == Posture::kMovingToSit && ticks < 1000) {
    float e = ease_cos(static_cast<float>(ticks + 1) * 0.01f / r.cfg.motion.sit_stand_s);
    r.pitch = planned_pitch_deg(r.cfg.motion, e);
    r.run(1);
    ++ticks;
    // Hip pitch and knee stay on the straight line between the keyframes.
    float fp = (r.ctl.setpoint_deg()[kLHipPitch] - r.cfg.stand[kLHipPitch]) / (r.cfg.sit[kLHipPitch] - r.cfg.stand[kLHipPitch]);
    float fk = (r.ctl.setpoint_deg()[kLKnee] - r.cfg.stand[kLKnee]) / (r.cfg.sit[kLKnee] - r.cfg.stand[kLKnee]);
    if (fabsf(fp - fk) > max_dev) max_dev = fabsf(fp - fk);
    for (int j = 0; j < kNumJoints; ++j) {
      CHECK(r.ctl.setpoint_deg()[j] >= r.cfg.joints[j].min_deg && r.ctl.setpoint_deg()[j] <= r.cfg.joints[j].max_deg);
    }
  }
  CHECK(ticks >= 299 && ticks <= 301);
  CHECK(max_dev < 1e-3f);
  CHECK(r.ctl.status().posture == Posture::kSitting && r.ctl.status().mode == Mode::kArmed);
  r.run(50);
  for (int j = 0; j < kNumPoseJoints; ++j) CHECK_NEAR(r.servo_deg(j), r.cfg.sit[j], 0.3f);
  // Sitting: leg (and by default head) targets are ignored; the beak still works.
  t[kLHipPitch] = 0.0f;
  t[kHeadYaw] = 30.0f;
  t[kBeak] = 30.0f;
  r.ctl.set_targets(t, r.now);
  r.run(100);
  CHECK_NEAR(r.servo_deg(kLHipPitch), r.cfg.sit[kLHipPitch], 0.3f);
  CHECK_NEAR(r.servo_deg(kHeadYaw), 0.0f, 0.3f);
  CHECK_NEAR(r.servo_deg(kBeak), 30.0f, 0.3f);
  CHECK(r.ctl.sit(r.now, r.imu()) == Result::kWrongPosture);
  // Stand up again.
  CHECK(r.ctl.stand(false, r.now) == Result::kOk);
  ticks = 0;
  while (r.ctl.status().posture == Posture::kMovingToStand && ticks < 1000) {
    float e = ease_cos(static_cast<float>(ticks + 1) * 0.01f / r.cfg.motion.sit_stand_s);
    r.pitch = planned_pitch_deg(r.cfg.motion, 1.0f - e);
    r.run(1);
    ++ticks;
  }
  CHECK(ticks >= 299 && ticks <= 301);
  CHECK(r.ctl.status().posture == Posture::kStanding && r.ctl.status().mode == Mode::kArmed);
  r.run(50);
  for (int j = 0; j < kNumPoseJoints; ++j) CHECK_NEAR(r.servo_deg(j), r.cfg.stand[j], 0.3f);
}

static void test_balance_in_the_loop() {
  Rig r(calibrated_config());
  r.arm_at(r.cfg.stand);
  r.run(50);
  const float left0 = r.ctl.setpoint_deg()[kLAnkle], right0 = r.ctl.setpoint_deg()[kRAnkle];
  CHECK_NEAR(left0, r.cfg.stand[kLAnkle], 0.2f);   // the servo's step size
  // Trunk leans back 2 deg: left ankle -2.7, right ankle +2.7 (gain -1.5, dead zone 0.2).
  r.pitch = 2.0f;
  r.run(100);
  CHECK_NEAR(r.ctl.setpoint_deg()[kLAnkle], left0 - 2.7f, 0.01f);
  CHECK_NEAR(r.ctl.setpoint_deg()[kRAnkle], right0 + 2.7f, 0.01f);
  CHECK_NEAR(r.ctl.status().balance_deg, -2.7f, 0.01f);
  // IMU goes stale: the last correction is kept, flagged.
  r.imu_valid = false;
  r.run(30);
  CHECK(r.ctl.status().flags & kStatImuStale);
  CHECK_NEAR(r.ctl.setpoint_deg()[kLAnkle], left0 - 2.7f, 0.01f);
  CHECK(r.ctl.status().mode == Mode::kArmed);
}

static void test_faults_switch_torque_off() {
  // Overload: 95 % load for longer than the limit.
  {
    Rig r(calibrated_config());
    r.arm_at(r.cfg.stand);
    r.fake.servo[r.cfg.joints[kLKnee].servo_id].set_load(950);
    r.run(60);   // 0.6 s: not yet
    CHECK(r.ctl.status().mode == Mode::kArmed);
    r.run(80);
    CHECK(r.ctl.status().mode == Mode::kFault && r.ctl.status().fault == Fault::kOverload);
    CHECK(r.ctl.status().fault_joint == kLKnee);
    CHECK(r.fake.torque_count() == 0 && !r.bus.motion_allowed());
    // Latched: no re-arm until cleared, and clearing leaves it disarmed.
    CHECK(r.ctl.arm(kArmKey, r.now, r.imu()) == Result::kFaultLatched);
    size_t n = r.fake.writes.size();
    r.run(50);
    for (size_t i = n; i < r.fake.writes.size(); ++i) CHECK(r.fake.writes[i].addr == scs::reg::kTorqueEnable && r.fake.writes[i].data[0] == 0);
    CHECK(r.ctl.clear_fault() == Result::kOk && r.ctl.status().mode == Mode::kDisarmed);
    CHECK(r.fake.torque_count() == 0);
  }
  // A short load spike does not trip it.
  {
    Rig r(calibrated_config());
    r.arm_at(r.cfg.stand);
    r.fake.servo[4].set_load(-1000);
    r.run(40);
    r.fake.servo[4].set_load(300);
    r.run(200);
    CHECK(r.ctl.status().mode == Mode::kArmed);
  }
  // The faults that trip within a few ticks: armed at STAND, `setup` breaks something, `ticks` later the fault is
  // latched and every servo is off. Returns the status for the checks of each case.
  auto trips = [](void (*setup)(Rig&), int ticks, Fault expected) {
    Rig r(calibrated_config());
    r.arm_at(r.cfg.stand);
    setup(r);
    r.run(ticks);
    CHECK(r.ctl.status().fault == expected && r.fake.torque_count() == 0);
    return r.ctl.status();
  };
  // Over-temperature, under-voltage, the servo's own status byte.
  trips([](Rig& r) { r.fake.servo[9].mem[scs::reg::kPresentTemperature] = 66; }, 20, Fault::kOverTemp);
  trips([](Rig& r) { r.fake.servo[2].mem[scs::reg::kPresentVoltage] = 40; }, 20, Fault::kUnderVoltage);
  trips([](Rig& r) { r.fake.status_error = scs::kErrOverload; }, 20, Fault::kServoStatus);
  // A servo stops answering: comm_fail_count (5) reads in a row, one every 2 ticks at 8 reads a tick.
  const Status mute = trips([](Rig& r) { r.fake.servo[11].mute = true; }, 12, Fault::kComm);
  CHECK(mute.fault_joint == 10);
  // Fall: trunk far from the planned pitch, or rolled over.
  {
    Rig r(calibrated_config());
    r.arm_at(r.cfg.stand);
    r.run(20);
    r.pitch = -45.0f;
    r.run(5);
    CHECK(r.ctl.status().mode == Mode::kArmed);   // shorter than fall_seconds
    r.run(15);
    CHECK(r.ctl.status().fault == Fault::kFall && r.fake.torque_count() == 0);
  }
  trips([](Rig& r) { r.roll = 40.0f; }, 20, Fault::kFall);
}

static void test_gyro_in_status() {
  // The gyro is turned into the body frame by the mounting matrix and reported only while fresh and calibrated;
  // otherwise it reads as not measured (zeros with gyro_ok false), never as the last value.
  RobotConfig c = calibrated_config();
  const float rot[3][3] = {{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};  // sensor z = body x, sensor x = body -z
  memcpy(c.imu.sensor_to_body, rot, sizeof(rot));
  const float in[3] = {10.0f, 20.0f, 30.0f};
  float out[3];
  sensor_to_body(c.imu, in, out);
  CHECK_NEAR(out[0], 30.0f, 1e-5);
  CHECK_NEAR(out[1], 20.0f, 1e-5);
  CHECK_NEAR(out[2], -10.0f, 1e-5);

  Rig r(c, false);  // default build, disarmed: the gyro is telemetry and must work on the bench
  place_servos(r.fake, r.cfg, r.cfg.stand);
  auto sample = [&](uint32_t age_ms, bool valid) {
    ImuSample s = r.imu();
    for (int i = 0; i < 3; ++i) s.gyro_dps[i] = in[i];
    s.gyro_valid = valid;
    s.gyro_t_ms = r.now - age_ms;
    return s;
  };
  r.now += 10;
  r.ctl.tick(r.now, sample(0, true));
  Status st = r.ctl.status();
  CHECK(st.gyro_ok);
  CHECK_NEAR(st.gyro_dps[0], 30.0f, 1e-5);
  CHECK_NEAR(st.gyro_dps[1], 20.0f, 1e-5);
  CHECK_NEAR(st.gyro_dps[2], -10.0f, 1e-5);

  r.now += 10;
  r.ctl.tick(r.now, sample(r.cfg.imu.stale_ms + 1, true));  // stale
  st = r.ctl.status();
  CHECK(!st.gyro_ok && st.gyro_dps[0] == 0.0f && st.gyro_dps[1] == 0.0f && st.gyro_dps[2] == 0.0f);

  r.now += 10;
  r.ctl.tick(r.now, sample(0, false));  // no gyro report yet
  CHECK(!r.ctl.status().gyro_ok);

  r.imu_valid = false;  // gravity missing does not hide a fresh gyro: they are separate reports
  r.now += 10;
  r.ctl.tick(r.now, sample(0, true));
  CHECK(r.ctl.status().gyro_ok && !r.ctl.status().imu_ok);

  r.cfg.imu.calibrated = false;  // an uncalibrated mounting means the body frame is unknown
  r.now += 10;
  r.ctl.tick(r.now, sample(0, true));
  CHECK(!r.ctl.status().gyro_ok);
  CHECK(!r.fake.any_motion_write() && r.fake.torque_count() == 0);
}

static void test_status_display() {
  DisplayInput in;
  memset(&in, 0, sizeof(in));
  in.status.mode = Mode::kDisarmed;
  in.status.fault_joint = -1;
  in.ms_since_call = kNever;
  in.ms_since_command = kNever;
  in.message = nullptr;
  in.ms_since_message = kNever;
  in.bench.mode = BenchMode::kOff;
  in.bench.joint = -1;
  DisplayOutput out;
  char text[kTextMax];
  auto px = [&](int row, int col) { return out.matrix[row * kMatrixCols + col]; };
  auto text_lit = [&]() {
    int n = 0;
    for (int i = 0; i < 5 * kMatrixCols; ++i) n += out.matrix[i] != 0;
    return n;
  };

  // A bare board: the ticker says so, every servo dot dim, no IMU dots, green flash, link off.
  display_text(in, text);
  CHECK(strcmp(text, "SAFE  SV 0/15  NO IMU  NO LINUX") == 0);
  render_display(in, 1000, &out);
  CHECK(px(6, 0) == kLedDim && px(6, 12) == kLedDim && px(7, 8) == kLedDim);  // left hip yaw, right ankle, beak
  CHECK(px(7, 0) == 0 && px(7, 1) == 0 && px(6, 6) == 0);
  CHECK(px(7, 12) == kLedMid && px(5, 3) == 0);  // the loop's heartbeat, on for the first half of each second
  CHECK(out.led3 == kRgbGreen && out.led4 == kRgbOff);
  render_display(in, 1500, &out);
  CHECK(out.led3 == kRgbOff && px(7, 12) == 0);
  // The ticker scrolls in from the right after a blank screen.
  render_display(in, 0, &out);
  CHECK(text_lit() == 0);
  render_display(in, kScrollMs * 20, &out);  // 7 columns of text on screen: "SA"
  CHECK(text_lit() > 0);

  // Servos answering are bright; the IMU present but uncalibrated is dim; the link follows the Linux side.
  in.joint_answering[kRKnee] = true;
  in.imu_present = true;
  in.ms_since_call = 200;
  render_display(in, 1000, &out);
  CHECK(px(6, 11) == kLedFull && px(6, 3) == kLedDim);
  CHECK(px(7, 0) == kLedDim && px(7, 1) == kLedDim);
  CHECK(out.led4 == kRgbGreen && px(6, 6) == kLedMid);
  display_text(in, text);
  CHECK(strcmp(text, "SAFE  SV 1/15  IMU ?  LINUX OK") == 0);
  in.ms_since_command = 100;
  render_display(in, 1000, &out);
  CHECK(out.led4 == kRgbBlue && px(6, 6) == kLedFull);
  in.ms_since_call = in.ms_since_command = 5000;  // went silent
  render_display(in, 1000, &out);
  CHECK(out.led4 == kRgbRed);
  render_display(in, 1500, &out);
  CHECK(out.led4 == kRgbOff);

  // Calibrated IMU: both dots bright, the tilt in the ticker.
  in.status.imu_ok = in.status.gyro_ok = true;
  in.status.pitch_deg = 3.4f;
  in.status.roll_deg = -12.6f;
  render_display(in, 1000, &out);
  CHECK(px(7, 0) == kLedFull && px(7, 1) == kLedFull);
  display_text(in, text);
  CHECK(strcmp(text, "SAFE  SV 1/15  P+3 R-13  NO LINUX") == 0);

  // A short message from Linux stands still, centred; it gives way to the ticker when it is old.
  in.message = "HI";
  in.ms_since_message = 0;
  render_display(in, 1000, &out);
  // "HI" is 7 columns wide, so "H" is drawn at columns 3..5: 101 / 101 / 111 / 101 / 101
  CHECK(px(0, 3) == kLedFull && px(0, 4) == 0 && px(0, 5) == kLedFull && px(2, 4) == kLedFull);
  CHECK(px(0, 2) == 0 && px(0, 0) == 0);
  in.ms_since_message = kMessageHoldMs + 1;
  display_text(in, text);
  CHECK(strncmp(text, "SAFE", 4) == 0);

  // Armed: blue, posture in the ticker. Fault: steady red, the fault in words, and its joint blinking.
  in.status.mode = Mode::kArmed;
  in.status.posture = Posture::kStanding;
  render_display(in, 1500, &out);
  CHECK(out.led3 == kRgbBlue);
  display_text(in, text);
  CHECK(strncmp(text, "ARMED STAND  ", 13) == 0);
  in.status.mode = Mode::kFault;
  in.status.fault = Fault::kOverTemp;
  in.status.fault_joint = kLKnee;
  in.joint_answering[kLKnee] = true;
  in.ms_since_message = 0;  // a fault outranks a message
  display_text(in, text);
  CHECK(strcmp(text, "FAULT HOT L KNEE") == 0);
  render_display(in, 1000, &out);
  CHECK(out.led3 == kRgbRed && px(6, 3) == kLedFull);
  render_display(in, 1150, &out);
  CHECK(out.led3 == kRgbRed && px(6, 3) == 0);

  // A servo armed on the bench: magenta, said in words, outranking a message; its dot blinks. A bench fault: red.
  in.status.mode = Mode::kDisarmed;
  in.status.fault = Fault::kNone;
  in.status.fault_joint = -1;
  in.bench.mode = BenchMode::kArmed;
  in.bench.joint = kLKnee;
  in.ms_since_message = 0;
  display_text(in, text);
  CHECK(strcmp(text, "BENCH L KNEE") == 0);
  render_display(in, 1000, &out);
  CHECK(out.led3 == kRgbMagenta && px(6, 3) == kLedFull);
  render_display(in, 1150, &out);
  CHECK(px(6, 3) == 0);
  in.bench.mode = BenchMode::kFault;
  in.bench.fault = Fault::kOverload;
  display_text(in, text);
  CHECK(strcmp(text, "BENCH FAULT OVERLOAD L KNEE") == 0);
  render_display(in, 1000, &out);
  CHECK(out.led3 == kRgbRed);
  in.bench.mode = BenchMode::kOff;

  // Every printable character has a glyph or falls back to '?', and lower case draws as upper case.
  CHECK(text_width("") == 0 && text_width("A") == 3 && text_width("ABC") == 11);
  in.status.mode = Mode::kDisarmed;
  in.message = "hi";
  in.ms_since_message = 0;
  render_display(in, 1000, &out);
  CHECK(px(0, 3) == kLedFull && px(0, 5) == kLedFull && px(2, 4) == kLedFull);
}

static void test_armed_reads_every_servo_each_policy_tick() {
  // Armed, reads_per_tick servos are read each tick, round-robin: with the shipped 8, all fifteen within two ticks
  // (each joint at 50 Hz), and none twice in a tick even when more reads are allowed than there are servos.
  Rig r(calibrated_config());
  CHECK(r.cfg.servo.reads_per_tick == 8);
  r.arm_at(r.cfg.stand);
  for (int pass = 0; pass < 3; ++pass) {
    r.fake.reads.clear();
    r.run(1);
    CHECK(r.fake.reads.size() == 8);
    r.run(1);
    for (int j = 0; j < kNumJoints; ++j) {
      int n = 0;
      for (uint8_t id : r.fake.reads) n += id == r.cfg.joints[j].servo_id;
      CHECK(n >= 1);
    }
  }
  RobotConfig c = calibrated_config();
  c.servo.reads_per_tick = 99;
  Rig all(c);
  all.arm_at(all.cfg.stand);
  all.fake.reads.clear();
  all.run(1);
  CHECK(all.fake.reads.size() == kNumJoints);
  bool seen[256] = {};
  for (uint8_t id : all.fake.reads) {
    CHECK(!seen[id]);
    seen[id] = true;
  }
  // Disarmed it stays one read every 100 ms.
  all.ctl.disarm();
  all.fake.reads.clear();
  all.run(20);
  CHECK(all.fake.reads.size() == 2);
}

static void test_bench_servo_polled_every_turn() {
  // A bench with one servo assigned (the rest unassigned, as in the shipped config): disarmed, it is read on every
  // idle poll (10 Hz), not once per pass over all fifteen joints.
  RobotConfig c = kRobotConfig;
  for (int j = 0; j < kNumJoints; ++j) c.joints[j].servo_id = kServoIdUnassigned;
  c.joints[kLKnee].servo_id = 7;
  Rig r(c, false);
  r.fake.add(7);
  r.run(1);
  CHECK(r.ctl.feedback_valid(kLKnee));
  // Two servos: both read within two idle polls.
  c.joints[kBeak].servo_id = 9;
  Rig r2(c, false);
  r2.fake.add(7);
  r2.fake.add(9);
  r2.run(11);  // polls at the first tick and 100 ms later
  CHECK(r2.ctl.feedback_valid(kLKnee) && r2.ctl.feedback_valid(kBeak));
  CHECK(!r2.fake.any_motion_write() && r2.fake.torque_count() == 0);
}

static void test_bench_servo() {
  // One servo armed on its own: only that servo is written, inside its limits and speed, faults and silence
  // switch it off, and a build without bench motion cannot arm it at all.
  RobotConfig c = kRobotConfig;
  for (int j = 0; j < kNumJoints; ++j) {
    c.joints[j].servo_id = kServoIdUnassigned;
    c.joints[j].calibrated = false;
  }
  c.joints[kLKnee].servo_id = 7;
  c.joints[kLKnee].zero_step = 512;
  c.joints[kLKnee].direction = 1;
  c.joints[kLKnee].calibrated = true;
  const JointConfig& knee = c.joints[kLKnee];
  CHECK(!config_ready_for_motion(c));  // the robot could not be armed with this table

  FakeBus fake;
  ScsBus bus(fake);
  fake.add(7);
  fake.add(8);  // another servo on the bus, never to be touched

  {  // a build without bench motion
    BenchServo off(c, bus, false);
    CHECK(off.arm(kArmKey, kLKnee, 0) == Result::kMotionBuildDisabled);
    off.tick(10, false);
    CHECK(!fake.any_motion_write() && fake.torque_count() == 0);
  }

  BenchServo b(c, bus, true);
  uint32_t now = 1000;
  CHECK(b.arm(0, kLKnee, now) == Result::kBadKey);
  CHECK(b.arm(kArmKey, kRKnee, now) == Result::kNotCalibrated);
  CHECK(b.arm(kArmKey, 99, now) == Result::kNotCalibrated);
  fake.servo[7].present = false;
  CHECK(b.arm(kArmKey, kLKnee, now) == Result::kServoMissing);
  fake.servo[7].present = true;
  CHECK(!fake.any_motion_write() && fake.torque_count() == 0);
  CHECK(b.set_target(10.0f, now) == Result::kNotArmed);

  // Armed: torque on that servo only, starting where it is.
  CHECK(b.arm(kArmKey, kLKnee, now) == Result::kOk);
  CHECK(b.armed() && fake.servo[7].torque() && !fake.servo[8].torque());
  CHECK_NEAR(b.state().setpoint_deg, 0.0f, 1e-4);  // 512 = the zero step
  CHECK(b.arm(kArmKey, kLKnee, now) == Result::kAlreadyArmed);

  // A target past the limit is clamped, and approached at the joint's speed limit.
  CHECK(b.set_target(knee.max_deg + 50.0f, now) == Result::kOk);
  now += 10;
  b.tick(now, false);
  CHECK_NEAR(b.state().setpoint_deg, knee.max_speed_dps * 0.01f, 1e-3);
  for (int i = 0; i < 400; ++i) {
    now += 10;
    if (i % 10 == 0) b.set_target(knee.max_deg + 50.0f, now);  // keep talking
    b.tick(now, false);
  }
  CHECK_NEAR(b.state().setpoint_deg, knee.max_deg, 1e-3);
  CHECK(fake.servo[7].position() == static_cast<uint16_t>(deg_to_step(c, kLKnee, knee.max_deg)));
  CHECK(b.state().reading.ok && b.state().reading.ms == now);
  for (const WriteLog& w : fake.writes) CHECK(w.id == 7 || w.id == scs::kBroadcastId);

  // Silence for longer than the command timeout: torque off, not hold.
  now += c.cmd_timeout_ms + 20;
  b.tick(now, false);
  CHECK(!b.armed() && b.state().mode == BenchMode::kOff && fake.torque_count() == 0);

  // A fault (over temperature) switches it off and latches until an explicit disarm.
  CHECK(b.arm(kArmKey, kLKnee, now) == Result::kOk);
  fake.servo[7].mem[scs::reg::kPresentTemperature] = 90;
  now += 10;
  b.set_target(0.0f, now);
  b.tick(now, false);
  CHECK(b.state().mode == BenchMode::kFault && b.state().fault == Fault::kOverTemp && fake.torque_count() == 0);
  CHECK(b.arm(kArmKey, kLKnee, now) == Result::kFaultLatched);
  CHECK(b.set_target(1.0f, now) == Result::kFaultLatched);
  CHECK(!b.idle());
  // Torque-off is repeated after the fault (a write may have been lost), but never while the robot owns the bus.
  auto torque_off_broadcasts = [&]() {
    int n = 0;
    for (const WriteLog& w : fake.writes) n += w.id == scs::kBroadcastId && w.addr == scs::reg::kTorqueEnable && w.data[0] == 0;
    return n;
  };
  int before = torque_off_broadcasts();
  for (int i = 0; i < 20; ++i) b.tick(now += 10, false);
  CHECK(torque_off_broadcasts() - before >= 9);
  before = torque_off_broadcasts();
  for (int i = 0; i < 300; ++i) b.tick(now += 10, true);
  CHECK(torque_off_broadcasts() == before);
  fake.servo[7].mem[scs::reg::kPresentTemperature] = 30;
  b.disarm();
  CHECK(b.arm(kArmKey, kLKnee, now) == Result::kOk);
  b.disarm();
  CHECK(fake.torque_count() == 0 && !fake.servo[8].torque());
  // Disarming what is already off writes nothing: it cannot disturb an armed robot.
  for (int i = 0; i < 20; ++i) b.tick(now += 10, false);
  const size_t writes = fake.writes.size();
  b.disarm();
  b.tick(now += 10, false);
  CHECK(fake.writes.size() == writes && b.idle());
}

static void test_command_timeout_holds() {
  RobotConfig slow = calibrated_config();
  slow.joints[kNeckPitch].max_speed_dps = 40.0f;   // so the move outlasts the timeout
  Rig r(slow);
  r.arm_at(r.cfg.stand);
  float t[kNumJoints];
  pose_to_joints(r.cfg.stand, t);
  t[kNeckPitch] = -60.0f;
  r.ctl.set_targets(t, r.now);
  r.run(20, false);              // 0.2 s: still moving toward the target
  float mid = r.ctl.setpoint_deg()[kNeckPitch];
  CHECK(mid < 20.0f && mid > -60.0f);
  r.run(40, false);              // past 0.5 s without a heartbeat
  CHECK(r.ctl.status().flags & kStatCmdTimeout);
  float held = r.ctl.setpoint_deg()[kNeckPitch];
  CHECK(held > -60.0f);
  r.run(100, false);
  CHECK(r.ctl.setpoint_deg()[kNeckPitch] == held);        // holding, torque still on
  CHECK(r.ctl.status().mode == Mode::kArmed && r.fake.torque_count() == 15);
}

int main() {
  RUN(test_protocol_packets);
  RUN(test_protocol_parse);
  RUN(test_bus_reads);
  RUN(test_bus_gates);
  RUN(test_joint_conversion);
  RUN(test_safety_clamp_and_rate);
  RUN(test_head_interlock);
  RUN(test_hip_map);
  RUN(test_filter_hip_map);
  RUN(test_fmath_against_libm);
  RUN(test_motion_and_balance);
  RUN(test_arming_is_refused_by_default);
  RUN(test_arm_holds_position);
  RUN(test_sit_and_stand);
  RUN(test_balance_in_the_loop);
  RUN(test_faults_switch_torque_off);
  RUN(test_command_timeout_holds);
  RUN(test_gyro_in_status);
  RUN(test_status_display);
  RUN(test_armed_reads_every_servo_each_policy_tick);
  RUN(test_bench_servo_polled_every_turn);
  RUN(test_bench_servo);
  printf("\n%d tests, %d checks, %d failed\n", g_tests, g_checks, g_failed);
  return g_failed ? 1 : 0;
}
