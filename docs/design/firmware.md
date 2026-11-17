# Firmware design

How the firmware is split between the UNO Q's two processors, what keeps the robot safe, and the interface between
the two sides. Paths are relative to the `firmware/` folder unless they start with `firmware/`.

## Who does what

The UNO Q has two processors: a Qualcomm chip running Debian Linux and an STM32U585 microcontroller running Arduino
sketches on Zephyr. They talk through the **Bridge**: MessagePack-RPC over an internal serial line at **115200 baud**, with
messages from the sketch limited to **256 bytes**, routed by a Linux service (`arduino-router`).

| Microcontroller (STM32) | Linux |
|---|---|
| Servo bus (1 Mbaud, D0 / D1) | Behaviour, later a walking policy |
| IMU (Qwiic) | Camera (USB) |
| 100 Hz loop: setpoints, balance law, sit / stand trajectory | Sends requests: arm, stand, sit, joint targets |
| Joint limits, interlocks, rate limit | Reads status and telemetry |
| Overload / temperature / voltage / fall / comms watch, torque off | Sends a heartbeat while it wants the robot to move |

Why this split: the Bridge is a slow, non-real-time link (a full set of 15 joint targets is about 90 bytes, so some tens of
commands per second is the ceiling, with milliseconds of jitter), and Linux can stall or crash. So nothing that keeps the
robot upright or protects a part may depend on it. Linux only *asks*; every request goes through the microcontroller's
safety layer, and if Linux goes silent the robot holds its pose.

## Safety model

Three independent locks stand between a command and a moving servo:

1. **Build switch.** `DUCK_ENABLE_MOTION` in `sketch/duck_build_config.h` is `0` by default. In that build the bus driver
   refuses every write that could enable torque or set a goal position: the bytes are never sent, whatever is asked.
2. **Data.** `config/robot.json` ships with `servo_id: null` and `calibrated: false` on every joint and on the IMU. The
   firmware will not arm until all of them are filled in.
3. **Arm step.** Even then the robot starts disarmed. Arming needs the arm key, every servo answering with a healthy
   status, and a live, calibrated IMU. On arming, each servo's goal is first set to where it already is, then torque is
   switched on, so nothing jumps.

Writing servo EEPROM (angle limits, protection settings, IDs) is behind a separate switch, `DUCK_ENABLE_EEPROM_WRITE`, also
`0` by default.

Once armed, every joint target, from Linux or from the firmware's own trajectories, passes through the safety layer every
tick (`src/duck/safety.cpp`):

| Step | Rule | Source |
|---|---|---|
| Bad numbers | NaN / infinity are dropped; the joint keeps its setpoint | firmware |
| Joint limits | Clamp to the servo angle limits, including beak 0..+35 | JOINT_LIMITS.md section 1 |
| Rate limit | `max_speed_dps` per joint (placeholder 180 deg/s) | firmware |
| Head roll x head yaw | Roll only while abs(yaw) is inside the window; yaw only while abs(roll) <= 1 deg | JOINT_LIMITS.md section 2 |
| Hip map | Each leg's hip pitch / roll / yaw / knee must lie in the measured collision-free set; between grid values all neighbouring grid points must be free ("stricter neighbour"). A leg that would leave the set stops (all four joints) | JOINT_LIMITS.md section 2, `hip_collision_map.json` |
| Sit / stand | Only the firmware's own 3 s straight-line move between the STAND and SIT keyframes; leg targets from Linux are ignored unless the robot is standing | JOINT_LIMITS.md section 2 |

Faults (`config/robot.json`, `faults`): load above 90 % for 1 s, 65 degC, under 4.3 V, the servo's own overload / overheat
flag, a servo not answering 5 reads in a row (about 0.1 s armed), or the trunk more than 30 deg off the planned pitch (or rolled 30 deg) for
0.1 s. Any of them switches **torque off on every servo** and latches; clearing a fault leaves the robot disarmed.
Linux silent for 0.5 s: hold the pose, torque stays on (a standing robot should not collapse because a process died).

Servo reads: armed, `servo.reads_per_tick` servos (8) are read every 10 ms tick, round-robin, each at most once a
tick, so all fifteen every 20 ms: every joint at 50 Hz, the policy's rate. Each read is the full feedback block
(position, speed, load, voltage, temperature), because the fault checks need all of it. Disarmed, one servo every
100 ms: a servo that is assigned but absent costs a reply timeout per read. Linux's calls are answered between ticks,
so the reads delay them; the worst tick in `duck_status` shows by how much (bench checklist, step 13).

Things to know, because they differ from what you might assume:

- **The head interlock is tighter than the documented boundary.** The design review found the head base cutting into
  the head-pitch U inside the documented window (0.16-0.40 mm at head yaw -2 to -2.5 deg with roll -10 to -15 deg) and
  clear at yaw +-1 deg with roll up to +-15. The firmware subtracts a margin from both limits (`head_interlock` in
  `robot.json`: yaw window 2.5 - 1.5 = 1.0 deg, roll 15 - 2.0 = 13 deg). The roll margin is still a placeholder.
- **The hip map is stricter than the design's.** The firmware uses one map for both legs (right = mirror). The review
  found 37 poses the design map marks free that touch or collide: the right leg is not an exact mirror (leg forward
  45-90 deg with the knee at its 68 deg limit, the right foot meets the neck bearing plate and body frame, by up to
  6.5 mm), and a few left-leg poses on the map's edge touch by 0.06-0.48 mm. All 37 are blocked for both legs
  (`hip_map.review_extra_collisions` in `robot.json`; `tools/import_design.py` re-applies them after a refresh).
- **The SIT pose is outside the hip map.** SIT has the toes in 3 deg with hip pitch 70 deg; the map says "toe in at all:
  keep hip pitch under about +15". The sit path was checked separately on the exact parts, so the firmware skips the hip
  map only for its own sit / stand trajectory and while sitting. `test_hip_map` pins this down.
- **The hip map does not cover the ankle** (it was measured with the ankle at 0) or both legs at once at large pitch.
- **Sitting needs the IMU.** The sit is refused without a live IMU, when the robot is not at the STAND keyframe (head
  included; call stand first), or when it faces uphill by more than 4 deg.
- **The planned trunk pitch during the sit is a placeholder** (a straight ramp to 35.4 deg). The balance law corrects
  against this curve, so the real one must be exported from the simulation before a real sit (see open items).
- **No hardware watchdog yet.** If the microcontroller itself hangs, the servos keep holding their last goal, protected
  only by their own overload settings. See open items.

## Firmware functions (Bridge)

Call them from Python with `duck_client.Duck(Bridge)`, or directly: `Bridge.call("duck_status")`.

| Function | Returns | Notes |
|---|---|---|
| `duck_info` | protocol, motion build, EEPROM build, joint table ready, IMU present, servo UART is Serial1, bench build | the last word is absent from firmware before the bench build existed |
| `duck_status` | mode, posture, fault, fault joint, flags, IMU ok, pitch x10, roll x10, balance x10, worst tick (us) | |
| `duck_state` | 43 integers | protocol 4. Everything a control tick needs in one reply: result (0), mode, posture, fault, fault joint, flags, IMU ok, pitch x10, roll x10, worst tick in us, then 15 measured angles (degrees x10, -32768 = no answer), then the body angular velocity about x (forward), y (left), z (up) in degrees per second x10 (-32768 each when the gyro is absent, stale, or the IMU's mounting is not calibrated), then 15 joint speeds as each servo reported them in the same read as its angle (degrees per second x10, in the joint's direction; register 58 taken as steps/s, unconfirmed; -32768 = no answer). Protocol 3 was the same without the speeds, protocol 2 without the gyro either |
| `duck_step(15 integers)` | 43 integers, as `duck_state` | protocol 4. One control tick in one round trip: targets in degrees x10, same path as `duck_set_targets` (safety layer, ignored unless armed); `result` says what happened |
| `duck_joints` / `duck_setpoints` | 15 angles, degrees x10 | measured (-32768 = no answer) / commanded after the safety layer |
| `duck_health` | 15 x (load 0.1 %, voltage 0.1 V, temperature degC) | |
| `duck_ping(id)`, `duck_scan(first, last)`, `duck_read(id, addr, n)` | bus result / IDs found / register bytes | read-only bench tools; refused while armed |
| `duck_set_echo(on)` | | for an adapter that echoes transmitted bytes |
| `duck_bench_arm(key, joint)` | result | bench build only (`DUCK_BENCH_MOTION 1`). Arms ONE joint's servo on its own, for measuring it: its own table entry must be calibrated, the robot must be disarmed. Refused with "busy" while the robot is armed; `duck_arm` is refused while a bench servo is armed. See `src/duck/bench.h` |
| `duck_bench_step(target x10)` | 12 integers | bench build only. The joint's target in degrees x10 (-32768: just read), clamped to its limits and rate-limited to its `max_speed_dps`; back: result, mode (off / armed / fault), fault, joint, read ok, position x10, the servo's speed in deg/s x10, load 0.1 % (a duty cycle, not a torque), voltage 0.1 V, temperature C, the millisecond the servo was read, setpoint x10. No target for the command timeout: torque off |
| `duck_bench_disarm()` | 0 | bench build only. Torque off, a latched bench fault forgotten. `duck_disarm` does this too |
| `duck_say(text)` | characters kept | protocol 3. Text for the LED matrix, up to 32 characters; shown for 3 s after the last call (two passes if it scrolls), a fault always wins. See [status-lights.md](../robot/status-lights.md) |
| `duck_heartbeat` | mode | send at 5 Hz or faster while commanding |
| `duck_arm(key)`, `duck_disarm`, `duck_clear_fault` | result code | key = `0xD0C5A4B3` |
| `duck_stand(supported)`, `duck_sit` | result code | `supported = true`: robot held off the floor, balance and fall detection off during the move |
| `duck_set_targets(15 floats)`, `duck_set_head(neck, head pitch, yaw, roll, beak)` | result code | degrees; NaN = leave that joint |
| `duck_write_limits`, `duck_write_protection`, `duck_set_id` | | only in a provisioning build, only disarmed |

Result, fault and flag codes are spelled out in `duck_client.py`.

## SCS0009 registers: what is and is not confirmed

Sources: Feetech's own Arduino library (`ftservo/FTServo_Arduino`, `SCSCL.h`, `SCS.cpp`), Waveshare "SC Servo - Memory
Table Explanation", and the aifitlab "FEETECH SCSCL Servo - Memory Table Analysis". Nothing was read from a real servo.

| Item | Status |
|---|---|
| Packet format, checksum, instructions (ping 1, read 2, write 3, sync write 0x83) | Feetech library |
| Two-byte values are sent high byte first on SCS servos | Feetech library (`End = 1`) and Waveshare agree. The aifitlab page contradicts itself on this |
| ID 5, baud 6, angle limits 9 / 11, torque enable 40, goal position / time / speed 42 / 44 / 46, EEPROM lock 48, present position / speed / load / voltage / temperature 56 / 58 / 60 / 62 / 63, moving 66 | Feetech library header |
| Max temperature 13, unload condition 19, holding torque 37, protection time 38, overload torque 39, status 65 | **Unconfirmed.** Only in the two public tables. They agree on the decimal addresses, but the aifitlab table prints a wrong hex value for 39, and Waveshare's text describes address 48 both as the EEPROM lock and as a torque limit |
| Load is a signed value with the sign in bit 10; 1 Mbaud is baud code 0 | Feetech library |
| Goal speed 0 means "as fast as possible" | **Unconfirmed** (the "phase" register has a bit that changes this) |
| 3.41 steps per degree (1024 steps over 300 deg) | **Unconfirmed** on a servo |
| Factory angle limits 20..1003 | Waveshare table. Matters for head yaw: +-150 deg needs steps 0..1023 with the zero exactly mid-range, so +-150 is not reachable with factory limits (about +-144) |
| Whether the Waveshare adapter echoes transmitted bytes; whether it works when powered at 5 V | **Unconfirmed** (its page gives 9 to 12.6 V input) |
