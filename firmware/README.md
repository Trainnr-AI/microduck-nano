# Microduck Nano firmware

What runs on the robot's Arduino UNO Q: the servo bus, the IMU, joint limits and interlocks, sit / stand, and the link to
the Linux side.

> **Status: scaffold, bench-safe by default. Nothing here has moved a servo.**
> The code builds, its unit tests pass on a PC, and the default build has been flashed to a bare UNO Q (no servos, no IMU,
> no battery) where the Linux side and the microcontroller talk to each other. Every limit, threshold and register address
> is **provisional**: the mechanical design is still under review and no number has been confirmed on a real servo.
> There is no walking gait and no policy runtime yet.
>
> This robot is meant to be used around children. Read the [safety model](../docs/design/firmware.md#safety-model) and work through the
> [bench checklist](../docs/robot/bench-checklist.md) in order before
> enabling motion.

## What is where

| Path | What |
|---|---|
| `config/robot.json` | **The one data file**: joint limits, servo IDs and zeros (placeholders), poses, interlocks, hip map, balance, fault thresholds. Edit this, not the code |
| `tools/gen_config.py` | Turns `robot.json` into `robot_config_data.h` for the firmware; prints the servo limits in steps |
| `tools/import_design.py` | Refreshes the design-derived numbers in `robot.json` from the design repository |
| `tools/set-link-speed.sh` | Changes the speed of the link between the board's two processors (run on the board with sudo) |
| `tools/router_call.py` | Calls a firmware function from a shell on the board (debugging) |
| `app/microduck/` | The Arduino App Lab app that goes on the board |
| `app/microduck/sketch/` | Microcontroller side: `sketch.ino` (shared state, setup, control loop), `rpc.cpp` (Bridge functions; `rpc_bench.cpp` and `rpc_provision.cpp` for the bench and provisioning builds), `status_display.cpp` (LEDs), `uart_transport.h` (servo UART), `imu.cpp` (BNO086), `duck_build_config.h` (build switches) |
| `app/microduck/sketch/src/duck/` | Portable core, no Arduino dependency: SCS protocol and bus driver, joint table, safety layer, sit / stand, balance, state machine |
| `app/microduck/python/` | Linux side: `main.py` (prints the firmware's status), `duck_client.py` (one method per firmware function) |
| `tests/` | Unit tests of the portable core against simulated servos; run on a PC |

## Documentation

| Page | What |
|---|---|
| [docs/design/firmware.md](../docs/design/firmware.md) | Who does what on the two processors, the safety model, the Bridge functions, servo registers |
| [docs/robot/build-and-flash.md](../docs/robot/build-and-flash.md) | Reaching the board, building, flashing, wiring, things that went wrong |
| [docs/robot/bench-checklist.md](../docs/robot/bench-checklist.md) | First steps with real servos, IMU and power (not done yet) |

## Tests

```sh
make -C firmware/tests test        # needs g++ and make
```

21 tests of the portable core against simulated servos (`tests/fake_servos.h`): packet bytes and parsing, the write
gates, degree / step conversion, clamps, rate limit, head interlock (including that yaw 2.5 with roll 15 cannot be
reached), the hip map against the table in JOINT_LIMITS.md, sit and stand, the balance law, every fault switching torque
off, the command timeout, the gyro turned into the body frame and reported only while fresh, what the status lights show, that armed every servo is read each two ticks and none twice in one, that a bench with a few servos reads each of them every idle poll, the bench build's single servo (only it written, clamped, rate-limited, off on a fault or on silence, impossible to arm in other builds), and that a default build or a placeholder config never sends a torque or goal write.
The simulated servos follow the public protocol documents; passing these tests says nothing about a real SCS0009.

## Changing the robot's numbers

1. Edit `config/robot.json` (angles in degrees from each joint's zero, signs as in the robot's MJCF).
2. `python3 firmware/tools/gen_config.py` regenerates `robot_config_data.h` and prints each servo's limits in steps.
3. `make -C firmware/tests test`, then push the app again.

When the design changes: `python3 firmware/tools/import_design.py --design <design repo>/projects/xgoduck_scs0009`
refreshes the joint limits, the SIT pose and the hip map, and leaves everything else alone.

Per joint: `servo_id` (bus ID), `zero_step` (servo step at the joint's zero; horns are fitted with the servo at mid-range,
so near 512), `direction` (+1 or -1), `calibrated`. Step = zero_step + direction x round(degrees x 3.41), clamped to
0..1023.

## Open items

- Planned trunk pitch during sit / stand: export the simulation's reference curve (`reference_tilt()` in the design
  repository's `tools/sim/sit_stand_robust.py`) into `motion.planned_trunk_pitch_deg`.
- The head interlock's roll margin, and every other placeholder in `robot.json`. The review's yaw margin and extra
  hip-map collisions are in; they should be removed again only when the parts are fixed and re-measured.
- Servo IDs, zeros, directions; IMU mounting matrix.
- Hardware watchdog on the microcontroller, and what the servos should do if it resets.
- IMU driver is compiled but has never run with a sensor; the gravity report's sign and rate are unverified.
- Telemetry for system identification (logging positions and loads at the loop rate) is not built; the Bridge is too
  slow to stream it live, so it will need buffering on the microcontroller or a different link.
- Camera, behaviour, walking: nothing yet. No gait exists.
