# Making rigrs fit for general use

Microduck Nano will build its Rust runtime on [rigrs](https://github.com/Trainnr-AI/rigrs), Trainnr's robot-firmware
framework in Rust. This page says what rigrs gives us today, what it lacks for a robot it was not written for, and
the changes that would make it a framework any robot can use. Each change is meant to become an issue or pull
request in rigrs.

Read from rigrs `main` at commit `548a4c9` (2026-10-06) on 2026-11-15. The code was read, not built or run.

## What rigrs is today

A Cargo workspace of small crates, none using `unsafe`, with the control code written `no_std` so the same code runs
on a microcontroller, in a simulator and in unit tests.

| Part | What it gives |
|---|---|
| `arm` | Joint traits split by capability (`Joint`, `SensingJoint`, `Torque`), a `Guard` that authorises every command (limits, step size, stale commands, heat), homing, and `Plan` as the single input type for teleoperation, moves and learned policies |
| `sim-core` | Wheeled-base maths: kinematics, odometry, PID, command watchdog, stuck detection, a robot description |
| `hil-protocol`, `hil-host` | A host-to-robot message set compiled into both ends, and hardware-in-the-loop: the firmware runs on a chip or emulator while the host simulates the body; sessions can be recorded and replayed |
| Drivers | MPU6050 IMU, PCA9685 servo PWM board, OV7670 camera, quadrature encoder, a gearmotor joint, on `embedded-hal` 1.0 with mocked-bus tests |
| `vision`, `teleop-web`, `belief-viz`, `sim-run` | Host tools: detector and chase loop, phone teleoperation, Rerun views, a simulator |
| `firmware/` | Reference firmware on embassy for the Raspberry Pi Pico (RP2040) and Pico 2 (RP2350), runnable on an emulator |

It grew out of one bench: a two-wheeled base and a hobby-servo arm on a Pico. Its README says so, and says the APIs
will change as it grows to more robots.

## What fits the Nano as it is

- **The safety shape.** `Guard::authorise` returning a named `Verdict` is the same idea as our firmware's safety
  layer, and "hold the last position when commands go quiet" is the right failsafe for a standing biped too.
- **`Plan` as the one input.** A 50 Hz walking policy is a producer of one-step plans; sit/stand is an interpolated
  plan. Nothing new is needed for that.
- **Capability traits.** The SCS0009 reports position, load, voltage and temperature, so it is a `SensingJoint` with
  `Torque`.
- **The discipline:** no `unsafe`, no `unwrap` in loop code, fixed-size buffers, tests on the laptop first.

## What is missing

Ordered by how much each blocks a second robot.

### 1. Joint count and robot shape are fixed to one arm

`arm::MAX_JOINTS` is 6 and the description type is `ArmSpec`. The Nano has 15 servos in four chains (two legs,
neck and head, beak). **Change:** make the joint count a const generic (or a larger cap chosen by a feature), and
rename the joint-space parts to something shape-neutral (`joints`), leaving `arm` as one user of it. A robot
description should be a set of named joints with limits, zero, direction and gearing, loadable from data.

### 2. No bus-servo driver, and joints are commanded one at a time

There is no Feetech (SCS/STS) or Dynamixel protocol crate, although the `arm` crate is designed around Feetech's
SO-101. `Joint::command` and `SensingJoint::measured` act on one joint; fifteen joints at 50-100 Hz on one
half-duplex wire need group operations. **Change:** add `rigrs-feetech` (packet build and parse, ping, read, write,
sync write, register tables with each address marked confirmed or not) and a `JointBus` trait for "write all
targets" and "read all states" in one exchange. Extend sensing beyond temperature: load, supply voltage, status
flags. Our C++ protocol driver and its test vectors can seed this.

### 3. Safety rules stop at single joints

`Guard` checks each joint's limits, step, staleness and heat. Real robots also need rules across joints and across
sensors. **Change:** add
- **combination limits:** a window on one joint that depends on another (our head roll and yaw), and lookup-table
  maps over several joints with a "strictest neighbour" rule (our hip map);
- **more fault sources:** overload, under-voltage, servo status flags, communication loss;
- **body-level faults:** fall detection from an IMU;
- **a per-robot fault response:** an arm must hold; a fallen biped must release. Today "hold" is built in.

### 4. Only Pico firmware, and no way into an existing C or C++ firmware

The reference firmware and its support crates target the RP2040 and RP2350. The UNO Q's microcontroller is an
STM32U585 running Arduino's core on Zephyr, with a C++ library for the link to Linux. **Change:**
- split board support from robot logic, and add one STM32 reference firmware (embassy-stm32);
- add a supported path for using the `no_std` crates as a static library with a C header inside someone else's
  firmware (Arduino, Zephyr, vendor SDKs). Many boards will never run a pure-Rust image;
- say which crates need which chip features.

### 5. The scalar type is `f64` throughout

`arm` and `sim-core` use `f64` in about 270 places and `f32` in 2. Small microcontrollers have single-precision
float hardware or none, so every `f64` operation is a software routine. **Change:** make the control crates generic
over the scalar, or use `f32` in loop code, and measure a control tick on a Cortex-M0+ and an M33 in CI.

### 6. The host link is one text protocol over one serial port

`hil-protocol` is line-based text shaped around the bench robot's messages. **Change:** keep it, and add a versioned
fixed-size binary framing with a checksum, a state message sized by joint count plus IMU, and a transport trait so
the same messages travel over UART, USB, a Unix socket or an RPC bridge such as Arduino's (MessagePack-RPC). A link
budget belongs in the docs. Measured on the UNO Q with the Nano's runtime: one call to the microcontroller and back
takes about 9 ms at the default 115200 baud, so two reads fill 18 ms of a 20 ms tick.

### 7. No runtime for the computer that hosts the policy

The host side assumes a laptop driving a chip over serial. Robots like the Nano carry a Linux computer that runs
the policy every 20 ms. **Change:** add a `std` runtime skeleton: a fixed-rate loop with deadline accounting, the same
`Guard`, a state snapshot, and a command interface other processes can use. Pollen's `robotd` is the model.

### 8. No learned-policy runner

`Plan` is ready for policies but nothing loads or runs one. **Change:** add a `policy` crate: observation assembly
from robot state and commands, input normalisation, action scaling and default pose, a manifest that pins the
observation layout and joint order, and an ONNX backend behind a feature. It should refuse a policy whose manifest
does not match the robot, and take trainnr's exported deployment as its input. The Nano's runtime now has a first
version of this (`runtime/src/policy.rs`, using the pure-Rust `tract` for inference) that could move into rigrs.

### 9. One IMU driver and no IMU trait

Only the MPU6050 is supported. **Change:** an `Imu` trait (gravity direction, angular rate, timestamp, health) and a
BNO08x driver, so fall detection and balance do not depend on one chip.

### 10. Robot numbers live in code

`ArmSpec::so101_four_dof()` and friends are functions. **Change:** load the description from one data file
(checked at build time for `no_std`), the same file the training side reads, so limits, zeros and directions cannot
drift between simulation, firmware and host. Our `robot.json` plus generated header is one working pattern.

### 11. Recording that closes the loop with trainnr

rigrs can record and replay a session. **Change:** write recordings in a format trainnr ingests directly, with
commanded and measured position, load, voltage and temperature per joint at a known rate, so system identification
needs no conversion step.

### 12. Provisioning and calibration

Assigning servo IDs, setting zero offsets, writing angle limits and protection settings are needed on every
bus-servo robot and are risky. **Change:** a provisioning tool with permanent-memory writes behind an explicit
build feature, read-back verification, and one servo at a time.

### 13. Packaging

Crates are named `arm`, `blob`, `vision`, `sim-core`, at version 0.1.0 with path dependencies. Those names cannot be
published without colliding, so users must depend on the git repository. **Change:** prefix every crate `rigrs-`,
publish them, state a minimum Rust version policy (the toolchain is pinned to 1.97.1), document the features, and
keep a changelog per release.

### 14. Documentation written as a build diary

The docs record how the bench robot was built, stage by stage. That history is valuable but a newcomer needs a
guide. **Change:** separate "how to bring up your robot with rigrs" (describe it, implement the traits, test on the
laptop, run in the loop, flash) from the dated records, add an example per robot type, and move the bench's parts
list and rig notes under a `project/` folder.

### 15. Hardware-in-the-loop tied to one emulator

The loop test runs the firmware on an RP2040 emulator against rigrs's own wheeled physics. **Change:** a `Body` trait
for the simulated robot so the same harness can drive a MuJoCo model (the one trainnr trains on), and a transport
that does not assume the emulator.

## How the Nano will use it

1. **Linux-side runtime in Rust**, in this repository under [`runtime/`](../../runtime/), depending on rigrs. Started
   2026-11-15: it reads state from the microcontroller and passes targets through rigrs's `Guard`, one guard per
   chain of at most six joints, which works round item 1 for now. Running a policy at 50 Hz and sending targets needs
   items 6, 7 and 8.
2. **Move our tested C++ logic into rigrs crates** as each is needed: the Feetech protocol (item 2), then the
   combination limits and fault handling (item 3), checked against the same test vectors our C++ tests use.
3. **Keep the microcontroller firmware in C++** until the robot walks. Then try item 4's static-library route so the
   limits and safety code are one Rust implementation on both processors.

Until an item lands upstream, the Nano carries the code in its own runtime crate and marks it as destined for rigrs.

## Open questions for rigrs

- Should legged robots get their own crate beside `arm`, or should both sit on one shape-neutral joints crate?
- Is "no `unsafe` anywhere" compatible with the static-library route, which needs a C interface?
- Does rigrs want an ONNX dependency in the workspace, or a policy trait with the backend left to the user?
