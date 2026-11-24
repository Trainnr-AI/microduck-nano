# Microduck Nano runtime

The Linux side of the robot's software, in Rust, built on [rigrs](https://github.com/Trainnr-AI/rigrs). It runs on
the Arduino UNO Q next to the microcontroller firmware in [`firmware/`](../firmware/), which owns the servo bus,
the limits and the faults.

> **Status: dry run.** `duckd` talks to the firmware, describes the robot to rigrs, and runs the fixed-rate loop
> with or without a policy. The firmware ignores targets unless it was built with motion enabled and armed. No
> trained policy exists yet; the policy runner has only run a hand-made test network and a random one.

## What is here

| File | What |
|---|---|
| `src/bridge.rs` | Client for the board's router: MessagePack-RPC over a Unix socket |
| `src/firmware.rs` | Typed calls to the firmware: info, status, joint angles, the one-call-per-tick `state` and `step`, the LED matrix's `say`, the bench servo |
| `src/robot.rs` | Reads `firmware/config/robot.json` and builds rigrs joint specifications, as four chains (rigrs carries at most six joints per chain today) |
| `src/policy.rs` | The policy runner: reads trainnr's `deploy.json`, checks it against the robot and the model, assembles the observation, runs inference (pure-Rust `tract`), clips and scales the action |
| `src/control.rs` | The fixed-rate loop: read state, get targets (hold in place, or from a policy), pass them through each chain's rigrs `Guard`, then through the limits over several joints, account for time |
| `limits/` (crate `duck-limits`) | The limits over several joints, as `no_std` Rust with no allocation: each leg's hip map, knee x ankle, both legs, and `settle`, which keeps as much of a move as the rules allow. The same decisions as the firmware's C++ (`safety.cpp`), proved on 4,000 shared cases (`tests/data/leg_rule_vectors.txt`, written by `make -C firmware/tests leg-vectors`). Builds for the board's microcontroller (`thumbv8m.main-none-eabihf`); written to move into rigrs |
| `src/limits.rs` | Loads those limits from `robot.json` and applies them to the targets the loop sends: a leg joint the rules hold stays at the pose last sent |
| `src/record.rs` | MCAP recordings of ROS 2 messages for trainnr (`--record`) |
| `src/bench.rs` | One servo moved on the bench for system identification (`duckd bench`) |
| `src/hear.rs` | Hearing: a microphone's samples into loudness, claps and groups of claps (`duckd listen`, `duckd run --listen`) |
| `src/see.rs` | Seeing: the camera's frames into where one coloured thing is: direction, size, distance (`duckd see`) |
| `src/chase.rs` | Going after a thing: its direction and distance into the walk's command, inside what the policy was trained on (`duckd run --chase`) |
| `src/pet.rs` | One joint behaving like part of an animal: it looks at what the camera sees and struggles when held (`duckd pet`) |
| `src/status_led.rs` | The board's Linux-side RGB LED as `duckd`'s status light |
| `src/main.rs` | `duckd`, the command-line tool: one function per command |
| `tests/runtime.rs` | Host tests against a fake router and a tiny hand-made policy (`tests/data/`); no board needed |

## Build and test

```sh
cargo test --workspace           # on a PC
```

For the board, a static 64-bit Arm binary. The inference library has assembly parts, so the cross-build needs a
C compiler for Arm; `cargo-zigbuild` supplies one without installing a toolchain:

```sh
rustup target add aarch64-unknown-linux-musl
uv tool install cargo-zigbuild          # or: pip install cargo-zigbuild
# if it reports "Failed to find zig", point it at the zig binary inside its own install:
#   export CARGO_ZIGBUILD_ZIG_PATH=$(find ~/.local/share/uv/tools/cargo-zigbuild -name zig -type f)
cargo zigbuild --release --target aarch64-unknown-linux-musl
```

The binary is about 18 MB, nearly all of it the inference library.

## Policies

A policy is what trainnr's `export_deployment` writes: a folder with `policy.onnx` and `deploy.json` (schema
`trainnr-deploy/1`). `duckd run --policy FOLDER` and `duckd policy-time --policy FOLDER` read it as trainnr wrote
it ([an example manifest](tests/data/trainnr_export_example.json)): the joints in policy order, the default pose, the
action as `default_pos + offset + scale * action`, the control rate, and the observation terms in order with their
clip and scale. The terms this robot can feed are `base_ang_vel` (the gyro), `projected_gravity`, `joint_pos_rel`,
`joint_vel_rel`, `last_action` and `command`: exactly what trainnr's walk trains on (51 values for a walk).

`duckd run --policy FOLDER --command 0.2,0,0.5` asks the policy for something (for a walk: m/s forward, m/s to
the left, rad/s turning left; zero if not given). A command outside the ranges the deployment says the policy was
trained on is refused.

`duckd` refuses a deployment of another schema, an observation it cannot measure or one with a history, joints that
are not the robot's or not in its order, a default pose outside the joint limits, or a network whose sizes differ
from the manifest's. It reads `deploy.json` as trainnr specifies it (trainnr's docs/87, written for trainnr#106):
observations scaled then clipped, `target = default_pos + scale * action` with `action.offset` equal to the default
pose, the rate from `physics_timestep_s * decimation`, and any key or observation source it does not know refused. Checked on the board on 2026-11-15 with a real export (a 50-iteration pipeline check on Pollen's
microduck, whose joints match ours): loaded and checked, 0.40 ms per inference. On a tick where something the policy needs was not measured
(a silent servo, no IMU), the policy does not run and the loop holds; a missing value is never replaced by zero.

`angular_velocity` comes from the gyro in the firmware's state message (protocol 4), in the body frame (x forward,
y left, z up) after the IMU's mounting matrix in `robot.json`. It is "not measured" until that matrix is calibrated,
and with `--exchange two-reads`, which has no gyro. `duckd run` refuses `state` and `step` against an older firmware.

`joint_vel_rel` comes from each servo's own speed report, read with its angle and carried in the same state message
(protocol 4); it is also recorded as `/joint_states` velocity. Where a servo gave none (`two-reads`), the speed is
the difference from the previous tick. The speed register's unit (steps/s) was checked on one SCS0009 on
2026-11-23 against the change of its position between reads: on the larger moves the report is 0.91 to 0.94 of
that (six bench recordings, `scripts/servo-bench/bench_summary.py`), so the unit is right and the report reads
somewhat low or late; on small moves, where a read stamp's 1 ms matters most, 0.72 to 0.83. The 15 extra words cost about 1.2 ms an exchange at 460800
(`duck_step` 5.7 to 6.9 ms; a 50 Hz tick's exchange 4.0 to 5.2 ms, bare board, 2026-11-17).

The gap before a real policy can run: the sign conventions of `projected_gravity` and `angular_velocity` are unchecked
against both a real IMU and the training simulation.

## Status light and text

`duckd run` shows what each tick did on the board's Linux-side RGB LED (LED1): green holding in place, blue the
policy driving, yellow the policy skipped for a missing input, red a failed exchange, off when it exits. It puts
`DUCKD RUN` (or `DUCKD POLICY`) on the LED matrix as it starts and `DUCKD DONE` when it stops; `duckd say TEXT`
puts any short text there. The rest of the board's lights belong to the firmware: [status-lights.md](../docs/robot/status-lights.md).

## Recording for system identification

`duckd run --record FILE.mcap` writes every tick that reached the firmware into an MCAP file of ROS 2 messages:
`/joint_states` (measured angles, NaN where a servo did not answer), `/joint_commands` (the targets sent, with
`--exchange step`) and `/imu` (orientation from pitch and roll with yaw unknown, the gyro; NaN for what is not
measured). trainnr's `ingest_recording` reads it with its `mcap` adapter; checked on 2026-11-15 with a synthetic
recording and one from the bare board, and with an independent ROS 2 decoder (`mcap-ros2-support`).

What it cannot give yet: joint velocity and torque (the firmware reports neither; a servo's "load" is a duty cycle),
and the time each servo was read (messages carry the time `duckd` received the state). Armed, the firmware reads
every joint at 50 Hz (8 servos a 10 ms tick); disarmed, one servo every 100 ms, so a disarmed recording is slow.

For one servo on the bench, `duckd bench --joint NAME --supervised [--excite chirp|steps] [--amplitude 10]
[--record FILE.mcap]` does better: with a bench build of the firmware (`DUCK_BENCH_MOTION 1`) it arms that joint
alone, moves it around where it was (faded in and out), switches it off at the end or on Ctrl-C, and records the
targets asked for (`/joint_targets`), the goals the firmware actually wrote after its clamp and speed limit
(`/joint_commands`) and every read of the servo, 100 a second at the default `--hz 100`, with its own speed report
(`/joint_states`), stamped at the moment the servo was read (the microcontroller's clock, mapped onto this machine's
by the fastest reply). The recording file is created before anything moves. It refuses without
`--supervised` and against any other build. An exchange with the firmware that fails costs its tick and is counted
(the board's internal link garbles a call now and then while a servo has power: four of about 9,400 on 2026-11-23); the run
gives up only when they have failed for 0.2 s, well inside the 0.5 s after which the firmware switches the servo
off by itself. First used with a servo on 2026-11-23 (one SCS0009, a bare horn: results in
[`scripts/servo-bench/`](../scripts/servo-bench/README.md)). trainnr fits an actuator model from such recordings
(`identify_system`, method `servo-bench`) once the servo drives an arm of known mass and length:
`scripts/servo-bench/make_pendulum.py` writes the bench it wants.

## Seeing

`duckd see [--device /dev/video0] [--colour U,V] [--ball-mm 40] [--fov 60 | --at-mm 300]` looks for one coloured
thing and prints, twice a second, the frame rate, its direction (degrees to the left of and above where the camera
points), its size in the picture and, for a ball of known size, its distance. Without `--colour` it takes the
colour in the middle of the picture: hold the toy in front of the camera once and keep the `--colour` it prints.
`--at-mm` with the ball held at that distance measures the lens instead of assuming 60 degrees.

A colour, not a trained detector, because it is the cheapest thing that gives a direction: the frames come
uncompressed (YUYV, 320 x 240) from `v4l2-ctl` as a child process, and in that format a pixel's colour is two bytes
apart from its brightness. On the board with the robot's camera (2026-11-23): 16.7 frames a second in a dim room
(the camera's automatic exposure sets the pace; it offers 30), and under a tenth of one core for `duckd` even when
the whole picture matches. What came out is the direction of a grey wall, which is all the camera had in front of
it: **no coloured thing has been tracked yet**, and how steady a colour stays under the camera's automatic white
balance is not known.

The walking policy never sees a picture. The direction becomes the walk's command:
`duckd run --policy FOLDER --chase U,V [--ball-mm 40]` turns to face the thing, walks to it as fast as the policy
was trained to (slower as it nears, when the ball's size is given), and stops within 0.12 m (`src/chase.rs`). Out
of sight for half a second, or the camera gone, and the policy is asked for nothing. It assumes the camera looks
straight ahead of the body: turning the head to keep the thing in view, and allowing for where the head points,
are not written. Tested on made-up frames and commands only; no robot walks yet.

## One servo as a pet

`duckd pet --joint NAME --supervised [--colour U,V]` is the interaction in miniature, for the one servo on the
bench (it needs what `duckd bench` needs). The joint turns to point at the coloured thing the camera sees and
glances about when there is nothing to see. Hold the horn and it struggles: bursts of 8 degrees each way, a small
move between bursts to feel whether it is free, and after four bursts it gives up and goes where the hand takes it
for three seconds. The board's matrix shows its mood.

Being held is felt through the servo itself: a free joint is where it was told to be, give or take its lag (4
degrees plus 0.1 s of the target's speed, from the bench recordings); one that stays away from its target for
0.15 s is held. A joint held perfectly still looks free until it tries to move, which is why the idle joint
glances about and why "am I free" is a small move, not a pause. The same idea is what will tell a whole robot that
it has been picked up. Tested on a made-up servo in the host tests, and run twice with a hand on a real one
(2026-11-24, one SCS0009, four AA cells at 5.7 V): **the hold was felt both times, on the first move the joint
tried, and the struggle lasted a fifth of a second**, because a servo fighting a hand pulled the cells under the
firmware's 4.3 V limit and the firmware switched it off, as it should. The first try jumped 12 degrees from side
to side; the second swept 8 degrees at 240 deg/s and ended the same way. Giving up and calming down have not been
seen on a servo: that needs a supply that holds its voltage.

## Hearing

The walking policy does not hear: it is trained in a simulation with no sound. Sound picks what the policy is
asked to do instead.

- `duckd listen --device DEVICE [--seconds 10]` prints what a microphone hears: the loudest 10 ms and the room's
  noise twice a second, and each group of claps. `DEVICE` is an ALSA capture device as `arecord -L` names it.
- `duckd run --policy FOLDER --command 0.2,0,0 --listen DEVICE` starts with the policy asked for nothing; two
  claps switch to the command, two more switch back. If the microphone stops, the policy is asked for nothing from
  there on.

The samples come from ALSA's `arecord` as a child process (16-bit mono, 16 kHz), so `duckd` links no sound library.
A clap is a burst at least 20 dB over the room's noise and louder than -30 dB of full scale that is over within
120 ms; claps less than 0.6 s apart are one group.

**Not tried with a microphone.** The robot has none: its camera turned out to be video only
([build-and-flash.md](../docs/robot/build-and-flash.md)). The thresholds were chosen on made-up sound in the host
tests; the servos' own noise next to a microphone is the real test, and nothing has been tuned against it.

## Run on the board

The firmware app must be running (see [build-and-flash.md](../docs/robot/build-and-flash.md)).

```sh
adb shell mkdir -p /home/arduino/duck
adb push target/aarch64-unknown-linux-musl/release/duckd /home/arduino/duck/
adb push ../firmware/config/robot.json /home/arduino/duck/
adb shell '/home/arduino/duck/duckd status'
adb shell 'cd /home/arduino/duck && ./duckd run --robot robot.json --hz 50 --seconds 10'
adb shell 'cd /home/arduino/duck && ./duckd policy-time --robot robot.json --policy policy_nano.onnx'
```

For steadier timing (measured below: about 10 % off each tick's exchange and a lower worst case), run the loop on a
core of its own at real-time priority with the CPU at full speed. Both need root:

```sh
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
sudo chrt -f 50 taskset -c 3 ./duckd run --robot robot.json --hz 50 --seconds 10
```

Inside the loop a call waits one tick for its reply, no longer: a reply the link loses costs that tick (the loop
holds), and one that comes late is dropped when it arrives instead of being taken for the next call's.

## What it measured (2026-11-15, bare board, default 115200 baud link)

One call to the microcontroller and back costs a fixed 5.5 ms or so plus about 0.1 ms per byte of reply
(`duckd time`, 200 calls each, median):

| Call | Values in the reply | Round trip |
|---|---|---|
| `duck_info` | 6 | 6.3 ms |
| `duck_status` | 10 | 7.3 ms |
| `duck_joints` | 15 | 10.6 ms |
| `duck_state` | 25 | 11.7 ms |
| `duck_step` (15 targets out, state back) | 25 | 14.7 ms |
| `duck_health` | 45 | 13.9 ms |

The loop at 50 Hz for 10 s (`duckd run`):

| Exchange per tick | Median / 95th / max | Missed deadlines of 500 |
|---|---|---|
| Two reads: status, then joints (firmware protocol 1) | 17.9 / 18.8 / 22.0 ms | 11 |
| One read: `duck_state` (protocol 2) | 12.2 / 13.0 / 14.8 ms | 0 |

So at the default speed a tick was 18 of 20 ms with two reads and 12 ms with one, and a full exchange with targets
(`duck_step`) was 14.7 ms. The link, not the processor, was the limit.

### With a faster link

The link speed can be raised (see [build-and-flash.md](../docs/robot/build-and-flash.md), "A faster link"). Tried on
the same board:

| Link speed | `duck_info` | `duck_state` | `duck_step` | Result |
|---|---|---|---|---|
| 115200 (default) | 6.3 ms | 11.7 ms | 14.7 ms | works |
| 230400 | 3.8 ms | 6.9 ms | 8.7 ms | works |
| **460800** | 2.8 ms | 4.3 ms | 6.4 ms | works |
| 921600, 1000000, 2000000 | | | | no replies; the router logs garbled packets |

With the firmware's LED matrix running (protocol 3, 28-word state; measured 2026-11-15): `duck_info` 2.1 ms,
`duck_state` 4.1 ms, `duck_step` 5.7 ms median over 500 calls each; `duckd run --exchange step` 1,000 ticks at 50 Hz
and 2,000 at 100 Hz, no errors, no missed deadlines, a tick's exchange 3.9 ms median.

At 460800 the loop ran 9,000 ticks (30 s each at 50 and 100 Hz, as one read and as one exchange) with no errors, no
missed deadlines and no router errors; a tick's exchange took about 5 ms. That leaves 15 ms of a 50 Hz tick for the
policy, and makes 100 Hz possible.

Measured again on 2026-11-17 with [`scripts/uno-q-bench/link_speed.sh`](../scripts/uno-q-bench/link_speed.sh)
(raw output in `scripts/uno-q-bench/results/link-speed-*.txt`): protocol 3 firmware, bare board, `duckd run --exchange
step`, a tick's exchange as median / max, and calls that got no reply within 0.5 s:

| Link | Matrix | Exchange, plain | Exchange, real-time settings | Lost replies |
|---|---|---|---|---|
| **460800** | on | 3.96 / 5.10 ms | **3.59 / 4.80 ms** | none in 6,000 ticks (and 1,500 more after) |
| 1000000 | on | 2.77 / 5.65 ms | 2.33 / 3.42 ms | several, mostly just after the app started |
| 1000000 | off | 2.83 / 3.59 ms | 2.52 / 2.92 ms | 1 to 5 a minute once settled |
| 2000000 | on or off | | | most replies longer than about 15 values (`duck_state` 426 of 500, `duck_step` and `duck_health` all) |

"Real-time settings": CPU governor `performance`, `duckd` at `SCHED_FIFO` 50 on core 3. The LED matrix's interrupt
is not the cause of the losses (they are the same without it). They grow with the length of the reply while the
requests stay short, which points at the microcontroller-to-Linux direction; the cause is not established. Luwu's
XGO-Duck runtime reaches 2 Mbps on this board by switching on the UART's hardware buffer; not tried here.

**Kept at 460800:** 1 Mbaud saves about 1.2 ms of a 20 ms tick, not worth a lost reply every minute or so. Whether
460800 stays clean once the servo bus is busy at 1 Mbaud on the same microcontroller is untested.

### Policy inference

`duckd policy-time` on the board, a random network of the size mjlab trains by default (61 observations, layers of
512, 256 and 128, 14 actions; `scripts/uno-q-bench/make_models.py` writes it): **0.48 ms median, 0.58 ms 95th,
1.04 ms max** over 3,000 runs, on one core. With the 6.4 ms exchange that is about 7 ms of a 20 ms tick.

With no servos attached every chain reports "unmeasured": the loop never had targets to send (its `step` mode fell
back to reading; the `duck_step` figures come from `duckd time`), and a policy given to `duckd run` was skipped on
all 500 ticks for want of joint angles. The policy path through the loop is covered by the host tests only.

## Next

1. A command source that sets speed and direction (gamepad or network); a microphone, so that `--listen` can be
   tried at all.
2. A trained policy: needs the SCS0009 actuator model first (see the [roadmap](../docs/project/roadmap.md)).
3. Check the gravity and gyro sign conventions against the IMU on the bench and against the training simulation.
4. The UART's hardware buffer on the microcontroller, and finding which side loses bytes, if more than 460800 baud
   is ever needed.
