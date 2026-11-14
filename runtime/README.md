# Microduck Nano runtime

The Linux side of the robot's software, in Rust, built on [rigrs](https://github.com/Trainnr-AI/rigrs). It runs on
the Arduino UNO Q next to the microcontroller firmware in [`firmware/`](../firmware/), which owns the servo bus,
the limits and the faults.

> **Status: dry run.** `duckd` talks to the firmware, describes the robot to rigrs and runs the fixed-rate loop a
> walking policy will live in. Its targets are "stay where you are", and the firmware ignores targets unless it was
> built with motion enabled and armed. There is no policy yet.

## What is here

| File | What |
|---|---|
| `src/bridge.rs` | Client for the board's router: MessagePack-RPC over a Unix socket |
| `src/firmware.rs` | Typed calls to the firmware: info, status, joint angles, and the one-call-per-tick `state` and `step` |
| `src/robot.rs` | Reads `firmware/config/robot.json` and builds rigrs joint specifications, as four chains (rigrs carries at most six joints per chain today) |
| `src/control.rs` | The fixed-rate loop: read state, pass a target through each chain's rigrs `Guard`, account for time |
| `src/main.rs` | `duckd`, the command-line tool |
| `tests/runtime.rs` | Host tests against a fake router; no board needed |

## Build and test

```sh
cargo test                       # on a PC

# for the board: a static 64-bit Arm binary, no C toolchain needed
rustup target add aarch64-unknown-linux-musl
cargo build --release --target aarch64-unknown-linux-musl
```

## Run on the board

The firmware app must be running (see [build-and-flash.md](../docs/robot/build-and-flash.md)).

```sh
adb shell mkdir -p /home/arduino/duck
adb push target/aarch64-unknown-linux-musl/release/duckd /home/arduino/duck/
adb push ../firmware/config/robot.json /home/arduino/duck/
adb shell '/home/arduino/duck/duckd status'
adb shell 'cd /home/arduino/duck && ./duckd run --robot robot.json --hz 50 --seconds 10'
```

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

At 460800 the loop ran 9,000 ticks (30 s each at 50 and 100 Hz, as one read and as one exchange) with no errors, no
missed deadlines and no router errors; a tick's exchange took about 5 ms. That leaves 15 ms of a 50 Hz tick for the
policy, and makes 100 Hz possible.

The higher speeds most likely fail because the microcontroller receives one byte per interrupt and falls behind;
Luwu's XGO-Duck runtime reaches 2 Mbps on this board by switching on the UART's hardware buffer. Not tried here.
Whether 460800 stays clean once the servo bus is busy at 1 Mbaud on the same microcontroller is also untested.

With no servos attached every chain reports "unmeasured", so the loop never had targets to send: its `step` mode
fell back to reading, and the `duck_step` figures come from `duckd time`, not from the loop.

## Next

1. The hardware receive buffer on the microcontroller's UART, if more than 460800 baud is ever needed.
2. A policy runner: observation assembly, ONNX inference, action scaling, a manifest check.
3. Sending targets, behind the firmware's arm step.
