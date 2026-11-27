# The whole firmware on the bench, with the camera (2026-11-25)

One run through everything the board can do today, on the test rig: an UNO Q, one SCS0009 (ID 15, a bare horn)
behind the Mini560 regulator on four AA cells, the USB camera, no IMU answering. Every run was recorded and is in
the trainnr project `microduck-nano`.

## What ran

| Test | Result | Recording in trainnr |
|---|---|---|
| Host tests | firmware 34 tests, 44,801 checks, none failed; runtime tests pass, clippy clean | |
| `duckd time`, 100 calls of each kind | none failed; the whole state (`duck_state`) in 5.1 ms median, 6.4 ms at worst | |
| `duckd run`, read only, 10 s at 50 Hz | 500 ticks, 2 exchanges failed (the router's socket was busy), worst 27.6 ms | `fwtest-run-state-2026-11-25@01eaec2f2e2c` |
| `duckd run` with a policy and a command | 500 ticks, no error, no missed deadline; the policy ran on none, rightly: 14 of its 15 joints have no servo | `fwtest-run-policy-2026-11-25@49621ed1e2d5` |
| `duckd see` | 16.7 frames a second at 320 x 240; nothing coloured in view | |
| `duckd bench`, chirp of 15 degrees, 12 s | ran to the end, 1,003 reads, 5.0 V | `fwtest-bench-chirp-a15-2026-11-25@566555b9b533` |
| `duckd bench`, steps of 15 degrees | **the firmware switched the servo off after 1 s**: under-voltage, 4.2 V at 100 % load | `fwtest-bench-steps-a15-2026-11-25@560aedc76d4e` |
| `duckd pet` with the camera, 20 s | ran to the end; with nothing to look at it glanced about | `fwtest-pet-camera-2026-11-25@8b35f724fc3c` |

Seen and not followed up: the firmware reports an IMU present and not working; its longest tick during the servo
runs was 9.4 ms. The under-voltage is the AA cells again ([scs0009.md](scs0009.md)); the battery is the fix.

## A fault found: the camera's device number moves

After a restart of the board the camera was `/dev/video2`; `/dev/video0` and `1` were the processor's video
encoder and decoder. `duckd see` on `/dev/video0`, its default, waited for frames for ever. Now the runtime asks
each video device for its formats, takes the first that offers YUYV (`--device auto`, the default), and refuses a
named device that offers none.

## The camera gets hot

To show the camera live in the trainnr Studio it streamed without a break for ten minutes (640 x 480, MJPEG), the
longest it has ever been on: earlier runs were 4 to 20 s. The module became very hot to the touch, and cooled once
the stream stopped, so it is the heat of working, not a fault. A second run of five minutes at the lightest
setting of any use (320 x 240, YUYV, 5 frames a second asked for and delivered) left it "hot, but less". Nothing
measures its temperature; these are a finger's readings.

What follows for the robot, where the camera is on all the time and inside the head:

- The frame rate is not the lever: even 5 a second heats it, and the robot needs good frames to interact with
  what it sees (the owner's decision the same day). The runtime keeps the camera's full rate.
- The head's camera housing gets vents: six slots into the closed room in front of the camera board, in the
  design project (`agent-cad3d`, branch `camera-vents-2026-11-25`, not merged). Whether they are enough is not
  known until a printed face has been tried for ten minutes.
- Still to do in the runtime if the vents are not enough: stop the stream while the robot is idle.

## The camera live in trainnr

trainnr's camera capture takes a URL (`start_capture(source="camera", url=...)`). So that nothing on the board
listens on the network, the pictures come to the PC through SSH and are served there, to that PC only, by
[`scripts/camera-feed/feed.py`](../../scripts/camera-feed/feed.py); its first lines give the two commands. The
ten minutes are the footage `duck-camera-live-2026-11-25@771caa3bb038`.

## The walk trained afterwards

`nano-walk-04-rig`: [sim-to-real.md](../design/sim-to-real.md), stage "towards a walk that follows its command".
