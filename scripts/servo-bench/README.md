# One servo on the bench, from a PC

PowerShell scripts that talk to one Feetech SCS0009 through the Waveshare Bus Servo Adapter (A)'s USB port (jumper
on **B**), with the servo powered from the adapter's power terminals. They need Windows PowerShell and nothing else;
from WSL, run them with `powershell.exe -ExecutionPolicy Bypass -File <script>`. Find the port with the adapter's
USB ID `1A86:55D3` (a CH343).

| Script | What it does | Writes to the servo? |
|---|---|---|
| `scs_probe.ps1` | Pings one or more IDs and prints every byte that comes back | no |
| `scs_read.ps1` | Reads a range of the memory table, once or repeatedly (for watching position while the horn is turned by hand) | no |
| `scs_move.ps1` | **Moves the servo.** Centres it, then sweeps it +-60 degrees at rising speeds, recording position, speed, load, voltage and temperature about 900 times a second; switches torque off at the end, on an error from the servo, at 55 C, or after half a second at the load limit | RAM only: torque enable and goal. Nothing in its permanent memory |

Run `scs_move.ps1` only with the horn free to turn, fingers clear, and someone watching with a hand on the power.
The same measurements belong in `duckd bench` through the robot's own firmware
([bench checklist](../../docs/robot/bench-checklist.md), step 8b); these scripts exist because the adapter's USB port
worked before its UART header did.

## On the board, with the adapter on the UNO Q's own USB port

With a hub (or the adapter alone) on the UNO Q's USB-C port the board is the USB host: the adapter appears on its
Linux side as `/dev/ttyACM0`, and the board is reached over Wi-Fi with SSH instead of `adb`
([handoff](../../docs/project/handoff-2026-11-15.md)). One exchange with the servo takes about 1.9 ms this way.

| Script | What it does | Writes to the servo? |
|---|---|---|
| `scs_linux.py` | Pings the servo and reads its state for two seconds (Python's standard library only) | no |
| `imu_follow.py` | **Moves the servo.** For 30 s the servo follows the tilt of the IMU, +-45 degrees about its centre at a gentle speed, 50 times a second; torque off at the end and on any error. Needs `firmware/tools/router_call.py` beside it and the firmware app running with the IMU | RAM only |

This path goes round the microcontroller, so none of the firmware's limits apply to it. It is for one servo on a
bench, never for a servo fitted in the robot.

## On the board, through the firmware

With the adapter's UART header on the UNO Q's D0 / D1 (jumper on **A**) every command goes through the
microcontroller firmware, so its joint limits, speed limit and faults apply. These need
`firmware/tools/router_call.py` beside them and the firmware app running.

| Script | What it does | Writes to the servo? |
|---|---|---|
| `set_id.py` | Gives the one servo on the bus a new ID and reads it back. Refuses unless exactly one servo answers, at the old ID, cleanly. Provisioning build (`DUCK_ENABLE_EEPROM_WRITE 1`) | **its permanent memory**: the ID |
| `bench_swing.py` | **Moves one servo.** A few slow swings between two angles, then mid-range and torque off: for seeing which servo has which ID, and for leaving a servo where its horn should be fitted. Bench build | RAM only |
| `bench_follow.py` | **Moves one or two servos.** They follow the IMU's tilt, 50 times a second. Bench build, IMU running | RAM only |

## Measuring the servo for its simulation model

`duckd bench` ([runtime](../../runtime/README.md), bench checklist 8b) moves one servo through the firmware's
bench mode and records it for trainnr. Two scripts go with it, both run on the PC:

| Script | What it does |
|---|---|
| `bench_summary.py` | Reads such recordings and prints the read rate, the range, the top speed, the servo's speed report against its position, and for steps how fast it got there and where it rested |
| `to_lerobot.py` | Writes a recording as a LeRobot dataset, which trainnr's servo fit accepts today (see step 4) |
| `make_pendulum.py` | Writes the bench trainnr's `servo-bench` fit wants from three measurements: a one-hinge pendulum model and `bench.json` |

A bare horn gives speed and timing, not torque. For torque the servo has to drive something whose inertia and
weight are known:

1. An arm about 100 mm from the servo's axis to a weight at its end, stiff and light, screwed to the horn. Weigh
   the arm with the horn and screw, weigh the weight, measure the length.
2. Clamp the servo with its axis horizontal, so the arm hangs straight down. With the torque off, where it comes to
   rest is the joint's zero (`zero_step` of the bench joint).
3. `duckd bench` with steps and with chirps, 12 s or more each, starting at rest.
4. `make_pendulum.py` with the three numbers, the voltage the servo reported and its register 21;
   `onboard_robot` the folder, `ingest_recording` each recording, `identify_system` with method `servo-bench`.
   On 2026-11-24 that last call refuses any recording `duckd` writes ("no joint.command channel"): trainnr
   reads the MCAP's `/joint_commands` under another name
   ([Trainnr-AI/trainnr#147](https://github.com/Trainnr-AI/trainnr/issues/147)). Until that is fixed,
   `to_lerobot.py` writes the same recording as a LeRobot dataset, a format in which trainnr does read the
   command under the name the fit wants; three bare-horn recordings went in that way on 2026-11-24. The fit
   itself has not been run on them.
5. Again with a second and a third weight (trainnr asks for two or three), each as a bench of its own. About 30,
   60 and 100 g at 100 mm load an SCS0009 with up to roughly an eighth, a quarter and four tenths of its rated
   2.3 kg cm.

## Results

`results/sweep-20261118-aa-pack.csv.gz`: the first powered run, 2026-11-18. One new SCS0009, a bare horn, four
part-used AA cells (5.3 V at rest). Columns: milliseconds, step name, goal (steps), speed asked (steps/s), position,
speed word, load word, voltage x10, temperature, the servo's error byte. That run stopped itself in the 1350 steps/s
sweep: its stall guard counted readings, not time, and tripped after 28 ms at high load. The guard here is fixed.
What it measured is in [firmware.md](../../docs/design/firmware.md).

`imu_follow.py`, 2026-11-18, same servo and cells, the IMU on Qwiic and the adapter on the UNO Q's USB port: 1,500
ticks in 30 s (50 a second), the horn followed the IMU from -45 to +45 degrees (positions 360 to 662), settled within
4 steps of the target at rest, lowest supply 5.1 V, no error from the servo.

`bench_follow.py` with two joints, 2026-11-19, the first try with two servos through the firmware's bench mode (IDs 1 and 2 on the
adapter's UART header, the same part-used AA cells): both armed without a jump, and three ticks later the firmware
stopped both with an under-voltage fault. The pack fell from 5.1 V to 3.9 V the moment two servos drew current.
Nothing was wrong with the firmware or the servos; two servos need a supply that holds its voltage.

`bench_follow.py` with one joint, 2026-11-19 (`results/imu-follow-firmware-20261119.csv.gz`): the servo with ID 2
as `left_ankle`, through the firmware's bench mode, goal speed 300 steps/s, the same AA cells. 1,500 ticks in 30 s;
the firmware first brought the joint from 121 degrees to its 90 degree limit, then it followed the IMU's tilt between
-29 and +45 degrees; lowest supply 5.0 V, peak drive 30 %, 23 C. With the firmware's default goal speed of 0 ("as
fast as it can") the same servo pulled the pack down to 3.9 V on its first move and the firmware stopped it.

`bench_follow.py` with two joints, 2026-11-21 (`results/imu-follow-two-servos-20261121.csv.gz`): both servos
through the firmware's bench mode on four fresh AA cells (6.4 V at rest). 4,500 ticks in 90 s, no fault. Each servo
followed its tilt axis over the whole +-45 degrees (one stopped at -29: its tilt never went further), a median of
0.1 to 0.3 degrees from its target and about 80 ms behind it while moving; lowest supply 6.0 V, peak drive 34 %,
25 C. Three calls to the firmware came back garbled and were sent again (two gaps over 40 ms, the longest 180 ms):
with servos armed the link between the board's two processors loses a byte now and then, which it did not with
nothing armed. Not yet measured properly.

Numbering the servos, 2026-11-21 (`set_id.py`, then `bench_swing.py` on each): thirteen new SCS0009, one at a time
alone on the bus, from the factory ID 1 to IDs 3 through 15; with the two numbered earlier that is all fifteen. Each
write answered, each servo then read back its new ID with its memory locked, and nothing was left at ID 1. Every
one arrived parked at an end of its travel, at step 13 to 18 or 1001 to 1009, so at or just past the 20 / 1003
limits in its own memory; the firmware brought each inside its bench joint's limits at its speed limit without a
fault. One at a time on the AA cells: 5.8 V at the lowest, peak drive 24 to 34 %. Twice after changing servos
nothing answered on the bus until the plugs had been pushed home again, and once a servo that had just moved was
silent for 40 seconds and then answered; the IMU's Qwiic plug came loose the same afternoon. Which connection it
was is not known.

`duckd bench`, one servo with a bare horn, 2026-11-23: the first recordings for trainnr. Servo ID 15 as the bench
joint `head_yaw` (narrowed to +-100 degrees and given a speed limit of 1000 deg/s on the board, so a step of the
target reaches the servo as a step), goal speed 0, four AA cells reading 5.8 V with the servo at rest, 23 to
25 C. Steps and chirps of +-5, +-15 and +-30 degrees, 12 to 14 s each, about 88 reads of the servo a second, no
fault. They are in the trainnr project as `scs0009-bare-horn-steps-a5-2026-11-23@45236beb5a99`,
`-steps-a15-...@1870401d7cd6`, `-steps-a30-...@bd03836aefcd`, `-chirp-a5-...@c27e25cf442b`,
`-chirp-a15-...@a3a93b7f175c` and `-chirp-a30-...@12bd491ed9b1`. From `bench_summary.py`:

| Jump of the target | 10 % covered after | 90 % after | Past the target | Rest, 0.48 s after |
|---|---|---|---|---|
| 10 degrees | 39 ms | 85 ms | none | 0.05 degrees off |
| 30 degrees | 51 ms | 117 ms | 0.25 degrees | 0.35 degrees off |
| 60 degrees | 60 ms | 162 ms | 0.5 degrees | 0.5 degrees off |

Medians of 12 jumps each, timed from when `duckd` sent the target, so the link, the firmware's tick and its
1000 deg/s limit on the goal are in them. Top speed on the 60 degree jumps: 557 deg/s by the servo's own report,
about 700 by its positions (read stamps of 1 ms make that one noisy). The servo's speed report is 0.91 to 0.94 of
the speed its positions give on the larger moves, so the firmware reads register 58 in the right unit (steps/s).
Its registers 21 to 27 read 15, 15, 0, 0, 45, 1, 1; by Feetech's library header for this family those are the position
loop's P, D and I, the smallest drive, and the dead band each way (read only; not checked by changing them). **No torque follows from any of this**: with
nothing on the horn there is nothing known to push against, so no model was fitted. One call in the second run
came back garbled and `duckd bench` stopped half a second early; since then it passes over a single failed
exchange (four of about 9,400 that day).
