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
