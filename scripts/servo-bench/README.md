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
