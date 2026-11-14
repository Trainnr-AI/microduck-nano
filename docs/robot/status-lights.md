# Status lights

What the Arduino UNO Q's own LEDs say about the robot. Three sources, each showing what it knows best:

| Light | Driven by | Shows |
|---|---|---|
| LED matrix (8 x 13) | microcontroller firmware | text, every servo, the IMU, the link with Linux |
| RGB LED3 | microcontroller firmware | the safety mode |
| RGB LED4 | microcontroller firmware | the link with the Linux side |
| RGB LED1 | `duckd` on Linux | what the control loop is doing |
| RGB LED2 | the Linux system | panic (red), Wi-Fi traffic (green), Bluetooth (blue); not ours |

The firmware's lights come from `src/duck/display.cpp` (host-tested in `firmware/tests`); `duckd`'s from
`runtime/src/status_led.rs`.

## The matrix

```
col   0 1 2 3 4 5 6 7 8 9 10 11 12
row0  T T T T T T T T T T T  T  T
row1  T T T T T T T T T T T  T  T     T  text, letters 3 x 5
row2  T T T T T T T T T T T  T  T
row3  T T T T T T T T T T T  T  T
row4  T T T T T T T T T T T  T  T
row5  . . . . . . . . . . .  .  .
row6  l l l l l . K . r r r  r  r     l / r  left / right leg: hip yaw, hip roll, hip pitch, knee, ankle
row7  G W . . h h h h h . .  .  A     h  neck pitch, head pitch, head yaw, head roll, beak
```

**Text** (rows 0-4). Text that fits stands still; longer text scrolls in from the right. In order of priority:

1. A fault, in words: `FAULT HOT L KNEE`, `FAULT NO ANSWER R ANKLE`, `FAULT FALL`.
2. A message from the Linux side (`duck_say`, `duckd say TEXT`, `Duck.say()` in Python). It stays up for 3 s after
   it was last sent, or two passes if it scrolls; send it again to keep it. `duckd run` announces itself this way.
3. The status ticker, live: `SAFE` or `ARMED STAND`; servos answering (`SV 0/15`); the IMU (`NO IMU`, `IMU ?` when
   it answered but its mounting is not calibrated or its reading is stale, otherwise the trunk's pitch and roll in
   degrees, `P+3 R-1`); and `LINUX OK` or `NO LINUX`.

**Servo dots.** Bright: the servo answers. Dim: it does not. Blinking: it caused the fault.

**K, the link with Linux.** Bright: commands arrived in the last 0.5 s (`duckd run`, heartbeats). Half: only reads
in the last 1.5 s (the app's watcher reads once a second). A short flash each second: Linux was talking and went
silent. Off: nothing since power-up.

**G, W: gravity and gyro.** Bright: fresh and calibrated. Dim: the IMU answered at boot but this reading is stale or
the mounting matrix in `robot.json` is not calibrated. Off: no IMU.

**A** blinks once a second while the firmware's loop runs. If it stops, the loop has stopped.

Which side is "left": the matrix as you look at it. Where the board sits in the robot is not decided yet; flip the
dot table in `display.cpp` if it ends up mirrored.

## LED3: the safety mode

| Colour | Meaning |
|---|---|
| Green, a short flash each second | DISARMED: no torque, reads only |
| Blue, steady | ARMED |
| Blue, slow blink | ARMED in a supported stand (robot held off the floor) |
| Red, steady | FAULT: torque off, latched until cleared |
| Red, blinking 4 fast and 4 slow, everything else dark | not ours: the sketch crashed. Arduino's core does this |

A fault is steady on purpose, so it cannot be taken for the core's crash pattern on the same LED.

## LED4: the link with Linux

| Colour | Meaning |
|---|---|
| Blue | commands in the last 0.5 s |
| Green | reads in the last 1.5 s |
| Red, a short flash each second | Linux was talking and went silent |
| Off | nothing since power-up |

## LED1: `duckd`

| Colour | Meaning |
|---|---|
| Green | `duckd run`: holding in place (no policy) |
| Blue | the policy produced this tick's targets |
| Yellow | the policy could not run this tick: an input it needs was not measured; holding in place |
| Red | the exchange with the firmware failed |
| Off | `duckd` is not running a loop |

`duckd` writes `/sys/class/leds/unoq:user-{red,green,blue}1` (or `{red,green,blue}:user` on images that name them
that way); the `arduino` user may, through the `gpiod` group. Arduino App Lab also writes this LED when an app
starts or stops.

## Cost

The matrix is refreshed by an interrupt every 10 microseconds (Arduino's driver, timer TIM17, pins PF0-PF10: none
shared with the servo UART on D0/D1 or the Qwiic bus). Measured on the board on 2026-11-15 with the matrix on, no
servos attached, link at 460800 baud: firmware tick worst case about 50 us including the display; `duck_step`
5.7 ms median over 500 calls; `duckd run` 1,000 ticks at 50 Hz and 2,000 at 100 Hz with no errors and no missed
deadlines. The link was no slower than before the matrix (`runtime/README.md`).

Not measured: the servo bus at 1 Mbaud with the interrupt running. If servos stop answering or replies come back
short on the bench, build with `DUCK_STATUS_MATRIX 0` (`duck_build_config.h`) first: the RGB LEDs keep working.
