#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Bench demo ON THE BOARD: one SCS servo on a USB adapter follows the tilt of the IMU.

Reads the IMU through the firmware (duck_imu_raw), writes the servo's goal through the USB adapter. RAM registers
only (torque enable, goal). Torque is switched off at the end and on any error. Needs router_call.py beside it.
"""
import math, os, select, sys, termios, time
import router_call

PORT, SID = "/dev/ttyACM0", 1
CENTRE, STEPS_PER_DEG, MAX_DEG, SPEED = 512, 3.41, 45.0, 300
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0

fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
a = termios.tcgetattr(fd)
a[0] = 0; a[1] = 0; a[3] = 0
a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
a[4] = a[5] = termios.B1000000
a[6][termios.VMIN] = 0; a[6][termios.VTIME] = 0
termios.tcsetattr(fd, termios.TCSANOW, a)

def exchange(body, want, wait=0.03):
    pkt = bytes([0xFF, 0xFF]) + bytes(body) + bytes([(~sum(body)) & 0xFF])
    termios.tcflush(fd, termios.TCIFLUSH)
    os.write(fd, pkt)
    buf = b""; end = time.monotonic() + wait
    while len(buf) < want and time.monotonic() < end:
        if select.select([fd], [], [], 0.003)[0]:
            buf += os.read(fd, 64)
    return buf

def state():
    r = exchange([SID, 4, 2, 56, 8], 14)
    if len(r) != 14 or r[0] != 255 or (~sum(r[2:-1])) & 255 != r[-1]:
        return None
    return {"err": r[4], "pos": r[5] * 256 + r[6], "volt": r[11] / 10, "temp": r[12]}

def write(addr, data):
    r = exchange([SID, len(data) + 3, 3, addr] + list(data), 6)
    return r[4] if len(r) == 6 else -1

def goal(pos, speed):
    return write(42, [pos >> 8, pos & 255, 0, 0, speed >> 8, speed & 255])

def tilt_deg():
    v = router_call.call("duck_imu_raw")[3]
    if v[0] < 0 or v[0] > 200:
        return None
    return math.degrees(math.atan2((v[1] + v[2]) / 1000.0, v[3] / 1000.0))

why = "finished"
log = []
try:
    s = state()
    if s is None:
        raise SystemExit("the servo does not answer")
    print(f"start: position {s['pos']}, {s['volt']} V, {s['temp']} C")
    goal(s["pos"], 150)
    write(40, [1])
    goal(CENTRE, 150)
    time.sleep(1.5)
    t0 = time.monotonic(); misses = 0; nxt = t0
    while time.monotonic() - t0 < SECONDS:
        t = tilt_deg()
        s = state()
        if s is None or t is None:
            misses += 1
            if misses >= 10:
                why = "ten readings in a row missing (servo or IMU)"; break
            continue
        misses = 0
        if s["err"] & 0x24:
            why = f"servo reports overload or overheat (error byte {s['err']})"; break
        if s["temp"] >= 55:
            why = f"temperature {s['temp']} C"; break
        t = max(-MAX_DEG, min(MAX_DEG, t))
        target = int(round(CENTRE + t * STEPS_PER_DEG))
        goal(target, SPEED)
        log.append((time.monotonic() - t0, t, target, s["pos"], s["volt"], s["err"]))
        nxt += 0.02
        time.sleep(max(0.0, nxt - time.monotonic()))
finally:
    write(40, [0])
    s = state()
    print("torque off;", why)
    if s: print(f"end: position {s['pos']}, {s['volt']} V, {s['temp']} C, error byte {s['err']}")

print(f"{len(log)} loop ticks, {len(log) / max(SECONDS, 1e-9):.0f} a second")
if log:
    last = -1
    for t, tilt, target, pos, volt, err in log:
        if int(t) != last:
            last = int(t)
            print(f"{t:5.1f} s  tilt {tilt:6.1f} deg  target {target:4d}  position {pos:4d}  {volt:.1f} V  err {err}")
    tilts = [r[1] for r in log]
    print(f"tilt range {min(tilts):.0f} .. {max(tilts):.0f} deg; position range {min(r[3] for r in log)} .. {max(r[3] for r in log)}; lowest voltage {min(r[4] for r in log):.1f} V")
