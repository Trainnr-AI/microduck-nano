#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Ping an SCS servo and read its state through a USB serial adapter on Linux (no extra modules)."""
import os, sys, termios, time, select

def open_port(path, baud=termios.B1000000):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = 0; a[1] = 0; a[3] = 0
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    a[4] = a[5] = baud
    a[6][termios.VMIN] = 0; a[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, a)
    return fd

def exchange(fd, body, want, wait=0.05):
    pkt = bytes([0xFF, 0xFF]) + bytes(body) + bytes([(~sum(body)) & 0xFF])
    termios.tcflush(fd, termios.TCIFLUSH)
    os.write(fd, pkt)
    buf = b""; end = time.monotonic() + wait
    while len(buf) < want and time.monotonic() < end:
        r, _, _ = select.select([fd], [], [], 0.005)
        if r: buf += os.read(fd, 64)
    return buf

fd = open_port(sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0")
for sid in (1, 1):
    r = exchange(fd, [sid, 2, 1], 6)
    print("ping id", sid, "->", r.hex(" "))
t0 = time.monotonic(); n = 0; ok = 0
while time.monotonic() - t0 < 2.0:
    r = exchange(fd, [1, 4, 2, 56, 8], 14); n += 1
    if len(r) == 14 and r[0] == 255 and (~sum(r[2:-1])) & 255 == r[-1]: ok += 1; last = r
print(f"{n} state reads in 2 s ({n/2:.0f} a second), {ok} good")
if ok: print("position", last[5]*256+last[6], " voltage %.1f V" % (last[11]/10), " temperature", last[12], "C  error byte", last[4])
