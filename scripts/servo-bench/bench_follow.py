#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Bench demo ON THE BOARD: one or two servos follow the IMU's tilt, through the firmware's bench mode.

    python3 bench_follow.py [seconds] [joint index] [second joint index]

Tilting the IMU about one axis turns the first joint, about the other axis the second (if there is one), up to
+-45 degrees. Every
target goes through the microcontroller firmware (duck_bench_step / duck_bench2_step): its joint limits, speed
limit and faults apply, and when either servo stops for any reason both go limp. Needs a bench build of the
firmware (DUCK_ENABLE_MOTION 1, DUCK_BENCH_MOTION 1), the joints calibrated in config/robot.json, the IMU
running, and router_call.py beside this file. Run it only with the horns free and someone watching. The log goes
to /tmp/bench_follow.csv.
"""
import math
import sys
import time

import router_call

ARM_KEY = 0xD0C5A4B3
MISSING = -32768
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
JOINTS = [int(a) for a in sys.argv[2:4]] or [3]
CSV = "/tmp/bench_follow.csv"
MAX_DEG, SLEW_DPS, PERIOD = 45.0, 90.0, 0.02
FAULTS = ["none", "overload", "over-temperature", "under-voltage", "servo status", "servo not answering", "fall",
          "bus write", "position jump"]
RESULTS = {0: "ok", 1: "firmware built without motion", 3: "joint not calibrated", 4: "servo missing",
           6: "fault latched", 11: "busy"}


def call(method, *args):
    reply = router_call.call(method, *args)
    if reply[2] is not None:
        raise SystemExit(f"{method} refused: {reply[2]}")
    return reply[3]


def tilt():
    v = call("duck_imu_raw")
    if v[0] < 0 or v[0] > 200:
        return None
    x, y, z = v[1] / 1000.0, v[2] / 1000.0, v[3] / 1000.0
    return math.degrees(math.atan2(x, z)), math.degrees(math.atan2(y, z))


def describe(words):
    return f"mode {words[1]}, fault {FAULTS[words[2]] if words[2] < len(FAULTS) else words[2]}"


steps = ("duck_bench_step", "duck_bench2_step")[:len(JOINTS)]
why = "finished"
rows = []
try:
    for method, joint in zip(("duck_bench_arm", "duck_bench2_arm"), JOINTS):
        r = call(method, ARM_KEY, joint)
        if r != 0:
            raise SystemExit(f"{method}({joint}) refused: {RESULTS.get(r, r)}")
    state = [call(m, MISSING) for m in steps]
    print("armed. start: " + ", ".join(f"joint {s[3]} at {s[5] / 10:.1f} deg, {s[8] / 10:.1f} V" for s in state))
    target = [s[5] / 10.0 for s in state]
    t0 = time.monotonic()
    nxt = t0
    while time.monotonic() - t0 < SECONDS:
        tl = tilt()
        want = list(target) if tl is None else [max(-MAX_DEG, min(MAX_DEG, a)) for a in tl[:len(JOINTS)]]
        for i in range(len(JOINTS)):  # a gentle slew toward what the tilt asks for
            step = max(-SLEW_DPS * PERIOD, min(SLEW_DPS * PERIOD, want[i] - target[i]))
            target[i] += step
        state = [call(m, int(round(t * 10))) for m, t in zip(steps, target)]
        now = time.monotonic() - t0
        rows.append([now, tl[0] if tl else float("nan"), tl[1] if tl else float("nan")]
                    + [v for t, s in zip(target, state) for v in (t, s[5] / 10, s[8] / 10, s[7] / 10, s[9])])
        if any(s[1] != 1 for s in state):
            why = "stopped by the firmware: " + "; ".join(describe(s) for s in state)
            break
        nxt += PERIOD
        time.sleep(max(0.0, nxt - time.monotonic()))
finally:
    call("duck_bench_disarm")
    print("torque off;", why)

with open(CSV, "w") as f:
    f.write("t_s,tilt_x_deg,tilt_y_deg" + "".join(
        f",target{i}_deg,pos{i}_deg,volt{i},load{i}_pct,temp{i}_c" for i in range(1, len(JOINTS) + 1)) + "\n")
    for r in rows:
        f.write(",".join(f"{v:.2f}" for v in r) + "\n")
if rows:
    print(f"{len(rows)} ticks in {rows[-1][0]:.1f} s ({len(rows) / max(rows[-1][0], 1e-9):.0f} a second); log: {CSV}")
    last = -1
    for r in rows:
        if int(r[0] / 2) != last:
            last = int(r[0] / 2)
            per = "  ".join(f"target {r[3 + 5 * i]:6.1f} position {r[4 + 5 * i]:6.1f} {r[5 + 5 * i]:.1f} V"
                            for i in range(len(JOINTS)))
            print(f"{r[0]:5.1f} s  tilt {r[1]:6.1f} {r[2]:6.1f}  {per}")
    for i in range(len(JOINTS)):
        col = lambda k: [r[k + 5 * i] for r in rows]
        print(f"joint {JOINTS[i]}: position {min(col(4)):.0f} .. {max(col(4)):.0f} deg, lowest supply {min(col(5)):.1f} V, "
              f"peak load {max(abs(v) for v in col(6)):.0f} %, warmest {max(col(7)):.0f} C")
