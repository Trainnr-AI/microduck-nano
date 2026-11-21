#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""ON THE BOARD: swing one servo slowly a few times through the firmware's bench mode, to see which one it is.

    python3 bench_swing.py <joint index> [low deg, default -25] [high deg, default 25] [swings, default 3]

The joint goes from wherever it is to `high`, then between `low` and `high`, and stops at 0 degrees (mid-range when
the joint's zero is 512, which is where a servo should be when its horn is fitted) or at the nearest of `low` /
`high` when 0 is outside them. 60 degrees a second. Every target goes through the microcontroller firmware
(duck_bench_step): the joint's limits, the speed limit and the faults apply, and torque goes off at the end and on
any fault. A servo that starts outside its joint's limits is first brought inside them.

Needs a bench build of the firmware (DUCK_ENABLE_MOTION 1, DUCK_BENCH_MOTION 1), the joint given a servo ID and
marked calibrated in config/robot.json, and router_call.py beside this file. Run it only with the horn free and
someone watching.
"""
import sys
import time

import router_call

ARM_KEY = 0xD0C5A4B3
MISSING = -32768
PERIOD, DPS = 0.02, 60.0
FAULTS = ["none", "overload", "over-temperature", "under-voltage", "servo status", "servo not answering", "fall",
          "bus write", "position jump"]
RESULTS = {0: "ok", 1: "firmware built without motion", 3: "joint not calibrated", 4: "servo missing",
           6: "fault latched", 11: "busy"}

if len(sys.argv) < 2:
    sys.exit(__doc__)
JOINT = int(sys.argv[1])
LOW = float(sys.argv[2]) if len(sys.argv) > 2 else -25.0
HIGH = float(sys.argv[3]) if len(sys.argv) > 3 else 25.0
SWINGS = int(sys.argv[4]) if len(sys.argv) > 4 else 3
if not LOW < HIGH:
    sys.exit("low must be below high")
REST = min(max(0.0, LOW), HIGH)

resent = 0  # calls that came back refused or timed out and were sent again


def call(method, *args):
    """One call to the firmware, sent again up to five times: see bench_follow.py for why and for the timeout."""
    global resent
    problem = None
    for _ in range(5):
        try:
            reply = router_call.call(method, *args, timeout=0.08)
        except OSError as e:
            problem = repr(e)
        else:
            if reply[2] is None:
                return reply[3]
            problem = str(reply[2])
        resent += 1
    raise SystemExit(f"{method} failed five times in a row: {problem}")


why = "never armed"
seen, volts, loads = [], [], []
try:
    r = call("duck_bench_arm", ARM_KEY, JOINT)
    if r != 0:
        raise SystemExit(f"arming joint {JOINT} refused: {RESULTS.get(r, r)}")
    state = call("duck_bench_step", MISSING)
    target = state[5] / 10.0
    print(f"armed joint {state[3]} at {target:.1f} deg, {state[8] / 10:.1f} V")
    nxt = time.monotonic()
    stopped = False
    for goal in [HIGH, LOW] * SWINGS + [REST]:
        while not stopped and abs(goal - target) > 1e-6:
            target += max(-DPS * PERIOD, min(DPS * PERIOD, goal - target))
            state = call("duck_bench_step", int(round(target * 10)))
            seen.append(state[5] / 10)
            volts.append(state[8] / 10)
            loads.append(abs(state[7] / 10))
            if state[1] != 1:
                fault = FAULTS[state[2]] if state[2] < len(FAULTS) else state[2]
                why = f"stopped by the firmware: mode {state[1]}, fault {fault}"
                stopped = True
            nxt += PERIOD
            time.sleep(max(0.0, nxt - time.monotonic()))
    if not stopped:
        why = "finished"
finally:
    call("duck_bench_disarm")
    print("torque off;", why)
if seen:
    print(f"position {min(seen):.0f} .. {max(seen):.0f} deg, lowest supply {min(volts):.1f} V, "
          f"peak load {max(loads):.0f} %, {resent} call(s) sent again")
