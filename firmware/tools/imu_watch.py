#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Watch the IMU's raw readings ON THE BOARD while it is moved by hand (bench checklist, step 3).

    adb push tools/router_call.py tools/imu_watch.py /home/arduino/
    adb shell python3 /home/arduino/imu_watch.py --seconds 20

Polls the firmware's duck_imu_raw as fast as the link allows (about 100 times a second) and prints, for every half
second: which way is up in the sensor's own axes, the mean turning rate about each axis, and how far the sensor
has turned about each axis since the start (the gyro summed up). That is what the mounting matrix in
config/robot.json is worked out from, and how the gyro's sign is checked: a turn anticlockwise seen from the tip of
an axis must add to that axis's angle.

Needs router_call.py beside it. Nothing here writes to the robot.
"""
import argparse
import time

import router_call


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--every", type=float, default=0.5, help="seconds per printed line")
    a = ap.parse_args()

    start = time.monotonic()
    last_t = None
    last_gyro = None
    turned = [0.0, 0.0, 0.0]
    line_start, n, up_sum, gyro_sum, total, stale = start, 0, [0.0] * 3, [0.0] * 3, 0, 0
    print("  time |  up x     y     z  | turning, deg/s   x      y      z | turned since start, deg  x     y     z")
    while True:
        now = time.monotonic()
        if now - start >= a.seconds:
            break
        reply = router_call.call("duck_imu_raw")
        if reply[2] is not None:
            raise SystemExit(f"duck_imu_raw refused: {reply[2]}")
        v = reply[3]
        t = time.monotonic()
        up = [x / 1000.0 for x in v[1:4]]
        gyro = [x / 10.0 for x in v[5:8]]
        total += 1
        if v[0] < 0 or v[0] > 100 or v[4] < 0 or v[4] > 100:
            stale += 1  # the sensor has not reported lately: do not count this reading
            last_t = None
            continue
        if last_t is not None:
            for i in range(3):
                turned[i] += 0.5 * (gyro[i] + last_gyro[i]) * (t - last_t)
        last_t, last_gyro = t, gyro
        n += 1
        for i in range(3):
            up_sum[i] += up[i]
            gyro_sum[i] += gyro[i]
        if t - line_start >= a.every and n:
            print("%6.1f | %5.2f %5.2f %5.2f |               %6.1f %6.1f %6.1f |                     %6.0f %5.0f %5.0f"
                  % (t - start, *(s / n for s in up_sum), *(s / n for s in gyro_sum), *turned), flush=True)
            line_start, n, up_sum, gyro_sum = t, 0, [0.0] * 3, [0.0] * 3
    rate = total / max(time.monotonic() - start, 1e-9)
    print(f"{total} readings, {rate:.0f} a second, {stale} without a fresh report")
    print("turned since start, deg: x %.0f  y %.0f  z %.0f" % tuple(turned))


if __name__ == "__main__":
    main()
