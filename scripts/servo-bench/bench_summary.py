#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Read `duckd bench --record` recordings and print what the servo did in each.

    uv run --with mcap --with mcap-ros2-support --with numpy scripts/servo-bench/bench_summary.py FILE.mcap...

For every recording: how often the servo was read, the range it moved over, its top speed, and the servo's own
speed report against the change of its position between reads (bench checklist 8b: near 1 means the firmware
reads the speed register in the right unit). For a recording of steps also: how long after a jump of the target
the servo had covered 10 % and 90 % of it, how far it overshot and where it came to rest.

The times are from the moment `duckd` sent the new target, so they include the link to the microcontroller, the
firmware's next tick and its speed limit on the goal, not the servo alone. Read stamps have 1 ms resolution, which
alone puts some 10 % of scatter on a speed taken from two reads 10 ms apart.
"""
import sys

import numpy as np
from mcap_ros2.reader import read_ros2_messages


def load(path):
    """Topic to rows of (seconds, position rad, velocity rad/s or NaN) for the one joint a bench run records."""
    out = {"/joint_states": [], "/joint_commands": [], "/joint_targets": []}
    for m in read_ros2_messages(path):
        if m.channel.topic in out:
            r = m.ros_msg
            out[m.channel.topic].append((m.log_time_ns / 1e9, r.position[0], r.velocity[0] if len(r.velocity) else np.nan))
    return {topic: np.array(rows) for topic, rows in out.items()}


def summary(path):
    d = load(path)
    states, targets = d["/joint_states"], d["/joint_targets"]
    t0 = states[0, 0]
    t, q, reported = states[:, 0] - t0, np.degrees(states[:, 1]), np.degrees(states[:, 2])
    # speed from positions two reads apart, against the servo's report at the read between them
    derived = (q[2:] - q[:-2]) / (t[2:] - t[:-2])
    moving = np.abs(derived) > 40
    ratio = np.sum(reported[1:-1][moving] * derived[moving]) / np.sum(derived[moving] ** 2) if moving.sum() > 20 else np.nan
    print(f"{path}")
    print(f"  {len(t)} reads in {t[-1]:.1f} s (median {np.median(np.diff(t)) * 1e3:.1f} ms apart, longest "
          f"{np.diff(t).max() * 1e3:.0f} ms); {q.min():.1f} to {q.max():.1f} deg")
    print(f"  top speed {np.abs(derived).max():.0f} deg/s from positions, {np.nanmax(np.abs(reported)):.0f} deg/s reported; "
          f"reported / from positions = {ratio:.2f} over {moving.sum()} reads")

    tt, target = targets[:, 0] - t0, np.degrees(targets[:, 1])
    jumps = [i for i in range(1, len(target)) if abs(target[i] - target[i - 1]) > 0.9 * np.ptp(target)]
    rows = []
    for i in jumps:
        at, a, b = tt[i], target[i - 1], target[i]
        w = (t >= at) & (t < at + 0.48)
        held = target[(tt >= at) & (tt < at + 0.48)]
        if w.sum() < 10 or np.ptp(held) > 0.05:  # the target moved again (the fade at either end of a run)
            continue
        since, done = t[w] - at, (q[w] - a) / (b - a)
        reach = lambda level: since[np.argmax(done >= level)] if (done >= level).any() else np.nan  # noqa: E731
        rows.append((abs(b - a), reach(0.1), reach(0.9), (done.max() - 1) * abs(b - a), abs(q[w][-1] - b)))
    if rows:
        r = np.array(rows)
        print(f"  {len(r)} jumps of {r[:, 0].mean():.0f} deg: 10 % after {np.median(r[:, 1]) * 1e3:.0f} ms, 90 % after "
              f"{np.median(r[:, 2]) * 1e3:.0f} ms, past the target by {np.median(r[:, 3]):.2f} deg at most, "
              f"{np.median(r[:, 4]):.2f} deg from it after 0.48 s (worst {r[:, 4].max():.2f})")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for path in sys.argv[1:]:
        summary(path)
