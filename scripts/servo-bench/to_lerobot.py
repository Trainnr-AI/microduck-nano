#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Write a `duckd bench --record` recording as a LeRobot dataset (format v3), for trainnr's servo fit.

    uv run --with mcap --with mcap-ros2-support --with numpy --with pyarrow \
        scripts/servo-bench/to_lerobot.py FILE.mcap OUT_FOLDER

Why: trainnr's `servo-bench` fit wants the commanded position as the channel `joint.command`. Its MCAP reader
gives `/joint_commands` another name (Trainnr-AI/trainnr#147); its LeRobot reader maps a dataset's `action` to
exactly that channel and `observation.state` to `joint.position`. So the same recording, written in that format,
is one the fit accepts:

    ingest_recording("<OUT_FOLDER>")   then   identify_system(robot="<bench>", recording=..., method="servo-bench")

What goes in: one row per read of the servo. `observation.state` is the position read; `action` is the goal the
firmware had written at that read (after its clamp and speed limit: what the servo was told), both in radians;
`timestamp` is the read's own time, seconds from the first. The reads are not evenly spaced (about 88 a second);
`fps` in the dataset's header is their median rate, and trainnr goes by the timestamps.
"""
import json
import pathlib
import sys

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from mcap_ros2.reader import read_ros2_messages


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    source, out = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
    if out.exists() and any(out.iterdir()):
        sys.exit(f"{out} is not empty")
    states, commands, joint = [], {}, None
    for m in read_ros2_messages(str(source)):
        if m.channel.topic == "/joint_states":
            joint = joint or m.ros_msg.name[0]
            states.append((m.log_time_ns, m.ros_msg.position[0]))
        elif m.channel.topic == "/joint_commands":
            commands[m.log_time_ns] = m.ros_msg.position[0]
    rows = [(t, q, commands[t]) for t, q in states if t in commands and np.isfinite(q) and np.isfinite(commands[t])]
    if len(rows) < 10:
        sys.exit(f"{source}: {len(rows)} reads with a goal beside them; is it a `duckd bench` recording?")
    t = np.array([r[0] for r in rows], dtype=np.float64) / 1e9
    t -= t[0]
    n, fps = len(rows), int(round(1.0 / float(np.median(np.diff(t)))))

    (out / "meta").mkdir(parents=True, exist_ok=True)
    (out / "data" / "chunk-000").mkdir(parents=True, exist_ok=True)
    vector = {"dtype": "float32", "shape": [1], "names": [joint]}
    scalar = lambda dtype: {"dtype": dtype, "shape": [1], "names": None}  # noqa: E731
    info = {
        "codebase_version": "v3.0",
        "robot_type": "one servo on a bench (duckd bench)",
        "fps": fps,
        "total_episodes": 1,
        "total_frames": n,
        "total_tasks": 1,
        "features": {
            "observation.state": vector,
            "action": vector,
            "timestamp": scalar("float64"),
            "episode_index": scalar("int64"),
            "frame_index": scalar("int64"),
            "index": scalar("int64"),
            "task_index": scalar("int64"),
        },
        "source": {"file": source.name, "action": "/joint_commands: the goal the firmware wrote, at each read",
                   "observation.state": "/joint_states: the servo's position read", "unit": "rad",
                   "written_by": "scripts/servo-bench/to_lerobot.py"},
    }
    (out / "meta" / "info.json").write_text(json.dumps(info, indent=1) + "\n")
    count = np.arange(n, dtype=np.int64)
    table = pa.table({
        "observation.state": pa.array([[float(r[1])] for r in rows], type=pa.list_(pa.float32())),
        "action": pa.array([[float(r[2])] for r in rows], type=pa.list_(pa.float32())),
        "timestamp": pa.array(t, type=pa.float64()),
        "episode_index": pa.array(np.zeros(n, dtype=np.int64)),
        "frame_index": pa.array(count),
        "index": pa.array(count),
        "task_index": pa.array(np.zeros(n, dtype=np.int64)),
    })
    pq.write_table(table, out / "data" / "chunk-000" / "file-000.parquet")
    print(f"wrote {out}: {n} reads of `{joint}` over {t[-1]:.1f} s (median {fps} a second)")


if __name__ == "__main__":
    main()
