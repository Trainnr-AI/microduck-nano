#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Refresh the design-derived numbers in config/robot.json from the design repo.

Only these keys are touched: each joint's min_deg / max_deg, poses.SIT, and
hip_map. Everything else in robot.json (servo IDs, zero steps, margins,
thresholds) is left as it is.

    python3 tools/import_design.py --design <path to agent-cad3d>/projects/xgoduck_scs0009

Then run tools/gen_config.py to regenerate the C++ header.
"""
import argparse
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROBOT_JSON = HERE.parent / "config" / "robot.json"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--design", required=True, type=Path, help="projects/xgoduck_scs0009 in the design repo")
    a = ap.parse_args()

    ranges = json.loads((a.design / "sim" / "joint_ranges_v2.json").read_text())
    hip = json.loads((a.design / "data" / "hip_collision_map.json").read_text())
    robot = json.loads(ROBOT_JSON.read_text())

    for j in robot["joints"]:
        if j["name"] in ranges["joints"]:
            j["min_deg"], j["max_deg"] = ranges["joints"][j["name"]]
    sit = ranges["keyframes"]["SIT"]
    robot["poses"]["SIT"] = {j["name"]: sit[j["name"]] for j in robot["joints"] if j["name"] in sit}

    g = hip["grid"]
    table = {}
    for p in hip["poses"]:
        table[(p["hip_pitch"], p["hip_yaw"], p["knee"], p["hip_roll"])] = bool(p["collides"])
    rows = []
    for pitch in g["hip_pitch"]:
        for yaw in g["hip_yaw"]:
            for knee in g["knee"]:
                bits = "".join("1" if table[(pitch, yaw, knee, r)] else "0" for r in g["hip_roll"])
                rows.append({"hip_pitch": pitch, "hip_yaw": yaw, "knee": knee, "roll_collides": bits})
    # Poses an independent review found colliding that the design map marks free (see _review_note in robot.json).
    by_key = {(r["hip_pitch"], r["hip_yaw"], r["knee"]): r for r in rows}
    for e in robot["hip_map"].get("review_extra_collisions", []):
        r = by_key[(e["hip_pitch"], e["hip_yaw"], e["knee"])]
        i = g["hip_roll"].index(e["hip_roll"])
        r["roll_collides"] = r["roll_collides"][:i] + "1" + r["roll_collides"][i + 1:]
    robot["hip_map"]["grid"] = {k: g[k] for k in ("hip_pitch", "hip_roll", "hip_yaw", "knee")}
    robot["hip_map"]["rows"] = rows
    robot["hip_map"]["source"] = hip["what"]

    ROBOT_JSON.write_text(json.dumps(robot, indent=1) + "\n")
    print(f"updated {ROBOT_JSON}: {len(robot['joints'])} joints, {len(rows)} hip map rows, "
          f"{sum(r['roll_collides'].count('1') for r in rows)} colliding poses")


if __name__ == "__main__":
    main()
