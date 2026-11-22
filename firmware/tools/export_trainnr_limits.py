#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Write the firmware's limits over several joints as trainnr's `limits.json`.

    python3 firmware/tools/export_trainnr_limits.py --out <folder of the robot's model>/limits.json
    python3 firmware/tools/export_trainnr_limits.py --check-poses poses.txt     (or - for standard input)

trainnr reads `limits.json` beside a robot's model when the robot is onboarded (schema `trainnr-joint-limits/1`,
trainnr's docs/88-your-own-robot.md): a walk trained on that robot then ends an episode on a pose this firmware
would refuse, its evaluation counts them and its gate fails on them. Without the file a policy is free to learn
poses the robot will not take.

What is written, from `config/robot.json`: the head's roll and yaw interlock and its roll stop (two rules), knee
and ankle of each leg (two tables), each leg's hip map (two grids) and the both-legs rule with its two rolled-in
bands (three rules). The right leg is written in the left leg's signs, as the firmware reads it. Single-joint
ranges are not in the file: they are the model's own.

`--check-poses` reads poses with the firmware's verdict on each (`duck_tests --pose-verdicts`, see
firmware/tests/test_main.cpp) and compares them with this file's limits read the way trainnr's document says they
are read. `make -C firmware/tests trainnr-limits` runs it; it is part of `make test`.
"""
import argparse
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
LEG_PARTS = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle")
GRID_AXES = ("hip_pitch", "hip_yaw", "knee", "ankle", "hip_roll")  # the order of a forbidden cell's indices
GRID_TOLERANCE_DEG = 1e-3  # the firmware's bracket(); trainnr uses the same


def limits_from(robot):
    """trainnr's limits, as a dict ready for json.dump, from the firmware's robot.json."""
    limits = []

    h = robot["head_interlock"]
    window = max(0.0, h["yaw_window_deg"] - h["yaw_margin_deg"])
    roll_stop = max(h["roll_free_deg"], h["roll_max_deg"] - h["roll_margin_deg"])
    limits.append({"name": "head roll and yaw interlock", "form": "rule",
                   "when": [{"joint": "head_yaw", "outside": [-window, window]},
                            {"joint": "head_roll", "outside": [-h["roll_free_deg"], h["roll_free_deg"]]}],
                   "note": "roll only while the head's yaw is centred, and yaw only while its roll is"})
    limits.append({"name": "head roll stop", "form": "rule",
                   "when": [{"joint": "head_roll", "outside": [-roll_stop, roll_stop]}],
                   "note": "the roll's limit less the firmware's margin"})

    ka = robot["knee_ankle"]
    for side in ("left", "right"):
        limit = {"name": f"{side} knee and ankle", "form": "table",
                 "joint": f"{side}_ankle", "over": f"{side}_knee", "from": ka["knee_over_deg"],
                 "minimum_by": [[k, a] for k, a in ka["ankle_min_by_knee_deg"]],
                 "note": "past the knee angle, the ankle's minimum is that of the first row at or above the knee: "
                         "below it the foot meets the hip bracket or the thigh"}
        if side == "right":
            limit["signs"] = {"right_knee": -1, "right_ankle": -1}
        limits.append(limit)

    grid = robot["hip_map"]["grid"]
    index = {axis: {v: i for i, v in enumerate(grid[axis])} for axis in ("hip_pitch", "hip_yaw", "knee")}
    for side in ("left", "right"):
        forbidden = []
        for row in robot["hip_map"]["rows"]:
            first = [index[axis][row[axis]] for axis in ("hip_pitch", "hip_yaw", "knee")]
            for ankle_i, layer in enumerate(row[side].split(" ")):
                forbidden += [first + [ankle_i, roll_i] for roll_i, c in enumerate(layer) if c == "1"]
        limit = {"name": f"{side} hip map", "form": "grid",
                 "axes": [f"{side}_{axis}" for axis in GRID_AXES],
                 "values": {f"{side}_{axis}": grid[axis] for axis in GRID_AXES},
                 "forbidden": sorted(forbidden),
                 "note": "poses where the leg's parts cross or pass closer than 0.3 mm; between grid values the "
                         "stricter neighbour holds"}
        if side == "right":
            limit["signs"] = {f"right_{part}": -1 for part in LEG_PARTS}
        limits.append(limit)

    b = robot["both_legs"]

    def both(pitch_forward_of):
        return [{"joint": "left_hip_yaw", "above": b["yaw_out_over_deg"]},
                {"joint": "right_hip_yaw", "above": b["yaw_out_over_deg"], "sign": -1},
                {"joint": "left_hip_pitch", "below": pitch_forward_of},
                {"joint": "right_hip_pitch", "below": pitch_forward_of, "sign": -1}]

    limits.append({"name": "both legs forward, toed out", "form": "rule", "when": both(b["pitch_forward_of_deg"]),
                   "note": "the feet meet"})
    for side, sign in (("left", 1), ("right", -1)):
        limits.append({"name": f"both legs forward, toed out, {side} hip rolled in", "form": "rule",
                       "when": both(b["band_pitch_forward_of_deg"])
                       + [{"joint": f"{side}_hip_roll", "below": b["band_roll_in_past_deg"], "sign": sign}],
                       "note": "the band the design's sweep of both legs added to the rule"})

    return {"schema": "trainnr-joint-limits/1",
            "source": "Microduck Nano firmware, firmware/config/robot.json (firmware/tools/export_trainnr_limits.py)",
            "limits": limits}


def dumps(document):
    """One limit per line: the two grids are 1766 cells, unreadable either way, and this keeps a diff small."""
    head = {k: v for k, v in document.items() if k != "limits"}
    lines = [json.dumps(head)[:-1] + ', "limits": [']
    lines += [json.dumps(limit, allow_nan=False) + ("," if i + 1 < len(document["limits"]) else "")
              for i, limit in enumerate(document["limits"])]
    return "\n".join(lines) + "\n]}\n"


# ---- reading the limits the way trainnr's document says, for the check against the firmware -----------------


def condition_holds(cond, value):
    v = value * cond.get("sign", 1)
    if "above" in cond:
        return v > cond["above"]
    if "below" in cond:
        return v < cond["below"]
    if "outside" in cond:
        return v < cond["outside"][0] or v > cond["outside"][1]
    if "within" in cond:
        return cond["within"][0] <= v <= cond["within"][1]
    raise ValueError(f"condition without a test: {cond}")


def bracket(values, v):
    """Grid indices around v: one when v is on a grid value or beyond an end, else the two neighbours."""
    if v <= values[0] + GRID_TOLERANCE_DEG:
        return [0]
    if v >= values[-1] - GRID_TOLERANCE_DEG:
        return [len(values) - 1]
    for i, g in enumerate(values):
        if abs(v - g) <= GRID_TOLERANCE_DEG:
            return [i]
        if v < g:
            return [i - 1, i]
    return [len(values) - 1]


def forbidden_by(limit, pose, cells=None):
    """True when `pose` (joint name to degrees) breaks `limit`. `cells`: the grid's forbidden cells as a set."""
    signs = limit.get("signs", {})

    def angle(joint):
        return pose[joint] * signs.get(joint, 1)

    if limit["form"] == "rule":
        return all(condition_holds(c, angle(c["joint"])) for c in limit["when"])
    if limit["form"] == "table":
        over, value = angle(limit["over"]), angle(limit["joint"])
        if not over > limit["from"]:
            return False
        broken = False
        for key, rows in (("minimum_by", limit.get("minimum_by")), ("maximum_by", limit.get("maximum_by"))):
            if rows:
                bound = next((b for at, b in rows if over <= at), rows[-1][1])
                broken |= value < bound if key == "minimum_by" else value > bound
        return broken
    if limit["form"] == "grid":
        around = [bracket(limit["values"][axis], angle(axis)) for axis in limit["axes"]]
        cell = [0] * len(around)

        def any_forbidden(depth):
            if depth == len(around):
                return tuple(cell) in cells
            for i in around[depth]:
                cell[depth] = i
                if any_forbidden(depth + 1):
                    return True
            return False

        return any_forbidden(0)
    raise ValueError(f"unknown form {limit['form']}")


def check_against_firmware(document, robot, lines):
    """Compare with `duck_tests --pose-verdicts`. Returns (poses, disagreements, [allowed, refused] per rule)."""
    names = [j["name"] for j in robot["joints"]][:14]
    by_name = {limit["name"]: limit for limit in document["limits"]}
    cells = {name: {tuple(c) for c in limit["forbidden"]} for name, limit in by_name.items() if limit["form"] == "grid"}
    groups = [  # the firmware's bit, then the limits that together are that rule
        (1, ["left knee and ankle"]), (2, ["left hip map"]), (4, ["right knee and ankle"]), (8, ["right hip map"]),
        (16, ["both legs forward, toed out", "both legs forward, toed out, left hip rolled in",
              "both legs forward, toed out, right hip rolled in"]),
        (32, ["head roll and yaw interlock", "head roll stop"]),
    ]
    if sorted(n for _, group in groups for n in group) != sorted(by_name):
        sys.exit("the check does not cover every limit written: update its groups")
    poses, wrong, seen = 0, [], {bit: [0, 0] for bit, _ in groups}
    for line in lines:
        if line.startswith("#") or not line.strip():
            continue
        words = [int(w) for w in line.split()]
        if len(words) != 15:
            sys.exit(f"a pose line needs 14 angles and the verdict: {line!r}")
        pose = {name: tenths / 10 for name, tenths in zip(names, words)}
        poses += 1
        for bit, group in groups:
            firmware_allows = bool(words[14] & bit)
            here_allows = not any(forbidden_by(by_name[n], pose, cells.get(n)) for n in group)
            seen[bit][0 if firmware_allows else 1] += 1
            if firmware_allows != here_allows:
                wrong.append((group[0], firmware_allows, pose))
    return poses, wrong, {groups[i][1][0]: seen[bit] for i, (bit, _) in enumerate(groups)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--robot", default=str(ROOT / "config" / "robot.json"), help="the firmware's limits file")
    ap.add_argument("--out", help="where to write limits.json")
    ap.add_argument("--check-poses", help="poses with the firmware's verdicts (duck_tests --pose-verdicts), or -")
    args = ap.parse_args()
    if not args.out and not args.check_poses:
        ap.error("give --out, --check-poses, or both")
    robot = json.loads(pathlib.Path(args.robot).read_text())
    document = limits_from(robot)
    if args.check_poses:
        lines = sys.stdin if args.check_poses == "-" else pathlib.Path(args.check_poses).read_text().splitlines()
        poses, wrong, seen = check_against_firmware(document, robot, lines)
        for name, (allowed, refused) in seen.items():
            print(f"  {name:<28} firmware allows {allowed:>6}, refuses {refused:>6}")
            if not allowed or not refused:
                sys.exit(f"{name}: the poses never reach one side of this rule, so it was not checked")
        if wrong or not poses:
            for name, firmware_allows, pose in wrong[:10]:
                print(f"  DISAGREE on {name}: firmware {'allows' if firmware_allows else 'refuses'} {pose}")
            sys.exit(f"{len(wrong)} disagreement(s) with the firmware on {poses} poses")
        print(f"limits.json agrees with the firmware on {poses} poses, rule by rule")
    if args.out:
        pathlib.Path(args.out).write_text(dumps(document))
        grids = {limit["name"]: len(limit["forbidden"]) for limit in document["limits"] if limit["form"] == "grid"}
        print(f"wrote {args.out}: {len(document['limits'])} limits, forbidden cells {grids}")


if __name__ == "__main__":
    main()
