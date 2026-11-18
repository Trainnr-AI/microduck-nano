#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Refresh the design-derived numbers in config/robot.json from the design repo.

    python3 tools/import_design.py --design <path to agent-cad3d>/projects/xgoduck_scs0009

Reads the design's sim/joint_ranges_v2.json (servo limits, SIT, interlocks) and
data/hip_collision_map.json (every measured pose), and writes into robot.json:

    joints[].min_deg / max_deg   the servo angle limits
    poses.SIT
    head_interlock               yaw window and free roll (the firmware's own margins are kept)
    hip_map                      one map per leg, with the ankle as a fifth axis
    knee_ankle                   the knee x ankle rule
    both_legs                    the both-legs rule

Everything else in robot.json (servo IDs, zero steps, margins, thresholds) is left as it is.

The rules the design writes in words are checked against its own pose data, and the import
stops if a rule lets a measured collision through. Where the data is stricter than the
written rule, the data wins and the difference is printed.

Then run tools/gen_config.py to regenerate the C++ header.
"""
import argparse
import fnmatch
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROBOT_JSON = HERE.parent / "config" / "robot.json"

HIP_AXES = ("hip_pitch", "hip_roll", "hip_yaw", "knee")
ANKLE_LAYERS = (-23.0, 0.0, 23.0)   # the ankle angles the hip grid was measured at (left signs)
NO_BAND_ROLL = -999.0               # both_legs.band_roll_in_past_deg when the data adds nothing to the rule


def fail(msg):
    sys.exit("import_design: " + msg)


def pair_of(contact):
    return contact.split(":")[0].strip()


def contact_is_bad(contact, margin, ignore_patterns, ignore_pairs=()):
    """A listed contact counts against a pose when the two surfaces cross at all (even by 0.00 mm) or pass closer
    than the margin, unless the pair is one the design leaves out of the margin, or one judged by another rule."""
    pair, what = pair_of(contact), contact.split(":")[1]
    if pair in ignore_pairs:
        return False
    if "depth" in what:
        return True
    gap = float(what.split("gap")[1].split("mm")[0])
    if any(fnmatch.fnmatch(pair, "*" + pat) for pat in ignore_patterns):
        return False
    return gap < margin


def pose_is_bad(p, margin, ignore_patterns, ignore_pairs=()):
    if not ignore_pairs and (p.get("collides") or p.get("under_margin")):
        return True
    return any(contact_is_bad(c, margin, ignore_patterns, ignore_pairs) for c in p["contacts"])


def bracket(grid, v):
    """Grid values around v: one if v is on the grid or beyond its ends, else the two neighbours."""
    if v <= grid[0]:
        return [grid[0]]
    if v >= grid[-1]:
        return [grid[-1]]
    for i, x in enumerate(grid):
        if abs(v - x) < 1e-6:
            return [x]
        if v < x:
            return [grid[i - 1], x]
    raise AssertionError


def next_above(grid, v, what):
    for x in grid:
        if x > v + 1e-6:
            return x
    fail(f"{what}: {v} is at the end of the grid; the rule cannot be extended from the data")


def knee_ankle_allows(rule, knee, ankle):
    """Left signs. The firmware's knee_ankle_allowed() is the same rule."""
    if knee <= rule["knee_over_deg"]:
        return True
    for k, a in rule["ankle_min_by_knee_deg"]:
        if knee <= k:
            return ankle >= a
    return ankle >= rule["ankle_min_by_knee_deg"][-1][1]


def both_legs_forbids(rule, left, right):
    """left, right: dicts of hip angles in LEFT signs. The firmware's both_legs_allowed() is the same rule."""
    if not (left["hip_yaw"] > rule["yaw_out_over_deg"] and right["hip_yaw"] > rule["yaw_out_over_deg"]):
        return False
    if left["hip_pitch"] < rule["pitch_forward_of_deg"] and right["hip_pitch"] < rule["pitch_forward_of_deg"]:
        return True
    band = rule["band_pitch_forward_of_deg"]
    return (left["hip_pitch"] < band and right["hip_pitch"] < band
            and min(left["hip_roll"], right["hip_roll"]) < rule["band_roll_in_past_deg"])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--design", required=True, type=Path, help="projects/xgoduck_scs0009 in the design repo")
    a = ap.parse_args()

    ranges = json.loads((a.design / "sim" / "joint_ranges_v2.json").read_text())
    hip = json.loads((a.design / "data" / "hip_collision_map.json").read_text())
    robot = json.loads(ROBOT_JSON.read_text())
    locks = ranges.get("interlocks")
    if not locks:
        fail("sim/joint_ranges_v2.json has no 'interlocks' block: the design repo is older than 2026-11-18")
    margin = float(hip["margin_mm"])
    ignore = list(hip.get("margin_ignores", []))

    # ---- servo limits and SIT
    for j in robot["joints"]:
        if j["name"] not in ranges["joints"]:
            fail(f"the design has no limits for {j['name']}")
        j["min_deg"], j["max_deg"] = ranges["joints"][j["name"]]
    sit = ranges["keyframes"]["SIT"]
    robot["poses"]["SIT"] = {j["name"]: sit[j["name"]] for j in robot["joints"] if j["name"] in sit}
    limit = {j["name"]: (j["min_deg"], j["max_deg"]) for j in robot["joints"]}

    # ---- head interlock: the design's window; the margins are the firmware's own
    h = locks["head_roll_yaw"]
    robot["head_interlock"]["yaw_window_deg"] = float(h["abs_head_yaw_max_for_roll_deg"])
    robot["head_interlock"]["roll_free_deg"] = float(h["abs_head_roll_max_for_yaw_deg"])

    # ---- knee x ankle: the design's rule, checked against its knee x ankle sweep of each leg
    ka = locks["knee_ankle"]
    rule_ka = {"knee_over_deg": float(ka["knee_gt_deg"]),
               "ankle_min_by_knee_deg": [[float(k), float(v)] for k, v in ka["ankle_min_by_knee_deg"]]}
    rows_ka = rule_ka["ankle_min_by_knee_deg"]
    if any(rows_ka[i][0] >= rows_ka[i + 1][0] or rows_ka[i][1] > rows_ka[i + 1][1] for i in range(len(rows_ka) - 1)):
        fail("knee x ankle rows must rise in knee and not fall in ankle")
    if rows_ka[-1][0] < limit["left_knee"][1]:
        fail("knee x ankle rows stop short of the knee's limit")
    knee_ankle_pairs = set()
    for name, sign in (("knee_ankle_left", 1.0), ("knee_ankle_right", -1.0)):
        for p in hip[name]:
            knee, ankle = sign * p["knee"], sign * p["ankle"]
            knee_ankle_pairs.update(pair_of(c) for c in p["contacts"])
            inside = (limit["left_knee"][0] <= knee <= limit["left_knee"][1]
                      and limit["left_ankle"][0] <= ankle <= limit["left_ankle"][1])
            if inside and pose_is_bad(p, margin, ignore) and knee_ankle_allows(rule_ka, knee, ankle):
                fail(f"{name}: knee {p['knee']} ankle {p['ankle']} collides or is under the margin, "
                     f"but the design's knee x ankle rule allows it: {p['contacts']}")

    # ---- hip maps: one per leg, in left signs, at ankle -23 / 0 / +23
    g = {k: [float(v) for v in hip["grid"][k]] for k in HIP_AXES}
    cells = [(p, y, k, r) for p in g["hip_pitch"] for y in g["hip_yaw"] for k in g["knee"] for r in g["hip_roll"]]
    maps = {}     # leg -> ankle layer -> set of cells that are not allowed
    added = {}
    for leg, base, extra, sign in (("left", "poses", "ankle_pm23_left", 1.0),
                                   ("right", "right_poses", "ankle_pm23_right", -1.0)):
        def key(p):
            return tuple(sign * p[k] for k in ("hip_pitch", "hip_yaw", "knee", "hip_roll"))
        level = {key(p) for p in hip[base] if pose_is_bad(p, margin, ignore)}
        if {key(p) for p in hip[base]} != set(cells):
            fail(f"{base}: the poses do not cover the grid exactly")
        layers = {0.0: set(level)}
        for ankle in (-23.0, 23.0):
            layers[ankle] = set(level)
        # The foot against the same leg's bracket or thigh is the knee x ankle collision, judged by that rule on its
        # finer grid; every other contact with the ankle moved makes the hip cell not allowed at that ankle angle.
        for p in hip[extra]:
            ankle = sign * p["ankle"]
            if ankle not in layers or ankle == 0.0:
                fail(f"{extra}: unexpected ankle angle {p['ankle']}")
            if key(p) not in set(cells):
                fail(f"{extra}: pose off the grid: {p}")
            if pose_is_bad(p, margin, ignore, ignore_pairs=knee_ankle_pairs):
                layers[ankle].add(key(p))
        maps[leg] = layers
        added[leg] = {ankle: len(layers[ankle] - level) for ankle in (-23.0, 23.0)}

    # Poses an independent review found colliding (see _review_note in robot.json): kept as a second source.
    review = robot["hip_map"].get("review_extra_collisions", [])
    review_new = 0
    for e in review:
        cell = (e["hip_pitch"], e["hip_yaw"], e["knee"], e["hip_roll"])
        if cell not in set(cells):
            fail(f"review_extra_collisions: {e} is off the design's grid")
        for leg in (("left", "right") if e["leg"] == "both" else (e["leg"],)):
            for layer in maps[leg].values():
                if cell not in layer:
                    review_new += 1
                    layer.add(cell)

    rows = []
    for p in g["hip_pitch"]:
        for y in g["hip_yaw"]:
            for k in g["knee"]:
                row = {"hip_pitch": p, "hip_yaw": y, "knee": k}
                for leg in ("left", "right"):
                    row[leg] = " ".join("".join("1" if (p, y, k, r) in maps[leg][ankle] else "0" for r in g["hip_roll"])
                                        for ankle in ANKLE_LAYERS)
                rows.append(row)
    robot["hip_map"]["grid"] = dict(g, ankle=list(ANKLE_LAYERS))
    robot["hip_map"]["rows"] = rows
    robot["hip_map"].pop("source", None)
    robot["hip_map"]["source"] = hip["what"]

    def leg_blocked(leg, q):
        """Stricter neighbour over the four hip axes, ankle 0."""
        return any((p, y, k, r) in maps[leg][0.0]
                   for p in bracket(g["hip_pitch"], q["hip_pitch"]) for y in bracket(g["hip_yaw"], q["hip_yaw"])
                   for k in bracket(g["knee"], q["knee"]) for r in bracket(g["hip_roll"], q["hip_roll"]))

    # ---- both legs: the design's rule, extended where its both-legs sweep shows the feet meeting outside it
    design_rule = {"yaw_out_over_deg": 0.0, "pitch_forward_of_deg": -30.0}
    text = locks["both_legs"]
    if "left_hip_pitch < -30 and left_hip_yaw > 0" not in text["left"] or \
            "right_hip_pitch > 30 and right_hip_yaw < 0" not in text["right"]:
        fail("the design's both-legs rule changed: read it and update design_rule here: " + json.dumps(text))
    rule_bl = dict(design_rule, band_pitch_forward_of_deg=design_rule["pitch_forward_of_deg"],
                   band_roll_in_past_deg=NO_BAND_ROLL)

    def feet_meet(p):
        for c in p["contacts"]:
            x, y = pair_of(c).split(" x ")
            if {x.split("_")[0], y.split("_")[0]} == {"left", "right"} and contact_is_bad(c, margin, ignore):
                return True
        return bool(p.get("legs_collide"))

    def in_left_signs(p):
        return p["left"], {k: -v for k, v in p["right"].items()}

    outside = []
    for p in hip["both_legs_poses"]:
        left, right = in_left_signs(p)
        if feet_meet(p) and not leg_blocked("left", left) and not leg_blocked("right", right) \
                and not both_legs_forbids(rule_bl, left, right):
            outside.append((left, right, p))
    if outside:
        for left, right, p in outside:
            if not (left["hip_yaw"] > 0.0 and right["hip_yaw"] > 0.0):
                fail(f"both legs: the feet meet with a hip not toed out, which the rule's shape cannot cover: {p}")
        rule_bl["band_pitch_forward_of_deg"] = next_above(
            g["hip_pitch"], max(max(l["hip_pitch"], r["hip_pitch"]) for l, r, _ in outside), "both legs, hip pitch")
        rule_bl["band_roll_in_past_deg"] = next_above(
            g["hip_roll"], max(min(l["hip_roll"], r["hip_roll"]) for l, r, _ in outside), "both legs, hip roll")
    for p in hip["both_legs_poses"]:
        left, right = in_left_signs(p)
        if feet_meet(p) and not leg_blocked("left", left) and not leg_blocked("right", right) \
                and not both_legs_forbids(rule_bl, left, right):
            fail(f"both legs: still allowed after extending the rule: {p}")

    note_ka = robot.get("knee_ankle", {}).get("_note")
    note_bl = robot.get("both_legs", {}).get("_note")
    robot["knee_ankle"] = dict({"_note": note_ka} if note_ka else {}, **rule_ka)
    robot["both_legs"] = dict({"_note": note_bl} if note_bl else {}, **rule_bl)

    ROBOT_JSON.write_text(json.dumps(robot, indent=1) + "\n")

    print(f"updated {ROBOT_JSON}")
    print(f"  joints: {len(robot['joints'])} servo limits; head interlock: roll within +-"
          f"{robot['head_interlock']['yaw_window_deg']} deg of yaw 0")
    for leg in ("left", "right"):
        print(f"  hip map, {leg} leg: {len(maps[leg][0.0])} of {len(cells)} poses not allowed with the ankle level, "
              f"+{added[leg][-23.0]} with it at -23, +{added[leg][23.0]} at +23")
    print(f"  review's {len(review)} extra poses: {review_new} cell(s) not already in the design's maps")
    print(f"  knee x ankle: past knee {rule_ka['knee_over_deg']:g}, {len(rows_ka)} rows, agrees with both legs' sweeps")
    if outside:
        print(f"  both legs: the design's rule (forward of {design_rule['pitch_forward_of_deg']:g}, both toed out) lets "
              f"{len(outside)} measured feet contact(s) through:")
        for left, right, p in outside:
            print(f"    left {left}, right {p['right']}: {[c for c in p['contacts'] if 'left' in c and 'right' in c]}")
        print(f"    extended: also forward of {rule_bl['band_pitch_forward_of_deg']:g} with a hip rolled in past "
              f"{rule_bl['band_roll_in_past_deg']:g}")
    else:
        print("  both legs: the design's rule covers every measured feet contact")


if __name__ == "__main__":
    main()
