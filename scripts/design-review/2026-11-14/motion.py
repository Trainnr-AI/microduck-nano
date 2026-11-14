# SPDX-License-Identifier: Apache-2.0
# Driver for the repo's leg_motion_check.py: builds the part list from the scene, the servo limits from joint_ranges_v2.json,
# and combined poses; then execs the tool unchanged.
import bpy, json, math, itertools
A = ARGS
R = "duck_scs0009"
skipc = {'duck_v2_exploded','wiring_exploded','xgoduck_original','dimensions','orig_head_parts','coupon','wiring','wiring_mesh','cables','print_tools'}
def owner(o):
    while o is not None and not o.name.startswith(R + ":"):
        o = o.parent
    return o
parts = {}
for o in bpy.data.objects:
    if o.type != "MESH" or not o.users_collection or {c.name for c in o.users_collection} & skipc: continue
    if o.name.startswith(R + ":") or not o.visible_get(): continue
    n = o.name
    if "_horn_" in n or n.endswith("_spline") or n.endswith("_screw_M2x4kit") or n.endswith("_case_screws"): continue   # rotate with the output / inside the case
    b = owner(o)
    if b is None:
        print("UNOWNED", n); continue
    parts[n] = b.name[len(R) + 1:]
print("objects in the sweep:", len(parts))
rng = json.load(open(A["ranges"]))
keys = json.load(open(A["keys"]))["keys"]
rad = lambda d: {k: math.radians(v) for k, v in d.items()}
poses = {}
ST, SI = keys["STAND"], keys["SIT"]
names = list(rng["joints"])
full = lambda d: {j: d.get(j, 0.0) for j in names}
ST, SI = full(ST), full(SI)
if "base" in A["sets"]:
    poses["STAND"] = rad(ST); poses["SIT"] = rad(SI); poses["FOLD key"] = rad(full(keys["FOLD"]))
    for i in range(1, 20):
        t = i / 20
        poses[f"sit path {t:.2f}"] = rad({j: ST[j] + (SI[j] - ST[j]) * t for j in names})
    # hip yaw/roll box corners, both legs at once (claimed clear >= 0.5 mm)
    for ly, lr, ry, rr in itertools.product((-6.3, 19), (-12, 19.5), (-19, 6.3), (-19.5, 12)):
        poses[f"hipbox L y{ly} r{lr} R y{ry} r{rr}"] = rad({"left_hip_yaw": ly, "left_hip_roll": lr, "right_hip_yaw": ry, "right_hip_roll": rr})
    # neck x head pitch box, head yaw, head roll interlock boundary
    for n_, h in itertools.product((-80, -40, 0, 30), (-5, 30, 65)):
        poses[f"neck {n_} head {h}"] = rad({"neck_pitch": n_, "head_pitch": h})
    for y, r in itertools.product((-2.5, 0, 2.5), (-15, 15)):
        poses[f"headyaw {y} roll {r}"] = rad({"head_yaw": y, "head_roll": r})
    for y in (-150, -90, 90, 150):
        for n_, h in ((20, 20), (-80, 65), (30, -5), (30, 65)):
            poses[f"headyaw {y} neck {n_} head {h}"] = rad({"head_yaw": y, "neck_pitch": n_, "head_pitch": h})
if "walk" in A["sets"]:
    # walking envelope corners from JOINT_LIMITS.md, legs in antiphase (left values given; right = mirror of the opposite pitch phase)
    for y, r, p, k, a in itertools.product((-6.3, 12.6), (-10.9, 10.9), (-45.8, 45.8), (-55, 55), (-23, 23)):
        poses[f"walk L y{y} r{r} p{p} k{k} a{a}"] = rad({"left_hip_yaw": y, "left_hip_roll": r, "left_hip_pitch": p, "left_knee": k, "left_ankle": a,
            "right_hip_yaw": -y, "right_hip_roll": -r, "right_hip_pitch": p, "right_knee": k, "right_ankle": a})
G = {"ARGS": {"robot": R, "parts": parts, "servo_cases": [], "joints": names if "single" in A["sets"] else [], "samples": A.get("samples", 9),
              "ranges_deg": rng["joints"], "poses": poses, "min_mm3": 0.05, "json_out": A["out"], "full": True}, "__name__": "__main__"}
exec(compile(open(A["tool"]).read(), A["tool"], "exec"), G)
