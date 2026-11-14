# SPDX-License-Identifier: Apache-2.0
import itertools
src = open(ARGS["assy"]).read()
exec(src[:src.index("LATER_TRUNK")])
Z = Vector((0, 0, 0))
W = {"+x": Vector((1, 0, 0)), "-x": Vector((-1, 0, 0)), "+y": Vector((0, 1, 0)), "-y": Vector((0, -1, 0)), "+z": Vector((0, 0, 1)), "-z": Vector((0, 0, -1))}
def best(names, against, frame=None, lifts=(2, 3, 4, 4.5, 5, 6, 7.5, 10), tol=0.12e-3, reach=0.03):
    ax = axes(frame) if frame else W
    t = {}
    for n, d in ax.items():
        w = slide(names, against, [d * reach, Z]); t[n] = round(w[0] * 1000, 2)
        if w[0] <= tol: return "straight " + n, t
    for (n1, d1), (n2, d2) in itertools.permutations(ax.items(), 2):
        if abs(d1.dot(d2)) > 0.1: continue
        for h in lifts:
            w = slide(names, against, [d1 * h / 1000 + d2 * reach, d1 * h / 1000, Z])
            if w[0] <= tol: return f"L: offset {h} mm along {n1}, in along -{n2[1]} side {n2}, then seat", t
    return None, t
def case(j): return [f"scs_{j}_case", f"scs_{j}_spline"]
jobs = [
 ("left_yaw_connector onto hip-yaw horn", ["left_yaw_connector"], ["body_frame"] + case("left_hip_yaw") + ["left_hip_yaw_brg_6702ZZ"], "scs_left_hip_yaw", 0.3e-3),
 ("right_yaw_connector onto hip-yaw horn", ["right_yaw_connector"], ["body_frame"] + case("right_hip_yaw") + ["right_hip_yaw_brg_6702ZZ"], "scs_right_hip_yaw", 0.3e-3),
 ("left_shin onto knee horn (bearing not in yet)", ["left_shin"], ["left_thigh"] + case("left_knee"), "scs_left_knee", 0.3e-3),
 ("right_shin onto knee horn (bearing not in yet)", ["right_shin"], ["right_thigh"] + case("right_knee"), "scs_right_knee", 0.3e-3),
 ("6702 into left shin seat", ["left_knee_brg_6702ZZ"], ["left_shin", "left_thigh"] + case("left_knee"), "scs_left_knee", 0.12e-3),
 ("head-roll servo into head_base", case("head_roll"), ["head_base", "head_roll_brg_6702ZZ"], "scs_head_roll", 0.12e-3),
 ("yaw_block onto head-roll horn", ["yaw_block"], ["head_base", "head_roll_brg_6702ZZ"] + case("head_roll"), None, 0.3e-3),
 ("head-yaw servo into yaw_block", case("head_yaw"), ["yaw_block", "head_base"] + case("head_roll"), "scs_head_yaw", 0.12e-3),
 ("jaw into head_base", ["head_jaw"], ["head_base"] + case("head_roll"), None, 0.3e-3),
 ("beak servo into head_base (jaw in)", case("mouth"), ["head_base", "head_jaw"] + case("head_roll"), "scs_mouth", 0.3e-3),
 ("left hip-yaw servo into body_frame (right servo + neck fork in)", case("left_hip_yaw"), ["body_frame", "left_hip_yaw_brg_6702ZZ"] + case("right_hip_yaw"), "scs_left_hip_yaw", 0.12e-3),
 ("electronics_enclosure onto body_frame", ["electronics_enclosure"], ["body_frame"] + case("left_hip_yaw") + case("right_hip_yaw"), None, 0.12e-3),
 ("head_dome onto head_base", ["head_dome"], ["head_base", "head_face", "yaw_block"] + case("head_roll") + case("mouth") + ["cam_ov5640_b"], None, 0.12e-3),
 ("head_face onto head_base", ["head_face"], ["head_base", "head_jaw"] + case("mouth"), None, 0.12e-3),
]
for name, parts, ag, fr, tol in jobs:
    p, t = best(parts, [a for a in ag if a in MS], fr, tol=tol)
    print(("OK      " if p else "BLOCKED ") + name, "|", p or t, flush=True)
