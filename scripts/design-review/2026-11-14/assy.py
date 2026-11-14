# SPDX-License-Identifier: Apache-2.0
import bpy, json, itertools
from mathutils import Vector
exec(open(ARGS["lib"]).read())
bpy.context.view_layer.update()
def owner(o):
    while o is not None and not o.name.startswith(R + ":"): o = o.parent
    return o
obs = solids(); MS = {o.name: M(o) for o in obs}; K = {o.name: kind(o) for o in obs}; OW = {o.name: (owner(o).name if owner(o) else None) for o in obs}
rev = json.load(open(ARGS["review"]))
TOL = 0.12e-3
def depth_tree(a, ta, b, off):
    if any(a.lo[i] + off[i] > b.hi[i] or a.hi[i] + off[i] < b.lo[i] for i in range(3)): return 0.0
    pairs = ta.overlap(b.tree)
    if not pairs: return 0.0
    best = 0.0
    sa = list({i for i, _ in pairs}); sb = list({j for _, j in pairs})
    if len(sa) > 800: sa = sa[::len(sa) // 800 + 1]
    if len(sb) > 800: sb = sb[::len(sb) // 800 + 1]
    for i in sa:
        tri = a.t[i]
        for p in ((a.v[tri[0]] + a.v[tri[1]] + a.v[tri[2]]) / 3, a.v[tri[0]], a.v[tri[1]], a.v[tri[2]]):
            q = p + off
            if b.inside(q): best = max(best, b.dist(q))
    for j in sb:
        tri = b.t[j]
        for p in ((b.v[tri[0]] + b.v[tri[1]] + b.v[tri[2]]) / 3, b.v[tri[0]], b.v[tri[1]], b.v[tri[2]]):
            q = p - off
            if a.inside(q): best = max(best, a.dist(q))
    return best
from mathutils.bvhtree import BVHTree as _B
def slide(names, against, path, step=0.001):
    worst = (0.0, None, None)
    pts = []
    for a, b in zip(path[:-1], path[1:]):
        n = max(1, int((b - a).length / step))
        pts += [a + (b - a) * k / n for k in range(n)]
    pts.append(path[-1])
    for off in pts:
        for nm in names:
            A = MS[nm]
            if not any(all(A.lo[i] + off[i] <= MS[g].hi[i] and A.hi[i] + off[i] >= MS[g].lo[i] for i in range(3)) for g in against): continue
            ta = _B.FromPolygons([v + off for v in A.v], A.t) if off.length else A.tree
            for g in against:
                d = depth_tree(A, ta, MS[g], off)
                if d > worst[0]: worst = (d, off.copy(), g)
        if worst[0] > 1e-3: break
    return worst
def axes(frame):
    mw = bpy.data.objects[frame].matrix_world.to_3x3()
    return {s + n: (mw.col[i].normalized() * (1 if s == "+" else -1)) for i, n in enumerate("xyz") for s in "+-"}
LATER_TRUNK = {"electronics_enclosure", "body_lid", "uno_cover"}
res = []
for r in rev:
    j = r["servo"][4:]; frame = r["servo"]
    case = [n for n in MS if n.startswith(frame + "_case") and K[n] == "servo"] + [frame + "_spline"]
    holders = [h[0] for h in r["holder"]]; outs = [o[0] for o in r["output"]]
    hb = {OW[h] for h in holders if h in OW}
    against = [n for n in MS if K[n] in ("printed", "servo", "bearing") and OW[n] in hb and n not in case and n not in outs and n not in LATER_TRUNK
               and not n.endswith("_cover")]
    ax = axes(frame)
    best = None; tried = {}
    for nm, d in ax.items():
        w = slide(case, against, [d * 0.03, Vector((0, 0, 0))])
        tried[nm] = round(w[0] * 1000, 2)
        if w[0] <= TOL and best is None: best = ("straight " + nm, w)
    if best is None:   # L-path: lift along d1 by h, approach along d2
        for (n1, d1), (n2, d2) in itertools.permutations(ax.items(), 2):
            if abs(d1.dot(d2)) > 0.1: continue
            for h in (5, 7.5, 10):
                w = slide(case, against, [d1 * h / 1000 + d2 * 0.03, d1 * h / 1000, Vector((0, 0, 0))])
                if w[0] <= TOL: best = (f"L: in along {n2} at {h} mm lift, then {n1} down", w); break
            if best: break
    print('..', j, flush=True)
    res.append(dict(step=f"servo {j} into {holders}", ok=best is not None, path=best[0] if best else None, straight=tried, against=len(against)))
    # output part onto the horn along the shaft axis
    for o in outs:
        if o not in MS: continue
        ag = [n for n in MS if (n in case or n in holders or (K[n] == "bearing" and j in n) or (K[n] == "servo" and OW[n] in hb)) and n != o]
        w = slide([o], ag, [ax["+z"] * 0.010, Vector((0, 0, 0))])
        res.append(dict(step=f"{o} onto {j} horn (+z, 10 mm)", ok=w[0] <= 0.3e-3, depth_mm=round(w[0] * 1000, 2), at_mm=None if w[1] is None else round(w[1].length * 1000, 1), hit=w[2]))
# plates and caps: along the joint axis, either side
extra = [("body_neck_bearing_plate", "scs_neck_pitch", ["body_frame", "neck_cradle", "scs_neck_pitch_case", "neck_brg_693"]),
         ("head_pitch_bearing_plate", "scs_head_pitch", ["head_pitch_u", "neck_cradle", "scs_head_pitch_case", "head_pitch_brg_693"]),
         ("left_foot_bearing_plate", "scs_left_ankle", ["left_foot", "left_shin", "scs_left_ankle_case", "left_ankle_brg_693"]),
         ("right_foot_bearing_plate", "scs_right_ankle", ["right_foot", "right_shin", "scs_right_ankle_case", "right_ankle_brg_693"]),
         ("left_knee_cap", "scs_left_knee", ["left_thigh", "left_shin", "scs_left_knee_case", "left_knee_brg_6702ZZ"]),
         ("right_knee_cap", "scs_right_knee", ["right_thigh", "right_shin", "scs_right_knee_case", "right_knee_brg_6702ZZ"])]
for p, fr, ag in extra:
    ax = axes(fr); t = {}
    for nm, d in ax.items():
        w = slide([p], ag, [d * 0.02, Vector((0, 0, 0))]); t[nm] = round(w[0] * 1000, 2)
    ok = [n for n, v in t.items() if v <= 0.12]
    res.append(dict(step=f"{p}", ok=bool(ok), path=ok, straight=t))
json.dump(res, open(ARGS["out"], "w"), indent=1)
for r in res: print(("OK      " if r["ok"] else "BLOCKED ") + r["step"], "|", r.get("path") or "", r.get("depth_mm", ""), r.get("hit", "") or "", "" if r["ok"] else r.get("straight", ""))
