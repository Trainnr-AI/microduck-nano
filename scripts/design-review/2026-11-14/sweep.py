# SPDX-License-Identifier: Apache-2.0
# Reviewer's own pose sweep: exact meshes, surfaces crossing + penetration depth (no booleans).
import bpy, json, math, time
from mathutils import Quaternion, Vector, Matrix
from mathutils.bvhtree import BVHTree
exec(open(ARGS["lib"]).read())
bpy.context.view_layer.update()
def owner(o):
    while o is not None and not o.name.startswith(R + ":"): o = o.parent
    return o
obs = [o for o in solids() if kind(o) in ("printed", "servo", "bearing", "screw", "nut", "elec") and owner(o) is not None]
ms = [M(o) for o in obs]
rest = {m.name: m.o.matrix_world.copy() for m in ms}
joints = {e["joint"]: e for e in bpy.data.collections[R].objects if e.type == "EMPTY" and "joint" in e and e.name.startswith(R + ":")}
q0 = {n: e.rotation_quaternion.copy() for n, e in joints.items()}
TH = ARGS.get("th_mm", 0.05) / 1000
def depth_T(a, Ta, tra, b, Tb, trb):
    pairs = tra.overlap(trb)
    if not pairs: return 0.0
    Tai, Tbi = Ta.inverted(), Tb.inverted()
    best = 0.0
    sa = list({i for i, _ in pairs}); sb = list({j for _, j in pairs})
    if len(sa) > 1500: sa = sa[::len(sa) // 1500 + 1]
    if len(sb) > 1500: sb = sb[::len(sb) // 1500 + 1]
    X = Tbi @ Ta; Y = Tai @ Tb
    seen = set()
    for i in sa:
        tri = a.t[i]
        for p in [(a.v[tri[0]] + a.v[tri[1]] + a.v[tri[2]]) / 3] + [a.v[k] for k in tri if k not in seen]:
            q = X @ p
            if b.inside(q): best = max(best, b.dist(q))
        seen.update(tri)
    seen = set()
    for j in sb:
        tri = b.t[j]
        for p in [(b.v[tri[0]] + b.v[tri[1]] + b.v[tri[2]]) / 3] + [b.v[k] for k in tri if k not in seen]:
            q = Y @ p
            if a.inside(q): best = max(best, a.dist(q))
        seen.update(tri)
    return best
def corners(m, T):
    c = [T @ Vector((x, y, z)) for x in (m.lo[0], m.hi[0]) for y in (m.lo[1], m.hi[1]) for z in (m.lo[2], m.hi[2])]
    return [min(p[i] for p in c) for i in range(3)], [max(p[i] for p in c) for i in range(3)]
I4 = Matrix.Identity(4)
def check(q):
    for n, e in joints.items():
        e.rotation_quaternion = q0[n] @ Quaternion(Vector(e["joint_axis"]), math.radians(q.get(n, 0.0)))
    bpy.context.view_layer.update()
    T = {m.name: m.o.matrix_world @ rest[m.name].inverted() for m in ms}
    key = {m.name: tuple(round(x, 6) for r in T[m.name] for x in r) for m in ms}
    bb = {m.name: corners(m, T[m.name]) for m in ms}
    trees = {}
    def tr(m):
        if m.name not in trees:
            Tm = T[m.name]
            trees[m.name] = m.tree if key[m.name] == KEY0 else BVHTree.FromPolygons([Tm @ v for v in m.v], m.t)
        return trees[m.name]
    hits = []
    for i, a in enumerate(ms):
        for b in ms[i + 1:]:
            if key[a.name] == key[b.name]: continue                      # same rigid group in this pose
            (al, ah), (bl, bh) = bb[a.name], bb[b.name]
            if any(al[k] > bh[k] or ah[k] < bl[k] for k in range(3)): continue
            d = depth_T(a, T[a.name], tr(a), b, T[b.name], tr(b))
            if d > TH: hits.append((a.name, b.name, round(d * 1000, 2)))
    return hits
KEY0 = tuple(round(x, 6) for r in I4 for x in r)
out = {}; t0 = time.time()
try:
    for P in ARGS["poses"]:
        out[P["label"]] = check(P["q"])
finally:
    for n, e in joints.items(): e.rotation_quaternion = q0[n]
json.dump(out, open(ARGS["out"], "w"))
print("poses", len(out), "colliding", sum(1 for v in out.values() if v), "t", round(time.time() - t0))
