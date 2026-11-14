# SPDX-License-Identifier: Apache-2.0
# Reviewer's own geometry helpers (independent of the repo's check tools, except the ray-parity inside test idea).
import bpy, bmesh
from mathutils import Vector
from mathutils.bvhtree import BVHTree
R = "duck_scs0009"
SKIPC = {'duck_v2_exploded','wiring_exploded','xgoduck_original','dimensions','orig_head_parts','coupon','wiring','wiring_mesh','cables','print_tools'}
RAYS = (Vector((0.267, 0.534, 0.802)), Vector((-0.577, 0.577, 0.577)), Vector((0.408, -0.816, 0.408)))
def solids():
    out = []
    for o in bpy.data.objects:
        if o.type != "MESH" or not o.users_collection or {c.name for c in o.users_collection} & SKIPC: continue
        if o.name.startswith(R + ":") or not o.visible_get(): continue
        out.append(o)
    return out
def kind(o):
    n = o.name
    if "mass_g_solid_PLA" in o.keys(): return "printed"
    if n.endswith("_case_screws"): return "case_screws"
    if n.endswith("_case"): return "servo"
    if "_horn_screw" in n or n.endswith("_screw_M2x4kit"): return "horn_screw"
    if "_horn_" in n: return "horn"
    if n.endswith("_spline"): return "spline"
    if "_nut_" in n: return "nut"
    if "screw" in n: return "screw"
    if "_brg_" in n: return "bearing"
    if any(c.name == "electronics" for c in o.users_collection) or n.startswith("cam_"): return "elec"
    return "other"
class M:
    def __init__(s, o):
        s.o = o; s.name = o.name
        bm = bmesh.new(); bm.from_mesh(o.data); bmesh.ops.triangulate(bm, faces=bm.faces); bm.transform(o.matrix_world)
        bm.verts.ensure_lookup_table()
        s.v = [v.co.copy() for v in bm.verts]; s.t = [tuple(v.index for v in f.verts) for f in bm.faces]
        s.tree = BVHTree.FromPolygons(s.v, s.t)
        # closed components for the inside test
        seen = set(); s.comps = []
        for f in bm.faces:
            if f.index in seen: continue
            st = [f]; seen.add(f.index); comp = []
            while st:
                g = st.pop(); comp.append(g)
                for e in g.edges:
                    for h in e.link_faces:
                        if h.index not in seen: seen.add(h.index); st.append(h)
            vs = sorted({v.index for g in comp for v in g.verts}); ix = {v: i for i, v in enumerate(vs)}
            s.comps.append(BVHTree.FromPolygons([s.v[v] for v in vs], [[ix[v.index] for v in g.verts] for g in comp]))
        s.lo = Vector([min(v[i] for v in s.v) for i in range(3)]); s.hi = Vector([max(v[i] for v in s.v) for i in range(3)])
        bm.free()
    def inside(s, p):
        for t in s.comps:
            c = 0
            for d in RAYS:
                n, o = 0, p.copy()
                for _ in range(64):
                    loc, _, _, _ = t.ray_cast(o, d)
                    if loc is None: break
                    n += 1; o = loc + d * 1e-7
                c += n % 2
            if c >= 2: return True
        return False
    def dist(s, p):
        r = s.tree.find_nearest(p)
        return r[3] if r[0] is not None else 0.0
def depth(a, b, off=None, margin=0.0):
    """max penetration depth (m) of a (translated by off) into b or b into a; 0 if surfaces do not cross"""
    if off is None: off = Vector((0, 0, 0))
    if any(a.lo[i] + off[i] > b.hi[i] + margin or a.hi[i] + off[i] < b.lo[i] - margin for i in range(3)): return 0.0
    ta = a.tree if off.length == 0 else BVHTree.FromPolygons([v + off for v in a.v], a.t)
    pairs = ta.overlap(b.tree)
    if not pairs:
        return 0.0
    best = 0.0
    sa = {i for i, _ in pairs}; sb = {j for _, j in pairs}
    if len(sa) > 4000: sa = set(list(sa)[::len(sa) // 4000 + 1])
    if len(sb) > 4000: sb = set(list(sb)[::len(sb) // 4000 + 1])
    pts = set()
    for i in sa:
        tri = a.t[i]; pts.update(tri)
        c = (a.v[tri[0]] + a.v[tri[1]] + a.v[tri[2]]) / 3 + off
        if b.inside(c): best = max(best, b.dist(c))
    for vi in pts:
        p = a.v[vi] + off
        if b.inside(p): best = max(best, b.dist(p))
    pts = set()
    for j in sb:
        tri = b.t[j]; pts.update(tri)
        c = (b.v[tri[0]] + b.v[tri[1]] + b.v[tri[2]]) / 3 - off
        if a.inside(c): best = max(best, a.dist(c))
    for vi in pts:
        p = b.v[vi] - off
        if a.inside(p): best = max(best, a.dist(p))
    return best
