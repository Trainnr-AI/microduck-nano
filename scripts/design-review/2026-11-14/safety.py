# SPDX-License-Identifier: Apache-2.0
import bpy, bmesh, math, json
from mathutils import Vector
exec(open(ARGS["lib"]).read())
bpy.context.view_layer.update()
obs = solids(); MS = {o.name: M(o) for o in obs}; K = {o.name: kind(o) for o in obs}
def mind(a, b):
    A, B = MS[a], MS[b]; best = 1e9
    for X, Y in ((A, B), (B, A)):
        for v in X.v[::max(1, len(X.v) // 20000)]:
            r = Y.tree.find_nearest(v, 0.03)
            if r[0] is not None and r[3] < best: best = r[3]
    return best * 1000
bat = "battery_3s"
print("battery bbox mm", [round(x * 1000, 1) for x in MS[bat].lo], [round(x * 1000, 1) for x in MS[bat].hi])
near = sorted(((mind(bat, n), n, K[n]) for n in MS if n != bat and all(MS[n].lo[i] < MS[bat].hi[i] + 0.012 and MS[n].hi[i] > MS[bat].lo[i] - 0.012 for i in range(3))))
for d, n, k in near[:14]: print(f"  battery -> {n} ({k}): {d:.2f} mm")
print("ubec -> nearest:", sorted((round(mind("ubec", n), 2), n) for n in MS if n != "ubec" and all(MS[n].lo[i] < MS["ubec"].hi[i] + 0.01 and MS[n].hi[i] > MS["ubec"].lo[i] - 0.01 for i in range(3)))[:6])
# overall size and outer sharp edges of printed parts: convex edges with a face angle >= 80 deg, length in mm
tot = {}
for n, m in MS.items():
    if K[n] != "printed": continue
    bm = bmesh.new(); bm.from_mesh(m.o.data); bm.transform(m.o.matrix_world); bm.normal_update()
    L = 0.0; Lall = 0.0
    for e in bm.edges:
        if len(e.link_faces) != 2: continue
        ang = e.calc_face_angle(0.0)
        if ang < math.radians(20): continue
        ln = e.calc_length() * 1000; Lall += ln
        if e.is_convex and ang >= math.radians(80): L += ln
    tot[n] = (round(L), round(Lall)); bm.free()
print("convex edges >= 80 deg (mm of edge), top parts:", sorted(((v[0], n) for n, v in tot.items()), reverse=True)[:8])
lo = [min(m.lo[i] for m in MS.values()) * 1000 for i in range(3)]; hi = [max(m.hi[i] for m in MS.values()) * 1000 for i in range(3)]
print("robot bbox mm", [round(hi[i] - lo[i], 1) for i in range(3)])
# smallest loose bought parts
print("small parts (max dimension mm):", sorted({(round(max((m.hi - m.lo)) * 1000, 1), K[n]) for n, m in MS.items() if K[n] in ("bearing", "nut")})[:4])
