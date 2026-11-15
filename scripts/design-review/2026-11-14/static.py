# SPDX-License-Identifier: Apache-2.0
import bpy, json, sys, time
exec(open(ARGS["lib"]).read())
bpy.context.view_layer.update()
obs = solids(); ms = [M(o) for o in obs]; ks = {m.name: kind(m.o) for m in ms}
import collections
print("solids", len(ms), dict(collections.Counter(ks.values())), "tris", sum(len(m.t) for m in ms))
rows = []; npairs = 0; t0 = time.time()
for i, a in enumerate(ms):
    for b in ms[i + 1:]:
        if any(a.lo[k] > b.hi[k] or a.hi[k] < b.lo[k] for k in range(3)): continue
        npairs += 1
        d = depth(a, b)
        if d > 0: rows.append((a.name, ks[a.name], b.name, ks[b.name], round(d * 1000, 3)))
json.dump({"pairs_with_bbox_overlap": npairs, "rows": rows}, open(ARGS["out"], "w"))
print("pairs", npairs, "crossing", len(rows), "t", round(time.time() - t0))
