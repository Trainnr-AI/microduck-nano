# SPDX-License-Identifier: Apache-2.0
import bpy, bmesh, json, numpy as np, re, collections
bpy.context.view_layer.update()
skipc={'duck_v2_exploded','wiring_exploded','xgoduck_original','dimensions','orig_head_parts','coupon'}
rows=[]
for o in bpy.data.objects:
    if o.type!="MESH" or set(c.name for c in o.users_collection)&skipc or not o.users_collection: continue
    n=o.name.lower()
    if not any(k in n for k in ("screw","nut","brg")) or n.endswith("_case_screws"): continue
    bm=bmesh.new(); bm.from_mesh(o.data); bm.transform(o.matrix_world)
    seen={}; comps=[]
    for v in bm.verts:
        if v.index in seen: continue
        st=[v]; seen[v.index]=len(comps); pts=[]
        while st:
            x=st.pop(); pts.append(x.co[:])
            for e in x.link_edges:
                y=e.other_vert(x)
                if y.index not in seen: seen[y.index]=len(comps); st.append(y)
        comps.append(np.array(pts)*1000)
    cs=[]
    for P in comps:
        c=P.mean(0); u,s,vt=np.linalg.svd(P-c,full_matrices=False); ax=vt[0]
        t=(P-c)@ax; rad=np.linalg.norm((P-c)-np.outer(t,ax),axis=1)
        cs.append(dict(L=round(float(t.max()-t.min()),2),D=round(float(2*rad.max()),2),c=[round(float(x),1) for x in c]))
    rows.append(dict(name=o.name,ncomp=len(comps),comps=cs))
    bm.free()
json.dump(rows,open(ARGS["out"],"w"))
