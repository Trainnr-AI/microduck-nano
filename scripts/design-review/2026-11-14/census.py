# SPDX-License-Identifier: Apache-2.0
import bpy, bmesh, json
from mathutils import Vector
bpy.context.view_layer.update()
rows=[]
for o in bpy.data.objects:
    if o.type!="MESH": continue
    cols=[c.name for c in o.users_collection]
    bm=bmesh.new(); bm.from_mesh(o.data); bm.transform(o.matrix_world)
    nonman=sum(1 for e in bm.edges if len(e.link_faces)!=2)
    # shells
    seen=set(); shells=0
    for v in bm.verts:
        if v.index in seen: continue
        shells+=1; st=[v]; seen.add(v.index)
        while st:
            x=st.pop()
            for e in x.link_edges:
                y=e.other_vert(x)
                if y.index not in seen: seen.add(y.index); st.append(y)
    try: vol=bm.calc_volume(signed=True)*1e9
    except Exception: vol=None
    cs=[v.co for v in bm.verts]
    if cs:
        lo=[min(c[i] for c in cs)*1000 for i in range(3)]; hi=[max(c[i] for c in cs)*1000 for i in range(3)]
    else: lo=hi=[0,0,0]
    props={k:(o[k] if isinstance(o[k],(int,float,str)) else str(o[k])[:60]) for k in o.keys() if not k.startswith("_")}
    par=o.parent.name if o.parent else None
    rows.append(dict(name=o.name,cols=cols,verts=len(bm.verts),faces=len(bm.faces),nonman=nonman,shells=shells,vol_mm3=vol,
        dims=[round(hi[i]-lo[i],2) for i in range(3)],lo=[round(x,2) for x in lo],hi=[round(x,2) for x in hi],props=props,parent=par,visible=o.visible_get()))
    bm.free()
json.dump(rows,open(ARGS["out"],"w"))
print("meshes",len(rows))
