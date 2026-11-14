# SPDX-License-Identifier: Apache-2.0
# Reviewer's quasi-static single-support check: joint torques needed to hold the robot on ONE foot (gravity only),
# with the ground reaction (0,0,Mg) applied under the centre of mass on the stance foot.
import mujoco, numpy as np, math, sys, json
m=mujoco.MjModel.from_xml_path(sys.argv[1]); d=mujoco.MjData(m)
aid={m.actuator(i).name:i for i in range(m.nu)}
jadr={n:m.jnt_qposadr[m.actuator_trnid[i][0]] for n,i in aid.items()}
dadr={n:m.jnt_dofadr[m.actuator_trnid[i][0]] for n,i in aid.items()}
lim=0.1883; M=float(sum(m.body_mass)); g=9.81
stand=np.array(m.key("STAND").ctrl); q0=np.array(m.key("STAND").qpos)
def evalpose(ctrl, stance):
    d.qpos[:]=q0
    for n,i in aid.items(): d.qpos[jadr[n]]=ctrl[i]
    d.qvel[:]=0; mujoco.mj_forward(m,d)
    com=d.subtree_com[0].copy()
    fb=m.body(f"ankle_{stance}").id if any(m.body(i).name==f"ankle_{stance}" for i in range(m.nbody)) else None
    sid=m.site(f"{stance}_foot").id; fb=m.site_bodyid[sid]
    foot=d.site_xpos[sid].copy()
    p=np.array([com[0],com[1],foot[2]])
    jacp=np.zeros((3,m.nv)); mujoco.mj_jac(m,d,jacp,None,p,fb)
    tau=d.qfrc_bias - jacp.T@np.array([0,0,M*g])
    # foot footprint from its collision geoms
    lo=np.full(3,1e9); hi=-lo
    for gi in range(m.ngeom):
        if m.geom_bodyid[gi]==fb and m.geom_contype[gi]:
            c=d.geom_xpos[gi]; r=m.geom_rbound[gi]; 
            if m.geom_type[gi]==mujoco.mjtGeom.mjGEOM_MESH:
                vadr=m.mesh_vertadr[m.geom_dataid[gi]]; vn=m.mesh_vertnum[m.geom_dataid[gi]]
                V=m.mesh_vert[vadr:vadr+vn]@d.geom_xmat[gi].reshape(3,3).T+c
                lo=np.minimum(lo,V.min(0)); hi=np.maximum(hi,V.max(0))
    return com,foot,lo,hi,{n:tau[dadr[n]] for n in aid}, tau[:6]
out=[]
for stance,free,sg in (("left","right",1),("right","left",-1)):
    for lift in (0.0,0.25,0.45):
        best=None
        for s in np.arange(-0.34,0.341,0.005):
            c=stand.copy()
            for j in ("left_hip_roll","right_hip_roll"): c[aid[j]]+=s
            sh=1 if free=="right" else -1
            c[aid[f"{free}_hip_pitch"]]+=sh*lift; c[aid[f"{free}_knee"]]+=sh*2*lift; c[aid[f"{free}_ankle"]]+=-sh*lift
            com,foot,lo,hi,tau,root=evalpose(c,stance)
            off=abs(com[1]-(lo[1]+hi[1])/2)
            if best is None or off<best[0]: best=(off,s,com,lo,hi,tau,c)
        off,s,com,lo,hi,tau,c=best
        rng=m.jnt_range[m.actuator_trnid[aid[f"{stance}_hip_roll"]][0]]
        hr=c[aid[f"{stance}_hip_roll"]]; hr2=c[aid[f"{free}_hip_roll"]]; rng2=m.jnt_range[m.actuator_trnid[aid[f"{free}_hip_roll"]][0]]
        top=sorted(tau.items(),key=lambda kv:-abs(kv[1]))[:4]
        print(f"stance {stance}, swing lift {lift}: hip-roll shift {math.degrees(s):+.1f} deg -> CoM y {1000*com[1]:+.1f} mm vs foot y [{1000*lo[1]:+.1f},{1000*hi[1]:+.1f}] (centre miss {1000*off:.1f} mm); "
              f"stance roll {math.degrees(hr):+.1f} (limit {math.degrees(rng[0]):.1f}..{math.degrees(rng[1]):.1f}), swing roll {math.degrees(hr2):+.1f} (limit {math.degrees(rng2[0]):.1f}..{math.degrees(rng2[1]):.1f}); "
              +", ".join(f"{k} {100*abs(v)/lim:.0f}%" for k,v in top))
# double support STAND reference via the same method is not applicable; print mass
print("mass",M,"foot width etc. above")
print("--- within the hip-roll servo limit")
for stance,free in (("left","right"),("right","left")):
    for sdeg in (0,-3.5,-7.0) if stance=="left" else (0,3.5,7.0):
        c=stand.copy()
        for j in ("left_hip_roll","right_hip_roll"): c[aid[j]]+=math.radians(sdeg)
        com,foot,lo,hi,tau,root=evalpose(c,stance)
        inner=lo[1] if stance=="left" else hi[1]
        gap=(com[1]-inner) if stance=="left" else (inner-com[1])
        print(f"stance {stance} roll {math.degrees(c[aid[stance+'_hip_roll']]):+.1f}: CoM is {1000*gap:+.1f} mm inside the stance foot's inner edge (negative = outside the foot); {stance}_hip_roll {100*abs(tau[stance+'_hip_roll'])/lim:.0f}% of stall, knee {100*abs(tau[stance+'_knee'])/lim:.0f}%")
