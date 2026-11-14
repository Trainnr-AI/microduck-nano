# SPDX-License-Identifier: Apache-2.0
import mujoco, json, math, sys
m=mujoco.MjModel.from_xml_path(sys.argv[1])
out={"mass_kg":float(sum(m.body_mass)),"nu":m.nu,"timestep":m.opt.timestep,"act":{},"keys":{}}
for i in range(m.nu):
    j=m.actuator_trnid[i][0]
    out["act"][m.actuator(i).name]=dict(force=list(map(float,m.actuator_forcerange[i])),kp=float(m.actuator_gainprm[i][0]),kv=float(-m.actuator_biasprm[i][2]),
        range_deg=[round(math.degrees(x),2) for x in m.jnt_range[j]],ctrlrange_deg=[round(math.degrees(x),2) for x in m.actuator_ctrlrange[i]],
        armature=float(m.dof_armature[m.jnt_dofadr[j]]),damping=float(m.dof_damping[m.jnt_dofadr[j]]),frictionloss=float(m.dof_frictionloss[m.jnt_dofadr[j]]))
for k in range(m.nkey):
    out["keys"][m.key(k).name]={m.actuator(i).name:round(math.degrees(m.key(k).ctrl[i]),2) for i in range(m.nu)}
    out["keys"][m.key(k).name+"_rootz"]=float(m.key(k).qpos[2])
json.dump(out,open(sys.argv[2],"w"),indent=1)
print("mass",out["mass_kg"],"nu",m.nu,"dt",m.opt.timestep, "integrator",m.opt.integrator)
a=out["act"]["left_knee"]; print("left_knee",a)
print({k:(v if isinstance(v,float) else {j:x for j,x in v.items() if x}) for k,v in out["keys"].items()})
import collections
print("force limits",collections.Counter(round(v["force"][1],4) for v in out["act"].values()))
print("ranges",{k:v["range_deg"] for k,v in out["act"].items()})
