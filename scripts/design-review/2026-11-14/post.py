# SPDX-License-Identifier: Apache-2.0
import json,collections,sys
st=json.load(open('rv/static.json'))['rows']
base={frozenset((a,b)) for a,ka,b,kb,d in st if d>=0.0}
def load(tag, th=0.05):
    d=json.load(open(f'rv/sw_{tag}.json'))
    out={}
    for k,v in d.items():
        out[k]=[h for h in v if frozenset(h[:2]) not in base and h[2]>th]
    return out
if __name__=="__main__":
    hm=json.load(open('review-main/projects/xgoduck_scs0009/data/hip_collision_map.json'))["poses"]
    mine=load("hm")
    agree=both=onlymine=onlytheirs=0; om=[]; ot=[]
    for i,p in enumerate(hm):
        m=bool(mine[f"hm {i}"]); t=p["collides"]
        if m and t: both+=1
        elif m: onlymine+=1; om.append((i,p,mine[f"hm {i}"]))
        elif t: onlytheirs+=1; ot.append((i,p))
    print("hip map: theirs collide",sum(p["collides"] for p in hm),"mine",sum(bool(v) for v in mine.values()),"both",both,"only mine",onlymine,"only theirs",onlytheirs)
    pc=collections.Counter()
    for i,p,h in om:
        for x in h: pc[tuple(sorted(x[:2]))]+=1
    print(" only-mine pair counts:",pc.most_common(8))
    for i,p,h in om[:6]: print("   ",{k:p[k] for k in ('hip_pitch','hip_roll','hip_yaw','knee')},h[:3])
    for i,p in ot[:8]: print("  only theirs",{k:p[k] for k in ('hip_pitch','hip_roll','hip_yaw','knee')},p["contacts"][:2])
