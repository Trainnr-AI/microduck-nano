# Review scripts, 2026-11-14

The scripts used for the independent design review in [docs/project/design-review-2026-11-14.md](../../../docs/project/design-review-2026-11-14.md),
kept as they were run so the numbers can be reproduced. They are review tools, not a maintained library.

They run against the design repository (`agent-cad3d`, project `projects/xgoduck_scs0009/`), which is not public yet:
the Blender model `xgoduck_scs0009.blend` and the MuJoCo scene `sim/scene_scs0009_v2.xml`.

Blender scripts run headless through `run.py`, which passes a JSON file to the script as `ARGS`:

```sh
blender -b xgoduck_scs0009.blend --python run.py -- <script>.py <args>.json
```

| Script | Runs in | What it does |
|---|---|---|
| `rvlib.py` | Blender | Shared helpers: which objects are solid parts, closed-mesh inside test, penetration depth between two meshes |
| `census.py` | Blender | Every mesh: open edges, number of pieces, volume, size. `ARGS: {"out": ...}` |
| `screws.py` | Blender | Every screw, nut and bearing stand-in: count and size |
| `static.py` | Blender | All neighbouring pairs of parts at rest: do their surfaces cross, and how deep. `ARGS: {"lib": rvlib.py, "out": ...}` |
| `sweep.py` | Blender | Poses the robot and reports colliding pairs per pose. `ARGS: {"lib", "poses": [{"label", "q": {joint: deg}}], "out"}` |
| `post.py` | Python | Removes intended contacts (threads, press fits) from sweep results and compares with the design's hip map |
| `assy.py`, `assy2.py` | Blender | Slides each servo into its holder and each part onto its horn to find a clear path; `assy2.py` re-tests steps in the documented build order |
| `motion.py` | Blender | Driver for the design repository's own `leg_motion_check.py` |
| `safety.py` | Blender | Clearances round the battery and regulator, sharp-edge length, overall size |
| `keys.py`, `single_support.py` | Python + MuJoCo | Model mass, limits and keyframes; static joint loads when standing on one foot |

Collision test used throughout: two parts collide when their triangle meshes cross and a vertex or triangle centre of
one lies more than 0.05 mm inside the other. Pairs that already touch or overlap at rest by design are subtracted.
