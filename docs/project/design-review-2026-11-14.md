# Independent design review, 2026-11-14

A review of the Microduck Nano design before any hardware was built, done by re-running the design's own checks and
by independent checks written for the review. Design state reviewed: the design repository's `main` at commit `91b780c`
(Blender model `xgoduck_scs0009.blend`, simulation `scene_scs0009_v2.xml`).

**Conclusion: not ready to build for children.** The geometry checks mostly reproduce the designers' numbers, but the
review found three collision errors in the documented limits, the robot cannot balance on one foot, and the battery
and power design has no protection.

The scripts are in [`scripts/design-review/2026-11-14/`](../../scripts/design-review/2026-11-14/). The argument lists for the design repository's
`fit_audit` and `assembly_path_check` are not stored there, so for those two the review used its own equivalent checks.

## Results

| Check | Verdict | Measured | Design's own review says |
|---|---|---|---|
| Printed parts are one watertight piece | PASS | 29 robot parts, the trim jig and 5 coupon pieces: 0 open edges, 1 piece each; 311.0 g solid PLA | 27/27 (the two optional thigh covers excluded), 311 g |
| Printability (overhang area per part) | PASS | Identical: feet 436 / 438 mm², head base 1719, dome 1937, thigh 678, shin 523, face 466, lid 594 | same |
| Fits at rest (all 696 neighbouring pairs) | PASS | No printed part cuts another by more than 0.02 mm; worst servo-to-print overlap 0.06 mm (neck cradle); 15/15 horn pockets at 0.25 mm crush | 333 fits, none wrong |
| Screws | PASS on count | 89 screws, 27 flagged, none new | 89, 27 flagged |
| Assembly paths (42 steps) | PASS | Every servo, moving part, bearing plate and cap has a clear path in the documented order; three need an L-shaped path (left hip-yaw servo, yaw block, beak servo) | all OK |
| Single joints over their servo limits | PASS | 671 poses at 2.5° steps: 0 collisions | 0 |
| STAND, SIT, sit path, hip yaw/roll box, neck × head pitch | PASS | 153 poses: 0 collisions | 0 |
| Head yaw × roll interlock | **FAIL** | Inside the allowed zone, at yaw −2 to −2.5° with roll −10 to −15°, the head base cuts 0.16–0.40 mm into the head-pitch U; clear at ±1° yaw | roll allowed within ±2.5° of yaw 0 |
| Hip map, left leg | PASS with a caveat | All 245 stored collisions reproduced out of 1,248 poses; 7 more poses touch by 0.06–0.48 mm, 3 of them in cells the table allows | 245 |
| Hip map, right leg = mirror of the left | **FAIL** | 30 of 1,248 mirrored poses collide where the left is clear: right leg forward 45–90° with the knee at its 68° limit, the foot meets the neck bearing plate and body frame by up to 6.5 mm | mirror, 40/40 sampled |
| Walking-envelope corners | **FAIL** (documented by the designers) | 8 of 32 left-leg corners collide: knee cap into the enclosure 1.2 mm, foot into the L-bracket 2.1 mm | noted as a known limit |
| Servo loads: stand, crouch, neck, head, sit/stand | PASS | Stand 40 %, crouch 0.3 50 %, neck −57° 62 %, sit/stand at 3 s 52 % of stall | 40, 50, 62, 52 |
| Sit/stand robustness (300 random trials each) | PASS as stated | Open loop 68 pass / 189 fall; with IMU balance 245 / 4; with smoothed balance 254 / 1 | 66 / 192, 245 / 4, 255 / 1 |
| Walking poses | **FAIL** | The scripted one-foot test falls at 1.57 s with three servos at 100 %. Statically, at the hip-roll limit the centre of mass is still 9 mm outside the stance foot, and holding it needs 74–77 % of stall on the stance hip roll | no gait exists; not claimed |
| Bill of materials against the model | PASS | Every bought-part count matches the model, bearings and screws by size too (15 servos, 10 + 4 bearings, 35 M2×6, 3 M2×10, 8 M3×8) | all match |

Smaller findings:

- **Beak and head roll:** the beak sweep to 35° and the head-roll sweep at yaw 0 are clear.
- **Horn screws:** each joint is held by one screw with 1.46–1.87 mm of thread in the servo's spline; the design's
  review says 1.80–1.85.
- **Uno Q mounting:** the M3 screw heads overlap the Uno Q header model by 0.69 mm. Check on the real board.
- **Both legs together:** with both hips rolled in to −12° and the legs forward, the two ankle axles touch (0.2 mm).
- **Stale text in the BOM:** it still says the tolerance coupon is "not designed yet" and that the switch lever is "flush".
- **Duplicate stand-ins:** 14 joints carry two horn-screw stand-ins 1.1 mm apart.

## Not checked

- Screwdriver access and cable reach.
- Clearance margins: the review verified "no collision", not the stated gaps (hip box ≥ 0.5 mm, sit path ≥ 1.58 mm).
- The design's `fit_audit` and `assembly_path_check` exactly as the designers ran them.
- Anything physical: the kit horn and screws, battery thickness, IMU hole spacing, servo steps per degree and the
  protection registers are all unmeasured, and no SCS0009 actuator model exists.
- Walking, getting up after a fall, and fall loads on the joints.

## Unsafe for children as designed

- **Battery:** a bare 3S LiPo with no fuse, no protection circuit and no low-voltage cut-off (the alarm is only
  "suggested"). The charger is one the seller recommends for light use only.
- **Battery clearances:** it sits 0.30 mm from the enclosure wall and 0.62 mm from three steel screw heads, with no
  room to swell.
- **Over-voltage:** if the buck regulator fails short, 12.6 V reaches 15 servos rated for 7.4 V.
- **Hot parts:** the regulator sits 0.2 mm from PLA walls and 3.6 mm from the LiPo, in a closed bay. Stalled servos
  are limited only by a 70 °C setting that has not been confirmed on a real servo.
- **Pinch points:** the head, hips and knees can be driven into the body. The collisions above are prevented only by
  firmware limits.
- **Small parts:** M2 screws, M3 nuts (6 mm) and 8 mm bearings are a choking hazard, and each limb is held by one
  short screw with no retention.
- **Sharp edges and protrusions:** the parts are flat plates with unbroken 90° edges, and the toggle lever stands
  4–8 mm proud.
- **Falls:** it cannot get up, and without IMU balance it falls during sit/stand in about two thirds of trials.

## What the firmware does about it

The firmware's limits file (`firmware/config/robot.json`) carries two of these findings until the parts are fixed and
re-measured: head roll is allowed only within ±1.0° of head yaw 0, and the 37 extra colliding hip poses are blocked for
both legs. See [the firmware design](../design/firmware.md).
