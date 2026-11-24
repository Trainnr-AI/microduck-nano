# Roadmap

Status on 2026-11-18. Ordered by what blocks what, not by date.

## Works today

- **Firmware on a bare Arduino UNO Q.** Builds, passes 26 host tests, runs on the board on both the shipped and the
  updated board software. The Linux side and the microcontroller talk; arming is refused. No servo has been driven.
- **Runtime in Rust on rigrs** ([`runtime/`](../../runtime/)). Reads the firmware's state through the board's router
  and runs a dry-run control loop with rigrs guards. Measured on the board: a tick's exchange with the microcontroller took 18 ms
  as two reads at the default link speed and takes about 6 ms as one exchange at 460800 baud.
- **Policy runner** in the runtime: manifest-checked ONNX policies, tested with a hand-made network. No trained
  policy exists.
- **Simulation model.** The design's MuJoCo model (`scene_scs0009_v2.xml`) is onboarded in trainnr (the closed design of
  2026-11-18 as `microduck-nano-2026-11-18`). It stands, crouches, and sits and stands in simulation.
- **Measurements of the board.** [uno-q-compute.md](uno-q-compute.md).

## Designed, not built

- **The robot.** CAD exists in the design repository, closed for a first build on 2026-11-18; nothing is printed.
  The design's answers to the [design review](design-review-2026-11-14.md) have not been reviewed again.
- **Bench bring-up.** [The checklist](../robot/bench-checklist.md) is written; no step has been run.

## Next, in order

1. ~~Fix what the review found~~ Done in the design repository and closed on 2026-11-18; the firmware holds the
   new limits and leg rules (2026-11-18). Open: an independent look at the closed design (battery without protection
   electronics, the regulator's window), and the poses the design did not measure
   ([firmware.md](../design/firmware.md)).
2. **One servo on the bench.** Confirm steps per degree, the protection registers and the adapter at 5 V; record
   telemetry (`duckd run --record`) and fit an actuator model for the SCS0009. Nothing trained before this is
   trustworthy. A bench build (`DUCK_BENCH_MOTION 1`, off by default) and `duckd bench`
   move and record one servo on its own (first recordings 2026-11-23, bare horn). trainnr fits a servo model from
   such a recording since 2026-11-22; what stands in the way is a pendulum of known mass and length on the horn
   ([scripts/servo-bench](../../scripts/servo-bench/README.md)).
3. **Feed the policy runner.** It reads trainnr's exports (`trainnr-deploy/1`) as they are; the pipeline from
   trainnr to `duckd` is checked end to end on Pollen's microduck ([sim-to-real.md](../design/sim-to-real.md)). The Rust runtime can load a policy, check it against the robot and run it in the
   loop (0.48 ms per inference on the board; about 7 ms per tick with the exchange). The gyro is in the
   firmware's state message (protocol 3, running on the board with no IMU attached); it still needs a command source and above all a
   trained policy. The microcontroller firmware stays C++ until the robot walks.
4. **Tolerance coupon, one leg, then a full print.**
5. **Train walking** in trainnr on the fitted actuator model (waits on a trainnr walk family for any robot:
   [Trainnr-AI/trainnr#100](https://github.com/Trainnr-AI/trainnr/issues/100)), with backlash modelled; evaluate with pushes and delays;
   pass the deployment gate. Then sit/stand, then get-up, which may need stronger servos.
6. **Publish the hardware files** once permission for the XGO-Duck-derived shells is confirmed
   (`LICENSE-MODELS.md`).

## The pet: what it should do (direction set 2026-11-23)

With the camera as its only sense of the world (a microphone and a speaker come with the next version): look at
people and things, walk fast to a toy and catch it, sit and stand, throw a tantrum, struggle when picked up. In
the order things can be had:

| What | Needs | State |
|---|---|---|
| Find a coloured toy: direction and distance | the camera | `duckd see` runs on the board; no toy tracked yet |
| Look at it with the head | the head's four servos | one servo on the bench does it (`duckd pet`, not yet run) |
| Feel a hand: struggle when held, give up, calm down | nothing but the servos | the same, for one joint |
| Being picked up: struggle in the air | the assembled robot; the firmware must tell held from falling (today a tilt past 30 degrees switches everything off) | not started |
| Walk to the toy, fast | a walk that follows its command ([sim-to-real.md](../design/sim-to-real.md)); the SCS0009's measured model | three runs, none follows its command. The simulated robot tops out near 0.19 m/s whatever it is asked, and under commands that small the walk's reward pays for standing still ([Trainnr-AI/trainnr#146](https://github.com/Trainnr-AI/trainnr/issues/146)) |
| Look around while walking | a walk trained with the head moved from outside: [Trainnr-AI/trainnr#145](https://github.com/Trainnr-AI/trainnr/issues/145) | asked for upstream |
| Sit, stand, stamp, tantrum | trained motions for our robot: [Trainnr-AI/trainnr#144](https://github.com/Trainnr-AI/trainnr/issues/144) (trainnr tracks motions for one humanoid only) | asked for upstream; the firmware's scripted sit and stand exist, untried on a robot |
| Faces and people | a small detector on real frames (timed on random inputs only: [uno-q-compute.md](uno-q-compute.md)) | not started |

Not possible with one camera: the distance to anything whose size is not known, and edges. The robot stays on
the floor.

## Open questions

- Can the SCS0009 walk this robot at all? Static single-support load is 74-77 % of stall on the stance hip roll.
- Battery: a protected pack and a fuse, or a different chemistry.
- A camera pipeline and one small vision model on two cores: measured only with random inputs so far.
