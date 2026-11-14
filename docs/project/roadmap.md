# Roadmap

Status on 2026-11-15. Ordered by what blocks what, not by date.

## Works today

- **Firmware on a bare Arduino UNO Q.** Builds, passes 19 host tests, runs on the board on both the shipped and the
  updated board software. The Linux side and the microcontroller talk; arming is refused. No servo has been driven.
- **Runtime in Rust on rigrs** ([`runtime/`](../../runtime/)). Reads the firmware's state through the board's router
  and runs a dry-run control loop with rigrs guards. Measured on the board: a tick's exchange with the microcontroller took 18 ms
  as two reads at the default link speed and takes about 6 ms as one exchange at 460800 baud.
- **Policy runner** in the runtime: manifest-checked ONNX policies, tested with a hand-made network. No trained
  policy exists.
- **Simulation model.** The design's MuJoCo model (`scene_scs0009_v2.xml`) is onboarded in trainnr as
  `microduck-nano-v2`. It stands, crouches, and sits and stands in simulation.
- **Measurements of the board.** [uno-q-compute.md](uno-q-compute.md).

## Designed, not built

- **The robot.** CAD exists in the design repository; nothing is printed. The
  [design review](design-review-2026-11-14.md) lists what must change first.
- **Bench bring-up.** [The checklist](../robot/bench-checklist.md) is written; no step has been run.

## Next, in order

1. **Fix what the review found**, in the design repository: the head interlock collision, the right-leg hip map,
   battery clearances and protection, regulator placement. Then re-run the review.
2. **One servo on the bench.** Confirm steps per degree, the protection registers and the adapter at 5 V; record
   telemetry and fit an actuator model for the SCS0009 (trainnr `identify_system`). Nothing trained before this is
   trustworthy.
3. **Feed the policy runner.** The Rust runtime can load a policy, check it against the robot and run it in the
   loop (0.48 ms per inference on the board; about 7 ms per tick with the exchange). The gyro is in the
   firmware's state message (protocol 3, running on the board with no IMU attached); it still needs a command source and above all a
   trained policy. The microcontroller firmware stays C++ until the robot walks.
4. **Tolerance coupon, one leg, then a full print.**
5. **Train walking** in trainnr on the fitted actuator model (waits on a trainnr walk family for any robot:
   [Trainnr-AI/trainnr#100](https://github.com/Trainnr-AI/trainnr/issues/100)), with backlash modelled; evaluate with pushes and delays;
   pass the deployment gate. Then sit/stand, then get-up, which may need stronger servos.
6. **Publish the hardware files** once permission for the XGO-Duck-derived shells is confirmed
   (`LICENSE-MODELS.md`).

## Open questions

- Can the SCS0009 walk this robot at all? Static single-support load is 74-77 % of stall on the stance hip roll.
- Battery: a protected pack and a fuse, or a different chemistry.
- A camera pipeline and one small vision model on two cores: measured only with random inputs so far.
