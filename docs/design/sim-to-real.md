# The sim-to-real pipeline

How a walking policy gets from simulation to the robot, through trainnr (used through its MCP tools only) and this
repository's runtime. Status on 2026-11-15.

trainnr project: `microduck-nano` (under trainnr's data folder, not in this repository). The robot is onboarded as
`microduck-nano-v2@a415da90611d`, from the design repository's `sim/scene_scs0009_v2.xml`.

| Stage | trainnr tool | Here | Status |
|---|---|---|---|
| 1. Robot model | `onboard_robot` | | done: 16 bodies, 15 joints, 14 actuators; the names a walk needs (`trunk_base`, `left_foot` / `right_foot` sites, `*_foot_collision`, `imu` with a gyro) |
| 2. Telemetry | `ingest_recording` (MCAP) | `duckd run --record`, `duckd bench --record` | ready; no servo recorded yet (hardware due) |
| 3. Actuator model (SCS0009) | `identify_system` | bench checklist 8b | blocked: trainnr fits no servo model from a recording ([trainnr#101](https://github.com/Trainnr-AI/trainnr/issues/101)) |
| 4. Walk environment | `create_task` | | blocked: walk families are fixed to go1, go2 and Pollen's microduck ([trainnr#100](https://github.com/Trainnr-AI/trainnr/issues/100)) |
| 5. Train | `train_walk` | | needs 3 and 4, and a GPU: on this Mac's CPU a smoke run makes 785 environment steps a second, a full run (8,000 iterations x 4,096 environments x 24 steps) would take about 12 days |
| 6. Evaluate | `evaluate_walk` | | works (exact intervals) |
| 7. Export | `export_deployment` | | works: `policy.onnx` + `deploy.json` (`trainnr-deploy/1`) |
| 8. Sim-to-sim gate | `gate_deployment` (runtime `mujoco`) | | works; reads "passed" for a policy that agrees with its evaluation even when it never walked ([trainnr#108](https://github.com/Trainnr-AI/trainnr/issues/108)) |
| 9. Preflight | `preflight_deployment` | | works: refused the pipeline-check policy for targets 1.5 rad past a joint's range, as it should |
| 10. On the robot | | `duckd run --policy DEPLOYMENT [--command FORWARD,SIDEWAYS,TURN]` | `duckd` reads the export as trainnr wrote it; on the board 0.40 ms per inference. The command is refused outside the ranges the policy was trained on. The firmware's own limits, interlocks and faults stay underneath |
| 11. Drift | `check_drift` | `duckd run --record` | ready once 2 has real data |

## What matches already

The walk trainnr trains (seen on Pollen's microduck, whose joints are named and ordered as ours) observes 51 values:
`base_ang_vel` 3, `projected_gravity` 3, `joint_pos` 14, `joint_vel` 14, `actions` 14, `command` 3, and outputs 14
joint position targets at 50 Hz. `duckd` feeds exactly these: the gyro from the firmware's state message (protocol
3), gravity from the IMU's pitch and roll, joint angles and speeds (by difference) from the servos, the last action,
and the command. The model's 14 actuated joints are the firmware's first 14, in the same order; the beak (15th) is
not the policy's and is held.

## The pipeline check (2026-11-15)

To exercise stages 5 to 10 before our robot can be trained, a deliberately short run was trained on Pollen's
microduck in the `microduck-nano` project: experiment `pipeline-check-pollen-microduck` (50 iterations, 256
environments, CPU), evaluated (evaluation `pipeline-check-pollen-microduck-model_49-cpu-seed1000-n4-pea4574@26067d938ce3`:
it does not walk; it is not a policy), exported as `pipeline-check-export`, gated,
preflighted (refused), and loaded by `duckd` on the board. Nothing about walking follows from it; it shows the
tools connect.

## Not settled

- The sign conventions of the IMU terms against a real BNO086 (bench checklist step 3).
- How trainnr's `deploy.json` describes a BAM-servo robot's gains ([trainnr#107](https://github.com/Trainnr-AI/trainnr/issues/107))
  and its observation conventions ([trainnr#106](https://github.com/Trainnr-AI/trainnr/issues/106)).
- Joint velocity: trainnr's walk observes it; the firmware reports no speed for the robot's loop (the bench mode
  does), so `duckd` differences positions. Read at 6.7 Hz per servo with fifteen fitted, that is coarse.
- A GPU for real training runs (trainnr can train on a rented cloud GPU).
