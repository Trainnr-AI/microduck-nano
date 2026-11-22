# microduck-nano

An open build of **Microduck Nano**, a smaller variant of the Microduck walking robot: everything needed to print, wire, simulate, train and run one, kept in one repository so anyone can reproduce it.

Microduck is a small duck biped by [Pollen Robotics](https://github.com/pollen-robotics/microduck_rl). [XGO-Duck](https://github.com/LuwuDynamics/xgoduck_hardware) by Luwu Dynamics adapts it for Feetech 1910 servos and an Arduino UNO Q. Microduck Nano is XGO-Duck redesigned around 9 g hobby-size bus servos. This repository follows the build from parts to a walking policy on the real robot, using [trainnr](https://github.com/Trainnr-AI/trainnr) for the sim, training and deployment work.

> **Status: design closed for a first build (2026-11-18), nothing printed.** The robot exists as CAD and a simulation
> model. An [independent review](docs/project/design-review-2026-11-14.md) found problems; the design's answers to
> them (new limits, a regulator window, no battery protection electronics by the owner's choice) have not been
> reviewed again, and it is **not yet safe to build for children**. Firmware runs on an Arduino UNO Q with the IMU
> and holds the closed design's joint limits and leg rules; motion is locked off by default, and so far only one
> or two servos at a time have moved, on a bench. All fifteen servos have their IDs. A first walk has been
> trained in simulation on a servo model nobody has measured yet: it stays on its feet and does not yet follow the
> speed it is asked for. No trained policy has run on the robot.

## The robot

| | |
|---|---|
| Servos | 15 × Feetech SCS0009 (serial bus, 9 g): 5 per leg, 4 in the neck and head, 1 for the mouth |
| Size | Uniform 0.92 × XGO-Duck, about 240 mm tall; head shell a further 0.85 × |
| Motion | Same joint axes, directions and ranges as XGO-Duck, except head yaw (±150° instead of ±170°, the servo's travel) |
| Brain | Arduino UNO Q |
| Joints | Printed parts with a bearing at every joint; servos held by their two ears |

The SCS0009 is roughly four times weaker than the servo it replaces, so the design trades size and head mass for torque headroom. Whether it walks is the open question this build answers.

Design work happens in [agent-cad3d](https://github.com/Trainnr-AI/agent-cad3d) (Blender, driven by a coding agent); finished parts and the model are published here.

## What is here

The layout follows Pollen Robotics' Microduck repositories.

| Path | What | State |
|---|---|---|
| [`firmware/`](firmware/) | What runs on the Arduino UNO Q: servo bus, IMU, limits, interlocks, sit/stand, with host tests | Runs on the board with the IMU; servos moved one or two at a time in a bench build, never the whole robot |
| [`runtime/`](runtime/) | The Linux-side runtime in Rust, on rigrs: talks to the firmware, runs the control loop | Dry-run loop (nothing here arms the robot) and a supervised one-servo bench mode; measured on the board, never with a servo |
| [`docs/`](docs/README.md) | `robot/` (build and run one), `design/` (how it works), `project/` (roadmap, reviews, measurements) | Start at the index |
| [`scripts/`](scripts/) | Board benchmarks and the scripts behind each design review | As run |
| `hardware/` | Bill of materials, print files, wiring, assembly guide | Not here yet (see the licence note below) |
| `robot/` | The robot's simulation model | Not here yet |
| `training/` | Task definitions, training configs, records of each run | Not started |

Good first reads: the [roadmap](docs/project/roadmap.md), the [design review](docs/project/design-review-2026-11-14.md),
[what the board can run](docs/project/uno-q-compute.md), and [what Microduck and XGO-Duck do](docs/project/prior-art.md).

## How the robot is trained

All sim-side work goes through trainnr, driven by a coding agent over trainnr's MCP server and inspected in trainnr Studio. The loop, in order:

1. **Onboard** the robot's model
2. **Record** telemetry from the real robot
3. **Identify** its actuators and dynamics from that telemetry
4. **Define** the walking task
5. **Train** a policy in simulation
6. **Evaluate** it, with pushes and delays
7. **Export** it, pass the deployment gate, run pre-flight
8. **Monitor** drift once it is on the robot

Each step leaves a stamped record, so a result here can be traced back to the model, data and settings that produced it.

### Set up trainnr

```sh
git clone https://github.com/Trainnr-AI/trainnr && cd trainnr
cd trainnr && uv sync --extra sim --extra mcp --extra deploy --extra viz && cd ..
```

Then register the MCP server with your agent (from the trainnr checkout):

```sh
uv run --directory trainnr --extra sim --extra mcp trainnr mcp
```

Training needs an NVIDIA GPU with CUDA (`trainnr-mjlab`, about 6 GB). See trainnr's README for the Studio.

## When trainnr is missing something

This build doubles as a test of trainnr on a real robot. When a step needs a feature trainnr lacks, or hits a bug, the fix goes upstream as a pull request to [Trainnr-AI/trainnr](https://github.com/Trainnr-AI/trainnr), not as a workaround here. Open gaps are tracked as issues in this repository, each linked to its upstream issue or pull request.

## Credits and licence

Microduck and `microduck_rl` are by Pollen Robotics. Their code is Apache-2.0; their **3D model files are Creative Commons BY-SA-NC**, which forbids commercial use and requires derivatives to carry the same terms. XGO-Duck and `xgoduck_rl` are by Luwu Dynamics, built on Pollen's work. The SCS0009 servo model comes from Feetech's published CAD.

This repository uses two licences:

- **Code, configs and documentation:** [Apache-2.0](LICENSE).
- **Hardware files** (CAD, meshes, print files, and robot models that embed them), everything under `hardware/` and the meshes under `robot/`: [CC BY-NC-SA 4.0](LICENSES/CC-BY-NC-SA-4.0.txt), inherited from Pollen's models; scope in [LICENSE-MODELS.md](LICENSE-MODELS.md). You may build, modify and share the robot, with attribution and under the same terms, but not sell it.

XGO-Duck's hardware files carry no licence of their own yet; its maintainers state an intent to allow personal and educational replication. Parts here that derive from XGO-Duck's shells are published only once that permission is confirmed, and are marked as such.
