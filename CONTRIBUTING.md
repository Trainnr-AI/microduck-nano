# Contributing

Thanks for helping. This project is at an early stage: the robot has not been built yet, so the most useful
contributions right now are reviews, measurements from real parts, and fixes to what is here.

## Where things are

| Path | What | Licence |
|---|---|---|
| `firmware/` | What runs on the Arduino UNO Q (microcontroller sketch + Linux app), its limits file and host tests | Apache-2.0 |
| `runtime/` | The Linux-side runtime in Rust, built on rigrs | Apache-2.0 |
| `scripts/` | Board benchmarks and the scripts behind each design review | Apache-2.0 |
| `docs/` | `robot/` guides, `design/` explanations, dated `project/` records | Apache-2.0 |
| `hardware/`, `robot/` (not here yet) | Print files, assembly, the robot model | CC BY-NC-SA 4.0 (`LICENSE-MODELS.md`) |

## Before you open a pull request

- **Firmware:** run the host tests and the config check, and say that you did.

  ```sh
  python3 firmware/tools/gen_config.py --check
  make -C firmware/tests test
  ```

  For the runtime: `cargo test` and `cargo clippy --all-targets` in `runtime/`, with no warnings.

  Robot-specific numbers live only in `firmware/config/robot.json`; regenerate the header with
  `python3 firmware/tools/gen_config.py` after editing it. Never weaken a safety default (motion build off, arming
  refused with placeholder IDs) in the same change as something else.
- **Numbers:** a measured result comes with the command that produced it and the conditions (board software version,
  model, thread count). Do not round a guess into a fact; mark anything unmeasured as such.
- **Paths and secrets:** no machine-specific paths, network names, passwords or serial numbers in committed files.
- **Hardware files:** anything derived from Pollen Robotics' or Luwu Dynamics' models keeps their attribution and
  stays under CC BY-NC-SA 4.0. Do not add third-party meshes or model weights without their licence allowing it.
- One change per pull request, with a title that says what changed.

## Safety

This robot is meant to be used around children. If you find something that could hurt someone (electrical, thermal,
mechanical, or a firmware path that moves a servo when it should not), open an issue marked **safety** even if you
have no fix.
