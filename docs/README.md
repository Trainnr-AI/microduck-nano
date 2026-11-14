# Documentation

Laid out like Pollen Robotics' Microduck docs: pages for people who have a robot, pages that explain how things
work, and dated project records.

## `robot/`: building and running one

| Page | What |
|---|---|
| [build-and-flash.md](robot/build-and-flash.md) | Reaching the Arduino UNO Q, building and flashing the firmware, the wiring it assumes, things that went wrong |
| [bench-checklist.md](robot/bench-checklist.md) | The first steps with real servos, the IMU and power. None of it has been done yet |

## `design/`: how it works

| Page | What |
|---|---|
| [firmware.md](design/firmware.md) | Which processor does what, the safety model, the functions between the two sides, servo registers |

## `project/`: dated records

These describe the state on their date and are expected to go stale.

| Page | What |
|---|---|
| [roadmap.md](project/roadmap.md) | What works today, what is designed, what comes next |
| [design-review-2026-11-14.md](project/design-review-2026-11-14.md) | Independent review of the mechanical design: PASS / FAIL per check, safety findings |
| [uno-q-compute.md](project/uno-q-compute.md) | What the UNO Q can run: policies, vision, language models, CPU against GPU |
| [rigrs-for-general-use.md](project/rigrs-for-general-use.md) | What Trainnr's Rust firmware framework lacks for a robot it was not written for, and how the Nano will use it |
| [prior-art.md](project/prior-art.md) | What Pollen's Microduck and Luwu's XGO-Duck are built from and working on, and what we take from them |
