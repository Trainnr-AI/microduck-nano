# Prior art: Microduck and XGO-Duck

What the two robots Microduck Nano descends from are built from and working on, read from their repositories, product
pages and an independent teardown on 2026-11-15. Neither runs a language or vision-language model on the robot: both
are reinforcement-learning walkers.

## Pollen Robotics Microduck

| | |
|---|---|
| Product | $399 introductory price, 25 cm, 800 g, 15 motors |
| Compute | Radxa Zero 3W (Rockchip RK3566, 0.8 TOPS NPU). Raspberry Pi Zero 2 W builds are community reconstructions of an earlier prototype |
| Servos | 15 x Dynamixel XL330 at 1 Mbps; the policy drives 14, the beak is separate |
| Sensors | IMU on the servo bus (read in the same bus read as the servos), camera (IMX219), 8x8 time-of-flight at 15 Hz, microphone, speaker, NFC tags for modes |
| Power | Sony NP-F550 (2S Li-ion), no protection circuit on the robot's board; voltage read from the servos; sits down and powers off at 6.6 V |
| Runtime | Rust services, 50 Hz control loop, ONNX policies (61 inputs, 14 outputs), signed over-the-air updates with rollback |
| Skills | Walk, sit/stand, kick, pick up with the beak, forward roll, roller skating, get up |
| Training | mjlab (MuJoCo Warp), PPO; the servo's voltage control law and friction modelled (BAM); variants with +-1 deg of gear play |
| Vision | One-class detector (YOLO11n, 320 px, 8-bit) on the NPU, run at 2 Hz because flat out the board reaches 95 degC |
| Autonomy | A state machine on an energy and boredom model with time-of-flight obstacle avoidance; off by default |

Working on (roadmap of 2026-08-26): a channel for community policies, a phone app (design only), remote access,
mapping from the time-of-flight sensor and leg odometry, and the autonomous brain, deliberately last.

Licences: code Apache-2.0; 3D models CC BY-NC-SA 4.0 (`LICENSE-MODELS.md` in `microduck_rl`).

## Luwu Dynamics XGO-Duck

| | |
|---|---|
| Compute | Arduino UNO Q + Luwu's expansion board (QMI8658A 6-axis IMU) |
| Servos | 15 x Feetech 1910 |
| Power | 8.4 V 18650 pack |
| Runtime | Linux: ONNX policy at 50 Hz and a browser UI. STM32: separate 100 Hz servo and IMU threads. Link raised to 2 Mbps, 240-byte state message |
| Safety | Starts with motors unloaded; 1.5 A over-current limit on every servo; a fall cancels the action and triggers get-up |
| Policies shipped | Walk, get-up, pick, forward roll, sit/stand, left and right kick, skating |
| Known issues | Hip-roll motors heat in a toe-out stance; restarting the router service resets the STM32 |
| Licence | None declared for the runtime, its policies or the hardware files |

Working on: reproducibility (confirmed power and servo specifications, print profiles, repeatable actuator
characterisation, documented calibration, independent build records), not new AI.

## What we take from them

- **The policy is the AI.** A small ONNX network at 50 Hz on the CPU, plus at most one tiny detector.
- **Raise the link speed** between the UNO Q's processors, as Luwu does, before running a policy through it.
- **Fit the actuator and model its backlash** before training; both teams get walking from that.
- **Get-up is a learned policy** in both, and needs torque the Nano's servos may not have.
- **Heat limits vision** even with an NPU.

## Sources

[pollen-robotics/microduck](https://github.com/pollen-robotics/microduck) ·
[pollen-robotics/microduck_rl](https://github.com/pollen-robotics/microduck_rl) ·
[Microduck product page](https://pollen-robotics.com/microduck/) ·
[Microduck roadmap](https://github.com/pollen-robotics/microduck/blob/main/docs/project/roadmap.md) ·
[Microduck autonomy design](https://github.com/pollen-robotics/microduck/blob/main/docs/design/autonomous.md) ·
[independent teardown](https://github.com/fanhao375/microduck-replica/blob/master/docs/hardware-teardown.en.md) ·
[LuwuDynamics/xgoduck_hardware](https://github.com/LuwuDynamics/xgoduck_hardware) ·
[LuwuDynamics/xgoduck_runtime_arduino](https://github.com/LuwuDynamics/xgoduck_runtime_arduino)
