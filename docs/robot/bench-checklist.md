# Bench checklist

The first steps with real servos, the IMU and power. Paths are relative to the `firmware/` folder unless they start with `firmware/`.


Do the steps in order. Keep fingers clear, keep children away, and have the battery switch within reach whenever servos
are powered.

**A. Board only (no servos, no battery)**

1. Flash the default build ([build-and-flash.md](build-and-flash.md)). The log shows `DISARMED`, `motion_build: 0`.
2. If you can, update the board (Wi-Fi, `arduino-app-cli system update`), flash again, and confirm `duck_info` now
   reports `servo_uart_is_serial1: 1`.
3. With the IMU on Qwiic: set `DUCK_IMU_BNO08X 1`, add `- SparkFun BNO08x Cortex Based IMU (1.0.6)` to `sketch.yaml`
   (needs a network on the board), flash. `duck_info` should report the IMU present (done on the test board,
   2026-11-18: present, 100 reports a second). If it does not, `duck_imu_diag` says whether the sensor answers at
   0x4B or 0x4A and where its start stopped, and `duck_imu_probe` lists what it sends after a reset. Tilt the IMU by
   hand and read `duck_imu_raw`, or watch it with `python3 imu_watch.py` on the board (`tools/imu_watch.py`: up, turning
   rate and angle turned about each axis, 250 readings a second), to see which sensor axis reads "up" (lying flat,
   component side up: +z; x along the breakout's long side, y along its short side; gyro right-handed about all
   three, checked by hand on 2026-11-18); write the `sensor_to_body` matrix in `robot.json` so that a level robot gives pitch 0 / roll 0,
   nose up gives positive pitch and left side up gives positive roll; set `imu.calibrated` true. Confirm in `duck_status`.
   The matrix must be a rotation (`gen_config.py` refuses a mirror). Then the gyro, from `duck_state`'s last three
   words (degrees per second x10, body frame): turning the robot to its left reads positive z, nose down positive y,
   right side down positive x, each near zero at rest.
4. With a logic analyser or scope on D1: power-cycle the board and record what the pin does during boot. If anything is
   sent, the servo supply must come up only after the board has booted (or the servo UART has to move).

**B. One servo on the bench, nothing attached to its horn, default build (cannot move it)**

5. Power the adapter and one servo from a current-limited 5 V supply. Does the adapter work at 5 V at all?
   (2026-11-18: yes through its USB port, jumper on B, at 5.3 V from four AA cells: the servo answers and its whole
   memory table reads, [firmware.md](../design/firmware.md). Through the UART header to the UNO Q, jumper on A,
   nothing came back with RX / TX either way round; `duck_bus_raw` shows what arrives, and a wire from D0 to D1
   shows the board's own port is fine.)
6. `duck_scan` finds the servo (factory ID is usually 1). If every read fails, try `duck_set_echo(true)`.
7. `duck_read(id, 56, 2)` while turning the horn by hand to two known angles 90 deg apart: confirm 3.41 steps per degree
   and which way the numbers go.
8. Read registers 9..12 (angle limits), 13, 19, 37, 38, 39 and compare with the factory values in the register tables in [the firmware design](../design/firmware.md)
   (20 / 1003, 70, 32, 20, 100, 80 as read on 2026-11-18). This is the check of the unconfirmed addresses.
   To watch or record the servo through the runtime, give one joint its ID in `config/robot.json` (for example
   `left_knee`), regenerate the header and flash: disarmed, the firmware reads it every 100 ms, and
   `duckd run --robot robot.json --record bench.mcap` records it for trainnr. It cannot be moved this way: arming
   needs all fifteen servos calibrated.
8b. To measure it moving (for its actuator model), and only with the servo free to turn, a load you have chosen on
   its horn, someone watching and the 5 V supply's switch at hand:
   - calibrate that one joint's entry in `config/robot.json` (ID, `zero_step`, `direction`, `calibrated: true`; keep
     the limits), regenerate the header;
   - build with `DUCK_ENABLE_MOTION 1` and `DUCK_BENCH_MOTION 1`, push, start. LED3 must stay green until armed;
   - `duckd bench --joint left_knee --supervised --excite steps --amplitude 5 --seconds 10 --record steps.mcap`,
     then chirps (`--excite chirp`), then larger amplitudes. LED3 turns magenta while the servo has torque; Ctrl-C,
     a fault or `duckd` stopping switches it off;
   - go back to the default build (both switches 0) and confirm `duck_info` reports motion 0, bench 0.
   The recordings carry the targets asked for, the goals the firmware wrote after its clamp and speed limit (what
   the servo was told: fit against these), and every read of the servo (position, its own speed report), stamped
   when it was read. Check the speed report's unit with them: the servo's speed against the change of its position
   between reads (`duck_state` and the policy take register 58 as steps/s; if it is not, fix `speed_to_dps` in
   `robot_config.cpp` before any policy runs). trainnr cannot yet fit a servo model from them ([Trainnr-AI/trainnr#101](https://github.com/Trainnr-AI/trainnr/issues/101)); keep them.

**C. Provisioning, one servo at a time (provisioning build: `DUCK_ENABLE_EEPROM_WRITE 1`)**

9. Decide the 15 servo IDs; set each servo's ID with only that servo on the bus; label it.
10. Set every servo to mid-range before fitting its horn (ASSEMBLY.md step 0.3), then fill in `zero_step`, `direction`,
    `calibrated` in `robot.json` from readings at the joint's zero pose.
11. Write and read back the angle limits and the protection settings on every servo (JOINT_LIMITS.md sections 1 and 3).
    Stall one spare servo by hand against its limit and confirm that it lets go by itself.
12. Go back to a build with `DUCK_ENABLE_EEPROM_WRITE 0`.

**D. First motion (motion build: `DUCK_ENABLE_MOTION 1`), robot held in a stand or by a second person, feet off the floor**

13. The firmware holds the closed design's limits (2026-11-18): each leg's own hip map with the ankle, the knee x
   ankle rule and the both-legs rule ([firmware.md](../design/firmware.md) lists what is still not measured). If the
   design repository has changed since, run `tools/import_design.py` again first. One leg unplugged from the others if possible. Arm. Nothing should move. Disarm; torque goes.
    With every servo fitted and armed, read `worst firmware tick` in `duck_status` (`duckd status`): armed, the firmware
    reads `servo.reads_per_tick` servos (8) every 10 ms tick, and Linux's calls wait while it does. It should stay
    well under 10 ms; compare `duckd time` armed and disarmed. If it does not, lower `reads_per_tick` (6 still reads
    each joint at about 40 Hz), or look at the servos' return delay (register 7, unconfirmed) before anything else.
14. Move one joint at a time through small targets; watch `duck_setpoints` against `duck_joints` and the sign of each
    joint. Check goal speed 0 behaves as "fast" and the rate limit feels sane; lower `max_speed_dps` if not.
15. Drive each joint to both limits slowly and look for contact before the limit. Try the head roll / yaw interlock and
    the hip map edges by hand-commanded targets, watching the parts, not the screen.
16. Supported stand (`duck_stand(true)`), then tilt the robot by hand: both ankles should correct against the tilt, and
    past 30 deg the fault should cut torque.

**E. On the floor, soft surface, hands ready**

17. Only after the planned-pitch curve has been exported from the simulation: stand, then sit and stand with a hand
    under the trunk. Watch loads in `duck_health`.
