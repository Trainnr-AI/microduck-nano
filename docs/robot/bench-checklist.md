# Bench checklist

The first steps with real servos, the IMU and power. Paths are relative to the `firmware/` folder unless they start with `firmware/`.


Do the steps in order. Keep fingers clear, keep children away, and have the battery switch within reach whenever servos
are powered.

**A. Board only (no servos, no battery)**

1. Flash the default build ([build-and-flash.md](build-and-flash.md)). The log shows `DISARMED`, `motion_build: 0`.
2. If you can, update the board (Wi-Fi, `arduino-app-cli system update`), flash again, and confirm `duck_info` now
   reports `servo_uart_is_serial1: 1`.
3. With the IMU on Qwiic: set `DUCK_IMU_BNO08X 1`, add `- SparkFun BNO08x Cortex Based IMU (1.0.6)` to `sketch.yaml`
   (needs a network on the board), flash. `duck_info` should report the IMU present. Tilt the IMU by hand and note which
   sensor axis reads "up"; write the `sensor_to_body` matrix in `robot.json` so that a level robot gives pitch 0 / roll 0,
   nose up gives positive pitch and left side up gives positive roll; set `imu.calibrated` true. Confirm in `duck_status`.
   The matrix must be a rotation (`gen_config.py` refuses a mirror). Then the gyro, from `duck_state`'s last three
   words (degrees per second x10, body frame): turning the robot to its left reads positive z, nose down positive y,
   right side down positive x, each near zero at rest.
4. With a logic analyser or scope on D1: power-cycle the board and record what the pin does during boot. If anything is
   sent, the servo supply must come up only after the board has booted (or the servo UART has to move).

**B. One servo on the bench, nothing attached to its horn, default build (cannot move it)**

5. Power the adapter and one servo from a current-limited 5 V supply. Does the adapter work at 5 V at all?
6. `duck_scan` finds the servo (factory ID is usually 1). If every read fails, try `duck_set_echo(true)`.
7. `duck_read(id, 56, 2)` while turning the horn by hand to two known angles 90 deg apart: confirm 3.41 steps per degree
   and which way the numbers go.
8. Read registers 9..12 (angle limits), 13, 19, 37, 38, 39 and compare with the factory values in the register tables in [the firmware design](../design/firmware.md)
   (20 / 1003, 70, -, 20, 200, 80). This is the check of the unconfirmed addresses.

**C. Provisioning, one servo at a time (provisioning build: `DUCK_ENABLE_EEPROM_WRITE 1`)**

9. Decide the 15 servo IDs; set each servo's ID with only that servo on the bus; label it.
10. Set every servo to mid-range before fitting its horn (ASSEMBLY.md step 0.3), then fill in `zero_step`, `direction`,
    `calibrated` in `robot.json` from readings at the joint's zero pose.
11. Write and read back the angle limits and the protection settings on every servo (JOINT_LIMITS.md sections 1 and 3).
    Stall one spare servo by hand against its limit and confirm that it lets go by itself.
12. Go back to a build with `DUCK_ENABLE_EEPROM_WRITE 0`.

**D. First motion (motion build: `DUCK_ENABLE_MOTION 1`), robot held in a stand or by a second person, feet off the floor**

13. One leg unplugged from the others if possible. Arm. Nothing should move. Disarm; torque goes.
14. Move one joint at a time through small targets; watch `duck_setpoints` against `duck_joints` and the sign of each
    joint. Check goal speed 0 behaves as "fast" and the rate limit feels sane; lower `max_speed_dps` if not.
15. Drive each joint to both limits slowly and look for contact before the limit. Try the head roll / yaw interlock and
    the hip map edges by hand-commanded targets, watching the parts, not the screen.
16. Supported stand (`duck_stand(true)`), then tilt the robot by hand: both ankles should correct against the tilt, and
    past 30 deg the fault should cut torque.

**E. On the floor, soft surface, hands ready**

17. Only after the planned-pitch curve has been exported from the simulation: stand, then sit and stand with a hand
    under the trunk. Watch loads in `duck_health`.
