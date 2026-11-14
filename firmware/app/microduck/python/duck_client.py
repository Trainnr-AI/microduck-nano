# SPDX-License-Identifier: Apache-2.0
"""Thin Python client for the Microduck Nano microcontroller firmware.

Every method is one Bridge call to the sketch. Nothing here moves the robot
unless the firmware was built with motion enabled AND arm() succeeded; the
limits, interlocks and fault handling all live on the microcontroller.

The Bridge link is a 115200 baud serial line with messages of at most 256
bytes, so keep the call rate modest (tens of calls per second, not hundreds).
"""

ARM_KEY = 0xD0C5A4B3

JOINTS = ["left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
          "neck_pitch", "head_pitch", "head_yaw", "head_roll",
          "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle", "beak"]

MODES = ["DISARMED", "ARMED", "FAULT"]
POSTURES = ["UNKNOWN", "STANDING", "SITTING", "MOVING_TO_SIT", "MOVING_TO_STAND", "RETURNING_TO_STAND"]
FAULTS = ["none", "overload", "over-temperature", "under-voltage", "servo status", "servo not answering",
          "fall", "bus write"]
RESULTS = ["ok", "firmware built without motion", "bad key", "joint table not calibrated", "servo missing",
           "IMU required", "fault latched", "not armed", "already armed", "wrong posture",
           "not at the STAND keyframe (call stand() first)", "facing uphill: sit refused", "busy", "bus error"]
BUS_ERRORS = ["ok", "no reply", "bad reply", "wrong id", "refused by the safety gate", "bad argument"]
FLAGS = {0: "non-finite target dropped", 1: "joint limit clamp", 2: "rate limit", 3: "head interlock",
         4: "hip map (left)", 5: "hip map (right)", 8: "command timeout: holding", 9: "leg targets ignored",
         10: "IMU stale", 11: "supported stand"}


def _name(table, code):
    return table[code] if 0 <= code < len(table) else f"code {code}"


class Duck:
    def __init__(self, bridge):
        self._b = bridge

    # ---- read-only ----
    def info(self):
        v = self._b.call("duck_info")
        keys = ["protocol", "motion_build", "eeprom_build", "joint_table_ready", "imu_present", "servo_uart_is_serial1"]
        return dict(zip(keys, v))

    def status(self):
        v = self._b.call("duck_status")
        return {
            "mode": _name(MODES, v[0]), "posture": _name(POSTURES, v[1]), "fault": _name(FAULTS, v[2]),
            "fault_joint": JOINTS[v[3]] if 0 <= v[3] < len(JOINTS) else None,
            "flags": [text for bit, text in FLAGS.items() if v[4] & (1 << bit)],
            "imu_ok": bool(v[5]), "pitch_deg": v[6] / 10, "roll_deg": v[7] / 10, "balance_deg": v[8] / 10,
            "worst_tick_us": v[9],
        }

    def joints(self):
        """Measured joint angles in degrees (None where the servo did not answer)."""
        return {n: (None if x == -32768 else x / 10) for n, x in zip(JOINTS, self._b.call("duck_joints"))}

    def state(self):
        """Everything a control tick needs in one call (firmware protocol 3)."""
        v = self._b.call("duck_state")
        return self._state(v)

    def step(self, degrees):
        """One control tick in one call: 15 targets in degrees out, the state back. Ignored unless armed."""
        return self._state(self._b.call("duck_step", [int(round(float(x) * 10)) for x in degrees]))

    @staticmethod
    def _state(v):
        gyro = v[10 + len(JOINTS):]  # empty from a protocol 2 firmware
        return {
            "result": _name(RESULTS, v[0]), "mode": _name(MODES, v[1]), "posture": _name(POSTURES, v[2]),
            "fault": _name(FAULTS, v[3]), "imu_ok": bool(v[6]), "pitch_deg": v[7] / 10, "roll_deg": v[8] / 10,
            "joints": {n: (None if x == -32768 else x / 10) for n, x in zip(JOINTS, v[10:10 + len(JOINTS)])},
            # body frame (x forward, y left, z up), deg/s; None when the gyro is not measured
            "gyro_dps": (None if len(gyro) != 3 or -32768 in gyro else [x / 10 for x in gyro]),
        }

    def setpoints(self):
        return {n: x / 10 for n, x in zip(JOINTS, self._b.call("duck_setpoints"))}

    def health(self):
        v = self._b.call("duck_health")
        out = {}
        for i, n in enumerate(JOINTS):
            load, volt, temp = v[i * 3:i * 3 + 3]
            out[n] = None if load == -32768 else {"load_pct": load / 10, "voltage": volt / 10, "temperature_c": temp}
        return out

    # ---- bench reads: talk to servos but cannot move them ----
    def say(self, text):
        """Put text on the board's LED matrix for a few seconds (up to 32 characters; firmware protocol 3)."""
        return self._b.call("duck_say", str(text))

    def ping(self, servo_id):
        return _name(BUS_ERRORS, self._b.call("duck_ping", int(servo_id)))

    def scan(self, first=0, last=253):
        found = []
        for start in range(first, last + 1, 32):
            v = self._b.call("duck_scan", start, min(start + 31, last))
            found += v[1:1 + v[0]]
        return found

    def read_register(self, servo_id, address, length=1):
        v = self._b.call("duck_read", int(servo_id), int(address), int(length))
        if v[0] != 0:
            raise IOError(_name(BUS_ERRORS, v[0]))
        return v[2:2 + v[1]]

    def set_echo(self, on):
        return self._b.call("duck_set_echo", bool(on))

    # ---- commands ----
    def heartbeat(self):
        return _name(MODES, self._b.call("duck_heartbeat"))

    def arm(self):
        return _name(RESULTS, self._b.call("duck_arm", ARM_KEY))

    def disarm(self):
        self._b.call("duck_disarm")

    def clear_fault(self):
        return _name(RESULTS, self._b.call("duck_clear_fault"))

    def stand(self, supported=False):
        return _name(RESULTS, self._b.call("duck_stand", bool(supported)))

    def sit(self):
        return _name(RESULTS, self._b.call("duck_sit"))

    def set_targets(self, degrees):
        """degrees: dict joint name -> angle, or a list of 15 in JOINTS order. Missing joints keep their target."""
        if isinstance(degrees, dict):
            degrees = [float(degrees.get(n, float("nan"))) for n in JOINTS]
        if len(degrees) != len(JOINTS):
            raise ValueError(f"need {len(JOINTS)} joint targets")
        return _name(RESULTS, self._b.call("duck_set_targets", [float(x) for x in degrees]))

    def set_head(self, neck_pitch, head_pitch, head_yaw, head_roll, beak):
        return _name(RESULTS, self._b.call("duck_set_head", float(neck_pitch), float(head_pitch), float(head_yaw),
                                           float(head_roll), float(beak)))
