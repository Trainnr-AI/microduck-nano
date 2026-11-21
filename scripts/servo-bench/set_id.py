#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""ON THE BOARD: give the one servo on the bus a new ID, and read it back.

    python3 set_id.py <new id> [old id, default 1]

A new SCS0009 answers as ID 1, so the servos are numbered one at a time, each alone on the bus. This script writes
nothing unless exactly one servo answers in the whole ID range, it is the old ID, and sixty reads of it in a row
come back clean (two servos sharing an ID talk over each other, and reads fail). The new ID is stored in the servo's
own memory and survives power-off. Mark the servo straight away.

Needs a provisioning build of the firmware (DUCK_ENABLE_EEPROM_WRITE 1), nothing armed, and router_call.py beside
this file. Go back to a build with DUCK_ENABLE_EEPROM_WRITE 0 when the last servo is numbered.
"""
import sys

import router_call

ARM_KEY = 0xD0C5A4B3
REG_ID, REG_MIN_ANGLE, REG_MAX_ANGLE, REG_TORQUE, REG_LOCK, REG_POSITION, REG_VOLTAGE = 5, 9, 11, 40, 48, 56, 62

if len(sys.argv) < 2:
    sys.exit(__doc__)
NEW = int(sys.argv[1])
OLD = int(sys.argv[2]) if len(sys.argv) > 2 else 1
if not (0 <= NEW <= 253 and 0 <= OLD <= 253) or NEW == OLD:
    sys.exit("IDs are 0..253 and must differ")


def call(method, *args, timeout=3.0):
    reply = router_call.call(method, *args, timeout=timeout)
    if reply[2] is not None:
        raise SystemExit(f"{method} refused: {reply[2]}")
    return reply[3]


def read(servo, address, count):
    """One register read: the value (big-endian for two bytes), or None when the servo did not answer cleanly."""
    words = call("duck_read", servo, address, count)
    if words[0] != 0:
        return None
    return int.from_bytes(bytes(words[2:2 + count]), "big")


def scan():
    found = []
    for first in range(0, 254, 32):
        words = call("duck_scan", first, first + 31, timeout=5.0)
        found += words[1:1 + words[0]]
    return found


found = scan()
print(f"servos answering: {found}")
if found != [OLD]:
    sys.exit(f"expected exactly one servo, at ID {OLD}: nothing written")
positions = [read(OLD, REG_POSITION, 2) for _ in range(60)]
failed = positions.count(None)
print(f"60 position reads: {failed} failed, values {sorted(set(p for p in positions if p is not None))[:8]}; "
      f"limits {read(OLD, REG_MIN_ANGLE, 2)}..{read(OLD, REG_MAX_ANGLE, 2)}, "
      f"lock {read(OLD, REG_LOCK, 1)}, {read(OLD, REG_VOLTAGE, 1) / 10:.1f} V")
if failed:
    sys.exit("reads not clean (two servos sharing this ID?): nothing written")
if not call("duck_info")[2]:
    sys.exit("the servo is ready, but this firmware build cannot write servo memory (DUCK_ENABLE_EEPROM_WRITE 0): "
             "nothing written")

result = call("duck_set_id", ARM_KEY, OLD, NEW)
print(f"duck_set_id({OLD} -> {NEW}): {'ok' if result == 0 else f'failed, code {result}'}")
found = scan()
print(f"servos answering now: {found}")
print(f"ID {NEW}: ID register {read(NEW, REG_ID, 1)}, lock {read(NEW, REG_LOCK, 1)} (1 = memory locked again), "
      f"position {read(NEW, REG_POSITION, 2)}, torque {read(NEW, REG_TORQUE, 1)}")
if result != 0 or found != [NEW] or read(NEW, REG_ID, 1) != NEW or read(NEW, REG_LOCK, 1) != 1:
    sys.exit("NOT as expected: read the lines above before doing anything else")
print(f"done: mark this servo {NEW}")
