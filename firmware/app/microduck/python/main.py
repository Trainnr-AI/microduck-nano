# SPDX-License-Identifier: Apache-2.0
"""Microduck Nano - Linux side of the App.

For now this only watches the microcontroller: it prints what the firmware
reports once a second. It does not arm the robot, does not send motion
commands and does not send the heartbeat. Behaviour, the camera and (later) a
walking policy belong here; use duck_client.Duck for the calls.
"""
import time

from arduino.app_utils import App, Bridge

from duck_client import Duck

duck = Duck(Bridge)
_last = None


def loop():
    global _last
    try:
        line = f"{duck.info()} {duck.status()}"
    except Exception as e:  # the sketch may still be starting, or the Bridge restarting
        line = f"no answer from the microcontroller: {e!r}"
    if line != _last:
        print(line, flush=True)
        _last = line
    time.sleep(1.0)


App.run(user_loop=loop)
