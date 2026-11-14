#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# Change the speed of the link between the UNO Q's Linux side and its microcontroller. Run ON the board, with sudo:
#
#   sudo sh set-link-speed.sh 2000000     # faster link
#   sudo sh set-link-speed.sh default     # back to the board's own setting (115200)
#
# It adds (or removes) one override file for the router service and restarts it; nothing else on the board is
# changed. Restarting the router also restarts the microcontroller.
#
# The sketch must be built with the same speed (DUCK_BRIDGE_BAUD in duck_build_config.h), or its functions never
# appear. This is board-wide: every other sketch, including Arduino App Lab's examples, expects 115200 and will not
# talk to Linux until the default is restored.
set -eu
SPEED=${1:?usage: set-link-speed.sh <baud>|default}
DIR=/etc/systemd/system/arduino-router.service.d
FILE=$DIR/90-microduck-link-speed.conf

if [ "$SPEED" = default ]; then
  rm -f "$FILE"
else
  case "$SPEED" in *[!0-9]*|'') echo "speed must be a number or 'default'"; exit 2;; esac
  # The command the service runs now, with our own override taken out of the picture.
  rm -f "$FILE"
  systemctl daemon-reload
  CURRENT=$(systemctl cat arduino-router | grep '^ExecStart=/' | tail -1)
  case "$CURRENT" in
    *--serial-baudrate*) ;;
    *) echo "the router service has no --serial-baudrate option to change: $CURRENT"; exit 1;;
  esac
  NEW=$(printf '%s\n' "$CURRENT" | sed -E "s/--serial-baudrate +[0-9]+/--serial-baudrate $SPEED/")
  mkdir -p "$DIR"
  printf '# Written by microduck-nano firmware/tools/set-link-speed.sh. Delete this file to restore the default.\n[Service]\nExecStart=\n%s\n' "$NEW" > "$FILE"
fi
systemctl daemon-reload
systemctl restart arduino-router
systemctl cat arduino-router | grep '^ExecStart=/' | tail -1
