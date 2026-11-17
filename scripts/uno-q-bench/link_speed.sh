#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# Measure the link between the UNO Q's Linux side and its microcontroller at several speeds, and the control loop's
# timing with and without real-time settings. Run ON the board, with sudo, with nothing wired to it:
#
#   sudo sh link_speed.sh                      # 460800, 1000000 and 2000000 baud
#   sudo sh link_speed.sh 460800 2000000       # chosen speeds
#   sudo MATRIX=0 RUN_S=60 sh link_speed.sh 1000000 2000000   # LED matrix off, 60 s runs
#
# Needs, in /home/arduino/duck: duckd, robot.json and set-link-speed.sh; and the app in
# /home/arduino/ArduinoApps/microduck. For each speed it sets the router and the sketch's DUCK_BRIDGE_BAUD to it,
# restarts the app (which rebuilds and flashes the sketch: the default build, which cannot move a servo), then runs
# `duckd time` and `duckd run --exchange step` at 50 and 100 Hz, plain and with real-time settings (CPU governor
# "performance", duckd at SCHED_FIFO 50 on core 3). At the end the board's link speed, sketch and governors are put
# back as they were. MATRIX=0 or 1 builds the sketch with DUCK_STATUS_MATRIX set so (its interrupt every 10 us is the
# first suspect when the link drops bytes); RUN_S sets each run's length (default 20 s). Results go to the terminal and
# to /home/arduino/duck/link-speed-<time>.txt.
set -eu
DUCK=/home/arduino/duck
APP=/home/arduino/ArduinoApps/microduck
CONF=$APP/sketch/duck_build_config.h
OUT=$DUCK/link-speed-$(date +%Y%m%d-%H%M%S).txt
SPEEDS=${*:-460800 1000000 2000000}
RUN_S=${RUN_S:-20}

[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
ORIGINAL=$(sed -nE 's/^#define DUCK_BRIDGE_BAUD ([0-9]+).*/\1/p' "$CONF")
[ -n "$ORIGINAL" ] || { echo "no DUCK_BRIDGE_BAUD in $CONF"; exit 1; }
MATRIX_WAS=$(sed -nE 's/^#define DUCK_STATUS_MATRIX ([01]).*/\1/p' "$CONF")
[ -z "${MATRIX:-}" ] || [ -n "$MATRIX_WAS" ] || { echo "no DUCK_STATUS_MATRIX in $CONF"; exit 1; }
matrix() { sed -i -E "s/^#define DUCK_STATUS_MATRIX [01]/#define DUCK_STATUS_MATRIX $1/" "$CONF"; }
GOVERNORS=$(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor 2>/dev/null | sort -u | tr '\n' ' ')

log() { echo "$*" | tee -a "$OUT"; }
run() { log "\$ $*"; "$@" 2>&1 | tee -a "$OUT"; }
temps() { log "temperatures (m degC): $(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | tr '\n' ' ')"; }
governor() { for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do [ -w "$g" ] && echo "$1" > "$g"; done; true; }

use_speed() {
  sed -i -E "s/^#define DUCK_BRIDGE_BAUD [0-9]+/#define DUCK_BRIDGE_BAUD $1/" "$CONF"
  sudo -H -u arduino arduino-app-cli app stop "$APP" >/dev/null 2>&1 || true
  sh "$DUCK/set-link-speed.sh" "$1" >/dev/null
  sudo -H -u arduino arduino-app-cli app start "$APP" >/dev/null 2>&1 || true
  # The sketch registers its functions once it is running; wait for them.
  # Then let it settle, so start-up is not counted as the link's failures.
  for _ in $(seq 60); do "$DUCK/duckd" status >/dev/null 2>&1 && sleep 5 && return 0; sleep 2; done
  log "speed $1: the firmware did not answer within 2 minutes"
  return 1
}

restore() {
  log "--- restoring link speed $ORIGINAL, matrix ${MATRIX_WAS:-as it was} and governor(s) $GOVERNORS"
  [ -z "${MATRIX:-}" ] || matrix "$MATRIX_WAS"
  for g in $GOVERNORS; do governor "$g"; break; done
  use_speed "$ORIGINAL" || true
}
trap restore EXIT
[ -z "${MATRIX:-}" ] || matrix "$MATRIX"

log "link-speed measurement $(date -u +%Y-%m-%dT%H:%M:%SZ); started at $ORIGINAL baud, governor(s) $GOVERNORS"
log "matrix: $(sed -nE 's/^#define DUCK_STATUS_MATRIX ([01]).*/\1/p' "$CONF"); each run $RUN_S s"
for speed in $SPEEDS; do
  log ""
  log "=== $speed baud"
  use_speed "$speed" || continue
  temps
  run "$DUCK/duckd" status
  run "$DUCK/duckd" time --count 500
  for hz in 50 100; do
    run "$DUCK/duckd" run --robot "$DUCK/robot.json" --exchange step --hz "$hz" --seconds "$RUN_S"
  done
  log "--- real-time: governor performance, SCHED_FIFO 50, core 3"
  governor performance
  for hz in 50 100; do
    run chrt -f 50 taskset -c 3 "$DUCK/duckd" run --robot "$DUCK/robot.json" --exchange step --hz "$hz" --seconds "$RUN_S"
  done
  for g in $GOVERNORS; do governor "$g"; break; done
  temps
done
log ""
log "results: $OUT"
