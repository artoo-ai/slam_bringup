#!/usr/bin/env bash
# Start the WitMotion WT901 launch file.
# Idempotent: if the driver or its launch wrapper is already running,
# clean it up via kill_witmotion.sh first — Ctrl-C does not always
# fully release the serial port, and a second launch against a busy
# /dev/ttyUSB0 fails with "Device or resource busy" from pyserial.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash

if pgrep -f "slam_bringup.*wt901c_imu" > /dev/null \
   || pgrep -f "ros2 launch slam_bringup witmotion" > /dev/null; then
  echo "start_witmotion: driver already running — cleaning up first"
  "$SCRIPT_DIR/kill_witmotion.sh"

  # Wait up to 5s for the process to actually exit and serial port to release
  for _ in 1 2 3 4 5; do
    pgrep -f "slam_bringup.*wt901c_imu" > /dev/null || break
    sleep 1
  done

  if pgrep -f "slam_bringup.*wt901c_imu" > /dev/null; then
    echo "start_witmotion: ERROR — wt901c_imu still running after kill_witmotion.sh" >&2
    exit 1
  fi
fi

exec ros2 launch slam_bringup witmotion.launch.py "$@"
