#!/usr/bin/env bash
# Ctrl-C usually stops our wt901c_imu Python node cleanly via the
# KeyboardInterrupt handler in main(), but if the process gets wedged
# (e.g. during a stuck serial.read() call) it can leave /dev/ttyUSB0
# held open. Symptoms on the next ros2 launch slam_bringup witmotion:
# "Device or resource busy" from pyserial, or two publishers on /imu/data.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=kill_helpers.sh
source "$SCRIPT_DIR/kill_helpers.sh"

nuke_processes 'slam_bringup.*wt901c_imu'           'WitMotion wt901c node' || exit 1
nuke_processes 'ros2 launch slam_bringup witmotion' 'witmotion.launch.py wrapper' || true

ros2 daemon stop                                    # clear stale DDS discovery cache
