#!/usr/bin/env bash
# Ctrl-C does not always fully stop livox_ros_driver2_node — it
# can leave an orphan process holding UDP ports 56101/56201/56301/56401.
# Symptoms: the next ros2 launch slam_bringup mid360.launch.py
# starts but ros2 topic info /livox/lidar shows two publishers,
# or the driver fails to bind its listener sockets. Two publishers on
# /livox/lidar is also the #1 cause of FAST-LIO's "lidar loop back,
# clear buffer" flood — interleaved scans with slightly different
# timestamps look out-of-order to FAST-LIO and every one is rejected.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=kill_helpers.sh
source "$SCRIPT_DIR/kill_helpers.sh"

nuke_processes 'livox_ros_driver2_node'         'Mid-360 driver'         || exit 1
nuke_processes 'ros2 launch slam_bringup mid360' 'mid360.launch.py wrapper' || true

ros2 daemon stop                            # clear stale DDS discovery cache
