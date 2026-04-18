#!/usr/bin/env bash
# Start the combined sensors launch (Mid-360 + D435 + WitMotion).
# Idempotent: pre-cleans each per-sensor driver via its own kill_*.sh
# before relaunching, since each driver has its own way of clinging to
# its resources (UDP sockets, USB handle, serial port). This is the
# composition equivalent of start_mid360.sh + start_d435.sh + start_witmotion.sh.
#
# Pass-through args (forwarded to sensors.launch.py):
#   ./start_sensors.sh slam_mode:=true                 # for RTABMap
#   ./start_sensors.sh lidar_xfer_format:=1            # for FAST-LIO2
#   ./start_sensors.sh enable_rear:=true               # dual D435 (Phase 1.10)
#   ./start_sensors.sh enable_witmotion:=false         # drop one sensor

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash

NEEDS_CLEAN=0
pgrep -f livox_ros_driver2_node              > /dev/null && NEEDS_CLEAN=1
pgrep -f realsense2_camera_node              > /dev/null && NEEDS_CLEAN=1
pgrep -f "slam_bringup.*wt901c_imu"          > /dev/null && NEEDS_CLEAN=1
pgrep -f "ros2 launch slam_bringup sensors"  > /dev/null && NEEDS_CLEAN=1

if [ "$NEEDS_CLEAN" = "1" ]; then
  echo "start_sensors: at least one driver already running — cleaning up first"
  "$SCRIPT_DIR/kill_sensors.sh"

  # Wait up to 5s for processes to actually exit and resources to release
  for _ in 1 2 3 4 5; do
    pgrep -f livox_ros_driver2_node > /dev/null || \
      pgrep -f realsense2_camera_node > /dev/null || \
      pgrep -f "slam_bringup.*wt901c_imu" > /dev/null || break
    sleep 1
  done
fi

exec ros2 launch slam_bringup sensors.launch.py "$@"
