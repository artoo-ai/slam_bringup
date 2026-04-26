#!/usr/bin/env bash
# Start the URDF + sensors bringup (no SLAM).
# Idempotent: cleans stale per-sensor drivers + robot_state_publisher first.
#
# Pass-through args go to ros2 launch — e.g.:
#   ./start_perception.sh                              # bench_fixture, no rviz
#   ./start_perception.sh rviz:=true                   # bench_fixture + rviz
#   ./start_perception.sh platform:=go2 enable_rear:=true
#   ./start_perception.sh slam_mode:=true lidar_xfer_format:=1   # for full SLAM stack

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash

# Clean any stale launches before we relaunch — robot_state_publisher
# survives Ctrl-C surprisingly often, and a duplicate publisher of
# /robot_description silently picks the older URDF.
"$SCRIPT_DIR/kill_sensors.sh" 2>/dev/null
pkill -SIGINT -f robot_state_publisher 2>/dev/null
sleep 1
pkill -9      -f robot_state_publisher 2>/dev/null
pkill -9      -f "ros2 launch slam_bringup perception" 2>/dev/null

exec ros2 launch slam_bringup perception.launch.py "$@"
