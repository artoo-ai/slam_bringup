#!/usr/bin/env bash
# Start the full SLAM stack: URDF + sensors + FAST-LIO2 + RTABMap.
# Idempotent: kills every layered piece before relaunching.
#
# Pass-through args:
#   ./start_slam.sh                                 # bench_fixture, fresh resume of last DB
#   ./start_slam.sh platform:=go2                   # (once go2 PLATFORM_BRIDGES entry lands)
#   ./start_slam.sh delete_db_on_start:=true        # wipe rtabmap.db, start fresh map
#   ./start_slam.sh localization:=true              # reuse existing DB, no new mapping
#   ./start_slam.sh rviz:=true                      # spawn rviz2 alongside Foxglove

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash

# Kill every layer of the stack before relaunching. Order matters:
# RTABMap first (depends on FAST-LIO2 odom), then FAST-LIO2, then sensors.
"$SCRIPT_DIR/kill_rtabmap.sh"  2>/dev/null
"$SCRIPT_DIR/kill_fast_lio.sh" 2>/dev/null
"$SCRIPT_DIR/kill_viz_clip.sh" 2>/dev/null
"$SCRIPT_DIR/kill_sensors.sh"  2>/dev/null
pkill -SIGINT -f robot_state_publisher 2>/dev/null
sleep 1
pkill -9      -f robot_state_publisher           2>/dev/null
pkill -9      -f "static_transform_publisher.*body" 2>/dev/null
pkill -9      -f "ros2 launch slam_bringup slam"     2>/dev/null

exec ros2 launch slam_bringup slam.launch.py "$@"
