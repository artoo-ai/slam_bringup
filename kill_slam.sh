#!/usr/bin/env bash
# Force-kill the full SLAM stack. Layered: rtabmap → fast_lio → sensors,
# plus robot_state_publisher and the body→base_link static TF.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

"$SCRIPT_DIR/kill_rtabmap.sh"  2>/dev/null
"$SCRIPT_DIR/kill_fast_lio.sh" 2>/dev/null
"$SCRIPT_DIR/kill_sensors.sh"  2>/dev/null

pkill -SIGINT -f robot_state_publisher              2>/dev/null
sleep 1
pkill -9      -f robot_state_publisher              2>/dev/null
pkill -9      -f "static_transform_publisher.*body" 2>/dev/null
pkill -9      -f "ros2 launch slam_bringup slam"    2>/dev/null
ros2 daemon stop
