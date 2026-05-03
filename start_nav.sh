#!/usr/bin/env bash
# Start the SLAM stack in NAVIGATION mode: localization-only RTABMap + Nav2.
#
# Prereq: you have already built a map by running ./start_slam.sh and
# walking the rig through the space. The map lives in ~/.ros/rtabmap.db
# (override with database_path:=...). This script does NOT make a new
# map — it loads the existing one read-only and runs Nav2 on top.
#
# Usage:
#   ./start_nav.sh                                          # bench_fixture, default DB
#   ./start_nav.sh platform:=go2                            # (once go2 bridge lands)
#   ./start_nav.sh database_path:=~/maps/house.db            # alternate DB
#   ./start_nav.sh nav2_params_file:=~/cfg/go2_nav2.yaml    # per-platform tuning
#   ./start_nav.sh rviz:=true                               # spawn RViz for "2D Goal Pose"
#
# What it does (vs start_slam.sh):
#   localization:=true     — RTABMap loads ~/.ros/rtabmap.db read-only
#   nav2:=true             — spawns Nav2 navigation pipeline
#   delete_db_on_start:=false (forced; can't navigate against an empty DB)
#
# Send a goal:
#   - In RViz: "2D Goal Pose" tool (or use rviz:=true)
#   - From CLI:
#       ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
#         "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}" --once
#
# Cmd_vel: published on /cmd_vel. Bench fixture has no consumer; real
# platforms wire their own Twist→drive bridge.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash

DB_PATH="${HOME}/.ros/rtabmap.db"
for arg in "$@"; do
  case "$arg" in
    database_path:=*) DB_PATH="${arg#database_path:=}" ;;
    delete_db_on_start:=true)
      echo "start_nav: ERROR — delete_db_on_start:=true is incompatible with navigation." >&2
      echo "  Navigation requires an existing map. Run ./start_slam.sh to build one first." >&2
      exit 1
      ;;
  esac
done

DB_PATH_EXPANDED="${DB_PATH/#\~/$HOME}"
if [ ! -f "$DB_PATH_EXPANDED" ]; then
  echo "start_nav: ERROR — RTABMap database not found: $DB_PATH_EXPANDED" >&2
  echo "  Run ./start_slam.sh first to build a map, then re-run start_nav.sh." >&2
  echo "  Or pass an alternate DB: ./start_nav.sh database_path:=/path/to/map.db" >&2
  exit 1
fi

# Idempotent — kill the layered stack first.
"$SCRIPT_DIR/kill_nav.sh"      2>/dev/null
"$SCRIPT_DIR/kill_rtabmap.sh"  2>/dev/null
"$SCRIPT_DIR/kill_fast_lio.sh" 2>/dev/null
"$SCRIPT_DIR/kill_viz_clip.sh" 2>/dev/null
"$SCRIPT_DIR/kill_sensors.sh"  2>/dev/null
pkill -SIGINT -f robot_state_publisher 2>/dev/null
sleep 1
pkill -9 -f robot_state_publisher              2>/dev/null
pkill -9 -f "static_transform_publisher.*body" 2>/dev/null
pkill -9 -f "ros2 launch slam_bringup slam"    2>/dev/null

exec ros2 launch slam_bringup slam.launch.py \
    nav2:=true \
    localization:=true \
    delete_db_on_start:=false \
    "$@"
