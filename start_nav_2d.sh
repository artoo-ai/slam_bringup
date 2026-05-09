#!/usr/bin/env bash
# Start the simplified 2D NAVIGATION stack: localization-only slam_toolbox + Nav2.
# Mirrors start_nav.sh, but for the 2D path (slam_2d.launch.py).
#
# Prereq: you have already built a map by running ./start_slam_2d.sh and
# saving the serialized graph from RViz's SlamToolbox panel (the
# "Serialize Map" button — NOT "Save Map", which produces a .pgm/.yaml
# pair instead of the .data/.posegraph pair localization mode needs).
#
# Usage:
#   ./start_nav_2d.sh map_file:=~/maps/livingroom        # default mecanum
#   ./start_nav_2d.sh map_file:=~/maps/livingroom enable_drive:=false   # bench debug
#   ./start_nav_2d.sh map_file:=~/maps/foo nav2_params_file:=~/cfg/foo.yaml
#
# Foxglove bridge auto-spawns. Set SLAM_NO_FOXGLOVE=1 to suppress.
#
# What it does (vs start_slam_2d.sh):
#   mode:=localization  — slam_toolbox loads the serialized graph read-only
#   nav2:=true          — spawns Nav2 navigation pipeline
#   enable_drive:=true  — yahboom_bridge consumes /cmd_vel from Nav2

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash
# shellcheck source=start_helpers.sh
source "$SCRIPT_DIR/start_helpers.sh"
ensure_foxglove

# Pre-flight: extract map_file from args, expand ~, and verify the
# serialized pair exists. The launch file does this too, but failing
# here gives a cleaner error before any nodes start.
MAP_FILE=""
for arg in "$@"; do
  case "$arg" in
    map_file:=*) MAP_FILE="${arg#map_file:=}" ;;
    mode:=*)
      echo "start_nav_2d: ERROR — mode is forced to localization here." >&2
      echo "  To map, use ./start_slam_2d.sh instead." >&2
      exit 1
      ;;
  esac
done

if [ -z "$MAP_FILE" ]; then
  echo "start_nav_2d: ERROR — map_file:=<path without extension> is required." >&2
  echo "  Example: ./start_nav_2d.sh map_file:=~/maps/livingroom" >&2
  echo "  This loads ~/maps/livingroom.data + ~/maps/livingroom.posegraph." >&2
  exit 1
fi

MAP_FILE_EXPANDED="${MAP_FILE/#\~/$HOME}"
if [ ! -f "${MAP_FILE_EXPANDED}.data" ] || [ ! -f "${MAP_FILE_EXPANDED}.posegraph" ]; then
  echo "start_nav_2d: ERROR — slam_toolbox serialized files not found:" >&2
  [ -f "${MAP_FILE_EXPANDED}.data" ]      || echo "  missing: ${MAP_FILE_EXPANDED}.data"      >&2
  [ -f "${MAP_FILE_EXPANDED}.posegraph" ] || echo "  missing: ${MAP_FILE_EXPANDED}.posegraph" >&2
  echo "  Run ./start_slam_2d.sh first, drive around, then in RViz click" >&2
  echo "  the SlamToolbox panel's 'Serialize Map' button (NOT 'Save Map')." >&2
  exit 1
fi

# Tear down BOTH stacks. Same reasoning as start_slam_2d.sh — the 3D and
# 2D paths share the Mid-360 driver and perception layer.
"$SCRIPT_DIR/kill_nav.sh"      2>/dev/null
"$SCRIPT_DIR/kill_rtabmap.sh"  2>/dev/null
"$SCRIPT_DIR/kill_fast_lio.sh" 2>/dev/null
"$SCRIPT_DIR/kill_viz_clip.sh" 2>/dev/null
"$SCRIPT_DIR/kill_sensors.sh"  2>/dev/null

# 2D-specific orphans + Nav2 leftovers from a previous nav run.
pkill -SIGINT -f pointcloud_to_laserscan_node 2>/dev/null
pkill -SIGINT -f rf2o_laser_odometry_node     2>/dev/null
pkill -SIGINT -f async_slam_toolbox_node      2>/dev/null
pkill -SIGINT -f localization_slam_toolbox    2>/dev/null
pkill -SIGINT -f robot_state_publisher        2>/dev/null
pkill -SIGINT -f nav2_                        2>/dev/null
sleep 1
pkill -9 -f pointcloud_to_laserscan_node      2>/dev/null
pkill -9 -f rf2o_laser_odometry_node          2>/dev/null
pkill -9 -f async_slam_toolbox_node           2>/dev/null
pkill -9 -f localization_slam_toolbox         2>/dev/null
pkill -9 -f robot_state_publisher             2>/dev/null
pkill -9 -f nav2_                             2>/dev/null
pkill -9 -f "ros2 launch slam_bringup slam_2d" 2>/dev/null
pkill -9 -f "ros2 launch slam_bringup slam"    2>/dev/null

exec ros2 launch slam_bringup slam_2d.launch.py \
    mode:=localization \
    nav2:=true \
    enable_drive:=true \
    "$@"
