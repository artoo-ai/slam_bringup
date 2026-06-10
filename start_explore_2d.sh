#!/usr/bin/env bash
# Start autonomous frontier exploration with the 2D SLAM stack.
# Idempotent: kills all running stacks before launching.
#
# Usage:
#   ./start_explore_2d.sh                                            # fresh 15-min explore
#   ./start_explore_2d.sh resume:=true map_file:=~/maps/explore_latest  # continue
#   ./start_explore_2d.sh time_limit:=30                             # 30-minute explore
#   ./start_explore_2d.sh time_limit:=0                              # full coverage
#
# The robot will:
#   1. Start 2D SLAM + Nav2
#   2. Autonomously explore by driving toward map frontiers
#   3. Save the serialized map when done (or time runs out)
#   4. Return to its starting position

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash
# shellcheck source=start_helpers.sh
source "$SCRIPT_DIR/start_helpers.sh"
ensure_foxglove

# ── Parse args ──────────────────────────────────────────────
RESUME=false
TIME_LIMIT=15.0
MAP_FILE=""
SLAM_ARGS=()
EXPLORE_ARGS=()

for arg in "$@"; do
  case "$arg" in
    resume:=true)   RESUME=true ;;
    resume:=false)  RESUME=false ;;
    map_file:=*)    MAP_FILE="${arg#map_file:=}" ;;
    time_limit:=*)  TIME_LIMIT="${arg#time_limit:=}"
                    EXPLORE_ARGS+=("$arg") ;;
    *)              SLAM_ARGS+=("$arg") ;;
  esac
done

SLAM_MODE_ARG="mode:=mapping"

# Resume: continue mapping from a serialized graph (slam_toolbox stays in
# mapping mode; the launch file passes map_file_name + map_start_at_dock).
# The resumed session anchors to the graph's FIRST node, so place the
# robot at the original session's starting position before running this.
if [ "$RESUME" = "true" ]; then
  if [ -z "$MAP_FILE" ]; then
    MAP_FILE="$HOME/maps/explore_latest"
  fi
  MAP_FILE="${MAP_FILE/#\~/$HOME}"
  if [ ! -f "$MAP_FILE.data" ] || [ ! -f "$MAP_FILE.posegraph" ]; then
    echo "ERROR: resume requested but $MAP_FILE.data / .posegraph not found." >&2
    echo "       Pass map_file:=<path without extension>, or run a fresh explore." >&2
    exit 1
  fi
  SLAM_ARGS+=("map_file:=$MAP_FILE")
elif [ -n "$MAP_FILE" ]; then
  # map_file without resume:=true — pass through unchanged.
  SLAM_ARGS+=("map_file:=$MAP_FILE")
fi

# ── Teardown ────────────────────────────────────────────────
"$SCRIPT_DIR/kill_explore.sh"  2>/dev/null
"$SCRIPT_DIR/kill_nav.sh"      2>/dev/null
"$SCRIPT_DIR/kill_nav_2d.sh"   2>/dev/null
"$SCRIPT_DIR/kill_rtabmap.sh"  2>/dev/null
"$SCRIPT_DIR/kill_fast_lio.sh" 2>/dev/null
"$SCRIPT_DIR/kill_viz_clip.sh" 2>/dev/null
"$SCRIPT_DIR/kill_sensors.sh"  2>/dev/null
pkill -SIGINT -f pointcloud_to_laserscan_node 2>/dev/null
pkill -SIGINT -f rf2o_laser_odometry_node     2>/dev/null
pkill -SIGINT -f async_slam_toolbox_node      2>/dev/null
pkill -SIGINT -f robot_state_publisher        2>/dev/null
sleep 1
pkill -9 -f pointcloud_to_laserscan_node 2>/dev/null
pkill -9 -f rf2o_laser_odometry_node     2>/dev/null
pkill -9 -f async_slam_toolbox_node      2>/dev/null
pkill -9 -f robot_state_publisher        2>/dev/null
pkill -9 -f "ros2 launch slam_bringup slam_2d" 2>/dev/null
pkill -9 -f "ros2 launch slam_bringup slam"    2>/dev/null

# ── Launch 2D SLAM + Nav2 in background ────────────────────
echo "==> Starting 2D SLAM + Nav2 (resume=$RESUME)..."
ros2 launch slam_bringup slam_2d.launch.py \
    "$SLAM_MODE_ARG" \
    enable_drive:=true \
    nav2:=true \
    "${SLAM_ARGS[@]}" &
SLAM_PID=$!

echo "==> Waiting 10s for 2D SLAM + Nav2 to initialize..."
sleep 10

# ── Launch exploration layer ───────────────────────────────
echo "==> Starting frontier exploration (time_limit=${TIME_LIMIT} min)..."
ros2 launch slam_bringup explore.launch.py \
    slam_mode:=2d \
    time_limit:="$TIME_LIMIT" \
    "${EXPLORE_ARGS[@]}" &
EXPLORE_PID=$!

# ── Wait for either to exit ───────────────────────────────
cleanup() {
    echo ""
    echo "==> Shutting down exploration..."
    kill -SIGINT "$EXPLORE_PID" 2>/dev/null
    sleep 2
    kill -SIGINT "$SLAM_PID" 2>/dev/null
    wait
}
trap cleanup SIGINT SIGTERM

wait -n "$SLAM_PID" "$EXPLORE_PID"
cleanup
