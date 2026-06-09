#!/usr/bin/env bash
# Start autonomous frontier exploration with the 3D SLAM stack.
# Idempotent: kills all running stacks before launching.
#
# Usage:
#   ./start_explore.sh                          # fresh 15-min explore
#   ./start_explore.sh resume:=true             # continue from existing RTABMap DB
#   ./start_explore.sh time_limit:=30           # 30-minute explore
#   ./start_explore.sh time_limit:=0            # explore until full coverage
#
# The robot will:
#   1. Start SLAM + Nav2
#   2. Autonomously explore by driving toward map frontiers
#   3. Save the map when done (or time runs out)
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
SLAM_ARGS=()
EXPLORE_ARGS=()

for arg in "$@"; do
  case "$arg" in
    resume:=true)   RESUME=true ;;
    resume:=false)  RESUME=false ;;
    time_limit:=*)  TIME_LIMIT="${arg#time_limit:=}"
                    EXPLORE_ARGS+=("$arg") ;;
    *)              SLAM_ARGS+=("$arg") ;;
  esac
done

if [ "$RESUME" = "true" ]; then
  DELETE_DB=false
else
  DELETE_DB=true
fi

# ── Teardown ────────────────────────────────────────────────
"$SCRIPT_DIR/kill_explore.sh"  2>/dev/null
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

# ── Launch SLAM + Nav2 in background ───────────────────────
echo "==> Starting 3D SLAM + Nav2 (delete_db=$DELETE_DB, resume=$RESUME)..."
ros2 launch slam_bringup slam.launch.py \
    nav2:=true \
    enable_drive:=true \
    delete_db_on_start:="$DELETE_DB" \
    "${SLAM_ARGS[@]}" &
SLAM_PID=$!

# Give SLAM + Nav2 time to initialize before starting exploration.
echo "==> Waiting 15s for SLAM + Nav2 to initialize..."
sleep 15

# ── Launch exploration layer ───────────────────────────────
echo "==> Starting frontier exploration (time_limit=${TIME_LIMIT} min)..."
ros2 launch slam_bringup explore.launch.py \
    slam_mode:=3d \
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
