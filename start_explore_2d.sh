#!/usr/bin/env bash
# Start autonomous frontier exploration with the 2D SLAM stack.
# Idempotent: kills all running stacks before launching.
#
# Usage:
#   ./start_explore_2d.sh                                            # fresh 15-min explore
#   ./start_explore_2d.sh resume:=true                               # continue ~/maps/explore_latest;
#                                       # seeds the pose automatically from explore_latest.pose
#                                       # (written by the previous session) if the file exists
#   ./start_explore_2d.sh resume:=true map_file:=~/maps/explore_latest \
#       map_start_pose:=1.2,-0.4,3.14   # override: resume from an explicit map-frame pose
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
MAP_START_POSE_SET=false
SLAM_ARGS=()
EXPLORE_ARGS=()

for arg in "$@"; do
  case "$arg" in
    resume:=true)   RESUME=true ;;
    resume:=false)  RESUME=false ;;
    map_file:=*)    MAP_FILE="${arg#map_file:=}" ;;
    time_limit:=*)  TIME_LIMIT="${arg#time_limit:=}"
                    EXPLORE_ARGS+=("$arg") ;;
    map_start_pose:=*)
                    # Explicit seed pose wins over the auto-read .pose file.
                    MAP_START_POSE_SET=true
                    SLAM_ARGS+=("$arg") ;;
    *)              SLAM_ARGS+=("$arg") ;;
  esac
done

SLAM_MODE_ARG="mode:=mapping"

# Resume: continue mapping from a serialized graph (slam_toolbox stays in
# mapping mode). The seed pose MUST be roughly right — the scan matcher
# only corrects ~±0.25 m / ±20°; a 180° placement error is unrecoverable
# and corrupts the graph with bad loop closures. Seed priority:
#   1. explicit map_start_pose:=x,y,theta on the command line
#   2. $MAP_FILE.pose — written continuously by the previous session's
#      explore_manager, so it holds where the robot came to rest. Valid
#      as long as the robot hasn't been moved since.
#   3. dock anchoring (graph's FIRST node) — robot must physically sit at
#      the ORIGINAL session's start pose, same heading.
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
  # Let this session's explore_manager keep $MAP_FILE.pose current from launch.
  EXPLORE_ARGS+=("resume_map_file:=$MAP_FILE")
  if [ "$MAP_START_POSE_SET" = "false" ] && [ -f "$MAP_FILE.pose" ]; then
    read -r POSE_X POSE_Y POSE_TH _ < "$MAP_FILE.pose"
    NUM_RE='^-?[0-9]+([.][0-9]+)?$'
    if [[ "$POSE_X" =~ $NUM_RE && "$POSE_Y" =~ $NUM_RE && "$POSE_TH" =~ $NUM_RE ]]; then
      SLAM_ARGS+=("map_start_pose:=$POSE_X,$POSE_Y,$POSE_TH")
      echo "==> Resume: seeding pose from $MAP_FILE.pose (written $(date -r "$MAP_FILE.pose" 2>/dev/null || echo 'unknown time')):"
      echo "    x=$POSE_X y=$POSE_Y theta=$POSE_TH rad"
      echo "    Only valid if the robot has NOT been moved since the last session ended."
      echo "    If it has, pass map_start_pose:=x,y,theta or place it at the dock."
    else
      echo "==> WARNING: $MAP_FILE.pose is malformed ('$POSE_X $POSE_Y $POSE_TH') — ignoring it." >&2
      echo "    Falling back to dock anchoring: robot must sit at the ORIGINAL" >&2
      echo "    session's start pose, same heading." >&2
    fi
  elif [ "$MAP_START_POSE_SET" = "false" ]; then
    echo "==> Resume: no $MAP_FILE.pose found — using dock anchoring. The robot must"
    echo "    sit at the ORIGINAL session's start pose, SAME heading (±20 cm / ±20°)."
    echo "    (Sessions from now on write a .pose file so this is automatic next time.)"
  fi
elif [ -n "$MAP_FILE" ]; then
  # map_file without resume:=true — pass through unchanged.
  SLAM_ARGS+=("map_file:=$MAP_FILE")
fi

# ── Teardown ────────────────────────────────────────────────
# Idempotency + safety contract. Two failure modes this guards against:
#
#  1. Overlapping stacks. This block used to run the kill_*.sh scripts with
#     `2>/dev/null` and IGNORE their exit codes, so a failed teardown was
#     silent and we'd launch a second (or third) stack on top. Multiple
#     slam_toolbox/rf2o nodes then publish conflicting map->odom / odom->base
#     TFs, the robot's pose flip-flops, and Nav2 aborts every goal. Now we
#     honor exit codes, kill RViz (no kill_*.sh covers it), and VERIFY the
#     graph is clear before launching — aborting loudly if not.
#
#  2. Motor runaway. The Yahboom board has no command timeout of its own; it
#     coasts on the last velocity until something sends zero. The bridge zeroes
#     the motors on SIGINT (destroy_node) and on a /cmd_vel watchdog timeout —
#     but ONLY if /dev/myserial actually points at the board. So we stop the
#     motors FIRST, gracefully (SIGINT the bridge) and then directly over
#     /dev/myserial, before any SIGKILL that could strand a latched command.
source "$SCRIPT_DIR/kill_helpers.sh"

# (2) Stop motors before anything else. SIGINT the bridge so its destroy_node
#     zeroes the board; wait for it to drain; then command zero directly in
#     case no bridge was running (or it had already been hard-killed).
pkill -SIGINT -f 'slam_bringup.yahboom_bridge_node\|/yahboom_bridge' 2>/dev/null
sleep 2
python3 - <<'PYSTOP' 2>/dev/null || true
import time
try:
    from Rosmaster_Lib import Rosmaster
    bot = Rosmaster(com="/dev/myserial")
    bot.create_receive_threading()
    for _ in range(20):
        bot.set_car_motion(0.0, 0.0, 0.0)
        time.sleep(0.05)
except Exception:
    pass
PYSTOP

# (1a) Kill any OTHER start_explore_2d.sh wrappers (never ourselves) so a
#      prior run's wait/trap can't resurrect children mid-teardown.
for _pid in $(pgrep -f start_explore_2d.sh 2>/dev/null); do
  [ "$_pid" != "$$" ] && [ "$_pid" != "$PPID" ] && kill -9 "$_pid" 2>/dev/null
done

# (1b) Run the layer teardowns, honoring exit codes (no more silent 2>/dev/null).
TEARDOWN_OK=1
"$SCRIPT_DIR/kill_explore.sh"  || TEARDOWN_OK=0
"$SCRIPT_DIR/kill_nav.sh"      || TEARDOWN_OK=0
"$SCRIPT_DIR/kill_nav_2d.sh"   || TEARDOWN_OK=0
"$SCRIPT_DIR/kill_rtabmap.sh"  || TEARDOWN_OK=0
"$SCRIPT_DIR/kill_fast_lio.sh" || TEARDOWN_OK=0
"$SCRIPT_DIR/kill_viz_clip.sh" || TEARDOWN_OK=0
"$SCRIPT_DIR/kill_sensors.sh"  || TEARDOWN_OK=0
nuke_processes 'rviz2' 'RViz'  || TEARDOWN_OK=0   # no dedicated kill_*.sh covers RViz

# (1c) Fail-closed guard: never launch on top of a survivor.
_STACK_PAT='rf2o_laser_odometry_node\|async_slam_toolbox_node\|localization_slam_toolbox\|nav2_\|controller_server\|planner_server\|bt_navigator\|behavior_server\|smoother_server\|velocity_smoother\|waypoint_follower\|lifecycle_manager\|lib/explore_lite/explore\|explore_manager\|ros2 launch slam_bringup'
_SURVIVORS="$(pgrep -af "$_STACK_PAT" 2>/dev/null | grep -v start_explore_2d.sh)"
if [ -n "$_SURVIVORS" ]; then
  echo "" >&2
  echo "==> ABORT: a previous stack survived teardown — refusing to launch a" >&2
  echo "    second stack on top (overlapping stacks corrupt TF and abort every" >&2
  echo "    Nav2 goal). Surviving processes:" >&2
  echo "$_SURVIVORS" | sed 's/^/      /' >&2
  echo "    Clear them and retry, e.g.:" >&2
  echo "      pkill -9 -f 'rf2o|slam_toolbox|nav2_|rviz2|explore'   # or reboot" >&2
  exit 1
fi
[ "$TEARDOWN_OK" -ne 1 ] && \
  echo "==> Note: a kill_*.sh returned non-zero but no stack processes remain — continuing." >&2
echo "==> Teardown verified clean — no prior stack running."

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
