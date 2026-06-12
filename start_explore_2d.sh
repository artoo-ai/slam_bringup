#!/usr/bin/env bash
# Start autonomous frontier exploration with the 2D SLAM stack.
# Idempotent: kills all running stacks before launching.
#
# Usage:
#   ./start_explore_2d.sh                                            # fresh 15-min explore
#   ./start_explore_2d.sh resume:=true                               # continue ~/maps/explore_latest;
#                                       # seeds the pose automatically from explore_latest.pose
#                                       # (written by the previous session) if the file exists
#   ./start_explore_2d.sh resume:=true start_at_dock:=true           # robot carried back to the
#                                       # ORIGINAL start pose — ignore the .pose file
#   ./start_explore_2d.sh resume:=true map_file:=~/maps/explore_latest \
#       map_start_pose:=1.2,-0.4,3.14   # override: resume from an explicit map-frame pose
#   ./start_explore_2d.sh time_limit:=30                             # 30-minute explore
#   ./start_explore_2d.sh time_limit:=0                              # full coverage
#   ./start_explore_2d.sh d435:=false                                # skip the live camera
#                                       # (D435 is ON by default at light 15 fps profiles)
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
START_AT_DOCK=false
D435_SET=false
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
    start_at_dock:=true)
                    # "Robot is back at the ORIGINAL session start pose" —
                    # skip the auto-read .pose file (which holds where the
                    # robot ENDED last session) and use dock anchoring.
                    # Handled here, NOT forwarded to the launch file.
                    START_AT_DOCK=true ;;
    start_at_dock:=false)
                    START_AT_DOCK=false ;;
    d435:=*)        D435_SET=true
                    SLAM_ARGS+=("$arg") ;;
    *)              SLAM_ARGS+=("$arg") ;;
  esac
done

# Live camera view ON by default for exploration runs (light 15 fps
# profiles). Override with d435:=false if CPU margin gets tight — watch
# the GUI's pose-rate readout. NEVER view image_raw over WiFi (saturates
# the link and starves the GUI bridge); use the GUI MJPEG inset or a
# compressed topic.
if [ "$D435_SET" = "false" ]; then
  SLAM_ARGS+=("d435:=true")
fi

if [ "$MAP_START_POSE_SET" = "true" ] && [ "$START_AT_DOCK" = "true" ]; then
  echo "ERROR: map_start_pose:=... and start_at_dock:=true are mutually exclusive" >&2
  echo "       (one says 'I am HERE', the other says 'I am at the original start')." >&2
  exit 1
fi

SLAM_MODE_ARG="mode:=mapping"

# Resume: continue mapping from a serialized graph (slam_toolbox stays in
# mapping mode). The seed pose MUST be roughly right — the scan matcher
# only corrects ~±0.25 m / ±20°; a 180° placement error is unrecoverable
# and corrupts the graph with bad loop closures. Seed priority:
#   1. explicit map_start_pose:=x,y,theta on the command line
#   2. start_at_dock:=true — force dock anchoring, ignoring any .pose file
#      (use when the robot was carried back to the original start pose)
#   3. $MAP_FILE.pose — written continuously by the previous session's
#      explore_manager, so it holds where the robot came to rest. Valid
#      as long as the robot hasn't been moved since.
#   4. dock anchoring (graph's FIRST node) — robot must physically sit at
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
  if [ "$START_AT_DOCK" = "true" ]; then
    echo "==> Resume: start_at_dock:=true — ignoring any $MAP_FILE.pose. The robot"
    echo "    must sit at the ORIGINAL session's start pose, SAME heading (±20 cm / ±20°)."
  elif [ "$MAP_START_POSE_SET" = "false" ] && [ -f "$MAP_FILE.pose" ]; then
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
# Idempotency + safety contract, delegated to kill_explore_2d.sh (single
# source of truth, also the manual rescue script). It stops the motors
# FIRST, kills the ros2 launch PARENTS before node children (explore_lite
# runs with respawn=true — killing children first just resurrects them),
# runs the layer teardowns, and fails closed with SIGKILL escalation if
# anything survives. Refuse to launch on top of survivors.
if ! "$SCRIPT_DIR/kill_explore_2d.sh"; then
  echo "==> ABORT: teardown left survivors — refusing to launch a second" >&2
  echo "    stack on top (overlapping stacks corrupt TF and abort every" >&2
  echo "    Nav2 goal). See output above; clear them and retry." >&2
  exit 1
fi
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
# Ctrl-C contract: ONE Ctrl-C triggers a full ordered shutdown. Further
# Ctrl-Cs are absorbed (the trap is disabled inside cleanup) — they used
# to re-enter the handler mid-`wait`, strand the ros2 launch trees, and
# explore_lite's respawn=true then resurrected nodes forever until the
# user hunted them with ps (field, 2026-06-12). The final
# kill_explore_2d.sh sweep handles respawn parents, motors, and any node
# wedged in shutdown.
CLEANED_UP=0
cleanup() {
    [ "$CLEANED_UP" -eq 1 ] && return
    CLEANED_UP=1
    trap '' SIGINT SIGTERM    # absorb further Ctrl-C — it cannot speed this up
    echo ""
    echo "==> Shutting down exploration (takes ~10-20 s; additional Ctrl-C is ignored)..."
    kill -SIGINT "$EXPLORE_PID" 2>/dev/null
    kill -SIGINT "$SLAM_PID" 2>/dev/null
    # Bounded wait for both launch trees to exit gracefully.
    for _ in $(seq 1 20); do
        kill -0 "$EXPLORE_PID" 2>/dev/null || kill -0 "$SLAM_PID" 2>/dev/null || break
        sleep 1
    done
    # Sweep: respawn parents, stragglers, motors. Always run — cheap when
    # everything already exited cleanly.
    "$SCRIPT_DIR/kill_explore_2d.sh" || true
    echo "==> Shutdown complete."
}
trap cleanup SIGINT SIGTERM

wait -n "$SLAM_PID" "$EXPLORE_PID"
cleanup
