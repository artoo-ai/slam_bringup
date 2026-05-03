#!/usr/bin/env bash
# Publish a static TF from the Mid-360's livox_frame to the D435 root
# camera_link so Foxglove / RViz can render both point clouds in the
# same 3D panel. Temporary bench-only shim — Phase 1.7 replaces this
# with robot_state_publisher driven by URDF.
#
# Idempotent: if a matching static_transform_publisher is already
# running, it gets stopped first — two publishers for the same TF edge
# produce undefined lookups in downstream consumers.

# -----------------------------------------------------------------------
# Offsets: D435 camera_link relative to Mid-360 livox_frame.
# MEASURE YOUR RIG — the defaults are a rough placeholder from PLAN.md.
# Right-handed frame: +X forward, +Y left, +Z up (REP-103).
# -----------------------------------------------------------------------
X=0.20
Y=0.00
Z=0.00
ROLL=0.0
PITCH=0.0
YAW=0.0

PARENT=livox_frame
CHILD=camera_link

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash
# shellcheck source=start_helpers.sh
source "$SCRIPT_DIR/start_helpers.sh"
ensure_foxglove

# Match only our own edge — pgrep on parent+child names so an unrelated
# static_transform_publisher elsewhere is left alone.
PATTERN="static_transform_publisher.*${PARENT}.*${CHILD}"

if pgrep -f "$PATTERN" > /dev/null; then
  echo "start_bench_tf: existing ${PARENT}->${CHILD} publisher running — restarting"
  pkill -SIGINT -f "$PATTERN" 2>/dev/null
  sleep 1
  pkill -9      -f "$PATTERN" 2>/dev/null

  for _ in 1 2 3; do
    pgrep -f "$PATTERN" > /dev/null || break
    sleep 1
  done

  if pgrep -f "$PATTERN" > /dev/null; then
    echo "start_bench_tf: ERROR — previous publisher still running" >&2
    exit 1
  fi
fi

exec ros2 run tf2_ros static_transform_publisher \
  --x "$X" --y "$Y" --z "$Z" \
  --roll "$ROLL" --pitch "$PITCH" --yaw "$YAW" \
  --frame-id "$PARENT" --child-frame-id "$CHILD"
