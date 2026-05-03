#!/usr/bin/env bash
# Start the visualization-only z-clip republisher (/cloud_registered →
# /cloud_viz_clipped). Idempotent: cleans a stale viz_clip_container
# before relaunching.
#
# Standalone use — pair with FAST-LIO2 or with a bag replay:
#   ./start_viz_clip.sh                          # defaults (viz_z_max=2.0, house)
#   ./start_viz_clip.sh viz_z_max:=4.5           # garage / high ceilings
#   ./start_viz_clip.sh viz_z_min:=0.5 viz_z_max:=1.5  # narrow floorplan slice
#   ./start_viz_clip.sh viz_input_topic:=/Laser_map    # clip the accumulated map instead
#
# Note: start_fast_lio.sh and start_slam.sh already spawn this for you;
# only run it directly when you want a custom topic/range against an
# already-running pipeline or a bag.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash
# shellcheck source=start_helpers.sh
source "$SCRIPT_DIR/start_helpers.sh"
ensure_foxglove

if pgrep -f "slam_bringup.viz_clip_node" > /dev/null \
   || pgrep -f "ros2 launch slam_bringup viz_clip" > /dev/null; then
  echo "start_viz_clip: viz_z_clip already running — cleaning up first"
  "$SCRIPT_DIR/kill_viz_clip.sh"

  for _ in 1 2 3 4 5; do
    pgrep -f "slam_bringup.viz_clip_node" > /dev/null || break
    sleep 1
  done

  if pgrep -f "slam_bringup.viz_clip_node" > /dev/null; then
    echo "start_viz_clip: ERROR — viz_z_clip still running after kill_viz_clip.sh" >&2
    exit 1
  fi
fi

exec ros2 launch slam_bringup viz_clip.launch.py "$@"
