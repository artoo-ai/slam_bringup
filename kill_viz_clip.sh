#!/usr/bin/env bash
# Force-kill the viz-clip republisher node + its launch wrapper.
# Run this if a previous start_viz_clip.sh / start_fast_lio.sh / start_slam.sh
# left a stray viz_z_clip behind — symptom is "node name already exists" on
# the next launch, or two publishers on /cloud_viz_clipped.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=kill_helpers.sh
source "$SCRIPT_DIR/kill_helpers.sh"

# Match both the entry-point script (slam_bringup/viz_clip_node.py runs as
# `viz_clip` per setup.py) and the launch wrapper.
nuke_processes 'slam_bringup.viz_clip_node\|/viz_clip\b' 'viz_clip republisher' || exit 1
nuke_processes 'ros2 launch slam_bringup viz_clip'       'viz_clip.launch.py wrapper' || true
exit 0
