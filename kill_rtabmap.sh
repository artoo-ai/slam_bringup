#!/usr/bin/env bash
# Force-kill the RTABMap node + its launch wrapper.
# Symptoms this script fixes: next start_rtabmap.sh fails because /map or
# /mapData already has a publisher (previous rtabmap survived Ctrl-C), or
# the rtabmap.db is locked by an orphan process.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=kill_helpers.sh
source "$SCRIPT_DIR/kill_helpers.sh"

nuke_processes 'rtabmap_slam/rtabmap'           'RTABMap node'          || exit 1
nuke_processes 'ros2 launch slam_bringup rtabmap' 'rtabmap.launch.py wrapper' || true

ros2 daemon stop                                # clear stale DDS discovery cache
