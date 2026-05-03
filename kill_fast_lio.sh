#!/usr/bin/env bash
# Force-kill the FAST-LIO2 mapping node + its launch wrapper.
# Symptoms that this script fixes:
#   - next start_fast_lio.sh fails with "topic /Odometry already exists"
#     (means the previous fastlio_mapping survived Ctrl-C)
#   - two publishers on /cloud_registered (duplicate process silently active)
#   - "lidar loop back, clear buffer" flood from a stale imu_units republisher
#     double-publishing /livox/imu_ms2 with duplicate timestamps

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=kill_helpers.sh
source "$SCRIPT_DIR/kill_helpers.sh"

nuke_processes 'fastlio_mapping'                       'FAST-LIO2 mapping node' || exit 1
nuke_processes 'imu_units_node\|imu_units_g_to_ms2'    'IMU unit republisher'   || exit 1
nuke_processes 'ros2 launch slam_bringup fast_lio'     'fast_lio.launch.py wrapper' || true

# Take down the viz-clip republisher start_fast_lio.sh spawned alongside.
# Safe to call even if it isn't running.
"$SCRIPT_DIR/kill_viz_clip.sh" 2>/dev/null

ros2 daemon stop                                       # clear stale DDS discovery cache
