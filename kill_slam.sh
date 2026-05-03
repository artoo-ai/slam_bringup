#!/usr/bin/env bash
# Force-kill the full SLAM stack. Layered: rtabmap → fast_lio → sensors,
# plus robot_state_publisher and the body→base_link static TF.
#
# stderr is intentionally NOT redirected to /dev/null here — the layered
# kill scripts surface their own diagnostics ("X survived pkill -9,
# escalating to sudo") via stderr, and silently swallowing them is
# exactly how an orphan imu_units republisher or fastlio_mapping survived
# multiple kill_slam runs in the past, leading to phantom /livox/imu_ms2
# subscribers and FAST-LIO "lidar loop back" floods on the next launch.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

FAILED=0
"$SCRIPT_DIR/kill_yahboom.sh"  || FAILED=1
"$SCRIPT_DIR/kill_nav.sh"      || FAILED=1
"$SCRIPT_DIR/kill_rtabmap.sh"  || FAILED=1
"$SCRIPT_DIR/kill_fast_lio.sh" || FAILED=1
"$SCRIPT_DIR/kill_viz_clip.sh" || FAILED=1
"$SCRIPT_DIR/kill_sensors.sh"  || FAILED=1

# shellcheck source=kill_helpers.sh
source "$SCRIPT_DIR/kill_helpers.sh"
nuke_processes 'robot_state_publisher'              'robot_state_publisher'    || FAILED=1
nuke_processes 'static_transform_publisher.*body'   'body→base_link static TF' || FAILED=1
nuke_processes 'ros2 launch slam_bringup slam'      'slam.launch.py wrapper'   || FAILED=1

ros2 daemon stop

if [ $FAILED -ne 0 ]; then
  echo "kill_slam: one or more layers reported a surviving process — see warnings above." >&2
  echo "  Diagnostic: pgrep -af 'fastlio|livox|imu_units|rtabmap|nav|pointcloud_to_laserscan'" >&2
  exit 1
fi
