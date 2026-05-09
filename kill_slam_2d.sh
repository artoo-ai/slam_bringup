#!/usr/bin/env bash
# Force-kill the simplified 2D SLAM stack.
# Layered: slam_toolbox → rf2o → pointcloud_to_laserscan → sensors,
# plus robot_state_publisher and the launch wrapper.
#
# stderr is intentionally NOT redirected — kill_helpers.sh surfaces
# "X survived pkill -9, escalating to sudo" warnings the user needs to see.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

FAILED=0
"$SCRIPT_DIR/kill_yahboom.sh"  || FAILED=1
"$SCRIPT_DIR/kill_sensors.sh"  || FAILED=1

# shellcheck source=kill_helpers.sh
source "$SCRIPT_DIR/kill_helpers.sh"
nuke_processes 'async_slam_toolbox_node'             'slam_toolbox'              || FAILED=1
nuke_processes 'rf2o_laser_odometry_node'            'rf2o laser odometry'       || FAILED=1
nuke_processes 'pointcloud_to_laserscan_node'        'pointcloud_to_laserscan'   || FAILED=1
nuke_processes 'robot_state_publisher'               'robot_state_publisher'     || FAILED=1
nuke_processes 'ros2 launch slam_bringup slam_2d'    'slam_2d.launch.py wrapper' || FAILED=1

ros2 daemon stop

if [ $FAILED -ne 0 ]; then
  echo "kill_slam_2d: one or more layers reported a surviving process — see warnings above." >&2
  echo "  Diagnostic: pgrep -af 'slam_toolbox|rf2o|pointcloud_to_laserscan|livox|robot_state_publisher'" >&2
  exit 1
fi
