#!/usr/bin/env bash
# Start the simplified 2D SLAM stack: Mid-360 → 2D scan → rf2o → slam_toolbox.
# The vacuum-equivalent path. See docs/why_slam_is_hard_and_how_to_simplify.md.
# Idempotent: tears down both the 2D and the 3D stacks before launching.
#
# Pass-through args (see launch/slam_2d.launch.py for the full list):
#   ./start_slam_2d.sh                              # Option A defaults
#   ./start_slam_2d.sh enable_drive:=true           # + cmd_vel teleop bridge
#   ./start_slam_2d.sh slam_params_file:=/tmp/x.yaml  # custom slam_toolbox tuning
#   ./start_slam_2d.sh scan_z_min:=0.20 scan_z_max:=0.50  # adjust 2D-projection band
#   ./start_slam_2d.sh use_wheel_odom:=true         # (Option B stub today)
#
# Foxglove bridge auto-spawns. Set SLAM_NO_FOXGLOVE=1 to suppress.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash
# shellcheck source=start_helpers.sh
source "$SCRIPT_DIR/start_helpers.sh"
ensure_foxglove

# Tear down BOTH stacks. The 3D and 2D paths share the Mid-360 driver and
# the perception layer — leaving an old slam.launch.py running would
# duplicate-publish on /livox/lidar and confuse the new stack.
"$SCRIPT_DIR/kill_nav.sh"      2>/dev/null
"$SCRIPT_DIR/kill_rtabmap.sh"  2>/dev/null
"$SCRIPT_DIR/kill_fast_lio.sh" 2>/dev/null
"$SCRIPT_DIR/kill_viz_clip.sh" 2>/dev/null
"$SCRIPT_DIR/kill_sensors.sh"  2>/dev/null

# 2D-specific orphans (the pieces this stack adds on top of perception).
pkill -SIGINT -f pointcloud_to_laserscan_node 2>/dev/null
pkill -SIGINT -f rf2o_laser_odometry_node     2>/dev/null
pkill -SIGINT -f async_slam_toolbox_node      2>/dev/null
pkill -SIGINT -f robot_state_publisher        2>/dev/null
sleep 1
pkill -9      -f pointcloud_to_laserscan_node 2>/dev/null
pkill -9      -f rf2o_laser_odometry_node     2>/dev/null
pkill -9      -f async_slam_toolbox_node      2>/dev/null
pkill -9      -f robot_state_publisher        2>/dev/null
pkill -9      -f "ros2 launch slam_bringup slam_2d" 2>/dev/null
pkill -9      -f "ros2 launch slam_bringup slam"    2>/dev/null

exec ros2 launch slam_bringup slam_2d.launch.py "$@"
