#!/usr/bin/env bash
# Force-kill the Nav2 stack and the pointcloud_to_laserscan helper.
# Symptoms this fixes: next start_nav.sh fails because /cmd_vel,
# /local_costmap, or lifecycle services already have publishers/servers
# from a survived previous run.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=kill_helpers.sh
source "$SCRIPT_DIR/kill_helpers.sh"

# Group the Nav2 servers into one alternation so we make one verification
# pass over the whole pipeline instead of N separate ones. Each pattern
# is anchored loosely (no \b on every term) since pkill -f matches the
# full command line and these names are distinctive.
NAV2_NODES='controller_server\|planner_server\|smoother_server\|behavior_server\|bt_navigator\|waypoint_follower\|velocity_smoother\|lifecycle_manager'

nuke_processes "$NAV2_NODES"                'Nav2 nodes'                 || exit 1
nuke_processes 'pointcloud_to_laserscan'    'pointcloud_to_laserscan'    || exit 1
nuke_processes 'static_transform_publisher.*camera_init_to_odom' 'camera_init→odom static TF' || true
nuke_processes 'ros2 launch slam_bringup nav2' 'nav2.launch.py wrapper'  || true
