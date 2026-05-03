#!/usr/bin/env bash
# Force-kill the Nav2 stack and the pointcloud_to_laserscan helper.
# Symptoms this fixes: next start_nav.sh fails because /cmd_vel,
# /local_costmap, or lifecycle services already have publishers/servers
# from a survived previous run.

# Nav2 nodes spawned by navigation_launch.py:
pkill -SIGINT -f "controller_server"        2>/dev/null
pkill -SIGINT -f "planner_server"           2>/dev/null
pkill -SIGINT -f "smoother_server"          2>/dev/null
pkill -SIGINT -f "behavior_server"          2>/dev/null
pkill -SIGINT -f "bt_navigator"             2>/dev/null
pkill -SIGINT -f "waypoint_follower"        2>/dev/null
pkill -SIGINT -f "velocity_smoother"        2>/dev/null
pkill -SIGINT -f "lifecycle_manager"        2>/dev/null
pkill -SIGINT -f "pointcloud_to_laserscan"  2>/dev/null
sleep 2

pkill -9 -f "controller_server"             2>/dev/null
pkill -9 -f "planner_server"                2>/dev/null
pkill -9 -f "smoother_server"               2>/dev/null
pkill -9 -f "behavior_server"               2>/dev/null
pkill -9 -f "bt_navigator"                  2>/dev/null
pkill -9 -f "waypoint_follower"             2>/dev/null
pkill -9 -f "velocity_smoother"             2>/dev/null
pkill -9 -f "lifecycle_manager"             2>/dev/null
pkill -9 -f "pointcloud_to_laserscan"       2>/dev/null
pkill -9 -f "static_transform_publisher.*camera_init_to_odom" 2>/dev/null
pkill -9 -f "ros2 launch slam_bringup nav2"                    2>/dev/null
