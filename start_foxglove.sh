#!/usr/bin/env bash
# Start foxglove bridge

source /opt/ros/humble/setup.bash 
source ~/slam_ws/install/setup.bash


ros2 launch foxglove_bridge foxglove_bridge_launch.xml