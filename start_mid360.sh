#!/usr/bin/env bash
# Start the Livox Mid-360 launch file

source /opt/ros/humble/setup.bash 
source ~/slam_ws/install/setup.bash
ros2 launch slam_bringup mid360.launch.py
