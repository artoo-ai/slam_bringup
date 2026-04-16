#!/usr/bin/env bash
# Ctrl-C usually stops realsense2_camera_node cleanly, but occasionally
# the USB video device stays in a bad state — next start_d435.sh then
# fails with "Device or resource busy" or libuvc "couldn't resolve
# requests". Force a clean kill so the USB handle is released before
# the next launch.

pkill -SIGINT -f realsense2_camera_node          # try graceful first
sleep 2
pkill -9      -f realsense2_camera_node          # nuke if still alive
pkill -9      -f "ros2 launch slam_bringup d435" # and the launch wrapper
ros2 daemon stop                                 # clear stale DDS discovery cache
