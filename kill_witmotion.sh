#!/usr/bin/env bash
# Ctrl-C usually stops our wt901c_imu Python node cleanly via the
# KeyboardInterrupt handler in main(), but if the process gets wedged
# (e.g. during a stuck serial.read() call) it can leave /dev/ttyUSB0
# held open. Symptoms on the next ros2 launch slam_bringup witmotion:
# "Device or resource busy" from pyserial, or two publishers on /imu/data.
# Clean kill:

pkill -SIGINT -f "slam_bringup.*wt901c_imu"           # try graceful first
sleep 2
pkill -9      -f "slam_bringup.*wt901c_imu"           # nuke if still alive
pkill -9      -f "ros2 launch slam_bringup witmotion" # and the launch wrapper
ros2 daemon stop                                       # clear stale DDS discovery cache
