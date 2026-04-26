#!/usr/bin/env bash
# Force-kill the RTABMap node + its launch wrapper.
# Symptoms this script fixes: next start_rtabmap.sh fails because /map or
# /mapData already has a publisher (previous rtabmap survived Ctrl-C), or
# the rtabmap.db is locked by an orphan process.

pkill -SIGINT -f "rtabmap_slam/rtabmap"          # graceful first
sleep 2
pkill -9      -f "rtabmap_slam/rtabmap"          # nuke if still alive
pkill -9      -f "ros2 launch slam_bringup rtabmap"   # and the launch wrapper
ros2 daemon stop                                 # clear stale DDS discovery cache
