#!/usr/bin/env bash
# Force-kill the FAST-LIO2 mapping node + its launch wrapper.
# Symptoms that this script fixes: next start_fast_lio.sh fails with
# "Failed to create publisher: topic /Odometry already exists" (means
# the previous fastlio_mapping survived Ctrl-C) or two publishers on
# /cloud_registered (means a duplicate process is silently active).

pkill -SIGINT -f fastlio_mapping                 # try graceful first
sleep 2
pkill -9      -f fastlio_mapping                 # nuke if still alive
pkill -9      -f "ros2 launch slam_bringup fast_lio"   # and the launch wrapper
ros2 daemon stop                                 # clear stale DDS discovery cache
