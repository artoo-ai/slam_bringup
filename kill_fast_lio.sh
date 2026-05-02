#!/usr/bin/env bash
# Force-kill the FAST-LIO2 mapping node + its launch wrapper.
# Symptoms that this script fixes: next start_fast_lio.sh fails with
# "Failed to create publisher: topic /Odometry already exists" (means
# the previous fastlio_mapping survived Ctrl-C) or two publishers on
# /cloud_registered (means a duplicate process is silently active).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

pkill -SIGINT -f fastlio_mapping                 # try graceful first
sleep 2
pkill -9      -f fastlio_mapping                 # nuke if still alive
pkill -9      -f "ros2 launch slam_bringup fast_lio"   # and the launch wrapper

# Take down the viz-clip republisher start_fast_lio.sh spawned alongside.
# Safe to call even if it isn't running.
"$SCRIPT_DIR/kill_viz_clip.sh" 2>/dev/null

ros2 daemon stop                                 # clear stale DDS discovery cache
