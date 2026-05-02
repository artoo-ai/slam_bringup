#!/usr/bin/env bash
# Force-kill the viz-clip composable-node container + its launch wrapper.
# Run this if a previous start_viz_clip.sh / start_fast_lio.sh / start_slam.sh
# left a stray viz_clip_container behind — symptom is "node name already
# exists" on the next launch, or two publishers on /cloud_viz_clipped.

pkill -SIGINT -f viz_clip_container                       2>/dev/null
pkill -SIGINT -f "ros2 launch slam_bringup viz_clip"      2>/dev/null
sleep 1
pkill -9      -f viz_clip_container                       2>/dev/null
pkill -9      -f "ros2 launch slam_bringup viz_clip"      2>/dev/null
exit 0
