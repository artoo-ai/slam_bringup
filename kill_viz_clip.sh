#!/usr/bin/env bash
# Force-kill the viz-clip republisher node + its launch wrapper.
# Run this if a previous start_viz_clip.sh / start_fast_lio.sh / start_slam.sh
# left a stray viz_z_clip behind — symptom is "node name already exists" on
# the next launch, or two publishers on /cloud_viz_clipped.

# Match the entry-point script (slam_bringup/viz_clip_node.py runs as
# `viz_clip` per setup.py) and the launch wrapper. Both patterns are
# covered separately because pkill matches against the full command line.
pkill -SIGINT -f "slam_bringup.viz_clip_node"             2>/dev/null
pkill -SIGINT -f "/viz_clip\b"                            2>/dev/null
pkill -SIGINT -f "ros2 launch slam_bringup viz_clip"      2>/dev/null
sleep 1
pkill -9      -f "slam_bringup.viz_clip_node"             2>/dev/null
pkill -9      -f "/viz_clip\b"                            2>/dev/null
pkill -9      -f "ros2 launch slam_bringup viz_clip"      2>/dev/null
exit 0
