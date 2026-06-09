#!/usr/bin/env bash
# Kill explore_lite + explore_manager processes.

pkill -SIGINT -f explore_manager 2>/dev/null
pkill -SIGINT -f 'explore_lite'  2>/dev/null
sleep 1
pkill -9 -f explore_manager      2>/dev/null
pkill -9 -f 'explore_lite'       2>/dev/null
pkill -9 -f "ros2 launch slam_bringup explore" 2>/dev/null
