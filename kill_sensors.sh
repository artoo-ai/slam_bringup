#!/usr/bin/env bash
# Kill the combined sensors launch + every per-sensor driver it spawned.
# Each per-sensor kill_*.sh handles its driver's specific cleanup quirks
# (Livox UDP sockets, RealSense USB handle, WitMotion serial port);
# we just chain them and then nuke the parent ros2-launch wrapper.
# Trailing `ros2 daemon stop` clears stale DDS discovery — every per-sensor
# script does this too, but doing it once at the end is harmless and
# guarantees the cache is clean even if one of the chained scripts errored.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

"$SCRIPT_DIR/kill_mid360.sh"     2>/dev/null
"$SCRIPT_DIR/kill_d435.sh"       2>/dev/null
"$SCRIPT_DIR/kill_witmotion.sh"  2>/dev/null

pkill -9 -f "ros2 launch slam_bringup sensors" 2>/dev/null
ros2 daemon stop                                2>/dev/null
