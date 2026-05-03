#!/usr/bin/env bash
# Force-kill the FAST-LIO2 mapping node + its launch wrapper.
# Symptoms that this script fixes: next start_fast_lio.sh fails with
# "Failed to create publisher: topic /Odometry already exists" (means
# the previous fastlio_mapping survived Ctrl-C) or two publishers on
# /cloud_registered (means a duplicate process is silently active).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

pkill -SIGINT -f fastlio_mapping                 # try graceful first
pkill -SIGINT -f "imu_units_node\|imu_units_g_to_ms2"   # /livox/imu → /livox/imu_ms2 republisher
sleep 2
pkill -9      -f fastlio_mapping                 # nuke if still alive
pkill -9      -f "imu_units_node\|imu_units_g_to_ms2"
pkill -9      -f "ros2 launch slam_bringup fast_lio"   # and the launch wrapper

# Take down the viz-clip republisher start_fast_lio.sh spawned alongside.
# Safe to call even if it isn't running.
"$SCRIPT_DIR/kill_viz_clip.sh" 2>/dev/null

ros2 daemon stop                                 # clear stale DDS discovery cache

# Verify — a surviving fastlio_mapping or imu_units republisher is the #1
# cause of FAST-LIO's "lidar loop back, clear buffer" flood (two IMU
# republishers double-publish each /livox/imu_ms2 sample with duplicate
# timestamps; FAST-LIO's last_timestamp_lidar ratchets ahead of incoming
# scans and rejects them all).
LEAK=0
if pgrep -f fastlio_mapping >/dev/null; then
  echo "kill_fast_lio: WARNING — fastlio_mapping still running after pkill -9:" >&2
  pgrep -af fastlio_mapping >&2
  LEAK=1
fi
if pgrep -f "imu_units_node\|imu_units_g_to_ms2" >/dev/null; then
  echo "kill_fast_lio: WARNING — imu_units republisher still running after pkill -9:" >&2
  pgrep -af "imu_units_node\|imu_units_g_to_ms2" >&2
  LEAK=1
fi
if [ $LEAK -ne 0 ]; then
  echo "  Try: sudo pkill -9 -f fastlio_mapping; sudo pkill -9 -f imu_units" >&2
  exit 1
fi
