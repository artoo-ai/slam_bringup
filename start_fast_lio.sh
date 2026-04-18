#!/usr/bin/env bash
# Start FAST-LIO2 mapping against the Mid-360 + its onboard IMU.
# Idempotent: cleans a stale fastlio_mapping process before relaunching.
# Ctrl-C is usually clean, but if the node gets stuck in its TBB worker
# pool (rare but seen during long heavy-motion runs) it can leave the
# process holding the input topics in a way that confuses the next launch.
#
# Pre-requisite: Mid-360 must already be publishing in CustomMsg mode
# (xfer_format=1). Easiest way:
#   ./start_sensors.sh lidar_xfer_format:=1 enable_d435:=false enable_witmotion:=false
# (D435 + WitMotion are not used by FAST-LIO2 itself — drop them to free
# CPU during pose-quality bring-up tests.)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash

if pgrep -f fastlio_mapping > /dev/null \
   || pgrep -f "ros2 launch slam_bringup fast_lio" > /dev/null; then
  echo "start_fast_lio: fastlio_mapping already running — cleaning up first"
  "$SCRIPT_DIR/kill_fast_lio.sh"

  for _ in 1 2 3 4 5; do
    pgrep -f fastlio_mapping > /dev/null || break
    sleep 1
  done

  if pgrep -f fastlio_mapping > /dev/null; then
    echo "start_fast_lio: ERROR — fastlio_mapping still running after kill_fast_lio.sh" >&2
    exit 1
  fi
fi

exec ros2 launch slam_bringup fast_lio.launch.py "$@"
