#!/usr/bin/env bash
# Start the Intel RealSense D435 launch file.
# Idempotent: if the driver or its launch wrapper is already running,
# clean it up via kill_d435.sh first — re-launching against a stale
# USB handle can fail with "Device or resource busy" from libuvc.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash
# shellcheck source=start_helpers.sh
source "$SCRIPT_DIR/start_helpers.sh"
ensure_foxglove

if pgrep -f realsense2_camera_node > /dev/null \
   || pgrep -f "ros2 launch slam_bringup d435" > /dev/null; then
  echo "start_d435: driver already running — cleaning up first"
  "$SCRIPT_DIR/kill_d435.sh"

  # Wait up to 5s for the process to actually exit and USB handle to release
  for _ in 1 2 3 4 5; do
    pgrep -f realsense2_camera_node > /dev/null || break
    sleep 1
  done

  if pgrep -f realsense2_camera_node > /dev/null; then
    echo "start_d435: ERROR — realsense2_camera_node still running after kill_d435.sh" >&2
    exit 1
  fi
fi

exec ros2 launch slam_bringup d435.launch.py "$@"
