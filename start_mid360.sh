#!/usr/bin/env bash
# Start the Livox Mid-360 launch file.
# Idempotent: if the driver or its launch wrapper is already running,
# clean it up via kill_mid360.sh first — Ctrl-C does not always fully
# stop the driver, and a second launch against a still-bound socket
# produces duplicate publishers or silent bind failures.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash
# shellcheck source=start_helpers.sh
source "$SCRIPT_DIR/start_helpers.sh"
ensure_foxglove

if pgrep -f livox_ros_driver2_node > /dev/null \
   || pgrep -f "ros2 launch slam_bringup mid360" > /dev/null; then
  echo "start_mid360: driver already running — cleaning up first"
  "$SCRIPT_DIR/kill_mid360.sh"

  # Wait up to 5s for the process to actually exit and sockets to release
  for _ in 1 2 3 4 5; do
    pgrep -f livox_ros_driver2_node > /dev/null || break
    sleep 1
  done

  if pgrep -f livox_ros_driver2_node > /dev/null; then
    echo "start_mid360: ERROR — livox_ros_driver2_node still running after kill_mid360.sh" >&2
    exit 1
  fi
fi

exec ros2 launch slam_bringup mid360.launch.py "$@"
