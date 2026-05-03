#!/usr/bin/env bash
# Start the foxglove_bridge so dev-machine Foxglove Studio can connect.
#
# Foxglove is the canonical off-board viewer for this stack — every other
# start_*.sh auto-spawns the bridge via `ensure_foxglove` (see
# start_helpers.sh) so you generally don't need to run this manually.
# Direct use:
#
#   ./start_foxglove.sh                # idempotent — does nothing if already up
#   SLAM_NO_FOXGLOVE=0 ./start_foxglove.sh   # force-bring-up even if disabled
#   pkill -f foxglove_bridge           # take it down

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash
# shellcheck source=start_helpers.sh
source "$SCRIPT_DIR/start_helpers.sh"

ensure_foxglove

if pgrep -f foxglove_bridge >/dev/null 2>&1; then
  echo "foxglove_bridge running. Connect from Foxglove Studio:"
  echo "  ws://$(hostname).local:8765"
  echo "Log: /tmp/foxglove_bridge.log"
fi
