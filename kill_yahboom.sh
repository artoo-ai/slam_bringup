#!/usr/bin/env bash
# Force-kill the Yahboom cmd_vel bridge.
# Symptoms this fixes: next start_yahboom.sh fails with "serial port
# already open" (Rosmaster_Lib's serial handle survived from the previous
# run), or two yahboom_bridge processes both writing to /dev/myserial
# (race; motors get inconsistent commands and the rover lurches).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=kill_helpers.sh
source "$SCRIPT_DIR/kill_helpers.sh"

# Match both the entry-point name (slam_bringup/yahboom_bridge_node.py
# installs as console_script `yahboom_bridge`) and the launch wrapper.
nuke_processes 'slam_bringup.yahboom_bridge_node\|/yahboom_bridge\b' 'Yahboom bridge'         || exit 1
nuke_processes 'ros2 launch slam_bringup yahboom'                    'yahboom.launch wrapper' || true
