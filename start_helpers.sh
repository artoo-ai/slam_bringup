#!/usr/bin/env bash
# Shared helpers sourced by every start_*.sh script in this repo.
#
# Why this file exists:
#   Foxglove Studio (running on the dev machine) is the canonical
#   off-board viewer for this stack — Jetson-side RViz eats CPU and
#   slows everything else down. Foxglove only works if foxglove_bridge
#   is running on the Jetson, so every start_*.sh ensures the bridge
#   is up before doing its main launch. This file holds that ensure
#   function so we don't repeat the same 8 lines in 13 scripts.

# ensure_foxglove — idempotent foxglove_bridge bring-up.
#
# Spawns ros2 launch foxglove_bridge in the background (nohup, detached
# from the calling script's process group so a Ctrl-C on the parent
# doesn't take down Foxglove). If foxglove_bridge is already running,
# does nothing.
#
# Side effects:
#   /tmp/foxglove_bridge.log gets the bridge's stdout/stderr.
#
# Disable via environment for headless / CI contexts where you don't
# want a viewer:
#   SLAM_NO_FOXGLOVE=1 ./start_slam.sh
ensure_foxglove() {
  if [ "${SLAM_NO_FOXGLOVE:-0}" = "1" ]; then
    return 0
  fi
  if pgrep -f 'foxglove_bridge' >/dev/null 2>&1; then
    return 0
  fi
  echo "==> Starting foxglove_bridge in background (ws://<host>:8765, log: /tmp/foxglove_bridge.log)"
  # `setsid` detaches from the parent's process group so the bridge
  # survives Ctrl-C of the foreground launch.
  setsid nohup ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
      >/tmp/foxglove_bridge.log 2>&1 < /dev/null &
  disown 2>/dev/null || true
  # Give it a beat to bind the WebSocket port.
  sleep 1
}
