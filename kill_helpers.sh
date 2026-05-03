#!/usr/bin/env bash
# Shared kill helpers for the slam_bringup teardown scripts.
#
# Why this exists: orphan ROS nodes (livox_ros_driver2_node,
# imu_units_g_to_ms2, fastlio_mapping, etc.) silently double-publish on
# their topics. Two publishers with slightly different timestamps on
# /livox/lidar or /livox/imu_ms2 cause FAST-LIO to flood
# "lidar loop back, clear buffer" and reject every scan. The orphans
# usually appear because a previous launch crashed mid-startup (e.g.
# nav2 missing) and Ctrl-C only reached the foreground process.
#
# This helper escalates through SIGINT → SIGKILL → sudo SIGKILL with
# verification at each step, so the kill_*.sh wrappers don't have to
# repeat the boilerplate or stop at "exit 1, please intervene." If the
# user has NOPASSWD sudo (or a fresh sudo cache), the orphan dies
# without intervention; if sudo prompts and the user declines, we
# report cleanly and the caller can decide what to do.
#
# Usage from a kill_*.sh script:
#
#     SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
#     # shellcheck source=kill_helpers.sh
#     source "$SCRIPT_DIR/kill_helpers.sh"
#
#     nuke_processes 'livox_ros_driver2_node' 'Mid-360 driver'
#     nuke_processes 'imu_units_node\|imu_units_g_to_ms2' 'IMU unit republisher'
#
# Returns 0 on clean kill, 1 if processes still survive after sudo.

# Escalating kill: SIGINT (graceful) → SIGKILL (firm) → sudo SIGKILL (last resort).
# Verifies after each step. Does NOT exit the calling script — return value
# is the caller's to act on (or ignore).
#
# Args:
#   $1  pgrep/pkill -f pattern (extended regex; can use \| for alternation)
#   $2  optional human-readable label for log output (defaults to pattern)
#
# Returns:
#   0  pattern matches no processes after the kill sequence
#   1  at least one process survived sudo SIGKILL — caller should bail
nuke_processes() {
  local pattern="$1"
  local label="${2:-$pattern}"

  # Nothing matching? Done.
  pgrep -f "$pattern" >/dev/null 2>&1 || return 0

  # 1) Graceful SIGINT, give nodes 2s to drain ROS DDS connections.
  pkill -SIGINT -f "$pattern" 2>/dev/null
  sleep 2
  pgrep -f "$pattern" >/dev/null 2>&1 || return 0

  # 2) SIGKILL — try a few times, since pkill -9 occasionally races
  #    against an exiting process and pgrep can flicker.
  local i
  for i in 1 2 3; do
    pkill -9 -f "$pattern" 2>/dev/null
    sleep 1
    pgrep -f "$pattern" >/dev/null 2>&1 || return 0
  done

  # 3) Still alive after pkill -9 means the process is owned by
  #    another user (root) or wedged in D-state. Try sudo SIGKILL
  #    non-interactively first; if that fails (no NOPASSWD, no cached
  #    creds), fall back to interactive sudo. If the user Ctrl-C's the
  #    prompt, we report and return 1.
  echo "  ! $label survived pkill -9 — escalating to sudo" >&2
  pgrep -af "$pattern" >&2

  if sudo -n true 2>/dev/null; then
    sudo -n pkill -9 -f "$pattern" 2>/dev/null
  else
    sudo pkill -9 -f "$pattern" 2>/dev/null
  fi
  sleep 1

  if pgrep -f "$pattern" >/dev/null 2>&1; then
    echo "  ✗ $label STILL running after sudo SIGKILL:" >&2
    pgrep -af "$pattern" >&2
    echo "    Manual intervention required (D-state? kernel hang?). Try \`sudo kill -9 <pid>\` or reboot." >&2
    return 1
  fi

  echo "  ✓ $label killed (via sudo)" >&2
  return 0
}
