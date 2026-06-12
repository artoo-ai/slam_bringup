#!/usr/bin/env bash
# Tear down the ENTIRE 2D exploration stack: explore layer, Nav2, slam_toolbox,
# rf2o, sensors, RViz — motors stopped first, fail closed with escalation.
#
# Used two ways:
#   - by start_explore_2d.sh as its idempotent pre-launch teardown and as the
#     Ctrl-C sweep (it skips the calling wrapper automatically), and
#   - MANUALLY, as the rescue when a shutdown left survivors:
#         ./kill_explore_2d.sh
#
# Order matters:
#   1. Motors first — the Yahboom board coasts on the last command.
#   2. LAUNCH PARENTS before node children. explore_lite runs with
#      respawn=true (the stall watchdog restarts it to clear its frontier
#      blacklist), so killing the node before its `ros2 launch` parent just
#      resurrects it — the source of "Ctrl-C three times then hunt with ps"
#      (field, 2026-06-12).
#   3. Layer kill scripts, then verify, then escalate (SIGKILL via
#      nuke_processes) on anything that survived.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=kill_helpers.sh
source "$SCRIPT_DIR/kill_helpers.sh"

# ── 1. Motors ───────────────────────────────────────────────
# SIGINT the bridge so destroy_node zeroes the board; drain; then command
# zero directly over /dev/myserial in case no bridge was running.
pkill -SIGINT -f 'slam_bringup.yahboom_bridge_node\|/yahboom_bridge' 2>/dev/null
sleep 2
python3 - <<'PYSTOP' 2>/dev/null || true
import time
try:
    from Rosmaster_Lib import Rosmaster
    bot = Rosmaster(com="/dev/myserial")
    bot.create_receive_threading()
    for _ in range(20):
        bot.set_car_motion(0.0, 0.0, 0.0)
        time.sleep(0.05)
except Exception:
    pass
PYSTOP

# ── 2. Launch parents (respawn sources), then wrappers ─────
# SIGINT the ros2 launch trees first: a dead parent can't respawn children.
pkill -SIGINT -f 'ros2 launch slam_bringup explore'  2>/dev/null
pkill -SIGINT -f 'ros2 launch slam_bringup slam_2d'  2>/dev/null
sleep 3

# Other start_explore_2d.sh wrappers — never ourselves or our caller.
for _pid in $(pgrep -f start_explore_2d.sh 2>/dev/null); do
  [ "$_pid" != "$$" ] && [ "$_pid" != "$PPID" ] && kill -9 "$_pid" 2>/dev/null
done

# Escalate on launch parents that ignored SIGINT.
nuke_processes 'ros2 launch slam_bringup explore'  'explore launch' || true
nuke_processes 'ros2 launch slam_bringup slam_2d'  'slam_2d launch' || true

# ── 3. Layer teardowns ──────────────────────────────────────
TEARDOWN_OK=1
"$SCRIPT_DIR/kill_explore.sh"  || TEARDOWN_OK=0
"$SCRIPT_DIR/kill_nav.sh"      || TEARDOWN_OK=0
"$SCRIPT_DIR/kill_nav_2d.sh"   || TEARDOWN_OK=0
"$SCRIPT_DIR/kill_rtabmap.sh"  || TEARDOWN_OK=0
"$SCRIPT_DIR/kill_fast_lio.sh" || TEARDOWN_OK=0
"$SCRIPT_DIR/kill_viz_clip.sh" || TEARDOWN_OK=0
"$SCRIPT_DIR/kill_sensors.sh"  || TEARDOWN_OK=0
nuke_processes 'rviz2' 'RViz'  || TEARDOWN_OK=0

# ── 4. Verify, escalate on survivors, verify again ─────────
_STACK_PAT='rf2o_laser_odometry_node\|async_slam_toolbox_node\|localization_slam_toolbox\|nav2_\|controller_server\|planner_server\|bt_navigator\|behavior_server\|smoother_server\|velocity_smoother\|waypoint_follower\|lifecycle_manager\|lib/explore_lite/explore\|explore_manager\|livox_to_scan\|pointcloud_to_laserscan\|ros2 launch slam_bringup'
_survivors() {
  pgrep -af "$_STACK_PAT" 2>/dev/null | grep -v 'start_explore_2d.sh\|kill_explore_2d.sh'
}

if [ -n "$(_survivors)" ]; then
  echo "==> Survivors after graceful teardown — escalating:" >&2
  _survivors | sed 's/^/      /' >&2
  nuke_processes "$_STACK_PAT" '2D explore stack' || true
fi

if [ -n "$(_survivors)" ]; then
  echo "==> ✗ kill_explore_2d: processes STILL running after escalation:" >&2
  _survivors | sed 's/^/      /' >&2
  exit 1
fi

[ "$TEARDOWN_OK" -ne 1 ] && \
  echo "==> Note: a kill_*.sh returned non-zero but no stack processes remain." >&2
echo "==> kill_explore_2d: stack is down, motors stopped."
exit 0
