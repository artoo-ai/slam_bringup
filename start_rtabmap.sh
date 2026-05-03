#!/usr/bin/env bash
# Start RTABMap graph SLAM + visual loop closure on top of FAST-LIO2.
# Idempotent: cleans a stale rtabmap process before relaunching.
#
# Pre-requisites (this script preflights all of them):
#   1. Sensors up in SLAM mode:
#        ./start_sensors.sh slam_mode:=true lidar_xfer_format:=1 enable_witmotion:=false
#   2. FAST-LIO2 running:
#        ./start_fast_lio.sh
#   3. body ↔ d435_front_link static TF available
#      (Phase 1.7 URDF, or start_bench_tf.sh as a stopgap)
#
# Pass-through args go to ros2 launch — e.g.:
#   ./start_rtabmap.sh delete_db_on_start:=true   # wipe the DB and start fresh
#   ./start_rtabmap.sh localization:=true          # reuse existing DB, no new mapping
#   ./start_rtabmap.sh viz:=true                   # spawn rtabmap_viz GUI

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash
# shellcheck source=start_helpers.sh
source "$SCRIPT_DIR/start_helpers.sh"
ensure_foxglove

if pgrep -f "rtabmap_slam/rtabmap" > /dev/null \
   || pgrep -f "ros2 launch slam_bringup rtabmap" > /dev/null; then
  echo "start_rtabmap: rtabmap already running — cleaning up first"
  "$SCRIPT_DIR/kill_rtabmap.sh"

  for _ in 1 2 3 4 5; do
    pgrep -f "rtabmap_slam/rtabmap" > /dev/null || break
    sleep 1
  done

  if pgrep -f "rtabmap_slam/rtabmap" > /dev/null; then
    echo "start_rtabmap: ERROR — rtabmap still running after kill_rtabmap.sh" >&2
    exit 1
  fi
fi

# Preflight: RTABMap needs FAST-LIO2 odometry, FAST-LIO2 body cloud, and
# D435 RGB+aligned-depth+camera_info all simultaneously, or it'll come up
# and silently wait forever for the missing one. Catch each here.
if [ "${START_RTABMAP_SKIP_PREFLIGHT:-0}" != "1" ]; then
  required_topics=(
    "/Odometry"
    "/cloud_registered_body"
    "/d435_front/camera/color/image_raw"
    "/d435_front/camera/aligned_depth_to_color/image_raw"
    "/d435_front/camera/color/camera_info"
  )

  missing=()
  for topic in "${required_topics[@]}"; do
    if ! ros2 topic info "$topic" 2>/dev/null | grep -q '^Publisher count: [1-9]'; then
      missing+=("$topic")
    fi
  done

  if [ ${#missing[@]} -gt 0 ]; then
    echo "start_rtabmap: ERROR — required input topics have no publisher:" >&2
    for t in "${missing[@]}"; do echo "    $t" >&2; done
    echo "" >&2
    echo "  Fix:" >&2
    echo "    1. ./start_sensors.sh slam_mode:=true lidar_xfer_format:=1 enable_witmotion:=false" >&2
    echo "    2. ./start_fast_lio.sh" >&2
    echo "    3. (re-run this script)" >&2
    echo "" >&2
    echo "  If aligned_depth_to_color is the only missing one, sensors were" >&2
    echo "  started without slam_mode:=true. align_depth.enable defaults off." >&2
    exit 1
  fi

  # body → d435_front_link TF check. Without this, RTABMap fuses RGB-D
  # data at the wrong location and the map will look offset/skewed.
  # Allow start_bench_tf.sh's livox_frame → camera_link as a stopgap.
  if ! ros2 run tf2_ros tf2_echo body d435_front_link --timeout 2.0 \
         > /dev/null 2>&1 \
     && ! ros2 run tf2_ros tf2_echo body camera_link --timeout 2.0 \
         > /dev/null 2>&1; then
    echo "start_rtabmap: WARNING — no TF from body to d435_front_link (or camera_link)." >&2
    echo "  RTABMap will publish a map but the depth fusion will be at the wrong location." >&2
    echo "  Fix: launch perception with the URDF (Phase 1.7) or run ./start_bench_tf.sh." >&2
    echo "  Continuing anyway in 3 s — Ctrl-C to abort." >&2
    sleep 3
  fi

  # IR projector check. Default is on, but a one-line nag here means we
  # don't have to remember to verify it before every mapping run.
  # Whitewall / textureless-surface depth holes are the most common
  # silent-failure mode in indoor RTABMap mapping.
  emitter=$(ros2 param get /d435_front/camera depth_module.emitter_enabled 2>/dev/null || true)
  if [ -n "$emitter" ] && ! echo "$emitter" | grep -qE '(True|: 1)'; then
    echo "start_rtabmap: WARNING — D435 IR projector emitter appears to be OFF." >&2
    echo "  Without it, white walls and textureless surfaces produce Swiss-cheese" >&2
    echo "  depth → holes in the RTABMap occupancy grid. Verify with:" >&2
    echo "    ros2 param get /d435_front/camera depth_module.emitter_enabled" >&2
    echo "  To turn it on at runtime:" >&2
    echo "    ros2 param set /d435_front/camera depth_module.emitter_enabled 1" >&2
    echo "  Continuing anyway in 3 s — Ctrl-C to abort." >&2
    sleep 3
  fi
fi

exec ros2 launch slam_bringup rtabmap.launch.py "$@"
