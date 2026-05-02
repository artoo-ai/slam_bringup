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

# Split args: viz_clip-related → viz_args (passed to viz_clip.launch.py),
# everything else → fl_args (passed to fast_lio.launch.py). Lets you do
#   ./start_fast_lio.sh viz_z_max:=4.5
# without ros2 launch warning about an unknown arg on the FAST-LIO side.
viz_args=()
fl_args=()
viz_disabled=0
for arg in "$@"; do
  case "$arg" in
    enable_viz_clip:=*|viz_z_min:=*|viz_z_max:=*|viz_input_topic:=*|viz_output_topic:=*)
      viz_args+=("$arg")
      [ "$arg" = "enable_viz_clip:=false" ] && viz_disabled=1
      ;;
    *)
      fl_args+=("$arg")
      ;;
  esac
done

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

# Preflight: FAST-LIO2 is useless without a Mid-360 publishing CustomMsg.
# The silent-failure mode is painful: the node comes up, subscribes to
# /livox/lidar and /livox/imu, and just waits forever with no errors — you
# notice later by staring at an empty Foxglove. Catch both failure modes
# here:
#   1) Mid-360 driver isn't running at all (no publisher on /livox/lidar).
#   2) Driver IS running but in PointCloud2 mode (xfer_format=0). FAST-LIO
#      expects livox_ros_driver2/CustomMsg and silently drops everything
#      else. Fix: relaunch sensors with lidar_xfer_format:=1.
# Set START_FAST_LIO_SKIP_PREFLIGHT=1 to bypass (e.g. bag replay).
if [ "${START_FAST_LIO_SKIP_PREFLIGHT:-0}" != "1" ]; then
  # Wait up to 10s for /livox/lidar to appear + settle; sensor bringup
  # often lags the user by a couple seconds.
  lidar_type=""
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    lidar_type=$(ros2 topic info /livox/lidar 2>/dev/null | awk -F': ' '/^Type:/ {print $2}')
    [ -n "$lidar_type" ] && break
    sleep 1
  done

  if [ -z "$lidar_type" ]; then
    echo "start_fast_lio: ERROR — /livox/lidar has no publisher." >&2
    echo "  The Mid-360 driver is not running. FAST-LIO2 needs Mid-360 + IMU data." >&2
    echo "  Fix: ./start_sensors.sh lidar_xfer_format:=1 enable_d435:=false enable_witmotion:=false" >&2
    exit 1
  fi

  # Publisher type must be CustomMsg (xfer_format=1). Note ros2 renders the
  # type as a python list when multiple endpoints disagree, e.g.
  # "['livox_ros_driver2/msg/CustomMsg', 'sensor_msgs/msg/PointCloud2']" —
  # that case means the publisher is fine but some subscriber (often
  # foxglove_bridge) is on the wrong type, which is harmless for FAST-LIO.
  # So match CustomMsg anywhere in the string, and fail only if it's absent.
  if ! echo "$lidar_type" | grep -q 'livox_ros_driver2/msg/CustomMsg'; then
    echo "start_fast_lio: ERROR — /livox/lidar is publishing type '$lidar_type'." >&2
    echo "  FAST-LIO2 requires livox_ros_driver2/CustomMsg (xfer_format=1)." >&2
    echo "  Mid-360 was likely started in PointCloud2 mode. Restart with:" >&2
    echo "    ./kill_sensors.sh && ./start_sensors.sh lidar_xfer_format:=1 enable_d435:=false enable_witmotion:=false" >&2
    exit 1
  fi

  # /livox/imu check is a cheap sanity: same driver, so if the lidar
  # publisher is up the IMU one almost always is too, but catching the
  # "wrong driver config" case is worth the one extra call.
  if ! ros2 topic info /livox/imu 2>/dev/null | grep -q '^Publisher count: [1-9]'; then
    echo "start_fast_lio: WARNING — /livox/imu has no publisher." >&2
    echo "  FAST-LIO2 will not converge without the Mid-360 onboard IMU. Continuing anyway." >&2
  fi

  # Verify the imu_units_g_to_ms2 entry_point is installed. fast_lio.launch.py
  # spawns this node to convert /livox/imu (units of g) to /livox/imu_ms2
  # (m/s²). If setup.py was changed without a clean colcon rebuild, the
  # entry_point isn't installed and the Node action silently fails — FAST-LIO
  # then subscribes to /livox/imu_ms2 with no publisher and produces no
  # /Odometry or /cloud_registered. Catch that here.
  if ! ros2 pkg executables slam_bringup 2>/dev/null | grep -q 'imu_units_g_to_ms2'; then
    echo "start_fast_lio: ERROR — slam_bringup imu_units_g_to_ms2 entry_point is not installed." >&2
    echo "  setup.py was updated but the workspace wasn't clean-rebuilt." >&2
    echo "  Fix: cd ~/slam_ws/src/slam_bringup && ./build.sh --clean && source ~/slam_ws/install/setup.bash" >&2
    exit 1
  fi
fi

# Spawn the viz-clip republisher in the background so /cloud_viz_clipped
# is available alongside FAST-LIO2 for top-down RViz/Foxglove views.
# Always pre-clean any stale instance — both start_fast_lio.sh and
# start_slam.sh launch one, so running them back-to-back must not collide
# on the duplicate node name.
if [ "$viz_disabled" -ne 1 ]; then
  "$SCRIPT_DIR/kill_viz_clip.sh" >/dev/null 2>&1
  ros2 launch slam_bringup viz_clip.launch.py "${viz_args[@]}" \
       > /tmp/viz_clip.log 2>&1 &
  echo "start_fast_lio: spawned viz_clip in background (log: /tmp/viz_clip.log)"
fi

exec ros2 launch slam_bringup fast_lio.launch.py "${fl_args[@]}"
