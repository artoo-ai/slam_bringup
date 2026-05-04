#!/usr/bin/env bash
# install.sh — one-time setup for the slam_bringup workspace.
# Clones vendor drivers into the workspace, installs system deps, builds everything.
# Assumes: Ubuntu 22.04, ROS2 Humble already installed (/opt/ros/humble).
# Run from the slam_bringup repo root (anywhere under slam_ws/src/slam_bringup).

set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"

# Resolve workspace root — this script lives at slam_ws/src/slam_bringup/install.sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SRC_DIR="$WS_ROOT/src"

# Layout sanity check — catch clone-location mistakes before failing mid-build
if [ "$(basename "$(dirname "$SCRIPT_DIR")")" != "src" ] || [ ! -d "$SRC_DIR" ]; then
  PARENT_DIR="$(dirname "$SCRIPT_DIR")"
  REPO_NAME="$(basename "$SCRIPT_DIR")"
  cat <<EOF >&2

ERROR: install.sh is not in the expected workspace layout.

  Found:    $SCRIPT_DIR
  Expected: <ws_root>/src/$REPO_NAME/install.sh

A ROS2 workspace must have a src/ directory between the workspace root and
your package. You likely cloned directly into the workspace root.

Fix — move the clone into src/:

  cd $PARENT_DIR
  mkdir -p src
  mv $REPO_NAME src/
  cd src/$REPO_NAME
  ./install.sh

EOF
  exit 1
fi

echo "==> Workspace: $WS_ROOT"
echo "==> ROS distro: $ROS_DISTRO"

# ---------------------------------------------------------------------------
# System packages
# ---------------------------------------------------------------------------
echo "==> apt: base build tools"
sudo apt update
sudo apt install -y \
  git build-essential cmake \
  python3-colcon-common-extensions \
  python3-rosdep python3-vcstool \
  libudev-dev usbutils

# ---------------------------------------------------------------------------
# Intel RealSense — apt install is simpler than building librealsense2 from source
# ---------------------------------------------------------------------------
echo "==> apt: RealSense"
sudo apt install -y \
  ros-${ROS_DISTRO}-librealsense2* \
  ros-${ROS_DISTRO}-realsense2-camera \
  ros-${ROS_DISTRO}-realsense2-description

# ---------------------------------------------------------------------------
# SLAM stack — RTABMap, Nav2, CycloneDDS (required middleware for Mid-360)
# ---------------------------------------------------------------------------
echo "==> apt: SLAM stack (RTABMap, Nav2, CycloneDDS)"
sudo apt install -y \
  ros-${ROS_DISTRO}-rtabmap-ros \
  ros-${ROS_DISTRO}-navigation2 \
  ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-cyclonedds \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# ---------------------------------------------------------------------------
# Remote viewing — foxglove_bridge lets a MacBook / Foxglove Studio client
# connect to ROS2 topics over WebSocket (ws://<jetson-ip>:8765). Useful for
# headless Jetsons with no display attached.
#   ros2 launch foxglove_bridge foxglove_bridge_launch.xml
# ---------------------------------------------------------------------------
echo "==> apt: foxglove_bridge (remote viewing)"
sudo apt install -y \
  ros-${ROS_DISTRO}-foxglove-bridge

# ---------------------------------------------------------------------------
# Livox-SDK2 — C++ SDK that livox_ros_driver2 links against.
# MUST be installed before building livox_ros_driver2.
# ---------------------------------------------------------------------------
echo "==> Livox-SDK2"
if [ ! -d "$SRC_DIR/Livox-SDK2" ]; then
  cd "$SRC_DIR"
  git clone https://github.com/Livox-SDK/Livox-SDK2.git
  cd Livox-SDK2
  mkdir -p build && cd build
  cmake ..
  make -j"$(nproc)"
  sudo make install
  sudo ldconfig
else
  echo "  Livox-SDK2 already cloned, skipping"
fi

# ---------------------------------------------------------------------------
# Vendor ROS2 driver repos — cloned into src/ alongside slam_bringup
# ---------------------------------------------------------------------------
echo "==> Cloning vendor drivers"
cd "$SRC_DIR"

clone_if_missing() {
  local url="$1"
  local dir="$2"
  local branch="${3:-}"
  if [ ! -d "$dir" ]; then
    if [ -n "$branch" ]; then
      git clone --recursive -b "$branch" "$url" "$dir"
    else
      git clone --recursive "$url" "$dir"
    fi
  else
    if [ -n "$branch" ]; then
      echo "  $dir exists, ensuring branch $branch + updating submodules"
      (cd "$dir" && git fetch origin "$branch" && git checkout "$branch" && git pull --ff-only origin "$branch" && git submodule update --init --recursive)
    else
      echo "  $dir exists, updating submodules"
      (cd "$dir" && git submodule update --init --recursive)
    fi
  fi
}

clone_if_missing https://github.com/Livox-SDK/livox_ros_driver2.git      livox_ros_driver2
# witmotion_IMU_ros: default branch is ROS1 (catkin); ROS2 code lives on the `ros2` branch
clone_if_missing https://github.com/ElettraSciComp/witmotion_IMU_ros.git witmotion_ros       ros2
clone_if_missing https://github.com/Ericsii/FAST_LIO_ROS2.git            FAST_LIO_ROS2

# livox_ros_driver2 ships package_ROS1.xml + package_ROS2.xml (dual-distro repo).
# Upstream build.sh selects one before building; replicate that for colcon.
LIVOX_DIR="$SRC_DIR/livox_ros_driver2"
if [ -f "$LIVOX_DIR/package_ROS2.xml" ]; then
  echo "  livox_ros_driver2: selecting ROS2 package.xml + launch files"
  cp -f "$LIVOX_DIR/package_ROS2.xml" "$LIVOX_DIR/package.xml"
  if [ -d "$LIVOX_DIR/launch_ROS2" ]; then
    rm -rf "$LIVOX_DIR/launch"
    cp -rf "$LIVOX_DIR/launch_ROS2" "$LIVOX_DIR/launch"
  fi
fi

# realsense-ros + rtabmap_ros installed via apt above — no git clone needed

# ---------------------------------------------------------------------------
# rosdep — resolves ROS-level deps declared in every package.xml
# ---------------------------------------------------------------------------
echo "==> rosdep"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths "$SRC_DIR" --ignore-src -r -y \
  --rosdistro "$ROS_DISTRO"

# ---------------------------------------------------------------------------
# Serial port permissions for WitMotion (/dev/ttyUSB*)
# ---------------------------------------------------------------------------
if ! id -nG "$USER" | grep -qw dialout; then
  echo "==> Adding $USER to dialout group (log out/in required)"
  sudo usermod -aG dialout "$USER"
fi

# ---------------------------------------------------------------------------
# Yahboom YB-ERF01 udev rule — pin the CH340 serial chip to /dev/myserial
# so yahboom_bridge_node.py finds it regardless of plug order. The device
# enumerates as 1a86:7523 (CH340/CH341 USB-Serial). MODE=0666 plus dialout
# group above gives the bridge node read/write without sudo.
#
# IMPORTANT: if the system has more than one CH340 (e.g. a debug adapter,
# the YB-ERF01's micro-USB port enumerating alongside its USB-C port, or
# anything else with a 1a86:7523), the simple idVendor/idProduct rule
# races and /dev/myserial randomly points at the wrong one. We resolve
# this by pinning to the YB-ERF01's USB port path via KERNELS, falling
# back to the broad rule only when there's a single CH340.
# ---------------------------------------------------------------------------
YAHBOOM_UDEV_FILE="/etc/udev/rules.d/99-yahboom.rules"
YAHBOOM_USB_PATH="${YAHBOOM_USB_PATH:-}"     # override via env, e.g. YAHBOOM_USB_PATH=1-2.1.3

# Count current CH340s so we know whether we need to pin by port path
CH340_COUNT=$(lsusb 2>/dev/null | grep -c "1a86:7523" || true)

if [ -z "$YAHBOOM_USB_PATH" ] && [ "$CH340_COUNT" -gt 1 ]; then
  echo "!!  Found $CH340_COUNT CH340 (1a86:7523) devices on the bus."
  echo "    The vendor/product-only udev rule will race between them."
  echo "    Identify the YB-ERF01's USB port path:"
  echo "      udevadm info -a /dev/ttyUSB0 | grep 'KERNELS==\"[0-9].*[0-9.]*\"' | head -1"
  echo "    …then re-run install.sh with:"
  echo "      YAHBOOM_USB_PATH=1-2.1.3 ./install.sh   # use your own path"
  echo "    Skipping udev rule install for now."
elif [ -n "$YAHBOOM_USB_PATH" ]; then
  YAHBOOM_UDEV_RULE='SUBSYSTEM=="tty", KERNELS=="'"$YAHBOOM_USB_PATH"'", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="myserial", MODE="0666"'
  if [ ! -f "$YAHBOOM_UDEV_FILE" ] || ! sudo grep -qF "$YAHBOOM_UDEV_RULE" "$YAHBOOM_UDEV_FILE"; then
    echo "==> Installing Yahboom udev rule pinned to USB path $YAHBOOM_USB_PATH"
    echo "$YAHBOOM_UDEV_RULE" | sudo tee "$YAHBOOM_UDEV_FILE" >/dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
  else
    echo "==> Yahboom udev rule already installed (pinned to $YAHBOOM_USB_PATH)"
  fi
else
  YAHBOOM_UDEV_RULE='SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="myserial", MODE="0666"'
  if [ ! -f "$YAHBOOM_UDEV_FILE" ] || ! sudo grep -qF "$YAHBOOM_UDEV_RULE" "$YAHBOOM_UDEV_FILE"; then
    echo "==> Installing Yahboom udev rule (single CH340 — broad match)"
    echo "$YAHBOOM_UDEV_RULE" | sudo tee "$YAHBOOM_UDEV_FILE" >/dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
  else
    echo "==> Yahboom udev rule already installed"
  fi
fi

# Soft USB-enumeration check — informational only (the board may not be
# plugged in during install). If it IS connected, verify /dev/myserial
# resolved correctly.
if ls /dev/serial/by-id/ 2>/dev/null | grep -q "1a86_USB_Serial"; then
  if [ -e /dev/myserial ]; then
    echo "==> Yahboom YB-ERF01 detected → /dev/myserial → $(readlink -f /dev/myserial)"
  else
    echo "!!  Yahboom USB-serial device present but /dev/myserial symlink missing."
    echo "    Try: sudo udevadm trigger; or unplug/replug the USB cable."
  fi
else
  echo "==> Yahboom YB-ERF01 not currently plugged in (skip — udev rule will fire on plug-in)"
fi

# ---------------------------------------------------------------------------
# Yahboom Rosmaster_Lib (mecanum drive bridge — vendored)
# ---------------------------------------------------------------------------
# Yahboom does not publish Rosmaster_Lib to PyPI. We vendor v3.3.9 in
# vendor/Rosmaster_Lib_3.3.9/ so this script can install it without an
# internet detour through Yahboom's Google Drive. Used by
# slam_bringup/yahboom_bringup_node.py — only needed on platforms that
# drive the Yahboom YB-ERF01 board (mecanum UGV today).
ROSMASTER_LIB_DIR="$SCRIPT_DIR/vendor/Rosmaster_Lib_3.3.9"
if [ -d "$ROSMASTER_LIB_DIR" ]; then
  if ! python3 -c "from Rosmaster_Lib import Rosmaster" >/dev/null 2>&1; then
    echo "==> Installing vendored Rosmaster_Lib from $ROSMASTER_LIB_DIR"
    pip3 install --user "$ROSMASTER_LIB_DIR"
  else
    echo "==> Rosmaster_Lib already installed"
  fi
else
  echo "==> Skipping Rosmaster_Lib install — vendor/ directory not found"
fi

# ---------------------------------------------------------------------------
# CycloneDDS as default ROS2 middleware (FastDDS chokes on Mid-360 cloud sizes)
# ---------------------------------------------------------------------------
if ! grep -q "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" ~/.bashrc; then
  echo "==> Appending CycloneDDS export to ~/.bashrc"
  echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ---------------------------------------------------------------------------
# Build the workspace
# ---------------------------------------------------------------------------
echo "==> colcon build --symlink-install"
cd "$WS_ROOT"
# ROS2's setup.bash references unbound vars (e.g. AMENT_TRACE_SETUP_FILES) on a
# fresh shell — relax `set -u` while sourcing, then restore.
set +u
source /opt/ros/${ROS_DISTRO}/setup.bash
set -u
# livox_ros_driver2 requires -DROS_EDITION=ROS2 -DDISTRO_ROS=<distro> to pick
# the humble/jazzy branch in its CMakeLists (upstream build.sh passes these).
# Other packages ignore unused cmake vars, so it's safe to pass globally.
# Upstream yahboom_rosmaster ships three packages that target Jazzy/Rolling's
# ros2_control + realtime_tools API and do not compile on Humble. Our stack
# uses Path A (direct Python Rosmaster_Lib bridge via yahboom_bridge_node.py),
# not ros2_control, so these packages are unused. Skip them. See
# docs/troubleshooting.md "build.sh --all fails on mecanum_drive_controller".
colcon build --symlink-install \
  --cmake-args -DROS_EDITION=ROS2 "-DDISTRO_ROS=${ROS_DISTRO}" \
  --packages-skip \
    mecanum_drive_controller \
    yahboom_rosmaster_navigation \
    yahboom_rosmaster_system_tests \
    yahboom_rosmaster

cat <<EOF

==> Install complete.

Next steps:
  source $WS_ROOT/install/setup.bash
  ros2 launch slam_bringup perception.launch.py platform:=go2

If you were just added to the dialout group, log out and back in before
running witmotion.launch.py (it needs /dev/ttyUSB* access).
EOF
