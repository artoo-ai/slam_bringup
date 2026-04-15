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
  if [ ! -d "$dir" ]; then
    git clone "$url" "$dir"
  else
    echo "  $dir exists, skipping"
  fi
}

clone_if_missing https://github.com/Livox-SDK/livox_ros_driver2.git   livox_ros_driver2
clone_if_missing https://github.com/ElettraSciComp/witmotion_IMU_ros.git witmotion_ros

# FAST-LIO2 (ROS2 port by Ericsii) — needs submodules
if [ ! -d "FAST_LIO_ROS2" ]; then
  git clone --recursive https://github.com/Ericsii/FAST_LIO_ROS2.git
else
  echo "  FAST_LIO_ROS2 exists, updating submodules"
  (cd FAST_LIO_ROS2 && git submodule update --init --recursive)
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
colcon build --symlink-install

cat <<EOF

==> Install complete.

Next steps:
  source $WS_ROOT/install/setup.bash
  ros2 launch slam_bringup perception.launch.py platform:=go2

If you were just added to the dialout group, log out and back in before
running witmotion.launch.py (it needs /dev/ttyUSB* access).
EOF
