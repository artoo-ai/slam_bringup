#!/usr/bin/env bash
# build.sh — rebuild + source + verify slam_bringup workspace packages.
#
# Usage:
#   ./build.sh                    default: build slam_bringup only (fast)
#   ./build.sh --all              build the whole workspace (vendor + slam_bringup)
#   ./build.sh --clean            wipe build/ install/ for target packages first
#   ./build.sh --clean --all      clean rebuild of the entire workspace
#   ./build.sh <pkg> [<pkg> ...]  build specific packages
#   ./build.sh -h | --help        show this help
#
# After a successful build the script runs a quick pkg/launch/config check.
# Sourcing the workspace in your own shell still has to happen manually:
#     source ~/slam_ws/install/setup.bash

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BOLD='\033[1m'
NC='\033[0m'

info() { echo -e "${BOLD}==>${NC} $*"; }
ok()   { echo -e "${GREEN}  ✓${NC} $*"; }
warn() { echo -e "${YELLOW}  !${NC} $*"; }
fail() { echo -e "${RED}  ✗${NC} $*"; }

ROS_DISTRO="${ROS_DISTRO:-humble}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Layout check — same as install.sh
if [ "$(basename "$(dirname "$SCRIPT_DIR")")" != "src" ] || [ ! -d "$WS_ROOT/src" ]; then
  fail "build.sh must live at <ws>/src/slam_bringup/build.sh (found $SCRIPT_DIR)."
  exit 1
fi

CLEAN=0
BUILD_ALL=0
PACKAGES=()
for arg in "$@"; do
  case "$arg" in
    --clean) CLEAN=1 ;;
    --all)   BUILD_ALL=1 ;;
    -h|--help)
      sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    -*) fail "Unknown flag: $arg"; exit 1 ;;
    *)  PACKAGES+=("$arg") ;;
  esac
done

if [ ${#PACKAGES[@]} -eq 0 ] && [ $BUILD_ALL -eq 0 ]; then
  PACKAGES=(slam_bringup)
fi

info "Workspace: $WS_ROOT"
info "ROS distro: $ROS_DISTRO"
if [ $BUILD_ALL -eq 1 ]; then
  info "Target: all workspace packages"
else
  info "Target: ${PACKAGES[*]}"
fi

# Clean
if [ $CLEAN -eq 1 ]; then
  if [ $BUILD_ALL -eq 1 ]; then
    info "Cleaning workspace (build/ install/ log/)"
    rm -rf "$WS_ROOT/build" "$WS_ROOT/install" "$WS_ROOT/log"
  else
    for pkg in "${PACKAGES[@]}"; do
      info "Cleaning $pkg (build + install)"
      rm -rf "$WS_ROOT/build/$pkg" "$WS_ROOT/install/$pkg"
    done
  fi
fi

# Source ROS2
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

# Build
cd "$WS_ROOT"
BUILD_ARGS=(--symlink-install --cmake-args -DROS_EDITION=ROS2 "-DDISTRO_ROS=${ROS_DISTRO}")
if [ $BUILD_ALL -eq 0 ]; then
  BUILD_ARGS+=(--packages-select "${PACKAGES[@]}")
fi

info "colcon build ${BUILD_ARGS[*]}"
if ! colcon build "${BUILD_ARGS[@]}"; then
  fail "colcon build failed — check $WS_ROOT/log/latest_build/ for details."
  exit 1
fi
ok "colcon build succeeded"

# Source workspace for verification
set +u
# shellcheck disable=SC1090
source "$WS_ROOT/install/setup.bash"
set -u

# Verification
echo
info "Package verification"
PKG_LIST="$(ros2 pkg list 2>/dev/null || true)"
FAILED=0

check_pkg() {
  local pkg="$1"
  local required="${2:-required}"
  if echo "$PKG_LIST" | grep -qx "$pkg"; then
    ok "$pkg"
  elif [ "$required" = "optional" ]; then
    warn "$pkg — not installed (optional)"
  else
    fail "$pkg — NOT found"
    FAILED=1
  fi
}

check_pkg slam_bringup
check_pkg livox_ros_driver2
check_pkg witmotion_ros
check_pkg fast_lio
check_pkg realsense2_camera
check_pkg rtabmap_ros optional
# Nav2 layer (start_nav.sh / nav2.launch.py). Required for navigation;
# the SLAM-only pipeline still works without them, but start_nav.sh
# fails at launch with "package 'pointcloud_to_laserscan' not found"
# or "package 'nav2_bringup' not found". Install with:
#   sudo apt install ros-humble-nav2-bringup ros-humble-pointcloud-to-laserscan
check_pkg nav2_bringup
check_pkg pointcloud_to_laserscan

# Launch + config file listings (live under install/ share dir)
PREFIX="$(ros2 pkg prefix slam_bringup 2>/dev/null || echo "")"
if [ -n "$PREFIX" ]; then
  echo
  info "slam_bringup launch files"
  LAUNCH_DIR="$PREFIX/share/slam_bringup/launch"
  if compgen -G "$LAUNCH_DIR/*.py" > /dev/null; then
    for launch in "$LAUNCH_DIR"/*.py; do
      ok "$(basename "$launch")"
    done
  else
    warn "No launch files installed yet"
  fi

  echo
  info "slam_bringup configs"
  CONFIG_DIR="$PREFIX/share/slam_bringup/config"
  if [ -d "$CONFIG_DIR" ] && compgen -G "$CONFIG_DIR/*" > /dev/null; then
    for cfg in "$CONFIG_DIR"/*; do
      ok "$(basename "$cfg")"
    done
  else
    warn "No config files installed yet"
  fi
fi

echo
if [ $FAILED -eq 0 ]; then
  ok "Build + verification complete."
  echo
  echo "Source the workspace in your current shell to use the new build:"
  echo "  source $WS_ROOT/install/setup.bash"
else
  fail "One or more required packages are missing — review output above."
  exit 1
fi
