# slam_bringup — Implementation Plan

Source note: `rico/Robotics/Nvidia Jetson/Jeston Orin Nano Super/Jetson Orin Nano Go2 ROS2 Sensor Bringup.md` (Obsidian vault).

## 1. Overview

`slam_bringup` is a ROS2 Humble package that orchestrates a **portable sensor-rig SLAM stack** — one Jetson + one sensor set (Livox Mid-360, RealSense D435 front, D435i rear, WitMotion WT901) that moves between multiple mobile robots. The package contains **no node code** — it is a glue layer of launch files, configs, URDF, and rviz layouts that wire vendor drivers together. On top of the raw sensor bringup, this plan layers **FAST-LIO2** (LiDAR-inertial odometry) and **RTABMap** (visual loop closure + graph SLAM) to produce a working SLAM stack with persistent maps.

**Target platforms** (sensors identical across all; only mounting geometry differs):

| Platform | Role | `base_link` source |
|----------|------|---------------------|
| Unitree Go2 Pro | Quadruped | Published by Go2 SDK |
| Lifesize R2D2 | Animatronic → mobile | Published by URDF (no SDK) |
| Sharper Image Roboscout | Retro mobile toy | Published by URDF |
| Mecanum 4-wheel UGV | Omnidirectional rover | Published by URDF |

At launch, a `platform:=<name>` argument selects the per-platform URDF and any platform-specific parameters (`base_link → body` TF bridge offset for FAST-LIO2, Go2 control network enable, etc.).

**Jetson deployment model:** one Jetson + one sensor rig moves between platforms. One install, one build, many deployments — platform selected at launch time.

**Target hardware:** NVIDIA Jetson Orin Nano Super (8 GB, hostname `gizmo`), JetPack 6.x on NVMe SSD.

**Development workflow:** initial code is authored on Mac (`/Users/rico/Documents/Robots/slam/`, acting as the `slam_bringup` package root), committed to git, cloned onto the Jetson at `~/slam_ws/src/slam_bringup/`, and built with `colcon`. (Workspace directory was renamed from `go2_ws` to `slam_ws` to match the platform-agnostic scope; the source note references the old `go2_ws` name.)

## 2. Hardware Baseline

### 2.1 Sensor rig (shared across all platforms)

Frozen for the scope of this plan. Stretch swaps (D455, VLP-16, Unitree L1, Pandar40P, dual Mid-360, OAK-D Pro) are enumerated in §10 but deferred to separate plans.

| Sensor | Role | Interface | Notes |
|--------|------|-----------|-------|
| Livox Mid-360 | Primary LiDAR + built-in ICM40609 IMU for FAST-LIO2 | Ethernet, broadcast UDP | IP `192.168.1.202`, host dest `255.255.255.255` |
| RealSense D435 | Front depth + RGB | USB 3 | `d435_front` namespace |
| RealSense D435i | Rear depth + RGB | USB 3 | `d435_rear` namespace; built-in BMI055 IMU **disabled** (WitMotion is authoritative) |
| WitMotion WT901 | Redundant / backup IMU | USB serial (`/dev/ttyUSB*`) | 200 Hz; remapped to `/imu/data` |

**IMU authority:** FAST-LIO2 uses the **Mid-360 built-in ICM40609** on `/livox/imu` (rigid coupling guaranteed by the Livox housing; pre-calibrated extrinsic). WitMotion runs in parallel as a higher-grade backup for cross-checking and future fusion experiments.

### 2.2 Platform matrix

What varies per platform. Everything else is identical.

| Platform | URDF file | `base_link` origin | `base_link → sensor_plate` | Go2 `eth0` network | `base_link → body` bridge (m) |
|----------|-----------|---------------------|-----------------------------|--------------------|--------------------------------|
| Go2 | `urdf/go2.urdf.xacro` | Go2 SDK `base_link` | Sensor plate above Go2 body | **Required** (`192.168.123.0/24`) | Measured (e.g., `0.10 0.0 0.20`) |
| R2D2 | `urdf/r2d2.urdf.xacro` | URDF root | Sensor plate inside dome / torso | Not used | Measured |
| Roboscout | `urdf/roboscout.urdf.xacro` | URDF root | Sensor plate on chassis top | Not used | Measured |
| Mecanum UGV | `urdf/mecanum.urdf.xacro` | URDF root (with 4 mecanum wheel TFs) | Sensor plate on chassis top | Not used | Measured |

All four platforms share:
- Same `install.sh` (one invocation on the Jetson)
- Same sensor launches (`mid360`, `d435`, `witmotion`, `sensors`)
- Same SLAM pipeline (FAST-LIO2 + RTABMap, one config)
- Same `eth1` LiDAR network (`192.168.1.0/24`)
- Same `record_sensors.sh` and Phase 2.5 workflow

## 3. Architecture

### 3.1 Package layout

```
~/slam_ws/src/
├── slam_bringup/                      # this package — all configs live here
│   ├── launch/
│   │   ├── mid360.launch.py          # direct Node() — NOT IncludeLaunchDescription
│   │   ├── d435.launch.py            # wraps realsense2_camera (front + rear, slam_mode arg)
│   │   ├── witmotion.launch.py       # direct Node()
│   │   ├── sensors.launch.py         # includes all 3 raw sensors
│   │   ├── perception.launch.py      # sensors + TFs + rviz (platform arg)
│   │   ├── fast_lio.launch.py        # FAST-LIO2 odometry
│   │   ├── rtabmap.launch.py         # RTABMap graph SLAM + loop closure
│   │   └── slam.launch.py            # full SLAM stack (platform arg)
│   ├── config/
│   │   ├── mid360.json
│   │   ├── d435.yaml
│   │   ├── witmotion.yaml
│   │   └── fast_lio_mid360.yaml      # FAST-LIO2 tuned for Mid-360 built-in IMU
│   │   # Per-platform base_link → body bridge offsets live inline in slam.launch.py
│   │   # (PLATFORM_BRIDGES dict). See §7.5. Move to config/platforms/*.yaml if it grows.
│   ├── urdf/
│   │   ├── sensors_common.urdf.xacro # shared sensor-to-sensor TFs (livox_frame → imu_link, etc.)
│   │   ├── go2.urdf.xacro            # includes sensors_common, adds Go2-specific base_link + plate mount
│   │   ├── r2d2.urdf.xacro           # includes sensors_common, R2D2 body + plate mount
│   │   ├── roboscout.urdf.xacro      # includes sensors_common, Roboscout chassis + plate mount
│   │   └── mecanum.urdf.xacro        # includes sensors_common, mecanum chassis + 4 wheel frames
│   ├── rviz/
│   │   └── perception.rviz
│   ├── scripts/
│   │   └── record_sensors.sh         # MCAP bag recorder (Phase 2.5)
│   ├── foxglove/
│   │   └── default_layout.json       # Foxglove Studio saved layout (Phase 2.5)
│   ├── install.sh                    # one-shot dependency + build script
│   ├── package.xml
│   └── setup.py                      # ament_python — must install config/ + urdf/ via data_files
│
├── livox_ros_driver2/                # vendor (git clone)
├── witmotion_ros/                    # vendor (git clone)
└── FAST_LIO_ROS2/                    # vendor (Ericsii/FAST_LIO_ROS2, --recursive)
```

RealSense and RTABMap are apt-installed, not git-cloned.

### 3.2 Launch hierarchy

All top-level launches accept `platform:=go2|r2d2|roboscout|mecanum` (default `go2`) to select the URDF and platform-specific config.

```
perception.launch.py  platform:=<name>    ← dev laptop / RViz workflow
├── robot_state_publisher  (loads urdf/<platform>.urdf.xacro)
├── rviz2
└── sensors.launch.py
    ├── mid360.launch.py      → livox_ros_driver2_node
    ├── d435.launch.py        → realsense2_camera (front + optional rear)
    └── witmotion.launch.py   → witmotion_ros_node

slam.launch.py  platform:=<name>          ← Jetson / full SLAM workflow
├── robot_state_publisher  (loads urdf/<platform>.urdf.xacro)
├── mid360.launch.py (xfer_format:=1, CustomMsg for FAST-LIO2)
├── d435.launch.py (slam_mode:=true, align_depth on, own pointcloud off)
├── witmotion.launch.py
├── static_transform_publisher (base_link → body, offsets from config/platforms/<platform>.yaml)
├── fast_lio.launch.py
└── rtabmap.launch.py
```

### 3.3 TF tree

The tree below is platform-agnostic. The `base_link → sensor_plate` and `base_link → d435_*_link` edges are measured per platform and encoded in each `urdf/<platform>.urdf.xacro`. The `livox_frame → imu_link` edge is shared across platforms (sensor rig is rigidly coupled regardless of what it mounts to).

```
base_link                    (published by Go2 SDK on Go2; by robot_state_publisher on R2D2/Roboscout/mecanum)
├── sensor_plate             (rubber-isolated aluminum plate; offset is per-platform)
│   ├── livox_frame          (Mid-360 bolted to plate — shared geometry)
│   │   └── imu_link         (WitMotion — child of livox_frame, NOT base_link — shared geometry)
│   └── (plate-relative joints)
├── d435_front_link          (separate mount; offset per platform)
├── d435_rear_link           (separate mount, rpy yaw 180°; offset per platform)
└── body                     (FAST-LIO2 output frame; bridged via static TF from per-platform config)
    └── (camera_init ← map, published by FAST-LIO2/RTABMap)
```

`imu_link` is a child of `livox_frame` (not `base_link`) because they physically share the sensor plate — this encodes the rigid coupling FAST-LIO2 / MOLA LO expect, so the IMU↔LiDAR transform reduces to a pure translation.

### 3.4 SLAM pipeline

```
Mid-360 LiDAR+IMU ──→ livox_ros_driver2 ─┬─→ /livox/lidar (CustomMsg, xfer_format=1)
                                         └─→ /livox/imu
                                                 │
                                                 ▼
                                         ┌─────────────┐
                                         │  FAST-LIO2  │  LiDAR-inertial EKF, ~10 Hz
                                         └─────┬───────┘
                                               │ /Odometry
                                               │ /cloud_registered_body
                                               │ TF: camera_init → body
                                               ▼
D435 RGB+Depth ──→ realsense2_camera ──→ ┌─────────────┐
    (slam_mode=true →                    │   RTABMap   │  graph SLAM + loop closure
     align_depth on,                     └─────┬───────┘
     own pointcloud off)                       │ /map, /octomap_full, /mapData
                                               ▼
                                               Nav2 (future)
```

### 3.5 Networking

The Jetson uses up to **two Ethernet interfaces** (built-in + USB-to-GbE adapter, Realtek RTL8153 chipset). `eth1` (LiDAR) is **always required**; `eth0` (Go2 control) is only required when the rig is mounted on the Go2.

| Interface | Subnet | Purpose | Required on |
|-----------|--------|---------|-------------|
| `eth0` (built-in RJ45) | `192.168.123.0/24` | Go2 internal network, robot control | **Go2 only** — idle on R2D2/Roboscout/mecanum (cable unplugged) |
| `eth1` (USB-to-GbE) | `192.168.1.0/24` | LiDAR network, Mid-360 broadcast | **All platforms** |

Configure both interfaces once on the Jetson (see §6.1). When the rig moves to a non-Go2 platform, just leave the `eth0` connection profile up and unplugged — it doesn't interfere with anything.

LiDAR IP assignments on `192.168.1.0/24`:

| LiDAR | IP | Default host | Port |
|-------|-----|--------------|------|
| Velodyne VLP-16 | `192.168.1.201` | `192.168.1.100` | UDP 2368 |
| Livox Mid-360 | `192.168.1.202` | `255.255.255.255` (broadcast) | UDP 56300 (point data) |
| Hesai Pandar40P | `192.168.1.203` | `192.168.1.100` | UDP 2368 |

## 4. Phase 0 — Prerequisites (done)

These are Jetson-level (not platform-level) prereqs. The Jetson lives on the sensor rig, so it's set up once regardless of which platform the rig is on.

- [x] Ubuntu 22.04 / JetPack 6.x on NVMe SSD on `gizmo`
- [x] ROS2 Humble at `/opt/ros/humble`
- [x] Sensor plate built; Mid-360 + D435 front + D435i rear + WitMotion WT901 physically mounted to the plate
- [x] Mid-360 pre-configured via Livox Viewer 2 on x86 machine: static IP `192.168.1.202`, host dest `255.255.255.255` (broadcast), ports 56100–56500

**Per-platform hardware prerequisites** (done as each platform is readied):

- [ ] Go2 — sensor plate hardware-mounted on the Go2 body via rubber-isolated bracket
- [ ] R2D2 — plate-mount point inside dome or torso; cable routing identified
- [ ] Roboscout — plate-mount bracket fitted to chassis top
- [ ] Mecanum UGV — plate-mount bracket on chassis top

## 5. Phase 0.5 — Repo & Mac↔Jetson Workflow

**Goal:** Initial code authored on Mac lives in a git repo cloned onto `gizmo` at `~/slam_ws/src/slam_bringup/`. Edits flow Mac → git remote → Jetson via `git pull` + `colcon build`.

### 5.1 — Initialize repo on Mac

- [ ] `cd /Users/rico/Documents/Robots/slam`
- [ ] `git init`
- [ ] Create `.gitignore`:

  ```gitignore
  # ROS2 build artifacts
  build/
  install/
  log/

  # Python
  __pycache__/
  *.pyc
  *.pyo

  # Editor
  .vscode/
  .idea/
  *.swp
  .DS_Store

  # Recordings (bags live on NVMe, not in repo)
  bags/

  # RTABMap database (grows with every run)
  *.db
  ```

- [ ] First commit: `git add . && git commit -m "initial plan"`

### 5.2 — Create remote + push

- [ ] Add remote (user provides URL — GitHub/GitLab/self-hosted/bare — whichever)
- [ ] `git remote add origin <remote-url>`
- [ ] `git push -u origin main`

### 5.3 — Clone on gizmo

- [ ] SSH in: `ssh gizmo`
- [ ] `mkdir -p ~/slam_ws/src && cd ~/slam_ws/src`
- [ ] `git clone <remote-url> slam_bringup`
- [ ] **Verify:** `ls ~/slam_ws/src/slam_bringup/` shows this repo's contents

### 5.4 — Sync loop

Standard rhythm for the rest of the project:

```bash
# ── On Mac ─────────────────────────────────────────────
# Edit launch/, config/, urdf/, etc.
git add -A && git commit -m "message"
git push

# ── On gizmo ───────────────────────────────────────────
ssh gizmo
cd ~/slam_ws/src/slam_bringup
git pull
cd ~/slam_ws
colcon build --packages-select slam_bringup --symlink-install
source install/setup.bash
ros2 launch slam_bringup <target>.launch.py
```

With `--symlink-install`, subsequent edits to launch files / configs are picked up without rebuilding — `install/` symlinks back to `src/`.

### Phase 0.5 Exit Criteria

- [ ] Repo pushed to remote
- [ ] `~/slam_ws/src/slam_bringup/` cloned on gizmo
- [ ] Round-trip tested: edit on Mac → push → pull on gizmo → file is present

## 6. Phase 1 — Raw Sensors Baseline

**Goal:** All three sensors publishing ROS2 topics, launchable as one command, visible in rviz with a connected TF tree.

### 6.1 — Workspace + dependencies (install.sh)

Runs once on the Jetson. Clones vendor drivers alongside `slam_bringup/`, builds Livox-SDK2, apt-installs RealSense + SLAM stack, runs `rosdep`, adds user to `dialout`, sets CycloneDDS env, does the first `colcon build`.

**`install.sh` (at `slam_bringup/install.sh`):**

```bash
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
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install

cat <<EOF

==> Install complete.

Next steps:
  source $WS_ROOT/install/setup.bash
  ros2 launch slam_bringup perception.launch.py

If you were just added to the dialout group, log out and back in before
running witmotion.launch.py (it needs /dev/ttyUSB* access).
EOF
```

**Tasks:**

- [ ] Repo already cloned via Phase 0.5 at `~/slam_ws/src/slam_bringup/`
- [ ] `cd ~/slam_ws/src/slam_bringup && chmod +x install.sh`
- [ ] Run `./install.sh`
- [ ] **Verify:** `rosdep install --from-paths ~/slam_ws/src --ignore-src -r -y` runs clean with no missing deps
- [ ] Configure Ethernet interfaces via nmcli:
  - [ ] **`eth1` LiDAR (all platforms):** `sudo nmcli con add type ethernet ifname eth1 con-name lidar ipv4.method manual ipv4.addresses 192.168.1.100/24`
  - [ ] **`eth0` Go2 control (Go2 only, still configure on the Jetson — harmless idle on non-Go2 platforms):** `sudo nmcli con add type ethernet ifname eth0 con-name go2 ipv4.method manual ipv4.addresses 192.168.123.XX/24` (pick a free `.XX`)
- [ ] **Verify:** `ping 192.168.1.202` responds from gizmo (Mid-360 reachable on `eth1`)
- [ ] **Verify:** `lsusb | grep Realtek` shows the USB-GbE adapter
- [ ] **Verify:** `dmesg | grep r8152` shows the driver loaded
- [ ] Confirm CycloneDDS export in `~/.bashrc`; log out/in or re-source: `source ~/.bashrc`
- [ ] **Verify:** `echo $RMW_IMPLEMENTATION` prints `rmw_cyclonedds_cpp`

### 6.2 — slam_bringup package skeleton

**`package.xml`:**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>slam_bringup</name>
  <version>0.1.0</version>
  <description>Portable sensor-rig SLAM stack (Mid-360 + D435 + WitMotion) for multiple mobile platforms</description>
  <maintainer email="rico@makeorcode.com">rico</maintainer>
  <license>MIT</license>

  <exec_depend>livox_ros_driver2</exec_depend>
  <exec_depend>realsense2_camera</exec_depend>
  <exec_depend>witmotion_ros</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>xacro</exec_depend>

  <export><build_type>ament_python</build_type></export>
</package>
```

**`setup.py`** — config and URDF files MUST be installed to the package share dir or `get_package_share_directory()` won't find them at runtime:

```python
from setuptools import setup
from glob import glob
import os

package_name = 'slam_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'),   glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'),   glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rico',
    maintainer_email='rico@makeorcode.com',
    description='Portable sensor-rig SLAM stack (Mid-360 + D435 + WitMotion) for multiple mobile platforms',
    license='MIT',
    entry_points={'console_scripts': []},
)
```

**Tasks:**

- [ ] Create directory structure inside `~/slam_ws/src/slam_bringup/`:
  - [ ] `package.xml` (above)
  - [ ] `setup.py` (above)
  - [ ] `resource/slam_bringup` (empty marker file for ament_python — `touch resource/slam_bringup`)
  - [ ] Empty `launch/`, `config/`, `urdf/`, `rviz/` directories
  - [ ] Empty package module: `mkdir slam_bringup && touch slam_bringup/__init__.py`
- [ ] Build: `cd ~/slam_ws && colcon build --packages-select slam_bringup --symlink-install`
- [ ] Source: `source install/setup.bash`
- [ ] **Verify:** `ros2 pkg list | grep slam_bringup` shows the package
- [ ] **Verify:** `ros2 pkg prefix slam_bringup` points into the install tree

### 6.3 — Mid-360 launch (standalone)

**Critical:** Livox's vendor launch files (`rviz_MID360_launch.py` / `msg_MID360_launch.py`) hardcode `user_config_path` — it is **not** a `DeclareLaunchArgument`, so passing `user_config_path:=...` through `IncludeLaunchDescription` is silently ignored. Instantiate the driver node directly to get full control.

**`config/mid360.json`** — Mid-360 static IP `192.168.1.202` on `192.168.1.0/24`. **Two sides to get right, don't confuse them:**

- **Lidar-side (configured once in Livox Viewer 2, stored on the device):** outbound destination set to broadcast `255.255.255.255` so the lidar's on-device config is portable across any network where the Jetson has a different IP — you don't re-flash the lidar when moving between robots.
- **Driver-side (this JSON's `host_net_info.*_ip`):** must be the **Jetson's real interface IP** (`192.168.1.100`, which §6.1 reserves as the fixed LiDAR-network host IP). The driver `bind()`s UDP listener sockets to these addresses — a socket cannot bind to `255.255.255.255`, and the driver silently fails to open its host-side listeners if you try, so no data ever arrives even though the init log prints "successfully enable Livox Lidar imu". Match the vendor sample at `livox_ros_driver2/config/MID360_config.json` which uses a concrete host IP. See `README.md` §5 for how to change this if the Jetson IP ever differs.

```json
{
  "lidar_summary_info": { "lidar_type": 8 },
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port":   56100,
      "push_msg_port":   56200,
      "point_data_port": 56300,
      "imu_data_port":   56400,
      "log_data_port":   56500
    },
    "host_net_info": {
      "cmd_data_ip":     "192.168.1.100",
      "cmd_data_port":   56101,
      "push_msg_ip":     "192.168.1.100",
      "push_msg_port":   56201,
      "point_data_ip":   "192.168.1.100",
      "point_data_port": 56301,
      "imu_data_ip":     "192.168.1.100",
      "imu_data_port":   56401,
      "log_data_ip":     "",
      "log_data_port":   56501
    }
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.202",
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "extrinsic_parameter": {
        "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
        "x": 0, "y": 0, "z": 0
      }
    }
  ]
}
```

**`launch/mid360.launch.py`:**

```python
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = Path(get_package_share_directory('slam_bringup'))
    default_config = bringup_share / 'config' / 'mid360.json'

    config_arg = DeclareLaunchArgument(
        'user_config_path',
        default_value=str(default_config),
        description='Absolute path to MID360_config.json (lives in slam_bringup/config/)',
    )
    frame_id_arg = DeclareLaunchArgument('frame_id', default_value='livox_frame')
    xfer_format_arg = DeclareLaunchArgument(
        'xfer_format',
        default_value='0',
        description='0 = PointCloud2 (RViz viewable), 1 = CustomMsg (FAST-LIO2)',
    )

    livox_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format':      LaunchConfiguration('xfer_format'),
            'multi_topic':      0,
            'data_src':         0,
            'publish_freq':     10.0,
            'output_data_type': 0,
            'frame_id':         LaunchConfiguration('frame_id'),
            'user_config_path': LaunchConfiguration('user_config_path'),
        }],
    )

    return LaunchDescription([config_arg, frame_id_arg, xfer_format_arg, livox_node])
```

**Tasks:**

- [ ] Create `config/mid360.json` exactly as above (do not widen or round any IP/port values)
- [ ] Create `launch/mid360.launch.py` with the direct `Node()` pattern
- [ ] Rebuild: `colcon build --packages-select slam_bringup --symlink-install`
- [ ] Launch: `ros2 launch slam_bringup mid360.launch.py`
- [ ] **Verify:** `ros2 topic list` shows `/livox/lidar` and `/livox/imu`
- [ ] **Verify:** `ros2 topic hz /livox/lidar` → ~10 Hz
- [ ] **Verify:** `ros2 topic hz /livox/imu` → ~200 Hz
- [ ] **Verify:** `ros2 launch slam_bringup mid360.launch.py xfer_format:=1` → topic type changes to `livox_ros_driver2/msg/CustomMsg`
- [ ] Stop with Ctrl-C

### 6.4 — D435 launch (dual camera with SLAM + single/dual switches)

Three independent launch arguments: `enable_front` (default `true`), `enable_rear` (default `false`), `slam_mode` (default `false`). Default is **front-only in raw mode** — add the second camera only when ready to measure bandwidth. `device_type_front` / `device_type_rear` are parameterized now so a D435→D455 swap later is a launch-arg change only.

We invoke `realsense2_camera_node` directly with `Node()` rather than including the vendor's `rs_launch.py`. `rs_launch.py` maintains a hard-coded `configurable_parameters` list and silently drops any launch argument not in it — notably `pointcloud__neon_.enable`, the ARM NEON-accelerated pointcloud filter name that realsense2_camera v4.57.7 actually declares on Jetson (see gotchas). Going direct also removes rs_launch.py's "Parameter not supported" spam for every top-level `DeclareLaunchArgument` (`slam_mode`, `enable_front`, etc.).

**`launch/d435.launch.py`:**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    slam_mode_arg = DeclareLaunchArgument(
        'slam_mode',
        default_value='false',
        description='true = align_depth on, own pointcloud off (for RTABMap); false = raw mode',
    )
    enable_front_arg = DeclareLaunchArgument('enable_front', default_value='true')
    enable_rear_arg  = DeclareLaunchArgument('enable_rear',  default_value='false')
    device_type_front_arg = DeclareLaunchArgument('device_type_front', default_value='d435',
        description='Front camera device_type (d435, d455 for outdoor swap)')
    device_type_rear_arg  = DeclareLaunchArgument('device_type_rear',  default_value='d435i')

    slam = LaunchConfiguration('slam_mode')

    # Raw mode: pointcloud ON (direct Foxglove/RViz use).
    # SLAM mode: pointcloud OFF (RTABMap builds its own), align_depth ON.
    pc_enable = PythonExpression(["'false' if '", slam, "' == 'true' else 'true'"])

    def realsense_node(namespace, device_type_arg):
        return Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace=namespace,
            name='camera',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_type':                LaunchConfiguration(device_type_arg),
                'enable_depth':               True,
                'enable_color':               True,
                'enable_infra':               False,  # left IR + right IR not needed —
                'enable_infra1':              False,  # bypassing rs_launch.py means we
                'enable_infra2':              False,  # inherit the node's own defaults
                'enable_sync':                True,
                'align_depth.enable':         slam,
                'enable_gyro':                False,  # D435 has no IMU; D435i's BMI055 is disabled —
                'enable_accel':               False,  # WitMotion is the authoritative IMU.
                'pointcloud__neon_.enable':   pc_enable,  # Jetson NEON filter name
                'pointcloud.enable':          pc_enable,  # x86 / generic name
                'depth_module.depth_profile': '848x480x30',
                'rgb_camera.color_profile':   '848x480x30',
            }],
        )

    front = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_front')),
        actions=[realsense_node('d435_front', 'device_type_front')],
    )
    rear = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_rear')),
        actions=[realsense_node('d435_rear', 'device_type_rear')],
    )

    return LaunchDescription([
        slam_mode_arg, enable_front_arg, enable_rear_arg,
        device_type_front_arg, device_type_rear_arg,
        front, rear,
    ])
```

Namespaces keep topics separate: `/d435_front/camera/depth/*` vs `/d435_rear/camera/depth/*`. The D435i rear's BMI055 is disabled explicitly — running two IMU streams confuses state estimators. IR streams (`enable_infra1` / `enable_infra2`) are disabled too; rs_launch.py turned them off by default, and when we bypass rs_launch.py we inherit the node's own defaults (which leave IR **on**), so we re-disable explicitly.

**Gotchas hit during bringup (realsense-ros 4.57.7 on Humble/Jetson):**

- `depth_module.profile` is the realsense-ros **3.x** arg name. In 4.x it silently falls back to the default with a "Parameter not supported" warning. Use `depth_module.depth_profile` + `rgb_camera.color_profile` — both set to `848x480x30` so depth/color are resolution-matched for `align_depth`.
- `PushRosNamespace('d435_front')` + `camera_name='d435_front'` produces triple-nested topics like `/d435_front/camera/d435_front/color/image_raw` because rs_launch.py's default `camera_namespace` is `'camera'`. With direct `Node(namespace='d435_front', name='camera')` we just get `/d435_front/camera/color/image_raw`.
- **rs_launch.py silently drops unknown launch args.** Line 119–125: it builds `supported_params = set(configurable_parameters)`, warns on any launch argument outside that set, and (crucially) only forwards the supported set to the node. Any custom param like `pointcloud__neon_.enable` passed through `IncludeLaunchDescription(launch_arguments={...})` is lost. The only safe ways around it are (a) invoke the node directly (what we do now), (b) pass a YAML via `config_file` (rs_launch.py merges YAML params into the node even if they're "unsupported"), or (c) `ros2 param set` after launch.
- **Pointcloud filter param name is `pointcloud__neon_.enable` on Jetson**, not `pointcloud.enable`. realsense2_camera v4.57.7 on ARM NEON declares the pointcloud filter under `pointcloud__neon_.*`; `pointcloud.enable` is not declared on this build and setting it is a silent no-op. x86 builds likely declare the canonical `pointcloud.enable`. We set **both** names in the params dict so whichever the running node declares wins. Verify with `ros2 param list /d435_front/camera | grep pointcloud` after launch — on Jetson you'll see `pointcloud__neon_.*`.
- `ros2 topic hz` on an Image topic doesn't work out-of-the-box: it subscribes with RELIABLE QoS while realsense publishes BEST_EFFORT — no QoS match, no rate. Verify rate via the matching `*/camera_info` topic (tiny messages, same publish cadence) or a QoS-matched rclpy subscriber.

**Tasks:**

- [x] Create `launch/d435.launch.py` exactly as above
- [x] Rebuild + source
- [x] Launch single (default, front only): `ros2 launch slam_bringup d435.launch.py`
- [x] **Verify:** `ros2 topic list` shows only `/d435_front/...` topics (no `/d435_rear/`)
- [x] **Verify:** color at ~30 Hz (`/d435_front/camera/color/camera_info` → 29.93 Hz measured)
- [x] **Verify:** depth at ~30 Hz (`/d435_front/camera/depth/camera_info` → 28.74 Hz measured)
- [ ] **Verify:** `jtop` → single-camera CPU ~10% (realsense node steady-state)

**Bench visualization with Mid-360:** Phase 1.7 will publish the full TF tree via `robot_state_publisher`, but until then `start_bench_tf.sh` (repo root) publishes a single `livox_frame → camera_link` static TF — enough for Foxglove / RViz to render `/livox/lidar` and `/d435_front/camera/depth/color/points` in the same 3D panel. Offsets default to rough Go2 figures from §6.7; edit in place and re-run (idempotent). Verified end-to-end on the bench: both point clouds align in a Foxglove 3D panel with `Display frame = livox_frame`.

### 6.5 — WitMotion launch

**Driver decision (changed from original plan):** the actual sensor plate uses a **WT901C** (USB-C variant) which emits the modern WT9011-style `0x55 0x61` combined accel+gyro+angle packet (20 bytes, no checksum). The ElettraSar/witmotion_ros driver we install via `install.sh` only registers legacy packet IDs `0x51`–`0x54` (`witmotion-uart-qt/include/witmotion/types.h:44-47`) and silently drops every byte from this device. Rather than patch a Qt-heavy upstream we don't otherwise need, we ship a small Python parser inside `slam_bringup` itself.

**Live capture proves the protocol:** at 115200 baud the device emits packets like
```
55 61 13 00 b2 01 dd 07 00 00 00 00 00 00 ab 08 9b ff 0a b7
```
where bytes 6–7 (LE) = `0x07dd` = 2013 → 2013/32768 × 16g = 0.984 g ≈ 9.64 m/s² on Z (gravity, sensor laid flat). Per the WT9011DCL/BWT901CL spec: `header(1) + type(1) + AccXYZ(3×i16) + GyroXYZ(3×i16) + AngleXYZ(3×i16)` = 20 bytes back-to-back, no trailing checksum.

**Pre-bringup gotchas (Jetson Orin Nano Super, JetPack 6 / kernel 5.15.185-tegra):**

1. **No `ch341.ko` in the L4T kernel.** `lsusb` enumerates the WT901C's CH340 (`1a86:7523`) but `/dev/ttyUSB*` never appears. The Tegra kernel only ships `cp210x.ko` + `ftdi_sio.ko` for USB-serial. Build the out-of-tree module:
   ```bash
   git clone https://github.com/juliagoda/CH341SER.git ~/CH341SER
   cd ~/CH341SER && make && sudo make install   # persistent across reboots
   ```
2. **`brltty` hijacks CH340 devices** (it thinks they're braille displays). Mask both services or the kernel module never gets to claim the device:
   ```bash
   sudo systemctl stop brltty.service brltty-udev.service
   sudo systemctl mask brltty.service brltty-udev.service
   ```
3. **`pyserial` is not installed by default.** `sudo apt install -y python3-serial`.
4. **WT901C ships at 10 Hz from the factory and ignores volatile rate writes** — the unlock+rate sequence has no effect unless followed by a SAVE (`0xFF 0xAA 0x00 0x00 0x00`) which writes EEPROM. We tested this: unlock+rate alone leaves the device at 10 Hz, unlock+rate+save → 200 Hz sustained. Our node defaults to `output_rate_hz: 0` (= don't reconfigure) and exposes a one-time `output_rate_hz:=200` override for explicit re-flash. EEPROM is rated ~10k writes — fine for occasional one-shot reconfiguration, not for every launch.
5. **Python entry-point scripts land in `bin/` not `lib/<pkg>/` by default.** `ros2 launch` looks in `lib/<pkg>/`, so without a `setup.cfg` redirecting `install_scripts`, every launch fails with `libexec directory ... does not exist`. Required:
   ```ini
   [develop]
   script_dir=$base/lib/slam_bringup
   [install]
   install_scripts=$base/lib/slam_bringup
   ```

**`config/witmotion.yaml`:** simple — port, baud, frame, topic, optional rate override (default 0 = trust EEPROM). See the file for the inline comments explaining each field.

**`launch/witmotion.launch.py`:** invokes our own `slam_bringup wt901c_imu` entry point with the yaml as `params_file`. No `name=` override (the C++ ElettraSar driver hardcodes its name; we follow the same convention to keep yaml resolution simple).

**Parser node:** `slam_bringup/slam_bringup/wt901c_imu_node.py` — pyserial reader, syncs on `0x55 0x61`, parses 18 bytes, converts angle triplet → quaternion, publishes `sensor_msgs/Imu` to `/imu/data`. Periodic stats log (`stats_period_sec`, default 5s) reports actual Hz and resync-byte count.

**Tasks:**

- [ ] Find stable serial path: `ls /dev/serial/by-id/` — note the WitMotion entry; update `config/witmotion.yaml` to use `/dev/serial/by-id/<id>` instead of `/dev/ttyUSB0` to survive reboots
- [x] Build `juliagoda/CH341SER` kernel module + mask `brltty` so `/dev/ttyUSB0` appears
- [x] `sudo apt install -y python3-serial`
- [x] Create `config/witmotion.yaml`, `launch/witmotion.launch.py`, `slam_bringup/wt901c_imu_node.py`, `setup.cfg`
- [x] Rebuild + source
- [x] One-time bring-up: `ros2 launch slam_bringup witmotion.launch.py output_rate_hz:=200` to burn 200 Hz to EEPROM
- [x] Launch: `ros2 launch slam_bringup witmotion.launch.py`
- [x] **Verify:** `ros2 topic hz /imu/data` → ~200 Hz (measured 198.1–198.7 Hz over multiple 5s windows, 0 resync drops)
- [x] **Verify:** `ros2 topic echo /imu/data --once` shows sane values — gravity ≈ 9.8 m/s² on Z-axis when sensor plate is level (measured `linear_acceleration.z = 9.64`, `|a| = 9.86`)

### 6.6 — sensors.launch.py integration

**`launch/sensors.launch.py`:**

```python
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_dir = Path(get_package_share_directory('slam_bringup')) / 'launch'

    def include(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / name))
        )

    return LaunchDescription([
        include('mid360.launch.py'),
        include('d435.launch.py'),
        include('witmotion.launch.py'),
    ])
```

**Tasks:**

**As-built notes (deviations from the snippet above):**

- The minimal `IncludeLaunchDescription`-only composition above became **`GroupAction(condition=IfCondition(...))`-wrapped includes** so each sensor can be skipped at launch time (`enable_mid360:=false`, etc.) without editing this file. Useful when isolating a single driver under load.
- Forwards three pass-through args to per-sensor launches: `slam_mode` → d435, `lidar_xfer_format` → mid360 (renamed from `xfer_format` for clarity at the composition level), `enable_rear` → d435. These are passed via explicit `launch_arguments={...}.items()` rather than relying on LaunchConfiguration auto-propagation, so the wiring is grep-able.
- Companion scripts: `start_sensors.sh` chains the per-sensor `kill_*.sh` for idempotent re-launch (each driver clings to its resource differently — UDP sockets, USB handle, serial port). `kill_sensors.sh` does the same teardown without re-launching.

**Tasks:**

- [x] Create `launch/sensors.launch.py`, `start_sensors.sh`, `kill_sensors.sh`
- [x] Rebuild + source
- [x] Launch: `ros2 launch slam_bringup sensors.launch.py` (or `./start_sensors.sh` for idempotent variant)
- [x] **Verify:** `ros2 topic list` shows `/livox/lidar`, `/livox/imu`, `/d435_front/camera/{color,depth}/*`, `/imu/data` from a single launch
- [x] **Verify:** publisher-side rates remain nominal — `/livox/imu` 199.7 Hz, wt901c_imu's internal stats log shows 198.5 Hz sustained on `/imu/data`, `/d435 color` ~29 Hz. *Caveat:* `ros2 topic hz` measured against the sensors-up Jetson while a desktop session was running (rviz2 + foxglove + rustdesk + chrome eating ~250% CPU, load avg 10.94/6) showed degraded subscriber-side rates on the bandwidth-heavy topics (`/livox/lidar` 7.6 vs 10, `/d435 depth` 23 vs 30). Re-run with the GUI tools closed for a clean measurement.
- [ ] **Verify:** `jtop` baseline → CPU ~30%, RAM ~500 MB *(pending — needs measurement on a clean Jetson with no GUI session)*

### 6.7 — URDF / TF tree (per-platform + shared sensor macro)

The Mid-360 and WitMotion are on a single rubber-isolated aluminum sensor plate. They are physically one rigid body — the URDF reflects this with `imu_link` as a child of `livox_frame`, so the extrinsic calibration between them is a pure translation (no rotation matrix). This sensor-to-sensor geometry is **shared across all platforms** and lives in one xacro macro. The `base_link → sensor_plate` and `base_link → d435_*_link` edges are **per-platform** and live in separate URDF files.

**`urdf/sensors_common.urdf.xacro`** — shared sensor rig geometry:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- Sensor-to-sensor geometry — identical regardless of which platform the
       sensor plate is mounted on. Consumers call this macro after they have
       defined base_link and a base_link → sensor_plate joint. -->
  <xacro:macro name="slam_sensors_rigid_group">

    <!-- Livox Mid-360 bolted to sensor plate -->
    <link name="livox_frame"/>
    <joint name="plate_to_livox" type="fixed">
      <parent link="sensor_plate"/>
      <child  link="livox_frame"/>
      <origin xyz="0.0 0.0 0.04" rpy="0 0 0"/>     <!-- Mid-360 sits ~40mm above plate -->
    </joint>

    <!-- WitMotion IMU on same plate, close to Livox center -->
    <!-- Pure translation from livox_frame — axes physically aligned -->
    <link name="imu_link"/>
    <joint name="livox_to_imu" type="fixed">
      <parent link="livox_frame"/>
      <child  link="imu_link"/>
      <origin xyz="-0.02 0.0 -0.04" rpy="0 0 0"/>  <!-- MEASURE with calipers once -->
    </joint>

  </xacro:macro>

</robot>
```

**`urdf/go2.urdf.xacro`** — Go2-specific (Go2 SDK publishes `base_link`):

```xml
<?xml version="1.0"?>
<robot name="slam_rig_go2" xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:include filename="$(find slam_bringup)/urdf/sensors_common.urdf.xacro"/>

  <!-- Declared here as a placeholder; Go2 SDK also publishes base_link at runtime.
       If the Go2 SDK publishes a different root (e.g., 'base'), rename accordingly
       or add a static_transform_publisher bridge. -->
  <link name="base_link"/>

  <!-- Rubber-isolated sensor plate offset on the Go2 body -->
  <link name="sensor_plate"/>
  <joint name="base_to_plate" type="fixed">
    <parent link="base_link"/>
    <child  link="sensor_plate"/>
    <origin xyz="0.10 0.0 0.20" rpy="0 0 0"/>    <!-- MEASURE Go2 base_link → plate center -->
  </joint>

  <!-- Mount the shared sensor group on the plate -->
  <xacro:slam_sensors_rigid_group/>

  <!-- D435 front — Go2-specific front mount -->
  <link name="d435_front_link"/>
  <joint name="base_to_d435_front" type="fixed">
    <parent link="base_link"/>
    <child  link="d435_front_link"/>
    <origin xyz="0.20 0.0 0.10" rpy="0 0 0"/>    <!-- MEASURE -->
  </joint>

  <!-- D435i rear — Go2-specific rear mount; yaw 180° so it faces backward -->
  <link name="d435_rear_link"/>
  <joint name="base_to_d435_rear" type="fixed">
    <parent link="base_link"/>
    <child  link="d435_rear_link"/>
    <origin xyz="-0.20 0.0 0.10" rpy="3.14159 0 0"/>
  </joint>

</robot>
```

**`urdf/r2d2.urdf.xacro`** — R2D2 has no external SDK publishing `base_link`, so `robot_state_publisher` publishes it from this URDF directly:

```xml
<?xml version="1.0"?>
<robot name="slam_rig_r2d2" xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:include filename="$(find slam_bringup)/urdf/sensors_common.urdf.xacro"/>

  <link name="base_link"/>

  <!-- R2D2 sensor plate — example offset; measure for the real mount -->
  <link name="sensor_plate"/>
  <joint name="base_to_plate" type="fixed">
    <parent link="base_link"/>
    <child  link="sensor_plate"/>
    <origin xyz="0.00 0.0 0.60" rpy="0 0 0"/>    <!-- MEASURE R2D2 base_link → plate center -->
  </joint>

  <xacro:slam_sensors_rigid_group/>

  <!-- D435 front on R2D2 — example mount on the dome front; MEASURE -->
  <link name="d435_front_link"/>
  <joint name="base_to_d435_front" type="fixed">
    <parent link="base_link"/>
    <child  link="d435_front_link"/>
    <origin xyz="0.15 0.0 0.70" rpy="0 0 0"/>
  </joint>

  <!-- D435i rear on R2D2 — yaw 180° -->
  <link name="d435_rear_link"/>
  <joint name="base_to_d435_rear" type="fixed">
    <parent link="base_link"/>
    <child  link="d435_rear_link"/>
    <origin xyz="-0.15 0.0 0.70" rpy="3.14159 0 0"/>
  </joint>

</robot>
```

**`urdf/roboscout.urdf.xacro`** and **`urdf/mecanum.urdf.xacro`** follow the same pattern — declare `base_link`, mount the sensor plate at the measured chassis offset, include `sensors_common`, and add per-platform `d435_front_link` / `d435_rear_link` joints. The mecanum URDF additionally declares four wheel frames (`fl_wheel`, `fr_wheel`, `rl_wheel`, `rr_wheel`) off `base_link` for the robot driver's odometry math, but those are orthogonal to the SLAM stack.

**URDF gotchas (platform-agnostic):**

- **`base_link` must match what the robot driver publishes** (Go2 SDK on Go2; `robot_state_publisher` on R2D2/Roboscout/mecanum). Verify with `ros2 run tf2_tools view_frames`. If the Go2 SDK publishes `base` instead of `base_link`, either rename the URDF root or add a `static_transform_publisher` bridge.
- **Mid-360 Z-axis points up** out of the top of the sensor. Mounted upside-down → add `rpy="3.14159 0 0"` to `plate_to_livox` in `sensors_common.urdf.xacro` (which affects all platforms — verify the rig's orientation first).
- **Measure offsets carefully.** Sub-cm / sub-degree accuracy between IMU and LiDAR is required. The `livox_to_imu` translation in `sensors_common` is what you calibrate — it's measured once for the sensor plate itself, not per platform.
- **D435 child frames auto-publish** (`d435_front_camera_depth_frame`, `d435_front_camera_color_optical_frame`, etc.) because of the namespace. Only TF the parent `d435_front_link` / `d435_rear_link`.

**`launch/perception.launch.py`** — platform-aware:

```python
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = Path(get_package_share_directory('slam_bringup'))
    rviz_cfg  = bringup_share / 'rviz' / 'perception.rviz'

    platform_arg = DeclareLaunchArgument(
        'platform',
        default_value='go2',
        description='Robot platform: go2 | r2d2 | roboscout | mecanum',
    )

    urdf_path = PathJoinSubstitution([
        str(bringup_share), 'urdf',
        [LaunchConfiguration('platform'), '.urdf.xacro'],
    ])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_path])}],
    )

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_share / 'launch' / 'sensors.launch.py')
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', str(rviz_cfg)],
        output='screen',
    )

    return LaunchDescription([platform_arg, rsp, sensors, rviz])
```

**Tasks:**

- [ ] Create `urdf/sensors_common.urdf.xacro` with the macro above. Measure `livox_frame → imu_link` on the sensor plate with calipers and substitute the measured values (do not ship placeholders).
- [ ] Create `urdf/go2.urdf.xacro`. Measure Go2's `base_link → sensor_plate`, `base_link → d435_front_link`, `base_link → d435_rear_link` and substitute.
- [ ] Create `urdf/r2d2.urdf.xacro` — when the R2D2 mount is fabricated, measure and fill in.
- [ ] Create `urdf/roboscout.urdf.xacro` — when the Roboscout mount is fitted, measure and fill in.
- [ ] Create `urdf/mecanum.urdf.xacro` — when the mecanum UGV mount is fitted, measure and fill in. Add wheel frames if the UGV driver expects them.
- [ ] Update `package.xml` to include `<build_depend>xacro</build_depend>` if not already present
- [ ] Create `rviz/perception.rviz` with PointCloud2 + Image + TF displays:
  - [ ] Set PointCloud2 Decay Time = 1.0s for Mid-360 (non-repetitive needs integration)
  - [ ] Style: Flat Squares, Size 0.02m
- [ ] Create `launch/perception.launch.py` with the platform arg
- [ ] Rebuild + source
- [ ] Launch for the current platform: `ros2 launch slam_bringup perception.launch.py platform:=go2`
- [ ] **Verify:** rviz opens with no red-text errors
- [ ] **Verify:** `/livox/lidar` displays in `livox_frame`, fills out nicely over 1s
- [ ] **Verify:** `/d435_front/camera/color/image_raw` displays as Image
- [ ] **Verify:** TF tree shows `base_link → sensor_plate → livox_frame → imu_link`, plus `base_link → {d435_front_link, d435_rear_link}`
- [ ] **Verify:** `ros2 run tf2_tools view_frames` generates a PDF with fully connected tree, no missing-TF warnings
- [ ] Repeat the verify block for each platform as its URDF gets measured and filled in

### 6.8 — Platform SDK / `base_link` integration check

Per platform. Confirms the robot driver's TF root matches the URDF's `base_link`.

**Go2:**

- [ ] Bring up the Unitree Go2 driver (separate process outside `slam_bringup`)
- [ ] In another terminal: `ros2 launch slam_bringup perception.launch.py platform:=go2`
- [ ] **Verify:** `ros2 run tf2_tools view_frames` shows Go2's base frame connects to this URDF's root
- [ ] If Go2 publishes a different root (e.g., `base` instead of `base_link`), either rename the URDF root or add a `static_transform_publisher` bridge

**R2D2 / Roboscout / Mecanum UGV:**

- [ ] No external SDK is expected to publish `base_link` — `robot_state_publisher` from this URDF publishes it
- [ ] Launch: `ros2 launch slam_bringup perception.launch.py platform:=<name>`
- [ ] **Verify:** `ros2 run tf2_tools view_frames` shows a fully connected tree with `base_link` as the root
- [ ] If the platform has its own base driver (e.g., mecanum odometry node publishing `odom → base_link`), verify the transform chain is consistent; chain `odom → base_link → sensor_plate → ...`

### 6.9 — install.sh smoke test

- [ ] On a clean build dir: `rm -rf build/ install/ log/`
- [ ] Re-run `./install.sh`
- [ ] **Verify:** reaches `colcon build` and completes with no manual intervention

### 6.10 — Dual camera (add rear D435i)

- [ ] Launch dual: `ros2 launch slam_bringup sensors.launch.py enable_rear:=true`
- [ ] **Verify:** Both `/d435_front/*/image_raw` and `/d435_rear/*/image_raw` at ~30 Hz with no dropped frames (`ros2 topic hz`)
- [ ] **Verify:** `lsusb -t` shows each D435 on its own USB 3 bus (not sharing a controller)
- [ ] **Verify:** `jtop` with dual cams → CPU ≤55% total, no USB saturation
- [ ] If dropped frames: move one camera to the other USB 3.2 Type-A port, or drop profile to `640x480x15`

### Phase 1 Exit Criteria

- [ ] `ros2 launch slam_bringup perception.launch.py platform:=<platform>` cold-boots to a working rviz view for the current platform
- [ ] All topic Hz rates nominal
- [ ] TF tree fully connected: either robot SDK `base_link` → `slam_bringup` URDF (Go2) or `slam_bringup` URDF rooted at `base_link` (R2D2 / Roboscout / mecanum) — no disconnected frames
- [ ] jtop baseline documented (typical CPU %, RAM MB)
- [ ] `install.sh` runs clean from scratch
- [ ] At least one platform URDF complete with measured offsets; additional platforms can be filled in later

## 7. Phase 2 — SLAM Layer

**Goal:** FAST-LIO2 + RTABMap producing `/Odometry`, `/map`, `/octomap_full`, persistent database. Loop closure fires on physical loops.

### 7.1 — SLAM dependencies

`install.sh` already apt-installs `ros-humble-rtabmap-ros`, `ros-humble-navigation2`, `ros-humble-nav2-bringup`, `ros-humble-cyclonedds`, `ros-humble-rmw-cyclonedds-cpp` and git-clones `FAST_LIO_ROS2` (Ericsii fork) with submodules. If skipped:

- [ ] `apt install ros-humble-rtabmap-ros ros-humble-navigation2 ros-humble-nav2-bringup`
- [ ] `git clone --recursive https://github.com/Ericsii/FAST_LIO_ROS2.git ~/slam_ws/src/FAST_LIO_ROS2`
- [ ] Rebuild: `cd ~/slam_ws && colcon build`
- [ ] **Verify:** `ros2 pkg list | grep fast_lio` exists
- [ ] **Verify:** `ros2 pkg list | grep rtabmap` shows `rtabmap_launch`, `rtabmap_ros`, etc.

### 7.2 — FAST-LIO2 config + launch

FAST-LIO2 uses the **Mid-360 built-in ICM40609** (pre-calibrated extrinsic `x=11.0mm, y=23.29mm, z=-44.12mm`), not WitMotion. `scan_bodyframe_pub_en: true` is required so RTABMap gets `/cloud_registered_body`.

**`config/fast_lio_mid360.yaml`:**

```yaml
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/livox/imu"
    time_sync_en: false
    time_offset_lidar_to_imu: 0.0

preprocess:
    lidar_type: 1                 # 1 = Livox
    scan_line: 4
    blind: 0.2                    # see "blind vs rig height" note below

mapping:
    acc_cov: 0.1
    gyr_cov: 0.1
    b_acc_cov: 0.0001
    b_gyr_cov: 0.0001
    fov_degree: 360
    det_range:  100.0
    extrinsic_est_en: false       # set true once to auto-calibrate, then lock to false
    extrinsic_T: [ -0.011, -0.02329, 0.04412 ]   # Livox pre-calibrated IMU→LiDAR (m)
    extrinsic_R: [ 1, 0, 0,
                   0, 1, 0,
                   0, 0, 1 ]
    filter_size_surf: 0.3         # overrides FAST-LIO's 0.5m default
    filter_size_map:  0.3         # overrides FAST-LIO's 0.5m default

publish:
    path_en:  true
    scan_publish_en:  true
    dense_publish_en: true
    scan_bodyframe_pub_en: true   # REQUIRED — RTABMap subscribes to /cloud_registered_body
```

**`launch/fast_lio.launch.py`:**

```python
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = Path(get_package_share_directory('slam_bringup'))
    fast_lio_share = Path(get_package_share_directory('fast_lio'))

    default_config = bringup_share / 'config' / 'fast_lio_mid360.yaml'

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=str(default_config),
    )

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(fast_lio_share / 'launch' / 'mapping.launch.py')
        ),
        launch_arguments={'config_file': LaunchConfiguration('config_file')}.items(),
    )

    return LaunchDescription([config_arg, fast_lio_launch])
```

**As-built note:** the snippet above shows `IncludeLaunchDescription(... launch_arguments={'config_file': full_path}.items())`. That is **wrong** — the upstream `fast_lio/launch/mapping.launch.py` takes **two** args, `config_path` (a directory) and `config_file` (a filename), and joins them internally via `PathJoinSubstitution`. Pass them separately. The wrapper also defaults upstream's `rviz` arg to `false` (we view via Foxglove; rviz2 on the Jetson eats CPU during stress tests).

Our `config/fast_lio_mid360.yaml` uses the `/**:`-wildcard yaml root so parameters are delivered regardless of the `fastlio_mapping` node's actual resolved name. Float types are explicit (`360.0` not `360`) because the upstream parameter declarations are strictly typed.

**Deviations from the Ericsii/FAST_LIO_ROS2 sample config** (validated 2026-04-20/21 on gizmo — do not revert without understanding the failure mode each fixes):

- `blind: 0.2` (sample: `0.5`). Mid-360 is mounted ~0.3 m above the floor on our portable rig. With `blind: 0.5`, every ground-plane return is discarded as "too close"; FAST-LIO then has nothing constraining Z translation, the stationary accel bias (~0.02 g horizontal) integrates unchecked, and the pose dives through the floor until ICP loses correspondences — `No Effective Points!` then SIGSEGV. `0.2` keeps ground returns while still rejecting self-hits from the 2020/2040 frame (tightest frame member is >20 cm from the LiDAR head). Raise this only if the Mid-360 is mounted >0.6 m up (top of a Go2 or a tall cart).
- `filter_size_surf: 0.3` and `filter_size_map: 0.3` (FAST-LIO defaults: `0.5` both). Mid-360's non-repetitive scan pattern delivers a sparse per-sweep cloud. At `0.5 m` voxel size, downsampling leaves too few surf points for ICP to find correspondences in indoor/cluttered spaces — same "No Effective Points!" failure mode. `0.3 m` preserves enough points per scan while still rate-limiting map growth.
- IMU unit note for future debugging: the Livox driver publishes accel in **g-units** (comment in `livox_ros_driver2/src/comm/lidar_imu_data_queue.h`), not m/s². FAST-LIO auto-adapts via `acc_avr * G_m_s2 / mean_acc.norm()` in `IMU_Processing.hpp:260`, so this is **not** a bug. Do not "fix" the driver to multiply accel by 9.81 — doing so will double-apply the scale.

**Tasks:**

- [x] Create `config/fast_lio_mid360.yaml`
- [x] Create `launch/fast_lio.launch.py` (fixed to pass `config_path` + `config_file` separately)
- [x] Create `start_fast_lio.sh` + `kill_fast_lio.sh` (script preflights for Mid-360 CustomMsg publisher before launching FAST-LIO)
- [x] Rebuild + source
- [x] Terminal A: `./start_sensors.sh lidar_xfer_format:=1 enable_d435:=false enable_witmotion:=false`
- [x] Terminal B: `./start_fast_lio.sh`
- [x] **Verify:** `ros2 topic hz /Odometry` → ~10 Hz (measured 10.004 Hz)
- [x] **Verify:** `/cloud_registered_body` publishing (subscriber-side 7.6 Hz measured under competing desktop-session CPU load; publisher is at-rate)
- [x] **Verify:** `ros2 run tf2_ros tf2_echo camera_init body` returns a live transform with translation + rotation matching the rig's pose
- [x] **Verify:** live pose sample — `x=-2.03 y=2.76 z=-2.05` in `camera_init`, covariance ~1e-5 m², orientation near-identity with small roll — consistent with a rig carried ~4m from origin
- [x] **Verify (2026-04-21, living room floor):** 5-minute stationary hold after 30 s cold-start settle — pose stable to within centimeters. Empty-room bench-table test diverged; furnished-space test converges. See TEST_PLAN.md *Phase 2 — hard-won constraints* for environment/mount requirements.

### 7.3 — base_link ↔ body TF bridge (per-platform)

FAST-LIO2 hardcodes its output body frame as `body`. The URDF uses `base_link` as root. Bridge them with a static TF at the sensor plate's position (reuse the `base_link → sensor_plate` measurement from each platform's URDF in §6.7).

The bridge offset **is per-platform** — the sensor plate sits in a different place on Go2 vs R2D2 vs Roboscout vs mecanum UGV. `slam.launch.py` looks up the per-platform offsets from an inline dict keyed by the `platform` arg (see §7.5).

**Tasks:**

- [ ] Using each platform's `base_link → sensor_plate` measurement from §6.7, fill in the per-platform bridge offsets in `slam.launch.py`'s `PLATFORM_BRIDGES` dict
- [ ] **Verify (per platform):** With FAST-LIO2 + bridge running, `ros2 run tf2_tools view_frames` shows one connected tree: `map → camera_init → body → livox_frame → imu_link`

### 7.4 — RTABMap launch

Livox point clouds are **unorganized** — `Grid/NormalsSegmentation: false` is non-negotiable (the default `true` silently produces all-unknown occupancy grids). `visual_odometry:=false` — FAST-LIO2 provides odom; enabling RTABMap's visual odom duplicates work and eats CPU.

**`launch/rtabmap.launch.py`:**

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rtabmap_share = get_package_share_directory('rtabmap_launch')

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{rtabmap_share}/launch/rtabmap.launch.py'
        ),
        launch_arguments={
            'frame_id':                  'body',
            'odom_frame_id':             'camera_init',
            'map_frame_id':              'map',
            'subscribe_scan_cloud':      'true',
            'subscribe_depth':           'true',
            'subscribe_rgb':             'true',
            'visual_odometry':           'false',
            'approx_sync':               'true',
            'approx_sync_max_interval':  '0.05',
            'scan_cloud_topic':          '/cloud_registered_body',
            'rgb_topic':                 '/d435_front/camera/color/image_raw',
            'depth_topic':               '/d435_front/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic':         '/d435_front/camera/color/camera_info',
            'odom_topic':                '/Odometry',
            'qos':                       '1',
            'rtabmap_args':
                '--Reg/Strategy 1 --Icp/PointToPlane true --Icp/VoxelSize 0.1 '
                '--Icp/MaxCorrespondenceDistance 1.0 '
                '--Grid/Sensor 0 --Grid/3D true --Grid/RayTracing true '
                '--Grid/NormalsSegmentation false --Grid/CellSize 0.2 '
                '--Grid/RangeMax 30.0 --Grid/GroundIsObstacle true '
                '--Vis/MinInliers 15 --RGBD/OptimizeMaxError 3.0',
        }.items(),
    )

    return LaunchDescription([rtabmap])
```

**Tasks:**

- [ ] Create `launch/rtabmap.launch.py` exactly as above
- [ ] Rebuild + source
- [ ] Launch piecewise for debug (individual launches, not `sensors.launch.py`, so `xfer_format` can be set on mid360):
  - [ ] Terminal A: `ros2 launch slam_bringup mid360.launch.py xfer_format:=1`
  - [ ] Terminal B: `ros2 launch slam_bringup d435.launch.py slam_mode:=true`
  - [ ] Terminal C: `ros2 launch slam_bringup witmotion.launch.py`
  - [ ] Terminal D: `ros2 launch slam_bringup fast_lio.launch.py`
  - [ ] Terminal E: static TF bridge (from §7.3): `ros2 run tf2_ros static_transform_publisher 0.10 0.0 0.20 0 0 0 base_link body` (substitute measured values)
  - [ ] Terminal F: `ros2 launch slam_bringup rtabmap.launch.py`
- [ ] **Verify:** `ros2 topic hz /map` publishes (takes ~5-10s of motion before first map)
- [ ] Walk/push the robot through a small area; watch `/map` grow in rviz

### 7.5 — slam.launch.py top-level (platform-aware)

Combines all SLAM pieces: platform URDF + robot_state_publisher, mid360 in CustomMsg mode, d435 in SLAM mode, WitMotion (backup IMU), per-platform `base_link → body` TF bridge, FAST-LIO2, RTABMap. `enable_rear` passthrough for dual-camera testing. `platform:=<name>` selects which URDF and which bridge offsets to use.

**`launch/slam.launch.py`:**

```python
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# Per-platform `base_link → body` bridge (x, y, z, roll, pitch, yaw in m / rad)
# Measured from the robot's base_link to the sensor plate. Fill in after §6.7.
PLATFORM_BRIDGES = {
    'go2':       ['0.10', '0.0', '0.20', '0', '0', '0'],
    'r2d2':      ['0.00', '0.0', '0.60', '0', '0', '0'],
    'roboscout': ['0.00', '0.0', '0.30', '0', '0', '0'],
    'mecanum':   ['0.00', '0.0', '0.25', '0', '0', '0'],
}


def _tf_bridge(context):
    platform = context.launch_configurations['platform']
    if platform not in PLATFORM_BRIDGES:
        raise RuntimeError(
            f"Unknown platform '{platform}'. "
            f"Supported: {sorted(PLATFORM_BRIDGES)}"
        )
    offsets = PLATFORM_BRIDGES[platform]
    return [Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[*offsets, 'base_link', 'body'],
    )]


def generate_launch_description():
    bringup_share = Path(get_package_share_directory('slam_bringup'))
    launch_dir = bringup_share / 'launch'

    platform_arg = DeclareLaunchArgument(
        'platform',
        default_value='go2',
        description='Robot platform: go2 | r2d2 | roboscout | mecanum',
    )
    enable_front_arg = DeclareLaunchArgument('enable_front', default_value='true')
    enable_rear_arg  = DeclareLaunchArgument('enable_rear',  default_value='false')

    # Platform URDF → robot_state_publisher
    urdf_path = PathJoinSubstitution([
        str(bringup_share), 'urdf',
        [LaunchConfiguration('platform'), '.urdf.xacro'],
    ])
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_path])}],
    )

    # Mid-360 in CustomMsg mode for FAST-LIO2
    mid360 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'mid360.launch.py')),
        launch_arguments={'xfer_format': '1'}.items(),
    )

    # D435s in SLAM mode — inherits enable_front / enable_rear from the caller
    d435 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'd435.launch.py')),
        launch_arguments={
            'slam_mode':    'true',
            'enable_front': LaunchConfiguration('enable_front'),
            'enable_rear':  LaunchConfiguration('enable_rear'),
        }.items(),
    )

    # WitMotion runs as backup IMU; FAST-LIO2 uses the Mid-360 built-in
    witmotion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'witmotion.launch.py'))
    )

    # base_link → body bridge — picks offsets based on `platform` at launch time
    tf_bridge = OpaqueFunction(function=_tf_bridge)

    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'fast_lio.launch.py'))
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'rtabmap.launch.py'))
    )

    return LaunchDescription([
        platform_arg, enable_front_arg, enable_rear_arg,
        rsp, mid360, d435, witmotion, tf_bridge, fast_lio, rtabmap,
    ])
```

**Tasks:**

- [ ] Create `launch/slam.launch.py` exactly as above
- [ ] Fill in each platform's measured offsets in `PLATFORM_BRIDGES` as that platform's URDF (§6.7) is completed
- [ ] Rebuild + source
- [ ] Cold-boot test on the current platform: `ros2 launch slam_bringup slam.launch.py platform:=go2`
- [ ] **Verify:** All topics from §7.2 + §7.4 come up in a single command
- [ ] **Verify:** `~/.ros/rtabmap.db` is being written (file grows over time)
- [ ] Repeat the cold-boot test for each additional platform (`platform:=r2d2`, etc.) as its URDF + bridge offsets land

### 7.6 — Loop closure validation

- [ ] Plan a short indoor loop — start at a visually distinctive spot, walk around, return so D435 RGB view overlaps the starting frame
- [ ] Start SLAM: `ros2 launch slam_bringup slam.launch.py`
- [ ] Drive/walk the loop slowly
- [ ] Watch RTABMap console output for "Loop closure detected" messages
- [ ] **Verify:** Map appears visually consistent after closure — no duplicate ghost at start position
- [ ] **Verify:** `rtabmap-databaseViewer ~/.ros/rtabmap.db` shows loop closure edges in the graph

### 7.7 — Performance check

Orin Nano Super (8 GB) baseline expectations:

| Component | CPU | RAM |
|-----------|-----|-----|
| livox_ros_driver2 | ~5% | ~50 MB |
| FAST-LIO2 | 15-25% | 200-400 MB (ikd-Tree grows with map) |
| realsense2_camera (both) | ~15% | ~300 MB |
| RTABMap | 20-40% | 500 MB – 2 GB (grows with map + loop closures) |
| **Total** | **~55-85%** | **~1-3 GB** |

- [ ] With `slam.launch.py` running 5+ min, take `jtop` snapshot:
  - [ ] CPU total ≤ 85%
  - [ ] RAM ≤ 3 GB
  - [ ] GPU ≤ 50% (RTABMap is CPU-bound)
  - [ ] Temp ≤ 70°C
- [ ] If CPU >85%: drop D435 to `640x480x15` via launch arg
- [ ] If RAM >3 GB: ensure `database_path` is on NVMe; reduce `Grid/CellSize` granularity
- [ ] Document baseline numbers at Phase 2 exit

### Phase 2 Exit Criteria

- [ ] `slam.launch.py` cold-boots the entire stack in one command
- [ ] `/Odometry` + `/cloud_registered_body` + `/map` all publishing at nominal Hz
- [ ] Loop closure fires on a real indoor loop; map remains consistent
- [ ] RTABMap DB persists to disk; ready for Phase 2.5 replay
- [ ] CPU ≤85% on Orin Nano Super

## 8. Phase 2.5 — Recording & Playback

**Goal:** Record raw sensor bags for offline SLAM tuning; replay through FAST-LIO2 + RTABMap deterministically; reload saved maps in localization-only mode; inspect bags natively in Foxglove Studio.

### 8.1 — Bag format + record script

MCAP is Foxglove-native, zstd-compressible, and faster than the default SQLite3. Bags go to `~/bags/` on the **NVMe SSD** (not SD card — bag writes are heavy).

**`scripts/record_sensors.sh`:**

```bash
#!/usr/bin/env bash
mkdir -p ~/bags
ros2 bag record \
  -s mcap --compression-mode file --compression-format zstd \
  -o ~/bags/go2_$(date +%Y%m%d_%H%M%S) \
  /livox/lidar /livox/imu \
  /d435_front/camera/color/image_raw \
  /d435_front/camera/aligned_depth_to_color/image_raw \
  /d435_front/camera/color/camera_info \
  /imu/data \
  /tf /tf_static
```

**Tasks:**

- [ ] Create `scripts/record_sensors.sh` with the content above
- [ ] `chmod +x scripts/record_sensors.sh`
- [ ] Record with `xfer_format:=0` (PointCloud2) for the Mid-360 — CustomMsg is harder to replay into non-FAST-LIO2 consumers

### 8.2 — Record a baseline run

- [ ] Terminal A: `ros2 launch slam_bringup sensors.launch.py` (raw PointCloud2 mode)
- [ ] Terminal B: `./scripts/record_sensors.sh`
- [ ] Walk the sensor plate (or drive the robot) through an interesting indoor path for 5 min
- [ ] Ctrl-C to stop
- [ ] **Verify:** `ros2 bag info ~/bags/go2_YYYYMMDD_HHMMSS` shows expected topics with non-zero message counts
- [ ] **Verify:** Bag size is 1-3 GB (reasonable for 5 min)

### 8.3 — Playback: raw viz only

- [ ] Terminal A: `ros2 bag play ~/bags/go2_YYYYMMDD_HHMMSS --clock`
- [ ] Terminal B: `rviz2 --ros-args -p use_sim_time:=true` with saved perception layout
- [ ] **Verify:** rviz shows recorded point clouds + images at real-time pace

### 8.4 — Playback: through FAST-LIO2

- [ ] Terminal A: `ros2 bag play <bag> --clock`
- [ ] Terminal B: `ros2 launch slam_bringup fast_lio.launch.py use_sim_time:=true`
- [ ] **Verify:** `/Odometry` publishes from bag data
- [ ] Replay twice — compare `/Odometry.pose.pose` at the same bag timestamp; should be deterministic (identical)

### 8.5 — Playback: full SLAM reconstruction

- [ ] Terminal A: bag play `--clock`
- [ ] Terminal B: fast_lio launch `use_sim_time:=true`
- [ ] Terminal C: static TF bridge (as §7.3)
- [ ] Terminal D: rtabmap launch `use_sim_time:=true`
- [ ] **Verify:** `/map` is rebuilt from the bag
- [ ] Useful for param tuning: change `Grid/CellSize` or `Icp/VoxelSize`, replay, compare results

### 8.6 — RTABMap localization-only mode

- [ ] After a good mapping session, copy DB: `mkdir -p ~/maps && cp ~/.ros/rtabmap.db ~/maps/indoor_01.db`
- [ ] Either add a `localize:=true` launch arg to `rtabmap.launch.py`, or create `launch/rtabmap_localize.launch.py` that sets:
  - [ ] `Mem/IncrementalMemory:=false`
  - [ ] `Mem/InitWMWithAllNodes:=true`
  - [ ] `database_path:=~/maps/indoor_01.db`
- [ ] Launch `slam.launch.py` with the localization-mode rtabmap
- [ ] **Verify:** robot localizes in saved map without adding new graph nodes
- [ ] **Verify:** DB file size stays constant — no growth

### 8.7 — Foxglove Studio integration

- [ ] Install Foxglove Studio on dev laptop (Mac): https://foxglove.dev/download
- [ ] Open MCAP bag: File → Open Local File → `go2_YYYYMMDD_HHMMSS.mcap`
- [ ] Build saved layout:
  - [ ] PointCloud panel on `/livox/lidar`
  - [ ] Image panel on `/d435_front/camera/color/image_raw`
  - [ ] TF panel
- [ ] Export layout JSON → commit to `foxglove/default_layout.json`

### Phase 2.5 Exit Criteria

- [ ] `scripts/record_sensors.sh` produces a replayable MCAP bag
- [ ] Bag playback through fast_lio + rtabmap reconstructs `/map` deterministically
- [ ] Saved RTABMap DB reloads in localization-only mode; robot localizes without graph growth
- [ ] Foxglove layout committed and opens the bag natively

## 9. Gotchas — Quick Reference

### Livox
- Vendor launch files hardcode `user_config_path` — use direct `Node()`, NOT `IncludeLaunchDescription`
- No official apt package for `livox_ros_driver2` — build from source; `install.sh` handles this
- `xfer_format: 0` = `sensor_msgs/PointCloud2` (RViz displays)
- `xfer_format: 1` = `livox_ros_driver2/msg/CustomMsg` (FAST-LIO2 requires)
- Livox Viewer 2 is x86-only — won't run on Jetson; use Mac/PC for sensor config
- RViz2 decay time for Mid-360: **1.0 s stationary** (non-repetitive needs ~5× VLP-16's decay)
- **`mid360.json` `host_net_info.*_ip` must be the Jetson's real IP (`192.168.1.100`), NOT `255.255.255.255`** — the driver `bind()`s sockets to these addresses; broadcast is not bindable and the driver silently never opens its host-side listeners. Symptom: init log prints through "successfully enable Livox Lidar imu" but `/livox/lidar` / `/livox/imu` never publish and `ss -uln | grep 5630` shows no listener. (Broadcast destination on the *lidar's* own config is fine — see §6.3.)

### RealSense
- Two cameras = two namespaces (`d435_front`, `d435_rear`) to keep topics separate
- Disable gyro/accel on both — WitMotion is the authoritative IMU; D435i's BMI055 would create a second stream
- `depth_module.profile: 848x480x30` is the default; bump to `1280x720x30` if you need range + have USB bandwidth

### WitMotion
- Pin the serial port via `/dev/serial/by-id/` — `/dev/ttyUSB0` reassigns on reboot
- WT901B default baud is 9600; can reconfigure to 115200 for higher `frequency`
- Remap `/imu` → `/imu/data` so downstream nodes find the standard topic

### URDF
- `imu_link` is child of `livox_frame`, NOT `base_link` — encodes the rigid sensor-plate coupling
- `base_link` must match what the robot driver publishes (Go2 SDK on Go2; `robot_state_publisher` from the URDF on R2D2 / Roboscout / mecanum). Verify with `ros2 run tf2_tools view_frames`.

### Network
- Two interfaces: `eth0` (Go2 control, `192.168.123.0/24`) and `eth1` (LiDAR, `192.168.1.0/24`)
- Mid-360's *on-device* outbound destination is `255.255.255.255` broadcast (set once in Livox Viewer 2) — keeps the lidar's config portable across networks. This does NOT mean the ROS driver's `host_net_info.*_ip` should be broadcast — those must be the Jetson's real IP; see Livox gotcha above.
- The Jetson's `eth1` IP (e.g., `192.168.1.100`) can be anything on `192.168.1.0/24` **except** `.0`, `.201`, `.202`, `.203`, `.255`

### SLAM (FAST-LIO2 + RTABMap)
- **`xfer_format: 1`** (CustomMsg) required on Mid-360 launch for FAST-LIO2 — `slam.launch.py` sets this automatically
- **`scan_bodyframe_pub_en: true`** in `fast_lio_mid360.yaml` — RTABMap subscribes to `/cloud_registered_body`; without this RTABMap starves
- **`Grid/NormalsSegmentation: false`** — critical for Livox's unorganized clouds; default `true` silently produces all-unknown occupancy grids
- **`align_depth.enable: true`** on D435 — RTABMap needs depth aligned to RGB; `slam_mode:=true` handles this
- **Disable D435's own pointcloud** in SLAM mode — RTABMap generates its own from depth; `slam_mode:=true` handles this
- **CycloneDDS required** — `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`; FastDDS drops Mid-360 point cloud frames
- **FAST-LIO2 uses Mid-360 built-in IMU**, not WitMotion — requires rigid LiDAR coupling the Livox ICM40609 already has
- **`base_link` ↔ `body` bridge** needed — FAST-LIO2 hardcodes `body`; `slam.launch.py` ships a static TF bridge
- **Save RTABMap DB to NVMe SSD**, not SD card — constant writes will wear the SD card and throttle
- **`visual_odometry:=false`** on RTABMap — FAST-LIO2 provides odom; enabling RTABMap's visual odom duplicates work

## 9.5 Phase 3 — Navigation Layer (landed 2026-05-02)

Nav2 was added on top of the SLAM stack mid-session and is fully documented in **README.md → Navigation (Nav2)**. Summary of what was built:

- `launch/nav2.launch.py` — `pointcloud_to_laserscan` (Mid-360 body cloud → `/scan` slice at the rover-collision band, default `[0.10, 0.45]` m above `base_link`), identity `camera_init → odom` static TF, and `nav2_bringup/navigation_launch.py` (controller / planner / smoother / behavior / BT / waypoint / velocity-smoother / lifecycle_manager). **No** `map_server` (RTABMap publishes `/map`) and **no** `amcl` (RTABMap publishes the `map → camera_init` correction).
- `config/nav2_params.yaml` — circular `robot_radius: 0.25`, `inflation_radius: 0.30`, DWB local planner, NavFn global planner. Global costmap uses **only** `static_layer + inflation_layer` (no `obstacle_layer` — see costmap-layout note in README); local costmap uses `obstacle_layer + inflation_layer` for transient reactivity.
- `start_nav.sh` / `kill_nav.sh` — wraps `slam.launch.py nav2:=true localization:=true delete_db_on_start:=false`. Refuses to launch if `~/.ros/rtabmap.db` doesn't exist.
- New launch args plumbed through `slam.launch.py`: `nav2`, `nav2_params_file`, `nav2_autostart`, `force_3dof`, `scan_min_height`, `scan_max_height`.
- `force_3dof:=true` clamps z/roll/pitch on wheeled rovers — required for indoor wheeled platforms where FAST-LIO drifts in altitude/tilt without LiDAR ceiling constraints. Default false to preserve 6 DoF for the bench fixture and handheld testing.
- `RGBD/OptimizeMaxError` bumped 3.0 → 5.0 in `launch/rtabmap.launch.py` to keep legit large loop-closure corrections from being rejected against drift-laden saved maps. Re-mapping with `force_3dof:=true` is the long-term fix.
- `kill_helpers.sh` introduced — every `kill_*.sh` (except `kill_d435.sh`) now uses `nuke_processes` which escalates SIGINT → SIGKILL → `sudo` SIGKILL with `pgrep` verification at each stage. Surviving orphans (the cause of two earlier "lidar loop back" debugging sessions) now block the next launch via exit 1 instead of silently coexisting.

**Tasks remaining for Phase 3 to be platform-complete:**
- [ ] Per-platform `cmd_vel → drive base` bridge (Roboscout, Go2, mecanum). Without this, Nav2 plans but the rover doesn't move.
- [ ] Per-platform `nav2_<platform>_params.yaml` — tuned `robot_radius`, footprint, vel limits.
- [ ] Per-platform `scan_max_height` defaults — `0.45` works for small rovers, ~`0.8` for Go2 standing, `~1.2` for the bench fixture.

## 10. Deferred (not in this plan)

Each stretch is an independent milestone — tackled when motivated, in any order, after Phase 2.5 is stable. Each should get its own plan document when started.

- **3.1 D435 → D455 (outdoor camera upgrade)** — Parameterize done in §6.4; physical swap + outdoor loop-closure validation
- **3.2 Mid-360 → VLP-16 (outdoor LiDAR)** — No built-in IMU, WitMotion becomes authoritative; `Grid/NormalsSegmentation: true` (VLP-16 is organized)
- **3.3 Mid-360 → Unitree L1** — Drop-in; L1 has built-in 6-axis IMU, 360° × 90° FOV
- **3.4 Mid-360 → Pandar40P (long range)** — Heavy (1.46 kg); WitMotion required; 200m range benefit outdoors
- **3.5 Dual Mid-360 multi-LiDAR SLAM** — Requires 3rd Mid-360; PTPv2 time sync; merged config
- **3.6 OAK-D Pro parallel AI** — Object detection alongside SLAM, NOT routed into RTABMap; runs on its own `/oak_front` namespace

## 11. References

### Source notes (Obsidian vault)
- **Source for this plan:** `rico/Robotics/Nvidia Jetson/Jeston Orin Nano Super/Jetson Orin Nano Go2 ROS2 Sensor Bringup.md`
- `rico/Robotics/Nvidia Jetson/Jeston Orin Nano Super/Jetson Orin Nano Go2 Control Panel.md` — FastAPI + systemd + htmx control panel for starting/stopping launches, monitoring
- `rico/Robotics/.../FAST-LIO2 RTABMap ROS2 - LiDAR Visual SLAM Pipeline on Jetson.md` — full SLAM pipeline background, tuning rationale, alternatives (FAST-LIVO2, LVI-SAM)
- `rico/Robotics/Robots/Robot Dog/Unitree Go2 Pro/Hardware/Unitree Go2 VLP-16 Lidar, Depth Camera and IMU Mounting.md` — sensor plate design, vibration isolation, weight budget, USB/networking hardware
- `rico/Robotics/Sensors/Livox Mid-360 - LiDAR Sensor.md` — IP history, broadcast setup rationale, `xfer_format` notes
- `rico/Robotics/Sensors/Velodyne VLP-16 (Puck).md`
- `rico/Robotics/Sensors/Hesai Pandar40P.md`
- `rico/Robotics/Sensors/Unitree Lidar L1 RM.md`
- `rico/Robotics/Sensors/LiDAR Universal Interface — VLP-16 & Pandar40 Pinout.md`
- `rico/Robotics/Sensors/Depth Camera Comparison for Unitree Go2.md`
- `rico/Robotics/Sensors/Stereo Labs Zed Camera.md`
- `rico/AI/YouTube/Multi Livox FAST-LIO2 - YouTube - MID360 MID70 Multi-LiDAR SLAM.md`
- `Wiki/Robotics/LiDAR.md`

### External
- FAST-LIO2 (ROS2 port): https://github.com/Ericsii/FAST_LIO_ROS2
- Livox-SDK2: https://github.com/Livox-SDK/Livox-SDK2
- livox_ros_driver2: https://github.com/Livox-SDK/livox_ros_driver2
- witmotion_IMU_ros: https://github.com/ElettraSciComp/witmotion_IMU_ros
- unilidar_sdk2 (for stretch 3.3): https://github.com/unitreerobotics/unilidar_sdk2
- Foxglove Studio: https://foxglove.dev/download
