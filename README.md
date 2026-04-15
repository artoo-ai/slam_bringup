# slam_bringup

Portable sensor-rig SLAM stack for ROS2 Humble. One NVIDIA Jetson Orin Nano Super + one sensor plate moves between multiple mobile robots; the package selects per-platform URDF + TF bridge offsets at launch time.

## Sensors (shared across all platforms)

- **Livox Mid-360** — 360° × 59° non-repetitive LiDAR with built-in ICM40609 IMU (primary for FAST-LIO2)
- **Intel RealSense D435** (front) + **D435i** (rear) — depth + RGB, D435i's BMI055 IMU disabled
- **WitMotion WT901** — redundant / backup IMU

## Target platforms

| Platform | `platform:=` | `base_link` source |
|----------|--------------|---------------------|
| Unitree Go2 Pro | `go2` | Go2 SDK |
| Lifesize R2D2 | `r2d2` | URDF via `robot_state_publisher` |
| Sharper Image Roboscout | `roboscout` | URDF via `robot_state_publisher` |
| Mecanum 4-wheel UGV | `mecanum` | URDF via `robot_state_publisher` |

Each platform ships its own `urdf/<platform>.urdf.xacro` with measured offsets and its own `base_link → body` TF bridge entry in `launch/slam.launch.py`.

## SLAM stack

- **FAST-LIO2** — LiDAR-inertial odometry (Mid-360 built-in IMU, rigid coupling)
- **RTABMap** — graph SLAM with visual loop closure via D435 front RGB + depth
- Output: `/Odometry`, `/cloud_registered_body`, `/map`, `/octomap_full`, persistent `~/.ros/rtabmap.db`

## First-time setup (on the Jetson)

```bash
mkdir -p ~/slam_ws/src && cd ~/slam_ws/src
git clone <this-repo-url> slam_bringup
cd slam_bringup
./install.sh
```

`install.sh` clones the three vendor drivers (`livox_ros_driver2`, `witmotion_ros@ros2`, `FAST_LIO_ROS2`), builds Livox-SDK2, apt-installs RealSense + RTABMap + Nav2 + CycloneDDS, runs `rosdep`, adds the user to `dialout`, exports `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, and runs the first `colcon build` with the livox-required cmake args.

One-time per platform: network interfaces (`eth0` Go2 control, `eth1` LiDAR on `192.168.1.0/24`) — see PLAN.md §6.1.

## Dev loop (after edits)

```bash
cd ~/slam_ws/src/slam_bringup
git pull
./build.sh                        # fast rebuild of slam_bringup only
source ~/slam_ws/install/setup.bash
```

`build.sh` flags:

| Command | Purpose |
|---------|---------|
| `./build.sh` | Build only `slam_bringup` — fast |
| `./build.sh --all` | Build the whole workspace (vendor + `slam_bringup`) |
| `./build.sh --clean` | `rm -rf build/slam_bringup install/slam_bringup` first |
| `./build.sh --clean --all` | Full clean workspace rebuild |
| `./build.sh <pkg> [<pkg> ...]` | Build specific packages |
| `./build.sh --help` | Show help |

After each build the script runs package / launch / config sanity checks with ✓ / ✗ output so you know the new setup is good before sourcing.

## Running

```bash
# Mid-360 standalone (Phase 1.3 — works today)
ros2 launch slam_bringup mid360.launch.py                   # xfer_format=0, RViz-viewable
ros2 launch slam_bringup mid360.launch.py xfer_format:=1    # CustomMsg for FAST-LIO2

# Planned (not yet implemented — see PLAN.md phases below)
ros2 launch slam_bringup d435.launch.py      enable_rear:=true        # Phase 1.4
ros2 launch slam_bringup witmotion.launch.py                          # Phase 1.5
ros2 launch slam_bringup sensors.launch.py                            # Phase 1.6
ros2 launch slam_bringup perception.launch.py platform:=go2           # Phase 1.7
ros2 launch slam_bringup slam.launch.py       platform:=go2           # Phase 2.5
```

Swap `platform:=r2d2|roboscout|mecanum` when the rig moves. `enable_rear:=true` adds the D435i rear camera.

## Status

Phase 1 in progress. Detailed task checklist in [PLAN.md](./PLAN.md).

- [x] Phase 0 — hardware + Jetson prereqs
- [x] Phase 0.5 — repo + Mac↔Jetson workflow
- [x] Phase 1.1–1.2 — `install.sh` + package skeleton
- [x] Phase 1.3 — Mid-360 standalone
- [ ] Phase 1.4 — D435 (front + optional rear)
- [ ] Phase 1.5 — WitMotion
- [ ] Phase 1.6 — `sensors.launch.py` integration
- [ ] Phase 1.7 — URDF (`go2`, `r2d2`, `roboscout`, `mecanum`) + `perception.launch.py` + rviz
- [ ] Phase 1.8 — Go2 SDK integration check
- [ ] Phase 1.9 — `install.sh` smoke test
- [ ] Phase 1.10 — Dual camera (rear D435i)
- [ ] Phase 2 — FAST-LIO2 + RTABMap SLAM
- [ ] Phase 2.5 — MCAP recording / playback / Foxglove

## Source note

Detailed background, tuning rationale, and hardware history live in the Obsidian vault:
`rico/Robotics/Nvidia Jetson/Jeston Orin Nano Super/Jetson Orin Nano Go2 ROS2 Sensor Bringup.md`
