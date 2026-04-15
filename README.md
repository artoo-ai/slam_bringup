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

## Quick start (once implemented)

```bash
# On gizmo (Jetson), after clone + ./install.sh + source
ros2 launch slam_bringup perception.launch.py platform:=go2   # sensors + URDF + rviz
ros2 launch slam_bringup slam.launch.py       platform:=go2   # full SLAM stack
```

Swap `platform:=r2d2|roboscout|mecanum` when the rig moves. `enable_rear:=true` adds the D435i rear camera.

## Status

**Planning phase.** See [PLAN.md](./PLAN.md) for the full multi-phase implementation plan (Phase 0.5 repo/sync, Phase 1 raw sensors, Phase 2 SLAM, Phase 2.5 recording/playback).

## Source note

Detailed background, tuning rationale, and hardware history live in the Obsidian vault:
`rico/Robotics/Nvidia Jetson/Jeston Orin Nano Super/Jetson Orin Nano Go2 ROS2 Sensor Bringup.md`
