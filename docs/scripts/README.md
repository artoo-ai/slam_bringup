# Script Reference

Per-script docs for everything in the repo root. Each page covers what
the script does, what must already be running, every launch arg with
its default and "when to override" guidance, and common usage patterns.
The top-level [README.md](../../README.md) is still the right place for
cross-cutting concepts (TF tree, navigation theory, troubleshooting);
this folder is the lookup-by-script-name reference.

## Index

### Top-level workflows (use these day-to-day)

| Script | Purpose | Doc |
|---|---|---|
| `./start_slam.sh` | Full SLAM stack — URDF + sensors + FAST-LIO2 + RTABMap; builds `~/.ros/rtabmap.db` | [start_slam.md](start_slam.md) |
| `./start_nav.sh` | Navigation mode — loads saved map + Nav2 on top (no new mapping) | [start_nav.md](start_nav.md) |
| `./start_perception.sh` | URDF + sensors with no SLAM — for calibration / sensor debugging | [start_perception.md](start_perception.md) |
| `./start_sensors.sh` | All three sensors (Mid-360 + D435 + WitMotion) without URDF or SLAM | [start_sensors.md](start_sensors.md) |

### Per-stage / per-driver

| Script | Purpose | Doc |
|---|---|---|
| `./start_fast_lio.sh` | FAST-LIO2 mapping node alone | [start_fast_lio.md](start_fast_lio.md) |
| `./start_rtabmap.sh` | RTABMap on top of an already-running FAST-LIO + D435 | [start_rtabmap.md](start_rtabmap.md) |
| `./start_mid360.sh` | Livox Mid-360 driver alone | [start_mid360.md](start_mid360.md) |
| `./start_d435.sh` | RealSense D435 (front) alone | [start_d435.md](start_d435.md) |
| `./start_witmotion.sh` | WitMotion WT901 IMU node alone | [start_witmotion.md](start_witmotion.md) |
| `./start_viz_clip.sh` | z-clip republisher for top-down floorplan view | [start_viz_clip.md](start_viz_clip.md) |
| `./start_yahboom.sh` | Yahboom YB-ERF01 `/cmd_vel` → mecanum bridge | [start_yahboom.md](start_yahboom.md) |
| `./start_bench_tf.sh` | Bench-fixture stopgap static TF (pre-Phase-1.7) | [start_bench_tf.md](start_bench_tf.md) |
| `./start_foxglove.sh` | Foxglove Bridge for remote viewing | [start_foxglove.md](start_foxglove.md) |

### Kill scripts

Every `start_*.sh` has a matching `kill_*.sh`. They all route through
[kill_helpers.sh](../../kill_helpers.sh)'s `nuke_processes` (SIGINT →
SIGKILL → sudo SIGKILL with verification at each step). See
[kill scripts in the main README](../../README.md#kill-scripts-and-orphan-process-handling)
for the full pattern. Scripts:

`./kill_slam.sh` — chains every layer below
`./kill_nav.sh` — Nav2 + pointcloud_to_laserscan
`./kill_rtabmap.sh` — RTABMap node
`./kill_fast_lio.sh` — FAST-LIO + imu_units republisher + viz_clip
`./kill_sensors.sh` — chains kill_mid360 + kill_d435 + kill_witmotion
`./kill_mid360.sh` — Livox driver (verifies orphan-free)
`./kill_d435.sh` — RealSense driver + USB sysfs reset
`./kill_witmotion.sh` — WitMotion node
`./kill_viz_clip.sh` — z-clip republisher
`./kill_yahboom.sh` — Yahboom bridge

## Common dependency chains

```
start_slam.sh                                ← top-level "build a map"
  ├─ start_perception.sh                     (URDF + sensors)
  │   ├─ start_sensors.sh
  │   │   ├─ start_mid360.sh
  │   │   ├─ start_d435.sh
  │   │   └─ start_witmotion.sh
  │   └─ robot_state_publisher (URDF)
  ├─ start_fast_lio.sh                       (LiDAR-inertial odometry)
  │   ├─ slam_bringup/imu_units_g_to_ms2     (g → m/s² republisher)
  │   └─ start_viz_clip.sh                   (top-down section view)
  └─ start_rtabmap.sh                        (graph SLAM + loop closure)

start_nav.sh                                 ← top-level "navigate the map"
  ├─ start_slam.sh layers above (in localization mode)
  ├─ launch/nav2.launch.py
  │   ├─ pointcloud_to_laserscan             (/cloud_registered_body → /scan)
  │   ├─ static TF camera_init → odom
  │   └─ nav2_bringup/navigation_launch.py   (controller / planner / BT / …)
  └─ (optional, mecanum only) start_yahboom.sh   /cmd_vel → motors
```

If a script's "Dependencies" section says **"composes via slam.launch.py"**,
running its top-level wrapper (e.g. `./start_slam.sh`) handles the chain
automatically. The per-stage scripts are useful only for isolated
debugging.

## Conventions used in these docs

- **Defaults** are what you get with `./start_<x>.sh` (no args).
- **`arg:=value`** is ROS 2 launch syntax — pass through any of the
  scripts that exec `ros2 launch`.
- **"When to override"** describes the actual symptom that calls for
  the override, not just the abstract knob description.
- Required apt packages are listed under **Dependencies** for each
  script; the umbrella `./install.sh` covers them all.
