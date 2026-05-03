# `start_perception.sh` — URDF + sensors, no SLAM

Launches the URDF (`robot_state_publisher`), all three sensors, and an optional `rviz2` window. **No FAST-LIO, no RTABMap, no Nav2.** Use for sensor calibration, URDF debugging, or recording a bag without the SLAM compute load.

## What it does

`ros2 launch slam_bringup perception.launch.py`, which composes:
- `robot_state_publisher` with the per-platform URDF
- `sensors.launch.py` (Mid-360 + D435 + WitMotion)
- Optional `rviz2` with `rviz/perception.rviz`

## Dependencies

- All sensors physically connected.
- Workspace built.

## Parameters

| Arg | Default | What it does | When to override |
|---|---|---|---|
| `platform` | `bench_fixture` | Selects URDF (`urdf/<platform>.urdf.xacro`). | `mecanum`, `go2`, `r2d2`, `roboscout` once URDFs land. |
| `rviz` | `false` | Spawn RViz on the Jetson with `rviz/perception.rviz`. | `true` only on dev machine — Jetson CPU. |
| `rviz_config` | `rviz/perception.rviz` | RViz config path. | Custom view layouts. |
| `slam_mode` | `false` | D435 `align_depth.enable=true`, RealSense pointcloud OFF. Mid-360 `xfer_format=1`. | `true` when you'll layer FAST-LIO/RTABMap on top. |
| `lidar_xfer_format` | `0` (PointCloud2) | `0` = RViz/Foxglove viewable, `1` = CustomMsg for FAST-LIO. | `1` for SLAM. |
| `enable_witmotion` | `true` | Run the WitMotion node. | `false` for FAST-LIO (uses Mid-360 IMU). |
| `enable_mid360` | `true` | Run Mid-360 driver. | `false` for D435-only debug. |
| `enable_d435` | `true` | Run D435 front. | `false` for LiDAR-only debug. |
| `enable_rear` | `false` | Spawn the rear D435i (Phase 1.10 stub). | `true` once dual-camera lands. |

## Examples

```bash
# All sensors + URDF, viewer-friendly cloud format:
./start_perception.sh

# With dev-machine RViz showing the URDF + live clouds:
./start_perception.sh rviz:=true

# SLAM-mode preview (matches what start_slam.sh sees):
./start_perception.sh slam_mode:=true lidar_xfer_format:=1

# Mecanum platform once URDF lands:
./start_perception.sh platform:=mecanum
```

## See also

- [start_sensors.md](start_sensors.md) — sensors without URDF
- [start_slam.md](start_slam.md) — adds FAST-LIO + RTABMap on top
- Main README §[Sensors](../../README.md#sensors-shared-across-all-platforms)
