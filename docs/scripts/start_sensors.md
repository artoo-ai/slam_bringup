# `start_sensors.sh` — all three sensors

`sensors.launch.py` — Mid-360 + D435 + WitMotion in one launch. No URDF, no SLAM. Pre-cleans every per-sensor driver before relaunching.

## What it does

Chains all three per-sensor launches with shared args. After this returns, you have:
- `/livox/lidar` (PointCloud2 by default, CustomMsg with `lidar_xfer_format:=1`)
- `/livox/imu` (Mid-360 internal IMU, units of g)
- `/d435_front/camera/color/image_raw`, `/d435_front/camera/depth/image_rect_raw`, etc.
- `/imu/data` (WitMotion WT901)

## Parameters

| Arg | Default | What it does | When to override |
|---|---|---|---|
| `lidar_xfer_format` | `0` | Mid-360 wire format. 0 = PointCloud2 (RViz/Foxglove), 1 = CustomMsg (FAST-LIO). | `1` for FAST-LIO; matches what `slam_mode:=true` enforces. |
| `slam_mode` | `false` | D435 `align_depth.enable=true`, RealSense pointcloud OFF. | `true` when SLAM will layer on top — saves CPU. |
| `enable_mid360` | `true` | Run Mid-360. | `false` for camera-only debug. |
| `enable_d435` | `true` | Run D435 front. | `false` for LiDAR-only debug. |
| `enable_witmotion` | `true` | Run WitMotion. | `false` for FAST-LIO (uses Mid-360 IMU). |
| `enable_rear` | `false` | Bring up rear D435i (Phase 1.10). | Wait for dual-camera Phase. |

## Examples

```bash
# Default — all three sensors, RViz-viewable cloud:
./start_sensors.sh

# FAST-LIO-ready (LiDAR-only, CustomMsg):
./start_sensors.sh lidar_xfer_format:=1 enable_d435:=false enable_witmotion:=false

# RTABMap-ready (LiDAR + D435 with align_depth):
./start_sensors.sh slam_mode:=true
```

## See also

- [start_perception.md](start_perception.md) — same plus URDF
- [start_mid360.md](start_mid360.md) / [start_d435.md](start_d435.md) / [start_witmotion.md](start_witmotion.md) — individual drivers
