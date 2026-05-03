# `start_fast_lio.sh` — FAST-LIO2 LiDAR-inertial odometry

Brings up FAST-LIO2 against an already-running Mid-360 driver. Publishes `/Odometry` (10 Hz), `/cloud_registered` (per-scan in `camera_init`), `/cloud_registered_body` (per-scan in `body`), and TF `camera_init → body`. Almost always called via [`start_slam.sh`](start_slam.md); use this script standalone for FAST-LIO-only debugging.

## What it does

1. Kills any prior FAST-LIO + viz_clip + imu_units instances.
2. Preflight: verifies `imu_units_g_to_ms2` is in `slam_bringup`'s entry_points (catches stale `--symlink-install` builds).
3. Splits args — `viz_*` and `enable_viz_clip` go to a backgrounded `viz_clip.launch.py`; the rest go to `fast_lio.launch.py`.
4. `fast_lio.launch.py` spawns:
   - `slam_bringup/imu_units_g_to_ms2` — rescales `/livox/imu` (units of g, per Livox 1.2.6) to `/livox/imu_ms2` (m/s²) so FAST-LIO sees correct gravity.
   - Upstream `FAST_LIO_ROS2/mapping.launch.py` (with `rviz=false` baked in here).

## Dependencies

- **Mid-360 must already be running** in `xfer_format=1` (CustomMsg) mode. Easiest: `./start_sensors.sh lidar_xfer_format:=1 enable_d435:=false enable_witmotion:=false`.
- Workspace built (`./build.sh`).
- `Rosmaster_Lib` NOT needed for this script.

## Default usage

```bash
# Terminal 1 — Mid-360 in CustomMsg mode
./start_sensors.sh lidar_xfer_format:=1 enable_d435:=false enable_witmotion:=false

# Terminal 2 — FAST-LIO + viz_clip
./start_fast_lio.sh
```

What you get: `/Odometry` at 10 Hz after ~30 s settle, `/cloud_viz_clipped` for top-down view.

## Parameters

All args pass through to `fast_lio.launch.py` and `viz_clip.launch.py`:

| Arg | Default | What it does | When to override |
|---|---|---|---|
| `config_file` | `fast_lio_mid360.yaml` | FAST-LIO YAML in `config/`. | Custom tuning; copy + edit + point at the new file. |
| `rviz` | `false` | Spawn upstream FAST-LIO's RViz with `fastlio.rviz`. | Only when debugging FAST-LIO alone — gives you `/Odometry`, `/Path`, `/cloud_registered`, `/Laser_map` pre-configured. |
| `enable_viz_clip` | `true` | Run the z-clip republisher. | `false` for FAST-LIO-only output. |
| `viz_z_min` / `viz_z_max` | `-3.0` / `3.0` m | Clip range in `camera_init`. | `4.5` for high-ceiling spaces. |

## Examples

```bash
# Default — FAST-LIO + viz_clip
./start_fast_lio.sh

# With FAST-LIO's debug RViz (its own config; not perception.rviz):
./start_fast_lio.sh rviz:=true

# FAST-LIO without viz_clip (faster startup, raw /cloud_registered only):
./start_fast_lio.sh enable_viz_clip:=false
```

## Tuning lever — `config/fast_lio_mid360.yaml`

| Param | Default | What it does |
|---|---|---|
| `acc_cov` | `0.5` | IMU accel covariance. Higher = trust ICP more, IMU less. Bumped from 0.1 after IMU unit fix. Drop to 0.1 if odom shakes during fast turns. |
| `gyr_cov` | `0.1` | IMU gyro covariance. |
| `filter_size_surf` / `filter_size_map` | `0.3` m | Voxel grid for ICP. Smaller catches more features but burns CPU. |
| `blind` | `0.5` m | Self-hit filter — drops returns inside this radius. **Must clear all rig fixtures.** |
| `extrinsic_T` / `extrinsic_R` | Mid-360 datasheet | IMU→LiDAR extrinsic. Recalibrate via `scripts/measure_imu_tilt.py` if rig geometry changes. |
| `scan_bodyframe_pub_en` | `true` | **REQUIRED** — RTABMap subscribes to `/cloud_registered_body`. |

## See also

- [start_slam.md](start_slam.md) — full SLAM stack (the usual entry point)
- [start_viz_clip.md](start_viz_clip.md) — top-down floorplan view
- [start_mid360.md](start_mid360.md) — Mid-360 driver alone
- Main README §[FAST-LIO2 operating conditions](../../README.md#startup-commands-current--phase-3--navigation)
