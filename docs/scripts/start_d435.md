# `start_d435.sh` — Intel RealSense D435 (front)

Standalone front-D435 bring-up. Auto-cleans a stale `realsense2_camera_node` and resets the USB at sysfs level.

## What it does

`ros2 launch slam_bringup d435.launch.py` — `realsense2_camera_node` under namespace `d435_front`. With `slam_mode:=true`, enables `align_depth` and disables the on-board pointcloud (saves CPU; RTABMap projects depth itself).

## Dependencies

- D435 plugged into a USB 3.x port (USB 2 won't deliver enough bandwidth for color+depth+IMU).
- `realsense2_camera` apt package — `./install.sh`.
- Workspace built.

## Parameters

| Arg | Default | What it does | When to override |
|---|---|---|---|
| `slam_mode` | `false` | `align_depth.enable=true` + `pointcloud.enable=false`. | `true` for RTABMap; the on-board pointcloud is redundant and ~30% CPU. |
| `serial_no` | (auto) | Pin to a specific serial number. | Multi-camera setups — front D435 vs rear D435i. |
| `camera_namespace` | `d435_front` | Topic prefix. | `d435_rear` for the second camera. |

## Examples

```bash
./start_d435.sh                  # raw mode — pointcloud on, no align_depth
./start_d435.sh slam_mode:=true  # RTABMap-ready
```

## Gotchas

- **"Frames didn't arrive within 5 seconds"** — USB handle stuck. `./kill_d435.sh` does a sysfs `authorized=0/1` reset (needs sudo) which clears most firmware hangs. If still failing: physically replug.
- **Stale handle on next launch** — `kill_d435.sh` waits for the rs node to exit before touching sysfs to avoid races.
- **D435 vs D435i** — D435i has BMI055 IMU; we **disable** it because Mid-360's onboard IMU is the FAST-LIO authority.

## See also

- [start_sensors.md](start_sensors.md) — bundled with Mid-360 + WitMotion
- `scripts/measure_d435_pitch.py` — calibrate the front-D435 mounting pitch against the floor
