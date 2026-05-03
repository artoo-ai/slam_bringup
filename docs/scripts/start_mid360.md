# `start_mid360.sh` — Livox Mid-360 driver

Standalone Mid-360 bring-up. Auto-cleans a stale `livox_ros_driver2_node` first.

## What it does

`ros2 launch slam_bringup mid360.launch.py` — runs `livox_ros_driver2_node` against `config/mid360.json`. Publishes `/livox/lidar` and `/livox/imu`.

## Dependencies

- Mid-360 powered + connected to the Jetson's secondary Ethernet at `192.168.1.100/24`.
- Lidar IP `192.168.1.202`. Verify: `ping 192.168.1.202`.
- Workspace built.

## Parameters

| Arg | Default | What it does | When to override |
|---|---|---|---|
| `xfer_format` | `0` | Wire format. 0 = PointCloud2, 1 = CustomMsg. | `1` for FAST-LIO. |
| `frame_id` | `livox_frame` | TF frame on the published clouds. | Almost never. |
| `user_config_path` | `share/slam_bringup/config/mid360.json` | Driver config. | Custom IP / port assignments. |

## Examples

```bash
./start_mid360.sh                      # PointCloud2 (RViz / Foxglove)
./start_mid360.sh xfer_format:=1       # CustomMsg (FAST-LIO)
```

## Gotchas

- **Two publishers on `/livox/lidar`** — orphan from previous run. The new `kill_mid360.sh` (commit `c7e4184`) verifies a clean kill via `pgrep` after `pkill -9` and exits 1 if a survivor is found. Symptom in FAST-LIO if missed: `lidar loop back, clear buffer` flood.
- **`The message type 'livox_ros_driver2/msg/CustomMsg' is invalid`** when running `ros2 topic hz /livox/lidar` — that's `ros2 topic hz` not loading custom message types. Not a bug; ignore. Use `/livox/lidar` count via `ros2 topic info` to confirm the driver is publishing.
- **Driver fails to bind UDP sockets** — orphan holding `:5610x`. `./kill_mid360.sh` then re-launch.

## See also

- [start_sensors.md](start_sensors.md) — bundled with D435 + WitMotion
- Main README §[Network setup](../../README.md#network-setup-one-time-per-jetson)
