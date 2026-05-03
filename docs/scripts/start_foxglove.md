# `start_foxglove.sh` — Foxglove Bridge for remote viewing

Runs `foxglove_bridge` so a Foxglove Studio / App on your dev machine can connect to the Jetson's ROS 2 graph over WebSocket. Recommended way to view the stack from off-board — far cheaper than running RViz on the Jetson.

## What it does

`ros2 run foxglove_bridge foxglove_bridge` with our default settings — listens on port `8765` for WebSocket connections. Streams every topic the Studio panel subscribes to.

## Dependencies

- `ros-humble-foxglove-bridge` apt package (`./install.sh`).
- Network reachability from your dev machine to the Jetson (LAN or VPN).

## Parameters

The script wraps the upstream node; edit it directly to change port / address. Common knobs:

| Param | Default | What it does |
|---|---|---|
| `port` | `8765` | WebSocket listen port. |
| `address` | `0.0.0.0` | Listen on all interfaces. |
| `tls` | off | Set up cert + key for HTTPS connections. |
| `topic_whitelist` | (all) | Regex whitelist; useful if Foxglove eats CPU subscribing to everything. |

## Usage

```bash
# On the Jetson
./start_foxglove.sh

# On your dev machine — open Foxglove Studio / App:
#   New Connection → Foxglove WebSocket → ws://gizmo.local:8765
```

## Tips

- **Display Frame** matters: `camera_init` for FAST-LIO views, `map` for RTABMap views, `livox_frame` for sensor-only debug.
- **PointCloud2 Decay time**: set high (e.g. 1e9) to client-side accumulate `/cloud_registered` per-scan clouds into a map view without growing Jetson memory.
- Foxglove **panels** survive across stack restarts (the Bridge re-streams the same topics) — your layout is preserved.

## See also

- Main README §[FAST-LIO2 SLAM workflow](../../README.md#startup-commands-current--phase-3--navigation) for Foxglove display-frame guidance
