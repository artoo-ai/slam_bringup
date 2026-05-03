# `start_viz_clip.sh` — top-down floorplan view

Standalone z-clip republisher. Subscribes `/cloud_registered` (or any cloud), clips the z field to `[viz_z_min, viz_z_max]`, republishes to `/cloud_viz_clipped`. Use against a bag replay or alongside an already-running pipeline that doesn't auto-spawn viz_clip.

`start_slam.sh` and `start_fast_lio.sh` already auto-spawn this; you only need standalone for bag replay or custom topic remaps.

## What it does

`ros2 launch slam_bringup viz_clip.launch.py` — runs `slam_bringup/viz_clip_node.py` (rclpy, numpy mask, ~10 Hz on Orin Nano). The raw `/cloud_registered_body` still flows into FAST-LIO/RTABMap unchanged; this is purely visualization.

## Why a custom Python node?

`pcl_ros::PassThrough` was the obvious choice but the Humble `ros-humble-pcl-ros` 2.4.5 deb does NOT consistently expose its filter classes as composable-node plugins (`class_loader` fails the lookup). The rclpy node sidesteps the packaging issue.

## Parameters

| Arg | Default | What it does | When to override |
|---|---|---|---|
| `enable_viz_clip` | `true` | Toggle the node. | `false` to disable when included from a parent launch. |
| `viz_input_topic` | `/cloud_registered` | Cloud to clip. | `/Laser_map` for accumulated maps; bag-replay topic names. |
| `viz_output_topic` | `/cloud_viz_clipped` | Republished topic. | Custom names if you have multiple clips. |
| `viz_z_min` | `-3.0` m | Lower bound (in `camera_init` frame). | Raise to skip the floor. |
| `viz_z_max` | `3.0` m | Upper bound. | `4.5` for high-ceiling spaces. |

`z = 0` is the body pose at FAST-LIO startup, NOT the floor. Tune against the rig's startup height.

## Runtime tuning

The node accepts ROS params — change z bounds without restarting:

```bash
ros2 param set /viz_z_clip z_max 4.5
ros2 param set /viz_z_clip z_min -1.0
```

## Examples

```bash
# Default — clips /cloud_registered, publishes /cloud_viz_clipped
./start_viz_clip.sh

# Tighter slice for a clean section view:
./start_viz_clip.sh viz_z_min:=0.5 viz_z_max:=1.5

# Against a bag-replay topic:
./start_viz_clip.sh viz_input_topic:=/replay/cloud_registered
```

## See also

- [start_fast_lio.md](start_fast_lio.md) — auto-spawns this
- [start_slam.md](start_slam.md) — full stack
- Main README §[Top-down floorplan view](../../README.md#top-down-floorplan-view-cloud_viz_clipped)
