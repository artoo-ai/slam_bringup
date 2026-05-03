# `start_rtabmap.sh` — RTABMap graph SLAM

Brings up RTABMap on top of an already-running FAST-LIO + D435. Preflight checks 5 input topics + the `body→d435_front_link` static TF before launching.

Usually called via [`start_slam.sh`](start_slam.md). Standalone for RTABMap-only debugging.

## What it does

`ros2 launch slam_bringup rtabmap.launch.py` — runs `rtabmap_slam/rtabmap` in **external-odometry mode** (FAST-LIO owns pose; RTABMap consumes it, runs ICP scan refinement, builds the graph, and detects loop closures via D435 RGB BoW). Saves ~20–40% CPU on the Orin Nano vs. letting RTABMap compute its own visual odometry.

## Dependencies

- `start_fast_lio.sh` running — RTABMap subscribes `/Odometry` and `/cloud_registered_body`.
- `start_d435.sh` (or `start_sensors.sh slam_mode:=true`) — RTABMap subscribes the RGB image, aligned depth, and camera_info.
- `body → d435_front_link` static TF available — published by `robot_state_publisher` in `start_perception.sh`, OR by `start_bench_tf.sh` as a stopgap.
- apt: `ros-humble-rtabmap-ros` (`./install.sh`).

## Parameters

| Arg | Default | What it does | When to override |
|---|---|---|---|
| `database_path` | `~/.ros/rtabmap.db` | Where the graph + visual word DB persists. | Per-map snapshots. |
| `delete_db_on_start` | `false` | Wipe DB at launch. | `true` for a fresh map (gates against accidental wipe at the launch script level). |
| `localization` | `false` | `Mem/IncrementalMemory: false` — read-only DB load. | `true` for navigation. |
| `force_3dof` | `false` | `Reg/Force3DoF: true` — clamps z/roll/pitch in graph optimization. | **Always `true` on wheeled rovers**. |
| `viz` | `false` | Spawn `rtabmap_viz` GUI (heavy on Jetson). | Only on dev machine. |
| `rgb_topic` / `depth_topic` / `camera_info_topic` | D435 front | RTABMap input topic remaps. | Custom camera setups. |

## Tuning levers in `launch/rtabmap.launch.py`

| Param | Default | What it does |
|---|---|---|
| `Reg/Strategy` | `1` | 0=visual, 1=ICP, 2=visual+ICP. Livox needs ICP. |
| `Grid/NormalsSegmentation` | `false` | **CRITICAL — must be false for Livox** (Mid-360 emits unorganized clouds; segmentation expects organized rows/cols and produces a blank `/map`). |
| `RGBD/OptimizeMaxError` | `5.0` σ | Bumped from 3.0 — accept large loop-closure corrections against drift-laden saved maps. |
| `RGBD/LocalRadius` | `2.0` m | Bumped from 1.0 — neighbouring keyframes look 1.06–1.13 m apart in odom space due to FAST-LIO drift. |
| `Vis/MinInliers` | `15` | Loop-closure threshold on visual feature matches. |
| `Rtabmap/DetectionRate` | `1.0` Hz | Loop-closure check rate. Lower to `0.5` if Jetson is saturated. |

## Gotchas

- **Blank `/map`** with Livox — `Grid/NormalsSegmentation` not `false`. Already set; if you see this, you're on stale code.
- **`Rejecting all added loop closures` flood** — drift-laden saved map; `RGBD/OptimizeMaxError` at 5.0 mostly fixes it. Re-mapping with `force_3dof:=true` is the long-term fix.
- **`Ignoring local loop closure ... resulting transform is too large`** — `RGBD/LocalRadius` at 2.0 m mostly fixes it. Same root cause as above.
- **`Not found word X (dict size=0)`** — DB was loaded with corrupted state. Pass `delete_db_on_start:=true` or wipe the file.

## See also

- [start_slam.md](start_slam.md) — full chain
- [start_fast_lio.md](start_fast_lio.md) — odometry layer below
- Main README §[Map persistence](../../README.md#map-persistence-and-rosrtabmapdb)
