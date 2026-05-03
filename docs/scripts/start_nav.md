# `start_nav.sh` — navigation mode

Loads a previously-built `~/.ros/rtabmap.db` in localization mode and runs Nav2 on top. Use this to **drive an autonomous goal** in a saved map. To make a NEW map, use [`start_slam.sh`](start_slam.md).

## What it does

Wraps `slam.launch.py` with four forced args:
- `localization:=true` (RTABMap read-only)
- `nav2:=true` (Nav2 navigation pipeline included)
- `delete_db_on_start:=false` (refuses to clobber the map)
- `enable_drive:=true` (spawns the Yahboom mecanum bridge — pass `enable_drive:=false` to override on bench fixture)

Refuses to launch if the saved DB doesn't exist, or if you try to pass `delete_db_on_start:=true`. Everything else is a normal slam.launch.py arg.

Foxglove bridge auto-spawns. View on your dev machine:
`Foxglove Studio → New Connection → ws://gizmo.local:8765`.

What runs:
- `start_slam.sh`'s full layer stack in localization mode (RTABMap loads DB, no new keyframes)
- `pointcloud_to_laserscan` (`/cloud_registered_body` → `/scan` for Nav2 costmaps)
- `camera_init → odom` identity TF (so Nav2's stock plugin frame names resolve)
- `nav2_bringup/navigation_launch.py` (controller / planner / smoother / behavior / BT navigator / waypoint follower / velocity smoother / lifecycle_manager)

NOT run: `map_server` (RTABMap publishes `/map`) or `amcl` (RTABMap publishes the `map → camera_init` correction).

## Dependencies

- `~/.ros/rtabmap.db` must exist (or pass `database_path:=...`). Build it first with `./start_slam.sh`.
- apt: `ros-humble-nav2-bringup`, `ros-humble-pointcloud-to-laserscan` — `./build.sh` verifies these are installed.
- For RViz panels: `ros-humble-nav2-rviz-plugins`, `ros-humble-slam-toolbox` (panels referenced by `rviz/perception.rviz`).
- For the rover to physically move: a `/cmd_vel` consumer. On the mecanum platform that's the Yahboom bridge — pass `enable_drive:=true`.

## Default usage

```bash
./start_nav.sh
```

What you get:
- Platform: `mecanum` (the rover physically moves on goal-pose clicks)
- Loads `~/.ros/rtabmap.db`
- `force_3dof: true` (clamps z/roll/pitch on the wheeled rover)
- Drive bridge ON (`enable_drive:=true` forced — Yahboom subscribed to `/cmd_vel`)
- Foxglove bridge auto-spawns
- No Jetson-side RViz

## Parameters

`start_nav.sh` accepts every `slam.launch.py` arg AND every `nav2.launch.py` arg. The most relevant:

| Arg | Default | What it does | When to override |
|---|---|---|---|
| `platform` | `bench_fixture` | URDF + body→base_link bridge. | `mecanum` on the rover. |
| `force_3dof` | `false` | Clamp z/roll/pitch to 0 in RTABMap. | **Always `true` on the rover.** Without it, drift propagates into the live `map → base_link` and Nav2 plans against the wrong pose. |
| `enable_drive` | `false` | Spawn the per-platform `/cmd_vel → motor` bridge. | `true` on the mecanum rover. Without this, Nav2 commands `/cmd_vel` but no motors move. |
| `database_path` | `~/.ros/rtabmap.db` | Saved map to load. | `~/maps/lab-2026-05-03.db` for a snapshot. |
| `nav2_params_file` | `config/nav2_params.yaml` | Costmap + planner + controller + BT settings. | Per-platform tuning. |
| `nav2_autostart` | `true` | Lifecycle-manager auto-activates Nav2 nodes. | `false` to step through lifecycle manually. |
| `scan_min_height` | `0.10` m | Lower bound of the pointcloud→laserscan slice (above `base_link`). | Lower for very-low-clearance platforms. |
| `scan_max_height` | `0.45` m | Upper bound. Default catches typical rover collision height. | **`~0.8` for Go2 standing**, **`~1.2` for the bench fixture on a table**, otherwise the laserscan has no data. |
| `rviz` | `false` | Jetson-side RViz. | Almost never — use a dev-machine RViz/Foxglove. |
| `use_sim_time` | `false` | `/clock` from a bag replay. | `true` only with `ros2 bag play`. |

## Examples

```bash
# Mecanum rover, full autonomous nav (THE ROVER MOVES):
./start_nav.sh platform:=mecanum force_3dof:=true enable_drive:=true

# Bench-test Nav2 planning without driving anything (planner viz only):
./start_nav.sh force_3dof:=true       # /cmd_vel publishes but no drive bridge

# Use a snapshot map instead of the live DB:
./start_nav.sh database_path:=~/maps/lab-2026-05-03.db force_3dof:=true

# Tall robot — raise the laserscan slice ceiling so it sees obstacles:
./start_nav.sh platform:=mecanum force_3dof:=true enable_drive:=true scan_max_height:=0.8
```

## Gotchas

- **`Localization: inactive`** in the Nav2 RViz panel is **expected and harmless**. That indicator monitors `lifecycle_manager_localization` (AMCL + map_server), which we deliberately don't run. Verify RTABMap localization via the rtabmap log (`Loop closure detected ...`) instead.
- **Goal aborts immediately with "Distance remaining: 0.00 m"** — usually one of: (a) RTABMap hasn't relocalized yet (use **2D Pose Estimate** to seed `/initialpose`, or drive ~1 m to trigger a loop closure), (b) the costmap is solid magenta (`scan_max_height` too high or `inflation_radius` too aggressive — see Troubleshooting in the main README), (c) FAST-LIO drift puts the robot in an unknown costmap cell (turn on `force_3dof:=true`).
- **"Behavior tree threw exception: Empty Tree"** — fixed in commit `4b9d402`. If you see this, you're on stale code; `git pull` + `./build.sh`.
- **`/cmd_vel` publishes but rover doesn't move** — the drive bridge isn't running. Add `enable_drive:=true` (mecanum only) or wire your platform's bridge.

## Sending a goal

```bash
# RViz: click "2D Goal Pose", drop on the map
# CLI:
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}" --once
```

## See also

- [start_slam.md](start_slam.md) — build the map first
- [start_yahboom.md](start_yahboom.md) — the mecanum drive bridge
- Main README §[Navigation (Nav2)](../../README.md#navigation-nav2)
- Main README §[Troubleshooting Nav2 + RTABMap on a real rover](../../README.md#troubleshooting-nav2--rtabmap-on-a-real-rover)
