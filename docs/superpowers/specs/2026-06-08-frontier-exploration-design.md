# Frontier Exploration — Autonomous Mapping Mode

**Date:** 2026-06-08
**Status:** Approved design, pending implementation

## Summary

Add autonomous exploration to the SLAM stack so the robot can map a house without manual RC driving. The robot identifies unexplored frontiers on the occupancy grid, drives toward them via Nav2, and repeats until either a time limit expires or no frontiers remain. It then saves the map and returns to its starting position.

Two shell scripts mirror the existing `start_slam.sh` / `start_slam_2d.sh` pattern:
- `start_explore.sh` — 3D (FAST-LIO2 + RTABMap + Nav2 + explore_lite)
- `start_explore_2d.sh` — 2D (rf2o + slam_toolbox + Nav2 + explore_lite)

Both support `resume:=true` to continue from a previously saved partial map.

## Architecture

```
start_explore.sh / start_explore_2d.sh
  │
  ├─ Existing SLAM stack (unchanged)
  │   3D: perception → FAST-LIO2 → RTABMap → Nav2
  │   2D: perception → pointcloud_to_laserscan → rf2o → slam_toolbox → Nav2
  │   Publishes: /map, /Odometry (or /odom), /scan, TF
  │
  └─ explore.launch.py (NEW)
      ├─ explore_lite node (m-explore-lite package)
      │   Reads /map costmap, sends NavigateToPose goals to Nav2
      └─ explore_manager node (NEW, slam_bringup package)
          Timer, frontier monitor, map save, return-to-home
```

### New Files

| File | Purpose |
|------|---------|
| `launch/explore.launch.py` | Wraps explore_lite + explore_manager, remaps topics per 3D/2D mode |
| `config/explore_params.yaml` | explore_lite tuning + explore_manager settings |
| `slam_bringup/explore_manager_node.py` | Orchestrates exploration lifecycle |
| `start_explore.sh` | 3D autonomous explore script |
| `start_explore_2d.sh` | 2D autonomous explore script |
| `docs/visualizations/frontier_explorer.html` | Interactive parameter reference (Make or Code branded) |

### Modified Files

| File | Change |
|------|--------|
| `setup.py` | Add `explore_manager` console_scripts entry point |

No other existing files are modified.

## explore_manager Node — Lifecycle

```
START
  │
  ▼
Record home pose (current robot position in map frame)
  │
  ▼
Wait for /map + Nav2 lifecycle ready
  │
  ▼
┌─► EXPLORING
│   explore_lite sends NavigateToPose goals to Nav2
│   Manager publishes /explore/status every 5s:
│     • state (EXPLORING / SAVING / RETURNING / COMPLETE)
│     • time_remaining_s
│     • frontier_count
│     • free_cells_mapped
│
│   Check every 5s:
│   ├─ time_limit exceeded? ──────────► SAVING MAP
│   ├─ frontier_count == 0 for 30s
│   │  consecutive? ──────────────────► SAVING MAP
│   └─ neither → continue loop
│
│                    ▼
│             SAVING MAP
│             3D: ros2 service call /rtabmap/backup
│                 copies rtabmap.db → ~/maps/explore_<timestamp>.db
│             2D: ros2 service call /slam_toolbox/serialize_map
│                 saves to ~/maps/explore_<timestamp> (.data + .posegraph)
│                    │
│                    ▼
│             RETURNING HOME
│             Set explore_lite's 'explore' parameter to false (stops goal publishing)
│             Send NavigateToPose action goal to saved home pose
│             If nav goal fails → retry once, then stop in place and log warning
│                    │
│                    ▼
│             COMPLETE
│             Publish final stats
│             Node stays alive (map still served for Foxglove visualization)
└─────────────────────────────────────────────────────────────────────────
```

### Frontier Staleness Guard

Frontiers must read zero for 30 consecutive seconds before declaring exploration complete. This avoids false stops when RTABMap is processing a loop closure and the occupancy grid temporarily has no frontier cells.

### Graceful Shutdown

On SIGINT (Ctrl-C from shell script), the manager:
1. Cancels the current Nav2 goal
2. Publishes zero-velocity Twist to `/cmd_vel`
3. Shuts down cleanly

### Status Topic

`/explore/status` — published as `std_msgs/String` containing JSON:

```json
{
  "state": "EXPLORING",
  "time_remaining_s": 542,
  "time_elapsed_s": 358,
  "frontier_count": 7,
  "free_cells_mapped": 48230,
  "home_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}
}
```

Using `std_msgs/String` with JSON avoids needing a custom message type and is directly readable in Foxglove.

## explore_lite — How It Works

The `m-explore-lite` package implements frontier-based exploration:

1. **Frontier detection:** Scans the occupancy grid (`/map`) for cells adjacent to both free space (value 0) and unknown space (value -1). These boundary cells are "frontiers."
2. **Frontier clustering:** Groups adjacent frontier cells into clusters. Clusters smaller than `min_frontier_size` are discarded (tiny crevices, sensor noise).
3. **Goal selection:** Ranks clusters by a weighted score combining distance cost (closer = cheaper) and information gain (larger frontier = more new area). Controlled by `potential_scale` and `gain_scale`.
4. **Goal execution:** Sends the winning frontier centroid as a `NavigateToPose` action goal to Nav2.
5. **Re-evaluation:** Every `planner_frequency` seconds (default 3s), re-scans the map. If a better frontier appears or the current goal is reached, sends a new goal.
6. **Multi-area coverage:** The frontier list is global across the entire map. When one room is fully explored, the next-best frontier is in a different room — the robot navigates there automatically. No restart needed.

### Starting Position

Does not matter. In mapping mode, the robot's power-on position becomes `(0, 0, 0)` in the map frame. SLAM initializes from wherever the robot is. Initial pose only matters when localizing against an existing map (localization mode).

## Nav2 Recovery Behaviors

When the robot gets stuck (controller fails to make progress within `movement_time_allowance: 10.0s`), Nav2's behavior tree triggers recovery behaviors in sequence:

1. **Spin** — Rotate in place to clear costmap artifacts and find a new path
2. **Backup** — Reverse ~0.3m to free the robot from tight spots
3. **Wait** — Pause 2 seconds, let the costmap refresh with new sensor data
4. **Clear costmap** — Wipe stale obstacle data from the costmap and re-plan

If all recoveries fail, Nav2 aborts that goal. explore_lite then picks the next-best frontier and tries a different direction. The robot naturally routes around unreachable areas.

These behaviors are already configured in `config/nav2_params.yaml` under `behavior_server` — no changes needed.

## Resume Mode

Both scripts accept `resume:=true` to continue from a previously saved map.

### 3D Resume (RTABMap)

- Sets `delete_db_on_start:=false` so RTABMap loads the existing `~/.ros/rtabmap.db`
- Keeps `localization:=false` (mapping mode) so new areas are added to the graph
- RTABMap re-localizes via ICP and visual loop closure against the existing map
- explore_lite sees existing mapped area + unknown edges and picks up where it left off

### 2D Resume (slam_toolbox)

- Deserializes a previously saved map file via `map_file_name` parameter
- Runs in async mapping mode (not localization) so new areas are stitched onto the existing graph
- The explore script accepts `map_file:=~/maps/explore_20260608_1430` to specify which map to resume from

### Constraint

The robot must start in or near already-mapped territory so SLAM can recognize where it is. If started in a completely unmapped room, it builds a disconnected map until it drives into known space, at which point SLAM merges the graphs.

## Configuration — config/explore_params.yaml

```yaml
explore_lite:
  ros__parameters:
    costmap_topic: /map
    min_frontier_size: 0.75          # meters — ignore tiny crevice frontiers
    potential_scale: 3.0             # weight: prefer closer frontiers (distance cost)
    gain_scale: 1.0                  # weight: prefer larger frontiers (info gain)
    transform_tolerance: 0.3
    planner_frequency: 0.33          # re-evaluate frontiers every ~3s
    progress_timeout: 30.0           # seconds without progress → pick new frontier
    visualize_frontiers: true        # publish markers for Foxglove/RViz
    robot_base_frame: base_link

explore_manager:
  ros__parameters:
    time_limit_minutes: 15.0         # 0.0 = no time limit (coverage-only)
    frontier_done_patience: 30.0     # seconds of zero frontiers before declaring done
    map_save_dir: "~/maps"
    auto_save_map: true
    return_home: true
    status_publish_rate: 0.2         # Hz (every 5 seconds)
    slam_mode: "3d"                  # set by launch file, not user — "3d" or "2d"
```

## Shell Scripts

### start_explore.sh

```
Usage:
  ./start_explore.sh                          # fresh 15-min explore
  ./start_explore.sh resume:=true             # continue from existing RTABMap DB
  ./start_explore.sh time_limit:=30           # 30-minute explore
  ./start_explore.sh time_limit:=0            # explore until full coverage

Internally:
  1. Kill any running SLAM / nav / explore stacks (idempotent teardown)
  2. ros2 launch slam_bringup slam.launch.py \
       nav2:=true enable_drive:=true \
       delete_db_on_start:=<inverse of resume>
  3. ros2 launch slam_bringup explore.launch.py \
       slam_mode:=3d time_limit:=<N> resume:=<bool>
```

### start_explore_2d.sh

```
Usage:
  ./start_explore_2d.sh                       # fresh 15-min explore
  ./start_explore_2d.sh resume:=true map_file:=~/maps/explore_latest
  ./start_explore_2d.sh time_limit:=0         # full coverage

Internally:
  1. Kill any running SLAM / nav / explore stacks
  2. ros2 launch slam_bringup slam_2d.launch.py \
       enable_drive:=true mode:=mapping \
       map_file:=<if resume>
  3. ros2 launch slam_bringup explore.launch.py \
       slam_mode:=2d time_limit:=<N>
```

## HTML Visualization — docs/visualizations/frontier_explorer.html

Make or Code branded page matching the existing `slam_config_explorer.html` style (Audiowide/JetBrains Mono/Orbitron, Hot Pink/Electric Blue/Deep Purple palette, slate-900 cards).

### Sections

1. **How Frontier Exploration Works**
   - Animated or static visual: occupancy grid with free (green), occupied (red), unknown (gray), and frontier (highlighted) cells
   - Shows how the robot picks the nearest/largest frontier as its next goal
   - Explains the explore → drive → re-evaluate loop

2. **Exploration Parameters** (interactive cards)
   - `min_frontier_size` — slider, shows effect on which frontiers are candidates
   - `potential_scale` / `gain_scale` — sliders showing the distance-vs-size tradeoff
   - `planner_frequency` — how often frontiers are re-evaluated
   - `progress_timeout` — when to give up on a stuck goal
   - `time_limit_minutes` — session duration
   - `frontier_done_patience` — how long zero-frontiers must persist
   - Each card: param name, config file path, description, EXPLORE mode chip, observation notes

3. **Recovery Behaviors**
   - Visual flowchart: stuck detection → spin → backup → wait → clear costmap → re-plan → next frontier
   - Links to Nav2 recovery params in nav2_params.yaml

4. **Session Lifecycle**
   - State machine diagram: EXPLORING → SAVING → RETURNING → COMPLETE
   - What triggers each transition
   - What the status topic reports in each state

5. **Resume Mode**
   - Explains 3D vs 2D resume behavior
   - Starting position constraint

6. **FAQ**
   - "Does starting position matter?" → No (mapping mode)
   - "What if it gets stuck?" → Nav2 recovery chain, then next frontier
   - "Can it explore multiple rooms?" → Yes, automatically
   - "How do I know when it's done?" → /explore/status topic, frontier count trending to zero
   - "Can I take over with RC mid-explore?" → Yes, explore_lite will resume when you release

## Dependencies

### New Package Required

`m-explore-lite` — install on the Jetson:
```bash
sudo apt install ros-humble-explore-lite
# or if not in apt:
cd ~/ros2_ws/src && git clone https://github.com/robo-friends/m-explore-lite.git
cd ~/ros2_ws && colcon build --packages-select explore_lite
```

### Existing Dependencies (no changes)

- nav2_bringup (already in package.xml)
- nav2_msgs (NavigateToPose action)
- std_msgs (status topic)
- geometry_msgs (PoseStamped for home pose)

## Testing Plan

1. **Bench fixture (no drive):** Launch with `enable_drive:=false` to verify explore_lite publishes frontier markers and goals without moving. Confirm status topic publishes correctly.
2. **Mecanum rover, small room:** Run `start_explore.sh time_limit:=5` in a single room. Verify it maps the room, saves the map, and returns to start.
3. **Resume test:** Run for 5 minutes, stop. Run `start_explore.sh resume:=true time_limit:=5`. Verify it loads the partial map and continues exploring new areas.
4. **Multi-room:** Run with `time_limit:=15` with doors open between rooms. Verify it autonomously transitions between rooms.
5. **Recovery test:** Place a temporary obstacle in the robot's path mid-explore. Verify Nav2 triggers recovery behaviors and the robot routes around it.
