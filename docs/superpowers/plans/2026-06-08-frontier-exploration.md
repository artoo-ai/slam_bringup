# Frontier Exploration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add autonomous frontier-based exploration to the SLAM stack so the robot can map a house without RC driving, then save the map and return home.

**Architecture:** explore_lite (m-explore-lite) reads the occupancy grid and sends NavigateToPose goals to Nav2. A custom explore_manager node orchestrates the session: countdown timer, frontier monitoring, map save via service calls, and return-to-home via NavigateToPose. Two shell scripts (`start_explore.sh`, `start_explore_2d.sh`) launch the existing SLAM+Nav2 stack plus the new exploration layer.

**Tech Stack:** ROS2 Humble, m-explore-lite, Nav2, RTABMap, slam_toolbox, Python 3

---

## File Structure

| File | Responsibility |
|------|---------------|
| `config/explore_params.yaml` | explore_lite + explore_manager parameters |
| `slam_bringup/explore_manager_node.py` | Session lifecycle: timer, frontier monitor, map save, return-home, status publisher |
| `launch/explore.launch.py` | Wraps explore_lite + explore_manager, remaps topics per 3D/2D mode |
| `start_explore.sh` | 3D explore: teardown → slam.launch.py (nav2:=true) → explore.launch.py |
| `start_explore_2d.sh` | 2D explore: teardown → slam_2d.launch.py (nav2:=true) → explore.launch.py |
| `kill_explore.sh` | Kill explore_manager + explore_lite processes |
| `docs/visualizations/frontier_explorer.html` | Interactive parameter reference (Make or Code branded) |
| `setup.py` | Add `explore_manager` entry point |

---

### Task 1: Configuration File

**Files:**
- Create: `config/explore_params.yaml`

- [ ] **Step 1: Create explore_params.yaml**

```yaml
# Autonomous frontier exploration parameters.
#
# explore_lite (m-explore-lite) finds frontiers on the occupancy grid
# and sends NavigateToPose goals to Nav2. explore_manager orchestrates
# the session lifecycle: timer, frontier monitoring, map save, return-home.
#
# Override at launch: explore.launch.py explore_params_file:=/path/to/custom.yaml

explore_lite:
  ros__parameters:
    costmap_topic: /map
    min_frontier_size: 0.75
    potential_scale: 3.0
    gain_scale: 1.0
    transform_tolerance: 0.3
    planner_frequency: 0.33
    progress_timeout: 30.0
    visualize_frontiers: true
    robot_base_frame: base_link

explore_manager:
  ros__parameters:
    time_limit_minutes: 15.0
    frontier_done_patience: 30.0
    map_save_dir: "~/maps"
    auto_save_map: true
    return_home: true
    status_publish_rate: 0.2
    slam_mode: "3d"
```

- [ ] **Step 2: Commit**

```bash
git add config/explore_params.yaml
git commit -m "config: add explore_params.yaml for frontier exploration"
```

---

### Task 2: explore_manager Node

**Files:**
- Create: `slam_bringup/explore_manager_node.py`
- Modify: `setup.py`

This is the core new code. The node manages the exploration lifecycle: records home pose, monitors frontiers, enforces time limit, saves the map, and sends the robot home.

- [ ] **Step 1: Create explore_manager_node.py**

```python
#!/usr/bin/env python3
"""Exploration session manager — timer, frontier monitor, map save, return-home.

Lifecycle:
  1. Record home pose (current robot position in map frame)
  2. Wait for /map to be available
  3. EXPLORING: monitor time + frontier count
  4. SAVING: call RTABMap backup or slam_toolbox serialize_map
  5. RETURNING: NavigateToPose to home pose
  6. COMPLETE: publish final stats, stay alive for visualization
"""

import json
import math
import os
import time
from enum import Enum, auto
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener, TransformException


class State(Enum):
    WAITING = auto()
    EXPLORING = auto()
    SAVING = auto()
    RETURNING = auto()
    COMPLETE = auto()


class ExploreManager(Node):

    def __init__(self):
        super().__init__('explore_manager')

        self.declare_parameter('time_limit_minutes', 15.0)
        self.declare_parameter('frontier_done_patience', 30.0)
        self.declare_parameter('map_save_dir', '~/maps')
        self.declare_parameter('auto_save_map', True)
        self.declare_parameter('return_home', True)
        self.declare_parameter('status_publish_rate', 0.2)
        self.declare_parameter('slam_mode', '3d')

        self._time_limit_s = self.get_parameter('time_limit_minutes').value * 60.0
        self._patience_s = self.get_parameter('frontier_done_patience').value
        self._save_dir = os.path.expanduser(
            self.get_parameter('map_save_dir').value
        )
        self._auto_save = self.get_parameter('auto_save_map').value
        self._return_home = self.get_parameter('return_home').value
        self._status_rate = self.get_parameter('status_publish_rate').value
        self._slam_mode = self.get_parameter('slam_mode').value

        self._state = State.WAITING
        self._start_time = None
        self._home_pose = None
        self._frontier_count = 0
        self._free_cells = 0
        self._zero_frontier_since = None
        self._map_received = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._status_pub = self.create_publisher(String, '/explore/status', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, 10
        )

        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self._check_timer = self.create_timer(5.0, self._check_loop)
        status_period = 1.0 / self._status_rate if self._status_rate > 0 else 5.0
        self._status_timer = self.create_timer(status_period, self._publish_status)

        self.get_logger().info(
            f'explore_manager: slam_mode={self._slam_mode}, '
            f'time_limit={self._time_limit_s:.0f}s, '
            f'patience={self._patience_s:.0f}s'
        )

    def _map_cb(self, msg: OccupancyGrid):
        self._map_received = True
        data = msg.data
        self._free_cells = sum(1 for v in data if v == 0)
        unknown = sum(1 for v in data if v == -1)
        occupied = sum(1 for v in data if v > 0)
        free = self._free_cells

        frontier = 0
        w = msg.info.width
        h = msg.info.height
        for y in range(h):
            for x in range(w):
                idx = y * w + x
                if data[idx] != 0:
                    continue
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        if data[ny * w + nx] == -1:
                            frontier += 1
                            break
        self._frontier_count = frontier

    def _record_home_pose(self):
        odom_frame = 'camera_init' if self._slam_mode == '3d' else 'odom'
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = t.transform.translation.x
            pose.pose.position.y = t.transform.translation.y
            pose.pose.position.z = 0.0
            pose.pose.orientation = t.transform.rotation
            self._home_pose = pose
            self.get_logger().info(
                f'Home pose recorded: ({pose.pose.position.x:.2f}, '
                f'{pose.pose.position.y:.2f})'
            )
            return True
        except TransformException as e:
            self.get_logger().warn(f'Cannot record home pose yet: {e}')
            return False

    def _check_loop(self):
        if self._state == State.WAITING:
            if not self._map_received:
                return
            if self._record_home_pose():
                self._state = State.EXPLORING
                self._start_time = time.monotonic()
                self.get_logger().info('Exploration started!')
            return

        if self._state == State.EXPLORING:
            elapsed = time.monotonic() - self._start_time

            if self._time_limit_s > 0 and elapsed >= self._time_limit_s:
                self.get_logger().info('Time limit reached — finishing exploration.')
                self._transition_to_saving()
                return

            if self._frontier_count == 0:
                if self._zero_frontier_since is None:
                    self._zero_frontier_since = time.monotonic()
                elif time.monotonic() - self._zero_frontier_since >= self._patience_s:
                    self.get_logger().info(
                        f'No frontiers for {self._patience_s:.0f}s — exploration complete.'
                    )
                    self._transition_to_saving()
                    return
            else:
                self._zero_frontier_since = None

        if self._state == State.COMPLETE:
            pass

    def _transition_to_saving(self):
        self._state = State.SAVING
        if self._auto_save:
            self._save_map()
        if self._return_home and self._home_pose is not None:
            self._state = State.RETURNING
            self._navigate_home()
        else:
            self._state = State.COMPLETE
            self._log_final_stats()

    def _save_map(self):
        os.makedirs(self._save_dir, exist_ok=True)
        timestamp = time.strftime('%Y%m%d_%H%M%S')

        if self._slam_mode == '3d':
            self._save_rtabmap(timestamp)
        else:
            self._save_slam_toolbox(timestamp)

    def _save_rtabmap(self, timestamp):
        cli = self.create_client(Empty, '/rtabmap/backup')
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('RTABMap /rtabmap/backup service not available.')
            return
        future = cli.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        src = os.path.expanduser('~/.ros/rtabmap.db')
        dst = os.path.join(self._save_dir, f'explore_{timestamp}.db')
        if os.path.exists(src):
            import shutil
            shutil.copy2(src, dst)
            self.get_logger().info(f'RTABMap DB saved to {dst}')
        else:
            self.get_logger().warn(f'RTABMap DB not found at {src}')

    def _save_slam_toolbox(self, timestamp):
        from slam_toolbox.srv import SerializePoseGraph
        cli = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('slam_toolbox serialize_map service not available.')
            return
        req = SerializePoseGraph.Request()
        req.filename = os.path.join(self._save_dir, f'explore_{timestamp}')
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        self.get_logger().info(f'slam_toolbox map saved to {req.filename}')

    def _navigate_home(self, retry=True):
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose action server not available.')
            self._state = State.COMPLETE
            self._log_final_stats()
            return

        goal = NavigateToPose.Goal()
        goal.pose = self._home_pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info('Navigating home...')

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f: self._on_home_goal_response(f, retry)
        )

    def _on_home_goal_response(self, future, retry):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Return-home goal rejected.')
            if retry:
                self.get_logger().info('Retrying return-home...')
                self._navigate_home(retry=False)
            else:
                self._stop_robot()
                self._state = State.COMPLETE
                self._log_final_stats()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_home_result(f, retry)
        )

    def _on_home_result(self, future, retry):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Returned home successfully!')
            self._state = State.COMPLETE
            self._log_final_stats()
        else:
            self.get_logger().warn(f'Return-home failed (status={result.status}).')
            if retry:
                self.get_logger().info('Retrying return-home...')
                self._navigate_home(retry=False)
            else:
                self.get_logger().warn('Return-home failed after retry. Stopping in place.')
                self._stop_robot()
                self._state = State.COMPLETE
                self._log_final_stats()

    def _stop_robot(self):
        self._cmd_vel_pub.publish(Twist())

    def _log_final_stats(self):
        elapsed = time.monotonic() - self._start_time if self._start_time else 0
        self.get_logger().info(
            f'Exploration complete. '
            f'Duration: {elapsed:.0f}s, '
            f'Free cells: {self._free_cells}, '
            f'Remaining frontiers: {self._frontier_count}'
        )

    def _publish_status(self):
        elapsed = time.monotonic() - self._start_time if self._start_time else 0
        remaining = max(0, self._time_limit_s - elapsed) if self._time_limit_s > 0 else -1

        status = {
            'state': self._state.name,
            'time_elapsed_s': round(elapsed, 1),
            'time_remaining_s': round(remaining, 1) if remaining >= 0 else None,
            'frontier_count': self._frontier_count,
            'free_cells_mapped': self._free_cells,
        }
        if self._home_pose:
            status['home_pose'] = {
                'x': round(self._home_pose.pose.position.x, 3),
                'y': round(self._home_pose.pose.position.y, 3),
            }

        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ExploreManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down — stopping robot.')
        node._stop_robot()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
```

- [ ] **Step 2: Add entry point to setup.py**

In `setup.py`, add the following line to the `console_scripts` list, after the `yahboom_bridge` entry:

```python
            'explore_manager = slam_bringup.explore_manager_node:main',
```

- [ ] **Step 3: Verify the module imports cleanly**

Run (on the Jetson or dev machine with ROS2 sourced):
```bash
cd ~/slam_ws && colcon build --packages-select slam_bringup --symlink-install 2>&1 | tail -5
```
Expected: `Summary: 1 package finished`

- [ ] **Step 4: Commit**

```bash
git add slam_bringup/explore_manager_node.py setup.py
git commit -m "feat: add explore_manager node — exploration session lifecycle"
```

---

### Task 3: Launch File

**Files:**
- Create: `launch/explore.launch.py`

This launch file wraps explore_lite and explore_manager, remapping topics and frames for the 3D vs 2D SLAM modes.

- [ ] **Step 1: Create explore.launch.py**

```python
"""Autonomous frontier exploration layer.

Sits on top of the SLAM + Nav2 stack (either 3D or 2D) and drives the
robot toward unexplored frontiers until a time limit or full coverage,
then saves the map and returns home.

Launched by start_explore.sh / start_explore_2d.sh — not typically
called directly.

Nodes:
  1. explore_lite (m-explore-lite) — frontier detection + goal publishing
  2. explore_manager (slam_bringup) — session lifecycle orchestration
"""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('slam_bringup'))
    default_params = str(pkg_share / 'config' / 'explore_params.yaml')

    params_arg = DeclareLaunchArgument(
        'explore_params_file', default_value=default_params,
        description='YAML with explore_lite + explore_manager parameters.',
    )
    slam_mode_arg = DeclareLaunchArgument(
        'slam_mode', default_value='3d',
        description='Which SLAM backend is running: "3d" (RTABMap) or "2d" (slam_toolbox).',
    )
    time_limit_arg = DeclareLaunchArgument(
        'time_limit', default_value='15.0',
        description='Exploration time limit in minutes. 0 = no limit (coverage-only).',
    )
    return_home_arg = DeclareLaunchArgument(
        'return_home', default_value='true',
        description='Navigate back to starting position when exploration ends.',
    )

    explore_lite_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_lite',
        output='screen',
        parameters=[
            LaunchConfiguration('explore_params_file'),
        ],
    )

    explore_manager_node = Node(
        package='slam_bringup',
        executable='explore_manager',
        name='explore_manager',
        output='screen',
        parameters=[
            LaunchConfiguration('explore_params_file'),
            {
                'slam_mode': LaunchConfiguration('slam_mode'),
                'time_limit_minutes': LaunchConfiguration('time_limit'),
                'return_home': LaunchConfiguration('return_home'),
            },
        ],
    )

    return LaunchDescription([
        params_arg,
        slam_mode_arg,
        time_limit_arg,
        return_home_arg,
        explore_lite_node,
        explore_manager_node,
    ])
```

- [ ] **Step 2: Rebuild to pick up the new launch file**

```bash
cd ~/slam_ws && colcon build --packages-select slam_bringup --symlink-install 2>&1 | tail -5
```
Expected: `Summary: 1 package finished`

- [ ] **Step 3: Commit**

```bash
git add launch/explore.launch.py
git commit -m "launch: add explore.launch.py — explore_lite + explore_manager"
```

---

### Task 4: Shell Scripts

**Files:**
- Create: `start_explore.sh`
- Create: `start_explore_2d.sh`
- Create: `kill_explore.sh`

- [ ] **Step 1: Create kill_explore.sh**

```bash
#!/usr/bin/env bash
# Kill explore_lite + explore_manager processes.

pkill -SIGINT -f explore_manager 2>/dev/null
pkill -SIGINT -f 'explore_lite'  2>/dev/null
sleep 1
pkill -9 -f explore_manager      2>/dev/null
pkill -9 -f 'explore_lite'       2>/dev/null
pkill -9 -f "ros2 launch slam_bringup explore" 2>/dev/null
```

- [ ] **Step 2: Create start_explore.sh**

```bash
#!/usr/bin/env bash
# Start autonomous frontier exploration with the 3D SLAM stack.
# Idempotent: kills all running stacks before launching.
#
# Usage:
#   ./start_explore.sh                          # fresh 15-min explore
#   ./start_explore.sh resume:=true             # continue from existing RTABMap DB
#   ./start_explore.sh time_limit:=30           # 30-minute explore
#   ./start_explore.sh time_limit:=0            # explore until full coverage
#
# The robot will:
#   1. Start SLAM + Nav2
#   2. Autonomously explore by driving toward map frontiers
#   3. Save the map when done (or time runs out)
#   4. Return to its starting position

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash
# shellcheck source=start_helpers.sh
source "$SCRIPT_DIR/start_helpers.sh"
ensure_foxglove

# ── Parse args ──────────────────────────────────────────────
RESUME=false
TIME_LIMIT=15.0
SLAM_ARGS=()
EXPLORE_ARGS=()

for arg in "$@"; do
  case "$arg" in
    resume:=true)   RESUME=true ;;
    resume:=false)  RESUME=false ;;
    time_limit:=*)  TIME_LIMIT="${arg#time_limit:=}"
                    EXPLORE_ARGS+=("$arg") ;;
    *)              SLAM_ARGS+=("$arg") ;;
  esac
done

if [ "$RESUME" = "true" ]; then
  DELETE_DB=false
else
  DELETE_DB=true
fi

# ── Teardown ────────────────────────────────────────────────
"$SCRIPT_DIR/kill_explore.sh"  2>/dev/null
"$SCRIPT_DIR/kill_nav.sh"      2>/dev/null
"$SCRIPT_DIR/kill_rtabmap.sh"  2>/dev/null
"$SCRIPT_DIR/kill_fast_lio.sh" 2>/dev/null
"$SCRIPT_DIR/kill_viz_clip.sh" 2>/dev/null
"$SCRIPT_DIR/kill_sensors.sh"  2>/dev/null
pkill -SIGINT -f robot_state_publisher 2>/dev/null
sleep 1
pkill -9 -f robot_state_publisher              2>/dev/null
pkill -9 -f "static_transform_publisher.*body" 2>/dev/null
pkill -9 -f "ros2 launch slam_bringup slam"    2>/dev/null

# ── Launch SLAM + Nav2 in background ───────────────────────
echo "==> Starting 3D SLAM + Nav2 (delete_db=$DELETE_DB, resume=$RESUME)..."
ros2 launch slam_bringup slam.launch.py \
    nav2:=true \
    enable_drive:=true \
    delete_db_on_start:="$DELETE_DB" \
    "${SLAM_ARGS[@]}" &
SLAM_PID=$!

# Give SLAM + Nav2 time to initialize before starting exploration.
echo "==> Waiting 15s for SLAM + Nav2 to initialize..."
sleep 15

# ── Launch exploration layer ───────────────────────────────
echo "==> Starting frontier exploration (time_limit=${TIME_LIMIT} min)..."
ros2 launch slam_bringup explore.launch.py \
    slam_mode:=3d \
    time_limit:="$TIME_LIMIT" \
    "${EXPLORE_ARGS[@]}" &
EXPLORE_PID=$!

# ── Wait for either to exit ───────────────────────────────
cleanup() {
    echo ""
    echo "==> Shutting down exploration..."
    kill -SIGINT "$EXPLORE_PID" 2>/dev/null
    sleep 2
    kill -SIGINT "$SLAM_PID" 2>/dev/null
    wait
}
trap cleanup SIGINT SIGTERM

wait -n "$SLAM_PID" "$EXPLORE_PID"
cleanup
```

- [ ] **Step 3: Create start_explore_2d.sh**

```bash
#!/usr/bin/env bash
# Start autonomous frontier exploration with the 2D SLAM stack.
# Idempotent: kills all running stacks before launching.
#
# Usage:
#   ./start_explore_2d.sh                                            # fresh 15-min explore
#   ./start_explore_2d.sh resume:=true map_file:=~/maps/explore_latest  # continue
#   ./start_explore_2d.sh time_limit:=30                             # 30-minute explore
#   ./start_explore_2d.sh time_limit:=0                              # full coverage
#
# The robot will:
#   1. Start 2D SLAM + Nav2
#   2. Autonomously explore by driving toward map frontiers
#   3. Save the serialized map when done (or time runs out)
#   4. Return to its starting position

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash
# shellcheck source=start_helpers.sh
source "$SCRIPT_DIR/start_helpers.sh"
ensure_foxglove

# ── Parse args ──────────────────────────────────────────────
RESUME=false
TIME_LIMIT=15.0
SLAM_ARGS=()
EXPLORE_ARGS=()

for arg in "$@"; do
  case "$arg" in
    resume:=true)   RESUME=true ;;
    resume:=false)  RESUME=false ;;
    time_limit:=*)  TIME_LIMIT="${arg#time_limit:=}"
                    EXPLORE_ARGS+=("$arg") ;;
    *)              SLAM_ARGS+=("$arg") ;;
  esac
done

if [ "$RESUME" = "true" ]; then
  SLAM_MODE_ARG="mode:=mapping"
else
  SLAM_MODE_ARG="mode:=mapping"
fi

# ── Teardown ────────────────────────────────────────────────
"$SCRIPT_DIR/kill_explore.sh"  2>/dev/null
"$SCRIPT_DIR/kill_nav.sh"      2>/dev/null
"$SCRIPT_DIR/kill_nav_2d.sh"   2>/dev/null
"$SCRIPT_DIR/kill_rtabmap.sh"  2>/dev/null
"$SCRIPT_DIR/kill_fast_lio.sh" 2>/dev/null
"$SCRIPT_DIR/kill_viz_clip.sh" 2>/dev/null
"$SCRIPT_DIR/kill_sensors.sh"  2>/dev/null
pkill -SIGINT -f pointcloud_to_laserscan_node 2>/dev/null
pkill -SIGINT -f rf2o_laser_odometry_node     2>/dev/null
pkill -SIGINT -f async_slam_toolbox_node      2>/dev/null
pkill -SIGINT -f robot_state_publisher        2>/dev/null
sleep 1
pkill -9 -f pointcloud_to_laserscan_node 2>/dev/null
pkill -9 -f rf2o_laser_odometry_node     2>/dev/null
pkill -9 -f async_slam_toolbox_node      2>/dev/null
pkill -9 -f robot_state_publisher        2>/dev/null
pkill -9 -f "ros2 launch slam_bringup slam_2d" 2>/dev/null
pkill -9 -f "ros2 launch slam_bringup slam"    2>/dev/null

# ── Launch 2D SLAM + Nav2 in background ────────────────────
echo "==> Starting 2D SLAM + Nav2 (resume=$RESUME)..."
ros2 launch slam_bringup slam_2d.launch.py \
    "$SLAM_MODE_ARG" \
    enable_drive:=true \
    nav2:=true \
    "${SLAM_ARGS[@]}" &
SLAM_PID=$!

echo "==> Waiting 10s for 2D SLAM + Nav2 to initialize..."
sleep 10

# ── Launch exploration layer ───────────────────────────────
echo "==> Starting frontier exploration (time_limit=${TIME_LIMIT} min)..."
ros2 launch slam_bringup explore.launch.py \
    slam_mode:=2d \
    time_limit:="$TIME_LIMIT" \
    "${EXPLORE_ARGS[@]}" &
EXPLORE_PID=$!

# ── Wait for either to exit ───────────────────────────────
cleanup() {
    echo ""
    echo "==> Shutting down exploration..."
    kill -SIGINT "$EXPLORE_PID" 2>/dev/null
    sleep 2
    kill -SIGINT "$SLAM_PID" 2>/dev/null
    wait
}
trap cleanup SIGINT SIGTERM

wait -n "$SLAM_PID" "$EXPLORE_PID"
cleanup
```

- [ ] **Step 4: Make all scripts executable**

```bash
chmod +x start_explore.sh start_explore_2d.sh kill_explore.sh
```

- [ ] **Step 5: Commit**

```bash
git add start_explore.sh start_explore_2d.sh kill_explore.sh
git commit -m "feat: add start_explore.sh + start_explore_2d.sh — autonomous exploration scripts"
```

---

### Task 5: HTML Visualization Page

**Files:**
- Create: `docs/visualizations/frontier_explorer.html`

Make or Code branded interactive parameter reference. Matches the existing `slam_config_explorer.html` style: Audiowide/JetBrains Mono/Orbitron fonts, Hot Pink (#ff3d7f) / Electric Blue (#00b8ff) / Deep Purple (#7a2dff) palette, slate-900 card backgrounds.

The page has these sections:
1. **How Frontier Exploration Works** — visual grid diagram showing free/occupied/unknown/frontier cells, robot picking next target
2. **Exploration Parameters** — interactive cards for each explore_lite + explore_manager param with EXPLORE mode chips
3. **Recovery Behaviors** — visual flowchart: stuck → spin → backup → wait → clear costmap → re-plan
4. **Session Lifecycle** — state machine: EXPLORING → SAVING → RETURNING → COMPLETE with transition triggers
5. **Resume Mode** — 3D vs 2D resume behavior, starting position constraint
6. **FAQ** — starting position, stuck handling, multi-room, completion detection, RC takeover

- [ ] **Step 1: Create the HTML file**

Create `docs/visualizations/frontier_explorer.html` with the full content. This is a large single-file HTML page (~800-1200 lines) following the exact same CSS foundation as `slam_config_explorer.html`:

Key CSS variables to reuse from the existing pages:
```css
:root {
    --brand-pink:   #ff3d7f;
    --brand-blue:   #00b8ff;
    --brand-purple: #7a2dff;
    --brand-bg:     #0a0a14;
    --brand-bg-2:   #140820;
    --font-display:     'Audiowide', 'Orbitron', sans-serif;
    --font-display-alt: 'Orbitron', 'Audiowide', sans-serif;
    --font-ui:          'JetBrains Mono', 'Fira Mono', monospace;
}
```

Card structure per parameter (matches existing pattern):
```html
<div class="card">
  <div class="card-head">
    <span class="paramName">min_frontier_size</span>
    <span class="paramFile">config/explore_params.yaml → explore_lite</span>
    <p class="paramDesc">Minimum frontier cluster size in meters. Clusters smaller than this are ignored — filters out tiny crevice frontiers and sensor noise.</p>
  </div>
  <div class="modes">
    <span class="mode-chip mode-explore">EXPLORE</span>
  </div>
  <!-- slider or static value display -->
  <div class="ex-observe">
    <b>Observation:</b> Start at 0.75 m. If the robot wastes time investigating tiny gaps between furniture, raise to 1.0–1.5. If it misses doorways, lower to 0.5.
  </div>
</div>
```

Add an `.mode-explore` chip style:
```css
.mode-explore { background: rgba(52, 211, 153, 0.18); color: #6ee7b7; border: 1px solid rgba(52, 211, 153, 0.4); }
```

Sections to include:

**How It Works section:** Use a CSS grid or canvas to draw a simplified occupancy grid showing:
- Gray cells = unknown (-1)
- White/green cells = free (0)
- Dark cells = occupied (walls)
- Highlighted border cells = frontiers (pink glow)
- Robot icon at current position
- Arrow pointing to selected frontier goal

**Recovery flowchart:** Use CSS flexbox with arrows (→) between recovery stage boxes:
```
[Stuck detected] → [Spin] → [Backup] → [Wait] → [Clear costmap] → [Re-plan] → [Next frontier]
```

**Lifecycle state machine:** Similar visual with state boxes and labeled transitions:
```
[EXPLORING] ──time limit or no frontiers──► [SAVING] ──map saved──► [RETURNING] ──arrived──► [COMPLETE]
```

Build the complete page with all 6 sections, all parameter cards, and the visual diagrams inline. The page must be fully self-contained (no external JS dependencies except Google Fonts).

- [ ] **Step 2: Open in browser and verify**

```bash
open docs/visualizations/frontier_explorer.html
```

Verify: page renders correctly, brand styling matches other pages, all sections present, parameter cards readable.

- [ ] **Step 3: Commit**

```bash
git add docs/visualizations/frontier_explorer.html
git commit -m "docs/viz: frontier explorer — interactive param reference for autonomous exploration"
```

---

### Task 6: Package Dependency Update

**Files:**
- Modify: `package.xml`

- [ ] **Step 1: Add explore_lite as an exec dependency**

Add the following line to `package.xml` in the `<exec_depend>` section (alongside the other dependencies):

```xml
  <exec_depend>explore_lite</exec_depend>
```

- [ ] **Step 2: Rebuild**

```bash
cd ~/slam_ws && colcon build --packages-select slam_bringup --symlink-install 2>&1 | tail -5
```

- [ ] **Step 3: Commit**

```bash
git add package.xml
git commit -m "deps: add explore_lite (m-explore-lite) to package.xml"
```

---

### Task 7: Integration Smoke Test

No new files — verify the full stack wires up correctly.

- [ ] **Step 1: Verify explore_manager node runs standalone**

```bash
source ~/slam_ws/install/setup.bash
ros2 run slam_bringup explore_manager --ros-args -p slam_mode:=3d -p time_limit_minutes:=1.0
```

Expected: Node starts, logs parameter values, prints "Cannot record home pose yet" (no TF available without SLAM running). Ctrl-C exits cleanly.

- [ ] **Step 2: Verify explore.launch.py parses**

```bash
ros2 launch slam_bringup explore.launch.py --show-args
```

Expected: Shows `explore_params_file`, `slam_mode`, `time_limit`, `return_home` arguments.

- [ ] **Step 3: Verify start_explore.sh --help-style output**

```bash
head -12 start_explore.sh
head -14 start_explore_2d.sh
```

Expected: Usage comments visible, scripts are executable.

- [ ] **Step 4: Full stack test on the rover (when hardware available)**

On the Jetson with the mecanum rover:

```bash
./start_explore.sh time_limit:=5
```

Watch Foxglove for:
- `/explore/status` topic publishing JSON with state, frontier count, time remaining
- Frontier markers visible (if `visualize_frontiers: true`)
- Robot driving toward frontiers autonomously
- After 5 minutes: map saves, robot returns home

- [ ] **Step 5: Test resume mode**

```bash
./start_explore.sh resume:=true time_limit:=5
```

Verify: RTABMap loads existing DB, explore_lite finds remaining frontiers, continues mapping new areas.
