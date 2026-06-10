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
import os
import time
from enum import Enum, auto

import numpy as np

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
        w = msg.info.width
        h = msg.info.height
        grid = np.array(msg.data, dtype=np.int8).reshape((h, w))

        free = grid == 0
        self._free_cells = int(np.count_nonzero(free))

        unknown = grid == -1
        has_unknown_neighbor = np.zeros_like(free)
        has_unknown_neighbor[1:, :]  |= unknown[:-1, :]
        has_unknown_neighbor[:-1, :] |= unknown[1:, :]
        has_unknown_neighbor[:, 1:]  |= unknown[:, :-1]
        has_unknown_neighbor[:, :-1] |= unknown[:, 1:]

        self._frontier_count = int(np.count_nonzero(free & has_unknown_neighbor))

    def _record_home_pose(self):
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
        self._update_latest_symlinks(req.filename)

    def _update_latest_symlinks(self, basename):
        """Point explore_latest.{data,posegraph} at the newest serialized
        pair so `start_explore_2d.sh resume:=true` works without naming a
        specific timestamped file."""
        for ext in ('.data', '.posegraph'):
            target = basename + ext
            if not os.path.exists(target):
                self.get_logger().warn(
                    f'{target} missing after serialize — skipping explore_latest symlink')
                continue
            link = os.path.join(self._save_dir, f'explore_latest{ext}')
            try:
                if os.path.islink(link) or os.path.exists(link):
                    os.remove(link)
                os.symlink(target, link)
            except OSError as exc:
                self.get_logger().warn(f'could not update {link}: {exc}')

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
