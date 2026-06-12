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
    resume_map_file_arg = DeclareLaunchArgument(
        'resume_map_file', default_value='',
        description='Serialized graph basename (no extension) this session '
                    'resumed from. Enables explore_manager pose-file tracking '
                    '(<basename>.pose) from launch instead of waiting for the '
                    'first serialize. Set by start_explore_2d.sh on resume:=true.',
    )

    # respawn=True is load-bearing: explore_lite's frontier BLACKLIST is
    # permanent for the node's lifetime and /explore/resume does NOT clear
    # it. explore_manager's stall watchdog escalates from a resume kick to
    # SIGINT-ing this process — respawn brings it back blacklist-free and
    # it re-evaluates the (now larger) map from scratch.
    explore_lite_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_lite',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
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
                'resume_map_file': LaunchConfiguration('resume_map_file'),
            },
        ],
    )

    return LaunchDescription([
        params_arg,
        slam_mode_arg,
        time_limit_arg,
        return_home_arg,
        resume_map_file_arg,
        explore_lite_node,
        explore_manager_node,
    ])
