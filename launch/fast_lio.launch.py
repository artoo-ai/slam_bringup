"""Wrap upstream fast_lio mapping.launch.py with our config + headless defaults.

Why a wrapper instead of pointing users at the upstream launch directly:

1. The upstream launch takes TWO args — `config_path` (a directory) and
   `config_file` (a filename) — and joins them via PathJoinSubstitution.
   It does NOT take a single full path. Documenting this gotcha in a
   wrapper is more reliable than expecting every script that touches
   FAST-LIO2 to remember it.
2. Upstream defaults `rviz=true`. On the Jetson we view via Foxglove and
   don't want rviz2 eating CPU during sensor stress tests, so default
   to false here. Override with `rviz:=true` if you actually want it.
3. Keeps FAST-LIO2 launches consistent with the rest of slam_bringup
   (same config dir layout, same launch arg style).

Topics published by fastlio_mapping:
  /Odometry              nav_msgs/Odometry      ~10 Hz, IMU-tightly-coupled pose
  /path                  nav_msgs/Path          accumulated trajectory
  /cloud_registered      sensor_msgs/PointCloud2  world-frame cloud (rendered map)
  /cloud_registered_body sensor_msgs/PointCloud2  body-frame cloud (RTABMap input)
  /cloud_effected        sensor_msgs/PointCloud2  points that contributed to the update
  /Laser_map             sensor_msgs/PointCloud2  current local map

TF: publishes camera_init → body. The Mid-360 driver publishes the
LiDAR head as livox_frame; bridging body ↔ livox_frame is Phase 7.3
(per-platform static TF) — until then the body and livox_frame point
clouds appear at small offsets in 3D viewers.
"""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = Path(get_package_share_directory('slam_bringup'))
    fast_lio_share = Path(get_package_share_directory('fast_lio'))

    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=str(bringup_share / 'config'),
        description='Directory holding the FAST-LIO2 yaml',
    )
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='fast_lio_mid360.yaml',
        description='Filename of the FAST-LIO2 yaml within config_path',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Spawn rviz2 alongside fastlio_mapping (we view via Foxglove by default)',
    )

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(fast_lio_share / 'launch' / 'mapping.launch.py')
        ),
        launch_arguments={
            'config_path': LaunchConfiguration('config_path'),
            'config_file': LaunchConfiguration('config_file'),
            'rviz':        LaunchConfiguration('rviz'),
        }.items(),
    )

    return LaunchDescription([
        config_path_arg, config_file_arg, rviz_arg, fast_lio_launch,
    ])
