"""Bring up Mid-360 + D435 + WitMotion in one shot.

Composition over the three per-sensor launches rather than re-declaring
their nodes here. That keeps the per-sensor launches as the single source
of truth — they still own their own param defaults, frame_ids, and the
ugly direct-Node() workarounds (rs_launch.py drops, NEON pointcloud filter
naming, witmotion 0x61 protocol). This file only forwards the launch args
that an integrated bringup actually needs to flip:

  - slam_mode             → d435 align_depth on / pointcloud off (RTABMap mode)
  - lidar_xfer_format     → mid360 0=PointCloud2 (rviz/foxglove) | 1=CustomMsg (FAST-LIO2)
  - enable_rear           → second D435i (Phase 1.10 dual-cam)
  - enable_{mid360,d435,witmotion} → drop any one driver for debugging

We pass launch_arguments={...} explicitly to each include rather than
relying on LaunchConfiguration auto-propagation, so the wiring is grep-able
when something downstream goes silently missing.
"""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_dir = Path(get_package_share_directory('slam_bringup')) / 'launch'

    args = [
        DeclareLaunchArgument(
            'slam_mode', default_value='false',
            description='Forwarded to d435.launch.py — true = align_depth on, RealSense pointcloud off',
        ),
        DeclareLaunchArgument(
            'lidar_xfer_format', default_value='0',
            description='Forwarded to mid360.launch.py — 0 = PointCloud2 (rviz), 1 = CustomMsg (FAST-LIO2)',
        ),
        DeclareLaunchArgument(
            'enable_rear', default_value='false',
            description='Forwarded to d435.launch.py — bring up the D435i rear camera',
        ),
        DeclareLaunchArgument(
            'enable_mid360', default_value='true',
            description='Set false to skip the Mid-360 driver (for isolated D435/WitMotion debugging)',
        ),
        DeclareLaunchArgument(
            'enable_d435', default_value='true',
            description='Set false to skip the D435 driver',
        ),
        DeclareLaunchArgument(
            'enable_witmotion', default_value='true',
            description='Set false to skip the WitMotion driver',
        ),
    ]

    mid360 = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_mid360')),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / 'mid360.launch.py')),
            launch_arguments={'xfer_format': LaunchConfiguration('lidar_xfer_format')}.items(),
        )],
    )

    d435 = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_d435')),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / 'd435.launch.py')),
            launch_arguments={
                'slam_mode':   LaunchConfiguration('slam_mode'),
                'enable_rear': LaunchConfiguration('enable_rear'),
            }.items(),
        )],
    )

    witmotion = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_witmotion')),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / 'witmotion.launch.py')),
        )],
    )

    return LaunchDescription([*args, mid360, d435, witmotion])
