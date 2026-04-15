from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = Path(get_package_share_directory('slam_bringup'))
    default_config = bringup_share / 'config' / 'mid360.json'

    config_arg = DeclareLaunchArgument(
        'user_config_path',
        default_value=str(default_config),
        description='Absolute path to MID360_config.json (lives in slam_bringup/config/)',
    )
    frame_id_arg = DeclareLaunchArgument('frame_id', default_value='livox_frame')
    xfer_format_arg = DeclareLaunchArgument(
        'xfer_format',
        default_value='0',
        description='0 = PointCloud2 (RViz viewable), 1 = CustomMsg (FAST-LIO2)',
    )

    livox_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format':      LaunchConfiguration('xfer_format'),
            'multi_topic':      0,
            'data_src':         0,
            'publish_freq':     10.0,
            'output_data_type': 0,
            'frame_id':         LaunchConfiguration('frame_id'),
            'user_config_path': LaunchConfiguration('user_config_path'),
        }],
    )

    return LaunchDescription([config_arg, frame_id_arg, xfer_format_arg, livox_node])
