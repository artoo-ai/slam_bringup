from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = Path(get_package_share_directory('slam_bringup'))
    default_params = bringup_share / 'config' / 'witmotion.yaml'

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=str(default_params),
        description='Absolute path to witmotion.yaml',
    )

    # We launch our own slam_bringup wt901c_imu Python node — not the
    # ElettraSar witmotion_ros_node — because the WT901C uses the 0x61
    # combined-packet protocol that ElettraSar's witmotion-uart-qt does
    # not register. See slam_bringup/wt901c_imu_node.py for the parser
    # and the multi-line rationale at the top of that file.
    witmotion_node = Node(
        package='slam_bringup',
        executable='wt901c_imu',
        name='wt901c_imu',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([params_arg, witmotion_node])
