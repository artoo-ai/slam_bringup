from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    realsense_share = get_package_share_directory('realsense2_camera')
    rs_launch = f'{realsense_share}/launch/rs_launch.py'

    slam_mode_arg = DeclareLaunchArgument(
        'slam_mode',
        default_value='false',
        description='true = align_depth on, own pointcloud off (for RTABMap); false = raw mode',
    )
    enable_front_arg = DeclareLaunchArgument(
        'enable_front',
        default_value='true',
        description='Launch the D435 front camera',
    )
    enable_rear_arg = DeclareLaunchArgument(
        'enable_rear',
        default_value='false',
        description='Launch the D435i rear camera (default off — enable when ready to test dual-camera)',
    )
    device_type_front_arg = DeclareLaunchArgument(
        'device_type_front',
        default_value='d435',
        description='Front camera device_type (d435, d455 for outdoor swap)',
    )
    device_type_rear_arg = DeclareLaunchArgument(
        'device_type_rear',
        default_value='d435i',
        description='Rear camera device_type',
    )

    slam = LaunchConfiguration('slam_mode')

    # In SLAM mode: align_depth ON, RealSense pointcloud OFF (RTABMap builds its own)
    # In raw mode: align_depth OFF, RealSense pointcloud ON (for direct RViz use)
    pc_enable = PythonExpression(["'false' if '", slam, "' == 'true' else 'true'"])

    # Front camera — D435 (no IMU in sensor)
    front = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_front')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rs_launch),
                launch_arguments={
                    'camera_namespace':     'd435_front',
                    'camera_name':          'camera',
                    'device_type':          LaunchConfiguration('device_type_front'),
                    'enable_depth':         'true',
                    'enable_color':         'true',
                    'enable_sync':          'true',
                    'align_depth.enable':   slam,
                    'enable_gyro':          'false',
                    'enable_accel':         'false',
                    'pointcloud.enable':        pc_enable,
                    'depth_module.depth_profile': '848x480x30',
                    'rgb_camera.color_profile':   '848x480x30',
                }.items(),
            ),
        ],
    )

    # Rear camera — D435i (has built-in BMI055 IMU — disable, WitMotion is authoritative)
    rear = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_rear')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rs_launch),
                launch_arguments={
                    'camera_namespace':     'd435_rear',
                    'camera_name':          'camera',
                    'device_type':          LaunchConfiguration('device_type_rear'),
                    'enable_depth':         'true',
                    'enable_color':         'true',
                    'enable_sync':          'true',
                    'align_depth.enable':   slam,
                    'enable_gyro':          'false',     # D435i BMI055 disabled — WitMotion wins
                    'enable_accel':         'false',
                    'pointcloud.enable':        pc_enable,
                    'depth_module.depth_profile': '848x480x30',
                    'rgb_camera.color_profile':   '848x480x30',
                }.items(),
            ),
        ],
    )

    return LaunchDescription([
        slam_mode_arg,
        enable_front_arg,
        enable_rear_arg,
        device_type_front_arg,
        device_type_rear_arg,
        front,
        rear,
    ])
