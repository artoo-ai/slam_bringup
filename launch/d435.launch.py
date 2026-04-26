from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
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

    # In SLAM mode: align_depth ON, RealSense pointcloud OFF (RTABMap builds its own).
    # In raw mode: align_depth OFF, RealSense pointcloud ON (for direct Foxglove/RViz use).
    pc_enable = PythonExpression(["'false' if '", slam, "' == 'true' else 'true'"])

    # Invoke realsense2_camera_node directly instead of including rs_launch.py.
    # rs_launch.py maintains a hard-coded list of "configurable parameters" and
    # silently drops any launch_argument that is not in that list. In particular
    # it drops pointcloud__neon_.enable — the ARM NEON-accelerated pointcloud
    # filter name that realsense2_camera v4.57.7 actually declares on Jetson —
    # so passing it through launch_arguments={...} is a no-op. Direct Node()
    # also removes all the "Parameter '<our custom launch arg>' is not
    # supported" spam rs_launch.py prints on every startup.
    def realsense_node(namespace, device_type_arg):
        return Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace=namespace,
            name='camera',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_type':                LaunchConfiguration(device_type_arg),
                # camera_name controls the prefix on every TF frame the
                # node publishes (e.g. d435_front_link, d435_front_color_frame).
                # Must match the link name in urdf/sensors_common.urdf.xacro
                # so the URDF's static-TF stops at d435_front_link and the
                # camera's own internal TF tree continues from there.
                'camera_name':                namespace,
                # base_frame_id is the camera root frame name, but the
                # realsense node ALWAYS prepends camera_name to it (so
                # the published frame is f"{camera_name}_{base_frame_id}").
                # Set to "link" → published as "d435_front_link" → matches
                # the URDF's sensor_plate → d435_front_link joint.
                # If you set it to "d435_front_link" you'll get the doubled
                # prefix "d435_front_d435_front_link" and the URDF tree
                # won't connect to the camera optical frames (verified
                # 2026-04-26 with realsense2_camera v4.57.7 on Jetson).
                'base_frame_id':              'link',
                'enable_depth':               True,
                'enable_color':               True,
                'enable_infra':               False,  # left IR + right IR not needed —
                'enable_infra1':              False,  # bypassing rs_launch.py means we
                'enable_infra2':              False,  # inherit the node's own defaults
                'enable_sync':                True,
                'align_depth.enable':         slam,
                'enable_gyro':                False,  # D435 has no IMU; D435i's BMI055 is disabled —
                'enable_accel':               False,  # WitMotion is the authoritative IMU.
                # Jetson NEON-accelerated pointcloud filter. On x86 builds the
                # filter is declared as plain pointcloud.enable; set both and
                # whichever the running node declares wins.
                'pointcloud__neon_.enable':   pc_enable,
                'pointcloud.enable':          pc_enable,
                'depth_module.depth_profile': '848x480x30',
                'rgb_camera.color_profile':   '848x480x30',
            }],
        )

    front = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_front')),
        actions=[realsense_node('d435_front', 'device_type_front')],
    )
    rear = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_rear')),
        actions=[realsense_node('d435_rear', 'device_type_rear')],
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
