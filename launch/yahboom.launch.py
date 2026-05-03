"""Yahboom YB-ERF01 (mecanum X3) cmd_vel bridge.

Run standalone for teleop testing:

    ./start_yahboom.sh                       # default /dev/myserial, car_type=1 (X3 mecanum)
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # press 'b' to enable holonomic mode → u/o/m/. command strafe (vy)

Or include from slam.launch.py via enable_drive:=true. See PLATFORM_BRIDGES
in slam.launch.py for the per-platform mecanum entry. Companion vault
notes:
  - "Yahboom Mecanum Configuration - Motor Wiring and Taranis SBUS Setup"
  - "Mecanum UGV - GitHub - AutomaticAddison ROSMASTER X3 ROS2"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/myserial',
        description='USB-serial path to the Yahboom STM32. Pinned to /dev/myserial '
                    'via /etc/udev/rules.d/99-yahboom.rules so it survives ttyUSB '
                    'renumbering. If udev pin is missing, fall back to /dev/ttyUSB0.',
    )
    car_type_arg = DeclareLaunchArgument(
        'car_type', default_value='1',
        description='Yahboom firmware car_type constant. 1=X3 mecanum (default), '
                    '2=X3 PLUS, 4=X1, 5=R2.',
    )
    cmd_timeout_arg = DeclareLaunchArgument(
        'cmd_timeout', default_value='0.5',
        description='Watchdog: zero motors if /cmd_vel is silent for this many seconds.',
    )
    max_vx_arg = DeclareLaunchArgument('max_vx', default_value='0.5')
    max_vy_arg = DeclareLaunchArgument('max_vy', default_value='0.3')
    max_wz_arg = DeclareLaunchArgument('max_wz', default_value='1.0')

    bridge = Node(
        package='slam_bringup',
        executable='yahboom_bridge',
        name='yahboom_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'car_type':    LaunchConfiguration('car_type'),
            'cmd_timeout': LaunchConfiguration('cmd_timeout'),
            'max_vx':      LaunchConfiguration('max_vx'),
            'max_vy':      LaunchConfiguration('max_vy'),
            'max_wz':      LaunchConfiguration('max_wz'),
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        car_type_arg,
        cmd_timeout_arg,
        max_vx_arg,
        max_vy_arg,
        max_wz_arg,
        bridge,
    ])
