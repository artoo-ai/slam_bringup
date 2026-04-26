"""Perception bringup: URDF + sensors + (optional) rviz2.

This is the dev-laptop / RViz workflow. It loads the platform URDF via
robot_state_publisher and brings up all three sensor drivers, but does
NOT start FAST-LIO2 or RTABMap. For the full SLAM stack use
slam.launch.py.

What this launch produces:
  - /robot_description     full URDF for the selected platform
  - /tf, /tf_static        sensor_plate + sensor children + (per platform) base_link
  - All sensor topics     (Mid-360, D435 front[+rear], WitMotion)
  - rviz2                  optional, default off

Layered launch arg model — pass through to children, don't re-declare:

  perception.launch.py
  ├── platform                         (this file)
  ├── rviz                             (this file)
  └── sensors.launch.py forwards:
      slam_mode, lidar_xfer_format,
      enable_rear, enable_{mid360,d435,witmotion}

So `./start_sensors.sh enable_d435:=false` and
`ros2 launch slam_bringup perception.launch.py enable_d435:=false`
both work the same way — sensors.launch.py is the single source of
truth for sensor toggles.

Platform argument selects the URDF in urdf/<platform>.urdf.xacro:

  platform:=bench_fixture   2040 benchtop test rig (default)
  platform:=go2             Unitree Go2 Pro              (Phase 1.7 — placeholder until urdf/ drops)
  platform:=r2d2            Lifesize R2D2                (Phase 1.7 — placeholder)
  platform:=roboscout       Sharper Image Roboscout      (Phase 1.7 — placeholder)
  platform:=mecanum         Mecanum 4-wheel UGV          (Phase 1.7 — placeholder)

Until the per-robot xacros are written, only platform:=bench_fixture
will actually expand. The launch will fail loudly (xacro error) if you
pass a platform whose .urdf.xacro file doesn't exist — that's intended.
"""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = Path(get_package_share_directory('slam_bringup'))
    launch_dir = bringup_share / 'launch'
    urdf_dir = bringup_share / 'urdf'

    platform_arg = DeclareLaunchArgument(
        'platform',
        default_value='bench_fixture',
        description='Selects urdf/<platform>.urdf.xacro. '
                    'Currently implemented: bench_fixture. '
                    'Stubs reserved: go2, r2d2, roboscout, mecanum.',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Spawn rviz2 (off by default — dev workflow uses Foxglove).',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=str(bringup_share / 'rviz' / 'perception.rviz'),
        description='RViz config file. Default points at rviz/perception.rviz '
                    '(may not exist yet — Phase 1.7 deliverable).',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Set true when replaying bags with simulated clock.',
    )

    # Sensor-bringup pass-through args (re-declared here so users can
    # see them in `ros2 launch slam_bringup perception.launch.py --show-args`
    # without having to drill into sensors.launch.py).
    sensor_args = [
        DeclareLaunchArgument('slam_mode',          default_value='false'),
        DeclareLaunchArgument('lidar_xfer_format',  default_value='0'),
        DeclareLaunchArgument('enable_rear',        default_value='false'),
        DeclareLaunchArgument('enable_mid360',      default_value='true'),
        DeclareLaunchArgument('enable_d435',        default_value='true'),
        DeclareLaunchArgument('enable_witmotion',   default_value='true'),
    ]

    # ---------- robot_description = xacro <urdf/<platform>.urdf.xacro> ----------
    # Using PathJoinSubstitution lets the platform arg resolve at launch
    # time. Command(...) runs xacro and pipes its expanded XML into the
    # robot_description parameter — same pattern Nav2 / MoveIt use.
    urdf_path = PathJoinSubstitution([
        str(urdf_dir),
        [LaunchConfiguration('platform'), '.urdf.xacro'],
    ])
    robot_description = {
        'robot_description': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ', urdf_path]),
            value_type=str,
        ),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ---------- Sensors ----------
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'sensors.launch.py')),
        launch_arguments={
            'slam_mode':         LaunchConfiguration('slam_mode'),
            'lidar_xfer_format': LaunchConfiguration('lidar_xfer_format'),
            'enable_rear':       LaunchConfiguration('enable_rear'),
            'enable_mid360':     LaunchConfiguration('enable_mid360'),
            'enable_d435':       LaunchConfiguration('enable_d435'),
            'enable_witmotion':  LaunchConfiguration('enable_witmotion'),
        }.items(),
    )

    # ---------- rviz2 (optional) ----------
    # Rendered with the URDF tree visible (RobotModel display) so you
    # can sanity-check sensor placement against the measured offsets.
    rviz = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        platform_arg,
        rviz_arg,
        rviz_config_arg,
        use_sim_time_arg,
        *sensor_args,
        robot_state_publisher,
        sensors,
        rviz,
    ])
