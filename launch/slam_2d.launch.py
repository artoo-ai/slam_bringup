"""Simplified 2D SLAM stack: Mid-360 → 2D scan → rf2o odom → slam_toolbox.

The "vacuum-equivalent" architecture proposed in
docs/why_slam_is_hard_and_how_to_simplify.md. Replaces the full
FAST-LIO2 + RTABMap stack from slam.launch.py with the smallest possible
2D pipeline:

  /livox/lidar  ──pointcloud_to_laserscan──>  /scan
                                                │
                                                ├──> rf2o_laser_odometry  ──>  /odom + (odom→base_link TF)
                                                │
                                                └──> slam_toolbox  ──>  /map + (map→odom TF)

That is Option A from §7.2 of the design note: no wheel odometry, no IMU
fusion. Pure laser-derived motion. The point is to establish a working
2D baseline that mirrors what consumer robot vacuums do, before adding
sensor fusion on top.

Option B (wheel odom + IMU EKF) is stubbed via the use_wheel_odom and
use_imu_ekf launch args. Both default false today; flipping them to
true currently logs a notice and continues with Option A, because the
prerequisite work isn't landed yet:

  1. yahboom_bridge_node.py needs a `publish_odom:=true` opt-in to
     publish nav_msgs/Odometry from encoder telemetry on /wheel/odom.
  2. config/ekf_mecanum.yaml needs to exist (robot_localization EKF
     fusing /wheel/odom + Mid-360 IMU into a synthetic /odom).
  3. The block at the bottom of this file marked "Option B wiring" needs
     to actually start those nodes instead of just logging.

Two phases, one launch file:

  mode:=mapping                       # build a new map (default)
  mode:=localization map_file:=...    # load saved graph + (optionally) Nav2

In mapping mode this spawns async_slam_toolbox_node. In localization
mode it spawns localization_slam_toolbox_node and loads the serialized
.data + .posegraph pair from `map_file` (path WITHOUT extension; e.g.
~/maps/livingroom resolves ~/maps/livingroom.data + .posegraph). Saving
is done from the RViz "SlamToolbox" panel — "Serialize Map" produces
the pair localization mode needs.

Run:
  ./start_slam_2d.sh                                       # mapping
  ./start_slam_2d.sh enable_drive:=true                    # + cmd_vel teleop
  ./start_slam_2d.sh slam_params_file:=/tmp/foo.yaml       # override config

  ./start_nav_2d.sh map_file:=~/maps/livingroom            # localization + Nav2
  ros2 launch slam_bringup slam_2d.launch.py \
      mode:=localization map_file:=~/maps/livingroom nav2:=true enable_drive:=true

This launch deliberately does NOT include FAST-LIO2 or RTABMap. They
remain available via slam.launch.py / start_nav.sh for the 3D path.
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def _option_b_warning(context, *args, **kwargs):
    """Log a clear notice if the user passes use_wheel_odom or use_imu_ekf
    before the Option B prerequisites are wired up. Doesn't fail — just
    tells the user what they're actually getting (Option A).
    """
    use_wheel = LaunchConfiguration('use_wheel_odom').perform(context).lower() in ('1', 'true', 'yes')
    use_ekf   = LaunchConfiguration('use_imu_ekf').perform(context).lower()  in ('1', 'true', 'yes')
    if use_wheel or use_ekf:
        return [LogInfo(msg=(
            'slam_2d.launch: NOTE — use_wheel_odom / use_imu_ekf passed, '
            'but Option B wiring is not yet implemented in this launch. '
            'Falling back to Option A (rf2o laser odometry, no wheel/IMU '
            'fusion). See docs/why_slam_is_hard_and_how_to_simplify.md §8 '
            'for the prerequisite work (yahboom_bridge publish_odom + '
            'config/ekf_mecanum.yaml).'
        ))]
    return []


def _spawn_slam_toolbox(context, *args, **kwargs):
    """Pick mapping vs localization mode and validate map_file at launch.

    mapping       -> async_slam_toolbox_node, params from slam_params_file
    localization  -> localization_slam_toolbox_node, params from
                     slam_params_file PLUS overrides for mode / map_file_name
                     so the serialized graph (.data + .posegraph) is loaded.

    Validates that map_file points at an existing pair before spawning;
    starting localization mode without the files just makes slam_toolbox
    sit silent and you'll spend an hour wondering why /map is empty.
    """
    mode = LaunchConfiguration('mode').perform(context).strip().lower()
    map_file = os.path.expanduser(
        LaunchConfiguration('map_file').perform(context).strip()
    )
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('1', 'true', 'yes')
    params_file = LaunchConfiguration('slam_params_file').perform(context)

    if mode not in ('mapping', 'localization'):
        return [LogInfo(msg=(
            f"slam_2d.launch: ERROR — mode:='{mode}' invalid. "
            f"Use mode:=mapping (default) or mode:=localization."
        ))]

    if mode == 'mapping':
        return [Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('/scan', LaunchConfiguration('scan_topic'))],
        )]

    # Localization mode — validate the serialized pair is on disk.
    if not map_file:
        return [LogInfo(msg=(
            'slam_2d.launch: ERROR — mode:=localization requires '
            'map_file:=<path without extension>. Example: '
            'map_file:=~/maps/livingroom (loads livingroom.data + '
            'livingroom.posegraph).'
        ))]

    data_path  = map_file + '.data'
    graph_path = map_file + '.posegraph'
    missing = [p for p in (data_path, graph_path) if not os.path.exists(p)]
    if missing:
        return [LogInfo(msg=(
            f"slam_2d.launch: ERROR — slam_toolbox serialized map files "
            f"not found: {missing}. In RViz mapping mode use the "
            f"SlamToolbox panel's 'Serialize Map' button (NOT 'Save Map' "
            f"— that produces a .pgm/.yaml pair, not the .data/.posegraph "
            f"pair localization mode needs)."
        ))]

    overrides = {
        'use_sim_time':       use_sim_time,
        'mode':               'localization',
        'map_file_name':      map_file,
        'map_start_at_dock':  True,
    }

    return [
        LogInfo(msg=f"slam_2d.launch: localization — loading {map_file}.{{data,posegraph}}"),
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file, overrides],
            remappings=[('/scan', LaunchConfiguration('scan_topic'))],
        ),
    ]


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('slam_bringup'))
    launch_dir = pkg_share / 'launch'
    default_slam_params = pkg_share / 'config' / 'slam_toolbox_2d.yaml'
    default_nav2_params = pkg_share / 'config' / 'nav2_params_2d.yaml'

    # ---------- Args ----------
    platform_arg = DeclareLaunchArgument(
        'platform', default_value='mecanum',
        description='URDF + sensor mount selector. Same convention as slam.launch.py.',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Set true for bag replay with /clock.',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Spawn rviz2 alongside (off by default — Foxglove on the Jetson).',
    )

    # Phase selector. mapping (default) builds a new graph; localization
    # loads a serialized graph + (optionally) starts Nav2 on top.
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='mapping',
        description='mapping = build a new map (default). localization = load '
                    'serialized graph from map_file and re-localize. Use '
                    './start_nav_2d.sh as the wrapper for the localization+Nav2 '
                    'phase.',
    )
    map_file_arg = DeclareLaunchArgument(
        'map_file', default_value='',
        description='slam_toolbox serialized graph base path WITHOUT extension. '
                    'Example: ~/maps/livingroom resolves '
                    '~/maps/livingroom.data + ~/maps/livingroom.posegraph. '
                    'Required when mode:=localization. Save these from RViz '
                    'via the SlamToolbox panel "Serialize Map" button.',
    )
    nav2_arg = DeclareLaunchArgument(
        'nav2', default_value='false',
        description='Spawn Nav2 (controller + planner + BT + behaviors) on top '
                    'of slam_toolbox. Typically true alongside mode:=localization.',
    )
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file', default_value=str(default_nav2_params),
        description='Nav2 parameters YAML. Default: config/nav2_params_2d.yaml '
                    '(differs from the 3D nav2_params.yaml only by odom_topic '
                    '/odom vs /Odometry).',
    )
    nav2_autostart_arg = DeclareLaunchArgument(
        'nav2_autostart', default_value='true',
        description='If true, lifecycle_manager activates Nav2 nodes immediately.',
    )

    # Option A vs Option B selectors. Both default false — Option A is the
    # baseline. Today, flipping either to true just logs a notice (see
    # _option_b_warning); Option B is not yet wired (see module docstring).
    use_wheel_odom_arg = DeclareLaunchArgument(
        'use_wheel_odom', default_value='false',
        description='Option B: feed yahboom encoder odom to slam_toolbox via the EKF. '
                    'Stub today — falls back to Option A (rf2o).',
    )
    use_imu_ekf_arg = DeclareLaunchArgument(
        'use_imu_ekf', default_value='false',
        description='Option B: launch robot_localization EKF fusing IMU + wheel odom. '
                    'Stub today — falls back to Option A (rf2o).',
    )

    # Topic + framing knobs
    livox_topic_arg = DeclareLaunchArgument(
        'livox_topic', default_value='/livox/lidar',
        description='PointCloud2 input from the Mid-360 (xfer_format=0).',
    )
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic', default_value='/scan',
        description='Flattened LaserScan output topic; consumed by rf2o + slam_toolbox.',
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/odom',
        description='nav_msgs/Odometry topic that rf2o publishes (and slam_toolbox '
                    'consumes via TF). Change only if you are running a separate '
                    'odom source on /odom and want rf2o on a different name.',
    )
    target_frame_arg = DeclareLaunchArgument(
        'target_frame', default_value='base_link',
        description='Frame the LaserScan should be expressed in. base_link is correct '
                    'for slam_toolbox + rf2o on a wheeled rover.',
    )

    # 2D-projection band — which Z slice of the 3D cloud becomes the scan.
    # Defaults aim for chair-leg height on a typical mecanum rover.
    z_min_arg = DeclareLaunchArgument(
        'scan_z_min', default_value='0.15',
        description='Lower Z (m, base_link frame) of the 3D→2D projection band.',
    )
    z_max_arg = DeclareLaunchArgument(
        'scan_z_max', default_value='0.45',
        description='Upper Z (m, base_link frame) of the 3D→2D projection band.',
    )
    range_min_arg = DeclareLaunchArgument(
        'scan_range_min', default_value='0.20',
        description='Inner cutoff (m) — drop points closer than this (rover body).',
    )
    range_max_arg = DeclareLaunchArgument(
        'scan_range_max', default_value='20.0',
        description='Outer cutoff (m) — drop points beyond this. 20 m is plenty indoors.',
    )

    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file', default_value=str(default_slam_params),
        description='YAML config for slam_toolbox. Default: config/slam_toolbox_2d.yaml.',
    )

    # Optional drive bridge (cmd_vel → Yahboom). Off by default.
    drive_args = [
        DeclareLaunchArgument('enable_drive', default_value='false'),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    livox_topic  = LaunchConfiguration('livox_topic')
    scan_topic   = LaunchConfiguration('scan_topic')
    odom_topic   = LaunchConfiguration('odom_topic')
    target_frame = LaunchConfiguration('target_frame')

    # ---------- Perception (URDF + sensors) ----------
    # Mid-360 in PointCloud2 mode (xfer_format=0). D435 OFF — Option A doesn't
    # use it. WitMotion OFF — Option B's IMU EKF would turn this on, but the
    # EKF isn't wired yet, so leave it off to save CPU + USB.
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'perception.launch.py')),
        launch_arguments={
            'platform':          LaunchConfiguration('platform'),
            'use_sim_time':      use_sim_time,
            'rviz':              LaunchConfiguration('rviz'),
            'slam_mode':         'true',
            'lidar_xfer_format': '0',         # PointCloud2 (NOT CustomMsg)
            'enable_witmotion':  'false',
            'enable_mid360':     'true',
            'enable_d435':       'false',     # Option A is laser-only
            'enable_rear':       'false',
        }.items(),
    )

    # ---------- pointcloud_to_laserscan (3D → 2D) ----------
    p2l_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='livox_to_scan',
        output='screen',
        parameters=[{
            'target_frame':      target_frame,
            'transform_tolerance': 0.05,
            'min_height':        LaunchConfiguration('scan_z_min'),
            'max_height':        LaunchConfiguration('scan_z_max'),
            'angle_min':         -3.14159,
            'angle_max':          3.14159,
            'angle_increment':    0.0087,    # ~0.5°
            'scan_time':          0.1,       # 10 Hz
            'range_min':         LaunchConfiguration('scan_range_min'),
            'range_max':         LaunchConfiguration('scan_range_max'),
            'use_inf':            True,
            'inf_epsilon':        1.0,
            'use_sim_time':       use_sim_time,
        }],
        remappings=[
            ('cloud_in', livox_topic),
            ('scan',     scan_topic),
        ],
    )

    # ---------- rf2o_laser_odometry (laser-only odom, Option A) ----------
    # Publishes /odom + odom→base_link TF from successive scans. No wheels,
    # no IMU. This is the "vacuum-equivalent" odometry source for Option A.
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic':  scan_topic,
            'odom_topic':        odom_topic,
            'publish_tf':        True,
            'base_frame_id':     target_frame,
            'odom_frame_id':     'odom',
            'init_pose_from_topic': '',     # disable; start at origin
            'freq':              20.0,
            'use_sim_time':      use_sim_time,
        }],
    )

    # ---------- slam_toolbox (graph SLAM, mode-aware) ----------
    # OpaqueFunction so we can pick async vs localization executable AND
    # validate map_file existence at launch time, after substitutions.
    slam_toolbox_node = OpaqueFunction(function=_spawn_slam_toolbox)

    # ---------- Nav2 (optional) ----------
    # Include nav2_bringup/navigation_launch.py DIRECTLY rather than going
    # through slam_bringup's nav2.launch.py — that one is FAST-LIO2-specific
    # (publishes its own /scan from /cloud_registered_body and a static
    # camera_init→odom alias). Here, /scan and odom→base_link are already
    # published by p2l_node + rf2o_node; we just need the Nav2 pipeline.
    nav2_share = FindPackageShare('nav2_bringup')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([nav2_share, 'launch', 'navigation_launch.py']),
        ]),
        launch_arguments={
            'use_sim_time':    use_sim_time,
            'params_file':     LaunchConfiguration('nav2_params_file'),
            'autostart':       LaunchConfiguration('nav2_autostart'),
            'use_composition': 'False',
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2')),
    )

    # ---------- Drive bridge (optional) ----------
    yahboom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'yahboom.launch.py')),
        condition=IfCondition(LaunchConfiguration('enable_drive')),
    )

    # ---------- Option B stub warning ----------
    option_b_notice = OpaqueFunction(function=_option_b_warning)

    return LaunchDescription([
        platform_arg, use_sim_time_arg, rviz_arg,
        mode_arg, map_file_arg,
        nav2_arg, nav2_params_arg, nav2_autostart_arg,
        use_wheel_odom_arg, use_imu_ekf_arg,
        livox_topic_arg, scan_topic_arg, odom_topic_arg, target_frame_arg,
        z_min_arg, z_max_arg, range_min_arg, range_max_arg,
        slam_params_arg,
        *drive_args,
        option_b_notice,
        perception,
        p2l_node,
        rf2o_node,
        slam_toolbox_node,
        nav2,
        yahboom,
    ])
