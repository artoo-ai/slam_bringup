"""Full SLAM stack: URDF + sensors + FAST-LIO2 + RTABMap.

Composition over re-declaration. This file does not duplicate any of
the per-stage launches — it just chains:

  slam.launch.py
  ├── perception.launch.py    (URDF + sensors, slam_mode=true, xfer_format=1)
  ├── static TF: base_link → body  (per-platform offset, see PLATFORM_BRIDGES)
  ├── fast_lio.launch.py      (Mid-360 + IMU → /Odometry, /cloud_registered_body)
  └── rtabmap.launch.py       (RTABMap graph SLAM + visual loop closure)

TF tree at runtime:

  map ──RTABMap──> camera_init ──FAST-LIO2──> body ──slam.launch.py bridge──> base_link
                                                                                │
                                                                                └── sensor_plate
                                                                                       ├── livox_frame
                                                                                       ├── imu_link
                                                                                       └── d435_front_link
                                                                                              └── (realsense2_camera child frames)

PLATFORM_BRIDGES — single source of truth for `body → base_link`
(static, per-platform). Switch platforms with `platform:=<name>`; the
matching bridge is published.

The bridge maps FAST-LIO2's `body` (the Mid-360's IMU body frame, by
construction the rigid coupling FAST-LIO2 estimates) onto each
platform's `base_link`. For the bench fixture, body sits ~106.6 mm
above base_link (Mid-360 IMU center ≈ LiDAR optical center for a
first-pass bridge: 36.61 mm above plate top + 69.987 mm plate-top above
base_link; the few-mm offset between IMU and LiDAR per the Livox
factory extrinsic is below the noise floor of the bridge and can be
ignored — FAST-LIO2 absorbs it internally).

Real platforms (Go2, R2D2, Roboscout, mecanum) get their own entries
once their URDF + measured plate-mount offsets are landed (Phase 1.7).
"""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# Per-platform body → base_link static TF.
# Format: 'platform': (x, y, z, roll, pitch, yaw)  — meters / radians.
# Convention: this is the transform FROM `body` (FAST-LIO2 IMU frame)
# TO `base_link` (URDF root for that platform). The static_transform_publisher
# below publishes `body` as the parent, `base_link` as the child.
PLATFORM_BRIDGES = {
    # Bench fixture: base_link is at the bottom of the 2040 cage
    # (table contact). body (Mid-360 IMU) sits at plate top + 36.61 mm
    # ≈ 0.069987 + 0.03661 = 0.106597 m above base_link. Bridge from
    # body's perspective places base_link 0.1066 m BELOW body.
    'bench_fixture': (0.0, 0.0, -0.1066, 0.0, 0.0, 0.0),

    # Stubs — fill in once each platform's URDF + measured plate offsets
    # are landed. Until then `platform:=<name>` will exit with a clear
    # error (see _build_bridge below). Numbers below are placeholders;
    # do NOT trust them.
    # 'go2':       (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    # 'r2d2':      (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    # 'roboscout': (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    # 'mecanum':   (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
}


def _build_bridge(context, *args, **kwargs):
    """Resolve the platform arg at launch time, then emit the bridge node.

    OpaqueFunction lets us read the platform LaunchConfiguration value
    AFTER substitution and look it up in PLATFORM_BRIDGES — a plain
    Node() can't do that because Python dict lookups don't accept
    LaunchConfiguration objects.
    """
    platform = LaunchConfiguration('platform').perform(context)
    if platform not in PLATFORM_BRIDGES:
        return [LogInfo(msg=(
            f"slam.launch: ERROR — no PLATFORM_BRIDGES entry for "
            f"'{platform}'. Add one to launch/slam.launch.py once the "
            f"platform's URDF and measured plate-mount offset are "
            f"available. Known platforms: "
            f"{sorted(PLATFORM_BRIDGES.keys())}"
        ))]

    x, y, z, r, p, yaw = PLATFORM_BRIDGES[platform]
    return [Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'body_to_base_link_{platform}',
        output='screen',
        arguments=[
            '--x',     str(x),
            '--y',     str(y),
            '--z',     str(z),
            '--roll',  str(r),
            '--pitch', str(p),
            '--yaw',   str(yaw),
            '--frame-id',       'body',
            '--child-frame-id', 'base_link',
        ],
    )]


def generate_launch_description():
    launch_dir = Path(get_package_share_directory('slam_bringup')) / 'launch'

    platform_arg = DeclareLaunchArgument(
        'platform', default_value='bench_fixture',
        description='Selects URDF + body→base_link bridge from PLATFORM_BRIDGES.',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Set true for bag replay with simulated clock.',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Spawn rviz2 alongside (off by default — Foxglove on the Jetson).',
    )

    # RTABMap pass-through — common knobs we want at the top level
    # without users having to drill into rtabmap.launch.py.
    rtabmap_args = [
        DeclareLaunchArgument('database_path',      default_value='~/.ros/rtabmap.db'),
        DeclareLaunchArgument('delete_db_on_start', default_value='false'),
        DeclareLaunchArgument('localization',       default_value='false'),
    ]

    # ---------- perception (URDF + sensors in SLAM mode) ----------
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'perception.launch.py')),
        launch_arguments={
            'platform':          LaunchConfiguration('platform'),
            'use_sim_time':      LaunchConfiguration('use_sim_time'),
            'rviz':              LaunchConfiguration('rviz'),
            # SLAM-mode forcings — these are NOT user-overridable args
            # at this level on purpose. SLAM needs:
            'slam_mode':         'true',     # D435 align_depth on, RealSense pointcloud off
            'lidar_xfer_format': '1',        # Mid-360 CustomMsg for FAST-LIO2
            'enable_witmotion':  'false',    # not used by FAST-LIO2; saves CPU
            'enable_mid360':     'true',
            'enable_d435':       'true',
            'enable_rear':       'false',    # rear D435i = Phase 1.10
        }.items(),
    )

    # ---------- body → base_link bridge ----------
    bridge = OpaqueFunction(function=_build_bridge)

    # ---------- FAST-LIO2 ----------
    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'fast_lio.launch.py')),
    )

    # ---------- RTABMap ----------
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'rtabmap.launch.py')),
        launch_arguments={
            'use_sim_time':       LaunchConfiguration('use_sim_time'),
            'database_path':      LaunchConfiguration('database_path'),
            'delete_db_on_start': LaunchConfiguration('delete_db_on_start'),
            'localization':       LaunchConfiguration('localization'),
        }.items(),
    )

    return LaunchDescription([
        platform_arg, use_sim_time_arg, rviz_arg,
        *rtabmap_args,
        perception,
        bridge,
        fast_lio,
        rtabmap,
    ])
