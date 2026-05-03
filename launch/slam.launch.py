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
from launch.conditions import IfCondition
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
    # Bench fixture: base_link is at the bottom of the lower 2040 cage
    # (table contact). body (≈ Mid-360 IMU center) sits at:
    #   base_link → sensor_plate top:           +0.207670 m  (8.176" stack-up)
    #   sensor_plate → livox_frame (Z):         +0.036610 m  (Livox datasheet)
    #   ───────────────────────────────────
    #   body Z above base_link:                  0.244280 m
    #
    # First-pass approximation: body ≈ LiDAR optical center. The
    # Mid-360's onboard IMU is offset from the LiDAR by the Livox
    # factory extrinsic [-0.011, -0.02329, 0.04412] (in fast_lio_mid360
    # .yaml), which is below the noise floor of the bridge for
    # short-range mapping — FAST-LIO2 absorbs the mm-scale offset
    # internally during ESKF convergence.
    #
    # Bridge transform body → base_link: base_link sits 0.244280 m
    # BELOW body, so the static TF translation is (0, 0, -0.244280).
    'bench_fixture': (0.0, 0.0, -0.244280, 0.0, 0.0, 0.0),

    # Mecanum UGV (Yahboom YB-ERF01-V3.0 chassis, ROSMASTER X3 firmware).
    # PLACEHOLDER offset — measure from the wheel-bottom contact point
    # (= base_link in the Yahboom URDF convention) up to the top of the
    # sensor plate, then add the plate-top → livox_frame offset (36.61 mm
    # per Livox datasheet). Until measured, falls back to the bench-fixture
    # number minus an estimated 100 mm chassis height. Re-measure before
    # trusting the map z-axis.
    'mecanum':   (0.0, 0.0, -0.144280, 0.0, 0.0, 0.0),

    # Stubs — fill in once each platform's URDF + measured plate offsets
    # are landed. Until then `platform:=<name>` will exit with a clear
    # error (see _build_bridge below). Numbers below are placeholders;
    # do NOT trust them.
    # 'go2':       (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    # 'r2d2':      (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    # 'roboscout': (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
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
    pkg_share = Path(get_package_share_directory('slam_bringup'))
    launch_dir = pkg_share / 'launch'
    default_nav2_params = pkg_share / 'config' / 'nav2_params.yaml'

    platform_arg = DeclareLaunchArgument(
        'platform', default_value='mecanum',
        description='Selects URDF + body→base_link bridge from PLATFORM_BRIDGES. '
                    'Default `mecanum` because it is the working test bed; pass '
                    '`platform:=bench_fixture` for the rare standalone-rig debug case.',
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
        # force_3dof clamps z/roll/pitch in RTABMap's loop-closure
        # optimizer — the right model for an indoor wheeled rover on a
        # flat floor. Default ON because the mecanum rover is the
        # working platform; flip to false only when handheld / bench
        # testing where the rig genuinely uses 6 DoF. Symptom this
        # avoids: tf2_echo map base_link reporting z = -4 m while
        # stationary.
        DeclareLaunchArgument('force_3dof',         default_value='true'),
    ]

    # Visualization-only z-clip on /cloud_registered. See viz_clip.launch.py.
    # Defaults are tuned for indoor/house viewing; raise viz_z_max for garages
    # (e.g. viz_z_max:=4.5) or set enable_viz_clip:=false to disable.
    viz_clip_args = [
        DeclareLaunchArgument('enable_viz_clip', default_value='true'),
        DeclareLaunchArgument('viz_z_min',       default_value='-3.0'),
        DeclareLaunchArgument('viz_z_max',       default_value='3.0'),
    ]

    # Nav2 pass-through. Off by default — only enable once you have a
    # localized map (./start_nav.sh wraps that). Bench fixture has no
    # cmd_vel consumer, so this is purely planner-visualization there.
    nav2_args = [
        DeclareLaunchArgument('nav2',             default_value='false'),
        DeclareLaunchArgument('nav2_params_file', default_value=str(default_nav2_params)),
        DeclareLaunchArgument('nav2_autostart',   default_value='true'),
    ]

    # Per-platform /cmd_vel → drive-base bridge. Off by default because
    # the bench fixture has no motors and most early debugging doesn't
    # want a watchdog timer running on a serial port. Enable on the
    # mecanum rover with `enable_drive:=true` (only valid with
    # platform:=mecanum today; other platforms need their own bridge).
    drive_args = [
        DeclareLaunchArgument('enable_drive', default_value='false'),
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
    # rviz='false' is forced here on purpose. fast_lio.launch.py cascades
    # rviz to upstream FAST_LIO_ROS2/mapping.launch.py, which spawns its
    # own rviz2 with the upstream debug config — and perception.launch.py
    # ALSO spawns rviz2 (with rviz/perception.rviz that has the URDF +
    # sensors). Without this pin, `start_slam.sh rviz:=true` opens two
    # RViz windows. Keep the perception one; it's the keeper.
    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'fast_lio.launch.py')),
        launch_arguments={'rviz': 'false'}.items(),
    )

    # ---------- RTABMap ----------
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'rtabmap.launch.py')),
        launch_arguments={
            'use_sim_time':       LaunchConfiguration('use_sim_time'),
            'database_path':      LaunchConfiguration('database_path'),
            'delete_db_on_start': LaunchConfiguration('delete_db_on_start'),
            'localization':       LaunchConfiguration('localization'),
            'force_3dof':         LaunchConfiguration('force_3dof'),
        }.items(),
    )

    # ---------- Viz-only z-clip (top-down floorplan view) ----------
    viz_clip = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'viz_clip.launch.py')),
        launch_arguments={
            'enable_viz_clip': LaunchConfiguration('enable_viz_clip'),
            'viz_z_min':       LaunchConfiguration('viz_z_min'),
            'viz_z_max':       LaunchConfiguration('viz_z_max'),
        }.items(),
    )

    # ---------- Nav2 (optional) ----------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'nav2.launch.py')),
        launch_arguments={
            'use_sim_time':     LaunchConfiguration('use_sim_time'),
            'nav2_params_file': LaunchConfiguration('nav2_params_file'),
            'nav2_autostart':   LaunchConfiguration('nav2_autostart'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2')),
    )

    # ---------- Drive-base bridge (optional, per-platform) ----------
    # Currently only mecanum is wired (Yahboom YB-ERF01 over USB-serial).
    # Go2 / R2D2 / Roboscout will get their own includes here once their
    # bridges land.
    yahboom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / 'yahboom.launch.py')),
        condition=IfCondition(LaunchConfiguration('enable_drive')),
    )

    return LaunchDescription([
        platform_arg, use_sim_time_arg, rviz_arg,
        *rtabmap_args,
        *viz_clip_args,
        *nav2_args,
        *drive_args,
        perception,
        bridge,
        fast_lio,
        rtabmap,
        viz_clip,
        nav2,
        yahboom,
    ])
