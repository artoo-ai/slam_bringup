"""RTABMap graph SLAM + visual loop closure on top of FAST-LIO2 odometry.

Pipeline:
  Mid-360 ──FAST-LIO2──> /Odometry, /cloud_registered_body
                                                │
  D435 front ─realsense2─> rgb + aligned depth ──┤
                                                ▼
                                            RTABMap
                                                │
                                                ├─> /map (OccupancyGrid for Nav2)
                                                ├─> /octomap_full (3D occupancy)
                                                └─> /mapData (graph DB)

External-odometry mode: visual_odometry:=false. FAST-LIO2 owns pose; RTABMap
only consumes it, runs ICP scan refinement, builds the graph, and detects
loop closures via D435 RGB BoW. Saves ~20–40% CPU on the Orin Nano vs
letting RTABMap compute its own visual odometry.

Topics RTABMap subscribes to (after remap):
  odom              ← /Odometry                              (FAST-LIO2)
  scan_cloud        ← /cloud_registered_body                 (FAST-LIO2 body-frame cloud)
  rgb/image         ← /d435_front/camera/color/image_raw
  depth/image       ← /d435_front/camera/aligned_depth_to_color/image_raw
  rgb/camera_info   ← /d435_front/camera/color/camera_info

Frames:
  frame_id      = body         (FAST-LIO2 body frame — the moving rig)
  odom_frame_id = camera_init  (FAST-LIO2 world/odom frame)
  map_frame_id  = map          (RTABMap-corrected world frame; map → camera_init published by RTABMap)

Livox-specific gotchas baked in here (do NOT change without re-reading the wiki):
  Grid/NormalsSegmentation = false
    Default true. Livox emits unorganized clouds; the normals segmentation
    expects organized (row/column) clouds and produces an all-unknown grid
    if left at default. THIS IS THE #1 CAUSE OF "RTABMap runs but /map is
    blank" with a Livox LiDAR.
  Reg/Strategy = 1 (ICP)  +  Icp/PointToPlane = true
    Point-to-plane ICP with the body-frame LiDAR cloud is what tightens the
    graph between keyframes. Visual features alone are too noisy for
    geometric refinement.
  Vis/MinInliers = 15
    Loop-closure threshold on visual feature matches. 15 is conservative
    enough to survive moderate viewpoint change without false positives.
  Rtabmap/DetectionRate = 1.0
    Loop-closure check at 1 Hz. Camera streams faster but the graph only
    needs new keyframes occasionally. Higher rates burn CPU for no gain.

Pre-reqs (will fail loudly via start_rtabmap.sh preflight):
  - sensors.launch.py running with slam_mode:=true (D435 align_depth on)
  - fast_lio.launch.py running (publishing /Odometry + /cloud_registered_body)
  - body ↔ d435_front_link static TF available (URDF or static_transform_publisher)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Set true when replaying bags with simulated clock',
    )
    database_path_arg = DeclareLaunchArgument(
        'database_path', default_value='~/.ros/rtabmap.db',
        description='Where RTABMap persists the graph + visual word DB',
    )
    delete_db_on_start_arg = DeclareLaunchArgument(
        'delete_db_on_start', default_value='false',
        description='true = wipe rtabmap.db at launch (start fresh map). false = resume previous.',
    )
    localization_arg = DeclareLaunchArgument(
        'localization', default_value='false',
        description='true = localization-only mode (no new mapping; reuse existing DB)',
    )
    # Force3DoF clamps z/roll/pitch to 0 — required for indoor wheeled rovers
    # where FAST-LIO drifts in altitude and tilt without LiDAR ceiling
    # constraints. Default false because the bench fixture / handheld rig
    # genuinely uses 6 DoF; enable on the actual rover with force_3dof:=true.
    # Symptoms when this is needed: tf2_echo map base_link shows z drift
    # (e.g. z=-4m while stationary) or persistent pitch/roll while the
    # rover sits still on a flat floor.
    force_3dof_arg = DeclareLaunchArgument(
        'force_3dof', default_value='false',
        description='Constrain pose to x/y/yaw only (recommended for wheeled rovers)',
    )
    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/d435_front/camera/color/image_raw',
        description='RGB image input (D435 front in slam_bringup)',
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/d435_front/camera/aligned_depth_to_color/image_raw',
        description='Aligned depth input (requires D435 align_depth.enable=true)',
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/d435_front/camera/color/camera_info',
        description='RGB camera intrinsics for the depth registration',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    # Parameters shared across rtabmap + rtabmap_viz nodes. Strings, per
    # RTABMap convention — the C++ side does its own parsing.
    rtabmap_params = {
        # --- Frames ---------------------------------------------------------
        'frame_id':       'body',          # FAST-LIO2 body frame
        'odom_frame_id':  'camera_init',   # FAST-LIO2 world/odom frame
        'map_frame_id':   'map',
        'publish_tf':     True,            # RTABMap publishes map → camera_init
        'use_sim_time':   use_sim_time,

        # --- External odometry (FAST-LIO2 owns pose) ------------------------
        # We do NOT subscribe to odom messages — RTABMap reads the
        # camera_init → body TF that FAST-LIO2 publishes. odom_topic stays
        # subscribed for the timestamp sync but visual_odometry is OFF.
        'subscribe_odom_info': False,
        'wait_for_transform': 0.2,
        'approx_sync': True,
        'approx_sync_max_interval': 0.05,
        'queue_size': 30,
        'qos':        1,                   # match D435/FAST-LIO2 (BEST_EFFORT)

        # --- Inputs ---------------------------------------------------------
        'subscribe_rgb':        True,
        'subscribe_depth':      True,
        'subscribe_scan_cloud': True,      # FAST-LIO2 body-frame cloud → ICP

        # --- Persistence ----------------------------------------------------
        'database_path':       database_path,
        'Mem/IncrementalMemory': 'true',   # flipped to false in localization mode below
        'Mem/InitWMWithAllNodes': 'false',

        # --- Registration: ICP scan-matching (Livox needs this) ------------
        'Reg/Strategy': '1',               # 0 = vis only, 1 = ICP, 2 = vis+ICP
        # Reg/Force3DoF is overridden at launch time via force_3dof:=true,
        # see _spawn_rtabmap below. Default 'false' here is the 6 DoF
        # baseline used by handheld + bench testing.
        'Reg/Force3DoF': 'false',
        'Icp/PointToPlane': 'true',
        'Icp/Iterations': '10',
        'Icp/VoxelSize': '0.1',
        'Icp/MaxCorrespondenceDistance': '1.0',
        'Icp/Epsilon': '0.001',
        'Icp/MaxTranslation': '2.0',
        # RGBD/LocalImmunityRadius: how close (in odom space) two
        # keyframes must be for RTABMap to consider a "local" loop
        # closure. Default 1.0 m is too tight for our stack — FAST-LIO
        # drift makes neighboring real-world keyframes look 1.06–1.13 m
        # apart in odom space ("Ignoring local loop closure with X
        # because resulting transform is too large!? (1.060074m >
        # 1.000000m)"). 2.0 m absorbs the drift without admitting
        # genuinely-distant closures.
        'RGBD/LocalImmunizationRatio': '0.0',
        'RGBD/MaxLocalRetrieved':      '5',
        'RGBD/LocalRadius':            '2.0',

        # --- Occupancy grid (Livox-unorganized-cloud rules) -----------------
        'Grid/Sensor':              '0',     # 0 = laser-scan mode
        'Grid/3D':                  'true',
        'Grid/RayTracing':          'true',
        'Grid/NormalsSegmentation': 'false', # CRITICAL — must be false for Livox
        'Grid/CellSize':            '0.1',
        'Grid/RangeMax':            '30.0',
        'Grid/RangeMin':            '0.1',
        'Grid/GroundIsObstacle':    'true',  # bench fixture has no ground plane
        'Grid/NoiseFilteringRadius': '0.0',
        'Grid/MinClusterSize':      '5',
        'Grid/MaxObstacleHeight':   '2.0',

        # --- Loop closure ---------------------------------------------------
        'Vis/MinInliers':           '15',
        'Vis/EstimationType':       '1',     # 1 = PnP (RGB-D friendly)
        'Vis/MaxFeatures':          '1000',
        # OptimizeMaxError sets the σ multiplier above which the optimizer
        # rejects a new loop closure as "the resulting correction is so
        # large it must be wrong." Bumped from 3.0 → 5.0 because the
        # initial Roboscout map had heavy FAST-LIO drift baked in (942 m
        # unoptimized error on a 52 m trajectory) — at 3σ, every legit
        # loop closure trying to reel that drift back in was getting
        # flagged as "wrong." 5σ lets the corrections through. Re-mapping
        # with force_3dof:=true is the long-term fix; this knob is the
        # workaround for already-saved maps.
        'RGBD/OptimizeMaxError':    '5.0',
        'RGBD/ProximityBySpace':    'true',
        'RGBD/AngularUpdate':       '0.05',  # rad — keyframe trigger
        'RGBD/LinearUpdate':        '0.05',  # m   — keyframe trigger

        # --- Memory / pacing ------------------------------------------------
        'Rtabmap/DetectionRate':    '1.0',
        'Rtabmap/MemoryThr':        '0',     # 0 = no working-memory cap
        'Rtabmap/TimeThr':          '0',
    }

    # Localization-only override: load the DB but don't add new nodes.
    # (delete_db_on_start must be false for this to make sense — guarded
    # at the launch script level, not here.)
    localization_params = {
        'Mem/IncrementalMemory':   'false',
        'Mem/InitWMWithAllNodes':  'true',
    }

    def _spawn_rtabmap(context, *args, **kwargs):
        """Resolve delete_db_on_start + localization + force_3dof at launch
        time so the rtabmap CLI flag and the Mem/IncrementalMemory and
        Reg/Force3DoF params actually reflect what the user passed. Plain
        LaunchConfiguration in Node.arguments doesn't gate a flag — empty
        string vs. flag has to be decided after substitution, which is
        what OpaqueFunction does."""
        delete_db = LaunchConfiguration('delete_db_on_start').perform(context).lower() in ('true', '1')
        localize  = LaunchConfiguration('localization').perform(context).lower() in ('true', '1')
        force3dof = LaunchConfiguration('force_3dof').perform(context).lower() in ('true', '1')

        node_args = []
        if delete_db:
            node_args.append('--delete_db_on_start')

        params = dict(rtabmap_params)
        if localize:
            params.update(localization_params)
        if force3dof:
            params['Reg/Force3DoF'] = 'true'

        return [Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            emulate_tty=True,
            parameters=[params],
            arguments=node_args,
            remappings=[
                ('rgb/image',       rgb_topic),
                ('depth/image',     depth_topic),
                ('rgb/camera_info', camera_info_topic),
                ('scan_cloud',      '/cloud_registered_body'),
                ('odom',            '/Odometry'),
            ],
        )]

    rtabmap_node = OpaqueFunction(function=_spawn_rtabmap)

    # rtabmap_viz: optional GUI. Off by default — Foxglove is the primary
    # viewer on the Jetson. Enable with viz:=true for desktop debugging.
    viz_arg = DeclareLaunchArgument(
        'viz', default_value='false',
        description='Spawn rtabmap_viz GUI (heavy on the Jetson; off by default)',
    )

    return LaunchDescription([
        use_sim_time_arg,
        database_path_arg,
        delete_db_on_start_arg,
        localization_arg,
        force_3dof_arg,
        rgb_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        viz_arg,
        rtabmap_node,
    ])
