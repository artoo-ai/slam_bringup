"""Nav2 navigation on top of FAST-LIO2 + RTABMap.

What this launches:
  1. pointcloud_to_laserscan — converts /cloud_registered_body (Mid-360,
     360°, body-frame) to /scan (sensor_msgs/LaserScan). Nav2's stock
     costmap plugins want LaserScan; the Mid-360 doesn't natively emit
     one, so we slice the registered body cloud into a 2D ring at the
     robot height range (min_height/max_height below).
  2. static TF camera_init → odom — Nav2's costmap plugins use a frame
     literally named "odom" by default. FAST-LIO2 calls its odom frame
     "camera_init", so we publish an identity TF from camera_init to
     odom rather than rewriting every plugin's frame parameter.
  3. nav2_bringup/navigation_launch.py — controller + planner + BT
     navigator + behavior server + waypoint follower + velocity smoother.
     We do NOT include map_server (RTABMap publishes /map) or amcl
     (RTABMap publishes the map → camera_init correction).

Topics:
  /map               (in)  ← RTABMap                 — global_costmap static_layer
  /Odometry          (in)  ← FAST-LIO2               — controller velocity feedback
  /scan              (in)  ← pointcloud_to_laserscan — costmap obstacle layers
  /goal_pose         (in)  ← RViz "2D Goal Pose"     — bt_navigator entrypoint
  /cmd_vel           (out) → platform velocity bridge

Bench fixture caveat: nothing consumes /cmd_vel — useful only for
visualizing planned trajectories in RViz. Real platforms (Go2/R2D2/
mecanum) need a Twist→drive-base bridge in their own launch files.

Footprint: see config/nav2_params.yaml — circular, robot_radius 0.4 m.
Override per-platform with `nav2_params_file:=...`.
"""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('slam_bringup'))
    default_params = pkg_share / 'config' / 'nav2_params.yaml'

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Set true for bag replay with simulated clock.',
    )
    params_file_arg = DeclareLaunchArgument(
        'nav2_params_file', default_value=str(default_params),
        description='Nav2 parameters YAML. Override per-platform for footprint, vel limits, etc.',
    )
    autostart_arg = DeclareLaunchArgument(
        'nav2_autostart', default_value='true',
        description='If true, lifecycle_manager activates Nav2 nodes immediately.',
    )

    # ---------- /cloud_registered_body → /scan ---------------------------
    # Slice the Mid-360 body-frame cloud into a 2D laserscan at robot
    # height. Bench fixture / Go2 / R2D2 all live somewhere between the
    # floor and 2 m, so [0.1, 2.0] catches walls/furniture without the
    # ceiling. target_frame=base_link puts /scan in the URDF root frame
    # so Nav2's costmaps don't need extra TF hops.
    pc2laser = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time':          LaunchConfiguration('use_sim_time'),
            'target_frame':          'base_link',
            'transform_tolerance':   0.05,
            'min_height':            0.1,
            'max_height':            2.0,
            'angle_min':            -3.14159,
            'angle_max':             3.14159,
            'angle_increment':       0.0087,   # ~0.5°
            'scan_time':             0.1,
            'range_min':             0.5,      # matches FAST-LIO blind
            'range_max':             30.0,
            'use_inf':               True,
            'inf_epsilon':           1.0,
            # Mid-360 publishes ~10 Hz; concurrency 1 keeps ordering simple.
            'concurrency_level':     1,
        }],
        remappings=[
            ('cloud_in', '/cloud_registered_body'),
            ('scan',     '/scan'),
        ],
    )

    # ---------- camera_init → odom alias ---------------------------------
    # Nav2 plugins (controller_server local_frame, behavior_server local_frame,
    # local_costmap.global_frame) default to a frame named "odom". FAST-LIO2
    # calls its odom frame "camera_init". Rather than override each plugin
    # individually, publish identity camera_init → odom.
    camera_init_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init_to_odom',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id',       'camera_init',
            '--child-frame-id', 'odom',
        ],
    )

    # ---------- Nav2 navigation pipeline ---------------------------------
    # navigation_launch.py spawns: controller_server, planner_server,
    # smoother_server, behavior_server, bt_navigator, waypoint_follower,
    # velocity_smoother, lifecycle_manager_navigation.
    nav2_share = FindPackageShare('nav2_bringup')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([nav2_share, 'launch', 'navigation_launch.py']),
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file':  LaunchConfiguration('nav2_params_file'),
            'autostart':    LaunchConfiguration('nav2_autostart'),
            'use_composition': 'False',
        }.items(),
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        autostart_arg,
        pc2laser,
        camera_init_to_odom,
        nav2,
    ])
