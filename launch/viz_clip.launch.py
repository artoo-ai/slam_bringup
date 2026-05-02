"""Visualization-only z-axis clip on the FAST-LIO2 world-frame cloud.

Problem: FAST-LIO2 publishes /cloud_registered in `camera_init` (world)
with full vertical extent — ceiling, rafters, the works. In a top-down
RViz/Foxglove view, that hides the floorplan: every camera angle either
ends up inside the room walls (zoomed in) or so high it just shows
new-room outlines (zoomed out). We want a top-down section through the
living space.

Approach: a parallel `pcl_ros::PassThrough` filter that subscribes to
/cloud_registered, clips the z field to [viz_z_min, viz_z_max], and
republishes on /cloud_viz_clipped — purely for visualization. The raw
/cloud_registered_body still goes to RTABMap unchanged, so SLAM/ICP and
the occupancy grid keep their full vertical info (still useful for
measuring rafters via the RTABMap 3D map and /octomap_full).

Frame note — z=0 in /cloud_registered:
    /cloud_registered is in FAST-LIO2's `camera_init` frame. camera_init's
    origin is the body pose AT STARTUP — i.e. wherever the IMU was when
    you launched FAST-LIO2. So `viz_z_max = 2.0` means "2.0 m above the
    rig's startup height," not "2.0 m above the floor."

Defaults are tuned for a house: viz_z_max = 2.0 m above startup is a
reasonable section line for typical 2.4 m residential ceilings with a
robot starting near the floor. For a garage with high ceilings or
exposed rafters, override at launch:

    ./start_slam.sh viz_z_max:=4.5

To disable entirely (publish nothing extra):

    ./start_slam.sh enable_viz_clip:=false

The original /cloud_registered topic is left untouched in either case.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    enable_arg = DeclareLaunchArgument(
        'enable_viz_clip', default_value='true',
        description='If true, run the PassThrough z-clip and publish /cloud_viz_clipped.',
    )
    input_topic_arg = DeclareLaunchArgument(
        'viz_input_topic', default_value='/cloud_registered',
        description='World-frame cloud to clip (FAST-LIO2 default: /cloud_registered).',
    )
    output_topic_arg = DeclareLaunchArgument(
        'viz_output_topic', default_value='/cloud_viz_clipped',
        description='Republished z-clipped cloud (subscribe to this in RViz/Foxglove).',
    )
    z_min_arg = DeclareLaunchArgument(
        'viz_z_min', default_value='-1.0',
        description='Lower z bound (m) in camera_init frame. Below this is dropped.',
    )
    z_max_arg = DeclareLaunchArgument(
        'viz_z_max', default_value='2.0',
        description='Upper z bound (m) in camera_init frame. Above this is dropped — '
                    'the "ceiling clip." Raise for garages / high-ceiling spaces.',
    )

    enable_viz_clip = LaunchConfiguration('enable_viz_clip')
    viz_input_topic = LaunchConfiguration('viz_input_topic')
    viz_output_topic = LaunchConfiguration('viz_output_topic')
    viz_z_min = LaunchConfiguration('viz_z_min')
    viz_z_max = LaunchConfiguration('viz_z_max')

    # pcl_ros ships PassThrough as a composable node. Loading it via a
    # component container is the lightweight path — no extra process,
    # no copy on the message bus.
    container = ComposableNodeContainer(
        condition=IfCondition(enable_viz_clip),
        name='viz_clip_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='pcl_ros',
                plugin='pcl_ros::PassThrough',
                name='viz_z_clip',
                remappings=[
                    ('input',  viz_input_topic),
                    ('output', viz_output_topic),
                ],
                parameters=[{
                    'filter_field_name': 'z',
                    'filter_limit_min':  viz_z_min,
                    'filter_limit_max':  viz_z_max,
                    'filter_limit_negative': False,
                    'keep_organized':    False,
                    'input_frame':       '',
                    'output_frame':      '',
                }],
            ),
        ],
    )

    return LaunchDescription([
        enable_arg,
        input_topic_arg,
        output_topic_arg,
        z_min_arg,
        z_max_arg,
        container,
    ])
