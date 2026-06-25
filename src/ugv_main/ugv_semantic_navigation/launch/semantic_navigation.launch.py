"""Launch semantic exploration and navigation on the physical UGV rover."""

from datetime import datetime
import os
from pathlib import Path
import shutil
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _arg(name, default, description):
    return DeclareLaunchArgument(name, default_value=default, description=description)


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _start_fresh(context):
    if not _as_bool(LaunchConfiguration('start_fresh').perform(context)):
        return []

    runtime_path = Path(os.path.expanduser(LaunchConfiguration('runtime_dir').perform(context)))
    runtime_path.parent.mkdir(parents=True, exist_ok=True)
    if runtime_path.exists() and any(runtime_path.iterdir()):
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        archive_path = runtime_path.with_name(f'{runtime_path.name}_{stamp}')
        suffix = 1
        while archive_path.exists():
            archive_path = runtime_path.with_name(f'{runtime_path.name}_{stamp}_{suffix}')
            suffix += 1
        shutil.move(str(runtime_path), str(archive_path))
        print(
            '[ugv_semantic_navigation] start_fresh archived '
            f'{runtime_path} -> {archive_path}'
        )
    runtime_path.mkdir(parents=True, exist_ok=True)
    return []


def _rosbridge(context):
    requested = int(LaunchConfiguration('rosbridge_port').perform(context))
    selected = requested
    for port in range(requested, requested + 20):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                sock.bind(('0.0.0.0', port))
                selected = port
                break
            except OSError:
                continue
    return [Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': selected}],
    )]


def generate_launch_description():
    ugv_nav = get_package_share_directory('ugv_nav')
    ugv_vision = get_package_share_directory('ugv_vision')
    runtime_dir_default = PathJoinSubstitution([
        EnvironmentVariable('HOME'), '.ros', 'ugv_semantic_navigation'
    ])

    declarations = [
        _arg('runtime_dir', runtime_dir_default, 'Semantic navigation runtime data directory.'),
        _arg('start_fresh', 'false', 'Archive existing runtime data before starting semantic nodes.'),
        _arg('rover_bringup', 'true', 'Launch the existing UGV SLAM and Nav2 stack.'),
        _arg('camera', 'true', 'Launch the existing OAK-D Lite driver.'),
        _arg('rviz', 'false', 'Launch the UGV SLAM RViz view.'),
        _arg('explore', 'true', 'Start autonomous biased frontier exploration.'),
        _arg('ldlidar_model', os.environ.get('LDLIDAR_MODEL', 'ld19'), 'UGV LiDAR model.'),
        _arg('image_topic', '/oak/rgb/image_rect', 'OAK-D RGB image topic.'),
        _arg('scan_topic', '/scan', 'LiDAR scan topic.'),
        _arg('odom_topic', '/odom_rf2o', 'Rover odometry topic.'),
        _arg('cmd_vel_topic', '/cmd_vel', 'Rover velocity command topic.'),
        _arg('voltage_topic', '/voltage', 'Rover battery voltage topic.'),
        _arg('min_voltage', '10.5', 'Stop semantic motion below this voltage; zero disables it.'),
        _arg('rover_base_frame', 'base_footprint', 'Rover base TF frame.'),
        _arg(
            'semantic_store',
            PathJoinSubstitution([LaunchConfiguration('runtime_dir'), 'objects.json']),
            'JSON memory path.',
        ),
        _arg(
            'semantic_vector_store',
            PathJoinSubstitution([LaunchConfiguration('runtime_dir'), 'semantic_memory.sqlite3']),
            'SQLite memory path.',
        ),
        _arg(
            'semantic_image_dir',
            PathJoinSubstitution([LaunchConfiguration('runtime_dir'), 'semantic_images']),
            'Captured image directory.',
        ),
        _arg('vision_model', 'gpt-4.1-mini', 'Vision model used when OPENAI_API_KEY is set.'),
        _arg('embedding_model', 'text-embedding-3-small', 'Embedding model name.'),
        _arg('capture_angles', '4', 'Views captured at each reached frontier.'),
        _arg(
            'capture_radius',
            '1.0',
            'Radius in meters used to decide if an area already has enough images.',
        ),
        _arg(
            'area_captured_min_views',
            '3',
            'Distinct nearby yaw buckets required before an area is considered covered.',
        ),
        _arg(
            'max_similar_hamming',
            '6',
            'Average-hash distance threshold for visually similar image suppression.',
        ),
        _arg('capture_timeout', '150.0', 'Maximum seconds for one camera sweep.'),
        _arg(
            'extra_exploration',
            'false',
            'After frontiers are exhausted, visit under-covered semantic coverage goals.',
        ),
        _arg(
            'extra_exploration_goals',
            '3',
            'Maximum successful extra coverage goals when extra_exploration is true.',
        ),
        _arg(
            'extra_exploration_goal_spacing',
            '0.8',
            'Free-space sampling spacing in meters for extra coverage goals.',
        ),
        _arg('duration', '0', 'Explorer duration; zero means unlimited.'),
        _arg('min_frontier_size', '0.2', 'Minimum frontier cluster size in meters.'),
        _arg('min_goal_distance', '0.75', 'Ignore frontier goals closer than this distance.'),
        _arg('frontier_goal_timeout', '90.0', 'Seconds before blacklisting a frontier goal.'),
        _arg('frontier_arrival_tolerance', '0.45', 'Report near-arrival while waiting for Nav2 success.'),
        _arg('frontier_wall_clearance', '0.18', 'Minimum occupied-cell clearance.'),
        _arg('rosbridge_port', '9090', 'Preferred ROS-MCP rosbridge websocket port.'),
    ]

    rover = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ugv_nav, 'launch', 'slam_nav.launch.py')),
        launch_arguments={
            'use_rviz': LaunchConfiguration('rviz'),
            'ldlidar_model': LaunchConfiguration('ldlidar_model'),
            'use_sim_time': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('rover_bringup')),
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ugv_vision, 'launch', 'oak_d_lite.launch.py')),
        condition=IfCondition(LaunchConfiguration('camera')),
    )

    memory = Node(
        package='ugv_semantic_navigation', executable='semantic_memory_node', output='screen',
        arguments=['--store', LaunchConfiguration('semantic_store'),
                   '--vector-store', LaunchConfiguration('semantic_vector_store'),
                   '--embedding-model', LaunchConfiguration('embedding_model'),
                   '--base-frame', LaunchConfiguration('rover_base_frame')])
    semantic_nav = Node(
        package='ugv_semantic_navigation', executable='semantic_nav_node', output='screen',
        arguments=['--store', LaunchConfiguration('semantic_store'),
                   '--vector-store', LaunchConfiguration('semantic_vector_store'),
                   '--vision-model', LaunchConfiguration('vision_model'),
                   '--image-topic', LaunchConfiguration('image_topic'),
                   '--base-frame', LaunchConfiguration('rover_base_frame'),
                   '--voltage-topic', LaunchConfiguration('voltage_topic'),
                   '--min-voltage', LaunchConfiguration('min_voltage')])
    observer = Node(
        package='ugv_semantic_navigation', executable='semantic_camera_observer', output='screen',
        arguments=['--vector-store', LaunchConfiguration('semantic_vector_store'),
                   '--image-dir', LaunchConfiguration('semantic_image_dir'),
                   '--vision-model', LaunchConfiguration('vision_model'),
                   '--embedding-model', LaunchConfiguration('embedding_model'),
                   '--image-topic', LaunchConfiguration('image_topic'),
                   '--scan-topic', LaunchConfiguration('scan_topic'),
                   '--cmd-vel-topic', LaunchConfiguration('cmd_vel_topic'),
                   '--voltage-topic', LaunchConfiguration('voltage_topic'),
                   '--min-voltage', LaunchConfiguration('min_voltage'),
                   '--base-frame', LaunchConfiguration('rover_base_frame'),
                   '--capture-angles', LaunchConfiguration('capture_angles'),
                   '--capture-radius', LaunchConfiguration('capture_radius'),
                   '--area-captured-min-views', LaunchConfiguration('area_captured_min_views'),
                   '--max-similar-hamming', LaunchConfiguration('max_similar_hamming'),
                   '--capture-timeout', LaunchConfiguration('capture_timeout'),
                   '--store-image-only-captures', 'true', '--landmark-fallback', 'false'])
    explorer = Node(
        package='ugv_semantic_navigation', executable='biased_frontier_explorer', output='screen',
        arguments=['--duration', LaunchConfiguration('duration'),
                   '--min-frontier-size', LaunchConfiguration('min_frontier_size'),
                   '--min-goal-distance', LaunchConfiguration('min_goal_distance'),
                   '--goal-timeout', LaunchConfiguration('frontier_goal_timeout'),
                   '--arrival-tolerance', LaunchConfiguration('frontier_arrival_tolerance'),
                   '--wall-clearance', LaunchConfiguration('frontier_wall_clearance'),
                   '--semantic-capture-timeout', LaunchConfiguration('capture_timeout'),
                   '--semantic-vector-store', LaunchConfiguration('semantic_vector_store'),
                   '--extra-exploration', LaunchConfiguration('extra_exploration'),
                   '--extra-exploration-goals', LaunchConfiguration('extra_exploration_goals'),
                   '--extra-exploration-goal-spacing', LaunchConfiguration('extra_exploration_goal_spacing'),
                   '--extra-exploration-radius', LaunchConfiguration('capture_radius'),
                   '--extra-exploration-min-views', LaunchConfiguration('area_captured_min_views'),
                   '--odom-topic', LaunchConfiguration('odom_topic'),
                   '--cmd-vel-topic', LaunchConfiguration('cmd_vel_topic'),
                   '--voltage-topic', LaunchConfiguration('voltage_topic'),
                   '--min-voltage', LaunchConfiguration('min_voltage'),
                   '--base-frame', LaunchConfiguration('rover_base_frame')],
        condition=IfCondition(LaunchConfiguration('explore')))

    return LaunchDescription([
        *declarations,
        OpaqueFunction(function=_start_fresh),
        rover,
        camera,
        OpaqueFunction(function=_rosbridge),
        Node(package='rosapi', executable='rosapi_node', name='rosapi', output='screen'),
        TimerAction(period=5.0, actions=[memory]),
        TimerAction(period=7.0, actions=[semantic_nav]),
        TimerAction(period=9.0, actions=[observer]),
        TimerAction(period=15.0, actions=[explorer]),
    ])
