from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    transport = LaunchConfiguration('transport')
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')

    return LaunchDescription([
        DeclareLaunchArgument('transport', default_value='streamable-http'),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='8766'),
        Node(
            package='ugv_navigation_mcp',
            executable='navigation_mcp_server',
            name='navigation_mcp_server',
            output='screen',
            arguments=[
                '--transport', transport,
                '--host', host,
                '--port', port,
            ],
        ),
    ])
