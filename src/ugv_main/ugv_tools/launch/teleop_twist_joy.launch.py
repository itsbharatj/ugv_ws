from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Create a node to read joystick input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
    )

    # Create a node to control the robot using joystick input
    joy_ctrl_node = Node(
        package='ugv_tools',
        executable='joy_ctrl',
        output='screen',
        parameters=[{
            'xspeed_limit': LaunchConfiguration('xspeed_limit'),
            'yspeed_limit': LaunchConfiguration('yspeed_limit'),
            'angular_speed_limit': LaunchConfiguration('angular_speed_limit'),
            'axis_linear_x': LaunchConfiguration('axis_linear_x'),
            'axis_linear_y': LaunchConfiguration('axis_linear_y'),
            'axis_angular': LaunchConfiguration('axis_angular'),
            'linear_gear_button': LaunchConfiguration('linear_gear_button'),
            'angular_gear_button': LaunchConfiguration('angular_gear_button'),
        }]
    )

    # Return the launch description
    return LaunchDescription([
        DeclareLaunchArgument('xspeed_limit', default_value='0.5'),
        DeclareLaunchArgument('yspeed_limit', default_value='0.5'),
        DeclareLaunchArgument('angular_speed_limit', default_value='1.0'),
        DeclareLaunchArgument('axis_linear_x', default_value='1'),
        DeclareLaunchArgument('axis_linear_y', default_value='0'),
        DeclareLaunchArgument('axis_angular', default_value='3'),
        DeclareLaunchArgument('linear_gear_button', default_value='9'),
        DeclareLaunchArgument('angular_gear_button', default_value='10'),
        joy_node,
        joy_ctrl_node
    ])
