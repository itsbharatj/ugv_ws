#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction


def launch_setup(context, *args, **kwargs):
    ldlidar_model = context.launch_configurations['ldlidar_model']
    supported_models = {'ld06', 'ld19', 'stl27l'}

    if ldlidar_model not in supported_models:
        raise ValueError(
            f"Unsupported LDLiDAR model '{ldlidar_model}'. "
            f"Use one of: {', '.join(sorted(supported_models))}"
        )

    ldlidar_launch_file = ldlidar_model + '.launch.py'
    launch_path = os.path.join(
        get_package_share_directory('ldlidar'),
        'launch',
        ldlidar_launch_file
    )

    if not os.path.exists(launch_path):
        raise FileNotFoundError(f"LDLiDAR launch file not found: {launch_path}")

    laser_bringup_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        launch_path)
    )

    return [laser_bringup_launch]


def generate_launch_description():
    default_model = os.environ.get('LDLIDAR_MODEL', 'ld19')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'ldlidar_model',
        default_value=default_model,
        description='LDLiDAR model: ld06, ld19, or stl27l'
    ))
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
