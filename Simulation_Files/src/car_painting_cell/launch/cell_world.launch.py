#!/usr/bin/env python3
"""
Launch file for Yuktii AI Labs - Robotic Car Painting Cell Simulation (Stage 1).
Launches Gazebo Classic with the industrial painting cell world and stationary car model.
"""

import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_name = 'car_painting_cell'

    # Support running both from installed share directory and directly from source workspace
    try:
        pkg_share = get_package_share_directory(pkg_name)
    except PackageNotFoundError:
        # Fallback to source directory
        pkg_share = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..')
        )

    # Resolve world and models paths
    world_path = os.path.join(pkg_share, 'worlds', 'car_painting_cell.world')
    models_path = os.path.join(pkg_share, 'models')

    # Gazebo system model path fallback
    system_models = '/usr/share/gazebo-11/models'
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')

    new_model_path = f"{models_path}:{system_models}"
    if existing_model_path:
        new_model_path = f"{new_model_path}:{existing_model_path}"

    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # Declare Launch Arguments
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "true" to run with Gazebo GUI (gzclient), "false" for headless.'
    )

    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='true',
        description='Set to "true" to run Gazebo with verbose terminal output.'
    )

    paused_arg = DeclareLaunchArgument(
        'paused',
        default_value='false',
        description='Set to "true" to start Gazebo simulation paused.'
    )

    # Set GAZEBO_MODEL_PATH environment variable for spawned subprocesses
    set_model_path_env = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=new_model_path
    )

    # Include Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': LaunchConfiguration('verbose'),
            'gui': LaunchConfiguration('gui'),
            'pause': LaunchConfiguration('paused'),
        }.items()
    )

    return LaunchDescription([
        gui_arg,
        verbose_arg,
        paused_arg,
        set_model_path_env,
        gazebo_launch,
    ])
