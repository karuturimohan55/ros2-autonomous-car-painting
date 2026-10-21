#!/usr/bin/env python3
"""
Launch file for Yuktii AI Labs - Robotic Car Painting Cell Simulation (Stage 2).
Launches Gazebo Classic with the painting cell world and spawns the mobile painting robot.
"""

import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'car_painting_cell'

    try:
        pkg_share = get_package_share_directory(pkg_name)
    except PackageNotFoundError:
        pkg_share = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..')
        )

    # Robot Xacro file path
    xacro_file = os.path.join(pkg_share, 'urdf', 'mobile_painting_robot.urdf.xacro')

    # Default spawn coordinates: On cyan navigation corridor, left flank of car
    # Standoff ~1.5m from car side (car left side at y=0.95m, robot at y=2.45m)
    # Yaw = -1.5708 rad (-90 deg), facing the vehicle body
    default_x = '0.0'
    default_y = '2.45'
    default_z = '0.05'
    default_yaw = '-1.570796'

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

    x_arg = DeclareLaunchArgument(
        'x', default_value=default_x,
        description='X spawn coordinate for the mobile painting robot.'
    )
    y_arg = DeclareLaunchArgument(
        'y', default_value=default_y,
        description='Y spawn coordinate for the mobile painting robot.'
    )
    z_arg = DeclareLaunchArgument(
        'z', default_value=default_z,
        description='Z spawn coordinate for the mobile painting robot.'
    )
    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value=default_yaw,
        description='Yaw orientation for the mobile painting robot (radians).'
    )

    # 1. Include the environment-only world launch
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'cell_world.launch.py')
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'verbose': LaunchConfiguration('verbose'),
        }.items()
    )

    # 2. Robot State Publisher (processes xacro and publishes robot_description + TF)
    from launch_ros.parameter_descriptions import ParameterValue
    robot_description_content = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # 3. Spawn Entity Node (spawns URDF into Gazebo simulation)
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mobile_painting_robot',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
            '-timeout', '60.0'
        ]
    )

    # 4. Arm Controller Node (active joint trajectory interpolation and hold)
    arm_controller_node = Node(
        package='car_painting_cell',
        executable='arm_controller.py',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gui_arg,
        verbose_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        world_launch,
        robot_state_publisher_node,
        spawn_entity_node,
        arm_controller_node,
    ])
