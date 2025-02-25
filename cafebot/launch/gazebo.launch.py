#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('cafebot')

    # Specify the path to the URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'cafebot.urdf')

    # Check if the file exists
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found at {urdf_file}")

    # Read the URDF file
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(pkg_dir, 'worlds', 'new_cafe.world'),
        description='Path to the world file')

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 
                         'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )

    # Define the robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Define a Gazebo node for spawning the robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'cafebot',
            '-topic', 'robot_description',
            '-x', '-3.0',
            '-y', '-2.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Define the RViz2 node
    rviz_config_file = os.path.join(pkg_dir, 'config', 'new.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Return the launch description
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_world_file_cmd,
        gazebo,
        #robot_state_publisher_node,
        spawn_entity,
        #rviz_node
    ])