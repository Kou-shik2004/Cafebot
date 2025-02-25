#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
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

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
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

    # Define the joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # Define the joint state publisher GUI node (optional)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # Define the RViz2 node
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'cafebot.rviz')
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
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])