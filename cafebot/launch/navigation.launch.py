#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get directories
    pkg_dir = get_package_share_directory('cafebot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Specify the path to the URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'cafebot.urdf')
    
    # Check if the file exists
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found at {urdf_file}")
        
    # Read the URDF file
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')
    map_file = LaunchConfiguration('map_file')
    params_file = LaunchConfiguration('params_file')
    
    # Set default paths
    default_map_path = os.path.join(pkg_dir, 'maps', 'cafe_map.yaml')
    
    nav_params_path = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )
    
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(pkg_dir, 'worlds', 'new_cafe.world'),
        description='Path to the world file'
    )
    
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_path,
        description='Full path to map file to load'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav_params_path,
        description='Full path to the ROS2 parameters file for navigation'
    )
    
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
    
    # Map Server Node
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_file}
        ]
    )
    
    # Map Server Lifecycle Manager
    map_lifecycle_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )
    
    # Navigation Stack
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )
    
    # AMCL Localization
    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': params_file
        }.items()
    )
    
    # Return the LaunchDescription - Order is important for proper initialization
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time_cmd,
        declare_world_file_cmd,
        declare_map_file_cmd,
        declare_params_file_cmd,
        
        # Simulation environment
        gazebo,
        
        # Robot description and visualization
        robot_state_publisher_node,
        spawn_entity,
        
        # Map components
        map_server_cmd,
        map_lifecycle_cmd,
        
        # Navigation components
        localization_cmd,
        navigation_cmd,
        
        # Visualization
        rviz_node
    ])