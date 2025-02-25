#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get directories
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    prefix_address = get_package_share_directory('cafebot')
    
    # Set default paths
    default_map_path = os.path.join(
        get_package_share_directory('cafebot'),
        'final_map.yaml'
    )
    
    param_file_name = 'nav2_params.yaml'
    default_params_path = os.path.join(
        get_package_share_directory('cafebot'),
        'config',
        param_file_name
    )
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_dir = LaunchConfiguration('map', default=default_map_path)
    param_dir = LaunchConfiguration('params_file', default=default_params_path)
    
    # Include the Nav2 bringup launch file
    navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_dir, '/bringup_launch.py']),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'params_file': param_dir
        }.items(),
    )
    
    # Return the LaunchDescription
    return LaunchDescription([
        navigation_launch_cmd,
    ])