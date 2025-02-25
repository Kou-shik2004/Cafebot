# cafe_robot/launch/mapping.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('cafebot')
    
    # Launch file parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_config_file = LaunchConfiguration('slam_config_file',
                                           default=os.path.join(pkg_dir, 'config', 'slam.yaml'))
    rviz_config_file = LaunchConfiguration('rviz_config_file',
                                           default=os.path.join(pkg_dir, 'config', 'mapping.rviz'))
    
    # Include robot launch file (should include robot state publisher, etc.)
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'slam_config_file',
            default_value=slam_config_file,
            description='Path to SLAM configuration file'
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=rviz_config_file,
            description='Path to RViz configuration file'
        ),
        
        # Nodes to launch
        robot_launch,
        slam_toolbox_node,
        rviz_node
    ])