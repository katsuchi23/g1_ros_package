#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('fast_lio_localization')
    
    # Default map path - try install directory first, then source directory
    default_map_path = os.path.join(pkg_dir, 'PCD', 'scans.pcd')
    
    # Default 2D map path (PGM)
    default_2d_map_path = os.path.join(pkg_dir, 'maps', 'mapping4.yaml')
    
    # Default rosbag path
    default_bag_path = os.path.join(
        os.path.dirname(pkg_dir),
        'fast_lio_localization',
        'rosbag2_2025_09_08-15_50_47'
    )
    
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Path to the PCD map file'
    )
    
    map_2d_arg = DeclareLaunchArgument(
        'map_2d',
        default_value=default_2d_map_path,
        description='Path to the 2D map YAML file'
    )
    
    use_2d_map_arg = DeclareLaunchArgument(
        'use_2d_map',
        default_value='true',
        description='Launch map_server to publish 2D occupancy grid map'
    )
    
    use_bag_arg = DeclareLaunchArgument(
        'use_bag',
        default_value='false',
        description='Play rosbag data'
    )
    
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=default_bag_path,
        description='Path to the rosbag file'
    )
    
    bag_rate_arg = DeclareLaunchArgument(
        'bag_rate',
        default_value='1.0',
        description='Playback rate for rosbag'
    )
    
    # Load YAML config
    config_file = PathJoinSubstitution([
        FindPackageShare('fast_lio_localization'),
        'config',
        'mid360.yaml'
    ])
    
    # Main mapping node
    fastlio_mapping_node = Node(
        package='fast_lio_localization',
        executable='fastlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[config_file]
    )
    
    # Global localization node
    global_localization_node = Node(
        package='fast_lio_localization',
        executable='global_localization.py',
        name='global_localization',
        output='screen'
    )
    
    # Transform fusion node
    transform_fusion_node = Node(
        package='fast_lio_localization',
        executable='transform_fusion.py',
        name='transform_fusion',
        output='screen'
    )
    
    # Map publisher (using pcl_ros pcd_to_pointcloud)
    map_publisher_node = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='map_publisher',
        output='screen',
        parameters=[{
            'file_name': LaunchConfiguration('map'),
            'frame_id': 'map',
            'publishing_rate': 5.0,
        }],
        remappings=[('cloud_pcd', '/map_pointcloud')]
    )
    
    # 2D Map server (publishes occupancy grid)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map_2d'),
            'frame_id': 'map'
        }],
        condition=IfCondition(LaunchConfiguration('use_2d_map'))
    )
    
    # Lifecycle manager for map_server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }],
        condition=IfCondition(LaunchConfiguration('use_2d_map'))
    )
    
    # RViz2
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('fast_lio_localization'),
        'rviz_cfg',
        'localization.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # Rosbag player
    bag_player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', 
             LaunchConfiguration('bag_path'),
             '--rate', LaunchConfiguration('bag_rate'),
             '--clock'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_bag'))
    )
    
    return LaunchDescription([
        rviz_arg,
        map_arg,
        # map_2d_arg,
        # use_2d_map_arg,
        use_bag_arg,
        bag_path_arg,
        bag_rate_arg,
        fastlio_mapping_node,
        global_localization_node,
        transform_fusion_node,
        map_publisher_node,
        # map_server_node,
        # lifecycle_manager_node,
        # rviz_node,
        bag_player,
    ])
