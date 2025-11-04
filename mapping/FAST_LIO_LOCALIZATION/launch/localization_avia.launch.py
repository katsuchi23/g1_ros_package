#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('fast_lio_localization').find('fast_lio_localization')
    
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to the PCD map file'
    )
    
    # Load YAML config
    config_file = PathJoinSubstitution([
        FindPackageShare('fast_lio_localization'),
        'config',
        'avia.yaml'
    ])
    
    # Main mapping node
    fastlio_mapping_node = Node(
        package='fast_lio_localization',
        executable='fastlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            config_file,
            {
                'feature_extract_enable': False,
                'point_filter_num': 1,
                'max_iteration': 3,
                'filter_size_surf': 0.5,
                'filter_size_map': 0.5,
                'cube_side_length': 1000.0,
                'runtime_pos_log_enable': False,
                'pcd_save_enable': False,
            }
        ]
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
            'frame_id': '/map',
        }],
        arguments=[LaunchConfiguration('map'), '5'],
        remappings=[('cloud_pcd', '/map')]
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
    
    return LaunchDescription([
        rviz_arg,
        map_arg,
        fastlio_mapping_node,
        global_localization_node,
        transform_fusion_node,
        map_publisher_node,
        rviz_node,
    ])
