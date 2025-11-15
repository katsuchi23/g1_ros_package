from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pc_to_baselink_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_convert.py',
        name='pc_body_to_base_link',
        output='screen'
    )

    laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[
            ('cloud_in', '/cloud_in_base_link'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': -1.0,
            'max_height': 0.5,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00872,   # 0.5 deg
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 100.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
    )

    return LaunchDescription([
        pc_to_baselink_node,
        laserscan_node
    ])
