from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    repo_root_guess = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..")
    )
    default_from_repo = os.path.join(
        repo_root_guess, "unitree_sdk2", "build", "bin", "g1_loco_client"
    )
    default_from_home_ws = os.path.join(
        os.environ.get('HOME', ''), 'reynaldy_ws', 'unitree_g1_control', 'unitree_sdk2', 'build', 'bin', 'g1_loco_client'
    )
    default_from_home = os.path.join(
        os.environ.get('HOME', ''), 'unitree_g1_control', 'unitree_sdk2', 'build', 'bin', 'g1_loco_client'
    )
    default_g1_loco_client_path = (
        default_from_repo
        if os.path.exists(default_from_repo)
        else default_from_home_ws
        if os.path.exists(default_from_home_ws)
        else default_from_home
    )

    # Declare launch arguments
    network_interface_arg = DeclareLaunchArgument(
        'network_interface',
        default_value='eno1',
        description='Network interface for G1 robot communication'
    )
    
    g1_loco_client_path_arg = DeclareLaunchArgument(
        'g1_loco_client_path',
        default_value=default_g1_loco_client_path,
        description='Path to g1_loco_client executable'
    )
    
    command_timeout_arg = DeclareLaunchArgument(
        'command_timeout',
        default_value='0.1',
        description='Minimum time between commands in seconds (safety debouncing)'
    )

    send_stop_on_zero_cmd_arg = DeclareLaunchArgument(
        'send_stop_on_zero_cmd',
        default_value='true',
        description='Send --stop_move when cmd_vel is exactly zero'
    )
    
    max_x_linear_velocity_arg = DeclareLaunchArgument(
        'max_x_linear_velocity',
        default_value='0.5',
        description='Maximum linear velocity (x, y) in m/s (safety limit)'
    )

    max_y_linear_velocity_arg = DeclareLaunchArgument(
        'max_y_linear_velocity',
        default_value='0.5',
        description='Maximum linear velocity (y) in m/s (safety limit)'
    )
    
    max_angular_velocity_arg = DeclareLaunchArgument(
        'max_angular_velocity',
        default_value='0.5',
        description='Maximum angular velocity (z) in rad/s (safety limit)'
    )
    
    # Create the node
    cmd_vel_to_g1_node = Node(
        package='g1_controller',
        executable='cmd_vel_to_g1',
        name='cmd_vel_to_g1',
        output='screen',
        parameters=[{
            'network_interface': LaunchConfiguration('network_interface'),
            'g1_loco_client_path': LaunchConfiguration('g1_loco_client_path'),
            'command_timeout': LaunchConfiguration('command_timeout'),
            'send_stop_on_zero_cmd': LaunchConfiguration('send_stop_on_zero_cmd'),
            'max_x_linear_velocity': LaunchConfiguration('max_x_linear_velocity'),
            'max_y_linear_velocity': LaunchConfiguration('max_y_linear_velocity'),
            'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
        }]
    )
    
    return LaunchDescription([
        network_interface_arg,
        g1_loco_client_path_arg,
        command_timeout_arg,
        send_stop_on_zero_cmd_arg,
        max_x_linear_velocity_arg,
        max_y_linear_velocity_arg,
        max_angular_velocity_arg,
        cmd_vel_to_g1_node
    ])
