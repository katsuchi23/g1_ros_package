import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

''' 
Note:
- When the launch file gets launched, the file gets launched from the ros2_ws/install/share folder instead of the ros2_ws/src folder. Hence the os.getcwd() will also return /home/delta/ros2_ws/

'''

def generate_launch_description():
    # Paths to dependencies
    nav2_bringup_dir = get_package_share_directory('g1_navigation')

    # Declare and launch configuration
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav2_bringup_dir, 'maps', 'rrc2.yaml'),
        description='Full path to map yaml file to load')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav2_bringup_dir, 'config', 'params.yaml'), # for this we can still use install dir since params folder also get inside. But, rviz and maps need to use absolute src path
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_enable_ipc_bridge_cmd = DeclareLaunchArgument(
        'enable_ipc_bridge',
        default_value='true',
        description='Run ROS->ZMQ IPC bridge for pose and costmaps')

    declare_enable_semnav_bridge_nodes_cmd = DeclareLaunchArgument(
        'enable_semnav_bridge_nodes',
        default_value='true',
        description='Run additional semantic navigation bridge/helper nodes')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration("map")
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    enable_ipc_bridge = LaunchConfiguration('enable_ipc_bridge')
    enable_semnav_bridge_nodes = LaunchConfiguration('enable_semnav_bridge_nodes')
    
    # bring up launch
    bring_up_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'map': map_yaml,
                'params_file': params_file,
            }.items())
    
    laserscan_filter_node = Node(
        package='g1_navigation',
        executable='laserscan_filter.py',
        name='laserscan_filter',
        output='screen',
    )

    ros_ipc_bridge_node = Node(
        package='g1_navigation',
        executable='ros_ipc_bridge.py',
        name='ros_ipc_bridge',
        output='screen',
        condition=IfCondition(enable_ipc_bridge),
    )

    nav2_sender_node = Node(
        package='g1_navigation',
        executable='nav2_sender_node.py',
        name='nav2_sender',
        output='screen',
        condition=IfCondition(enable_semnav_bridge_nodes),
    )

    image_sender_node = Node(
        package='g1_navigation',
        executable='image_sender_node.py',
        name='image_sender_node',
        output='screen',
        condition=IfCondition(enable_semnav_bridge_nodes),
    )

    depth_sender_node = Node(
        package='g1_navigation',
        executable='depth_sender_node.py',
        name='depth_sender_node',
        output='screen',
        condition=IfCondition(enable_semnav_bridge_nodes),
    )

    laserscan_sender_node = Node(
        package='g1_navigation',
        executable='laserscan_sender_node.py',
        name='laserscan_sender',
        output='screen',
        condition=IfCondition(enable_semnav_bridge_nodes),
    )

    trajectory_visualization_node = Node(
        package='g1_navigation',
        executable='trajectory_visualization_node.py',
        name='trajectory_visualization_node',
        output='screen',
        condition=IfCondition(enable_semnav_bridge_nodes),
    )

    nav_through_poses_bridge_node = Node(
        package='g1_navigation',
        executable='nav_through_poses_bridge.py',
        name='nav_through_poses_bridge',
        output='screen',
        condition=IfCondition(enable_semnav_bridge_nodes),
    )

    semnav_rviz_node = Node(
        package='g1_navigation',
        executable='semnav_rviz_node.py',
        name='semnav_rviz_node',
        output='screen',
        condition=IfCondition(enable_semnav_bridge_nodes),
    )

    # Rviz Node
    rviz_config_file_path = os.path.join(nav2_bringup_dir, 'rviz', 'view.rviz')
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d", rviz_config_file_path,
            "--ros-args",
            "--log-level", "WARN"
        ]
    )

    ld = LaunchDescription()

    # Add Declaration
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_enable_ipc_bridge_cmd)
    ld.add_action(declare_enable_semnav_bridge_nodes_cmd)
    
    # Add all nodes and launch files
    ld.add_action(bring_up_cmd)
    ld.add_action(laserscan_filter_node)
    ld.add_action(ros_ipc_bridge_node)
    ld.add_action(nav2_sender_node)
    ld.add_action(image_sender_node)
    ld.add_action(depth_sender_node)
    ld.add_action(laserscan_sender_node)
    ld.add_action(trajectory_visualization_node)
    ld.add_action(nav_through_poses_bridge_node)
    ld.add_action(semnav_rviz_node)
    ld.add_action(rviz_node)

    return ld
