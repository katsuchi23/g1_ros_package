import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
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
        default_value=os.path.join(nav2_bringup_dir, 'maps', 'rrc.yaml'),
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
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration("map")
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    
    # bring up launch
    bring_up_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'map': map_yaml,
                'params_file': params_file,
            }.items())
    

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
    
    # Add all nodes and launch files
    ld.add_action(bring_up_cmd)
    ld.add_action(rviz_node)

    return ld