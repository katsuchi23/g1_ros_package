from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import math
import yaml

import os


def create_body_to_base_link_tf(context):
    config_dir = LaunchConfiguration("config_path").perform(context)
    config_name = LaunchConfiguration("config_file").perform(context)
    config_file_path = os.path.join(config_dir, config_name)

    with open(config_file_path, "r") as file:
        yaml_config = yaml.safe_load(file)

    publish_cfg = yaml_config["/**"]["ros__parameters"]["publish"]
    odom_roll = publish_cfg["odom_roll"]
    odom_pitch = publish_cfg["odom_pitch"]
    odom_yaw = publish_cfg["odom_yaw"]

    # body->base_link is the inverse of odom->camera_init
    roll = -odom_roll * math.pi / 180.0
    pitch = odom_pitch * math.pi / 180.0
    yaw = -odom_yaw * math.pi / 180.0

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="body_to_base_link_broadcaster",
            arguments=[
                "0", "0", "0",
                str(qx), str(qy), str(qz), str(qw),
                "body",
                "base_link",
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    package_path = get_package_share_directory("fast_lio_localization")
    default_config_path = os.path.join(package_path, "config")
    default_rviz_config_path = os.path.join(package_path, "rviz", "fastlio_localization.rviz")
    default_map_path = "/home/ntu/reynaldy_ws/unitree_g1_control/ros2_ws/src/mapping/FAST_LIO_ROS2/PCD/rrc2.pcd"

    use_sim_time = LaunchConfiguration("use_sim_time")
    config_path = LaunchConfiguration("config_path")
    config_file = LaunchConfiguration("config_file")
    rviz_use = LaunchConfiguration("rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")
    pcd_map_topic = LaunchConfiguration("pcd_map_topic")
    pcd_map_path = LaunchConfiguration("map")

    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        "config_path", default_value=default_config_path, description="Yaml config file path"
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        "config_file", default_value="mid360.yaml", description="Config file"
    )
    declare_rviz_cmd = DeclareLaunchArgument("rviz", default_value="true", description="Use RViz to monitor results")

    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        "rviz_cfg", default_value=default_rviz_config_path, description="RViz config file path"
    )

    declare_map_path = DeclareLaunchArgument("map", default_value=default_map_path, description="Path to PCD map file")
    declare_pcd_map_topic = DeclareLaunchArgument(
        "pcd_map_topic", default_value="/map", description="Topic to publish PCD map"
    )
    # Load parameters from yaml file

    fast_lio_node = Node(
        package="fast_lio_localization",
        executable="fastlio_mapping",
        parameters=[PathJoinSubstitution([config_path, config_file]), {"use_sim_time": use_sim_time}],
        output="screen",
    )
    # Global localization node
    global_localization_node = Node(
        package="fast_lio_localization",
        executable="global_localization.py",
        name="global_localization",
        output="screen",
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {"map_voxel_size": 0.4,
                     "scan_voxel_size": 0.1,
                     "freq_localization": 0.5,
                     "freq_global_map": 0.25,
                     "localization_threshold": 0.8,
                     "fov": 6.28319,
                     "fov_far": 300,
                     "pcd_map_path": pcd_map_path,
                     "pcd_map_topic": pcd_map_topic}],
    )

    # Transform fusion node
    transform_fusion_node = Node(
        package="fast_lio_localization",
        executable="transform_fusion.py",
        name="transform_fusion",
        output="screen",
        parameters=[PathJoinSubstitution([config_path, config_file])],
    )
    
    # PCD to PointCloud2 publisher
    pcd_publisher_node = Node(
        package="pcl_ros",
        executable="pcd_to_pointcloud",
        name="map_publisher",
        output="screen",
        parameters=[{"file_name": pcd_map_path,
                     "tf_frame": "map",
                    "cloud_topic": pcd_map_topic,
                    "period_ms_": 500}],
        remappings=[
            ("cloud_pcd", pcd_map_topic),
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d", rviz_cfg,
            "--ros-args",
            "--log-level", "WARN"
        ],
        condition=IfCondition(rviz_use),
    )

    odom_converter_node = Node(
        package="fast_lio_localization",
        executable="odom_topic.py",
        name="fastlio_odom_converter",
        output="screen",
        parameters=[PathJoinSubstitution([config_path, config_file])],
    )
    body_to_base_link_tf = OpaqueFunction(function=create_body_to_base_link_tf)

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_map_path)
    ld.add_action(declare_pcd_map_topic)

    ld.add_action(fast_lio_node)
    # ld.add_action(rviz_node)
    ld.add_action(global_localization_node)
    ld.add_action(transform_fusion_node)
    ld.add_action(pcd_publisher_node)
    ld.add_action(body_to_base_link_tf)
    ld.add_action(odom_converter_node)


    return ld
