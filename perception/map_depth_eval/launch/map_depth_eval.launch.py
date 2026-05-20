from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("pointcloud_topic", default_value="/livox/lidar/pcd2"),
        DeclareLaunchArgument(
            "camera_info_topic", default_value="/camera/camera/color/camera_info"
        ),
        DeclareLaunchArgument(
            "image_topic", default_value="/camera/camera/color/image_raw/compressed"
        ),
        DeclareLaunchArgument("use_compressed_image", default_value="true"),
        DeclareLaunchArgument("camera_frame", default_value="camera_color_optical_frame"),
        DeclareLaunchArgument("min_depth_m", default_value="0.1"),
        DeclareLaunchArgument("max_depth_m", default_value="15.0"),
        DeclareLaunchArgument("publish_camera_mount_tf", default_value="true"),
        DeclareLaunchArgument("camera_parent_frame", default_value="body2"),
        DeclareLaunchArgument("camera_link_frame", default_value="camera_link"),
        DeclareLaunchArgument("camera_tx", default_value="0.056744402376091595"),
        DeclareLaunchArgument("camera_ty", default_value="0.0175"),
        DeclareLaunchArgument("camera_tz", default_value="0.01598012825293377"),
        DeclareLaunchArgument("camera_roll", default_value="0.0"),
        DeclareLaunchArgument("camera_pitch", default_value="0.7906341511534315"),
        DeclareLaunchArgument("camera_yaw", default_value="0.0"),
    ]

    evaluator = Node(
        package="map_depth_eval",
        executable="depth_projection_evaluator",
        name="depth_projection_evaluator",
        output="screen",
        parameters=[
            {
                "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "image_topic": LaunchConfiguration("image_topic"),
                "use_compressed_image": LaunchConfiguration("use_compressed_image"),
                "camera_frame": LaunchConfiguration("camera_frame"),
                "min_depth_m": LaunchConfiguration("min_depth_m"),
                "max_depth_m": LaunchConfiguration("max_depth_m"),
                "publish_camera_mount_tf": LaunchConfiguration("publish_camera_mount_tf"),
                "camera_parent_frame": LaunchConfiguration("camera_parent_frame"),
                "camera_link_frame": LaunchConfiguration("camera_link_frame"),
                "camera_tx": LaunchConfiguration("camera_tx"),
                "camera_ty": LaunchConfiguration("camera_ty"),
                "camera_tz": LaunchConfiguration("camera_tz"),
                "camera_roll": LaunchConfiguration("camera_roll"),
                "camera_pitch": LaunchConfiguration("camera_pitch"),
                "camera_yaw": LaunchConfiguration("camera_yaw"),
            }
        ],
    )
    return LaunchDescription(arguments + [evaluator])
