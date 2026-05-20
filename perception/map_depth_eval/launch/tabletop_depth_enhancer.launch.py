from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument(
            "input_depth_topic",
            default_value="/camera/camera/aligned_depth_to_color/image_raw",
        ),
        DeclareLaunchArgument("filtered_depth_topic", default_value="/tabletop_depth/filtered"),
        DeclareLaunchArgument(
            "enhanced_viz_topic", default_value="/tabletop_depth/enhanced_viz"
        ),
        DeclareLaunchArgument("height_mask_topic", default_value="/tabletop_depth/range_mask"),
        DeclareLaunchArgument("d_min", default_value="0.70"),
        DeclareLaunchArgument("d_max", default_value="1.00"),
        DeclareLaunchArgument("median_kernel", default_value="5"),
        DeclareLaunchArgument("bilateral_diameter", default_value="5"),
        DeclareLaunchArgument("bilateral_sigma_color", default_value="0.03"),
        DeclareLaunchArgument("bilateral_sigma_space", default_value="5.0"),
        DeclareLaunchArgument("temporal_window", default_value="3"),
        DeclareLaunchArgument("temporal_blend_alpha", default_value="0.4"),
        DeclareLaunchArgument("use_inverse_depth_viz", default_value="true"),
        DeclareLaunchArgument("publish_mask", default_value="true"),
    ]

    node = Node(
        package="map_depth_eval",
        executable="tabletop_depth_enhancer",
        name="tabletop_depth_enhancer",
        output="screen",
        parameters=[
            {
                "input_depth_topic": LaunchConfiguration("input_depth_topic"),
                "filtered_depth_topic": LaunchConfiguration("filtered_depth_topic"),
                "enhanced_viz_topic": LaunchConfiguration("enhanced_viz_topic"),
                "height_mask_topic": LaunchConfiguration("height_mask_topic"),
                "d_min": LaunchConfiguration("d_min"),
                "d_max": LaunchConfiguration("d_max"),
                "median_kernel": LaunchConfiguration("median_kernel"),
                "bilateral_diameter": LaunchConfiguration("bilateral_diameter"),
                "bilateral_sigma_color": LaunchConfiguration("bilateral_sigma_color"),
                "bilateral_sigma_space": LaunchConfiguration("bilateral_sigma_space"),
                "temporal_window": LaunchConfiguration("temporal_window"),
                "temporal_blend_alpha": LaunchConfiguration("temporal_blend_alpha"),
                "use_inverse_depth_viz": LaunchConfiguration("use_inverse_depth_viz"),
                "publish_mask": LaunchConfiguration("publish_mask"),
            }
        ],
    )

    return LaunchDescription(arguments + [node])
