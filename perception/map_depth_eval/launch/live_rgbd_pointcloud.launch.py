from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument('depth_topic', default_value='/camera/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/camera/color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera/aligned_depth_to_color/camera_info'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/camera/live_rgbd_cloud'),
        DeclareLaunchArgument('output_frame', default_value='camera_link'),
        DeclareLaunchArgument('camera_optical_frame', default_value='camera_color_optical_frame'),
        DeclareLaunchArgument('pixel_step', default_value='4'),
        DeclareLaunchArgument('depth_min_m', default_value='0.2'),
        DeclareLaunchArgument('depth_max_m', default_value='2.5'),
        DeclareLaunchArgument('publish_camera_mount_tf', default_value='true'),
        DeclareLaunchArgument('camera_parent_frame', default_value='body'),
        DeclareLaunchArgument('camera_link_frame', default_value='camera_link'),
        DeclareLaunchArgument('camera_tx', default_value='0.056744402376091595'),
        DeclareLaunchArgument('camera_ty', default_value='0.0175'),
        DeclareLaunchArgument('camera_tz', default_value='0.01598012825293377'),
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='-0.7906341511534315'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),
        DeclareLaunchArgument('publish_realsense_internal_tf', default_value='true'),
    ]

    node = Node(
        package='map_depth_eval',
        executable='live_rgbd_pointcloud',
        name='live_rgbd_pointcloud',
        output='screen',
        parameters=[{
            'depth_topic': LaunchConfiguration('depth_topic'),
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'output_frame': LaunchConfiguration('output_frame'),
            'camera_optical_frame': LaunchConfiguration('camera_optical_frame'),
            'pixel_step': LaunchConfiguration('pixel_step'),
            'depth_min_m': LaunchConfiguration('depth_min_m'),
            'depth_max_m': LaunchConfiguration('depth_max_m'),
            'publish_camera_mount_tf': LaunchConfiguration('publish_camera_mount_tf'),
            'camera_parent_frame': LaunchConfiguration('camera_parent_frame'),
            'camera_link_frame': LaunchConfiguration('camera_link_frame'),
            'camera_tx': LaunchConfiguration('camera_tx'),
            'camera_ty': LaunchConfiguration('camera_ty'),
            'camera_tz': LaunchConfiguration('camera_tz'),
            'camera_roll': LaunchConfiguration('camera_roll'),
            'camera_pitch': LaunchConfiguration('camera_pitch'),
            'camera_yaw': LaunchConfiguration('camera_yaw'),
            'publish_realsense_internal_tf': LaunchConfiguration('publish_realsense_internal_tf'),
        }],
    )

    return LaunchDescription(arguments + [node])
