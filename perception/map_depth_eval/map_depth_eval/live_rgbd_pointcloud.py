from __future__ import annotations

from typing import Optional

import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import tf2_ros
from tf_transformations import quaternion_from_euler, quaternion_matrix


class LiveRgbdPointCloud(Node):
    def __init__(self) -> None:
        super().__init__("live_rgbd_pointcloud")

        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/aligned_depth_to_color/camera_info")
        self.declare_parameter("pointcloud_topic", "/camera/live_rgbd_cloud")
        self.declare_parameter("output_frame", "camera_link")
        self.declare_parameter("camera_optical_frame", "camera_color_optical_frame")
        self.declare_parameter("pixel_step", 4)
        self.declare_parameter("depth_min_m", 0.2)
        self.declare_parameter("depth_max_m", 2.5)
        self.declare_parameter("publish_camera_mount_tf", True)
        self.declare_parameter("camera_parent_frame", "body")
        self.declare_parameter("camera_link_frame", "camera_link")
        self.declare_parameter("camera_tx", 0.056744402376091595)
        self.declare_parameter("camera_ty", 0.0175)
        self.declare_parameter("camera_tz", 0.01598012825293377)
        self.declare_parameter("camera_roll", 0.0)
        self.declare_parameter("camera_pitch", 0.0)
        self.declare_parameter("camera_yaw", 0.0)
        self.declare_parameter("publish_realsense_internal_tf", True)

        self.depth_topic = self.get_parameter("depth_topic").value
        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self.output_frame = self.get_parameter("output_frame").value
        self.camera_optical_frame = self.get_parameter("camera_optical_frame").value
        self.pixel_step = int(self.get_parameter("pixel_step").value)
        self.depth_min_m = float(self.get_parameter("depth_min_m").value)
        self.depth_max_m = float(self.get_parameter("depth_max_m").value)
        self.publish_camera_mount_tf = bool(self.get_parameter("publish_camera_mount_tf").value)
        self.publish_realsense_internal_tf = bool(self.get_parameter("publish_realsense_internal_tf").value)

        self.bridge = CvBridge()
        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None
        self.latest_rgb: Optional[np.ndarray] = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.publish_static_transforms()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(Image, self.rgb_topic, self.rgb_callback, qos)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, qos)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)

        self.cloud_pub = self.create_publisher(PointCloud2, self.pointcloud_topic, 10)
        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]

        self.get_logger().info(f"Publishing live RGB-D cloud on {self.pointcloud_topic}")

    def make_tf(self, parent: str, child: str, xyz, quat) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = xyz[0]
        transform.transform.translation.y = xyz[1]
        transform.transform.translation.z = xyz[2]
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        return transform

    def publish_static_transforms(self) -> None:
        transforms = []
        if self.publish_camera_mount_tf:
            q_body = quaternion_from_euler(
                float(self.get_parameter("camera_roll").value),
                float(self.get_parameter("camera_pitch").value),
                float(self.get_parameter("camera_yaw").value),
            )
            transforms.append(
                self.make_tf(
                    self.get_parameter("camera_parent_frame").value,
                    self.get_parameter("camera_link_frame").value,
                    (
                        float(self.get_parameter("camera_tx").value),
                        float(self.get_parameter("camera_ty").value),
                        float(self.get_parameter("camera_tz").value),
                    ),
                    q_body,
                )
            )
        if self.publish_realsense_internal_tf:
            transforms.extend(
                [
                    self.make_tf(
                        "camera_link",
                        "camera_color_frame",
                        (-0.000256, 0.015003, 0.000005),
                        (0.004794, 0.002488, 0.000797, 0.999985),
                    ),
                    self.make_tf(
                        "camera_color_frame",
                        "camera_color_optical_frame",
                        (0.0, 0.0, 0.0),
                        (-0.5, 0.5, -0.5, 0.5),
                    ),
                ]
            )
        if transforms:
            self.static_tf_broadcaster.sendTransform(transforms)

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def rgb_callback(self, msg: Image) -> None:
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def depth_callback(self, msg: Image) -> None:
        if None in (self.fx, self.fy, self.cx, self.cy) or self.latest_rgb is None:
            return

        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if msg.encoding == "16UC1":
            depth = depth.astype(np.float32) * 0.001
        elif msg.encoding == "32FC1":
            depth = depth.astype(np.float32)
        else:
            self.get_logger().warn(f"Unsupported depth encoding: {msg.encoding}")
            return

        vv, uu = np.mgrid[0:depth.shape[0]:self.pixel_step, 0:depth.shape[1]:self.pixel_step]
        z = depth[vv, uu]
        valid = np.isfinite(z) & (z >= self.depth_min_m) & (z <= self.depth_max_m)
        if not np.any(valid):
            return

        u = uu[valid].astype(np.float32)
        v = vv[valid].astype(np.float32)
        z = z[valid].astype(np.float32)
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        points_optical = np.column_stack((x, y, z)).astype(np.float32)

        try:
            tf_msg = self.tf_buffer.lookup_transform(self.output_frame, self.camera_optical_frame, Time())
        except Exception as exc:
            self.get_logger().warn(f"TF lookup failed: {exc}")
            return

        transform = quaternion_matrix(
            [
                tf_msg.transform.rotation.x,
                tf_msg.transform.rotation.y,
                tf_msg.transform.rotation.z,
                tf_msg.transform.rotation.w,
            ]
        ).astype(np.float32)
        transform[0, 3] = tf_msg.transform.translation.x
        transform[1, 3] = tf_msg.transform.translation.y
        transform[2, 3] = tf_msg.transform.translation.z

        points_h = np.hstack((points_optical, np.ones((points_optical.shape[0], 1), dtype=np.float32)))
        points_out = (transform @ points_h.T).T[:, :3]

        rgb = self.latest_rgb[v.astype(np.int32), u.astype(np.int32), :]
        rgb_uint32 = (
            (rgb[:, 2].astype(np.uint32) << 16)
            | (rgb[:, 1].astype(np.uint32) << 8)
            | rgb[:, 0].astype(np.uint32)
        )
        cloud = np.zeros((points_out.shape[0], 4), dtype=np.float32)
        cloud[:, :3] = points_out
        cloud[:, 3] = rgb_uint32.view(np.float32)

        header = Header(stamp=msg.header.stamp, frame_id=self.output_frame)
        cloud_msg = point_cloud2.create_cloud(header, self.fields, cloud.tolist())
        try:
            self.cloud_pub.publish(cloud_msg)
        except Exception:
            return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LiveRgbdPointCloud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
