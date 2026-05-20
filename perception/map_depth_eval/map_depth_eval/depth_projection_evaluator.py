from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import tf2_ros
from tf_transformations import quaternion_from_euler, quaternion_matrix


@dataclass
class CameraModel:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    frame_id: str


class DepthProjectionEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("depth_projection_evaluator")

        self.declare_parameter("pointcloud_topic", "/livox/lidar/pcd2")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw/compressed")
        self.declare_parameter("use_compressed_image", True)
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("depth_topic", "/map_depth/projected_depth")
        self.declare_parameter("overlay_topic", "/map_depth/projected_overlay")
        self.declare_parameter("backprojected_cloud_topic", "/map_depth/backprojected_cloud")
        self.declare_parameter("visible_cloud_topic", "/map_depth/visible_map_points")
        self.declare_parameter("min_depth_m", 0.1)
        self.declare_parameter("max_depth_m", 15.0)
        self.declare_parameter("use_latest_tf", False)
        self.declare_parameter("publish_visible_cloud", True)
        self.declare_parameter("publish_overlay", True)
        self.declare_parameter("publish_camera_mount_tf", True)
        self.declare_parameter("camera_parent_frame", "body2")
        self.declare_parameter("camera_link_frame", "camera_link")
        self.declare_parameter("camera_tx", 0.056744402376091595)
        self.declare_parameter("camera_ty", 0.0175)
        self.declare_parameter("camera_tz", 0.01598012825293377)
        self.declare_parameter("camera_roll", 0.0)
        self.declare_parameter("camera_pitch", 0.7906341511534315)
        self.declare_parameter("camera_yaw", 0.0)

        self.pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.image_topic = self.get_parameter("image_topic").value
        self.use_compressed_image = bool(self.get_parameter("use_compressed_image").value)
        self.camera_frame_override = self.get_parameter("camera_frame").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.overlay_topic = self.get_parameter("overlay_topic").value
        self.backprojected_cloud_topic = self.get_parameter("backprojected_cloud_topic").value
        self.visible_cloud_topic = self.get_parameter("visible_cloud_topic").value
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.use_latest_tf = bool(self.get_parameter("use_latest_tf").value)
        self.publish_visible_cloud = bool(self.get_parameter("publish_visible_cloud").value)
        self.publish_overlay = bool(self.get_parameter("publish_overlay").value)
        self.publish_camera_mount_tf = bool(self.get_parameter("publish_camera_mount_tf").value)
        self.camera_parent_frame = self.get_parameter("camera_parent_frame").value
        self.camera_link_frame = self.get_parameter("camera_link_frame").value
        self.camera_tx = float(self.get_parameter("camera_tx").value)
        self.camera_ty = float(self.get_parameter("camera_ty").value)
        self.camera_tz = float(self.get_parameter("camera_tz").value)
        self.camera_roll = float(self.get_parameter("camera_roll").value)
        self.camera_pitch = float(self.get_parameter("camera_pitch").value)
        self.camera_yaw = float(self.get_parameter("camera_yaw").value)

        self.bridge = CvBridge()
        self.latest_camera_model: Optional[CameraModel] = None
        self.map_points: Optional[np.ndarray] = None
        self.map_frame: Optional[str] = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        if self.publish_camera_mount_tf:
            self.publish_camera_mount_transform()

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        default_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(PointCloud2, self.pointcloud_topic, self.pointcloud_callback, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
        if self.use_compressed_image:
            self.create_subscription(
                CompressedImage, self.image_topic, self.compressed_image_callback, image_qos
            )
        else:
            self.create_subscription(Image, self.image_topic, self.image_callback, image_qos)

        self.depth_pub = self.create_publisher(Image, self.depth_topic, default_qos)
        self.overlay_pub = self.create_publisher(Image, self.overlay_topic, default_qos)
        self.backprojected_cloud_pub = self.create_publisher(PointCloud2, self.backprojected_cloud_topic, 10)
        self.visible_cloud_pub = self.create_publisher(PointCloud2, self.visible_cloud_topic, 10)

        self.xyz_fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

    def publish_camera_mount_transform(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.camera_parent_frame
        transform.child_frame_id = self.camera_link_frame
        transform.transform.translation.x = self.camera_tx
        transform.transform.translation.y = self.camera_ty
        transform.transform.translation.z = self.camera_tz

        qx, qy, qz, qw = quaternion_from_euler(
            self.camera_roll, self.camera_pitch, self.camera_yaw
        )
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.static_tf_broadcaster.sendTransform(transform)
        self.get_logger().info(
            f"Publishing static TF {self.camera_parent_frame} -> {self.camera_link_frame}"
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.latest_camera_model = CameraModel(
            width=msg.width,
            height=msg.height,
            fx=float(msg.k[0]),
            fy=float(msg.k[4]),
            cx=float(msg.k[2]),
            cy=float(msg.k[5]),
            frame_id=self.camera_frame_override or msg.header.frame_id,
        )

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        points = np.array(
            [
                (point[0], point[1], point[2])
                for point in point_cloud2.read_points(
                    msg, field_names=("x", "y", "z"), skip_nans=True
                )
            ],
            dtype=np.float32,
        )
        if points.size == 0:
            self.get_logger().warn("Received empty map cloud")
            return

        self.map_points = points
        self.map_frame = msg.header.frame_id

    def compressed_image_callback(self, msg: CompressedImage) -> None:
        try:
            image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Failed to decode compressed image: {exc}")
            return
        self.process_image(image, msg.header)

    def image_callback(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Failed to decode image: {exc}")
            return
        self.process_image(image, msg.header)

    def process_image(self, image: np.ndarray, header: Header) -> None:
        if self.latest_camera_model is None:
            self.get_logger().warn("Waiting for CameraInfo")
            return
        if self.map_points is None or self.map_frame is None:
            self.get_logger().warn("Waiting for map point cloud")
            return

        camera_frame = self.latest_camera_model.frame_id
        stamp = Time() if self.use_latest_tf else Time.from_msg(header.stamp)

        try:
            transform = self.tf_buffer.lookup_transform(camera_frame, self.map_frame, stamp)
        except Exception as exc:
            self.get_logger().warn(
                f"TF lookup failed from '{self.map_frame}' to '{camera_frame}': {exc}"
            )
            return

        t_camera_map = transform_to_matrix(transform)
        depth_image, visible_points_map, overlay = project_depth(
            points_map=self.map_points,
            t_camera_map=t_camera_map,
            camera=self.latest_camera_model,
            bgr_image=image,
            min_depth_m=self.min_depth_m,
            max_depth_m=self.max_depth_m,
            draw_overlay=self.publish_overlay,
        )

        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
        depth_msg.header = header
        self.depth_pub.publish(depth_msg)

        if self.publish_overlay:
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            overlay_msg.header = header
            self.overlay_pub.publish(overlay_msg)

        backprojected_points_camera = backproject_depth(depth_image, self.latest_camera_model)
        if backprojected_points_camera.shape[0] == 0:
            return

        t_map_camera = np.linalg.inv(t_camera_map)
        backprojected_points_map = transform_points(backprojected_points_camera, t_map_camera)
        self.backprojected_cloud_pub.publish(
            make_xyz_cloud(
                points=backprojected_points_map,
                frame_id=self.map_frame,
                stamp=header.stamp,
                fields=self.xyz_fields,
            )
        )

        if self.publish_visible_cloud and visible_points_map.shape[0] > 0:
            self.visible_cloud_pub.publish(
                make_xyz_cloud(
                    points=visible_points_map,
                    frame_id=self.map_frame,
                    stamp=header.stamp,
                    fields=self.xyz_fields,
                )
            )


def transform_to_matrix(msg: TransformStamped) -> np.ndarray:
    q = msg.transform.rotation
    t = msg.transform.translation
    matrix = quaternion_matrix([q.x, q.y, q.z, q.w]).astype(np.float32)
    matrix[0, 3] = t.x
    matrix[1, 3] = t.y
    matrix[2, 3] = t.z
    return matrix


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    homogeneous = np.hstack((points, np.ones((points.shape[0], 1), dtype=np.float32)))
    transformed = (transform @ homogeneous.T).T
    return transformed[:, :3]


def project_depth(
    points_map: np.ndarray,
    t_camera_map: np.ndarray,
    camera: CameraModel,
    bgr_image: np.ndarray,
    min_depth_m: float,
    max_depth_m: float,
    draw_overlay: bool,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    points_camera = transform_points(points_map, t_camera_map)
    x = points_camera[:, 0]
    y = points_camera[:, 1]
    z = points_camera[:, 2]

    valid = np.logical_and(z > min_depth_m, z < max_depth_m)
    if not np.any(valid):
        return (
            np.zeros((camera.height, camera.width), dtype=np.float32),
            np.empty((0, 3), dtype=np.float32),
            bgr_image.copy(),
        )

    x = x[valid]
    y = y[valid]
    z = z[valid]
    points_map = points_map[valid]

    u = np.rint(camera.fx * x / z + camera.cx).astype(np.int32)
    v = np.rint(camera.fy * y / z + camera.cy).astype(np.int32)

    valid = (
        (u >= 0)
        & (u < camera.width)
        & (v >= 0)
        & (v < camera.height)
    )
    if not np.any(valid):
        return (
            np.zeros((camera.height, camera.width), dtype=np.float32),
            np.empty((0, 3), dtype=np.float32),
            bgr_image.copy(),
        )

    u = u[valid]
    v = v[valid]
    z = z[valid]
    points_map = points_map[valid]

    flat_indices = v * camera.width + u
    order = np.argsort(z)
    flat_sorted = flat_indices[order]
    first_occurrence = np.unique(flat_sorted, return_index=True)[1]
    keep = order[first_occurrence]

    depth_image = np.zeros((camera.height, camera.width), dtype=np.float32)
    depth_image[v[keep], u[keep]] = z[keep]

    overlay = bgr_image.copy()
    if draw_overlay:
        depth_range = max(max_depth_m - min_depth_m, 1e-6)
        for ui, vi, zi in zip(u[keep], v[keep], z[keep]):
            ratio = float(np.clip((zi - min_depth_m) / depth_range, 0.0, 1.0))
            color = (int(255.0 * ratio), int(255.0 * (1.0 - ratio)), 0)
            cv2.circle(overlay, (int(ui), int(vi)), 1, color, -1)

    return depth_image, points_map[keep], overlay


def backproject_depth(depth_image: np.ndarray, camera: CameraModel) -> np.ndarray:
    v_coords, u_coords = np.nonzero(depth_image > 0.0)
    if v_coords.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    z = depth_image[v_coords, u_coords]
    x = (u_coords.astype(np.float32) - camera.cx) * z / camera.fx
    y = (v_coords.astype(np.float32) - camera.cy) * z / camera.fy
    return np.column_stack((x, y, z)).astype(np.float32)


def make_xyz_cloud(
    points: np.ndarray,
    frame_id: str,
    stamp,
    fields,
) -> PointCloud2:
    header = Header(stamp=stamp, frame_id=frame_id)
    return point_cloud2.create_cloud(header, fields, points.tolist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthProjectionEvaluator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
