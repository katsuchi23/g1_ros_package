from __future__ import annotations

from typing import Optional

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2

from .ply_loader import load_ply_vertices


class MapCloudPublisher(Node):
    def __init__(self) -> None:
        super().__init__("map_cloud_publisher")

        self.declare_parameter("ply_path", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("pointcloud_topic", "/map_cloud")
        self.declare_parameter("publish_period_s", 1.0)
        self.declare_parameter("point_stride", 1)

        self.frame_id = self.get_parameter("frame_id").value
        self.pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self.publish_period_s = float(self.get_parameter("publish_period_s").value)
        self.point_stride = max(1, int(self.get_parameter("point_stride").value))

        ply_path = self.get_parameter("ply_path").value
        if not ply_path:
            raise ValueError("Parameter 'ply_path' must be set")

        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(PointCloud2, self.pointcloud_topic, qos)

        xyz, rgb = load_ply_vertices(ply_path)
        self.cloud_points = self._pack_points(xyz, rgb)[:: self.point_stride]
        self.get_logger().info(
            f"Loaded {self.cloud_points.shape[0]} points from {ply_path} into frame '{self.frame_id}'"
        )

        self.pointcloud_msg: Optional[PointCloud2] = None
        self.publish_cloud()
        self.timer = self.create_timer(self.publish_period_s, self.publish_cloud)

    def publish_cloud(self) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        self.pointcloud_msg = point_cloud2.create_cloud(header, self.fields, self.cloud_points.tolist())
        self.publisher.publish(self.pointcloud_msg)

    def _pack_points(self, xyz: np.ndarray, rgb: np.ndarray | None) -> np.ndarray:
        packed = np.zeros((xyz.shape[0], 4), dtype=np.float32)
        packed[:, :3] = xyz

        if rgb is None:
            rgb = np.full((xyz.shape[0], 3), 255, dtype=np.uint8)

        rgb_uint32 = (
            (rgb[:, 0].astype(np.uint32) << 16)
            | (rgb[:, 1].astype(np.uint32) << 8)
            | rgb[:, 2].astype(np.uint32)
        )
        packed[:, 3] = rgb_uint32.view(np.float32)
        return packed


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapCloudPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
