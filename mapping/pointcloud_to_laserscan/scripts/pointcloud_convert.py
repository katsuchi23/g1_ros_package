#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import numpy as np
import tf_transformations
from sensor_msgs_py import point_cloud2


class BodyToBaseLinkPC(Node):
    def __init__(self):
        super().__init__("pc_body_to_base_link")

        # --- STATIC TF: BODY -> BASE_LINK ---
        roll  = np.radians(-180.0)
        pitch = np.radians(-7.5)
        yaw   = 0.0

        self.T_body_to_base = tf_transformations.euler_matrix(roll, pitch, yaw)

        # Subscriber: raw point cloud in body frame
        self.create_subscription(
            PointCloud2,
            "/livox/lidar/pcd2",
            self.cb,
            10
        )

        # Publisher: transformed cloud in base_link
        self.pub = self.create_publisher(
            PointCloud2,
            "/cloud_in_base_link",
            10
        )

        self.get_logger().info("PointCloud conversion body -> base_link started.")

    def cb(self, msg):

        # Read only xyz points
        raw_points = list(point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        ))

        if len(raw_points) == 0:
            return

        # Convert structured rows → simple tuples
        pts = [(p[0], p[1], p[2]) for p in raw_points]

        # Now convert into numpy Nx3 array
        pc = np.array(pts, dtype=np.float32)

        # Homogeneous
        N = pc.shape[0]
        pc_h = np.hstack([pc, np.ones((N, 1), dtype=np.float32)])  # (N,4)

        # Apply static transform: base_link = T * body_points
        pc_trans = (self.T_body_to_base @ pc_h.T).T[:, :3]  # (N,3)

        # Build PointCloud2 message
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = "base_link"

        cloud_out = point_cloud2.create_cloud(header, fields, pc_trans)

        self.pub.publish(cloud_out)


def main(args=None):
    rclpy.init(args=args)
    node = BodyToBaseLinkPC()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
