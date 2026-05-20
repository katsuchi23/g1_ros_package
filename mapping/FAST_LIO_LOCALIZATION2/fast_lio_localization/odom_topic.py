#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import numpy as np
try:
    import tf_transformations
except ImportError:
    import tf_transformations_compat as tf_transformations


class FastLioOdomConverter(Node):
    def __init__(self):
        super().__init__("fastlio_odom_converter")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("publish.odom_roll", 180.0),
                ("publish.odom_pitch", 21.0),
                ("publish.odom_yaw", 0.0),
            ],
        )

        odom_roll = self.get_parameter("publish.odom_roll").value
        odom_pitch = self.get_parameter("publish.odom_pitch").value
        odom_yaw = self.get_parameter("publish.odom_yaw").value

        # --- STATIC TRANSFORMS ---------------------------------------
        # 1. odom -> camera_init
        R1 = self.rpy_to_matrix(np.radians(odom_roll),
                                np.radians(odom_pitch),
                                np.radians(odom_yaw))

        # 2. body -> base_link
        R2 = self.rpy_to_matrix(np.radians(-odom_roll),
                                np.radians(odom_pitch),
                                np.radians(-odom_yaw))

        # Combined static transform:
        # odom -> base_link = (odom->camera_init) * (body->base_link)
        self.T_static = R1 @ R2

        # Subscribers and publishers
        self.sub = self.create_subscription(Odometry, "/Odometry", self.cb, 10)
        self.pub = self.create_publisher(Odometry, "/odom", 10)

        self.get_logger().info(
            f"FAST-LIO pure-static odom converter started with roll={odom_roll}°, "
            f"pitch={odom_pitch}°, yaw={odom_yaw}°."
        )

    # Utility: Convert RPY to 4x4 matrix
    def rpy_to_matrix(self, roll, pitch, yaw):
        T = tf_transformations.euler_matrix(roll, pitch, yaw)
        return T

    # Utility: FAST-LIO odometry: camera_init -> body -> matrix
    def pose_to_mat(self, pose):
        T = np.eye(4)
        T[0, 3] = pose.position.x
        T[1, 3] = pose.position.y
        T[2, 3] = pose.position.z
        q = [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w]
        T[:3, :3] = tf_transformations.quaternion_matrix(q)[:3, :3]
        return T

    def cb(self, msg):
        # camera_init -> body (dynamic odometry)
        T_cam_to_body = self.pose_to_mat(msg.pose.pose)

        # odom -> base_link = static ⨉ dynamic
        T_odom_to_base = self.T_static @ T_cam_to_body

        # Extract xyz and quaternion
        xyz = T_odom_to_base[:3, 3]
        quat = tf_transformations.quaternion_from_matrix(T_odom_to_base)

        # Construct new odometry
        odom = Odometry()
        odom.header.stamp = msg.header.stamp    # OK to reuse
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position = Point(
            x=xyz[0], y=xyz[1], z=xyz[2]
        )
        odom.pose.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )

        odom.twist = msg.twist  # same twist

        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = FastLioOdomConverter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
