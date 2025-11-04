#!/usr/bin/env python3
# coding=utf8

import argparse
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('z', type=float)
    parser.add_argument('yaw', type=float)
    parser.add_argument('pitch', type=float)
    parser.add_argument('roll', type=float)
    args = parser.parse_args()

    rclpy.init()
    node = Node('publish_initial_pose')
    pub_pose = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)

    # 转换为pose
    rot = R.from_euler('xyz', [args.roll, args.pitch, args.yaw])
    quat = rot.as_quat()  # [x, y, z, w]
    
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.pose.pose = Pose(
        position=Point(x=args.x, y=args.y, z=args.z),
        orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    )
    initial_pose.header.stamp = node.get_clock().now().to_msg()
    initial_pose.header.frame_id = 'map'
    
    # Wait a bit for connections to be established
    time.sleep(1.0)
    
    node.get_logger().info(f'Initial Pose: {args.x} {args.y} {args.z} {args.yaw} {args.pitch} {args.roll}')
    pub_pose.publish(initial_pose)
    
    # Keep node alive for a moment to ensure message is sent
    rclpy.spin_once(node, timeout_sec=0.5)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import time
    main()
