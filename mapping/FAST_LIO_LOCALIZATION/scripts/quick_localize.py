#!/usr/bin/env python3
"""
Quick Initial Pose Publisher
Usage: ros2 run fast_lio_localization quick_localize.py --x 0 --y 0 --yaw 0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import argparse
import math


class QuickLocalizer(Node):
    def __init__(self, x, y, z, yaw):
        super().__init__('quick_localizer')
        
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # Wait a moment for the publisher to be ready
        self.get_logger().info('Publishing initial pose...')
        self.timer = self.create_timer(0.5, lambda: self.publish_pose(x, y, z, yaw))
        self.published_count = 0
        
    def publish_pose(self, x, y, z, yaw):
        if self.published_count >= 3:  # Publish 3 times to ensure it's received
            self.get_logger().info(f'Initial pose published: x={x}, y={y}, z={z}, yaw={yaw}°')
            self.timer.cancel()
            rclpy.shutdown()
            return
            
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Position
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = float(z)
        
        # Orientation (from yaw)
        yaw_rad = math.radians(yaw)
        msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        
        # Covariance (small values = high confidence)
        msg.pose.covariance[0] = 0.25  # x
        msg.pose.covariance[7] = 0.25  # y
        msg.pose.covariance[35] = 0.06853  # yaw
        
        self.publisher.publish(msg)
        self.published_count += 1


def main():
    parser = argparse.ArgumentParser(description='Publish initial pose for localization')
    parser.add_argument('--x', type=float, default=0.0, help='X position in meters')
    parser.add_argument('--y', type=float, default=0.0, help='Y position in meters')
    parser.add_argument('--z', type=float, default=0.0, help='Z position in meters')
    parser.add_argument('--yaw', type=float, default=0.0, help='Yaw angle in degrees')
    
    args = parser.parse_args()
    
    rclpy.init()
    node = QuickLocalizer(args.x, args.y, args.z, args.yaw)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
