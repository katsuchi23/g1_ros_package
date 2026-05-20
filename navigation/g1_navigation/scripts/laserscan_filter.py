#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserScanFilter(Node):
    def __init__(self):
        super().__init__('laserscan_filter')
        
        # Parameters
        self.declare_parameter('min_range', 0.30)  # 20cm minimum range
        self.min_range = self.get_parameter('min_range').value
        
        # Subscriber to raw laser scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publisher for filtered laser scan
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan_filtered',
            10
        )
        
        self.get_logger().info(f'LaserScan Filter initialized with min_range: {self.min_range}m')
    
    def scan_callback(self, msg):
        """Filter laser scan data to ignore readings below minimum range"""
        filtered_msg = LaserScan()
        
        # Copy header and scan parameters
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = max(msg.range_min, self.min_range)
        filtered_msg.range_max = msg.range_max
        
        # Filter ranges - set values below min_range to inf (ignored by navigation)
        filtered_msg.ranges = []
        for range_val in msg.ranges:
            if range_val < self.min_range:
                filtered_msg.ranges.append(float('inf'))
            else:
                filtered_msg.ranges.append(range_val)
        
        # Copy intensities if available
        filtered_msg.intensities = msg.intensities
        
        # Publish filtered scan
        self.scan_pub.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanFilter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
