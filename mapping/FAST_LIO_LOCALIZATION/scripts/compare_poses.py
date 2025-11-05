#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class PoseComparator(Node):
    def __init__(self):
        super().__init__('pose_comparator')
        
        self.localization_pose = None
        self.amcl_pose = None
        
        self.sub_localization = self.create_subscription(
            Odometry, '/localization', self.cb_localization, 10)
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl, 10)
        
        # Print comparison every 2 seconds
        self.timer = self.create_timer(2.0, self.compare_poses)
        
        self.get_logger().info('Pose Comparator started. Comparing /localization vs /amcl_pose')
    
    def cb_localization(self, msg):
        self.localization_pose = msg.pose.pose
    
    def cb_amcl(self, msg):
        self.amcl_pose = msg.pose.pose
    
    def compare_poses(self):
        if self.localization_pose is None:
            self.get_logger().warn('/localization not received yet')
            return
        if self.amcl_pose is None:
            self.get_logger().warn('/amcl_pose not received yet')
            return
        
        # Extract positions
        loc_x = self.localization_pose.position.x
        loc_y = self.localization_pose.position.y
        loc_z = self.localization_pose.position.z
        
        amcl_x = self.amcl_pose.position.x
        amcl_y = self.amcl_pose.position.y
        amcl_z = self.amcl_pose.position.z
        
        # Calculate differences
        diff_x = loc_x - amcl_x
        diff_y = loc_y - amcl_y
        diff_z = loc_z - amcl_z
        diff_xy = math.sqrt(diff_x**2 + diff_y**2)
        diff_3d = math.sqrt(diff_x**2 + diff_y**2 + diff_z**2)
        
        # Extract orientations (quaternions)
        loc_qw = self.localization_pose.orientation.w
        loc_qz = self.localization_pose.orientation.z
        
        amcl_qw = self.amcl_pose.orientation.w
        amcl_qz = self.amcl_pose.orientation.z
        
        # Convert to yaw angle (simplified for 2D)
        loc_yaw = 2 * math.atan2(loc_qz, loc_qw)
        amcl_yaw = 2 * math.atan2(amcl_qz, amcl_qw)
        yaw_diff = abs(loc_yaw - amcl_yaw)
        if yaw_diff > math.pi:
            yaw_diff = 2 * math.pi - yaw_diff
        
        self.get_logger().info('='*60)
        self.get_logger().info(f'/localization:  x={loc_x:7.3f}, y={loc_y:7.3f}, z={loc_z:7.3f}, yaw={math.degrees(loc_yaw):7.2f}°')
        self.get_logger().info(f'/amcl_pose:     x={amcl_x:7.3f}, y={amcl_y:7.3f}, z={amcl_z:7.3f}, yaw={math.degrees(amcl_yaw):7.2f}°')
        self.get_logger().info(f'Difference:     Δx={diff_x:7.3f}, Δy={diff_y:7.3f}, Δz={diff_z:7.3f}, Δyaw={math.degrees(yaw_diff):7.2f}°')
        self.get_logger().info(f'Distance: 2D={diff_xy:.3f}m, 3D={diff_3d:.3f}m')
        
        # Warning if difference is large
        if diff_xy > 0.5:
            self.get_logger().warn(f'Large XY difference: {diff_xy:.3f}m!')
        if math.degrees(yaw_diff) > 10:
            self.get_logger().warn(f'Large yaw difference: {math.degrees(yaw_diff):.2f}°!')

def main(args=None):
    rclpy.init(args=args)
    node = PoseComparator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
