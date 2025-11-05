#!/usr/bin/env python3
# coding=utf8

"""
Script to find the best initial position for localization by testing multiple positions
and returning the one with the highest fitness score.
"""

import copy
import os
os.environ['DISPLAY'] = os.environ.get('DISPLAY', ':0')

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

# Global variables
global_map = None
cur_scan = None
cur_odom = None

# Parameters (should match global_localization.py)
MAP_VOXEL_SIZE = 0.4
SCAN_VOXEL_SIZE = 0.1
FOV = 1.6
FOV_FAR = 150


def msg_to_array(pc_msg):
    """Convert PointCloud2 message to numpy array"""
    pc_list = []
    for point in pc2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True):
        pc_list.append([point[0], point[1], point[2]])
    return np.array(pc_list)


def pose_to_mat(pose_msg):
    """Convert pose message to 4x4 transformation matrix"""
    pos = pose_msg.pose.pose.position
    ori = pose_msg.pose.pose.orientation
    
    trans_mat = np.eye(4)
    trans_mat[0:3, 3] = [pos.x, pos.y, pos.z]
    
    rot = R.from_quat([ori.x, ori.y, ori.z, ori.w])
    rot_mat = np.eye(4)
    rot_mat[0:3, 0:3] = rot.as_matrix()
    
    return trans_mat @ rot_mat


def inverse_se3(trans):
    trans_inverse = np.eye(4)
    trans_inverse[:3, :3] = trans[:3, :3].T
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse


def voxel_down_sample(pcd, voxel_size):
    return pcd.voxel_down_sample(voxel_size)


def registration_at_scale(pc_scan, pc_map, initial, scale):
    result_icp = o3d.pipelines.registration.registration_icp(
        voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale), 
        voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale),
        1.0 * scale, initial,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
    )
    return result_icp.transformation, result_icp.fitness


def crop_global_map_in_FOV(global_map, pose_estimation, cur_odom):
    T_odom_to_base_link = pose_to_mat(cur_odom)
    T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
    T_base_link_to_map = inverse_se3(T_map_to_base_link)

    global_map_in_map = np.array(global_map.points)
    global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
    global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

    if FOV > 3.14:
        indices = np.where(
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    else:
        indices = np.where(
            (global_map_in_base_link[:, 0] > 0) &
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    
    global_map_in_FOV = o3d.geometry.PointCloud()
    global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

    return global_map_in_FOV


def test_localization(x, y, yaw, cur_scan, global_map, cur_odom):
    """Test localization at a specific position and return fitness score"""
    # Create transformation matrix for test position
    pose_estimation = np.eye(4)
    rot_mat = R.from_euler('z', yaw).as_matrix()
    pose_estimation[:3, :3] = rot_mat
    pose_estimation[0, 3] = x
    pose_estimation[1, 3] = y
    pose_estimation[2, 3] = 0.0
    
    try:
        # Crop map to FOV
        global_map_in_FOV = crop_global_map_in_FOV(global_map, pose_estimation, cur_odom)
        
        if len(global_map_in_FOV.points) < 100:
            return 0.0  # Not enough points in FOV
        
        # Run ICP
        transformation, _ = registration_at_scale(cur_scan, global_map_in_FOV, initial=pose_estimation, scale=5)
        transformation, fitness = registration_at_scale(cur_scan, global_map_in_FOV, initial=transformation, scale=1)
        
        return fitness
    except Exception as e:
        return 0.0


class BestPositionFinderNode(Node):
    def __init__(self):
        super().__init__('best_position_finder')
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        from visualization_msgs.msg import Marker
        self.marker_pub = self.create_publisher(Marker, '/best_position_marker', 10)
        
        # Subscribers
        self.sub_scan = self.create_subscription(
            PointCloud2, '/cloud_registered', self.cb_save_cur_scan, sensor_qos)
        self.sub_odom = self.create_subscription(
            Odometry, '/Odometry', self.cb_save_cur_odom, sensor_qos)
        self.sub_map = self.create_subscription(
            PointCloud2, '/map_pointcloud', self.initialize_global_map, map_qos)
        
        self.get_logger().info('Best Position Finder Node Started')
        self.get_logger().info('Waiting for map and scan data...')
        self.get_logger().info('')
        self.get_logger().info('Commands:')
        self.get_logger().info('  1. Wait for "Ready to search" message')
        self.get_logger().info('  2. Call service: ros2 service call /find_best_position std_srvs/srv/Trigger')
        self.get_logger().info('  3. Or run: ros2 run fast_lio_localization find_best_position.py --search')
        
        # Create service
        from std_srvs.srv import Trigger
        self.srv = self.create_service(Trigger, 'find_best_position', self.search_callback)
        
        self.map_received = False
        self.scan_received = False
    
    def initialize_global_map(self, pc_msg):
        global global_map
        
        if global_map is None:
            global_map = o3d.geometry.PointCloud()
            pc_array = msg_to_array(pc_msg)
            global_map.points = o3d.utility.Vector3dVector(pc_array[:, :3])
            global_map = voxel_down_sample(global_map, MAP_VOXEL_SIZE)
            self.map_received = True
            self.get_logger().info(f'Map received with {len(global_map.points)} points')
            self.check_ready()
    
    def cb_save_cur_odom(self, odom_msg):
        global cur_odom
        cur_odom = odom_msg
    
    def cb_save_cur_scan(self, pc_msg):
        global cur_scan
        
        pc = msg_to_array(pc_msg)
        cur_scan = o3d.geometry.PointCloud()
        cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3])
        
        if not self.scan_received:
            self.scan_received = True
            self.get_logger().info(f'Scan received with {len(cur_scan.points)} points')
            self.check_ready()
    
    def check_ready(self):
        if self.map_received and self.scan_received:
            self.get_logger().info('')
            self.get_logger().info('=== Ready to search for best position! ===')
            self.get_logger().info('Call: ros2 service call /find_best_position std_srvs/srv/Trigger')
            self.get_logger().info('')
    
    def publish_position_marker(self, x, y, yaw):
        """Publish a marker at the best position"""
        from visualization_msgs.msg import Marker
        from builtin_interfaces.msg import Duration
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "best_position"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 1.0  # Raised higher so it's very visible
        
        # Orientation (yaw to quaternion)
        quat = R.from_euler('z', yaw).as_quat()
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        
        # Scale (arrow size) - MUCH BIGGER!
        marker.scale.x = 5.0  # Length: 5 meters (very long arrow)
        marker.scale.y = 1.0  # Width: 1 meter (thick)
        marker.scale.z = 1.0  # Height: 1 meter (thick)
        
        # Color (BRIGHT RED - very visible!)
        marker.color.r = 1.0  # Full red
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque
        
        # Lifetime (0 = forever)
        marker.lifetime = Duration(sec=0, nanosec=0)
        
        # Publish
        self.marker_pub.publish(marker)
        self.get_logger().info('Marker published! Check RViz.')
    
    def search_callback(self, request, response):
        """Service callback to start searching"""
        self.get_logger().info('Starting position search...')
        
        if global_map is None or cur_scan is None or cur_odom is None:
            response.success = False
            response.message = 'Map, scan, or odometry not available'
            return response
        
        best_x, best_y, best_yaw, best_fitness = self.search_best_position()
        
        response.success = True
        response.message = f'Best position: x={best_x:.2f}, y={best_y:.2f}, yaw={best_yaw:.2f} rad ({np.degrees(best_yaw):.1f}°), fitness={best_fitness:.4f}'
        
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('SEARCH COMPLETE!')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Best Position:')
        self.get_logger().info(f'  X: {best_x:.3f} meters')
        self.get_logger().info(f'  Y: {best_y:.3f} meters')
        self.get_logger().info(f'  Yaw: {best_yaw:.3f} radians ({np.degrees(best_yaw):.1f}°)')
        self.get_logger().info(f'  Fitness Score: {best_fitness:.4f}')
        self.get_logger().info('='*60)
        self.get_logger().info('')
        self.get_logger().info('Publishing marker to /best_position_marker')
        self.get_logger().info('Add "Marker" display in RViz and subscribe to /best_position_marker')
        self.get_logger().info('')
        
        # Publish marker
        self.publish_position_marker(best_x, best_y, best_yaw)
        
        return response
    
    def search_best_position(self):
        """Search for the best position by testing a grid of positions"""
        global global_map, cur_scan, cur_odom
        
        # Get map bounds
        map_points = np.array(global_map.points)
        x_min, y_min = map_points[:, 0].min(), map_points[:, 1].min()
        x_max, y_max = map_points[:, 0].max(), map_points[:, 1].max()
        
        self.get_logger().info(f'Map bounds: X=[{x_min:.1f}, {x_max:.1f}], Y=[{y_min:.1f}, {y_max:.1f}]')
        
        # Define search parameters
        x_step = 2.0  # Search every 2 meters
        y_step = 2.0
        yaw_step = np.pi / 4  # Search every 45 degrees
        
        best_fitness = 0.0
        best_x = 0.0
        best_y = 0.0
        best_yaw = 0.0
        
        total_tests = 0
        x_range = np.arange(x_min, x_max, x_step)
        y_range = np.arange(y_min, y_max, y_step)
        yaw_range = np.arange(-np.pi, np.pi, yaw_step)
        
        total_positions = len(x_range) * len(y_range) * len(yaw_range)
        self.get_logger().info(f'Testing {total_positions} positions...')
        self.get_logger().info('This may take a few minutes...')
        
        for i, x in enumerate(x_range):
            for j, y in enumerate(y_range):
                for k, yaw in enumerate(yaw_range):
                    fitness = test_localization(x, y, yaw, cur_scan, global_map, cur_odom)
                    total_tests += 1
                    
                    if fitness > best_fitness:
                        best_fitness = fitness
                        best_x = x
                        best_y = y
                        best_yaw = yaw
                        self.get_logger().info(f'New best! X={x:.1f}, Y={y:.1f}, Yaw={np.degrees(yaw):.0f}°, Fitness={fitness:.4f}')
                    
                    # Progress update every 50 tests
                    if total_tests % 50 == 0:
                        progress = (total_tests / total_positions) * 100
                        self.get_logger().info(f'Progress: {progress:.1f}% ({total_tests}/{total_positions}), Current best fitness: {best_fitness:.4f}')
        
        return best_x, best_y, best_yaw, best_fitness


def main(args=None):
    rclpy.init(args=args)
    node = BestPositionFinderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
