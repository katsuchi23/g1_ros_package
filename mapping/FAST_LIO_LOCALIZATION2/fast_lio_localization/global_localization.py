#!/usr/bin/env python3

import copy
import threading
import time

import open3d as o3d
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
# from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import numpy as np
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf_transformations
import ros2_numpy
from rclpy.duration import Duration


class FastLIOLocalization(Node):
    def __init__(self):
        super().__init__("fast_lio_localization")
        self.global_map = None
        self.T_map_to_odom = np.eye(4)
        self.cur_odom = None
        self.cur_scan = None
        self.initialized = False

        self.declare_parameters(
            namespace="",
            parameters=[
                ("map_voxel_size", 0.4),
                ("scan_voxel_size", 0.1),
                ("freq_localization", 0.5),
                ("freq_global_map", 0.25),
                ("localization_threshold", 0.8),
                ("fov", 6.28319),
                ("fov_far", 300),
                ("pcd_map_topic", "/map"),
                ("pcd_map_path", ""),
                ("use_odom_transform", True),
                ("odom_roll", 180.0),
                ("odom_pitch", -7.5),
                ("odom_yaw", 0.0),
            ],
        )
        
        # Check if odom transformation is enabled
        self.use_odom_transform = self.get_parameter("use_odom_transform").value
        if self.use_odom_transform:
            self.get_logger().info(f"Odom transformation enabled: roll={self.get_parameter('odom_roll').value}°, "
                                   f"pitch={self.get_parameter('odom_pitch').value}°, "
                                   f"yaw={self.get_parameter('odom_yaw').value}°")
            self.get_logger().info("Will use TF tree to transform point clouds from camera_init to odom")

        # Initialize TF buffer and listener (must be before subscriptions)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # self.pub_global_map = self.create_publisher(PointCloud2, self.get_parameter("pcd_map_topic").value, 10)
        self.pub_pc_in_map = self.create_publisher(PointCloud2, "/cur_scan_in_map", 10)
        self.pub_submap = self.create_publisher(PointCloud2, "/submap", 10)
        self.pub_map_to_odom = self.create_publisher(Odometry, "/map_to_odom", 10)

        self.get_logger().info("Waiting for global map...")
        # global_map_msg = wait_for_message(msg_type = PointCloud2, node = self, topic = "/cloud_pcd")[1]
        # self.initialize_global_map(global_map_msg)
        
        self.initialize_global_map()
        self.get_logger().info("Global map received.")
        
        self.create_subscription(PointCloud2, "/cloud_registered", self.cb_save_cur_scan, 10)
        self.create_subscription(Odometry, "/Odometry", self.cb_save_cur_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.cb_initialize_pose, 10)

        self.timer_localisation = self.create_timer(1.0 / self.get_parameter("freq_localization").value, self.localisation_timer_callback)
        # self.timer_global_map = self.create_timer(1/ self.get_parameter("freq_global_map").value, self.global_map_callback)

    def global_map_callback(self):
        # self.get_logger().info(np.array(self.global_map.points).shape)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        self.publish_point_cloud(self.pub_global_map, header, np.array(self.global_map.points))
        
    def pose_to_mat(self, pose):
        trans = np.eye(4)
        trans[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        trans[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]
        return trans
    
    def tf_to_matrix(self, transform):
        """Convert geometry_msgs/Transform to 4x4 transformation matrix"""
        trans = np.eye(4)
        trans[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
        quat = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
        trans[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]
        return trans
    
    def msg_to_array(self, pc_msg):
        pc_array = ros2_numpy.numpify(pc_msg)
        return pc_array["xyz"]
    
    def registration_at_scale(self, scan, map, initial, scale):
        result_icp = o3d.pipelines.registration.registration_icp(
        self.voxel_down_sample(scan, self.get_parameter("scan_voxel_size").value * scale),
        self.voxel_down_sample(map, self.get_parameter("map_voxel_size").value * scale),
        1.0 * scale,
        initial,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20),
        )
        return result_icp.transformation, result_icp.fitness
            
    def inverse_se3(self, trans):
        trans_inverse = np.eye(4)
        # R
        trans_inverse[:3, :3] = trans[:3, :3].T
        # t
        trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
        return trans_inverse

    def publish_point_cloud(self, publisher, header, pc):
        data = dict()
        data["xyz"] = pc[:, :3]
        
        if pc.shape[1] == 4:
            data["intensity"] = pc[:, 3]
        # else:
            # data["rgb"] = np.ones_like(pc)
        msg = ros2_numpy.msgify(PointCloud2, data)
        msg.header = header
        if len(msg.fields) == 4:
            msg.point_step = 16
        else:
            msg.point_step = 12
            
        publisher.publish(msg)
        
    def crop_global_map_in_FOV(self, pose_estimation):
        # FAST-LIO publishes odometry as camera_init -> body
        # cur_odom contains the transform from camera_init to body
        T_camera_init_to_body = self.pose_to_mat(self.cur_odom.pose.pose)
        
        # pose_estimation is T_map_to_odom (in odom frame, which is the correct orientation)
        if self.use_odom_transform:
            # We need to get body position in odom frame
            # TF chain: odom -> camera_init -> body
            try:
                # Get static TF: odom -> camera_init (the flip transform)
                tf_odom_to_camera = self.tf_buffer.lookup_transform(
                    'camera_init',  # target frame (flipped)
                    'odom',         # source frame (correct orientation)
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5)
                )
                
                # Convert TF to transformation matrix
                T_odom_to_camera_init = self.tf_to_matrix(tf_odom_to_camera.transform)
                
                # Compute body position in odom frame
                # T_odom_to_body = T_odom_to_camera_init * T_camera_init_to_body
                T_odom_to_body = np.matmul(T_odom_to_camera_init, T_camera_init_to_body)
                
                self.get_logger().debug('Using odom->camera_init->body transform chain for FOV cropping')
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f'TF lookup failed in crop_global_map_in_FOV: {e}. Using camera_init as odom.')
                T_odom_to_body = T_camera_init_to_body
        else:
            # No odom transform: treat camera_init as odom
            T_odom_to_body = T_camera_init_to_body
            
        # Now compute body position in map frame
        # T_map_to_body = T_map_to_odom * T_odom_to_body
        T_map_to_body = np.matmul(pose_estimation, T_odom_to_body)
        T_body_to_map = self.inverse_se3(T_map_to_body)

        global_map_in_map = np.array(self.global_map.points)
        global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
        global_map_in_body = np.matmul(T_body_to_map, global_map_in_map.T).T

        if self.get_parameter("fov").value > 3.14:
            indices = np.where(
                (global_map_in_body[:, 0] < self.get_parameter("fov_far").value)
                & (np.abs(np.arctan2(global_map_in_body[:, 1], global_map_in_body[:, 0])) < self.get_parameter("fov").value / 2.0)
            )
        else:
            indices = np.where(
                (global_map_in_body[:, 0] > 0)
                & (global_map_in_body[:, 0] < self.get_parameter("fov_far").value)
                & (np.abs(np.arctan2(global_map_in_body[:, 1], global_map_in_body[:, 0])) < self.get_parameter("fov").value / 2.0)
            )
        global_map_in_FOV = o3d.geometry.PointCloud()
        global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

        header = self.cur_odom.header
        header.frame_id = "map"
        self.publish_point_cloud(self.pub_submap, header, np.array(global_map_in_FOV.points)[::10])

        return global_map_in_FOV

    def global_localization(self, pose_estimation):
        scan_tobe_mapped = copy.copy(self.cur_scan)
        global_map_in_FOV = self.crop_global_map_in_FOV(pose_estimation)
        
        transformation, _ = self.registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)
        
        transformation, fitness = self.registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=1)
        
        if fitness > self.get_parameter("localization_threshold").value:
            self.T_map_to_odom = transformation
            self.publish_odom(transformation)
        else:
            self.get_logger().warn(f"Fitness score {fitness} less than localization threshold {self.get_parameter('localization_threshold').value}")

    def voxel_down_sample(self, pcd, voxel_size):
        # print(pcd)
        
        try:
            pcd_down = pcd.voxel_down_sample(voxel_size)
        
        except Exception as e:
            # for opend3d 0.7 or lower
            pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
            
        return pcd_down

    def cb_save_cur_odom(self, msg):
        self.cur_odom = msg
        
    def cb_save_cur_scan(self, msg):
        # Use TF to transform point cloud from camera_init to odom
        if self.use_odom_transform:
            try:
                # Lookup transform from camera_init to odom
                transform = self.tf_buffer.lookup_transform(
                    'odom',                   # target frame
                    msg.header.frame_id,      # source frame (camera_init)
                    rclpy.time.Time(),        # get latest transform
                    timeout=Duration(seconds=1.0)
                )
                
                # Transform the entire PointCloud2 message using TF
                msg_transformed = do_transform_cloud(msg, transform)
                pc = self.msg_to_array(msg_transformed)
                
                self.get_logger().info(f"[TF TRANSFORM] ✓ Transformed point cloud from '{msg.header.frame_id}' to 'odom' using TF", 
                                    throttle_duration_sec=5.0)
                
                # Create corrected header for odom frame
                header_odom = Header()
                header_odom.stamp = msg.header.stamp
                header_odom.frame_id = "odom"  # ✓ Correct frame
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f'TF lookup failed: {e}. Using original frame.')
                pc = self.msg_to_array(msg)
                header_odom = msg.header  # Fallback to original
        else:
            pc = self.msg_to_array(msg)
            header_odom = msg.header
        
        # Create point cloud (already in odom frame if transform was successful)
        self.cur_scan = o3d.geometry.PointCloud()
        self.cur_scan.points = o3d.utility.Vector3dVector(pc)
        
        # Publish for visualization with CORRECT header
        self.publish_point_cloud(self.pub_pc_in_map, header_odom, pc)  # ✓ Use odom header
        
    def initialize_global_map(self): #, pc_msg):
        # self.global_map = o3d.geometry.PointCloud()
        # self.global_map.points = o3d.utility.Vector3dVector(self.msg_to_array(pc_msg)[:, :3])
        self.global_map = o3d.io.read_point_cloud(self.get_parameter("pcd_map_path").value)
        self.global_map = self.voxel_down_sample(self.global_map, self.get_parameter("map_voxel_size").value)
        # o3d.io.write_point_cloud("/home/wheelchair2/laksh_ws/pcds/lab_map_with_outside_corridor (with ground pcd)_downsampled.pcd", self.global_map)
        self.get_logger().info("Global map received.")

    def cb_initialize_pose(self, msg):
        initial_pose = self.pose_to_mat(msg.pose.pose)
        self.initialized = True
        self.get_logger().info("Initial pose received.")
        
        if self.cur_scan is not None:
            self.global_localization(initial_pose)
            
    def publish_odom(self, transform):
        odom_msg = Odometry()
        xyz = transform[:3, 3]
        quat = tf_transformations.quaternion_from_matrix(transform)
        odom_msg.pose.pose = Pose(
            position = Point(x = xyz[0], y = xyz[1], z = xyz[2]), 
            orientation = Quaternion(x = quat[0], y = quat[1], z = quat[2], w = quat[3])
        )
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"
        self.pub_map_to_odom.publish(odom_msg)

    def localisation_timer_callback(self):
        if not self.initialized:
            self.get_logger().info("Waiting for initial pose...")
            return
        
        if self.cur_scan is not None:
            self.global_localization(self.T_map_to_odom)


def main(args=None):
    rclpy.init(args=args)
    node = FastLIOLocalization()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()