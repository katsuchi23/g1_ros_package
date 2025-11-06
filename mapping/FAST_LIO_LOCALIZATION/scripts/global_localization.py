#!/usr/bin/env python3
# coding=utf8

import copy
import os
import threading
import time

# Fix for Wayland/X11 compatibility
os.environ['DISPLAY'] = os.environ.get('DISPLAY', ':0')

import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformBroadcaster

global_map = None
initialized = False
T_map_to_odom = np.eye(4)
cur_odom = None
cur_scan = None


def pose_to_mat(pose_msg):
    """Convert pose message to 4x4 transformation matrix
    Following ROS1 logic: multiply translation matrix with rotation matrix
    This matches tf.listener.xyz_to_mat44 @ tf.listener.xyzw_to_mat44
    """
    pos = pose_msg.pose.pose.position
    ori = pose_msg.pose.pose.orientation
    
    # Create translation matrix
    trans_mat = np.eye(4)
    trans_mat[0:3, 3] = [pos.x, pos.y, pos.z]
    
    # Create rotation matrix from quaternion
    rot = R.from_quat([ori.x, ori.y, ori.z, ori.w])
    rot_mat = np.eye(4)
    rot_mat[0:3, 0:3] = rot.as_matrix()
    
    # ROS1 does: np.matmul(xyz_to_mat44, xyzw_to_mat44)
    # which is translation @ rotation
    return np.matmul(trans_mat, rot_mat)


def msg_to_array(pc_msg):
    """Convert PointCloud2 message to numpy array"""
    pc_list = []
    for point in pc2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True):
        pc_list.append([point[0], point[1], point[2]])
    return np.array(pc_list)


def registration_at_scale(pc_scan, pc_map, initial, scale):
    result_icp = o3d.pipelines.registration.registration_icp(
        voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale), 
        voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale),
        1.0 * scale, initial,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)  # match ROS1 logic
    )
    return result_icp.transformation, result_icp.fitness


def inverse_se3(trans):
    trans_inverse = np.eye(4)
    # R
    trans_inverse[:3, :3] = trans[:3, :3].T
    # t
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse


def publish_point_cloud(publisher, header, pc):
    """Publish point cloud"""
    points_list = []
    for point in pc:
        points_list.append([point[0], point[1], point[2]])  # x, y, z only
    
    msg = pc2.create_cloud_xyz32(header, points_list)
    publisher.publish(msg)


def crop_global_map_in_FOV(global_map, pose_estimation, cur_odom):
    # 当前scan原点的位姿 (Copied exactly from ROS1)
    T_odom_to_base_link = pose_to_mat(cur_odom)
    T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
    T_base_link_to_map = inverse_se3(T_map_to_base_link)

    # 把地图转换到lidar系下
    global_map_in_map = np.array(global_map.points)
    global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
    global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

    # 将视角内的地图点提取出来
    if FOV > 3.14:
        # 环状lidar 仅过滤距离
        indices = np.where(
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    else:
        # 非环状lidar 保前视范围
        # FOV_FAR>x>0 且角度小于FOV
        indices = np.where(
            (global_map_in_base_link[:, 0] > 0) &
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    
    global_map_in_FOV = o3d.geometry.PointCloud()
    global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

    return global_map_in_FOV


def global_localization(pose_estimation, node, pub_map_to_odom, publish_initial=True):
    global global_map, cur_scan, cur_odom, T_map_to_odom
    # 用icp配准 (Copied exactly from ROS1)
    node.get_logger().info('Global localization by scan-to-map matching......')

    # TODO 这里注意线程安全
    scan_tobe_mapped = copy.copy(cur_scan)

    tic = time.time()

    global_map_in_FOV = crop_global_map_in_FOV(global_map, pose_estimation, cur_odom)
    
    # Publish submap for visualization
    header = cur_odom.header
    header.frame_id = 'map'
    publish_point_cloud(node.pub_submap, header, np.array(global_map_in_FOV.points)[::10])

    # 粗配准
    # ICP: aligns source (scan in camera_init) to target (map in map frame)
    # Returns transformation from camera_init -> map
    transformation, _ = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)

    # 精配准
    transformation, fitness = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=transformation, scale=1)
    
    toc = time.time()
    node.get_logger().info('Time: {}'.format(toc - tic))
    node.get_logger().info('')

    # 当全局定位成功时才更新map2odom
    if fitness > LOCALIZATION_TH:
        # CRITICAL FIX: ICP gives us camera_init->map, but we need map->camera_init
        # The scan is in camera_init frame, map is in map frame
        # ICP returns T such that: map_points ≈ T @ camera_init_points
        # So T is camera_init->map, but we publish map->odom (which is map->camera_init)
        # Therefore we must INVERT the ICP result
        T_map_to_odom = inverse_se3(transformation)

        # Log the comparison for debugging (always show this)
        node.get_logger().info(f'Initial estimate:     x={pose_estimation[0,3]:.2f}, y={pose_estimation[1,3]:.2f}, z={pose_estimation[2,3]:.2f}')
        node.get_logger().info(f'ICP raw result:       x={transformation[0,3]:.2f}, y={transformation[1,3]:.2f}, z={transformation[2,3]:.2f}')
        node.get_logger().info(f'ICP inverted (final): x={T_map_to_odom[0,3]:.2f}, y={T_map_to_odom[1,3]:.2f}, z={T_map_to_odom[2,3]:.2f}')

        # 发布map_to_odom (Exact ROS1 style) - this goes to camera_init frame
        map_to_odom = Odometry()
        rot = R.from_matrix(T_map_to_odom[:3, :3])
        quat = rot.as_quat()  # [x, y, z, w]
        
        map_to_odom.pose.pose = Pose(
            position=Point(x=T_map_to_odom[0, 3], y=T_map_to_odom[1, 3], z=T_map_to_odom[2, 3]),
            orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        )
        map_to_odom.header.stamp = cur_odom.header.stamp
        map_to_odom.header.frame_id = 'map'
        pub_map_to_odom.publish(map_to_odom)
        
        node.get_logger().info('✓ LOCALIZATION SUCCESS!')
        return True
    else:
        node.get_logger().warn('Not match!!!!')
        node.get_logger().warn('{}'.format(transformation))
        node.get_logger().warn('fitness score:{}'.format(fitness))
        return False


def voxel_down_sample(pcd, voxel_size):
    return pcd.voxel_down_sample(voxel_size)


class GlobalLocalizationNode(Node):
    def __init__(self):
        super().__init__('global_localization')
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS for map - match the pcd_to_pointcloud publisher QoS (VOLATILE)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.pub_pc_in_map = self.create_publisher(PointCloud2, '/cur_scan_in_map', 1)
        self.pub_submap = self.create_publisher(PointCloud2, '/submap', 1)
        self.pub_map_to_odom = self.create_publisher(Odometry, '/map_to_odom', 1)
        
        # TF broadcaster for debug frames
        self.debug_tf_broadcaster = TransformBroadcaster(self)
        
        # Store initial pose transform for continuous republishing
        self.initial_transform = None
        self.initial_transform_lock = threading.Lock()
        
        # Simple counters to limit logging frequency for high-rate topics
        self.odom_log_count = 0
        self.scan_log_count = 0
        
        # Subscribers
        self.sub_scan = self.create_subscription(
            PointCloud2, '/cloud_registered', self.cb_save_cur_scan, sensor_qos)
        self.sub_odom = self.create_subscription(
            Odometry, '/Odometry', self.cb_save_cur_odom, sensor_qos)
        self.sub_map = self.create_subscription(
            PointCloud2, '/map_pointcloud', self.initialize_global_map, map_qos)
        self.sub_initialpose = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.cb_initialpose, 10)
        
        self.get_logger().info('Localization Node Inited...')
        self.get_logger().warn('Waiting for global map from /map_pointcloud topic......')
        
        self.localization_thread = None
        
        # Create a timer to check map status periodically
        self.map_check_timer = self.create_timer(2.0, self.check_map_status)
        self.map_received = False
        
        # Timer to continuously publish initial transform
        self.initial_tf_timer = self.create_timer(0.1, self.publish_initial_transform)  # 10 Hz
        
        # Start thread to publish identity transform before initialization
        self.pre_init_publisher_thread = threading.Thread(target=self.publish_identity_before_init, daemon=True)
        self.pre_init_publisher_thread.start()
        
    def check_map_status(self):
        """Periodically check if map has been received"""
        global global_map
        if not self.map_received and global_map is None:
            self.get_logger().warn('Still waiting for map on /map_pointcloud topic. Make sure pcd_to_pointcloud is publishing.')
    
    def publish_initial_transform(self):
        """Continuously publish the initial transform to keep it alive in TF tree"""
        with self.initial_transform_lock:
            if self.initial_transform is not None:
                # Update timestamp to current time
                self.initial_transform.header.stamp = self.get_clock().now().to_msg()
                self.debug_tf_broadcaster.sendTransform(self.initial_transform)
    
    def publish_identity_before_init(self):
        """Publish identity transform (zeros) before initialization to avoid jittering"""
        global initialized, cur_odom
        rate = self.create_rate(10)  # 10 Hz
        
        while rclpy.ok() and not initialized:
            try:
                if cur_odom is not None:
                    # Publish identity transform (map = camera_init)
                    map_to_odom = Odometry()
                    map_to_odom.pose.pose = Pose(
                        position=Point(x=0.0, y=0.0, z=0.0),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    )
                    map_to_odom.header.stamp = cur_odom.header.stamp
                    map_to_odom.header.frame_id = 'map'
                    self.pub_map_to_odom.publish(map_to_odom)
                
                rate.sleep()
            except Exception as e:
                self.get_logger().error(f'Pre-init publisher error: {e}')
                break
    
    def initialize_global_map(self, pc_msg):
        global global_map
        
        global_map = o3d.geometry.PointCloud()
        pc_array = msg_to_array(pc_msg)
        global_map.points = o3d.utility.Vector3dVector(pc_array[:, :3])
        global_map = voxel_down_sample(global_map, MAP_VOXEL_SIZE)
        self.map_received = True
        self.map_check_timer.cancel()  # Stop the checking timer once map is received
        # self.get_logger().info(f'Global map received with {len(global_map.points)} points.')
    
    def cb_save_cur_odom(self, odom_msg):
        global cur_odom
        cur_odom = odom_msg
        # Log odometry frame and a rough pose occasionally so we can check frame conventions
        self.odom_log_count += 1
        if self.odom_log_count % 20 == 1:
            try:
                p = cur_odom.pose.pose.position
                # self.get_logger().info(f"Received Odometry - header.frame_id={cur_odom.header.frame_id}, pose=({p.x:.2f},{p.y:.2f},{p.z:.2f})")
            except Exception:
                pass
                # self.get_logger().info('Received Odometry (could not parse pose)')
    
    def cb_save_cur_scan(self, pc_msg):
        global cur_scan
        
        # 注意这里fastlio直接将scan转到odom系下了 不是lidar局部系
        # Mark the incoming scan as in camera_init frame (FAST-LIO publishes in odom/camera_init)
        pc_msg.header.frame_id = 'camera_init'
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_pc_in_map.publish(pc_msg)

        # Convert to numpy array and create Open3D point cloud
        pc = msg_to_array(pc_msg)
        # Throttle scan logging to avoid log spam
        self.scan_log_count += 1
        if self.scan_log_count % 5 == 1:
            pass
            # self.get_logger().info(f'Received scan with {len(pc)} points from /cloud_registered')

        if len(pc) == 0:
            self.get_logger().warn('Received EMPTY point cloud from /cloud_registered!')
            cur_scan = o3d.geometry.PointCloud()  # Create empty point cloud
        else:
            cur_scan = o3d.geometry.PointCloud()
            cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3])
    
    def cb_initialpose(self, pose_msg):
        global initialized, T_map_to_odom, cur_odom
        
        if not initialized:
            self.get_logger().info('========================================')
            self.get_logger().info('Received initial pose estimate from RViz')
            self.get_logger().info(f'Position: x={pose_msg.pose.pose.position.x:.2f}, y={pose_msg.pose.pose.position.y:.2f}, z={pose_msg.pose.pose.position.z:.2f}')
            
            # Check what's missing
            if global_map is None:
                self.get_logger().error('Map not received yet! Make sure:')
                self.get_logger().error('  1. The map_publisher (pcd_to_pointcloud) node is running')
                self.get_logger().error('  2. The PCD file path is correct')
                self.get_logger().error('  3. Check with: ros2 topic echo /map_pointcloud --once')
                return
            
            if cur_scan is None:
                self.get_logger().error('Scan not received yet! Make sure:')
                self.get_logger().error('  1. The laserMapping node is running')
                self.get_logger().error('  2. LiDAR data is being published')
                self.get_logger().error('  3. Check with: ros2 topic echo /cloud_registered --once')
                return
            
            initial_pose = pose_to_mat(pose_msg)
            self.get_logger().info(f'Initial pose header.frame_id: {pose_msg.header.frame_id}')
            self.get_logger().info(f'Initial pose matrix:\n{initial_pose}')
            
            # DEBUG: Store initial transform for continuous publishing
            initial_transform = TransformStamped()
            initial_transform.header.stamp = self.get_clock().now().to_msg()
            initial_transform.header.frame_id = 'map'
            initial_transform.child_frame_id = 'camera_init_initial'
            rot_initial = R.from_matrix(initial_pose[:3, :3])
            quat_initial = rot_initial.as_quat()
            initial_transform.transform.translation.x = initial_pose[0, 3]
            initial_transform.transform.translation.y = initial_pose[1, 3]
            initial_transform.transform.translation.z = initial_pose[2, 3]
            initial_transform.transform.rotation.x = quat_initial[0]
            initial_transform.transform.rotation.y = quat_initial[1]
            initial_transform.transform.rotation.z = quat_initial[2]
            initial_transform.transform.rotation.w = quat_initial[3]
            
            # Store it for continuous republishing by timer
            with self.initial_transform_lock:
                self.initial_transform = initial_transform
            
            self.get_logger().info(f'Set camera_init_initial at: x={initial_pose[0,3]:.2f}, y={initial_pose[1,3]:.2f}, z={initial_pose[2,3]:.2f}')
            
            initialized = global_localization(initial_pose, self, self.pub_map_to_odom, publish_initial=False)
            if initialized:
                self.get_logger().info('')
                self.get_logger().info('Initialize successfully!!!!!!')
                self.get_logger().info('')
                # 开始定期全局定位
                self.localization_thread = threading.Thread(target=self.thread_localization, daemon=True)
                self.localization_thread.start()
            else:
                self.get_logger().warn('Localization failed. Try a different initial pose or check alignment.')
    
    def thread_localization(self):
        global T_map_to_odom
        rate = self.create_rate(FREQ_LOCALIZATION)
        localization_count = 0
        
        while rclpy.ok():
            try:
                # 每隔一段时间进行全局定位
                rate.sleep()
                localization_count += 1
                # Only log every 10th iteration to reduce spam
                if localization_count % 10 == 1:
                    self.get_logger().info(f'Periodic localization update #{localization_count}...')
                # TODO 由于这里Fast lio发布的scan是已经转换到odom系下了 所以每次全局定位的初始解就是上一次的map2odom 不需要再拿odom了
                # Don't publish initial frame during periodic updates
                global_localization(T_map_to_odom, self, self.pub_map_to_odom, publish_initial=False)
            except Exception as e:
                self.get_logger().error(f'Localization error: {e}')


# Global parameters
MAP_VOXEL_SIZE = 0.4
SCAN_VOXEL_SIZE = 0.1

# Global localization frequency (HZ)
FREQ_LOCALIZATION = 0.5

# The threshold of global localization
LOCALIZATION_TH = 0.95  # Lowered from 0.95 - fitness >0.3 is reasonable for real scans

# FOV(rad), modify this according to your LiDAR type
FOV = 1.6

# The farthest distance(meters) within FOV
FOV_FAR = 300  # Increased to capture more map points for debugging


def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
