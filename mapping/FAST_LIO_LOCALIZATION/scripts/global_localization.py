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

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

global_map = None
initialized = False
T_map_to_odom = np.eye(4)
cur_odom = None
cur_scan = None


def pose_to_mat(pose_msg):
    """Convert pose message to 4x4 transformation matrix"""
    pos = pose_msg.pose.pose.position
    ori = pose_msg.pose.pose.orientation
    
    # Create translation matrix
    trans_mat = np.eye(4)
    trans_mat[0:3, 3] = [pos.x, pos.y, pos.z]
    
    # Create rotation matrix from quaternion
    rot = R.from_quat([ori.x, ori.y, ori.z, ori.w])
    rot_mat = np.eye(4)
    rot_mat[0:3, 0:3] = rot.as_matrix()
    
    return trans_mat @ rot_mat


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
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
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
        points_list.append([point[0], point[1], point[2], 0.0])  # x, y, z, intensity
    
    msg = pc2.create_cloud_xyz32(header, points_list[:, :3])
    publisher.publish(msg)


def crop_global_map_in_FOV(global_map, pose_estimation, cur_odom, pub_submap):
    # 当前scan原点的位姿
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

    # 发布fov内点云
    header = cur_odom.header
    header.frame_id = 'map'
    publish_point_cloud(pub_submap, header, np.array(global_map_in_FOV.points)[::10])

    return global_map_in_FOV


def global_localization(pose_estimation, node, pub_map_to_odom):
    global global_map, cur_scan, cur_odom, T_map_to_odom
    
    node.get_logger().info('Global localization by scan-to-map matching......')

    # TODO 这里注意线程安全
    scan_tobe_mapped = copy.copy(cur_scan)

    tic = time.time()

    global_map_in_FOV = crop_global_map_in_FOV(global_map, pose_estimation, cur_odom, node.pub_submap)

    # 粗配准
    transformation, _ = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)

    # 精配准
    transformation, fitness = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=transformation, scale=1)
    
    toc = time.time()
    node.get_logger().info(f'Time: {toc - tic}')
    node.get_logger().info('')

    # 当全局定位成功时才更新map2odom
    if fitness > LOCALIZATION_TH:
        T_map_to_odom = transformation

        # 发布map_to_odom
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
        return True
    else:
        node.get_logger().warn('Not match!!!!')
        node.get_logger().warn(f'{transformation}')
        node.get_logger().warn(f'fitness score:{fitness}')
        return False


def voxel_down_sample(pcd, voxel_size):
    return pcd.voxel_down_sample(voxel_size)


class GlobalLocalizationNode(Node):
    def __init__(self):
        super().__init__('fast_lio_localization')
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.pub_pc_in_map = self.create_publisher(PointCloud2, '/cur_scan_in_map', 1)
        self.pub_submap = self.create_publisher(PointCloud2, '/submap', 1)
        self.pub_map_to_odom = self.create_publisher(Odometry, '/map_to_odom', 1)
        
        # Subscribers
        self.sub_scan = self.create_subscription(
            PointCloud2, '/cloud_registered', self.cb_save_cur_scan, sensor_qos)
        self.sub_odom = self.create_subscription(
            Odometry, '/Odometry', self.cb_save_cur_odom, sensor_qos)
        self.sub_map = self.create_subscription(
            PointCloud2, '/map', self.initialize_global_map, sensor_qos)
        self.sub_initialpose = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.cb_initialpose, 10)
        
        self.get_logger().info('Localization Node Inited...')
        self.get_logger().warn('Waiting for global map......')
        
        self.localization_thread = None
        
    def initialize_global_map(self, pc_msg):
        global global_map
        
        global_map = o3d.geometry.PointCloud()
        pc_array = msg_to_array(pc_msg)
        global_map.points = o3d.utility.Vector3dVector(pc_array[:, :3])
        global_map = voxel_down_sample(global_map, MAP_VOXEL_SIZE)
        self.get_logger().info('Global map received.')
    
    def cb_save_cur_odom(self, odom_msg):
        global cur_odom
        cur_odom = odom_msg
    
    def cb_save_cur_scan(self, pc_msg):
        global cur_scan
        
        # 注意这里fastlio直接将scan转到odom系下了 不是lidar局部系
        pc_msg.header.frame_id = 'camera_init'
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_pc_in_map.publish(pc_msg)
        
        # 转换为pcd
        pc = msg_to_array(pc_msg)
        cur_scan = o3d.geometry.PointCloud()
        cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3])
    
    def cb_initialpose(self, pose_msg):
        global initialized, T_map_to_odom
        
        if not initialized:
            self.get_logger().warn('Waiting for initial pose....')
            
            initial_pose = pose_to_mat(pose_msg)
            if cur_scan and global_map:
                initialized = global_localization(initial_pose, self, self.pub_map_to_odom)
                if initialized:
                    self.get_logger().info('')
                    self.get_logger().info('Initialize successfully!!!!!!')
                    self.get_logger().info('')
                    # 开始定期全局定位
                    self.localization_thread = threading.Thread(target=self.thread_localization, daemon=True)
                    self.localization_thread.start()
            else:
                self.get_logger().warn('First scan or map not received!!!!!')
    
    def thread_localization(self):
        global T_map_to_odom
        rate = self.create_rate(FREQ_LOCALIZATION)
        
        while rclpy.ok():
            try:
                # 每隔一段时间进行全局定位
                rate.sleep()
                # TODO 由于这里Fast lio发布的scan是已经转换到odom系下了 所以每次全局定位的初始解就是上一次的map2odom 不需要再拿odom了
                global_localization(T_map_to_odom, self, self.pub_map_to_odom)
            except Exception as e:
                self.get_logger().error(f'Localization error: {e}')


# Global parameters
MAP_VOXEL_SIZE = 0.4
SCAN_VOXEL_SIZE = 0.1

# Global localization frequency (HZ)
FREQ_LOCALIZATION = 0.5

# The threshold of global localization
LOCALIZATION_TH = 0.95

# FOV(rad), modify this according to your LiDAR type
FOV = 1.6

# The farthest distance(meters) within FOV
FOV_FAR = 150


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
