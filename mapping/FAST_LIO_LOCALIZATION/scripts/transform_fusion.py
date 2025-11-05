#!/usr/bin/env python3
# coding=utf8

import copy
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

cur_odom_to_baselink = None
cur_map_to_odom = None


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


class TransformFusionNode(Node):
    def __init__(self):
        super().__init__('transform_fusion')
        
        # tf and localization publishing frequency (HZ)
        self.FREQ_PUB_LOCALIZATION = 50
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.pub_localization = self.create_publisher(Odometry, '/localization', 1)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.sub_odom = self.create_subscription(
            Odometry, '/Odometry', self.cb_save_cur_odom, sensor_qos)
        self.sub_map_to_odom = self.create_subscription(
            Odometry, '/map_to_odom', self.cb_save_map_to_odom, sensor_qos)
        
        self.get_logger().info('Transform Fusion Node Inited...')
        
        # Start fusion thread
        self.fusion_thread = threading.Thread(target=self.transform_fusion, daemon=True)
        self.fusion_thread.start()
    
    def cb_save_cur_odom(self, odom_msg):
        global cur_odom_to_baselink
        cur_odom_to_baselink = odom_msg
    
    def cb_save_map_to_odom(self, odom_msg):
        global cur_map_to_odom
        cur_map_to_odom = odom_msg
    
    def transform_fusion(self):
        rate = self.create_rate(self.FREQ_PUB_LOCALIZATION)
        
        while rclpy.ok():
            try:
                # TODO 这里注意线程安全
                cur_odom = copy.copy(cur_odom_to_baselink)
                if cur_map_to_odom is not None:
                    T_map_to_odom = pose_to_mat(cur_map_to_odom)
                else:
                    T_map_to_odom = np.eye(4)
                
                # DISABLED: Broadcast TF (uncomment to enable)
                # This is disabled so AMCL can publish map->odom transform instead
                # rot = R.from_matrix(T_map_to_odom[:3, :3])
                # quat = rot.as_quat()  # [x, y, z, w]
                
                # t = TransformStamped()
                # t.header.stamp = self.get_clock().now().to_msg()
                # t.header.frame_id = 'map'
                # t.child_frame_id = 'camera_init'
                # t.transform.translation.x = T_map_to_odom[0, 3]
                # t.transform.translation.y = T_map_to_odom[1, 3]
                # t.transform.translation.z = T_map_to_odom[2, 3]
                # t.transform.rotation.x = quat[0]
                # t.transform.rotation.y = quat[1]
                # t.transform.rotation.z = quat[2]
                # t.transform.rotation.w = quat[3]
                
                # self.tf_broadcaster.sendTransform(t)
                
                # Publish localization odometry (ENABLED)
                if cur_odom is not None:
                    # 发布全局定位的odometry
                    localization = Odometry()
                    T_odom_to_base_link = pose_to_mat(cur_odom)
                    # 这里T_map_to_odom短时间内变化缓慢 暂时不考虑与T_odom_to_base_link时间同步
                    T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)
                    
                    rot = R.from_matrix(T_map_to_base_link[:3, :3])
                    quat = rot.as_quat()  # [x, y, z, w]
                    
                    localization.pose.pose = Pose(
                        position=Point(
                            x=T_map_to_base_link[0, 3],
                            y=T_map_to_base_link[1, 3],
                            z=T_map_to_base_link[2, 3]
                        ),
                        orientation=Quaternion(
                            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
                        )
                    )
                    localization.twist = cur_odom.twist
                    
                    localization.header.stamp = cur_odom.header.stamp
                    localization.header.frame_id = 'map'
                    localization.child_frame_id = 'body'
                    
                    self.pub_localization.publish(localization)
                
                rate.sleep()
                
            except Exception as e:
                self.get_logger().error(f'Transform fusion error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TransformFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
