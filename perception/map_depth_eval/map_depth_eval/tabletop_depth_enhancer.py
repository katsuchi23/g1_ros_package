from __future__ import annotations

from collections import deque
from typing import Deque, Optional

import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image


class TabletopDepthEnhancer(Node):
    def __init__(self) -> None:
        super().__init__("tabletop_depth_enhancer")

        self.declare_parameter(
            "input_depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter("filtered_depth_topic", "/tabletop_depth/filtered")
        self.declare_parameter("enhanced_viz_topic", "/tabletop_depth/enhanced_viz")
        self.declare_parameter("height_mask_topic", "/tabletop_depth/range_mask")
        self.declare_parameter("d_min", 0.70)
        self.declare_parameter("d_max", 1.00)
        self.declare_parameter("median_kernel", 5)
        self.declare_parameter("bilateral_diameter", 5)
        self.declare_parameter("bilateral_sigma_color", 0.03)
        self.declare_parameter("bilateral_sigma_space", 5.0)
        self.declare_parameter("temporal_window", 3)
        self.declare_parameter("temporal_blend_alpha", 0.4)
        self.declare_parameter("use_inverse_depth_viz", True)
        self.declare_parameter("publish_mask", True)

        self.input_depth_topic = self.get_parameter("input_depth_topic").value
        self.filtered_depth_topic = self.get_parameter("filtered_depth_topic").value
        self.enhanced_viz_topic = self.get_parameter("enhanced_viz_topic").value
        self.height_mask_topic = self.get_parameter("height_mask_topic").value
        self.d_min = float(self.get_parameter("d_min").value)
        self.d_max = float(self.get_parameter("d_max").value)
        self.median_kernel = int(self.get_parameter("median_kernel").value)
        self.bilateral_diameter = int(self.get_parameter("bilateral_diameter").value)
        self.bilateral_sigma_color = float(self.get_parameter("bilateral_sigma_color").value)
        self.bilateral_sigma_space = float(self.get_parameter("bilateral_sigma_space").value)
        self.temporal_window = max(1, int(self.get_parameter("temporal_window").value))
        self.temporal_blend_alpha = float(self.get_parameter("temporal_blend_alpha").value)
        self.use_inverse_depth_viz = bool(self.get_parameter("use_inverse_depth_viz").value)
        self.publish_mask = bool(self.get_parameter("publish_mask").value)

        self.bridge = CvBridge()
        self.temporal_buffer: Deque[np.ndarray] = deque(maxlen=self.temporal_window)
        self.last_filtered_depth: Optional[np.ndarray] = None

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(Image, self.input_depth_topic, self.depth_callback, qos)
        self.filtered_depth_pub = self.create_publisher(Image, self.filtered_depth_topic, 10)
        self.enhanced_viz_pub = self.create_publisher(Image, self.enhanced_viz_topic, 10)
        self.mask_pub = self.create_publisher(Image, self.height_mask_topic, 10)

        self.get_logger().info(
            "Enhancing depth in range "
            f"[{self.d_min:.3f}, {self.d_max:.3f}] m from {self.input_depth_topic}"
        )

    def depth_callback(self, msg: Image) -> None:
        if not rclpy.ok():
            return

        depth_m = self.depth_msg_to_meters(msg)
        if depth_m is None:
            return

        valid = np.isfinite(depth_m) & (depth_m > 0.0)
        clipped = np.zeros_like(depth_m, dtype=np.float32)
        in_range = valid & (depth_m >= self.d_min) & (depth_m <= self.d_max)
        clipped[in_range] = depth_m[in_range]

        filtered = self.apply_filters(clipped)
        viz = self.make_visualization(filtered)

        filtered_msg = self.bridge.cv2_to_imgmsg(filtered, encoding="32FC1")
        filtered_msg.header = msg.header
        try:
            self.filtered_depth_pub.publish(filtered_msg)
        except Exception:
            return

        viz_msg = self.bridge.cv2_to_imgmsg(viz, encoding="mono8")
        viz_msg.header = msg.header
        try:
            self.enhanced_viz_pub.publish(viz_msg)
        except Exception:
            return

        if self.publish_mask:
            mask = np.zeros_like(viz, dtype=np.uint8)
            mask[filtered > 0.0] = 255
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            mask_msg.header = msg.header
            try:
                self.mask_pub.publish(mask_msg)
            except Exception:
                return

    def depth_msg_to_meters(self, msg: Image) -> Optional[np.ndarray]:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().error(f"Failed to convert depth image: {exc}")
            return None

        if msg.encoding == "16UC1":
            return depth.astype(np.float32) * 0.001
        if msg.encoding == "32FC1":
            return depth.astype(np.float32)

        self.get_logger().warn(f"Unsupported depth encoding: {msg.encoding}")
        return None

    def apply_filters(self, depth_m: np.ndarray) -> np.ndarray:
        filtered = depth_m.copy()

        if self.median_kernel >= 3 and self.median_kernel % 2 == 1:
            filtered_mm = np.round(filtered * 1000.0).astype(np.uint16)
            filtered_mm = cv2.medianBlur(filtered_mm, self.median_kernel)
            filtered = filtered_mm.astype(np.float32) * 0.001
            filtered[depth_m <= 0.0] = 0.0

        if self.bilateral_diameter > 1:
            valid_mask = filtered > 0.0
            bilateral_input = filtered.copy()
            bilateral_input[~valid_mask] = self.d_max
            filtered = cv2.bilateralFilter(
                bilateral_input,
                d=self.bilateral_diameter,
                sigmaColor=self.bilateral_sigma_color,
                sigmaSpace=self.bilateral_sigma_space,
            )
            filtered[~valid_mask] = 0.0

        valid_pixels = filtered > 0.0
        if np.any(valid_pixels):
            self.temporal_buffer.append(filtered.copy())
            averaged = np.mean(np.stack(self.temporal_buffer, axis=0), axis=0)
            filtered[valid_pixels] = averaged[valid_pixels]

            if self.last_filtered_depth is not None:
                last_valid = self.last_filtered_depth > 0.0
                blend_mask = valid_pixels & last_valid
                filtered[blend_mask] = (
                    self.temporal_blend_alpha * filtered[blend_mask]
                    + (1.0 - self.temporal_blend_alpha) * self.last_filtered_depth[blend_mask]
                )

        self.last_filtered_depth = filtered.copy()
        filtered[(filtered < self.d_min) | (filtered > self.d_max)] = 0.0
        return filtered

    def make_visualization(self, depth_m: np.ndarray) -> np.ndarray:
        viz = np.zeros(depth_m.shape, dtype=np.uint8)
        valid = depth_m > 0.0
        if not np.any(valid):
            return viz

        if self.use_inverse_depth_viz:
            inv = np.zeros_like(depth_m, dtype=np.float32)
            inv[valid] = 1.0 / np.maximum(depth_m[valid], 1e-6)
            inv_min = 1.0 / self.d_max
            inv_max = 1.0 / self.d_min
            scaled = (inv - inv_min) / max(inv_max - inv_min, 1e-6)
        else:
            scaled = (self.d_max - depth_m) / max(self.d_max - self.d_min, 1e-6)

        scaled = np.clip(scaled, 0.0, 1.0)
        viz[valid] = np.round(scaled[valid] * 255.0).astype(np.uint8)
        return viz


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TabletopDepthEnhancer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
