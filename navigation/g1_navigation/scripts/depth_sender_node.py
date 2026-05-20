#!/usr/bin/env python3
import threading
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
import zmq


def _resize_nearest(depth: np.ndarray, out_h: int = 360, out_w: int = 640) -> np.ndarray:
    in_h, in_w = depth.shape[:2]
    if (in_h, in_w) == (out_h, out_w):
        return depth

    y_idx = np.linspace(0, in_h - 1, out_h).astype(np.int32)
    x_idx = np.linspace(0, in_w - 1, out_w).astype(np.int32)
    return depth[np.ix_(y_idx, x_idx)]


class DepthSenderNode(Node):
    def __init__(self):
        super().__init__("depth_sender_node")

        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("zmq_endpoint", "tcp://127.0.0.1:5561")
        self.declare_parameter("depth_scale", 0.001)

        self._depth_topic = str(self.get_parameter("depth_topic").value)
        self._endpoint = str(self.get_parameter("zmq_endpoint").value)
        self._depth_scale = float(self.get_parameter("depth_scale").value)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_depth: Optional[bytes] = None

        self.create_subscription(Image, self._depth_topic, self._on_depth, 10)

        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.REP)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.bind(self._endpoint)

        self.create_timer(0.01, self._poll_requests)
        self.create_timer(2.0, self._warn_if_empty)

        self.get_logger().info(
            f"DepthSenderNode subscribed to {self._depth_topic}, serving float32 depth on {self._endpoint}"
        )

    def _on_depth(self, msg: Image) -> None:
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            depth = np.asarray(depth)

            if depth.ndim != 2:
                raise ValueError(f"Expected a single-channel depth image, got shape {depth.shape}")

            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) * self._depth_scale
            else:
                depth = depth.astype(np.float32, copy=False)

            depth = _resize_nearest(depth, 360, 640)

            with self._lock:
                self._latest_depth = depth.astype(np.float32, copy=False).tobytes(order="C")
        except Exception as exc:
            self.get_logger().error(f"Failed to convert depth image: {exc}")

    def _poll_requests(self) -> None:
        try:
            if self._sock.poll(timeout=0) == 0:
                return

            _ = self._sock.recv_string()
            with self._lock:
                payload = self._latest_depth or b""
            self._sock.send(payload)
        except Exception as exc:
            self.get_logger().error(f"Depth REP error: {exc}")
            try:
                self._sock.send(b"")
            except Exception:
                pass

    def _warn_if_empty(self) -> None:
        with self._lock:
            has_depth = self._latest_depth is not None
        if not has_depth:
            self.get_logger().warn(f"No depth image received yet on {self._depth_topic}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
