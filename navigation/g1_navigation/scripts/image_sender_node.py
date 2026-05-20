#!/usr/bin/env python3
import io
import threading
from typing import Optional

import numpy as np
from PIL import Image as PILImage
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
import zmq


class ImageSenderNode(Node):
    def __init__(self):
        super().__init__("image_sender_node")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("zmq_endpoint", "tcp://127.0.0.1:5555")
        self.declare_parameter("jpeg_quality", 90)

        self._image_topic = str(self.get_parameter("image_topic").value)
        self._endpoint = str(self.get_parameter("zmq_endpoint").value)
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_jpeg: Optional[bytes] = None

        self.create_subscription(Image, self._image_topic, self._on_image, 10)

        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.REP)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.bind(self._endpoint)

        self.create_timer(0.01, self._poll_requests)
        self.create_timer(2.0, self._warn_if_empty)

        self.get_logger().info(
            f"ImageSenderNode subscribed to {self._image_topic}, serving JPEG frames on {self._endpoint}"
        )

    def _on_image(self, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            rgb = bgr[..., ::-1]
            image = PILImage.fromarray(rgb.astype(np.uint8, copy=False), mode="RGB")

            buf = io.BytesIO()
            image.save(buf, format="JPEG", quality=self._jpeg_quality)

            with self._lock:
                self._latest_jpeg = buf.getvalue()
        except Exception as exc:
            self.get_logger().error(f"Failed to convert RGB image: {exc}")

    def _poll_requests(self) -> None:
        try:
            if self._sock.poll(timeout=0) == 0:
                return

            _ = self._sock.recv_string()
            with self._lock:
                payload = self._latest_jpeg or b""
            self._sock.send(payload)
        except Exception as exc:
            self.get_logger().error(f"Image REP error: {exc}")
            try:
                self._sock.send(b"")
            except Exception:
                pass

    def _warn_if_empty(self) -> None:
        with self._lock:
            has_image = self._latest_jpeg is not None
        if not has_image:
            self.get_logger().warn(f"No image received yet on {self._image_topic}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
