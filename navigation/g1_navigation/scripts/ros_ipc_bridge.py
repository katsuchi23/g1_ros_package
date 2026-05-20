#!/usr/bin/env python3

"""ROS2 -> ZMQ bridge for semantic-nav IPC clients.

Bridges:
- TF pose bundle (PUB):   tcp://127.0.0.1:5557, topic prefix "TF "
- Local costmap (REP):    tcp://127.0.0.1:5564, request "GET"
- Global costmap (REP):   tcp://127.0.0.1:5565, request "GET"
- CmdVel command (REP):   tcp://127.0.0.1:5559, payload {"v": <m/s>, "w": <rad/s>}
"""

from __future__ import annotations

import json
import math
import time
from typing import Dict, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

import tf2_ros

try:
    import zmq
except Exception as exc:  # pragma: no cover
    raise RuntimeError(
        "pyzmq is required for ros_ipc_bridge.py (pip install pyzmq)."
    ) from exc


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class RosIpcBridge(Node):
    def __init__(self) -> None:
        super().__init__("ros_ipc_bridge")

        # Frames
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_link")

        # ROS topics
        self.declare_parameter("local_costmap_topic", "/local_costmap/costmap")
        self.declare_parameter("global_costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        # ZMQ endpoints
        self.declare_parameter("pose_pub_endpoint", "tcp://127.0.0.1:5557")
        self.declare_parameter("local_costmap_rep_endpoint", "tcp://127.0.0.1:5564")
        self.declare_parameter("global_costmap_rep_endpoint", "tcp://127.0.0.1:5565")
        self.declare_parameter("cmd_vel_rep_endpoint", "tcp://127.0.0.1:5559")

        # Rates
        self.declare_parameter("pose_pub_hz", 50.0)
        self.declare_parameter("rep_poll_hz", 100.0)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)

        self.local_costmap_topic = str(self.get_parameter("local_costmap_topic").value)
        self.global_costmap_topic = str(self.get_parameter("global_costmap_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        pose_pub_endpoint = str(self.get_parameter("pose_pub_endpoint").value)
        local_rep_endpoint = str(self.get_parameter("local_costmap_rep_endpoint").value)
        global_rep_endpoint = str(self.get_parameter("global_costmap_rep_endpoint").value)
        cmd_vel_rep_endpoint = str(self.get_parameter("cmd_vel_rep_endpoint").value)

        pose_pub_hz = float(self.get_parameter("pose_pub_hz").value)
        rep_poll_hz = float(self.get_parameter("rep_poll_hz").value)

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Costmap state
        self._latest_local: Optional[OccupancyGrid] = None
        self._latest_global: Optional[OccupancyGrid] = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, self.local_costmap_topic, self._on_local_costmap, qos)
        self.create_subscription(OccupancyGrid, self.global_costmap_topic, self._on_global_costmap, qos)
        self._cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # ZMQ
        self._ctx = zmq.Context.instance()

        self._pose_pub = self._ctx.socket(zmq.PUB)
        self._pose_pub.setsockopt(zmq.LINGER, 0)
        self._pose_pub.setsockopt(zmq.SNDHWM, 1)
        self._pose_pub.bind(pose_pub_endpoint)

        self._local_rep = self._ctx.socket(zmq.REP)
        self._local_rep.setsockopt(zmq.LINGER, 0)
        self._local_rep.bind(local_rep_endpoint)

        self._global_rep = self._ctx.socket(zmq.REP)
        self._global_rep.setsockopt(zmq.LINGER, 0)
        self._global_rep.bind(global_rep_endpoint)

        self._cmd_vel_rep = self._ctx.socket(zmq.REP)
        self._cmd_vel_rep.setsockopt(zmq.LINGER, 0)
        self._cmd_vel_rep.bind(cmd_vel_rep_endpoint)

        self._poller = zmq.Poller()
        self._poller.register(self._local_rep, zmq.POLLIN)
        self._poller.register(self._global_rep, zmq.POLLIN)
        self._poller.register(self._cmd_vel_rep, zmq.POLLIN)

        self.create_timer(max(0.001, 1.0 / max(1e-3, pose_pub_hz)), self._publish_pose_bundle)
        self.create_timer(max(0.001, 1.0 / max(1e-3, rep_poll_hz)), self._poll_rep_requests)

        self._last_tf_warn_ns = 0
        self._last_map_base_stamp = None
        self._map_base_frozen_since = None
        self._last_tf_stale_warn = 0.0

        self.get_logger().info(
            "ros_ipc_bridge started: "
            f"pose_pub={pose_pub_endpoint}, "
            f"local_rep={local_rep_endpoint}({self.local_costmap_topic}), "
            f"global_rep={global_rep_endpoint}({self.global_costmap_topic}), "
            f"cmd_vel_rep={cmd_vel_rep_endpoint}->{self.cmd_vel_topic}, "
            f"frames=({self.map_frame},{self.odom_frame},{self.base_frame},{self.camera_frame})"
        )

    def destroy_node(self) -> bool:
        for s in (
            getattr(self, "_pose_pub", None),
            getattr(self, "_local_rep", None),
            getattr(self, "_global_rep", None),
            getattr(self, "_cmd_vel_rep", None),
        ):
            if s is not None:
                try:
                    s.close(0)
                except Exception:
                    pass
        return super().destroy_node()

    def _on_local_costmap(self, msg: OccupancyGrid) -> None:
        self._latest_local = msg

    def _on_global_costmap(self, msg: OccupancyGrid) -> None:
        self._latest_global = msg

    def _lookup_tf(self, parent: str, child: str) -> Optional[Dict[str, object]]:
        try:
            tf_msg = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
        except Exception:
            return None

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        stamp = tf_msg.header.stamp
        return {
            "x": float(t.x),
            "y": float(t.y),
            "z": float(t.z),
            "yaw": float(_yaw_from_quat(q.x, q.y, q.z, q.w)),
            "stamp_sec": int(stamp.sec),
            "stamp_nanosec": int(stamp.nanosec),
            "parent": str(tf_msg.header.frame_id).lstrip("/"),
            "child": str(tf_msg.child_frame_id).lstrip("/"),
        }

    def _publish_pose_bundle(self) -> None:
        map_base = self._lookup_tf(self.map_frame, self.base_frame)
        map_odom = self._lookup_tf(self.map_frame, self.odom_frame)
        odom_base = self._lookup_tf(self.odom_frame, self.base_frame)
        map_camera = self._lookup_tf(self.map_frame, self.camera_frame)

        if map_base is None:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_tf_warn_ns > 2_000_000_000:
                self.get_logger().warn(
                    f"No TF yet for {self.map_frame}->{self.base_frame}; pose bundle will contain null map_base."
                )
                self._last_tf_warn_ns = now_ns

        payload = {
            "bundle_wall_time": time.time(),
            "map_frame": self.map_frame,
            "odom_frame": self.odom_frame,
            "base_frame": self.base_frame,
            "camera_frame": self.camera_frame,
            "tfs": {
                "map_base": map_base,
                "map_odom": map_odom,
                "odom_base": odom_base,
                "map_camera": map_camera,
            },
        }

        if map_base is not None:
            map_base_stamp = (int(map_base["stamp_sec"]), int(map_base["stamp_nanosec"]))
            now = time.time()
            if map_base_stamp == self._last_map_base_stamp:
                if self._map_base_frozen_since is None:
                    self._map_base_frozen_since = now
                elif (now - self._map_base_frozen_since) >= 0.25 and (now - self._last_tf_stale_warn) >= 1.0:
                    self.get_logger().warn(
                        "map_base TF stamp is not advancing; "
                        f"frozen_for={now - self._map_base_frozen_since:.2f}s stamp={map_base_stamp}"
                    )
                    self._last_tf_stale_warn = now
            else:
                self._last_map_base_stamp = map_base_stamp
                self._map_base_frozen_since = None

        try:
            self._pose_pub.send_string("TF " + json.dumps(payload, separators=(",", ":")), flags=zmq.NOBLOCK)
        except zmq.Again:
            pass

    @staticmethod
    def _encode_costmap(msg: Optional[OccupancyGrid]):
        if msg is None:
            meta = {"ok": False, "error": "no_costmap"}
            return [json.dumps(meta).encode("utf-8")]

        w = int(msg.info.width)
        h = int(msg.info.height)
        if w <= 0 or h <= 0:
            meta = {"ok": False, "error": "empty_costmap", "width": w, "height": h}
            return [json.dumps(meta).encode("utf-8")]

        arr = np.asarray(msg.data, dtype=np.int8)
        if arr.size != w * h:
            meta = {
                "ok": False,
                "error": "bad_costmap_size",
                "width": w,
                "height": h,
                "size": int(arr.size),
            }
            return [json.dumps(meta).encode("utf-8")]

        stamp = msg.header.stamp
        meta = {
            "ok": True,
            "width": w,
            "height": h,
            "dtype": "int8",
            "resolution": float(msg.info.resolution),
            "origin_x": float(msg.info.origin.position.x),
            "origin_y": float(msg.info.origin.position.y),
            "frame_id": str(msg.header.frame_id).lstrip("/"),
            "stamp_sec": int(stamp.sec),
            "stamp_nanosec": int(stamp.nanosec),
        }
        return [json.dumps(meta, separators=(",", ":")).encode("utf-8"), arr.tobytes(order="C")]

    def _poll_rep_requests(self) -> None:
        events = dict(self._poller.poll(timeout=0))

        if self._local_rep in events and events[self._local_rep] & zmq.POLLIN:
            try:
                _ = self._local_rep.recv_string(flags=zmq.NOBLOCK)
            except Exception:
                return
            self._local_rep.send_multipart(self._encode_costmap(self._latest_local))

        if self._global_rep in events and events[self._global_rep] & zmq.POLLIN:
            try:
                _ = self._global_rep.recv_string(flags=zmq.NOBLOCK)
            except Exception:
                return
            self._global_rep.send_multipart(self._encode_costmap(self._latest_global))

        if self._cmd_vel_rep in events and events[self._cmd_vel_rep] & zmq.POLLIN:
            try:
                req = self._cmd_vel_rep.recv_string(flags=zmq.NOBLOCK)
            except Exception:
                return

            try:
                data = json.loads(req)
                v = float(data.get("v", 0.0))
                w = float(data.get("w", 0.0))

                msg = Twist()
                msg.linear.x = v
                msg.angular.z = w
                self._cmd_vel_pub.publish(msg)

                self._cmd_vel_rep.send_string("OK")
            except Exception as exc:
                self.get_logger().warn(f"Invalid cmd_vel IPC payload: {exc}")
                self._cmd_vel_rep.send_string("ERR")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RosIpcBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
