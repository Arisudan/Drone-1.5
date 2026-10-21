#!/usr/bin/env python3
"""
================================================================================
MODULE: lio_map_receiver.py
PURPOSE: Jetson Point-LIO map stream (TCP) -> local ROS 2 topics for the embedded RViz2
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       the GCS laptop, as a subprocess the Tactical SLAM tab starts with RViz2
  * Upstream:      jetson/lio_map_stream.py (mavlink_radiomaster) on tcp <jetson>:5766
  * Downstream:    config/pointlio_3d.rviz, on a PRIVATE localhost-only ROS domain

WHY TCP AND A PRIVATE DOMAIN:
  The Jetson's flight chain is ROS_LOCALHOST_ONLY on purpose - Wi-Fi discovery must not reach
  the estimator feeding EKF2 - so the map crosses the link as a compact TCP stream instead
  (the same reason the 2D map uses tcp 5765). Here it is republished on ROS_DOMAIN_ID 73 with
  discovery limited to this machine, so neither this node nor RViz2 can merge with another ROS
  stack on the lab network (a foreign map->odom once poisoned TF for a whole flight).

TOPICS (frame lio_world = Point-LIO's gravity-aligned start frame, z up):
  /lio/map    PointCloud2  accumulated voxel map, transient-local, re-sent on change (1 Hz)
  /lio/scan   PointCloud2  latest registered scan, thinned
  /lio/odom   Odometry     L2 body pose
  /lio/path   Path         flown track (a point every 5 cm)
  TF          lio_world -> lio_body (L2) -> base_link (airframe; L2 mount yaw undone)

RUN:  python3 lio_map_receiver.py --host 10.42.0.1 [--port 5766]
Wire format: see jetson/lio_map_stream.py.
================================================================================
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import threading
import time

import numpy as np

MAGIC = b"LIO1"
T_POSE, T_SCAN, T_MAP_ADD, T_HELLO, T_RESET = 1, 2, 3, 4, 5
GCS_ROS_DOMAIN = 73


def parse_frames(buf: bytearray):
    """Yield (type, payload) for every complete frame in buf and drop them from it.
    Resynchronises on the magic if the stream is ever corrupted."""
    out = []
    while True:
        i = buf.find(MAGIC)
        if i < 0:
            del buf[:max(0, len(buf) - 3)]
            return out
        if i:
            del buf[:i]
        if len(buf) < 9:
            return out
        ftype, n = struct.unpack_from("<BI", buf, 4)
        if len(buf) < 9 + n:
            return out
        out.append((ftype, bytes(buf[9:9 + n])))
        del buf[:9 + n]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[3])
    ap.add_argument("--host", default="10.42.0.1")
    ap.add_argument("--port", type=int, default=5766)
    a = ap.parse_args()

    import rclpy
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav_msgs.msg import Odometry, Path
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    rclpy.init()
    node = rclpy.create_node("lio_map_receiver")
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub_map = node.create_publisher(PointCloud2, "/lio/map", latched)
    pub_scan = node.create_publisher(PointCloud2, "/lio/scan", 5)
    pub_odom = node.create_publisher(Odometry, "/lio/odom", 10)
    pub_path = node.create_publisher(Path, "/lio/path", latched)
    tfb, stfb = TransformBroadcaster(node), StaticTransformBroadcaster(node)
    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate("xyz")]

    state = {"map": [], "map_dirty": False, "path": Path(), "last_path": None, "frame": "lio_world"}
    state["path"].header.frame_id = "lio_world"
    lock = threading.Lock()

    def cloud(pts: np.ndarray, stamp) -> PointCloud2:
        m = PointCloud2()
        m.header.frame_id = state["frame"]
        m.header.stamp = stamp
        m.height, m.width = 1, len(pts)
        m.fields, m.is_bigendian, m.point_step = fields, False, 12
        m.row_step, m.is_dense = 12 * len(pts), True
        m.data = pts.astype(np.float32).tobytes()
        return m

    def static_mount(yaw_deg: float):
        t = TransformStamped()
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "lio_body", "base_link"
        h = -math.radians(yaw_deg) / 2.0                 # airframe = L2 body turned back
        t.transform.rotation.z, t.transform.rotation.w = math.sin(h), math.cos(h)
        stfb.sendTransform(t)

    def on_frame(ftype, payload):
        now = node.get_clock().now().to_msg()
        if ftype == T_HELLO:
            hello = json.loads(payload.decode())
            state["frame"] = hello.get("frame", "lio_world")
            static_mount(float(hello.get("mount_yaw_deg", 0.0)))
            node.get_logger().info(f"connected: {hello}")
        elif ftype == T_RESET:
            with lock:
                state["map"], state["map_dirty"] = [], True
                state["path"].poses, state["last_path"] = [], None
            node.get_logger().warn("Point-LIO restarted on the drone - map cleared")
        elif ftype == T_MAP_ADD:
            pts = np.frombuffer(payload, np.float32).reshape(-1, 3)
            with lock:
                state["map"].append(pts)
                state["map_dirty"] = True
        elif ftype == T_SCAN:
            pts = np.frombuffer(payload[8:], np.float32).reshape(-1, 3)
            pub_scan.publish(cloud(pts, now))
        elif ftype == T_POSE:
            _, x, y, z, qx, qy, qz, qw = struct.unpack("<d3f4f", payload)
            o = Odometry()
            o.header.stamp, o.header.frame_id, o.child_frame_id = now, state["frame"], "lio_body"
            o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = x, y, z
            q = o.pose.pose.orientation
            q.x, q.y, q.z, q.w = qx, qy, qz, qw
            pub_odom.publish(o)
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = now, state["frame"], "lio_body"
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
            t.transform.rotation = q
            tfb.sendTransform(t)
            lp = state["last_path"]
            if lp is None or math.dist(lp, (x, y, z)) > 0.05:
                ps = PoseStamped()
                ps.header = o.header
                ps.pose = o.pose.pose
                with lock:
                    state["path"].poses.append(ps)
                    state["last_path"] = (x, y, z)

    def publish_map():
        with lock:
            if not state["map_dirty"]:
                return
            pts = np.concatenate(state["map"]) if state["map"] else np.zeros((0, 3), np.float32)
            state["map_dirty"] = False
            path = Path()
            path.header.frame_id = state["frame"]
            path.poses = list(state["path"].poses)        # copy: the net thread appends
        now = node.get_clock().now().to_msg()
        pub_map.publish(cloud(pts, now))
        path.header.stamp = now
        pub_path.publish(path)

    node.create_timer(1.0, publish_map)

    def net_loop():
        while rclpy.ok():
            try:
                s = socket.create_connection((a.host, a.port), timeout=3.0)
                s.settimeout(5.0)
                node.get_logger().info(f"streaming from {a.host}:{a.port}")
                buf = bytearray()
                while rclpy.ok():
                    chunk = s.recv(1 << 16)
                    if not chunk:
                        raise ConnectionError("closed by the drone")
                    buf += chunk
                    for ftype, payload in parse_frames(buf):
                        on_frame(ftype, payload)
            except (OSError, ConnectionError) as e:
                node.get_logger().warn(f"no map stream from {a.host}:{a.port} ({e}) - retrying",
                                       throttle_duration_sec=10.0)
                time.sleep(2.0)

    threading.Thread(target=net_loop, daemon=True).start()
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
