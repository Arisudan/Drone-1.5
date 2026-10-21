#!/usr/bin/env python3
"""
================================================================================
MODULE: obstacle_distance_bridge.py
PURPOSE: Raycasts 2D SLAM Grid into 360° MAVLink OBSTACLE_DISTANCE for Collision Avoidance
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Radxa Q6A Companion Computer (Onboard System)
  * Upstream:      rtabmap_slam (/map) and rtabmap_odom (/odom)
  * Downstream:    px4_control.py obstacle safety loop & Pixhawk (UDP 127.0.0.1:14541)

DATA FLOW & INTERFACES:
  * Subscribes To: /map [nav_msgs/OccupancyGrid] (2D SLAM barrier map)
                   /odom [nav_msgs/Odometry] (Current drone pose and yaw)
  * MAVLink Out:   OBSTACLE_DISTANCE (msg ID 330) in MAV_FRAME_BODY_FRD
  * Sector Count:  72 sectors (5.0° angular increment per sector across 360°)

KEY ALGORITHMIC PIPELINE:
  1. Virtual Lidar Synthesis: Simulates a 360-degree laser rangefinder without physical
     hardware by Bresenham raycasting from the drone's current pose through the /map grid.
  2. Coordinate Conversion: ROS yaw (CCW from +X) is mapped to MAVLink Body-FRD
     (Index 0 = Nose forward, positive clockwise).
  3. Distance Clamping: Distances mapped to centimeters [min_dist_cm, max_dist_cm]
     with UINT16_MAX representing clear / out-of-range space.

RUN:
  source /opt/ros/jazzy/setup.bash
  python3 obstacle_distance_bridge.py --dest udpout:127.0.0.1:14541 --rate 10
================================================================================
"""
import math
import os

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import OccupancyGrid, Odometry

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

N_SECTORS = 72                  # fixed by the OBSTACLE_DISTANCE message
UINT16_MAX = 65535


class ObstacleDistanceBridge(Node):
    def __init__(self):
        super().__init__("obstacle_distance_bridge")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("sink_urls", "udpout:127.0.0.1:14541")
        self.declare_parameter("min_distance_m", 0.2)
        self.declare_parameter("max_distance_m", 8.0)     # RTAB-Map grid's own useful range, not a 20 m lidar's
        self.declare_parameter("occupancy_threshold", 65)  # matches GridGlobal/OccupancyThr=0.65 used elsewhere in this project
        self.declare_parameter("ray_step_m", 0.05)         # sub-cell stepping so a thin wall isn't skipped
        self.declare_parameter("rate_hz", 5.0)             # cap publish rate independent of /map's own rate

        gp = self.get_parameter
        self.min_m = float(gp("min_distance_m").value)
        self.max_m = float(gp("max_distance_m").value)
        self.min_cm = int(self.min_m * 100.0)
        self.max_cm = int(self.max_m * 100.0)
        self.occ_thr = int(gp("occupancy_threshold").value)
        self.ray_step = float(gp("ray_step_m").value)
        self.inc_deg = 360.0 / N_SECTORS
        self.clear = np.uint16(min(self.max_cm + 1, UINT16_MAX - 1))
        self.min_dt = 1.0 / max(1e-3, float(gp("rate_hz").value))

        # Best-effort sinks: a dead one must never take down this node.
        self.sinks = []
        comp = getattr(mavutil.mavlink, "MAV_COMP_ID_OBSTACLE_AVOIDANCE", 196)
        for url in (u.strip() for u in str(gp("sink_urls").value).split(",")):
            if not url:
                continue
            try:
                self.sinks.append(mavutil.mavlink_connection(url, source_component=comp))
                self.get_logger().info(f"OBSTACLE_DISTANCE -> {url}")
            except Exception as e:
                self.get_logger().warn(f"sink {url} not opened: {e}")

        self._pose = None       # (x, y, yaw) from the latest /odom
        self._last_odom_time = 0.0
        self._odom_stale_warned = False
        self._last_send = 0.0
        self._logged = False

        self.create_subscription(Odometry, gp("odom_topic").value, self._on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, gp("map_topic").value, self._on_map, 10)
        self.get_logger().info(
            f"/map -> OBSTACLE_DISTANCE ({N_SECTORS} x {self.inc_deg:.1f} deg, "
            f"range {self.min_m:.1f}-{self.max_m:.1f} m)")

    def _on_odom(self, msg: Odometry):
        self._last_odom_time = self.get_clock().now().nanoseconds / 1e9
        self._odom_stale_warned = False
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                          1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        self._pose = (p.x, p.y, yaw)

    def _on_map(self, msg: OccupancyGrid):
        now = self.get_clock().now().nanoseconds / 1e9
        if self._pose is None or not self.sinks:
            return
        # Item 11 Watchdog: halt raycasting if odometry is stale (> 0.5s)
        if (now - self._last_odom_time) > 0.5:
            if not self._odom_stale_warned:
                self.get_logger().warn(
                    f"Odometry stale ({now - self._last_odom_time:.2f}s > 0.5s); halting OBSTACLE_DISTANCE broadcast to prevent ghost collision fields."
                )
                self._odom_stale_warned = True
            return
        if now - self._last_send < self.min_dt:
            return
        self._last_send = now

        res = msg.info.resolution
        if res <= 0.0:
            return
        w, h = msg.info.width, msg.info.height
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        data = np.asarray(msg.data, dtype=np.int8).reshape(h, w)

        x_r, y_r, yaw = self._pose
        steps = np.arange(self.min_m, self.max_m, self.ray_step)
        ring = np.full(N_SECTORS, self.clear, dtype=np.uint16)

        for i in range(N_SECTORS):
            ang = yaw - math.radians(i * self.inc_deg)
            xs = x_r + steps * math.cos(ang)
            ys = y_r + steps * math.sin(ang)
            cols = np.floor((xs - ox) / res).astype(np.int64)
            rows = np.floor((ys - oy) / res).astype(np.int64)
            inb = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
            if not np.any(inb):
                continue
            vals = np.full(steps.shape, -1, dtype=np.int8)
            vals[inb] = data[rows[inb], cols[inb]]
            hit = np.nonzero(vals >= self.occ_thr)[0]
            if hit.size:
                r = steps[hit[0]]
                ring[i] = np.uint16(np.clip(r * 100.0, self.min_cm, self.max_cm))

        if not self._logged:
            self.get_logger().info(
                f"first grid: {int(np.count_nonzero(ring < self.clear))}/{N_SECTORS} "
                f"sectors occupied")
            self._logged = True

        t_us = int(self.get_clock().now().nanoseconds / 1000)
        payload = ring.tolist()
        for conn in self.sinks:
            try:
                conn.mav.obstacle_distance_send(
                    t_us,
                    mavutil.mavlink.MAV_DISTANCE_SENSOR_UNKNOWN,
                    payload,
                    int(round(self.inc_deg)),          # increment (uint8, deg)
                    self.min_cm, self.max_cm,
                    increment_f=float(self.inc_deg),
                    angle_offset=0.0,                  # index 0 = forward
                    frame=mavutil.mavlink.MAV_FRAME_BODY_FRD)
            except Exception:
                pass


def main():
    rclpy.init()
    node = ObstacleDistanceBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
