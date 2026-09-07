#!/usr/bin/env python3
"""RTAB-Map 2D occupancy grid -> PX4 OBSTACLE_DISTANCE (full-ring obstacle avoidance).

Feeds px4_control.py's obstacle/EV safety layer (its --obstacle-port, default
udpin:0.0.0.0:14541) with the OBSTACLE_DISTANCE half of what _obst_loop() reads;
the VISION_POSITION_ESTIMATE half comes from px4_vision_bridge.py's own extra_urls
fan-out to the same port.

Unlike the reference toolset's laserscan_to_obstacle.py (a real 360-degree lidar),
this airframe has no lidar - it raycasts the same 72-sector body-FRD ring directly
against RTAB-Map's own /map (nav_msgs/OccupancyGrid) using the drone's current /odom
pose as the raycast origin and heading reference. A sector that runs off the mapped
area without hitting an occupied cell is left CLEAR, not "unknown" - px4_control.py's
own _cone_min() already treats unknown as passable, so there's no behavioral
difference, and it keeps this node's ring format identical to laserscan_to_obstacle.py's.

Known simplification: raycasts against /odom position directly, not a map->odom TF
lookup, so a loop-closure correction that hasn't yet reached /odom could offset the
ray origin from the map's own frame - the same approximation px4_vision_bridge.py
already makes forwarding /odom straight to PX4 as ground truth. Fine for bench/indoor
runs over short distances; a real TF lookup would be needed for large drift.

FRAME: OBSTACLE_DISTANCE is MAV_FRAME_BODY_FRD (index 0 = forward, +clockwise). ROS
yaw is CCW-positive from the map frame's +x axis (REP 103), so the body-FRD bearing
for sector i is achieved by casting at world angle (yaw - radians(i * inc_deg)).
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
        self._last_send = 0.0
        self._logged = False

        self.create_subscription(Odometry, gp("odom_topic").value, self._on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, gp("map_topic").value, self._on_map, 10)
        self.get_logger().info(
            f"/map -> OBSTACLE_DISTANCE ({N_SECTORS} x {self.inc_deg:.1f} deg, "
            f"range {self.min_m:.1f}-{self.max_m:.1f} m)")

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                          1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        self._pose = (p.x, p.y, yaw)

    def _on_map(self, msg: OccupancyGrid):
        if self._pose is None or not self.sinks:
            return
        now = self.get_clock().now().nanoseconds / 1e9
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
