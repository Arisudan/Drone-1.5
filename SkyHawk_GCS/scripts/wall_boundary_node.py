#!/usr/bin/env python3
"""
================================================================================
MODULE: wall_boundary_node.py
PURPOSE: Extracts Geometric Wall Contours & Inflated Flight Boundaries for RViz
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Radxa Q6A Companion Computer (Onboard System)
  * Upstream:      RTAB-Map SLAM Node (/map)
  * Downstream:    RViz2, MRS Lite Trajectory Planner (/wall_boundaries)

DATA FLOW & INTERFACES:
  * Subscribes To: /map [nav_msgs/OccupancyGrid] (Raw 2.5cm/5cm occupancy grid)
  * Publishes To:  /wall_boundaries [visualization_msgs/MarkerArray]
  * QoS Profile:   Transient Local, Reliable

KEY ALGORITHMIC PIPELINE:
  1. Thresholding: Extracts occupied obstacle cells (prob >= 65).
  2. Obstacle Inflation: Dilates obstacles by inflation_radius_m (default 0.6m)
     to establish a safe drone clearance buffer.
  3. Contour Extraction: Runs OpenCV cv2.findContours on the dilated boundary.
  4. Douglas-Peucker Polygon Approximation: Simplifies raw contours into vector
     line strips using cv2.approxPolyDP with epsilon = 2.0 pixels.
  5. 3D Line Strip Markers: Emits LINE_STRIP RViz markers showing physical walls
     (Crimson Red) and clearance corridors (Cyan).

RUN:
  source /opt/ros/jazzy/setup.bash
  python3 wall_boundary_node.py --ros-args -p inflation_radius_m:=0.6
================================================================================
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
import cv2

class WallBoundaryNode(Node):
    def __init__(self):
        super().__init__('wall_boundary_node')
        
        # Declare parameters
        self.declare_parameter('inflation_radius_m', 0.6)
        self.declare_parameter('min_contour_area', 10.0)
        
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        # Subscriber & Publisher
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.pub_boundaries = self.create_publisher(MarkerArray, '/wall_boundaries', map_qos)
        
        self.get_logger().info("Wall Boundary Node initialized (extracting safe drone flight boundary).")

    def map_callback(self, msg: OccupancyGrid):
        inflation_radius_m = self.get_parameter('inflation_radius_m').get_parameter_value().double_value
        min_contour_area = self.get_parameter('min_contour_area').get_parameter_value().double_value
        
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        
        # Extract 2D yaw rotation from origin quaternion
        o = msg.info.origin.orientation
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        if width == 0 or height == 0 or resolution <= 0.0:
            return

        # 1. Convert grid data array to 2D numpy array (shape: height x width)
        grid_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # 2. Extract Free Space mask (cells with value between 0 and 49)
        free_mask = np.zeros((height, width), dtype=np.uint8)
        free_mask[(grid_data >= 0) & (grid_data < 50)] = 255

        # 3. Morphological closing to fill small unmapped gaps within free space
        kernel_3x3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        cleaned_free = cv2.morphologyEx(free_mask, cv2.MORPH_CLOSE, kernel_3x3)
        cleaned_free = cv2.morphologyEx(cleaned_free, cv2.MORPH_OPEN, kernel_3x3)

        # 4. Erode the free space by safety inflation radius (keeps boundary 0.6m inside walls)
        k_radius = int(round(inflation_radius_m / resolution))
        if k_radius > 0:
            k_size = 2 * k_radius + 1
            erosion_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_size, k_size))
            safe_flight_mask = cv2.erode(cleaned_free, erosion_kernel)
        else:
            safe_flight_mask = cleaned_free

        # 5. Find external contours outlining the safe flight boundaries
        contours, _ = cv2.findContours(safe_flight_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 6. Build MarkerArray output
        marker_array = MarkerArray()
        
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        stamp = self.get_clock().now().to_msg()
        marker_id = 0

        # Sort contours by area descending
        valid_contours = [c for c in contours if cv2.contourArea(c) >= min_contour_area]
        valid_contours.sort(key=cv2.contourArea, reverse=True)

        for cnt in valid_contours:
            # Smooth contour using polygon approximation for a clean line
            epsilon = max(1.0, 0.005 * cv2.arcLength(cnt, True))
            smoothed_cnt = cv2.approxPolyDP(cnt, epsilon, True)

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = stamp
            marker.ns = 'wall_boundaries'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.04  # Red boundary line thickness (0.04m)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Solid red

            for pt in smoothed_cnt:
                col = pt[0][0]
                row = pt[0][1]
                dx = col * resolution
                dy = row * resolution
                map_x = origin_x + (dx * cos_yaw - dy * sin_yaw)
                map_y = origin_y + (dx * sin_yaw + dy * cos_yaw)

                p = Point()
                p.x = float(map_x)
                p.y = float(map_y)
                p.z = 0.02  # Slightly above grid plane for crisp RViz rendering
                marker.points.append(p)

            # Close boundary loop
            if len(smoothed_cnt) > 0:
                first_col = smoothed_cnt[0][0][0]
                first_row = smoothed_cnt[0][0][1]
                dx0 = first_col * resolution
                dy0 = first_row * resolution
                p_first = Point()
                p_first.x = float(origin_x + (dx0 * cos_yaw - dy0 * sin_yaw))
                p_first.y = float(origin_y + (dx0 * sin_yaw + dy0 * cos_yaw))
                p_first.z = 0.02
                marker.points.append(p_first)

            marker_array.markers.append(marker)

        self.pub_boundaries.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = WallBoundaryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
