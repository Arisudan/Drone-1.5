#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
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
        
        # Subscriber & Publisher
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.pub_boundaries = self.create_publisher(MarkerArray, '/wall_boundaries', 10)
        
        self.get_logger().info("Wall Boundary Node initialized (subscribing to /map, publishing to /wall_boundaries).")

    def map_callback(self, msg: OccupancyGrid):
        inflation_radius_m = self.get_parameter('inflation_radius_m').get_parameter_value().double_value
        min_contour_area = self.get_parameter('min_contour_area').get_parameter_value().double_value
        
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        
        if width == 0 or height == 0 or resolution <= 0.0:
            return

        # 1. Convert grid data to 2D numpy array (shape: height x width)
        grid_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # 2. Threshold: cells >= 65 (occupied) become 255, others become 0
        binary_mask = np.zeros((height, width), dtype=np.uint8)
        binary_mask[grid_data >= 65] = 255

        # 3. Morphological cleanup: CLOSE then OPEN with 3x3 ellipse kernel
        kernel_3x3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        cleaned_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel_3x3)
        cleaned_mask = cv2.morphologyEx(cleaned_mask, cv2.MORPH_OPEN, kernel_3x3)

        # 4. Inflate obstacles using cv2.dilate based on inflation_radius_m
        k_radius = int(round(inflation_radius_m / resolution))
        if k_radius > 0:
            k_size = 2 * k_radius + 1
            dilation_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_size, k_size))
            inflated_mask = cv2.dilate(cleaned_mask, dilation_kernel)
        else:
            inflated_mask = cleaned_mask

        # 5. Find external contours
        contours, _ = cv2.findContours(inflated_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 6. Build MarkerArray output
        marker_array = MarkerArray()
        
        # Add DELETEALL action first to clear old boundaries
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        stamp = self.get_clock().now().to_msg()
        marker_id = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_contour_area:
                continue

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = stamp
            marker.ns = 'wall_boundaries'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.03  # Line width
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Solid red

            for pt in cnt:
                col = pt[0][0]
                row = pt[0][1]
                map_x = origin_x + col * resolution
                map_y = origin_y + row * resolution

                p = Point()
                p.x = float(map_x)
                p.y = float(map_y)
                p.z = 0.0
                marker.points.append(p)

            # Close boundary loop
            if len(cnt) > 0:
                first_col = cnt[0][0][0]
                first_row = cnt[0][0][1]
                p_first = Point()
                p_first.x = float(origin_x + first_col * resolution)
                p_first.y = float(origin_y + first_row * resolution)
                p_first.z = 0.0
                marker.points.append(p_first)

            marker_array.markers.append(marker)

        self.pub_boundaries.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = WallBoundaryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
