#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2

class MapThinningNode(Node):
    def __init__(self):
        super().__init__('map_thinning_node')
        
        self.declare_parameter('lock_threshold', 80)
        self.declare_parameter('occupancy_threshold', 65)
        self.declare_parameter('min_wall_area_pixels', 20)
        self.declare_parameter('unlock_miss_count', 15)
        
        self.sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.pub = self.create_publisher(
            OccupancyGrid,
            '/map_thin',
            10
        )
        
        # Stateful Hysteresis Wall Lock Buffer
        self.locked_walls_mask = None
        self.miss_counters = None
        self.map_shape = None

        self.get_logger().info('Hysteresis Wall Lock & Phantom Noise Purger Node initialized.')

    def map_callback(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        if width == 0 or height == 0:
            return

        lock_thresh = self.get_parameter('lock_threshold').value
        occ_thresh = self.get_parameter('occupancy_threshold').value
        min_area = self.get_parameter('min_wall_area_pixels').value
        unlock_limit = self.get_parameter('unlock_miss_count').value

        # 1. Convert 1D msg data to 2D numpy array
        raw_data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Re-initialize state buffers if map shape changes
        if self.map_shape != (height, width):
            self.map_shape = (height, width)
            self.locked_walls_mask = np.zeros((height, width), dtype=np.uint8)
            self.miss_counters = np.zeros((height, width), dtype=np.int32)

        # 2. Stateful Hysteresis Wall Locking (P >= 80 -> Lock)
        high_confidence_hits = (raw_data >= lock_thresh)
        self.locked_walls_mask[high_confidence_hits] = 255
        self.miss_counters[high_confidence_hits] = 0

        # Increment miss counter for locked cells currently seeing clear space (P < 40)
        clear_misses = (self.locked_walls_mask == 255) & (raw_data < 40) & (raw_data >= 0)
        self.miss_counters[clear_misses] += 1

        # Unlock cells that receive >= 15 consecutive clear miss observations
        unlocked = (self.miss_counters >= unlock_limit)
        self.locked_walls_mask[unlocked] = 0
        self.miss_counters[unlocked] = 0

        # 3. Combine active hits (P >= 65) with locked wall state
        active_hits = (raw_data >= occ_thresh).astype(np.uint8) * 255
        combined_binary = cv2.bitwise_or(active_hits, self.locked_walls_mask)

        if np.count_nonzero(combined_binary) == 0:
            self.pub.publish(msg)
            return

        # 4. Phantom Noise Purging: Connected Component Area Filter (< min_area deleted)
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(combined_binary, 8)
        purged_binary = np.zeros_like(combined_binary)
        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] >= min_area:
                purged_binary[labels == i] = 255

        # 5. Morphological Closing: Bridge 1-3 cell gaps along continuous walls
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        closed_binary = cv2.morphologyEx(purged_binary, cv2.MORPH_CLOSE, kernel)

        # 6. Apply Thinning / Skeletonization to get 1-pixel LiDAR-quality line
        thin_map = self.apply_thinning(closed_binary)

        # 7. Reconstruct Clean Output OccupancyGrid
        out_data = raw_data.copy()

        # Erase thick/noisy obstacle cells to free space (0)
        occupied_mask = (raw_data >= occ_thresh)
        out_data[occupied_mask] = 0

        # Set 1-pixel thin skeleton cells to occupied (100)
        out_data[thin_map > 0] = 100

        # 8. Publish /map_thin
        out_msg = OccupancyGrid()
        out_msg.header = msg.header
        out_msg.info = msg.info
        out_msg.data = out_data.flatten().tolist()
        self.pub.publish(out_msg)

    def apply_thinning(self, binary_img):
        # Try OpenCV ximgproc thinning if available
        if hasattr(cv2, 'ximgproc') and hasattr(cv2.ximgproc, 'thinning'):
            try:
                return cv2.ximgproc.thinning(binary_img)
            except Exception:
                pass
        
        # Robust Morphological Skeletonization Fallback
        skel = np.zeros(binary_img.shape, np.uint8)
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        img = binary_img.copy()
        
        while True:
            eroded = cv2.erode(img, element)
            temp = cv2.dilate(eroded, element)
            temp = cv2.subtract(img, temp)
            skel = cv2.bitwise_or(skel, temp)
            img = eroded.copy()
            if cv2.countNonZero(img) == 0:
                break
        return skel

def main(args=None):
    rclpy.init(args=args)
    node = MapThinningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
