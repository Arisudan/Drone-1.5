#!/usr/bin/env python3
"""Fake /odom publisher for testing odom_to_px4_vision.py without real SLAM.

Publishes a slowly-moving circle trajectory on /odom so you can verify the
full chain: this node -> odom_to_px4_vision.py -> /mavros/vision_pose/pose
-> MAVROS -> PX4 EKF2, all without a lidar/camera or SLAM stack attached.

NOT for real flight or real localization -- test/dev tool only.

RUN:
  python3 fake_odom_publisher.py
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__("fake_odom_publisher")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Odometry, "/odom", qos)
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.create_timer(0.1, self.tick)  # 10 Hz
        self.get_logger().info("Publishing fake /odom (circle trajectory) at 10 Hz")

    def tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        radius = 1.0
        omega = 0.2  # rad/s

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = radius * math.cos(omega * t)
        msg.pose.pose.position.y = radius * math.sin(omega * t)
        msg.pose.pose.position.z = 0.5

        yaw = omega * t + math.pi / 2.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # low, finite covariance so the injector's LOST-frame guard passes it
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[14] = 0.01

        self.pub.publish(msg)


def main():
    rclpy.init(args=sys.argv)
    node = FakeOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
