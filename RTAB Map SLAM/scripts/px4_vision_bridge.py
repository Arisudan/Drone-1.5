#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from pymavlink import mavutil
import math
import time

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class PX4VisionBridge(Node):
    def __init__(self):
        super().__init__('px4_vision_bridge')
        
        self.declare_parameter('device', '/dev/pixhawk')
        self.declare_parameter('baud', 921600)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('use_ned_conversion', True)
        
        self.device = self.get_parameter('device').value
        self.baud = int(self.get_parameter('baud').value)
        self.odom_topic = self.get_parameter('odom_topic').value
        self.use_ned_conversion = self.get_parameter('use_ned_conversion').value
        
        self.mav = None
        self.connect_mavlink()

        self.sub_odom = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        self.get_logger().info(f'Subscribed to {self.odom_topic}. Vision bridge active.')

    def connect_mavlink(self):
        try:
            self.get_logger().info(f'Connecting MAVLink to {self.device} @ {self.baud}...')
            self.mav = mavutil.mavlink_connection(self.device, baud=self.baud)
            # Wait for heartbeat with a short timeout to prevent blocking startup forever
            heartbeat = self.mav.wait_heartbeat(timeout=3.0)
            if heartbeat:
                self.get_logger().info('MAVLink connected to PX4 system!')
            else:
                self.get_logger().warn('MAVLink heartbeat timeout! Will attempt sending packets when odometry arrives.')
        except Exception as e:
            self.get_logger().error(f'Failed to open MAVLink device {self.device}: {e}')
            self.mav = None

    def odom_callback(self, msg: Odometry):
        # Retry connection if not currently established
        if self.mav is None:
            self.connect_mavlink()
            if self.mav is None:
                return

        # Extract timestamp in microseconds
        usec = int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1e3)
        
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        # ROS ENU coordinates
        x_ros = float(pos.x)
        y_ros = float(pos.y)
        z_ros = float(pos.z)
        
        # Convert ROS quaternion to Euler angles (roll, pitch, yaw) in ROS ENU frame
        q0, q1, q2, q3 = ori.w, ori.x, ori.y, ori.z
        roll_ros = math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2))
        pitch_ros = math.asin(max(-1.0, min(1.0, 2.0 * (q0 * q2 - q3 * q1))))
        yaw_ros = math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))

        if self.use_ned_conversion:
            # Convert ROS ENU (East-North-Up) to PX4 NED (North-East-Down)
            x_mav = y_ros       # North
            y_mav = x_ros       # East
            z_mav = -z_ros      # Down
            
            roll_mav = roll_ros
            pitch_mav = -pitch_ros
            yaw_mav = normalize_angle(-yaw_ros + (math.pi / 2.0))
        else:
            x_mav = x_ros
            y_mav = y_ros
            z_mav = z_ros
            roll_mav = roll_ros
            pitch_mav = pitch_ros
            yaw_mav = yaw_ros

        try:
            # Send VISION_POSITION_ESTIMATE (#102) over MAVLink to PX4 EKF2
            self.mav.mav.vision_position_estimate_send(
                usec,
                x_mav,
                y_mav,
                z_mav,
                roll_mav,
                pitch_mav,
                yaw_mav
            )
        except Exception as e:
            self.get_logger().error(f'Error sending MAVLink vision packet: {e}')
            self.mav = None

def main(args=None):
    rclpy.init(args=args)
    node = PX4VisionBridge()
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
