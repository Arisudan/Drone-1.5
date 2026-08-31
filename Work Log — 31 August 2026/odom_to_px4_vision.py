#!/usr/bin/env python3
"""SLAM odometry -> PX4 EKF2 external vision (via MAVROS vision_pose plugin).

PHASE 1 of autonomy: replace the SITL "cheat" (SIM_GZ_EN_ODOM=1, which feeds
ground-truth pose from the physics engine straight into EKF2) with the drone's OWN
SLAM estimate. This node subscribes to the SLAM /odom (from lidar_slam.launch.py or
rgbd_imu_slam.launch.py), converts ROS ENU -> PX4 NED, and republishes it as a
geometry_msgs/PoseStamped on /mavros/vision_pose/pose.

WHY THIS VERSION DIFFERS FROM THE ORIGINAL:
The original script opened its own pymavlink connection (udpout:127.0.0.1:14280)
and sent VISION_POSITION_ESTIMATE directly over MAVLink. That works fine over UDP,
where multiple independent links can share the FCU. Over a serial link (USB/TELEM),
only ONE process can own the port at a time -- and MAVROS is already holding it for
telemetry. Rather than fight over the serial device, this version hands the pose to
MAVROS itself: MAVROS's vision_pose plugin subscribes to /mavros/vision_pose/pose
and forwards it to PX4 as VISION_POSITION_ESTIMATE over the SAME link MAVROS already
owns. No second connection, no port contention, works the same whether MAVROS's
fcu_url is serial or UDP underneath.

FRAME CONVERSION (ROS "odom" frame is ENU: x=East, y=North, z=Up; PX4 local = NED):
  x_ned =  y_enu       (North)
  y_ned =  x_enu       (East)
  z_ned = -z_enu       (Down)
  yaw_ned  = pi/2 - yaw_enu   (ENU yaw is CCW-from-East; NED yaw is CW-from-North)
  roll_ned =  roll_enu ;  pitch_ned = -pitch_enu

NOTE ON FRAME_ID: MAVROS's vision_pose plugin expects the pose already in the
frame PX4/EKF2 wants (its internal convention, effectively NED-based local frame).
We do the ENU->NED math ourselves before publishing, same as the original script
did before handing off to MAVLink, so EKF2 receives the same numbers either way.

REQUIRED PX4 PARAMS (set by the 4032 airframe, or at runtime in pxh):
  SIM_GZ_EN_ODOM 0     # kill the cheat  (set 1 to compare against ground truth)
  EKF2_EV_CTRL   9     # fuse EV horizontal position + yaw
  EKF2_GPS_CTRL  0     # no GPS (indoor)
  EKF2_HGT_REF   0     # height from baro (2D-lidar SLAM has no z)
  EKF2_MAG_TYPE  5     # no mag; heading comes from EV yaw

REQUIRED MAVROS CONFIG:
MAVROS must have the vision_pose plugin enabled (it is by default) and must
already be connected to the FCU (serial or UDP -- doesn't matter here).

RUN (after a SLAM pipeline is up and publishing /odom, and MAVROS is running):
  python3 odom_to_px4_vision.py --ros-args -p use_sim_time:=true
  # options: -p odom_topic:=/odom -p vision_pose_topic:=/mavros/vision_pose/pose
Verify EKF is actually using it: check PX4 with `listener vehicle_visual_odometry`,
and confirm MAVROS is forwarding by watching `ros2 topic hz /mavros/vision_pose/pose`
and comparing it stays in sync with the injector's send rate.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


def quat_to_euler(x, y, z, w):
    """ROS quaternion -> (roll, pitch, yaw) in the ENU frame."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def euler_to_quat(roll, pitch, yaw):
    """(roll, pitch, yaw) -> quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class VisionInjector(Node):
    def __init__(self):
        super().__init__("odom_to_px4_vision")
        gp = lambda n: self.get_parameter(n)  # noqa: E731
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("vision_pose_topic", "/mavros/vision_pose/pose")
        self.declare_parameter("rate_hz", 30.0)      # cap the publish rate
        self.declare_parameter("stale_warn_s", 0.5)  # warn if /odom stops

        self.min_dt = 1.0 / max(1e-3, gp("rate_hz").value)
        self.stale_warn_s = gp("stale_warn_s").value
        self.last_send = 0.0
        self.last_odom_wall = self.now_s()
        self.n = 0
        self.dropped = 0

        # MAVROS's vision_pose plugin subscribes with RELIABLE QoS. Publishing
        # BEST_EFFORT here causes a QoS mismatch -- the subscription is created
        # but MAVROS silently receives nothing. Must match RELIABLE.
        pose_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST)
        self.pose_pub = self.create_publisher(
            PoseStamped, gp("vision_pose_topic").value, pose_qos)

        odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, gp("odom_topic").value, self.cb, odom_qos)

        self.create_timer(0.25, self._watchdog)
        self.get_logger().info(
            f"EV injector: '{gp('odom_topic').value}' -> "
            f"'{gp('vision_pose_topic').value}' (ENU->NED, via MAVROS vision_pose "
            f"plugin). Set SIM_GZ_EN_ODOM 0 to use it.")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cb(self, msg: Odometry):
        now = self.now_s()
        self.last_odom_wall = self.now_s()  # note: uses sim clock if use_sim_time
        if now - self.last_send < self.min_dt:
            return
        self.last_send = now

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # --- reject LOST / degenerate odometry so we never teleport EKF2 ------
        # RTAB-Map flags an un-registerable ("lost") scan with a huge pose
        # covariance (9999) and/or a null quaternion. With Odom/ResetCountdown=0
        # it no longer re-origins, but it still emits these lost frames - and
        # forwarding one injects a garbage EV pose. Skip it; EKF2 coasts on IMU
        # until a good frame returns.
        cov0 = msg.pose.covariance[0]
        qn = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        finite = all(math.isfinite(v) for v in
                     (p.x, p.y, p.z, q.x, q.y, q.z, q.w, cov0))
        if (not finite) or qn < 0.5 or cov0 > 100.0:
            self.dropped += 1
            if self.dropped % 30 == 1:
                self.get_logger().warn(
                    "SLAM /odom LOST (cov=%.0f |q|^2=%.2f) - skipping EV to avoid "
                    "a teleport (dropped %d)" % (cov0, qn, self.dropped))
            return

        roll, pitch, yaw = quat_to_euler(q.x, q.y, q.z, q.w)

        # ENU -> NED
        x_ned = p.y
        y_ned = p.x
        z_ned = -p.z
        roll_ned = roll
        pitch_ned = -pitch
        yaw_ned = wrap_pi(math.pi / 2.0 - yaw)
        qx, qy, qz, qw = euler_to_quat(roll_ned, pitch_ned, yaw_ned)

        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "odom_ned"
        out.pose.position.x = x_ned
        out.pose.position.y = y_ned
        out.pose.position.z = z_ned
        out.pose.orientation.x = qx
        out.pose.orientation.y = qy
        out.pose.orientation.z = qz
        out.pose.orientation.w = qw
        self.pose_pub.publish(out)

        self.n += 1
        if self.n % 60 == 0:
            self.get_logger().info(
                "EV #%d  NED x=%.2f y=%.2f z=%.2f yaw=%.1f deg"
                % (self.n, x_ned, y_ned, z_ned, math.degrees(yaw_ned)))

    def _watchdog(self):
        gap = self.now_s() - self.last_odom_wall
        if self.n > 0 and gap > self.stale_warn_s:
            self.get_logger().warn(
                "SLAM /odom STALE %.1fs - EKF2 has no external vision! "
                "(localization lost: hold position / do not navigate)" % gap)


def main():
    rclpy.init(args=sys.argv)
    node = VisionInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
