#!/usr/bin/env python3
import os
# Force MAVLink 2 + the "common" dialect before pymavlink picks a default. Without this,
# mavutil can default to the ardupilotmega dialect, whose VISION_POSITION_ESTIMATE lacks
# the covariance/reset_counter fields entirely - harmless today (we don't send them yet),
# but a latent trap if that ever changes.
os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "common")

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from pymavlink import mavutil
import math
import socket
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
        
        # Default routes through mavlink-router (holds /dev/pixhawk exclusively so
        # QGroundControl can share the link over WiFi) rather than opening the raw
        # serial device directly, which would now contend with the router for it.
        self.declare_parameter('device', 'tcp:127.0.0.1:5760')
        self.declare_parameter('baud', 921600)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('use_ned_conversion', True)
        self.declare_parameter('stale_warn_s', 0.5)
        # RTAB-Map's own "lost frame" convention: a fully-lost registration reports
        # pose covariance[0]=9999 (confirmed live via slam_health_monitor.py on
        # 2026-09-05 - LOST rows read cov0=9999.0 exactly). Reject anything at or
        # above a much lower bar than that so a degraded-but-not-yet-9999 frame is
        # also caught, not just the extreme case.
        self.declare_parameter('lost_cov_threshold', 100.0)
        self.declare_parameter('quat_norm_min', 0.5)
        # Fan the same VISION_POSITION_ESTIMATE out to px4_control.py's obstacle/EV
        # safety layer (its --obstacle-port, default udpin:0.0.0.0:14541) so its
        # localization-staleness failsafe has a real EV-alive heartbeat to watch.
        # Comma-separated; empty entries ignored. Matches odom_to_px4_vision.py's
        # extra_urls convention in the same downloaded toolset.
        self.declare_parameter('extra_urls', 'udpout:127.0.0.1:14541')

        self.device = self.get_parameter('device').value
        self.baud = int(self.get_parameter('baud').value)
        self.odom_topic = self.get_parameter('odom_topic').value
        self.use_ned_conversion = self.get_parameter('use_ned_conversion').value
        self.stale_warn_s = float(self.get_parameter('stale_warn_s').value)
        self.lost_cov_threshold = float(self.get_parameter('lost_cov_threshold').value)
        self.quat_norm_min = float(self.get_parameter('quat_norm_min').value)
        self.dropped = 0

        self.mav = None
        self.extra = []
        self.connect_mavlink()

        self.sub_odom = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        self.get_logger().info(f'Subscribed to {self.odom_topic}. Vision bridge active.')

        # Stale-odom watchdog: catches a dead SLAM node (no /odom at all), independent of
        # odom_callback's own degenerate-frame checks (which only fire when odom IS arriving).
        self.last_odom_wall = time.monotonic()
        self.create_timer(0.25, self._watchdog)

        # Drain incoming MAVLink traffic on a timer. This node only ever WRITES
        # (vision_position_estimate_send) - it never calls recv_match. But mavlink-router
        # mirrors the ENTIRE bus (every heartbeat, GPS, attitude, everything) onto every
        # connected endpoint, including this one's TCP connection. With nothing ever
        # reading it, that traffic just piles up in the OS socket receive buffer forever.
        # Confirmed for real on 2026-09-05: that buffer reached 15KB+ unread, which
        # stalled mavlink-router's own event loop and broke QGroundControl's connection
        # too - not just this node. Discarding what we don't need is cheap; not doing it
        # took down the whole link.
        self.create_timer(0.1, self._drain_incoming)

    def _drain_incoming(self):
        if self.mav is None:
            return
        # NOT recv_match(): confirmed by reading pymavlink's mavtcp.recv() source that
        # each call only reads self.mav.bytes_needed() bytes - whatever the PARSER
        # currently wants for the message it's mid-parsing, typically a few dozen
        # bytes - not however much is actually queued in the OS kernel buffer. Looping
        # that 200x/tick still couldn't keep up with the full-bus mirror rate (~18KB/s
        # measured earlier) - Recv-Q kept climbing (197 -> 1268 over 60s) even with the
        # bounded loop in place. This node never needs to interpret incoming traffic -
        # it only writes - so drain the raw socket directly instead, which reads
        # whatever's actually available per call (up to 64KB), not a parser-sized bite.
        # Do not also read via self.mav.recv_match()/recv_msg() on this connection -
        # interleaving raw discarded reads with pymavlink's own parse buffer would
        # desync it permanently.
        try:
            sock = self.mav.port
            while True:
                data = sock.recv(65536)
                if not data:
                    # Empty recv means the peer closed the connection (e.g.
                    # mavlink-router restarted) - mark it dead now rather than waiting
                    # for the next send attempt to fail, so odom_callback's
                    # reconnect-on-None path kicks in immediately.
                    self.get_logger().warn(
                        'MAVLink connection closed by peer - will reconnect on next odom message.')
                    self.mav = None
                    break
        except BlockingIOError:
            pass  # nothing to read right now - the normal, expected case every tick
        except (ConnectionResetError, BrokenPipeError, OSError, socket.error):
            # A real connection failure, not "no data yet" - BlockingIOError is also
            # technically an OSError subclass, which is why it's caught separately
            # above and must come first, or it would never reach this branch.
            self.get_logger().warn(
                'MAVLink connection error - will reconnect on next odom message.')
            self.mav = None
        except Exception:
            self.mav = None

    def _watchdog(self):
        gap = time.monotonic() - self.last_odom_wall
        if gap > self.stale_warn_s:
            self.get_logger().warn(
                f'No {self.odom_topic} message for {gap:.2f}s (warn threshold '
                f'{self.stale_warn_s:.2f}s) - SLAM may have stopped publishing.',
                throttle_duration_sec=2.0
            )

    def connect_mavlink(self):
        try:
            # baud is meaningless (and rejected by some pymavlink versions) for a
            # tcp:/udp:/udpin:/udpout: connection string - only pass it for an actual
            # serial device, same rule c2_validate.py already follows.
            is_serial = not self.device.startswith(('tcp:', 'udp:', 'udpin:', 'udpout:'))
            if is_serial:
                self.get_logger().info(f'Connecting MAVLink to {self.device} @ {self.baud}...')
            else:
                self.get_logger().info(f'Connecting MAVLink to {self.device} (via mavlink-router)...')
            comp_id = getattr(mavutil.mavlink, 'MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY', 197)
            kwargs = {'source_component': comp_id}
            if is_serial:
                kwargs['baud'] = self.baud
            self.mav = mavutil.mavlink_connection(self.device, **kwargs)
            # Wait for heartbeat with a short timeout to prevent blocking startup forever
            heartbeat = self.mav.wait_heartbeat(timeout=3.0)
            if heartbeat:
                self.get_logger().info('MAVLink connected to PX4 system!')
            else:
                self.get_logger().warn('MAVLink heartbeat timeout! Will attempt sending packets when odometry arrives.')
        except Exception as e:
            self.get_logger().error(f'Failed to open MAVLink device {self.device}: {e}')
            self.mav = None
            return

        # Extra sinks are best-effort: a dead one must never take down the flight
        # link to PX4, so each is opened independently and failures only warn. Only
        # opened once ever - connect_mavlink() can retry the main link many times
        # over the node's life (odom_callback reconnects on send failure), and these
        # UDP sinks don't need to reconnect when that happens.
        if not self.extra:
            for url in (u.strip() for u in str(self.get_parameter('extra_urls').value).split(',')):
                if not url:
                    continue
                try:
                    self.extra.append(mavutil.mavlink_connection(
                        url, source_component=getattr(
                            mavutil.mavlink, 'MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY', 197)))
                    self.get_logger().info(f'VISION_POSITION_ESTIMATE also -> {url}')
                except Exception as e:
                    self.get_logger().warn(f'extra EV sink {url} not opened: {e}')

    def odom_callback(self, msg: Odometry):
        self.last_odom_wall = time.monotonic()

        # Retry connection if not currently established
        if self.mav is None:
            self.connect_mavlink()
            if self.mav is None:
                return

        # Extract timestamp in microseconds
        usec = int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1e3)
        
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        # Reject lost/degenerate odometry before it ever reaches EKF2. RTAB-Map flags
        # an un-registerable frame with a huge pose covariance (cov0=9999, confirmed
        # live via slam_health_monitor.py) and/or a null-ish quaternion; forwarding
        # one injects a garbage pose EKF2 has no way to distinguish from a good one.
        # Skip it - EKF2 coasts on IMU/whatever else it's fusing until a good frame
        # returns.
        cov0 = msg.pose.covariance[0]
        qn = ori.x * ori.x + ori.y * ori.y + ori.z * ori.z + ori.w * ori.w
        finite = all(math.isfinite(v) for v in
                     (pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w, cov0))
        if (not finite) or qn < self.quat_norm_min or cov0 > self.lost_cov_threshold:
            self.dropped += 1
            if self.dropped % 30 == 1:
                self.get_logger().warn(
                    f'SLAM /odom LOST (cov0={cov0:.0f} |q|^2={qn:.2f}) - skipping '
                    f'this frame rather than forwarding a bad pose (dropped {self.dropped})'
                )
            return

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
            return

        # Fan the same pose out to the safety layer(s). Sent AFTER the lost-frame
        # guard above and the primary send succeeding, so a lost frame produces no
        # heartbeat here either and px4_control.py's EV-staleness failsafe sees it.
        for link in self.extra:
            try:
                link.mav.vision_position_estimate_send(
                    usec, x_mav, y_mav, z_mav, roll_mav, pitch_mav, yaw_mav
                )
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = PX4VisionBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.mav is not None:
            node.mav.close()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
