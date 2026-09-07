#!/usr/bin/env python3
"""Live SLAM-health monitor for real hardware - tells you WHY /odom is bad, in real time.

Adapted from a reference implementation built for a lidar+Gazebo stack. That version
leaned on two things this rig doesn't have: a 360-degree lidar (/scan_360) and a
simulator ground-truth topic (/gz_ground_truth) to compute a DRIFT% column and an
automatic ICP-FAILURE/CONTENTION verdict. Neither exists here, so both are removed
rather than left in as dead weight:
  - No lidar -> no /scan_360 subscription. Leaving it in place would read 0 Hz forever,
    which the original code treats as "sensor jitter" (scan_bad=True) permanently -
    that's not a diagnosis, it's a false alarm baked into every single line.
  - No ground truth -> no DRIFT%/GT_ext, and no gt_moving gate on the verdict. On the
    reference stack, the verdict silently never fires without a live truth publisher
    (gt_moving is always False), even though the underlying ICP/IMU checks are computed
    correctly from real data. Here the verdict falls back to those checks directly.

What's left is exactly what matters for the "Not enough inliers 0/10" / quality=0
bench failure: RTAB-Map's own /odom_info and IMU timing jitter on the D435i's real
IMU stream.

CORRECTED 2026-09-05 after a real bench run: this originally read OdomInfo's
icp_inliers_ratio/icp_correspondences fields, copied from the reference's ICP
(lidar) pipeline. Checked `ros2 interface show rtabmap_msgs/msg/OdomInfo` directly:
those two fields are ICP-specific and are always 0 for our stereo_odometry, which
is VISUAL (feature-based), not ICP - they were never going to show anything.
The fields that actually matter here are `inliers` and `matches` (the exact
numbers from "Not enough inliers 0/10 (matches=0)"), which the OdomInfo message
already carries but this script was discarding. Also fixed: the verdict logic
let "IMU TIMING DEGRADED" permanently mask a real tracking failure whenever both
were true at once - on a real run, EVERY row was IMU-flagged (a 15ms jitter
threshold turned out to be too strict for normal USB scheduling bursts at a
rock-solid ~200Hz average), which hid the tracking column completely. Both
conditions are now reported together instead of one hiding the other.

RUN (after the SLAM pipeline is up):
  source /opt/ros/jazzy/setup.bash
  python3 slam_health_monitor.py
  # then reproduce the bad scenario. Watch `inliers` collapse and 'lost' flip.
Options:  -p period_s:=1.0   -p odom_topic:=/odom   -p imu_topic:=/camera/imu
"""
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

try:
    from rtabmap_msgs.msg import OdomInfo
    HAVE_INFO = True
except Exception:
    HAVE_INFO = False


class Stamps:
    """Rolling window of message arrival times (wall clock) for rate/jitter stats."""
    def __init__(self, keep=400):
        self.t = deque(maxlen=keep)

    def add(self):
        self.t.append(time.monotonic())

    def stats(self, window_s):
        now = time.monotonic()
        ts = [t for t in self.t if now - t <= window_s]
        if len(ts) < 3:
            return 0.0, 0.0
        dts = [b - a for a, b in zip(ts, ts[1:])]
        rate = (len(ts) - 1) / (ts[-1] - ts[0]) if ts[-1] > ts[0] else 0.0
        return rate, max(dts) * 1000.0      # Hz, worst gap in ms


class Monitor(Node):
    def __init__(self):
        super().__init__("slam_health_monitor")
        gp = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        self.period = float(gp("period_s", 1.0))
        odom_topic = gp("odom_topic", "/odom")
        imu_topic = gp("imu_topic", "/camera/imu")

        self.imu = Stamps()
        self.odom_pts = []      # (x, y) history for extent (informational only, no GT to compare against)
        self.vis = None         # (matches, inliers, lost) - visual odometry, not ICP
        self.cov0 = float("nan")

        self.create_subscription(Imu, imu_topic,
                                 lambda m: self.imu.add(), qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self._odom, qos_profile_sensor_data)
        if HAVE_INFO:
            self.create_subscription(OdomInfo, "/odom_info", self._info,
                                     qos_profile_sensor_data)
        else:
            self.get_logger().warn("rtabmap_msgs not found - ICP columns will be blank")

        self.start = time.monotonic()
        self.create_timer(self.period, self._tick)
        self._header()

    def _odom(self, m):
        p = m.pose.pose.position
        self.odom_pts.append((p.x, p.y))
        if len(self.odom_pts) > 6000:
            self.odom_pts.pop(0)
        self.cov0 = m.pose.covariance[0]

    def _info(self, m):
        # matches/inliers are the visual-odometry fields (feature-based registration);
        # icp_inliers_ratio/icp_correspondences are ICP-only and always 0 here - see
        # module docstring for how that was confirmed against the message definition.
        self.vis = (m.matches, m.inliers, m.lost)

    @staticmethod
    def _extent(pts):
        """Max pairwise span (m) - cheap proxy: bbox diagonal of the path so far."""
        if len(pts) < 2:
            return 0.0, 0.0
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
        return max(xs) - min(xs), max(ys) - min(ys)

    def _header(self):
        print("\n  t     IMU(Hz/maxms)   matches inliers lost  cov0     EST_ext  VERDICT-HINT")
        print("  " + "-" * 88)

    def _tick(self):
        t = time.monotonic() - self.start
        ir, ij = self.imu.stats(3.0)
        dx, dy = self._extent(self.odom_pts)
        est = max(dx, dy)

        if self.vis:
            matches, inliers, lost = self.vis
            vis_s = f"{matches:7d} {inliers:7d}  {'LOST' if lost else ' ok '}"
        else:
            matches, inliers, lost = 999, 999, False
            vis_s = "      -       -    -  "

        # nominal for the D435i's IMU: unite_imu_method=2 @ ~200Hz per d435i_stereo_imu.launch.py.
        # Raised from an earlier 15ms: a real bench run showed 20-50ms worst-case gaps in every
        # single 3s window at a rock-solid ~200Hz average - normal USB scheduling jitter, not a
        # real problem. 60ms is roughly 12 missed intervals in a row before this flags anything.
        imu_bad = ir < 150.0 or ij > 60.0
        # Mirrors stereo_inertial_odom.launch.py's own Vis/MinInliers (10) - the actual threshold
        # RTAB-Map itself uses to accept/reject a frame's registration.
        vis_bad = inliers < 10 or lost

        if vis_bad and imu_bad:
            hint = "TRACKING FAILURE + IMU timing degraded (check both)"
        elif vis_bad:
            hint = "TRACKING FAILURE (timing clean - the scene/registration is the problem)"
        elif imu_bad:
            hint = "IMU TIMING DEGRADED (tracking OK - check USB bandwidth/CPU load)"
        else:
            hint = "healthy"

        flag = lambda bad: "!" if bad else " "
        print(f"  {t:5.0f}  {ir:5.0f}/{ij:5.1f}{flag(imu_bad)}    "
              f"{vis_s}{flag(vis_bad)}  {self.cov0:7.1f}  {est:6.2f}  {hint}")


def main():
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=sys.argv)
    node = Monitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
