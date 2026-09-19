#!/usr/bin/env python3
"""
================================================================================
NODE: map_recorder.py
PURPOSE: Capture One SLAM Run as Evaluable Artifacts (/map, /map_thin, /odom)
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Radxa SBC (alongside the live pipeline) or any machine that
                   can see the ROS 2 graph. Read-only - subscribes, never
                   publishes, never commands the vehicle.
  * Upstream:      /map, /map_thin [nav_msgs/OccupancyGrid], /odom [Odometry]
  * Downstream:    map_eval.py, which turns these artifacts into pass/fail
                   numbers (see docs/slam_evaluation.md).

WHY A RECORDER AND NOT JUST A ROSBAG:
  A bag captures everything and needs ROS to replay. This writes four small,
  self-describing files that any numpy install can open years from now - which
  is what a frozen evaluation set needs to stay comparable across builds. It
  is also ~1000x smaller than bagging the camera topics.

OUTPUT (one directory per run):
    out_mapping/run_<YYYYmmdd_HHMMSS>/
      map.npy        int8 (H, W)  final raw occupancy grid, ROS row-major
      map_thin.npy   int8 (H, W)  final skeletonised grid (if /map_thin is up)
      map.png                     8-bit render for eyeballing / picking cells
      map_thin.png                same for the skeleton
      meta.json                   resolution, origin, dims, counts, timing
      pose.csv                    t_s,x,y,z,yaw_deg,cov_xx,lost

  Cell coordinates read off map.png are exactly the COL,ROW that map_eval.py's
  --landmark / --wall / --corner flags expect (row 0 = image top = grid row 0).

VIO LOSS:
  RTAB-Map's stereo_odometry publishes covariance 9999.0 when tracking is lost
  (gotalldone.md dev log #6). That sentinel is captured per sample as `lost`,
  so an evaluation can say what fraction of a run had no usable odometry
  instead of guessing from a map that merely looks smeared.

USAGE:
  source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
  ros2 run rtabmap_drone_pkg map_recorder.py                  # Ctrl-C to finish
  ros2 run rtabmap_drone_pkg map_recorder.py --duration 120   # timed run
  ros2 run rtabmap_drone_pkg map_recorder.py --notes "loop, lab, walk test"
================================================================================
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry

# Matches map_eval.VO_LOST_COV - RTAB-Map's "tracking lost" covariance.
VO_LOST_COV = 9999.0


def quat_to_yaw_deg(x: float, y: float, z: float, w: float) -> float:
    """Yaw about Z in degrees, from a ROS quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def render_grid(grid: np.ndarray) -> np.ndarray:
    """int8 occupancy -> 8-bit greyscale for human inspection.

    unknown (-1) mid grey, free dark, occupied white. Deliberately NOT the
    GCS colour scheme: this image exists to read cell coordinates off, so
    plain greyscale keeps the indices unambiguous.
    """
    img = np.full(grid.shape, 128, dtype=np.uint8)
    img[grid == 0] = 40
    known = grid >= 0
    img[known] = np.clip(40 + (grid[known].astype(np.int16) * 215) // 100,
                         40, 255).astype(np.uint8)
    img[grid < 0] = 128
    return img


class MapRecorder(Node):
    """Latches the most recent grids and streams every pose sample to disk."""

    def __init__(self, out_dir: str, notes: str = ""):
        super().__init__("map_recorder")
        self.out_dir = out_dir
        self.notes = notes
        self.t0 = time.time()

        self.grid_raw: Optional[np.ndarray] = None
        self.grid_thin: Optional[np.ndarray] = None
        self.meta_raw: dict = {}
        self.meta_thin: dict = {}
        self.map_count = 0
        self.thin_count = 0
        self.pose_count = 0
        self.lost_count = 0

        os.makedirs(self.out_dir, exist_ok=True)
        # Stream poses straight to disk rather than buffering: a run that ends
        # with a crashed pipeline or a yanked battery still leaves a usable
        # trajectory behind, which is exactly when you most want one.
        self._pose_fh = open(os.path.join(self.out_dir, "pose.csv"), "w",
                             newline="", encoding="utf-8")
        self._pose_csv = csv.writer(self._pose_fh)
        self._pose_csv.writerow(["t_s", "x", "y", "z", "yaw_deg", "cov_xx", "lost"])

        # TRANSIENT_LOCAL + RELIABLE matches RTAB-Map's latched grid and
        # map_thinning_node, so a map published before this node started is
        # still delivered on subscribe.
        map_qos = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(OccupancyGrid, "/map_thin", self._on_thin, map_qos)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        self.create_timer(5.0, self._log_progress)
        self.get_logger().info(f"Recording run to {self.out_dir}")

    # ── subscriptions ───────────────────────────────────────────────

    @staticmethod
    def _grid_from_msg(msg: OccupancyGrid) -> np.ndarray:
        """Flat int8 data -> (H, W) row-major, ROS-native orientation.

        Note this keeps ROS orientation. The GCS map listener transposes to
        (W, H) for rendering; the evaluation pipeline stays row-major so cell
        indices match the OccupancyGrid spec and the saved PNG.
        """
        return np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))

    @staticmethod
    def _meta_from_msg(msg: OccupancyGrid) -> dict:
        return {
            "resolution": float(msg.info.resolution),
            "origin_x": float(msg.info.origin.position.x),
            "origin_y": float(msg.info.origin.position.y),
            "width": int(msg.info.width),
            "height": int(msg.info.height),
        }

    def _on_map(self, msg: OccupancyGrid):
        if msg.info.width <= 0 or msg.info.height <= 0:
            return
        self.grid_raw = self._grid_from_msg(msg)
        self.meta_raw = self._meta_from_msg(msg)
        self.map_count += 1

    def _on_thin(self, msg: OccupancyGrid):
        if msg.info.width <= 0 or msg.info.height <= 0:
            return
        self.grid_thin = self._grid_from_msg(msg)
        self.meta_thin = self._meta_from_msg(msg)
        self.thin_count += 1

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        cov_xx = float(msg.pose.covariance[0])
        lost = 1 if cov_xx >= VO_LOST_COV else 0
        self.lost_count += lost
        self.pose_count += 1
        self._pose_csv.writerow([
            f"{time.time() - self.t0:.3f}",
            f"{p.x:.4f}", f"{p.y:.4f}", f"{p.z:.4f}",
            f"{quat_to_yaw_deg(q.x, q.y, q.z, q.w):.3f}",
            f"{cov_xx:.4f}", lost,
        ])
        # Flush on a cadence rather than per row: per-row fsync on the Radxa's
        # eMMC costs more than the data is worth at 30 Hz odometry.
        if self.pose_count % 50 == 0:
            self._pose_fh.flush()

    def _log_progress(self):
        lost_pct = (100.0 * self.lost_count / self.pose_count
                    if self.pose_count else 0.0)
        self.get_logger().info(
            f"t={time.time() - self.t0:6.1f}s  maps={self.map_count} "
            f"thin={self.thin_count} poses={self.pose_count} "
            f"vio_lost={lost_pct:.1f}%")

    # ── finalisation ────────────────────────────────────────────────

    def finalize(self) -> str:
        """Write the grids, the render and meta.json. Safe to call twice."""
        try:
            self._pose_fh.flush()
            self._pose_fh.close()
        except Exception:
            pass

        if self.grid_raw is None:
            self.get_logger().error(
                "No /map received - nothing to evaluate. Is the pipeline up?")
        else:
            np.save(os.path.join(self.out_dir, "map.npy"), self.grid_raw)
            self._write_png("map.png", self.grid_raw)
        if self.grid_thin is not None:
            np.save(os.path.join(self.out_dir, "map_thin.npy"), self.grid_thin)
            self._write_png("map_thin.png", self.grid_thin)

        meta = dict(self.meta_raw)
        meta.update({
            "notes": self.notes,
            "duration_s": round(time.time() - self.t0, 2),
            "map_msgs": self.map_count,
            "map_thin_msgs": self.thin_count,
            "pose_samples": self.pose_count,
            "vo_lost_samples": self.lost_count,
            "thin_meta": self.meta_thin,
            "recorded_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "source": "map_recorder.py",
        })
        with open(os.path.join(self.out_dir, "meta.json"), "w",
                  encoding="utf-8") as fh:
            json.dump(meta, fh, indent=2)

        self.get_logger().info(f"Run written to {self.out_dir}")
        self.get_logger().info(
            f"Evaluate with: map_eval.py {self.out_dir}")
        return self.out_dir

    def _write_png(self, name: str, grid: np.ndarray) -> None:
        """Best-effort render. A missing cv2 must not cost us the .npy data."""
        try:
            import cv2
            cv2.imwrite(os.path.join(self.out_dir, name), render_grid(grid))
        except Exception as exc:
            self.get_logger().warn(f"could not write {name}: {exc}")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Record one SLAM run as artifacts for map_eval.py")
    parser.add_argument("--out", default="out_mapping",
                        help="parent directory for run folders (default out_mapping)")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="stop automatically after N seconds (0 = until Ctrl-C)")
    parser.add_argument("--notes", default="",
                        help="free text stored in meta.json, e.g. 'loop, lab, slow walk'")
    args, ros_args = parser.parse_known_args(argv)

    run_dir = os.path.join(args.out, time.strftime("run_%Y%m%d_%H%M%S"))

    rclpy.init(args=ros_args)
    node = MapRecorder(run_dir, notes=args.notes)
    deadline = time.time() + args.duration if args.duration > 0 else None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if deadline and time.time() >= deadline:
                node.get_logger().info("Duration reached, finalising...")
                break
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted, finalising...")
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
