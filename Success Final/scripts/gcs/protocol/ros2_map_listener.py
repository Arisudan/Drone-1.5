"""
================================================================================
MODULE: ros2_map_listener.py
PURPOSE: Dual-Transport (ROS 2 DDS + TCP Binary Bridge) SLAM Occupancy Grid Receiver
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Map Ingestion Thread)
  * Communicates:  Radxa Q6A Companion Computer over Wi-Fi / Ethernet
  * Upstream:      ROS 2 /map_thin, /map topics AND tcp_map_streamer_node.py (port 5765)
  * Downstream:    SlamMapWidget, A* Path Planner, and Tactical SLAM GUI

DATA FLOW & INTERFACES:
  * ROS 2 DDS:     Subscribes to /map_thin & /map [nav_msgs/OccupancyGrid] with
                   TRANSIENT_LOCAL durability & RELIABLE QoS (catches latched maps).
  * TCP Bridge:    Connects to 172.16.101.84:5765; receives DMAP binary framed packets
                   with zlib-decompressed int8 arrays and JSON origin/metadata.
  * Qt Signals:    map_received(grid, resolution, origin_x, origin_y, topic, QImage),
                   status_updated(str).

KEY LOGIC & FAILSAFES:
  * Zero Wi-Fi Multicast Dependency: Commercial Wi-Fi routers frequently block or
    drop DDS UDP multicast discovery packets. The built-in TCP client fallback (port 5765)
    guarantees map updates even when DDS discovery fails completely.
  * Fast In-Memory Processing: Unpacks compressed map payloads into 2D NumPy arrays
    (H x W) within milliseconds, bypassing Python interpreter latency.
  * Topic Prioritization: Prefers `/map_thin` (1-pixel skeleton) for clean wall
    boundaries while preserving `/map` (raw occupancy grid) as dense fallback.
  * Auto-Reconnection Loop: Automatically reconnects TCP socket with backoff if
    link drops during flight.

USAGE:
  listener = ROS2MapListener(tcp_host="172.16.101.84", tcp_port=5765)
  listener.map_received.connect(self.update_map_display)
  listener.start()
================================================================================
"""

from __future__ import annotations
import json
import os
import socket
import struct
import sys
import time
import threading
import zlib
from typing import Optional, Tuple
import numpy as np

# Ensure ROS 2 Jazzy paths are in sys.path
for _ros_path in [
    "/opt/ros/jazzy/lib/python3.12/site-packages",
    "/opt/ros/jazzy/lib/python3.10/site-packages",
]:
    if os.path.exists(_ros_path) and _ros_path not in sys.path:
        sys.path.insert(0, _ros_path)

from PyQt5.QtCore import QThread, pyqtSignal

from core.map_render import build_map_image

# Make the gcs package root importable even when this module is loaded
# directly (tests, diagnostics) rather than via drone_gcs.py.
_GCS_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _GCS_ROOT not in sys.path:
    sys.path.insert(0, _GCS_ROOT)

from core.health import EngineHealth, EngineStatus, get_registry

# Safely test for rclpy availability
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from std_srvs.srv import Empty, Trigger
    ROS2_AVAILABLE = True
except Exception:
    ROS2_AVAILABLE = False

MAGIC = b"DMAP"
RESULT_MAGIC = b"RSLT"  # matches tcp_map_streamer_node.py's command-reply framing


class ROS2MapListener(QThread):
    """
    Background worker thread listening for 2D SLAM OccupancyGrid maps.
    Uses native ROS 2 topic subscriptions (/map_thin, /map) when available,
    and concurrently runs an automatic TCP client fallback (port 5765)
    to guarantee map delivery even if Wi-Fi access points block UDP multicast.
    """

    # Emits: (grid_2d_numpy, resolution_m, origin_x_m, origin_y_m, topic_name)
    # (grid, resolution, origin_x, origin_y, topic, QImage). The image is built
    # here rather than in the canvas: the conversion is a lookup over every cell
    # and costs ~12 ms on a 25 x 25 m room at 2.5 cm, and it used to run on the
    # GUI thread at the stream's rate - on the same thread as the 10 Hz OFFBOARD
    # setpoint pump, whose PX4 deadline is 500 ms. QImage is safe to build on a
    # worker thread; QPixmap would not be.
    map_received = pyqtSignal(object, float, float, float, str, object)
    status_updated = pyqtSignal(str)

    def __init__(self, tcp_host: str = "172.16.101.84", tcp_port: int = 5765,
                 parent=None, stall_after_s: float = 20.0,
                 degrade_after_s: float = 8.0):
        super().__init__(parent)
        self._running = False
        self._node = None
        self._last_ros2_map_thin_time = 0.0
        self._last_ros2_map_raw_time = 0.0
        self._last_emitted_thin_stamp = 0.0
        self._last_emitted_raw_stamp = 0.0
        self.latest_map_info: Optional[dict] = None
        self.latest_raw_map_info: Optional[dict] = None
        self.latest_thin_map_info: Optional[dict] = None

        self._tcp_host = tcp_host
        self._tcp_port = tcp_port
        self._tcp_thread: Optional[threading.Thread] = None

        # Liveness bookkeeping. Without this a dead map feed is invisible:
        # the canvas keeps painting the last good grid and looks identical
        # to a live one. Thresholds are deliberately generous - RTAB-Map
        # runs at Rtabmap/DetectionRate=2.0 Hz but only republishes the
        # grid as the map actually changes, so a stationary drone can go
        # quiet legitimately. 20 s of total silence means the link or the
        # node is gone, not that nothing moved.
        self.health = EngineHealth(
            "MapListener",
            stall_after_s=stall_after_s,
            degrade_after_s=degrade_after_s,
        )
        get_registry().register(self.health)

    def set_tcp_host(self, host: str):
        """Update target IP for TCP streaming fallback (e.g. Radxa IP)."""
        self._tcp_host = host

    def run(self):
        """Worker loop running ROS 2 executor and background TCP fallback."""
        self._running = True
        self.health.set_status(EngineStatus.STARTING)

        # Always start background TCP fallback thread
        self._tcp_thread = threading.Thread(target=self._run_tcp_client, daemon=True)
        self._tcp_thread.start()

        # READY the moment a transport is up. Whether maps actually arrive is
        # what the heartbeats answer - this only says the listener is alive.
        self.health.set_status(EngineStatus.READY)

        if not ROS2_AVAILABLE:
            self.status_updated.emit("ROS 2 (rclpy) not available on GCS. Using TCP Map Stream fallback.")
            while self._running:
                time.sleep(0.5)
            return

        try:
            if not rclpy.ok():
                rclpy.init(args=None)

            self._node = Node("drone_gcs_map_listener")
            self.status_updated.emit("ROS 2 node created. Subscribing to /map_thin & /map...")

            # Transient Local QoS matches map_thinning_node and RTAB-Map latched topics
            qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )

            # Subscriptions
            self._node.create_subscription(
                OccupancyGrid,
                "/map_thin",
                self._on_map_thin_received,
                qos,
            )
            self._node.create_subscription(
                OccupancyGrid,
                "/map",
                self._on_map_raw_received,
                qos,
            )

            self.status_updated.emit("Listening for live /map_thin / /map (ROS 2 DDS + TCP Bridge)...")

            # Spin in loop with short timeout to allow responsive stopping
            while self._running and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.1)

        except Exception as e:
            self.status_updated.emit(f"ROS 2 listener error: {e}. TCP fallback remains active.")
            while self._running:
                time.sleep(0.5)
        finally:
            self._cleanup()

    def _run_tcp_client(self):
        """Persistent TCP client thread receiving maps over port 5765."""
        while self._running:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(3.0)
                sock.connect((self._tcp_host, self._tcp_port))
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.status_updated.emit(f"Connected to TCP Map Bridge at {self._tcp_host}:{self._tcp_port}")

                sock.settimeout(5.0)
                while self._running:
                    # Read 4 bytes magic
                    magic = self._recv_exact(sock, 4)
                    if not magic:
                        break
                    if magic != MAGIC:
                        continue

                    # Read header length (4 bytes unsigned int)
                    hdr_len_bytes = self._recv_exact(sock, 4)
                    if not hdr_len_bytes:
                        break
                    hdr_len = struct.unpack("!I", hdr_len_bytes)[0]
                    # Item 18: Bounded length checks (<= 64KB)
                    if hdr_len == 0 or hdr_len > 65536:
                        self.status_updated.emit(f"TCP Map Bridge: Invalid header length {hdr_len}, reconnecting...")
                        break

                    # Read header bytes
                    hdr_bytes = self._recv_exact(sock, hdr_len)
                    if not hdr_bytes:
                        break

                    # Item 19: Per-frame try/except isolation
                    try:
                        meta = json.loads(hdr_bytes.decode("utf-8"))
                    except Exception as json_err:
                        self.status_updated.emit(f"TCP Map Bridge: JSON decode error ({json_err}), skipping frame")
                        continue

                    # Read payload length (4 bytes unsigned int)
                    pay_len_bytes = self._recv_exact(sock, 4)
                    if not pay_len_bytes:
                        break
                    pay_len = struct.unpack("!I", pay_len_bytes)[0]
                    # Item 18: Bounded length checks (<= 20MB)
                    if pay_len == 0 or pay_len > 20971520:
                        self.status_updated.emit(f"TCP Map Bridge: Invalid payload length {pay_len}, reconnecting...")
                        break

                    # Read payload bytes
                    compressed_payload = self._recv_exact(sock, pay_len)
                    if not compressed_payload:
                        break

                    # Item 19: Per-frame try/except isolation for decompression and array construction
                    _t0 = time.perf_counter()
                    try:
                        source_name = meta.get("source", "map_thin")
                        is_thin = "thin" in source_name.lower()
                        pkt_ts = float(meta.get("timestamp", time.time()))

                        # Items 15 & 21: Per-topic TCP suppression with timestamp validation
                        if is_thin:
                            if time.time() - self._last_ros2_map_thin_time < 2.0:
                                continue  # Native DDS /map_thin is active; suppress TCP duplicate
                            if pkt_ts < self._last_emitted_thin_stamp:
                                continue  # Discard out-of-order or stale frame
                            self._last_emitted_thin_stamp = pkt_ts
                        else:
                            if time.time() - self._last_ros2_map_raw_time < 2.0:
                                continue  # Native DDS /map is active; suppress TCP duplicate
                            if pkt_ts < self._last_emitted_raw_stamp:
                                continue  # Discard out-of-order or stale frame
                            self._last_emitted_raw_stamp = pkt_ts

                        # Decompress and reconstruct grid
                        raw_bytes = zlib.decompress(compressed_payload)
                        h, w = meta["height"], meta["width"]
                        raw_grid = np.frombuffer(raw_bytes, dtype=np.int8).reshape((h, w))
                        # Transpose so Axis 0 is ROS X (North / forward) and Axis 1 is ROS Y (East)
                        grid = np.ascontiguousarray(raw_grid.T)

                        map_info = {
                            "grid": grid,
                            "resolution": meta["resolution"],
                            "origin_x": meta["origin_x"],
                            "origin_y": meta["origin_y"],
                            "width": w,
                            "height": h,
                            "topic": source_name,
                            "timestamp": pkt_ts,
                        }

                        self.latest_map_info = map_info
                        if is_thin:
                            self.latest_thin_map_info = map_info
                        else:
                            self.latest_raw_map_info = map_info

                        self.health.heartbeat()
                        self.health.record_latency((time.perf_counter() - _t0) * 1000.0)

                        image = build_map_image(grid, thin=is_thin)
                        self.map_received.emit(
                            grid, meta["resolution"], meta["origin_x"],
                            meta["origin_y"], source_name, image
                        )
                    except Exception as frame_err:
                        self.status_updated.emit(f"TCP Map Bridge: Frame decode error ({frame_err}), continuing...")
                        continue

                sock.close()
            except Exception:
                # Brief backoff before reconnect attempt
                time.sleep(3.0)

    def _recv_exact(self, sock: socket.socket, num_bytes: int) -> Optional[bytes]:
        data = bytearray()
        while len(data) < num_bytes and self._running:
            try:
                chunk = sock.recv(num_bytes - len(data))
                if not chunk:
                    return None
                data.extend(chunk)
            except socket.timeout:
                continue
            except Exception:
                return None
        return bytes(data)

    def _on_map_thin_received(self, msg: OccupancyGrid):
        """Handle 1-pixel skeletonized thin map."""
        self._last_ros2_map_thin_time = time.time()
        self._process_and_emit(msg, "/map_thin")

    def _on_map_raw_received(self, msg: OccupancyGrid):
        """Handle raw occupancy grid (full obstacle volume & explored space)."""
        self._last_ros2_map_raw_time = time.time()
        self._process_and_emit(msg, "/map")

    def _process_and_emit(self, msg: OccupancyGrid, topic_name: str):
        _t0 = time.perf_counter()
        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        if width <= 0 or height <= 0:
            return

        # Reshape flat 1D data into (height, width) 2D array, transposed to (width, height)
        # so Axis 0 is ROS X (North / forward) and Axis 1 is ROS Y (East)
        raw_grid = np.array(msg.data, dtype=np.int8).reshape((height, width))
        grid = np.ascontiguousarray(raw_grid.T)

        now = time.time()
        map_info = {
            "grid": grid,
            "resolution": res,
            "origin_x": ox,
            "origin_y": oy,
            "width": width,
            "height": height,
            "topic": topic_name,
            "timestamp": now,
        }

        self.latest_map_info = map_info
        if "thin" in topic_name.lower():
            self.latest_thin_map_info = map_info
            self._last_emitted_thin_stamp = now
        else:
            self.latest_raw_map_info = map_info
            self._last_emitted_raw_stamp = now

        self.health.heartbeat()
        self.health.record_latency((time.perf_counter() - _t0) * 1000.0)

        image = build_map_image(grid, thin="thin" in topic_name.lower())
        self.map_received.emit(grid, res, ox, oy, topic_name, image)

    def stop(self):
        """Request graceful shutdown of the worker."""
        self._running = False
        # Deliberate shutdown - absence of heartbeats is expected from here
        # on, so the watchdog must not report it as a stall.
        self.health.set_status(EngineStatus.STOPPED)
        self.wait(1000)

    @property
    def tcp_host(self) -> str:
        """Current Radxa IP, for SlamMapResetWorker's TCP fallback to reuse."""
        return self._tcp_host

    def _cleanup(self):
        if self._node:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None

    @staticmethod
    def generate_bench_mock_map() -> Tuple[np.ndarray, float, float, float]:
        """
        Generate a realistic 10m x 10m indoor room floorplan for bench testing
        when the real D435i / RTAB-Map SLAM node is not running.
        Resolution: 0.05m (200x200 grid).
        Features: Outer perimeter walls, an interior partition wall, and a doorway.
        """
        resolution = 0.05  # 5 cm per cell
        width = 200
        height = 200
        ox = -5.0  # Centers drone (0,0) inside the room
        oy = -5.0

        # Start with free space (0)
        grid = np.zeros((height, width), dtype=np.int8)

        # Outer walls (border)
        grid[10:12, 10:190] = 100   # South wall
        grid[188:190, 10:190] = 100 # North wall
        grid[10:190, 10:12] = 100   # West wall
        grid[10:190, 188:190] = 100 # East wall

        # Interior divider wall from East to center with a 1.2m doorway
        # Divide horizontally at Y = 0 (row 100)
        grid[99:101, 30:90] = 100   # Left section
        # Doorway: row 99:101, cols 90:120 is free (door opening ~1.5m)
        grid[99:101, 120:170] = 100 # Right section

        # A pillar / obstacle at (X=1.5, Y=1.5) -> row 130, col 130
        grid[125:135, 125:135] = 100

        # Unknown border cells outside outer walls
        grid[0:10, :] = -1
        grid[190:200, :] = -1
        grid[:, 0:10] = -1
        grid[:, 190:200] = -1

        return grid, resolution, ox, oy


class SlamMapResetWorker(QThread):
    """One-shot worker: wipes the live SLAM map on the Radxa - /rtabmap/reset clears
    RTAB-Map's own database and restarts mapping from scratch, and
    /map_thinning_node/reset clears that node's independent wall-lock buffer (without
    it, /map_thin would keep showing ghost walls from the old map for a while after
    the underlying /map has already gone empty).

    ROS 2 service calls are the primary path. If those aren't reachable (DDS service
    discovery can be as flaky over Wi-Fi as the topic discovery this project already
    works around for map data), it falls back to a plain TCP command to
    tcp_map_streamer_node.py's command relay - the same proven-reliable link already
    used for the map data itself.

    Never touches the shared rclpy context (no init()/shutdown() here) - the main
    ROS2MapListener thread owns that lifecycle; this worker only creates and destroys
    its own throwaway Node.
    """

    finished_result = pyqtSignal(bool, str)  # (success, human-readable message)

    def __init__(self, tcp_host: str, tcp_port: int = 5765, parent=None):
        super().__init__(parent)
        self.tcp_host = tcp_host
        self.tcp_port = tcp_port

    def run(self):
        ros2_msg = "ROS 2 not available on this GCS"
        if ROS2_AVAILABLE and rclpy.ok():
            ok, ros2_msg = self._try_ros2()
            if ok:
                self.finished_result.emit(True, ros2_msg)
                return

        ok, tcp_msg = self._try_tcp()
        if ok:
            self.finished_result.emit(True, tcp_msg)
        else:
            self.finished_result.emit(False, f"ROS 2: {ros2_msg} | TCP fallback: {tcp_msg}")

    def _try_ros2(self):
        node = None
        try:
            node = Node("gcs_slam_reset_client")
            rtabmap_client = node.create_client(Empty, "/rtabmap/reset")
            thinning_client = node.create_client(Trigger, "/map_thinning_node/reset")

            if not rtabmap_client.wait_for_service(timeout_sec=3.0):
                return False, "/rtabmap/reset not reachable (service discovery)"
            fut1 = rtabmap_client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(node, fut1, timeout_sec=5.0)
            if fut1.result() is None:
                return False, "/rtabmap/reset call timed out"

            if not thinning_client.wait_for_service(timeout_sec=3.0):
                return False, "map wiped, but /map_thinning_node/reset not reachable"
            fut2 = thinning_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, fut2, timeout_sec=5.0)
            if fut2.result() is None or not fut2.result().success:
                return False, "map wiped, but wall-lock buffer reset failed"

            return True, "SLAM map reset (ROS 2)"
        except Exception as e:
            return False, f"ROS 2 reset error: {e}"
        finally:
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass

    def _try_tcp(self):
        """This socket is also broadcasting ordinary DMAP map packets concurrently -
        tcp_map_streamer_node.py sends any newly connected client its cached maps
        immediately on accept, racing with our command reply. A plain recv() here
        would silently return a fragment of a map packet instead of our reply, so
        this reads the stream frame-by-frame, skipping past any DMAP packets and
        stopping only at the RSLT frame that's actually our command's answer."""
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(8.0)
            sock.connect((self.tcp_host, self.tcp_port))
            sock.sendall(b"RESET\n")

            deadline = time.time() + 10.0
            while time.time() < deadline:
                magic = self._recv_exact(sock, 4)
                if not magic:
                    return False, "connection closed before a reply arrived"
                if magic == RESULT_MAGIC:
                    length_bytes = self._recv_exact(sock, 4)
                    if not length_bytes:
                        return False, "connection closed mid-reply"
                    length = struct.unpack("!I", length_bytes)[0]
                    body = self._recv_exact(sock, length)
                    if body is None:
                        return False, "connection closed mid-reply"
                    text = body.decode(errors="replace")
                    return text.startswith("OK"), text
                elif magic == MAGIC:
                    # An interleaved map packet, not our reply - skip over it (same
                    # framing _run_tcp_client parses elsewhere in this file) and keep
                    # waiting for the RSLT frame.
                    hdr_len_bytes = self._recv_exact(sock, 4)
                    if not hdr_len_bytes:
                        return False, "connection closed while skipping a map packet"
                    hdr_len = struct.unpack("!I", hdr_len_bytes)[0]
                    if self._recv_exact(sock, hdr_len) is None:
                        return False, "connection closed while skipping a map packet"
                    pay_len_bytes = self._recv_exact(sock, 4)
                    if not pay_len_bytes:
                        return False, "connection closed while skipping a map packet"
                    pay_len = struct.unpack("!I", pay_len_bytes)[0]
                    if self._recv_exact(sock, pay_len) is None:
                        return False, "connection closed while skipping a map packet"
                    continue
                else:
                    return False, f"unrecognized frame magic {magic!r}"
            return False, "timed out waiting for a reply behind interleaved map traffic"
        except Exception as e:
            return False, f"{e}"
        finally:
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass

    @staticmethod
    def _recv_exact(sock: socket.socket, n: int) -> Optional[bytes]:
        data = bytearray()
        while len(data) < n:
            try:
                chunk = sock.recv(n - len(data))
            except socket.timeout:
                return None
            if not chunk:
                return None
            data.extend(chunk)
        return bytes(data)
