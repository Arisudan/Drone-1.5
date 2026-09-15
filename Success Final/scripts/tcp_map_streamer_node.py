#!/usr/bin/env python3
"""
================================================================================
MODULE: tcp_map_streamer_node.py
PURPOSE: Compresses & Streams 2D SLAM Occupancy Grids from Radxa to Laptop GCS
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Radxa Q6A Companion Computer (Onboard System)
  * Communicates:  Laptop GCS over Private Wi-Fi Access Point (TCP Port 5765)
  * Upstream:      rtabmap_slam (/map) and map_thinning_node (/map_thin)
  * Downstream:    Laptop GCS ros2_map_listener.py

DATA FLOW & INTERFACES:
  * Subscribes To: /map [nav_msgs/OccupancyGrid] (Raw 2.5cm SLAM occupancy grid)
                   /map_thin [nav_msgs/OccupancyGrid] (Single-pixel skeleton walls)
  * TCP Server:    0.0.0.0:5765 (Accepts incoming connections from Laptop GCS)
  * Wire Protocol: Custom Framed Binary:
                   [4-byte Magic: 0x4D415053 ("MAPS")]
                   [4-byte Header Length (Big-Endian)]
                   [JSON Metadata (width, height, resolution, origin_x, origin_y, topic)]
                   [4-byte Payload Length (Big-Endian)]
                   [zlib Level-1 Compressed Int8 Raw Grid Bytes]

KEY LOGIC & FAILSAFES:
  * Multicast UDP Bypass: Solves Wi-Fi router AP-isolation and IGMP snooping bugs
    that block ROS 2 DDS UDP discovery packets between Radxa and Laptop.
  * Fast zlib (Level 1): Compresses a 200x200 grid from 40KB down to sub-10KB in <1.5ms.
  * Non-Blocking Broadcast: Drops disconnected clients and never blocks the ROS 2 loop.
  * Multi-Client Broadcast: Multiple GCS laptops can connect simultaneously.

RUN:
  source /opt/ros/jazzy/setup.bash
  python3 tcp_map_streamer_node.py --port 5765 --rate 5
================================================================================
"""

from __future__ import annotations
import os
import sys
import json
import math
import socket
import struct
import threading
import time
import zlib
from typing import Set, Optional

# Ensure ROS 2 Jazzy paths are in sys.path
for _ros_path in [
    "/opt/ros/jazzy/lib/python3.12/site-packages",
    "/opt/ros/jazzy/lib/python3.10/site-packages",
]:
    if os.path.exists(_ros_path) and _ros_path not in sys.path:
        sys.path.insert(0, _ros_path)

import numpy as np
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from std_srvs.srv import Empty, Trigger
except ImportError:
    Node = object
    QoSProfile = None
    DurabilityPolicy = None
    ReliabilityPolicy = None
    OccupancyGrid = None
    Empty = None
    Trigger = None


MAGIC = b"DMAP"  # Drone Map Header Magic
RESULT_MAGIC = b"RSLT"  # Command reply framing - distinct from DMAP map packets so a
                         # one-shot command client (see SlamMapResetWorker on the GCS)
                         # can tell a command reply apart from ordinary map broadcast
                         # traffic interleaved on the same socket, rather than the
                         # reply getting silently swallowed by/mixed into a map packet.


class TCPMapStreamerNode(Node):
    """
    Subscribes to local /map_thin and /map on Radxa and broadcasts compressed
    occupancy grids over TCP (port 5765) to connected laptop Ground Control Stations.
    """

    def __init__(self):
        super().__init__("tcp_map_streamer_node")

        self.declare_parameter("port", 5765)
        self.declare_parameter("bind_host", "0.0.0.0")

        self.port = int(self.get_parameter("port").value)
        self.bind_host = str(self.get_parameter("bind_host").value)

        # TCP Clients and cache lock (avoid colliding with rclpy.node.Node._clients)
        self._tcp_clients: Set[socket.socket] = set()
        self._tcp_clients_lock = threading.Lock()
        self._latest_packet_raw: Optional[bytes] = None
        self._latest_packet_thin: Optional[bytes] = None
        self._last_map_origin = {"map_raw": None, "map_thin": None}
        self._last_map_shape = {"map_raw": None, "map_thin": None}
        self._seq = 0
        self._running = True

        # QoS profile matching map_thinning_node (Transient Local)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.sub_thin = self.create_subscription(
            OccupancyGrid,
            "/map_thin",
            self._on_map_thin,
            map_qos,
        )

        self.sub_raw = self.create_subscription(
            OccupancyGrid,
            "/map",
            self._on_map_raw,
            map_qos,
        )

        # Reset service clients - this node already has a live, proven-reliable TCP
        # link to whatever GCS is connected (see _run_command_reader below), so it
        # doubles as the fallback path for a GCS-triggered map reset when the native
        # ROS 2 service call itself is unreachable over a flaky Wi-Fi link.
        self._reset_rtabmap_client = self.create_client(Empty, "/rtabmap/reset")
        self._reset_thinning_client = self.create_client(Trigger, "/map_thinning_node/reset")

        # Start TCP Server thread
        self._server_thread = threading.Thread(target=self._run_tcp_server, daemon=True)
        self._server_thread.start()

        # Poll connected clients for an inbound "RESET" command line (this server was
        # write-only/broadcast-only until now - clients can send one short command back).
        self._cmd_thread = threading.Thread(target=self._run_command_reader, daemon=True)
        self._cmd_thread.start()

        self.get_logger().info(
            f"TCP Map Streamer Bridge active on {self.bind_host}:{self.port} (Fallback for Wi-Fi multicast)"
        )

    def _run_tcp_server(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server_sock.bind((self.bind_host, self.port))
            server_sock.listen(5)
            server_sock.settimeout(1.0)
            self.get_logger().info(f"TCP server listening on {self.bind_host}:{self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to bind TCP server on port {self.port}: {e}")
            return

        while self._running:
            try:
                client_sock, addr = server_sock.accept()
                client_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                # Item 17: Set 200ms socket timeout to prevent blocked Wi-Fi clients from hanging ROS 2 executor
                client_sock.settimeout(0.20)
                self.get_logger().info(f"GCS Laptop connected from {addr[0]}:{addr[1]}")

                with self._tcp_clients_lock:
                    self._tcp_clients.add(client_sock)
                    # Send immediately cached raw and thin maps if available
                    if self._latest_packet_raw:
                        try:
                            client_sock.sendall(self._latest_packet_raw)
                        except Exception:
                            pass
                    if self._latest_packet_thin:
                        try:
                            client_sock.sendall(self._latest_packet_thin)
                        except Exception:
                            pass
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    self.get_logger().warn(f"TCP accept error: {e}")
                break

        server_sock.close()

    def _on_map_thin(self, msg: OccupancyGrid):
        self._process_and_broadcast(msg, "map_thin")

    def _on_map_raw(self, msg: OccupancyGrid):
        self._process_and_broadcast(msg, "map_raw")

    def _process_and_broadcast(self, msg: OccupancyGrid, source: str):
        w = msg.info.width
        h = msg.info.height
        if w <= 0 or h <= 0:
            return

        self._seq += 1
        meta = {
            "seq": self._seq,
            "source": source,
            "width": w,
            "height": h,
            "resolution": float(msg.info.resolution),
            "origin_x": float(msg.info.origin.position.x),
            "origin_y": float(msg.info.origin.position.y),
            "timestamp": time.time(),
        }

        # Item 22: Detect map reset / origin jump and invalidate stale cached packets
        cur_ox = float(msg.info.origin.position.x)
        cur_oy = float(msg.info.origin.position.y)
        prev_orig = self._last_map_origin.get(source)
        prev_shp = self._last_map_shape.get(source)
        if prev_orig is not None:
            if math.hypot(cur_ox - prev_orig[0], cur_oy - prev_orig[1]) > 0.5:
                with self._tcp_clients_lock:
                    if "thin" in source.lower():
                        self._latest_packet_thin = None
                    else:
                        self._latest_packet_raw = None
        self._last_map_origin[source] = (cur_ox, cur_oy)
        self._last_map_shape[source] = (w, h)

        header_bytes = json.dumps(meta).encode("utf-8")
        # Fast zero-copy / buffer conversion
        if isinstance(msg.data, (bytes, bytearray)):
            raw_bytes = bytes(msg.data)
        elif hasattr(msg.data, "tobytes"):
            raw_bytes = msg.data.tobytes()
        else:
            raw_bytes = bytes(np.array(msg.data, dtype=np.int8))
        compressed_payload = zlib.compress(raw_bytes, level=1)

        # Wire format: MAGIC (4) + hdr_len (4) + header_bytes + payload_len (4) + compressed_payload
        packet = (
            MAGIC
            + struct.pack("!I", len(header_bytes))
            + header_bytes
            + struct.pack("!I", len(compressed_payload))
            + compressed_payload
        )

        with self._tcp_clients_lock:
            if "thin" in source.lower():
                self._latest_packet_thin = packet
            else:
                self._latest_packet_raw = packet

            disconnected = []
            for client in self._tcp_clients:
                try:
                    client.sendall(packet)
                except Exception:
                    disconnected.append(client)

            for dead in disconnected:
                self._tcp_clients.discard(dead)
                try:
                    dead.close()
                except Exception:
                    pass

    def _run_command_reader(self):
        """Poll each connected client for a short inbound command line. Non-blocking
        (short per-socket timeout) so this never stalls the broadcast path above -
        that's the actual map-streaming hot path and must stay unaffected."""
        while self._running:
            with self._tcp_clients_lock:
                clients = list(self._tcp_clients)
            for client in clients:
                try:
                    client.settimeout(0.05)
                    data = client.recv(64)
                except (socket.timeout, BlockingIOError):
                    continue
                except Exception:
                    continue  # disconnects are handled by the broadcast loop's own send failures
                finally:
                    try:
                        client.settimeout(0.20)  # restore the accept-loop default
                    except Exception:
                        pass
                if data and b"RESET" in data.upper():
                    peer = "unknown"
                    try:
                        peer = client.getpeername()
                    except Exception:
                        pass
                    self.get_logger().warn(f"RESET command received from {peer}")
                    self._handle_reset_command(client)
            time.sleep(0.1)

    def _handle_reset_command(self, client_sock: socket.socket):
        """Wipe the live SLAM map: /rtabmap/reset clears RTAB-Map's own database and
        restarts mapping from scratch; map_thinning_node's reset clears its independent
        wall-lock buffer so /map_thin doesn't keep showing ghost walls from the old map.
        Blocking .call() is safe here because main() runs a MultiThreadedExecutor -
        this command-reader thread and the node's own callback processing don't
        contend for the same single-threaded spin loop."""
        ok, msg = True, []

        if not self._reset_rtabmap_client.wait_for_service(timeout_sec=2.0):
            ok, msg = False, ["/rtabmap/reset service not available"]
        else:
            try:
                self._reset_rtabmap_client.call(Empty.Request())
                msg.append("rtabmap memory reset")
            except Exception as e:
                ok = False
                msg.append(f"rtabmap reset failed: {e}")

        if not self._reset_thinning_client.wait_for_service(timeout_sec=2.0):
            ok, msg = False, msg + ["/map_thinning_node/reset service not available"]
        else:
            try:
                resp = self._reset_thinning_client.call(Trigger.Request())
                msg.append(f"thinning reset: {resp.message}")
            except Exception as e:
                ok = False
                msg.append(f"thinning reset failed: {e}")

        reply_text = (("OK: " if ok else "ERR: ") + "; ".join(msg)).encode()
        # RESULT_MAGIC + 4-byte length prefix, matching the DMAP packets' own framing
        # style - this socket is also broadcasting ordinary map packets concurrently
        # (any newly connected client is sent cached maps immediately on accept), so
        # the reply needs its own distinguishable frame, not a bare newline-terminated
        # string that could land anywhere relative to interleaved map bytes.
        reply = RESULT_MAGIC + struct.pack("!I", len(reply_text)) + reply_text
        try:
            client_sock.sendall(reply)
        except Exception:
            pass

    def destroy_node(self):
        self._running = False
        with self._tcp_clients_lock:
            for client in self._tcp_clients:
                try:
                    client.close()
                except Exception:
                    pass
            self._tcp_clients.clear()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TCPMapStreamerNode()
    # MultiThreadedExecutor, not the default single-threaded rclpy.spin(): the command
    # reader thread makes blocking service .call()s (to /rtabmap/reset and
    # /map_thinning_node/reset) which need the executor able to process their
    # responses concurrently rather than waiting on this same node's own spin loop.
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
