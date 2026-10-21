#!/usr/bin/env python3
"""
================================================================================
MODULE: udp_mavlink_bridge.py
PURPOSE: High-Performance Bidirectional UDP 14550 <-> TCP 5760 MAVLink Relay
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Radxa Q6A Companion Computer (Background daemon or standalone)
  * Communicates:  Laptop GCS over Wi-Fi (UDP 14550) <-> mavlink-routerd (TCP 5760)
  * Upstream:      Laptop GCS commands, heartbeats, and offboard setpoints over UDP
  * Downstream:    Pixhawk 6X autopilot over UART (/dev/ttyACM0 at 921600 baud)

DATA FLOW & INTERFACES:
  * Inbound UDP:   0.0.0.0:14550 -> receives GCS commands and heartbeats from laptop.
  * Inbound TCP:   127.0.0.1:5760 -> receives Pixhawk telemetry and ACKs.
  * Outbound UDP:  Sends Pixhawk packets back to client_ip:client_port dynamically.
  * Outbound TCP:  Forwards GCS commands to Pixhawk via local mavlink-routerd socket.

KEY LOGIC & FAILSAFES:
  * Auto-Learning Routing: Learns Laptop GCS IP and port from incoming UDP packets
    and multiplexes Pixhawk telemetry back to all connected GCS clients.
  * Internal Echo Filtering: Identifies and discards mavlink-routerd loopback packets
    to prevent echo storms while allowing genuine local/remote test clients.
  * Continuous 1 Hz Heartbeat Injection: Maintains active stream registration on TCP 5760.

USAGE:
  python3 scripts/network/udp_mavlink_bridge.py [--udp-port 14550] [--tcp-port 5760]
================================================================================
"""

import sys
import socket
import select
import time
import argparse


def build_gcs_heartbeat_mavlink2(seq: int = 0) -> bytes:
    """Build standard MAVLink 2 Heartbeat packet for GCS (sysid 255, compid 190)."""
    custom_mode = 0
    mtype = 6          # MAV_TYPE_GCS
    autopilot = 0      # MAV_AUTOPILOT_INVALID
    base_mode = 0
    system_status = 4  # MAV_STATE_ACTIVE
    mavlink_version = 3

    payload = bytearray(9)
    payload[0:4] = custom_mode.to_bytes(4, byteorder="little")
    payload[4] = mtype
    payload[5] = autopilot
    payload[6] = base_mode
    payload[7] = system_status
    payload[8] = mavlink_version

    header = bytearray(10)
    header[0] = 0xFD   # MAVLink 2 magic
    header[1] = len(payload)
    header[2] = 0      # incompat_flags
    header[3] = 0      # compat_flags
    header[4] = seq & 0xFF
    header[5] = 255    # sysid
    header[6] = 190    # compid
    header[7] = 0      # msgid byte 0
    header[8] = 0      # msgid byte 1
    header[9] = 0      # msgid byte 2

    # CRC-16/MCRF4XX
    crc = 0xFFFF
    for b in header[1:] + payload:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
    # CRC extra for HEARTBEAT is 50
    tmp = 50 ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)

    return bytes(header + payload + crc.to_bytes(2, byteorder="little"))


def run_bridge(udp_port: int = 14550, tcp_host: str = "127.0.0.1", tcp_port: int = 5760):
    print("=" * 80)
    print(f"UDP MAVLINK BRIDGE: 0.0.0.0:{udp_port} <--> {tcp_host}:{tcp_port}")
    print("=" * 80)

    # 1. Bind UDP Server Socket on 0.0.0.0
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        udp_sock.bind(("0.0.0.0", udp_port))
        udp_sock.setblocking(False)
        print(f"[UDP Server] Listening on 0.0.0.0:{udp_port}")
    except OSError as e:
        if e.errno == 98:
            print(f"[NOTICE] Port {udp_port} is already bound natively by mavlink-routerd.")
            return 0
        else:
            print(f"[ERROR] Failed to bind UDP port {udp_port}: {e}")
            return 1

    # 2. Connect to local TCP mavlink-routerd
    print(f"[TCP Client] Connecting to mavlink-routerd at {tcp_host}:{tcp_port}...")
    tcp_sock = None

    def connect_tcp():
        nonlocal tcp_sock
        if tcp_sock:
            try:
                tcp_sock.close()
            except Exception:
                pass
            tcp_sock = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.connect((tcp_host, tcp_port))
            s.setblocking(False)
            tcp_sock = s
            # Send initial GCS heartbeat so mavlink-router registers this client immediately
            hb = build_gcs_heartbeat_mavlink2(0)
            tcp_sock.sendall(hb)
            print(f"[TCP Client] Connected to mavlink-routerd at {tcp_host}:{tcp_port} and registered GCS stream.")
            return True
        except Exception as e:
            tcp_sock = None
            return False

    connect_tcp()

    clients = set()
    last_client_seen = {}
    last_hb_time = time.time()
    hb_seq = 1
    rx_bytes_udp = 0
    tx_bytes_udp = 0
    last_stats = time.time()

    print("[RUNNING] Real-time UDP flight relay active. Waiting for GCS connection...")

    try:
        while True:
            now = time.time()

            # 1. Maintain TCP stream registration with 1 Hz GCS Heartbeat
            if tcp_sock and (now - last_hb_time >= 1.0):
                try:
                    hb = build_gcs_heartbeat_mavlink2(hb_seq)
                    tcp_sock.sendall(hb)
                    hb_seq = (hb_seq + 1) & 0xFF
                    last_hb_time = now
                except Exception:
                    tcp_sock = None

            # 2. Periodic stats and stale client pruning
            if now - last_stats >= 5.0:
                client_list_str = ", ".join(f"{c[0]}:{c[1]}" for c in clients) if clients else "None"
                print(f"[STATS] Clients: [{client_list_str}] | UDP RX: {rx_bytes_udp/1024:.1f} KB | UDP TX: {tx_bytes_udp/1024:.1f} KB")
                rx_bytes_udp = 0
                tx_bytes_udp = 0
                last_stats = now

                stale = [c for c, t in last_client_seen.items() if now - t > 15.0]
                for c in stale:
                    print(f"[CLIENT TIMEOUT] Removed idle client: {c[0]}:{c[1]}")
                    clients.discard(c)
                    del last_client_seen[c]

            # 3. Reconnect TCP if broken
            if tcp_sock is None:
                time.sleep(0.5)
                connect_tcp()
                continue

            # 4. Multiplex UDP <-> TCP
            rlist = [udp_sock, tcp_sock]
            readable, _, exceptional = select.select(rlist, [], rlist, 0.05)

            if exceptional:
                if tcp_sock in exceptional:
                    print("[TCP Exception] Reconnecting...")
                    tcp_sock.close()
                    tcp_sock = None
                    continue

            for s in readable:
                if s is udp_sock:
                    # Inbound datagram on 0.0.0.0:14550
                    try:
                        data, addr = udp_sock.recvfrom(4096)
                        if not data:
                            continue

                        # Filter out mavlink-routerd internal echo on 127.0.0.1
                        if addr[0].startswith("127."):
                            continue

                        # Genuine GCS packet (Laptop or test client)
                        rx_bytes_udp += len(data)
                        if addr not in clients:
                            clients.add(addr)
                            print(f"[NEW GCS CLIENT] Registered: {addr[0]}:{addr[1]}")
                        last_client_seen[addr] = now

                        # Forward to Pixhawk via TCP socket
                        if tcp_sock:
                            try:
                                tcp_sock.sendall(data)
                            except Exception as e:
                                print(f"[TCP Send Error] {e}")
                                tcp_sock.close()
                                tcp_sock = None

                    except BlockingIOError:
                        pass
                    except Exception as e:
                        print(f"[UDP Recv Error] {e}")

                elif s is tcp_sock:
                    # Inbound Pixhawk packet from mavlink-routerd TCP socket
                    try:
                        data = tcp_sock.recv(4096)
                        if not data:
                            print("[TCP Notice] Server closed socket. Reconnecting...")
                            tcp_sock.close()
                            tcp_sock = None
                        else:
                            # Forward Pixhawk downlink packet to all registered GCS clients
                            for client_addr in list(clients):
                                try:
                                    udp_sock.sendto(data, client_addr)
                                    tx_bytes_udp += len(data)
                                except Exception as e:
                                    print(f"[UDP Send Error to {client_addr}] {e}")
                    except BlockingIOError:
                        pass
                    except Exception as e:
                        print(f"[TCP Recv Error] {e}")
                        tcp_sock.close()
                        tcp_sock = None

    except KeyboardInterrupt:
        print("\n[STOPPING] Interrupted by user.")
    finally:
        if udp_sock:
            udp_sock.close()
        if tcp_sock:
            tcp_sock.close()
        print("[SHUTDOWN] Bridge exited cleanly.")
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="High-Speed UDP 14550 <-> TCP 5760 MAVLink Relay")
    parser.add_argument("--udp-port", type=int, default=14550, help="UDP listening port (default: 14550)")
    parser.add_argument("--tcp-host", type=str, default="127.0.0.1", help="Target TCP host (default: 127.0.0.1)")
    parser.add_argument("--tcp-port", type=int, default=5760, help="Target TCP port (default: 5760)")
    args = parser.parse_args()

    sys.exit(run_bridge(args.udp_port, args.tcp_host, args.tcp_port))
