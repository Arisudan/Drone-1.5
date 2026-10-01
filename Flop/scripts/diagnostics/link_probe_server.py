#!/usr/bin/env python3
"""
================================================================================
SCRIPT: link_probe_server.py
PURPOSE: Radxa-side UDP echo helper for link_range_test.py (packet-loss probe)
================================================================================

Echoes every probe datagram straight back, stamped with how many datagrams it has
received from that client so far. That one number is what lets the laptop tell
UPLINK loss (never reached the Radxa) from DOWNLINK loss (the echo never came
back). It sends nothing on its own, touches no MAVLink or camera port, writes
nothing but stdout, and exits by itself after --idle-exit seconds without traffic.

  python3 link_probe_server.py --port 9099 --idle-exit 30

(link_range_test.py --start-helper copies it to /tmp on the Radxa and runs it.)
Standard library only; it duplicates the 4-line packet format on purpose so this
file can be copied alone.
================================================================================
"""

import argparse
import socket
import struct
import sys
import time
from typing import Dict, Optional, Tuple

MAGIC = b"LRT1"
HDR = struct.Struct("!4sIdI")      # magic, seq, client send time, server count
SESSION_IDLE_S = 10.0              # a client silent this long starts a fresh count


def handle_datagram(data: bytes, addr, sessions: Dict[object, Tuple[int, float]],
                    now: float) -> Optional[bytes]:
    """The reply for one datagram, or None if it is not a probe packet."""
    if len(data) < HDR.size:
        return None
    magic, seq, t_send, _ = HDR.unpack_from(data)
    if magic != MAGIC:
        return None
    count, last = sessions.get(addr, (0, now))
    if now - last > SESSION_IDLE_S:
        count = 0
    count += 1
    sessions[addr] = (count, now)
    return HDR.pack(MAGIC, seq, t_send, count & 0xFFFFFFFF) + data[HDR.size:]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="UDP echo helper for link_range_test.py")
    ap.add_argument("--port", type=int, default=9099)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--idle-exit", type=float, default=30.0,
                    help="exit after this many seconds without a packet (0 = never)")
    args = ap.parse_args(argv)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind, args.port))
    sock.settimeout(1.0)
    sessions: Dict[object, Tuple[int, float]] = {}
    started = last_rx = last_print = time.monotonic()
    total = 0
    print(f"probe server listening on {args.bind}:{args.port}", flush=True)
    while True:
        now = time.monotonic()
        if args.idle_exit and now - last_rx > args.idle_exit:
            print(f"idle for {args.idle_exit:g} s - exiting ({total} packets echoed)", flush=True)
            return 0
        try:
            data, addr = sock.recvfrom(4096)
        except socket.timeout:
            continue
        except KeyboardInterrupt:
            return 0
        reply = handle_datagram(data, addr, sessions, time.monotonic())
        if reply is None:
            continue
        sock.sendto(reply, addr)
        total += 1
        last_rx = time.monotonic()
        if last_rx - last_print > 5:
            print(f"{total} echoed, {last_rx - started:.0f} s", flush=True)
            last_print = last_rx


if __name__ == "__main__":
    sys.exit(main())
