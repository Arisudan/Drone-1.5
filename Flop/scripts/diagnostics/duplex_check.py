#!/usr/bin/env python3
"""Verify the GCS <-> FC MAVLink path in BOTH directions, separately.

Downlink and uplink are proven by different evidence, so they get separate
verdicts. Nothing here commands the vehicle -- the uplink probe only asks
the FC to identify itself.

RadioMaster Nomad / ExpressLRS TX Backpack
------------------------------------------
The backpack relays MAVLink over WiFi on ASYMMETRIC UDP ports:

    GCS sends    -> 10.0.0.1:14555   (backpack "listen")
    GCS receives <- 0.0.0.0:14550    (backpack "send")

Sending to 14550 silently goes nowhere. Query the live config with:

    curl -s --compressed http://10.0.0.1/mavlink

which also exposes packets_down / packets_up counters -- useful to prove
the uplink relay even when the FC itself will not answer.

The backpack will not stream until the GCS announces itself, so the
heartbeats this tool sends are what start telemetry flowing.

Usage
-----
    # Nomad backpack over WiFi (asymmetric ports)
    python3 duplex_check.py --port 'udpin:0.0.0.0:14550' --send 10.0.0.1:14555

    # companion SBC / SITL pushing to us (symmetric)
    python3 duplex_check.py --port 'udpin:0.0.0.0:14550' --announce 10.0.0.1

    # wired handset USB-VCP (needs EdgeTX 2.10+ with a MAVLink VCP option)
    python3 duplex_check.py --port /dev/ttyACM0 --baud 460800

Expected verdicts
-----------------
    INAV       DOWNLINK PROVEN / UPLINK UNPROVEN
               (INAV's MAVLink is telemetry-only and never replies)
    ArduPilot  DOWNLINK PROVEN / UPLINK PROVEN
               (a COMMAND_ACK comes back -- this is the acceptance test)
"""
from __future__ import annotations

import argparse
import collections
import time

from pymavlink import mavutil

AP_VERSION_MSGID = 148  # AUTOPILOT_VERSION

# Messages that matter for odometry visualisation.
ODOM = {
    "LOCAL_POSITION_NED": "position+velocity (the odometry)",
    "ATTITUDE": "orientation",
    "GLOBAL_POSITION_INT": "global fix (zeros until GPS/EKF origin)",
    "EKF_STATUS_REPORT": "estimator health",
    "VFR_HUD": "speed/alt",
    "SYS_STATUS": "battery/sensors",
    "HEARTBEAT": "mode + arm state",
}


def open_link(args):
    kw = {"source_system": 255}
    if not args.port.startswith(("udp", "tcp")):
        kw["baud"] = args.baud
    print(f"[link] RX {args.port}")
    m = mavutil.mavlink_connection(args.port, **kw)
    if args.announce and args.port.startswith("udpin"):
        port = int(args.port.rsplit(":", 1)[1])
        m.destination_addr = (args.announce, port)
        print(f"[link] announcing to {args.announce}:{port}")
    return m


def beat(m):
    try:
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        return True
    except Exception as e:
        print(f"[tx  ] heartbeat failed: {e}")
        return False


def main() -> int:
    ap = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=__doc__)
    ap.add_argument("--port", required=True,
                    help="RX endpoint: /dev/ttyACM0 or udpin:0.0.0.0:14550")
    ap.add_argument("--baud", type=int, default=460800,
                    help="serial baud (ignored for UDP/TCP)")
    ap.add_argument("--announce", default=None,
                    help="IP to heartbeat at so a silent relay starts streaming")
    ap.add_argument("--send", default=None,
                    help="TX target for asymmetric links, e.g. 10.0.0.1:14555 "
                         "(ELRS backpack listens there, replies on 14550)")
    ap.add_argument("--secs", type=float, default=10.0,
                    help="rate sample window (default 10s)")
    args = ap.parse_args()

    m = open_link(args)
    tx = m
    if args.send:
        tx = mavutil.mavlink_connection(f"udpout:{args.send}", source_system=255)
        print(f"[link] TX {args.send} (asymmetric)")

    # ── 1. DOWNLINK ────────────────────────────────────────────────
    print("\n=== 1. DOWNLINK  (FC -> GCS) ===")
    print("[rx  ] waiting up to 15s for a vehicle HEARTBEAT ...")
    hb, deadline = None, time.time() + 15
    while time.time() < deadline:
        beat(tx)
        cand = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if cand and cand.type != mavutil.mavlink.MAV_TYPE_GCS:
            hb = cand
            break

    if hb is None:
        print("\nDOWNLINK  : FAILED - no vehicle heartbeat.")
        print("  Check, in order:")
        print("   1. FC powered and RX has link (handset not telemetry-lost)")
        print("   2. FC UART = MAVLink2 at the same baud as ELRS")
        print("      ArduPilot: SERIALn_PROTOCOL=2, SERIALn_BAUD=460")
        print("   3. ELRS TX+RX both 3.5.x+ with MAVLink enabled")
        print("   4. Backpack WiFi joined - NOT TX-module WiFi (that kills RF)")
        print("   5. curl -s --compressed http://10.0.0.1/mavlink")
        print("      packets_down climbing? then the FC side is fine and the")
        print("      problem is between backpack and this PC (ports!).")
        return 1

    sysid, compid = hb.get_srcSystem(), hb.get_srcComponent()
    apname = mavutil.mavlink.enums["MAV_AUTOPILOT"].get(hb.autopilot)
    apname = apname.name if apname else str(hb.autopilot)
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    print(f"[rx  ] heartbeat : sys={sysid} comp={compid}")
    print(f"[rx  ] autopilot : {apname}")
    print(f"[rx  ] armed     : {armed}")
    print("DOWNLINK  : PROVEN")

    # ── 2. UPLINK ──────────────────────────────────────────────────
    print("\n=== 2. UPLINK  (GCS -> FC) ===")
    print("[tx  ] requesting AUTOPILOT_VERSION (read-only, commands nothing)")

    got = None
    for attempt in range(1, 4):
        try:
            tx.mav.command_long_send(
                sysid, compid,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                attempt - 1,  # confirmation increments per retry
                AP_VERSION_MSGID, 0, 0, 0, 0, 0, 0)
        except Exception as e:
            print(f"[tx  ] send failed: {e}")
            break
        print(f"[tx  ]   attempt {attempt}, waiting 3s ...")
        end = time.time() + 3
        while time.time() < end:
            r = m.recv_match(type=["AUTOPILOT_VERSION", "COMMAND_ACK"],
                             blocking=True, timeout=1)
            if r is not None:
                got = r
                break
        if got:
            break

    if got is None:
        print("\nUPLINK    : UNPROVEN - no reply to a request the FC should answer.")
        print("  On INAV this is EXPECTED (telemetry-only MAVLink).")
        print("  It does NOT mean the RF uplink is broken. To prove the relay")
        print("  independently, watch packets_up rise while this runs:")
        print("    curl -s --compressed http://10.0.0.1/mavlink")
    else:
        if got.get_type() == "COMMAND_ACK":
            res = mavutil.mavlink.enums["MAV_RESULT"].get(got.result)
            print(f"[rx  ] COMMAND_ACK result={res.name if res else got.result}")
        else:
            print("[rx  ] AUTOPILOT_VERSION received")
        print("UPLINK    : PROVEN - the FC received our packet and replied.")

    # ── 3. RATES ───────────────────────────────────────────────────
    print(f"\n=== 3. DOWNLINK RATES  ({args.secs:.0f}s sample) ===")
    counts: collections.Counter = collections.Counter()
    nbytes, t0 = 0, time.time()
    while time.time() - t0 < args.secs:
        msg = m.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        t = msg.get_type()
        counts[t] += 1
        if t != "BAD_DATA":
            nbytes += len(msg.get_msgbuf())
    dt = time.time() - t0

    print(f"{'message':<26}{'Hz':>8}   note")
    print("-" * 68)
    for name, n in counts.most_common():
        print(f"{name:<26}{n / dt:8.1f}   {ODOM.get(name, '')}")
    print("-" * 68)
    print(f"{'TOTAL':<26}{sum(counts.values()) / dt:8.1f}   {nbytes / dt:.0f} B/s")

    if counts.get("BAD_DATA"):
        pct = 100.0 * counts["BAD_DATA"] / max(sum(counts.values()), 1)
        print(f"\n  note: {pct:.1f}% BAD_DATA. A little is normal on RF; sustained")
        print("        double digits means a baud mismatch or a saturated link.")

    missing = [k for k in ("LOCAL_POSITION_NED", "ATTITUDE") if k not in counts]
    if missing:
        print(f"\n  MISSING for odometry: {', '.join(missing)}")
        print("    ArduPilot: raise SRn_POSITION (LOCAL_POSITION_NED) and")
        print("               SRn_EXTRA1 (ATTITUDE) on this link's channel.")

    print("\n=== VERDICT ===")
    print(f"  downlink : PROVEN ({apname})")
    print(f"  uplink   : {'PROVEN' if got else 'UNPROVEN'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
