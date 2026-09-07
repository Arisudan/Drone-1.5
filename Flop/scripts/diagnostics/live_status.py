#!/usr/bin/env python3
"""Live, read-only FC status watcher - run this in your own terminal and leave it
running any time a command-sending script (c2_validate.py, px4_vision_bridge.py,
future flight-execution code) is also talking to the FC.

Never sends an arm, disarm, mode-change, or flight command of any kind. The only
thing it transmits is one MAV_CMD_SET_MESSAGE_INTERVAL request at startup, asking
PX4 to stream SERVO_OUTPUT_RAW (actual commanded PWM per channel) - that message
isn't in PX4's default stream set, which is why an earlier check for it came back
empty and inconclusive. Everything else here is passive listening.

Why this exists: on 2026-09-05, a disarmed-state MAV_CMD_NAV_TAKEOFF test armed the
FC for real (PX4 treats NAV_TAKEOFF as a combined arm+takeoff command, unlike
ArduPilot) and nobody was watching a live view when it happened - it was only caught
after the fact, from a single before/after heartbeat diff. This script exists so that
never has to happen again: always have a live, independent view running.

Run (through mavlink-router, not the raw serial device - it's already in use):
    python3 live_status.py
    python3 live_status.py --port tcp:127.0.0.1:5760   # default, shown explicitly

Ctrl+C to stop.
"""
import argparse
import sys
import time

from pymavlink import mavutil

# This is a live-monitoring tool - output must never sit in a buffer waiting to be
# seen. Line-buffer stdout even when not attached to a TTY (piped, redirected, or
# killed by SIGTERM rather than Ctrl-C) instead of relying on every print() call
# remembering flush=True.
sys.stdout.reconfigure(line_buffering=True)

RESULT = mavutil.mavlink.enums["MAV_RESULT"]


def result_name(result):
    if result is None:
        return "-"
    e = RESULT.get(result)
    return e.name if e else f"result={result}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="tcp:127.0.0.1:5760",
                     help="mavlink-router endpoint, not the raw serial device")
    ap.add_argument("--period", type=float, default=1.0, help="print interval (s)")
    args = ap.parse_args()

    print(f"connecting to {args.port} ...")
    try:
        m = mavutil.mavlink_connection(args.port, source_system=251)
    except Exception as e:
        print(f"  open failed: {e}")
        return 1

    # The very first bytes read off a freshly-opened TCP stream aren't guaranteed to
    # be aligned to a MAVLink frame boundary - the parser can briefly misread a
    # partial/garbled frame as a message before it resyncs (seen for real on
    # 2026-09-05: sys=0, mode=UNKNOWN, armed=True, for exactly one sample, then never
    # again). Don't trust m.wait_heartbeat()'s first result at all for display; only
    # trust two CONSECUTIVE heartbeats that (a) come from a sane, nonzero system id
    # and (b) agree on that system/component pair.
    if m.wait_heartbeat(timeout=10) is None:
        print("NO HEARTBEAT. Is mavlink-router running? (systemctl status mavlink-router)",
              flush=True)
        return 1

    hb = None
    prev = None
    deadline = time.time() + 10
    while time.time() < deadline:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is None or msg.get_srcSystem() == 0:
            continue  # invalid system id - discard outright, don't even count it
        if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
            continue  # mavlink-router mirrors QGroundControl's own heartbeat too -
                       # that's not the vehicle, never compare against it
        if prev is not None and (msg.get_srcSystem(), msg.get_srcComponent()) == \
                (prev.get_srcSystem(), prev.get_srcComponent()):
            hb = msg
            break
        prev = msg
    if hb is None:
        print("Never saw two consistent heartbeats from a real system id - "
              "aborting rather than trust a possibly-bad read.", flush=True)
        return 1

    tsys, tcomp = hb.get_srcSystem(), hb.get_srcComponent()
    print(f"heartbeat OK (validated): sys={tsys} comp={tcomp}\n", flush=True)

    # Ask PX4 to stream SERVO_OUTPUT_RAW - read-only telemetry request, no flight
    # behavior. message_id, interval_us, all else 0/unused for SET_MESSAGE_INTERVAL.
    m.mav.command_long_send(
        tsys, tcomp, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
        500000,  # 500ms = 2Hz
        0, 0, 0, 0, 0)

    last_ack = None
    last_servo = None
    armed_prev = None

    print(f"{'t':>6}  {'ARMED':<7} {'MODE':<16} {'SERVO1-4 (us)':<28} LAST COMMAND_ACK", flush=True)
    print("-" * 90, flush=True)

    start = time.time()
    last_print = 0.0
    try:
        while True:
            msg = m.recv_match(blocking=True, timeout=0.5)
            if msg is not None:
                t = msg.get_type()
                # Same validation as at startup, every message, every time - a
                # mid-stream resync glitch could in principle recur, not just at
                # connection time. Never let an invalid-sysid message update state.
                if msg.get_srcSystem() == 0:
                    pass
                elif t == "HEARTBEAT" and msg.get_srcSystem() == tsys \
                        and msg.get_srcComponent() == tcomp:
                    hb = msg
                elif t == "COMMAND_ACK" and msg.get_srcSystem() == tsys:
                    last_ack = (msg.command, msg.result)
                elif t == "SERVO_OUTPUT_RAW" and msg.get_srcSystem() == tsys:
                    last_servo = (msg.servo1_raw, msg.servo2_raw,
                                  msg.servo3_raw, msg.servo4_raw)

            now = time.time()
            if now - last_print < args.period:
                continue
            last_print = now

            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            # Decode mode from this SAME validated heartbeat object, not pymavlink's
            # own m.flightmode - that updates from ANY heartbeat pymavlink sees
            # internally during recv_match, bypassing the validation above entirely,
            # which is exactly what caused single-sample mode flickers seen earlier.
            mode = mavutil.mode_string_v10(hb) or "?"
            servo_s = str(last_servo) if last_servo else "(no data yet)"
            ack_s = f"cmd={last_ack[0]} {result_name(last_ack[1])}" if last_ack else "-"

            flag = "  <-- CHANGED" if armed_prev is not None and armed != armed_prev else ""
            armed_prev = armed

            print(f"{now - start:6.0f}  {'ARMED' if armed else 'disarmed':<7} "
                  f"{mode:<16} {servo_s:<28} {ack_s}{flag}", flush=True)
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        m.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
