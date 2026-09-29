#!/usr/bin/env python3
"""
================================================================================
MODULE: c2_validate.py
PURPOSE: Pre-Flight Ground Validation of GCS -> FC Command & Control (C2) Uplink and ACKs
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station or Radxa Companion Computer
  * Communicates:  Pixhawk 6X / ArduPilot via mavlink-router, ELRS Backpack, or Serial
  * Upstream:      Operator CLI test harness
  * Downstream:    Autopilot Command Receiver & State Estimator

DATA FLOW & INTERFACES:
  * MAVLink Out:   MAV_CMD_DO_SET_MODE (mode shifts), MAV_CMD_NAV_TAKEOFF (probe),
                   MAV_CMD_COMPONENT_ARM_DISARM (optional with --arm flag).
  * MAVLink In:    COMMAND_ACK (msg ID 77) with matching command ID and MAV_RESULT.
  * Validation:    Pass criterion is responsive acknowledgement (ACCEPTED, DENIED,
                   or TEMPORARILY_REJECTED). Network silence or packet drop is failure.

KEY LOGIC & FAILSAFES:
  * Motor Safety: Arming is strictly opt-in via `--arm` flag; default mode runs
    100% bench-safe commands (mode changes and takeoff probe that triggers safe rejection).
  * Transaction Matching: Matches incoming COMMAND_ACK packets to the specific
    in-flight command ID and increments sequence confirmation counters on retries.
  * MAV_RESULT Diagnostics: Decodes exact enum responses:
      - 0: MAV_RESULT_ACCEPTED
      - 1: MAV_RESULT_TEMPORARILY_REJECTED (Preflight check blocked)
      - 2: MAV_RESULT_DENIED (Safety state or mode blocked)
      - 3: MAV_RESULT_UNSUPPORTED
      - 4: MAV_RESULT_FAILED

RUN:
  # Safe non-arming C2 validation over mavlink-router TCP endpoint:
  python3 c2_validate.py --rx tcp:172.16.101.84:5760

  # Asymmetric ELRS TX Backpack Wi-Fi link:
  python3 c2_validate.py --rx udpin:0.0.0.0:14550 --send 10.0.0.1:14555

  # Complete arm/disarm validation (BENCH ONLY - PROPS REMOVED):
  python3 c2_validate.py --rx tcp:172.16.101.84:5760 --arm
================================================================================
"""
from __future__ import annotations

import argparse
import sys
import time

from pymavlink import mavutil

RESULT = mavutil.mavlink.enums["MAV_RESULT"]


class Link:
    """GCS link with the ELRS backpack's asymmetric UDP ports."""

    def __init__(self, rx_port, send, announce=None, baud=460800):
        kw = {"source_system": 255}
        if not rx_port.startswith(("udp", "tcp")):
            kw["baud"] = baud
        self.rx = mavutil.mavlink_connection(rx_port, **kw)
        if send:
            self.tx = mavutil.mavlink_connection(f"udpout:{send}",
                                                 source_system=255)
        else:
            self.tx = self.rx
            if announce and rx_port.startswith("udpin"):
                self.rx.destination_addr = (announce,
                                            int(rx_port.rsplit(":", 1)[1]))
        self.sysid = 1
        self.compid = 1
        self.mode = ""
        self.armed = False

    def beat(self):
        try:
            self.tx.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        except Exception:
            pass

    def pump(self, secs=0.0):
        """Drain inbound messages, tracking mode/arm state."""
        end = time.time() + secs
        while True:
            m = self.rx.recv_match(blocking=False)
            if m is None:
                if time.time() >= end:
                    return
                time.sleep(0.02)
                continue
            if m.get_type() == "HEARTBEAT" and \
                    m.type != mavutil.mavlink.MAV_TYPE_GCS:
                self.sysid = m.get_srcSystem()
                self.compid = m.get_srcComponent()
                self.armed = bool(m.base_mode &
                                  mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = self.rx.flightmode or ""

    def wait_vehicle(self, timeout=20):
        end = time.time() + timeout
        while time.time() < end:
            self.beat()
            m = self.rx.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if m and m.type != mavutil.mavlink.MAV_TYPE_GCS:
                self.sysid = m.get_srcSystem()
                self.compid = m.get_srcComponent()
                self.armed = bool(m.base_mode &
                                  mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = self.rx.flightmode or ""
                ap = mavutil.mavlink.enums["MAV_AUTOPILOT"].get(m.autopilot)
                return ap.name if ap else str(m.autopilot)
        return None

    # ── the pattern walle_gcs needs ────────────────────────────────
    def command_ack(self, command, *params, timeout=3.0, retries=3):
        """Send COMMAND_LONG, wait for the ack that matches `command`.

        Returns (MAV_RESULT or None, attempts). Retries with an
        incrementing confirmation field, and ignores acks belonging to
        other commands rather than mistaking them for ours.
        """
        p = list(params) + [0.0] * (7 - len(params))
        for attempt in range(retries):
            try:
                self.tx.mav.command_long_send(
                    self.sysid, self.compid, command,
                    attempt,          # confirmation increments per retry
                    *p[:7])
            except Exception as e:
                print(f"      send failed: {e}")
                return None, attempt + 1
            end = time.time() + timeout
            while time.time() < end:
                m = self.rx.recv_match(type="COMMAND_ACK", blocking=True,
                                       timeout=0.5)
                if m is None:
                    continue
                if m.command == command:          # <-- the matching that matters
                    return m.result, attempt + 1
                # ack for something else: keep waiting for ours
        return None, retries


def name(result):
    if result is None:
        return "NO ACK"
    e = RESULT.get(result)
    return e.name if e else f"result={result}"


def check(label, result, attempts, expect_ack=True):
    ok = (result is not None) if expect_ack else True
    mark = "PASS" if ok else "FAIL"
    tries = "" if attempts == 1 else f"  ({attempts} attempts)"
    print(f"  [{mark}] {label:<34} {name(result)}{tries}")
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=__doc__)
    ap.add_argument("--port", default="udpin:0.0.0.0:14550")
    ap.add_argument("--send", default=None,
                    help="TX target, e.g. 10.0.0.1:14555 (ELRS backpack)")
    ap.add_argument("--announce", default=None)
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--arm", action="store_true",
                    help="also test arm/disarm -- MOTORS WILL SPIN, PROPS OFF")
    args = ap.parse_args()

    print("=" * 62)
    print(" GROUND C2 VALIDATION -- PROPS MUST BE OFF")
    print("=" * 62)

    link = Link(args.port, args.send, args.announce, args.baud)
    print(f"\n[link] RX {args.port}" +
          (f"   TX {args.send}" if args.send else ""))
    ap_name = link.wait_vehicle()
    if ap_name is None:
        print("\nNo vehicle heartbeat -- run duplex_check.py first.")
        return 1
    print(f"[link] {ap_name}  sys={link.sysid}  mode={link.mode}  "
          f"armed={link.armed}")

    if link.armed:
        print("\nVehicle is ARMED. Disarm before running this. Aborting.")
        return 1

    results = []

    # ── 1. mode changes (no motion while disarmed) ─────────────────
    print("\n--- 1. mode changes ---")
    mapping = link.rx.mode_mapping() or {}
    if not mapping:
        print("  mode_mapping() empty -- wrong autopilot type?")
    for mode in ("STABILIZE", "ALT_HOLD", "LOITER", "GUIDED", "RTL",
                 "STABILIZE"):
        if mode not in mapping:
            print(f"  [SKIP] {mode:<34} not supported")
            continue
        entry = mapping[mode]
        if isinstance(entry, tuple):
            # PX4's mode_mapping() returns (base_mode, custom_mode, custom_sub_mode) -
            # all three are MAV_CMD_DO_SET_MODE's param1/2/3 verbatim (see pymavlink's
            # mavutil.set_mode_px4). The float(entry) path below is ArduPilot-only,
            # where mode_mapping() returns a single custom_mode int instead of a tuple.
            r, n = link.command_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE, *entry)
        else:
            r, n = link.command_ack(
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                float(entry))
        ok = check(f"set mode {mode}", r, n)
        results.append(ok)
        if r == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            link.pump(1.0)
            got = link.mode.upper()
            if got != mode:
                print(f"         note: ACCEPTED but mode reads {got!r}")
        link.beat()

    # ── 2. a command that SHOULD be refused ────────────────────────
    # Proves the reject path: the FC must answer, not ignore. This USED to send a
    # real MAV_CMD_NAV_TAKEOFF here, on the theory that a takeoff-while-disarmed
    # request is a safe thing to have refused. On 2026-09-05 that theory turned out
    # to be wrong on PX4 (unlike ArduPilot): PX4_CMD_NAV_TAKEOFF is a combined
    # arm+takeoff convenience command, and it actually armed the FC and began a
    # takeoff attempt - caught only because nothing was physically wired to the
    # airframe yet. MAV_CMD_USER_1 is reserved by the MAVLink spec with no defined
    # autopilot behavior, so PX4 answers MAV_RESULT_UNSUPPORTED - proving the same
    # "FC must answer a refusal, not go silent" property with zero possibility of
    # any flight-relevant side effect.
    print("\n--- 2. refusal path (undefined command MAV_CMD_USER_1) ---")
    r, n = link.command_ack(mavutil.mavlink.MAV_CMD_USER_1,
                            0, 0, 0, 0, 0, 0, 0)
    results.append(check("undefined command (expect UNSUPPORTED)", r, n))
    if r == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("         WARNING: MAV_CMD_USER_1 ACCEPTED -- this should never "
              "have any defined behavior; investigate before trusting this link")

    # ── 3. arm / disarm (opt-in) ───────────────────────────────────
    print("\n--- 3. arm / disarm ---")
    if not args.arm:
        print("  [SKIP] not requested (pass --arm; PROPS OFF)")
    else:
        print("  PROPS OFF? motors will spin. starting in 5s, Ctrl-C to abort")
        try:
            time.sleep(5)
        except KeyboardInterrupt:
            print("\n  aborted")
            return 1

        r, n = link.command_ack(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        results.append(check("ARM", r, n))
        link.pump(2.0)
        print(f"         armed flag now: {link.armed}")

        if link.armed:
            r, n = link.command_ack(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
            results.append(check("DISARM", r, n))
            link.pump(2.0)
            print(f"         armed flag now: {link.armed}")
            if link.armed:
                print("         still armed -- sending force disarm")
                r, n = link.command_ack(
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196)
                results.append(check("FORCE DISARM", r, n))
                link.pump(2.0)
                print(f"         armed flag now: {link.armed}")

    # ── summary ────────────────────────────────────────────────────
    print("\n" + "=" * 62)
    passed, total = sum(results), len(results)
    print(f" {passed}/{total} commands acknowledged")
    if passed == total:
        print(" C2 VALIDATED -- every command produced a matching ack.")
        print(" (DENIED/FAILED are passes: the FC answered. Silence is not.)")
    else:
        print(" INCOMPLETE -- some commands got no ack. The link delivered")
        print(" them or not; without an ack you cannot tell. Check")
        print(" packets_up at the backpack to see if they left the PC:")
        print("   curl -s --compressed http://10.0.0.1/mavlink")
    print("=" * 62)
    return 0 if passed == total else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ninterrupted")
        sys.exit(130)
