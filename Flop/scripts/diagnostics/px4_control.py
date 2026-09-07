#!/usr/bin/env python3
"""PX4 sibling of drone_control.py.

Same REPL/Fleet feel, but PX4 semantics instead of ArduPilot:
  * modes set via MAV_CMD_DO_SET_MODE with PX4 (main_mode, sub_mode) encoding
    (PX4 has no GUIDED; there is OFFBOARD/POSCTL/ALTCTL/AUTO.* instead)
  * position / velocity control runs through OFFBOARD mode, which requires a
    CONTINUOUS >=2 Hz setpoint stream. A background thread streams the current
    setpoint at 20 Hz; move/vel/goto/yaw just update it. OFFBOARD is only
    entered AFTER the stream is already running (PX4 rejects it otherwise).
  * indoor / GPS-denied: uses LOCAL_POSITION_NED (not global) as the frame.

Default link is PX4 SITL's offboard UDP port 14540 (same one MAVSDK uses).
"""

import argparse
import math
import os
import sys
import time
import threading

# Without this, mavutil defaults to MAVLink v1, whose 8-bit message-ID field cannot
# even represent OBSTACLE_DISTANCE (id 330) - a v2-only message. Confirmed live: the
# obstacle-avoidance safety layer's own OBSTACLE_DISTANCE feed (obstacle_distance_
# bridge.py) was sending real rings the whole time, but this connection silently
# couldn't parse them and status kept reporting "NO ring received yet" - not a
# feed problem, a protocol-version mismatch on THIS end. VISION_POSITION_ESTIMATE
# (id 102) fits in v1's range, which is why the EV/localization half of the same
# safety layer worked fine even before this fix and masked the other half's bug.
os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil

DEFAULT_PORT = "udpin:0.0.0.0:14540"   # PX4 SITL offboard stream (14550 = QGC)

# PX4 custom flight modes: (main_mode, sub_mode). sub_mode only used for AUTO.
PX4_MODES = {
    "MANUAL":     (1, 0),
    "ALTCTL":     (2, 0),
    "POSCTL":     (3, 0),
    "AUTO.TAKEOFF": (4, 2),
    "AUTO.LOITER":  (4, 3),   # position hold
    "AUTO.MISSION": (4, 4),
    "AUTO.RTL":     (4, 5),
    "AUTO.LAND":    (4, 6),
    "ACRO":       (5, 0),
    "OFFBOARD":   (6, 0),
    "STABILIZED": (7, 0),
}
_PX4_MODE_NAMES = {v: k for k, v in PX4_MODES.items()}


def px4_mode_name(custom_mode):
    """Decode a HEARTBEAT's custom_mode into a PX4_MODES name.

    mavutil.mode_string_v10() can't be used here - it decodes ArduPilot's flat
    flight-mode number, not PX4's packed (main_mode, sub_mode) custom_mode.
    Confirmed live: it printed 'UNKNOWN' for a heartbeat independently verified
    (via raw main_mode/sub_mode bit-unpacking) to be genuine OFFBOARD - PX4 was
    in the mode we asked for the whole time, mode_string_v10() just can't say so.
    """
    main_mode = (custom_mode >> 16) & 0xFF
    sub_mode = (custom_mode >> 24) & 0xFF
    name = _PX4_MODE_NAMES.get((main_mode, sub_mode))
    if name is not None:
        return name
    if main_mode == 4:
        return f"AUTO(sub={sub_mode})"
    return f"UNKNOWN(main={main_mode},sub={sub_mode})"

# SET_POSITION_TARGET_LOCAL_NED type_mask bits are IGNORE flags.
POS = mavutil.mavlink
IGN_POS   = POS.POSITION_TARGET_TYPEMASK_X_IGNORE | POS.POSITION_TARGET_TYPEMASK_Y_IGNORE | POS.POSITION_TARGET_TYPEMASK_Z_IGNORE
IGN_VEL   = POS.POSITION_TARGET_TYPEMASK_VX_IGNORE | POS.POSITION_TARGET_TYPEMASK_VY_IGNORE | POS.POSITION_TARGET_TYPEMASK_VZ_IGNORE
IGN_ACC   = POS.POSITION_TARGET_TYPEMASK_AX_IGNORE | POS.POSITION_TARGET_TYPEMASK_AY_IGNORE | POS.POSITION_TARGET_TYPEMASK_AZ_IGNORE
IGN_YAW   = POS.POSITION_TARGET_TYPEMASK_YAW_IGNORE
IGN_YAWR  = POS.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
IGN_FORCE = POS.POSITION_TARGET_TYPEMASK_FORCE_SET

# position + yaw setpoint: ignore velocity, accel, force, yaw-rate
MASK_POS_YAW = IGN_VEL | IGN_ACC | IGN_FORCE | IGN_YAWR
# velocity + yaw-rate setpoint: ignore position, accel, force, yaw
MASK_VEL_YAWR = IGN_POS | IGN_ACC | IGN_FORCE | IGN_YAW

# ---- obstacle-avoidance safety layer -------------------------------------
# OBSTACLE_DISTANCE (MAVLink 330) is produced by obstacle_distance_px4.py
# (RTAB obstacles -> body-FRD 72-sector ring) and fanned out to this port.
# When something is closer than OBST_TRIGGER_M in the DIRECTION OF TRAVEL, the
# watchdog PREEMPTS the streamed setpoint with a hold-here (the drone stops but
# stays in OFFBOARD) and latches `_blocked`. While blocked, move/goto auto-reject
# any command whose travel cone is not clear to >= OBST_CLEAR_M and prompt for a
# safe one; yaw is always allowed (rotating in place can't collide). The ring is
# body-FRD: sector 0 = forward, +angle clockwise (right), same frame move uses.
DEFAULT_OBSTACLE_PORT = "udpin:0.0.0.0:14541"
# ══ THE AIRFRAME ENVELOPE IS PER VEHICLE, AND IT IS NOT A DETAIL ═════════════
# Every threshold below is derived from two radii, and until 2026-08-27 both were the
# S500's - on every aircraft. Flying a drone2 (a 280 mm quad) with an S500's 620/762 mm
# envelope does not fail safe, it fails STUCK: measured on a nidar_maze2 run, 5 of 5
# consecutive mission legs were refused, each by a few centimetres, in corridors the
# aircraft fitted through with 28-111 mm to spare. The runner then abandoned each mission
# and asked the explorer for a different frontier, forever.
#
#   leg 0.40 m, obstacle 0.451 m from the stopping point -> needed 0.481 (S500) / 0.342
#   leg 0.44 m,                    0.370                 ->        0.481       / 0.342
#   leg 0.60 m,                    0.396                 ->        0.410       / 0.329
#   leg 0.62 m,                    0.453                 ->        0.481       / 0.342
#   leg 1.16 m, obstacle 0.348 m abeam                   ->        0.360       / 0.279
#
# This is the same bug maze_eval.py had (it scored every flight against the S500 and
# printed a spurious CANNOT TURN on a drone2), one layer further in: there it made the
# REPORT wrong, here it makes the aircraft refuse to fly.
#
# $VEHICLE is exported by run_allfather.sh for exactly this. Unknown vehicles keep the
# S500 numbers, which is the conservative direction and preserves the x500's behaviour.
#
#   prop     half the across-the-props width. Below this is a STRIKE.
#   yaw      half the circle the airframe sweeps rotating in place. Below this the
#            vehicle is not striking anything but CANNOT TURN - how a run ends wedged.
#   backoff  centre distance at which the vehicle retreats on its own. Sits BETWEEN the
#            two: the props are still clear, but rotation is already impossible, so
#            nothing else will get it out.
#   trigger  SKIN gap dead ahead that brakes it to a stop (reactive backstop).
#   clear    SKIN gap a new move/goto needs ahead of it to be accepted.
#
# trigger AND clear ARE CORRIDOR-BOUND, which is why they are in this table rather than
# being one number. MEASURED off nidar_maze2's own geometry (maze_geometry.load, 1397
# corridor-centre ridge points): median half-width 0.492 m, i.e. 0.985 m corridors. At a
# corridor centre the SKIN clearance is therefore
#     s500    0.492 - 0.381 = 0.111 m
#     drone2  0.492 - 0.242 = 0.250 m
# and a vehicle must be able to SIT there - that is where a junction turn happens. A
# trigger above that number brakes the vehicle before it can ever reach the middle of a
# corridor, which is precisely what 0.8 m did: it stopped the drone2 0.55 m short of
# every junction face and then held, because "ahead" was never clear again.
# VEHICLE_BASE first, then VEHICLE: run_allfather.sh exports both, and BASE is the one
# that names the GEOMETRY FAMILY - drone2wbat is the drone2 with a discharging battery
# (airframe 4039), the same airframe to the millimetre. The alias below covers the
# hand-run case, where only VEHICLE is set.
# drone4 is the drone2 one CAD revision on (airframe 4042): +8.4 g, taller gear, the L2
# 9 mm higher - and the SAME plan-view envelope, because guard_0 and the rotor poses did
# not move. Its L2 mount differs and run_allfather.sh handles that; nothing here does.
VEHICLE_ALIASES = {"drone2wbat": "drone2", "drone4": "drone2"}
VEHICLE_NAME = os.environ.get("VEHICLE_BASE") or os.environ.get("VEHICLE", "s500")
VEHICLE_NAME = VEHICLE_ALIASES.get(VEHICLE_NAME, VEHICLE_NAME)
VEHICLE_ENVELOPE = {
    #            prop     yaw     backoff  trigger  clear
    "s500":   (0.310,   0.381,   0.36,    0.80,    1.10),
    # drone2: arm 0.1400 (rotor pose) + 0.0889 prop collision radius = 0.2289 disc; yaw
    # circle 0.2420 from the guard_0 collision - both read out of drone2_7in/model.sdf,
    # the same two numbers maze_eval.py scores with. backoff 0.235 sits in the 13 mm
    # window between them. trigger 0.20 is 50 mm inside the 0.250 m a corridor centre
    # offers; clear 0.55 lets it commit to the next cell (a face one 0.985 m cell ahead
    # reads 0.985 - 0.242 = 0.743 m of skin).
    "drone2": (0.2289,  0.2420,  0.235,   0.20,    0.55),
}
_ENV = VEHICLE_ENVELOPE.get(VEHICLE_NAME, VEHICLE_ENVELOPE["s500"])


def _envf(name, default):
    """Env override for a tuning constant, so a corridor can be tried without an edit."""
    try:
        return float(os.environ[name])
    except (KeyError, ValueError):
        return default


# The lidar/camera measures range from the vehicle CENTRE, but the airframe extends
# to the prop tips. Treat clearance as (measured - DRONE_RADIUS_M) so the thresholds
# below are gaps from the drone's SKIN, not its centre.
#
# THE DEFAULT, 0.381 m, IS HALF THE S500's YAW CIRCLE (762 mm) - see the table above for
# what each aircraft actually uses. Raised from 0.35 on
# 2026-08-18 to match costmap_2d.py's inscribed_radius; the two MUST agree, or the
# planner and the reactive layer disagree about where the vehicle fits.
#
# The yaw circle, not the 620 mm prop-disc width (0.310 m), because a maze makes the
# vehicle TURN: a gap it can fly straight through is not necessarily one it can rotate
# in, and getting that wrong is how a run ends wedged in a corridor it cannot reverse
# out of. In the NIDAR arena's 980 mm narrowest passage the turn margin is 109 mm per
# side, so the old 0.35 under-modelled the aircraft by 31 mm - 28 % of that budget.
# (For the x500, whose prop tips are ~0.25 m out, this is simply conservative.)
DRONE_RADIUS_M = _envf('DRONE_RADIUS_M', _ENV[1])
# CORRIDOR-CAPABLE tuning (fly 1.9-2.5 m wide corridors). The key is a NARROW
# forward cone: a corridor's side walls run PARALLEL to travel, and the nearest
# wall point inside a cone of half-angle t sits at range g/sin(t) (g = half the
# corridor width). A wide cone sees those parallel walls up close and refuses to
# move; a narrow cone lets them slide past while a wall DEAD AHEAD (0deg) is still
# caught. At +/-22deg a 1.9 m corridor (g=0.95) shows its walls at 0.95/sin22 =
# 2.54 m (skin gap ~2.2 m) - easily above OBST_CLEAR_M - yet a head-on wall still
# trips OBST_TRIGGER_M. Trade-off: obstacles > ~22deg off the travel line are not
# treated as "ahead" (that's what makes corridor flight possible).
# PROP DISC, not the yaw circle. DRONE_RADIUS_M (0.381) is the radius the aircraft needs
# to ROTATE in place; the props themselves reach 0.310 m. The difference matters for the
# back-off below: between those two radii the vehicle is not striking anything but can no
# longer turn, which is precisely the state it gets wedged in.
PROP_RADIUS_M = _envf('PROP_RADIUS_M', _ENV[0])
# BACK-OFF TRIP: nearest return, ANY bearing, at which the vehicle retreats instead of
# waiting to be told where to go. Expressed as a centre distance, not a skin gap, because
# what matters is which of the two radii above has been crossed.
#
# On the S500 0.36 m sits between the prop disc (0.310) and the yaw circle (0.381): the
# props are still clear, so there is something to save, but rotation is already impossible,
# so the vehicle cannot yaw its way out and nothing else will move it. The drone2's two
# radii are only 13 mm apart, so its 0.235 is the midpoint of that window.
#
# WHY IT MUST NOT BE HIGHER: in nidar_maze2 the centred vehicle is 0.492 m from each wall
# (measured). A trip at 0.492 would fire continuously and the vehicle would back out of
# every corridor it entered. Per vehicle that leaves 0.13 m (s500) / 0.26 m (drone2) of
# drift off centre before this speaks.
BACKOFF_TRIP_M = _envf('BACKOFF_TRIP_M', _ENV[2])
# How far to retreat per attempt, and how many attempts before giving up and saying so.
BACKOFF_STEP_M = 0.35
BACKOFF_TRIES = 4
# How long a retreat waits for external vision before giving up. The failsafe refuses all
# motion while vision is stale, so retreating first requires a pose to retreat FROM.
BACKOFF_EV_WAIT_S = 10.0
# EXTRA CLEARANCE BEYOND THE END OF A MISSION LEG.
#
# WHY A MISSION LEG IS NOT GATED LIKE A `goto`. OBST_CLEAR_M (1.1 m) is the right test for
# move/goto, where the travel distance is OPEN-ENDED - the operator says "go that way" and
# the vehicle needs runway. A mission waypoint is different: the distance is KNOWN
# exactly, so the question is not "is there 1.1 m of runway" but "does this leg fit".
#
# MEASURED 2026-08-20, and this is why the distinction matters: a 0.60 m leg was refused
# because a wall sat 1.25 m ahead (skin gap 0.87 m < 1.1 m). The vehicle would have ended
# 0.65 m from that wall with 0.27 m of skin clearance - entirely safe - and the explorer
# kept re-publishing the same frontier, so the runner burned refusals against a leg it
# should simply have flown. Gating on the leg's own length accepts it and still refuses a
# leg that would actually reach the wall.
#
# 0.10, DOWN FROM 0.25 on 2026-08-20 after measuring what 0.25 cost. That flight was the
# safest and most accurate yet - 0.10-0.68 % wall contact, pose within 4-9 cm of truth,
# map precision 84.7 % - and it flew only 9.64 m of path at 44 % coverage, against 79.26 m
# on the previous run. The gate was refusing legs the aircraft could fly.
#
# The arithmetic says why. nidar_maze3's corridors are 1 m wide and the yaw circle is
# 0.381 m, so the TOTAL slack either side is 0.119 m - and the measured median turn margin
# was +0.012 m, i.e. half the flight had a centimetre to spare. Demanding a further 0.25 m
# of standoff at the end of every leg is disproportionate to a corridor with 0.119 m in
# it; requiring leg + 0.381 + 0.25 = 1.63 m of clearance for a 1 m leg is unsatisfiable
# where walls recur every 1-2 m, so legs were refused, the retreat fired, and the vehicle
# oscillated instead of progressing.
LEG_CLEAR_MARGIN_M = 0.10
# SEPARATE, SMALLER MARGIN FOR THE LATERAL TEST, because it answers a different question.
# Along-track margin buys room to decelerate and turn. Lateral margin only has to cover
# position error on a "will the props strike this wall" test - and 0.10 m on a 0.310 m
# radius is a 32 % inflation of the aircraft, which in a corridor is the difference between
# flyable and not.
#
# MEASURED 2026-08-20, the refusals that stopped a flight at 5/5:
#     obstacle 0.87 m along track, 0.37 m off centre -> blocked (0.37 <= 0.41)
#     obstacle 0.09 m along track, 0.41 m off centre -> blocked (0.41 <= 0.41)
# The second is a corridor wall the vehicle was already flying PAST - 0.09 m along track
# means beside, not ahead - with 0.100 m of genuine prop clearance. Both were refused on a
# knife edge by a threshold that had nothing to do with striking anything, and the retreat
# could not help because retreating does not change a corridor's width.
LEG_LATERAL_MARGIN_M = 0.05
# Below this the waypoint's yaw is the heading the vehicle already holds, so arriving does
# not rotate and the endpoint only needs the strike radius. Matches the spirit of
# global_planner's yaw deadband, which is what makes shared headings common.
LEG_TURN_DEADBAND_DEG = 15.0
OBST_TRIGGER_M = _envf('OBST_TRIGGER_M', _ENV[3])  # brake if the SKIN gap ahead is under this
OBST_CLEAR_M   = _envf('OBST_CLEAR_M', _ENV[4])    # SKIN gap a new move/goto needs ahead
# 15 deg, down from 22 deg on 2026-08-18, to make 1 m corridors flyable. The arithmetic
# above, run at the NIDAR arena's width instead of the 1.9 m this was tuned for:
#     +/-22 deg, g=0.50 -> walls at 0.50/sin22 = 1.34 m -> skin 0.95 m  < OBST_CLEAR_M
#     +/-15 deg, g=0.50 -> walls at 0.50/sin15 = 1.93 m -> skin 1.55 m  > OBST_CLEAR_M
#     +/-15 deg, g=0.49 -> walls at 1.89 m             -> skin 1.51 m  (narrowest passage)
# At 22 deg every `move`/`goto` inside a 1 m corridor was REFUSED - the side walls, which
# the vehicle fits between with 119 mm to spare, read as an obstacle dead ahead. A wall
# actually ahead is at 0 deg and is caught by any cone, so head-on protection is unchanged.
#
# WHAT IT COSTS, stated plainly: the cone only covers the swept path beyond
# r = DRONE_RADIUS_M / sin(t) - 1.47 m at 15 deg, up from 1.02 m at 22 deg. Inside that
# range an obstacle offset laterally by less than 0.381 m (i.e. one that WOULD hit) can
# sit outside the cone: 1.0 m ahead and 0.30 m to the side is 16.7 deg, seen at 22 deg and
# missed at 15 deg. That gap is covered by the other two layers - the costmap inflates
# every mapped obstacle by the same 0.381 m, and PX4 CP holds its own ring - so this is
# the reactive layer trading near-field width for the ability to enter a corridor at all.
OBST_CONE_DEG  = 15.0    # half-angle of the "ahead" cone (narrow -> corridors pass)
OBST_STALE_S   = 0.5     # ring older than this = "blind": don't trip or reject
UINT16_UNKNOWN = 65535
# Trip debounce + smooth braking. A single noisy lidar return that momentarily
# reads close should NOT trip a hold (nuisance stop-go-stop = erratic). Require a
# few consecutive fresh rings below the trigger first. And when a hold DOES trip,
# don't snap a position hold at the instantaneous (still-moving) spot - that makes
# PX4 brake hard and overshoot toward the obstacle. Instead command zero body
# velocity so the drone decelerates in a straight line, wait until it has actually
# settled, THEN latch a position hold where it stopped.
OBST_TRIP_HITS  = 3      # consecutive close rings before a hold trips (debounce)
BRAKE_SETTLE_V  = 0.15   # m/s; treat the drone as stopped below this ground speed
BRAKE_TIMEOUT_S = 2.0    # cap on how long to wait for the brake to settle

# ---- localization-loss failsafe ------------------------------------------
# GPS-denied indoors, EKF2's ONLY absolute position/heading fix is the external
# vision (VISION_POSITION_ESTIMATE) injected by odom_to_px4_vision.py from the SLAM
# /odom. If that stops, EKF2 does not fail loudly - it silently dead-reckons on the
# accelerometers, and position error grows quadratically. The drone keeps "holding"
# a setpoint in a frame that is drifting out from under it, so it physically wanders
# while its own telemetry looks fine. That is the failure this guards.
#
# HOW STALENESS IS DETECTED: the injector fans its VISION_POSITION_ESTIMATE out to
# this same 14541 endpoint. Each arrival is proof that a GOOD SLAM pose was just
# injected - the injector sends it only for frames that pass its lost-frame guard
# (cov[0] > 100 / null quaternion). So "no EV message for EV_STALE_S" covers both
# ways localization dies: /odom stopping entirely, and /odom still publishing but
# flagged lost. PX4 itself streams nothing over MAVLink that reports EV fusion
# health (cs_ev_pos is a uORB flag that never leaves the FMU), so this feed - not
# the autopilot - is the source of truth.
#
# LIMIT, worth being honest about: this detects EV that is ABSENT, not EV that is
# present but WRONG. ICP sliding along a featureless corridor still emits confident
# poses; only the ATE (or a second source) catches that.
# FLOOR for the staleness gate, not the whole story - see EV_STALE_PERIODS.
EV_STALE_S      = 0.6    # no EV for this long (at >=30 Hz) = localization lost
EV_RECOVER_HITS = 5      # consecutive fresh EV frames before the hold releases

# THE GATE HAS TO SCALE WITH THE SOURCE RATE, and 0.6 s alone does not.
# 0.6 s was chosen against a 30 Hz injector, where it means 18 missed messages - an
# unambiguous outage. But the EV rate is set by the SLAM front-end, and those differ by
# more than 5x:
#     camera (rgbd_odometry)    27.9 Hz  -> 0.6 s = 16.7 messages   fine
#     2D scan ring (csm)        ~10 Hz   -> 0.6 s = 6.0  messages   fine
#     3D lidar / Unitree L2      5.55 Hz -> 0.6 s = 3.3  MESSAGES   NOT fine
# The L2 spins at 5.55 Hz because the real part does, so /scan_360, /odom_csm and the EV
# stream downstream of it are all 5.55 Hz - one sample every 180 ms. At a fixed 0.6 s,
# FOUR consecutive dropped scans trip the failsafe, and four scans is one gz render hitch
# or a couple of scan-match rejections. Measured symptom on VEHICLE=s500: "localization
# lost / RECOVERED" cycling continuously with the vehicle sitting still and the stack
# otherwise healthy - the gate was reporting its own calibration, not a fault.
#
# So the limit is max(EV_STALE_S, EV_STALE_PERIODS x the MEASURED heartbeat interval),
# capped so a genuinely dead source is still caught promptly. The interval is measured
# from the heartbeats themselves, so this needs no knowledge of which front-end is
# running and adapts if you switch mid-session.
#
# NOTE THIS RAISES THE GATE, so it can mask a front-end that is genuinely sputtering.
# It does not change what "stale" MEANS - only how long silence must last before a
# 5.55 Hz source is declared dead. If you are chasing dropouts, check the front-end's
# own reject rate too, not just this hold.
EV_STALE_PERIODS = 5.0   # heartbeat intervals of silence before EV counts as lost
EV_STALE_MAX_S   = 2.0   # ...but never wait longer than this, whatever the rate
EV_RATE_WINDOW   = 20    # intervals kept for the median (outlier-robust vs a mean)

# Hardcoded indoor route, replayed by the `pattern` command. Each step is either
# ("move", dx, dy, dz) - a body-frame relative step in metres (dx fwd, dy right,
# dz down) - or ("yaw", deg) - a relative rotation in degrees (+CW / -CCW).
# Steps run in order, each waiting for arrival before the next, so the relative
# composition matches typing them one-by-one in the REPL. Edit freely.
INDOOR_PATTERN = [
    ("move", 2, 0, 0),
    ("yaw", 90),
    ("move", 2, 0, 0),
    ("yaw", 90),
    ("move", 2,0, 0),
    ("yaw", 90),
    ("move", 2, 0, 0),
    ("yaw", 90)
]
 #integrate the obstacle avoidance code into the pattern and goto commands
 #TO make tweaks to the pattern dynamically to not hit the obstacles
class Vehicle:
    """Wraps a MAVLink connection to one PX4 vehicle."""

    def __init__(self, master, sysid, compid, obst_master=None):
        self.master = master
        self.sysid = sysid
        self.compid = compid
        self.telemetry = {
            "mode": None, "armed": None,
            "x": None, "y": None, "z": None,      # LOCAL_POSITION_NED (m, NED)
            "yaw": None,                          # rad, from ATTITUDE
            "vx": None, "vy": None, "vz": None,
            "rel_alt": None, "dist_bottom": None,
            "battery_v": None, "battery_pct": None,
            "last_heartbeat": 0.0,
        }
        self._acks = {}
        self._lock = threading.Lock()      # guards telemetry + acks
        self._txlock = threading.Lock()    # serialises socket writes
        self._stop = threading.Event()

        # offboard setpoint streamed by _stream_loop when _offboard is set
        self._sp = None            # dict: {"type": "pos"|"vel", ...}
        self._offboard = threading.Event()

        # obstacle-avoidance safety layer (active only if an obstacle endpoint
        # was opened). Ring is metres per body-FRD sector; None = unknown/clear.
        self._obst_master = obst_master
        self._obst_ring = None
        self._obst_meta = None     # (increment_deg, angle_offset_deg)
        self._obst_stamp = 0.0
        self._blocked = False      # latched hold engaged by the watchdog
        self._block_reason = ""
        self._trip_hits = 0        # consecutive close rings (debounce, see _safety_check)

        # localization-loss failsafe. _ev_lost is a SEPARATE latch from _blocked:
        # an obstacle hold is cleared by the operator picking a safe direction, but a
        # localization hold must NOT be - no direction is safe when you don't know
        # where you are. Only returning EV clears it (see _ev_check).
        self._ev_stamp = 0.0       # wall time of the last EV heartbeat (0 = none yet)
        self._ev_lost = False
        self._ev_hits = 0          # consecutive fresh EV frames during recovery
        self._ev_seen = False      # has EV ever arrived? (no feed -> never trips)
        # Measured heartbeat intervals, for the rate-aware staleness gate (see
        # EV_STALE_PERIODS). Kept as a plain list under _lock; it is at most
        # EV_RATE_WINDOW long and only touched once per EV message.
        self._ev_iv = []

        # Start the background threads LAST, once every field they touch exists.
        # These three lines were stranded after the `return` in _ev_rate_hz, so they
        # never ran: no reader meant telemetry stayed empty (every command failed with
        # "no LOCAL_POSITION_NED / ATTITUDE yet (EKF not ready?)" no matter how healthy
        # EKF2 actually was), no streamer meant offboard setpoints were never sent, and
        # nothing drained the UDP sockets - measured at 211 KB stuck in the 14540 receive
        # queue, i.e. the kernel buffer full and dropping.
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self._streamer = threading.Thread(target=self._stream_loop, daemon=True)
        self._streamer.start()
        if obst_master is not None:
            self._obst = threading.Thread(target=self._obst_loop, daemon=True)
            self._obst.start()

    def _ev_stale_limit(self):
        """Seconds of silence that count as localization lost, for THIS source.

        max(EV_STALE_S, EV_STALE_PERIODS x median heartbeat interval), capped at
        EV_STALE_MAX_S. Falls back to the fixed floor until enough intervals have been
        seen to have an honest median - so a cold start is never MORE permissive than
        the old behaviour. Caller must hold _lock.
        """
        if len(self._ev_iv) < 5:
            return EV_STALE_S
        s = sorted(self._ev_iv)
        median = s[len(s) // 2]
        return max(EV_STALE_S, min(EV_STALE_MAX_S, EV_STALE_PERIODS * median))

    def _ev_rate_hz(self):
        """Measured EV heartbeat rate, or None until the median is trustworthy.
        Caller must hold _lock."""
        if len(self._ev_iv) < 5:
            return None
        s = sorted(self._ev_iv)
        median = s[len(s) // 2]
        return (1.0 / median) if median > 0 else None

    # ---- background telemetry reader ----------------------------------

    def _read_loop(self):
        while not self._stop.is_set():
            msg = self.master.recv_match(blocking=True, timeout=1.0)
            if msg is None or msg.get_srcSystem() != self.sysid:
                continue
            self._handle(msg)

    def _handle(self, msg):
        t = msg.get_type()
        if t == "STATUSTEXT":
            print(f"\n[FC] {msg.text}")
            return
        with self._lock:
            if t == "COMMAND_ACK":
                self._acks[msg.command] = msg.result
            elif t == "HEARTBEAT":
                self.telemetry["last_heartbeat"] = time.time()
                self.telemetry["armed"] = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.telemetry["mode"] = px4_mode_name(msg.custom_mode)
            elif t == "LOCAL_POSITION_NED":
                self.telemetry["x"] = msg.x
                self.telemetry["y"] = msg.y
                self.telemetry["z"] = msg.z
                self.telemetry["vx"] = msg.vx
                self.telemetry["vy"] = msg.vy
                self.telemetry["vz"] = msg.vz
            elif t == "ATTITUDE":
                self.telemetry["yaw"] = msg.yaw
            elif t == "GLOBAL_POSITION_INT":
                self.telemetry["rel_alt"] = msg.relative_alt / 1000.0
            elif t == "DISTANCE_SENSOR":
                self.telemetry["dist_bottom"] = msg.current_distance / 100.0  # cm -> m
            elif t == "BATTERY_STATUS":
                if msg.voltages and msg.voltages[0] != 0xFFFF:
                    self.telemetry["battery_v"] = msg.voltages[0] / 1000.0
                self.telemetry["battery_pct"] = msg.battery_remaining

    def snapshot(self):
        with self._lock:
            return dict(self.telemetry)

    def close(self):
        self._offboard.clear()
        self._stop.set()

    # ---- offboard setpoint streaming ----------------------------------

    def _stream_loop(self):
        """Stream the current setpoint at 20 Hz whenever offboard is active."""
        while not self._stop.is_set():
            if self._offboard.is_set() and self._sp is not None:
                self._send_setpoint(self._sp)
            time.sleep(0.05)

    def _send_setpoint(self, sp):
        if sp["type"] == "pos":
            mask, frame = MASK_POS_YAW, mavutil.mavlink.MAV_FRAME_LOCAL_NED
            args = (sp["x"], sp["y"], sp["z"], 0, 0, 0, 0, 0, 0, sp["yaw"], 0)
        else:  # velocity (body frame), hold heading (yaw_rate 0)
            mask, frame = MASK_VEL_YAWR, mavutil.mavlink.MAV_FRAME_BODY_NED
            args = (0, 0, 0, sp["vx"], sp["vy"], sp["vz"], 0, 0, 0, 0, sp["yaw_rate"])
        with self._txlock:
            self.master.mav.set_position_target_local_ned_send(
                0, self.sysid, self.compid, frame, mask, *args)

    def _wait_local_position(self, timeout=10.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            s = self.snapshot()
            if s["x"] is not None and s["yaw"] is not None:
                return s
            time.sleep(0.1)
        raise TimeoutError("no LOCAL_POSITION_NED / ATTITUDE yet (EKF not ready?)")

    def start_offboard(self, hold=True):
        """Begin streaming and switch PX4 into OFFBOARD.

        If no setpoint is set yet (hold=True), holds the current position/heading.
        PX4 requires the stream to already be flowing before it will accept OFFBOARD.
        """
        if self._sp is None and hold:
            s = self._wait_local_position()
            self._sp = {"type": "pos", "x": s["x"], "y": s["y"], "z": s["z"],
                        "yaw": s["yaw"]}
        self._offboard.set()
        time.sleep(0.5)                     # let ~10 setpoints go out first
        return self.set_mode("OFFBOARD")

    def stop_offboard(self):
        self._offboard.clear()

    # ---- obstacle-avoidance safety layer ------------------------------

    def _obst_loop(self):
        """Safety-feed reader for the 14541 endpoint. Two independent feeds arrive
        here and each drives its own failsafe:

          OBSTACLE_DISTANCE       (laserscan_to_obstacle.py) -> body-FRD range ring,
                                  trips a hold when the travel direction is blocked.
          VISION_POSITION_ESTIMATE (odom_to_px4_vision.py)   -> EV-alive heartbeat,
                                  trips a hold when localization goes stale.

        Runs only when the endpoint was opened; if nothing arrives, the ring stays
        empty and EV was never seen, so neither failsafe trips (fail-safe: no feed ->
        no nuisance holds). The 0.2 s timeout is what paces _ev_check, so staleness
        is noticed even when the feeds go completely silent."""
        while not self._stop.is_set():
            msg = self._obst_master.recv_match(
                type=["OBSTACLE_DISTANCE", "VISION_POSITION_ESTIMATE"],
                blocking=True, timeout=0.2)
            if msg is None:
                self._ev_check()          # silence is exactly what we're watching for
                continue
            if msg.get_type() == "VISION_POSITION_ESTIMATE":
                with self._lock:
                    now = time.time()
                    # Learn the source's cadence from the source itself. Intervals are
                    # only recorded while EV is HEALTHY: a gap measured across an outage
                    # is the outage, and folding it in would stretch the gate every time
                    # it tripped - the failsafe would get slower the more it was needed.
                    if self._ev_stamp and not self._ev_lost:
                        iv = now - self._ev_stamp
                        if 0.0 < iv < EV_STALE_MAX_S:
                            self._ev_iv.append(iv)
                            if len(self._ev_iv) > EV_RATE_WINDOW:
                                self._ev_iv.pop(0)
                    self._ev_stamp = now
                    self._ev_seen = True
                self._ev_check()
                continue
            inc = getattr(msg, "increment_f", 0.0) or float(msg.increment)
            off = getattr(msg, "angle_offset", 0.0)
            maxd = msg.max_distance                       # cm
            ring = [None if (v == UINT16_UNKNOWN or v == 0 or v > maxd)
                    else v / 100.0                        # cm -> m; else = clear
                    for v in msg.distances]
            with self._lock:
                self._obst_ring = ring
                self._obst_meta = (inc, off)
                self._obst_stamp = time.time()
            self._safety_check()
            self._ev_check()

    def _cone_min(self, bearing_deg, half_deg):
        """Smallest obstacle clearance (m) within +/- half_deg of a body-FRD
        bearing. Returns (min_dist_or_None, n_sectors_seen). None = blind/clear."""
        with self._lock:
            ring, meta = self._obst_ring, self._obst_meta
            fresh = (time.time() - self._obst_stamp) <= OBST_STALE_S
        if not ring or meta is None or not fresh:
            return None, 0
        inc, off = meta
        best, n = None, 0
        for i, d in enumerate(ring):
            if d is None:
                continue
            diff = (off + i * inc - bearing_deg + 180.0) % 360.0 - 180.0
            if abs(diff) <= half_deg:
                n += 1
                if best is None or d < best:
                    best = d
        return best, n

    def _ring_extremes(self):
        """(nearest_dist, nearest_bearing, freest_bearing) over the WHOLE ring.

        WHY THE WHOLE RING AND NOT THE TRAVEL CONE. The cone is deliberately narrow
        (+/-15 deg) so that a corridor's parallel side walls slide past instead of reading
        as an obstacle dead ahead - that is what makes 1 m corridors flyable at all, and
        the trade is written out beside OBST_CONE_DEG: the cone only covers the swept path
        beyond DRONE_RADIUS_M / sin(15 deg) = 1.47 m, so something 1.0 m ahead and 0.30 m
        to the side sits OUTSIDE it while still being on a collision course.

        That blind spot is exactly how the vehicle gets wedged, and it is what the
        2026-08-20 flight measured: first wall contact at t=90 s, then 100 % contact for
        the remaining 6 minutes, motionless from t=300 s with the prop disc 9.2 cm inside a
        wall. A gate that only looks along the travel line cannot see that state, so the
        back-off looks everywhere instead.

        Returns (None, None, None) if the ring is stale or empty - blind is not clear, but
        it is also not a licence to start flying blind manoeuvres, so callers treat it as
        "do nothing".
        """
        with self._lock:
            ring, meta = self._obst_ring, self._obst_meta
            fresh = self._obst_stamp and (time.time() - self._obst_stamp) <= OBST_STALE_S
        if not ring or meta is None or not fresh:
            return None, None, None
        inc, off = meta
        near_d = near_b = None
        far_d = far_b = None
        for i, d in enumerate(ring):
            if d is None:
                continue
            brg = (off + i * inc + 180.0) % 360.0 - 180.0
            if near_d is None or d < near_d:
                near_d, near_b = d, brg
            if far_d is None or d > far_d:
                far_d, far_b = d, brg
        return near_d, near_b, far_b

    def back_off(self, tries=None, step=None):
        """Retreat from a too-close obstacle. -> True if clear (or nothing to do).

        REFUSING TO MOVE IS NOT ENOUGH ONCE YOU ARE ALREADY WEDGED. The travel gate can
        only decline the next command; it has no way to undo the position the vehicle is
        already in, so a vehicle that arrives inside a wall stays there - which is what the
        2026-08-20 flight did for its final 3 minutes while the explorer kept publishing
        goals it could never reach.

        Retreat is along the FREEST bearing in the ring rather than the reciprocal of the
        obstacle: backing straight away from one wall in a corner drives you into the
        other. Steps are short and re-measured each time, because the ring is what tells us
        whether it worked.
        """
        tries = BACKOFF_TRIES if tries is None else tries
        step = BACKOFF_STEP_M if step is None else step
        if self._obst_master is None:
            return True
        for k in range(tries):
            # A RETREAT INTO AN EV HOLD IS NOT A RETREAT. When external vision goes stale
            # the localization failsafe brakes to a stop and refuses every move, so a
            # setpoint written here is accepted by nobody: the vehicle does not budge, the
            # ring does not change, and after `tries` rounds this reports "wedged" when the
            # truth is "blind and therefore holding".
            #
            # MEASURED 2026-08-20 live: nearest return sat at exactly 0.27 m across
            # attempts 2, 3 and 4 - not a millimetre of movement - with
            # "[EV] LOCALIZATION LOST ... 16.9 Hz" interleaved between them. This is the
            # same trap run_mission.py's ev_ok() was written for ("twelve yaw steps ...
            # issued into a vehicle that was holding and could not act on any of them"),
            # and back_off had to learn it too.
            if getattr(self, "_ev_lost", False):
                print(f"[AVOID] too close ({BACKOFF_TRIP_M:.2f} m trip) but LOCALIZATION "
                      f"IS LOST - waiting up to {BACKOFF_EV_WAIT_S:.0f}s for vision. A "
                      f"retreat commanded now would be refused by the failsafe and would "
                      f"look like being wedged.")
                end = time.time() + BACKOFF_EV_WAIT_S
                while time.time() < end and getattr(self, "_ev_lost", False):
                    time.sleep(0.25)
                if getattr(self, "_ev_lost", False):
                    print(f"[AVOID] vision did not return in {BACKOFF_EV_WAIT_S:.0f}s. "
                          f"CANNOT RETREAT - not wedged, BLIND. `land` still works "
                          f"(height comes from the downward lidar).")
                    return False
                print("[AVOID] vision back - retreating now")
            near_d, near_b, free_b = self._ring_extremes()
            if near_d is None:
                return True                      # blind / stale: nothing to act on
            if near_d >= BACKOFF_TRIP_M:
                if k:
                    print(f"[AVOID] backed off to {near_d:.2f} m - clear")
                return True
            strike = near_d < PROP_RADIUS_M
            print(f"[AVOID] TOO CLOSE: nearest return {near_d:.2f} m at "
                  f"{near_b:+.0f} deg ({'PROP DISC IN CONTACT' if strike else 'cannot rotate'}"
                  f"; trip {BACKOFF_TRIP_M:.2f} m). Retreating {step:.2f} m toward "
                  f"{free_b:+.0f} deg [{k + 1}/{tries}]")
            s = self.snapshot()
            if s["x"] is None or s["yaw"] is None:
                print("[AVOID] no position - cannot retreat")
                return False
            # Straight to the setpoint: this deliberately does NOT go through move(),
            # because move() consults the same gate that is currently refusing everything.
            brg = math.radians(free_b)
            dx, dy = step * math.cos(brg), step * math.sin(brg)
            yaw = s["yaw"]
            self._sp = {"type": "pos",
                        "x": s["x"] + dx * math.cos(yaw) - dy * math.sin(yaw),
                        "y": s["y"] + dx * math.sin(yaw) + dy * math.cos(yaw),
                        "z": s["z"], "yaw": yaw}
            if not self._offboard.is_set():
                self.start_offboard(hold=False)
            time.sleep(2.0)                       # let it actually move, then re-measure
        near_d, _, _ = self._ring_extremes()
        print(f"[AVOID] STILL too close after {tries} attempts "
              f"(nearest {near_d if near_d is None else round(near_d, 2)} m). "
              f"The vehicle is wedged - land or take manual control.")
        return False

    def _swept_path_blocker(self, bearing_deg, leg_m, radius_m=None, turns=True):
        """Nearest obstacle that the SWEPT PATH of this leg would actually hit.

        radius_m is accepted and ignored - the two radii that matter are chosen inside
        (see below), because they are not interchangeable and a caller picking one was
        exactly the bug.

        Returns (along_track_m, offset_m) for the worst offender, or None if the corridor
        the vehicle will sweep is clear.

        WHY NOT _cone_min. That returns the nearest range ANYWHERE inside a +/-15 deg
        wedge, and comparing it to the leg length treats it as if it were dead ahead. It
        may not be: an obstacle 1.30 m away at 15 deg off-axis is laterally offset by
        1.30 * sin(15) = 0.336 m, which a 0.310 m prop disc clears completely. Measured
        consequence 2026-08-20 - legs of 0.94/0.97/0.99 m were refused against a "1.30 m"
        obstacle, the vehicle never moved, and coverage came in at 9.64 m of path (44 %)
        against 79.26 m before the gate existed. Some of those refusals were the geometry
        of the test, not the geometry of the maze.

        The right question is the swept corridor: decompose each return into along-track
        (d cos t) and lateral (d sin t) components, and only count it if it is inside the
        vehicle's half-width AND within the distance actually being travelled. That is
        both stricter where it matters (a wall dead ahead at any range inside the leg is
        caught) and looser where it does not (a wall alongside is ignored, which is what
        makes a corridor flyable).
        """
        with self._lock:
            ring, meta = self._obst_ring, self._obst_meta
            fresh = self._obst_stamp and (time.time() - self._obst_stamp) <= OBST_STALE_S
        if not ring or meta is None or not fresh:
            return None
        inc, off = meta
        # TWO DIFFERENT RADII, because the two questions are different manoeuvres.
        #
        # LATERAL - "will I brush past it" - is a TRANSLATION, so the prop disc is the
        # strike radius. ALONG-TRACK REACH - "can I stop there" - has to allow for the
        # vehicle being asked to ROTATE at that waypoint, so it takes the yaw circle.
        #
        # Getting this backwards cost the 2026-08-20 14:57 flight. Using the prop disc for
        # BOTH let the vehicle enter gaps it could not then turn in - CANNOT TURN went
        # 1.8 % -> 31.3 % and wall contact 0.68 % -> 2.96 % with no coverage gain. Using
        # the yaw circle for BOTH is the other error: in a 1 m corridor the side walls sit
        # 0.50 m off the centre line and 0.381 + 0.10 = 0.481 m leaves 19 mm of slack, so
        # a couple of centimetres off centre refuses the corridor outright.
        # THE SWEPT REGION IS A CAPSULE, NOT A RECTANGLE, and using a rectangle is what
        # stopped the 2026-08-20 17:04 flight at 5/5 refusals. Every one of those five
        # obstacles lay BEYOND the end of the leg:
        #
        #   leg 0.53 m, obstacle 0.72 m along / 0.34 m off centre
        #        -> 0.19 m past the endpoint, true closest approach 0.389 m
        #   leg 0.43 m, obstacle 0.80 m along / 0.35 m off centre
        #        -> 0.37 m past the endpoint, true closest approach 0.509 m
        #
        # The rectangle counted them because they fell inside (along < leg + 0.481) and
        # (lateral < 0.36), but the vehicle STOPS at the endpoint - so what matters is the
        # distance from the endpoint, not membership of a box extended past it. Four of
        # those five were 0.50 m clear of a 0.310 m prop disc.
        #
        # Correct decomposition of distance to the swept segment [0, leg] along the travel
        # axis:
        #   along < 0        behind the vehicle; it is moving away, ignore
        #   0 <= along <= leg  the vehicle passes abeam it -> closest approach IS lateral
        #   along > leg      it is ahead of the stopping point -> closest approach is
        #                    measured from the ENDPOINT, hypot(along - leg, lateral)
        lat_limit = PROP_RADIUS_M + LEG_LATERAL_MARGIN_M
        # At the endpoint the vehicle stops, and may be asked to rotate there - but only
        # if this waypoint actually commands a yaw change. `turns` says whether it does;
        # when it does not, a pass-through stop only needs the strike radius.
        end_limit = ((DRONE_RADIUS_M if turns else PROP_RADIUS_M)
                     + LEG_CLEAR_MARGIN_M)
        worst = None
        for i, d in enumerate(ring):
            if d is None:
                continue
            diff = (off + i * inc - bearing_deg + 180.0) % 360.0 - 180.0
            if abs(diff) > 90.0:
                continue                     # behind the direction of travel
            t = math.radians(diff)
            along, lateral = d * math.cos(t), abs(d * math.sin(t))
            if along <= leg_m:
                if lateral < lat_limit:      # brushes past while travelling
                    if worst is None or lateral < worst[1]:
                        worst = (along, lateral, lateral)
            else:
                gap = math.hypot(along - leg_m, lateral)
                if gap < end_limit:          # too close to where it will stop
                    if worst is None or gap < worst[2]:
                        worst = (along, lateral, gap)
        return worst

    def retreat_to(self, north, east, down, yaw_deg=None, tol=0.35, timeout=None):
        """Fly BACK to a pose the vehicle already occupied. -> True if it arrived.

        WHY THIS IS NOT JUST goto(). goto() applies the forward travel gate, and when the
        vehicle is boxed into a dead-end that gate refuses every bearing - including the
        one it arrived on. Measured live 2026-08-20:

            refused mission leg: 0.94 m leg at -2 deg needs 1.57 m ... nearest 1.30 m
            refused mission leg: 0.97 m leg at +2 deg needs 1.60 m ... nearest 1.30 m
            refused mission leg: 0.99 m leg at -1 deg needs 1.62 m ... nearest 1.30 m

        Those refusals are CORRECT - the proposed waypoint would have parked the vehicle
        0.31 m from a wall, exactly the prop-disc radius. But refusing alone never MOVES
        the vehicle, so the explorer re-ranks from the same pose, proposes the same leg,
        and the mission deadlocks. Something has to break the symmetry by changing where
        the vehicle is.

        A pose the vehicle previously OCCUPIED is the safest possible target: it was
        reachable, it was clear enough to hover in, and the path back is the path it just
        flew. That is what justifies bypassing the forward gate here - not convenience.
        Localisation is still required, because a setpoint issued into an EV hold does
        nothing at all (see back_off for what that failure looks like).
        """
        if getattr(self, "_ev_lost", False):
            print(f"[RETREAT] localization lost - waiting up to {BACKOFF_EV_WAIT_S:.0f}s "
                  f"before retreating (a setpoint now would be refused by the failsafe)")
            end = time.time() + BACKOFF_EV_WAIT_S
            while time.time() < end and getattr(self, "_ev_lost", False):
                time.sleep(0.25)
            if getattr(self, "_ev_lost", False):
                print("[RETREAT] no vision - cannot retreat. `land` still works.")
                return False
        s = self._wait_local_position()
        if s is None or s.get("x") is None:
            print("[RETREAT] no position - cannot retreat")
            return False
        d0 = math.hypot(north - s["x"], east - s["y"])
        # SCALE THE TIMEOUT WITH THE DISTANCE. A fixed 25 s is shorter than the trip takes:
        # measured 2026-08-20 17:50, a 2.56 m retreat reported "TIMED OUT (None m)" because
        # this vehicle covers ~0.05-0.08 m/s indoors, i.e. 2.56 m needs ~40 s. Same class
        # of error as the frontier goal_timeout that was cancelling goals before arrival
        # was physically possible.
        if timeout is None:
            timeout = 20.0 + d0 / 0.08
        yaw = s["yaw"] if yaw_deg is None else math.radians(yaw_deg)
        print(f"[RETREAT] going back {d0:.2f} m to a pose already flown: "
              f"N={north:.2f} E={east:.2f} - breaking the refusal deadlock by CHANGING "
              f"WHERE THE VEHICLE IS, so the explorer ranks frontiers from somewhere else")
        self._sp = {"type": "pos", "x": float(north), "y": float(east),
                    "z": float(down), "yaw": yaw}
        if not self._offboard.is_set():
            self.start_offboard(hold=False)
        reached, dist = self._wait_arrival(float(north), float(east), float(down),
                                          tol, timeout)
        print(f"[RETREAT] {'arrived' if reached else 'TIMED OUT'} "
              f"({dist if dist is None else round(dist, 2)} m)")
        return reached

    def _leg_is_safe(self, north, east, final=False, turns=True, retry=False):
        """Gate one mission leg: retreat if wedged, then apply the travel gate.

        WHICH RADIUS APPLIES DEPENDS ON WHAT HAPPENS AT THE WAYPOINT, and conflating the
        two is what made this gate over-conservative. DRONE_RADIUS_M (0.381) is the radius
        the aircraft needs to ROTATE in place; the props themselves reach only
        PROP_RADIUS_M (0.310). An intermediate waypoint is flown THROUGH - nothing rotates
        there - so the strike radius is the right test. The FINAL waypoint is where
        frontier_explorer's raycast-chosen yaw gets applied, so that one does need room to
        turn and keeps the yaw circle.

        For a 1 m leg that is 1.41 m of required clearance instead of 1.63 m, which is the
        difference between a corridor the vehicle can work down and one where every leg is
        refused (measured: 9.64 m of path at 44 % coverage under the old rule, against
        79.26 m before the gate existed at all).

        BACK OFF *BEFORE* GATING, because the two failures need opposite responses. If the
        vehicle is already too close to something, every bearing looks bad and the gate
        would refuse the leg for the rest of the flight - the vehicle would sit there
        being correctly told 'no'. Clearing the immediate obstacle first is what turns a
        permanent stall back into a refusable-and-retryable leg.
        """
        # back_off ONLY on the first evaluation of a leg. The turn-suppression retry in
        # run_mission calls this a second time with turns=False, and without this guard
        # that re-ran the whole retreat - visible in the 2026-08-20 17:50 log as the
        # 4-attempt back-off sequence printing twice in a row, then "wedged" twice.
        near_d, _, _ = self._ring_extremes()
        if not retry and near_d is not None and near_d < BACKOFF_TRIP_M:
            if not self.back_off():
                return False
        s = self._wait_local_position()
        if s is None or s.get("x") is None or s.get("yaw") is None:
            return True                      # no pose: the gate cannot judge, do not block
        dn, de = north - s["x"], east - s["y"]
        leg = math.hypot(dn, de)
        if leg <= 0.15:
            return True                      # already there; nothing to travel through
        yaw = s["yaw"]
        fwd = dn * math.cos(yaw) + de * math.sin(yaw)
        right = -dn * math.sin(yaw) + de * math.cos(yaw)
        brg = math.degrees(math.atan2(right, fwd))

        # Localisation is checked the same way goto/move do - an EV hold cannot be cleared
        # by choosing a different direction, so it must refuse regardless of geometry.
        with self._lock:
            ev_lost = self._ev_lost
        if ev_lost:
            print(f"[EV] refused mission leg: LOCALIZATION LOST (no external vision). "
                  f"Holding until SLAM /odom recovers.")
            return False
        if self._obst_master is None:
            return True

        # DISTANCE-AWARE: the leg must FIT, rather than there being OBST_CLEAR_M of runway.
        # Radius per the docstring: strike radius to pass through, yaw circle to stop and
        # turn at the end.
        hit = self._swept_path_blocker(brg, leg, turns=turns)
        if hit is not None:
            if retry:
                return False                 # caller already reported the first refusal
            along, lateral, gap = hit
            end_limit = (DRONE_RADIUS_M if turns else PROP_RADIUS_M) + LEG_CLEAR_MARGIN_M
            print(f"[AVOID] refused mission leg: {leg:.2f} m leg at {brg:+.0f} deg comes "
                  f"within {gap:.3f} m of an obstacle ({along:.2f} m along track, "
                  f"{lateral:.2f} m off centre) - needs "
                  f"{PROP_RADIUS_M + LEG_LATERAL_MARGIN_M:.3f} m abeam or "
                  f"{end_limit:.3f} m at the stopping point"
                  f"{' (it must rotate there)' if turns else ''}")
            return False
        self._blocked = False            # a safe leg clears the hold, as goto/move do
        self._trip_hits = 0
        return True

    def _travel_bearing(self):
        """Body-FRD bearing (deg) from the current position toward the active
        position setpoint, or None if not currently travelling to one."""
        sp = self._sp
        if sp is None or sp.get("type") != "pos":
            return None
        s = self.snapshot()
        if s["x"] is None or s["yaw"] is None:
            return None
        dn, de = sp["x"] - s["x"], sp["y"] - s["y"]
        if math.hypot(dn, de) < 0.15:            # essentially arrived
            return None
        yaw = s["yaw"]
        fwd = dn * math.cos(yaw) + de * math.sin(yaw)          # world NED -> body
        right = -dn * math.sin(yaw) + de * math.cos(yaw)
        return math.degrees(math.atan2(right, fwd))

    def _safety_check(self):
        """Trip a hold if the current travel direction is blocked closer than
        OBST_TRIGGER_M. Called on every fresh ring."""
        if self._blocked or not self._offboard.is_set():
            return
        bearing = self._travel_bearing()
        if bearing is None:
            self._trip_hits = 0        # not travelling -> nothing to debounce
            return
        d, _ = self._cone_min(bearing, OBST_CONE_DEG)
        if d is not None and (d - DRONE_RADIUS_M) < OBST_TRIGGER_M:
            self._trip_hits += 1
            if self._trip_hits >= OBST_TRIP_HITS:   # sustained, not a noise spike
                self.engage_safety_hold(
                    f"obstacle {d:.2f} m (gap {d - DRONE_RADIUS_M:.2f} m) "
                    f"at {bearing:+.0f} deg (travel dir)")
        else:
            self._trip_hits = 0        # a clear reading resets the debounce

    def _ev_check(self):
        """Trip a position hold when external vision goes stale, release it when
        vision comes back. Called on every safety-feed tick (~5 Hz minimum).

        The threshold is rate-aware (see EV_STALE_PERIODS): silence is judged in units
        of the source's OWN heartbeat interval, so a 5.55 Hz lidar front-end is not
        declared dead after the 3.3 messages that 0.6 s buys it.

        Only arms once EV has been seen at least once: running without the injector
        (bench testing, cheat mode with SIM_GZ_EN_ODOM 1) must not trip a hold."""
        with self._lock:
            if not self._ev_seen:
                return
            age = time.time() - self._ev_stamp
            lost = self._ev_lost
            limit = self._ev_stale_limit()

        if age > limit:
            if not lost:
                with self._lock:
                    self._ev_lost = True
                    self._ev_hits = 0
                # Only meaningful while WE are flying it. In a PX4-owned mode
                # (POSCTL/AUTO.*) the autopilot runs its own EKF failsafes.
                # Report the measured source rate alongside the age. Without it a
                # dropout is unattributable: 0.6 s of silence is an outage on a 30 Hz
                # camera and three missed scans on a 5.55 Hz lidar, and those want
                # completely different fixes.
                with self._lock:
                    hz = self._ev_rate_hz()
                rate = f", source ~{hz:.1f} Hz" if hz else ""
                if self._offboard.is_set():
                    self.engage_safety_hold(
                        f"LOCALIZATION LOST - no external vision for {age:.1f}s "
                        f"(limit {limit:.1f}s{rate}; SLAM /odom stopped or flagged lost)",
                        tag="EV")
                else:
                    print(f"\n[EV] localization lost ({age:.1f}s > {limit:.1f}s{rate}, "
                          f"no vision) - not in OFFBOARD, PX4 owns the failsafe")
            return

        # fresh EV. Debounce the release so a single frame arriving mid-outage
        # doesn't unlatch and let a queued move run into a still-broken estimate.
        if lost:
            with self._lock:
                self._ev_hits += 1
                recovered = self._ev_hits >= EV_RECOVER_HITS
                if recovered:
                    self._ev_lost = False
            if recovered:
                print(f"\n[EV] localization RECOVERED - vision streaming again. "
                      f"Still holding; enter a move/goto to resume.")

    def ev_summary(self):
        """One-line localization health for `status`."""
        if self._obst_master is None:
            return "OFF (no safety endpoint; --no-avoid or feed not opened)"
        with self._lock:
            seen, lost, stamp = self._ev_seen, self._ev_lost, self._ev_stamp
        if not seen:
            return ("ON but NO vision received yet - is odom_to_px4_vision.py running "
                    "(run_phase1.sh) and fanning out to 14541?")
        age = time.time() - stamp
        if lost:
            return f"*** LOST *** no vision for {age:.1f}s - HOLDING, do not navigate"
        return f"ON, vision {age*1000:.0f}ms old"

    def engage_safety_hold(self, reason, tag="AVOID"):
        """PREEMPT the streamed setpoint and latch _blocked, braking SMOOTHLY.

        The mid-journey override: overwriting _sp makes the next 20 Hz stream tick
        send the brake instead of the interrupted move. We do NOT snap a position
        hold at the instantaneous spot - the drone is still moving, so PX4 would
        brake hard and overshoot toward the obstacle, oscillating (erratic). Instead
        command zero body velocity (decelerate in a straight line, heading held),
        wait until the drone has actually settled, THEN latch a position hold where
        it stopped - a smooth, predictable stop."""
        self._blocked = True                 # latch first: stop _safety_check re-tripping
        self._block_reason = reason
        self._trip_hits = 0
        print(f"\n[{tag}] {reason} -> braking to a stop ...")
        # 1) zero-velocity brake (body frame, heading held): straight-line deceleration
        self._sp = {"type": "vel", "vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw_rate": 0.0}
        deadline = time.time() + BRAKE_TIMEOUT_S
        while time.time() < deadline:
            s = self.snapshot()
            if s["vx"] is not None and math.sqrt(
                    s["vx"] ** 2 + s["vy"] ** 2 + s["vz"] ** 2) < BRAKE_SETTLE_V:
                break
            time.sleep(0.05)
        # 2) latch a position hold at the SETTLED spot (not where the trip fired)
        s = self.snapshot()
        if s["x"] is not None:
            self._sp = {"type": "pos", "x": s["x"], "y": s["y"], "z": s["z"],
                        "yaw": s["yaw"] if s["yaw"] is not None else 0.0}
        if tag == "EV":
            print("[EV] HOLDING. move/goto are refused until vision returns - "
                  "with no absolute fix there is no safe direction. "
                  "`land` still works (height is from the downward lidar, "
                  "which is independent of vision).")
        else:
            print(f"[{tag}] HOLDING. Enter a new move/yaw "
                  f"(unsafe directions are rejected).")

    def direction_is_safe(self, bearing_deg):
        """(safe, clearance_m) for a body-FRD travel bearing. Safe if the cone is
        clear to >= OBST_CLEAR_M, or nothing is seen there (blind)."""
        d, _ = self._cone_min(bearing_deg, OBST_CONE_DEG)
        return (d is None or (d - DRONE_RADIUS_M) >= OBST_CLEAR_M), d

    def obstacle_summary(self):
        """One-line avoidance state for `status`: is the ring live, how old, and the
        nearest obstacle in each quadrant (front/right/back/left). Lets you SEE
        whether the OBSTACLE_DISTANCE feed is actually arriving."""
        if self._obst_master is None:
            return "OFF (no obstacle endpoint; --no-avoid or feed not opened)"
        with self._lock:
            ring = self._obst_ring
            age = time.time() - self._obst_stamp if self._obst_stamp else None
        if ring is None or age is None:
            return ("ON but NO ring received yet - is laserscan_to_obstacle.py / "
                    "obstacle_distance_px4.py running and sending to 14541?")
        if age > OBST_STALE_S:
            return f"ON but STALE ({age:.1f}s old) - feed stopped? (blind: won't trip)"
        def near(brg):
            d, _ = self._cone_min(brg, 45.0)
            return f"{d:.1f}" if d is not None else "clr"
        return (f"ON, ring {age*1000:.0f}ms  F={near(0)} R={near(90)} "
                f"B={near(180)} L={near(270)} m (centre dist)")

    def _gate_travel(self, bearing_deg, what):
        """Shared move/goto guard: reject an unsafe travel bearing, else clear the
        latch. Returns True if the command may proceed."""
        # Localization first: an obstacle hold is cleared by choosing a safe
        # direction, but a localization hold cannot be - every direction is
        # computed from a pose we no longer trust, so there is no safe one. Only
        # returning vision releases this (see _ev_check).
        with self._lock:
            ev_lost = self._ev_lost
        if ev_lost:
            print(f"[EV] refused {what}: LOCALIZATION LOST (no external vision). "
                  f"Holding position until SLAM /odom recovers - check the "
                  f"lidar_slam launch and the injector. `land` is still available.")
            return False
        if self._obst_master is None:
            return True
        safe, d = self.direction_is_safe(bearing_deg)
        if not safe:
            gap = d - DRONE_RADIUS_M
            print(f"[AVOID] refused {what}: obstacle {d:.2f} m -> skin gap "
                  f"{gap:.2f} m at {bearing_deg:+.0f} deg (need {OBST_CLEAR_M:.1f} m). "
                  f"Try `yaw` or another direction.")
            return False
        self._blocked = False        # a safe command clears the hold
        self._trip_hits = 0          # and resets the trip debounce
        return True

    # ---- command helpers ---------------------------------------------

    def _command_long(self, command, *params, confirmation=0):
        params = list(params) + [0] * (7 - len(params))
        with self._lock:
            self._acks.pop(command, None)
        with self._txlock:
            self.master.mav.command_long_send(
                self.sysid, self.compid, command, confirmation, *params)
        return self._wait_ack(command)

    def _wait_ack(self, command, timeout=3.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                result = self._acks.get(command)
            if result is not None:
                return result == mavutil.mavlink.MAV_RESULT_ACCEPTED, result
            time.sleep(0.05)
        return False, None

    def set_mode(self, mode_name):
        """PX4 mode switch via MAV_CMD_DO_SET_MODE (main_mode, sub_mode)."""
        mode_name = mode_name.upper()
        if mode_name not in PX4_MODES:
            raise ValueError(f"Unknown PX4 mode {mode_name!r}. Known: {sorted(PX4_MODES)}")
        main_mode, sub_mode = PX4_MODES[mode_name]
        base = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        ok, res = self._command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, base, main_mode, sub_mode)
        return ok

    def arm(self, force=False):
        return self._command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            1, 21196 if force else 0)

    def disarm(self, force=False):
        self.stop_offboard()
        return self._command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 21196 if force else 0)

    def takeoff(self, altitude_m, force=True):
        """Offboard take-off: climb `altitude_m` above the current spot and hold.

        Streams the climb setpoint, enters OFFBOARD, then arms. If a normal arm is
        rejected (preflight check) and force=True, retries with a forced arm - which
        is usually what's needed in SITL when a non-estimator check is blocking.
        """
        s = self._wait_local_position()
        self._sp = {"type": "pos", "x": s["x"], "y": s["y"],
                    "z": s["z"] - float(altitude_m), "yaw": s["yaw"]}
        self._offboard.set()
        time.sleep(0.5)
        if not self.set_mode("OFFBOARD"):
            return False, "OFFBOARD rejected"
        ok, res = self.arm()
        if not ok and force:
            print("  arm rejected by preflight - retrying forced arm")
            ok, res = self.arm(force=True)
        return ok, res

    def land(self):
        self.stop_offboard()
        return self.set_mode("AUTO.LAND")

    def rtl(self):
        self.stop_offboard()
        return self.set_mode("AUTO.RTL")

    def hold(self):
        """Hold current position (offboard position setpoint = here)."""
        s = self._wait_local_position()
        self._sp = {"type": "pos", "x": s["x"], "y": s["y"], "z": s["z"], "yaw": s["yaw"]}
        if not self._offboard.is_set():
            return self.start_offboard(hold=False)
        return True

    def goto(self, north, east, down):
        """Absolute LOCAL_NED position setpoint (metres). Offboard.

        Rejected (returns False) if the straight-line path to the point is not
        clear to OBST_CLEAR_M in the travel direction."""
        s = self._wait_local_position()
        dn, de = float(north) - s["x"], float(east) - s["y"]
        if math.hypot(dn, de) > 0.15:
            yaw = s["yaw"]
            fwd = dn * math.cos(yaw) + de * math.sin(yaw)
            right = -dn * math.sin(yaw) + de * math.cos(yaw)
            if not self._gate_travel(math.degrees(math.atan2(right, fwd)), "goto"):
                return False
        self._sp = {"type": "pos", "x": float(north), "y": float(east),
                    "z": float(down), "yaw": s["yaw"]}
        if not self._offboard.is_set():
            self.start_offboard(hold=False)
        return True

    def set_velocity(self, vx, vy, vz):
        """Body-frame velocity m/s (vx fwd, vy right, vz down). Offboard."""
        self._sp = {"type": "vel", "vx": float(vx), "vy": float(vy),
                    "vz": float(vz), "yaw_rate": 0.0}
        if not self._offboard.is_set():
            self.start_offboard(hold=False)

    def move(self, dx, dy, dz=0.0):
        """Relative move in metres, body frame (dx fwd, dy right, dz down).

        Computed as an absolute LOCAL_NED position from the current pose, which
        is the right primitive for a GPS-denied indoor room.

        Rejected (returns False) if the body cone toward (dx, dy) is not clear to
        OBST_CLEAR_M - the caller is expected to yaw or pick another direction.
        """
        if abs(dx) > 1e-3 or abs(dy) > 1e-3:
            if not self._gate_travel(math.degrees(math.atan2(dy, dx)), "move"):
                return False
        s = self._wait_local_position()
        yaw = s["yaw"]
        n = s["x"] + dx * math.cos(yaw) - dy * math.sin(yaw)
        e = s["y"] + dx * math.sin(yaw) + dy * math.cos(yaw)
        d = s["z"] + dz
        self._sp = {"type": "pos", "x": n, "y": e, "z": d, "yaw": yaw}
        if not self._offboard.is_set():
            self.start_offboard(hold=False)
        return True

    def yaw(self, degrees, relative=True):
        """Yaw by (relative) or to (absolute) `degrees`. Offboard position setpoint."""
        s = self._wait_local_position()
        cur = s["yaw"]
        target = cur + math.radians(degrees) if relative else math.radians(degrees)
        self._sp = {"type": "pos", "x": s["x"], "y": s["y"], "z": s["z"], "yaw": target}
        if not self._offboard.is_set():
            self.start_offboard(hold=False)
        return True, None

    # ---- waypoint missions (local NED, GPS-denied indoor) -------------

    def _wait_arrival(self, tx, ty, tz, tol=0.3, timeout=40.0):
        """Block until within tol (m) of target NED point, or timeout."""
        deadline = time.time() + timeout
        dist = None
        while time.time() < deadline:
            if self._blocked:            # watchdog preempted us -> stop waiting
                return False, dist
            s = self.snapshot()
            if s["x"] is not None:
                dist = math.sqrt((s["x"] - tx) ** 2 + (s["y"] - ty) ** 2
                                 + (s["z"] - tz) ** 2)
                if dist <= tol:
                    return True, dist
            time.sleep(0.1)
        return False, dist

    def _wait_yaw(self, target, tol_deg=5.0, timeout=15.0):
        """Block until heading is within tol_deg of target (rad), or timeout."""
        deadline = time.time() + timeout
        err = None
        while time.time() < deadline:
            s = self.snapshot()
            if s["yaw"] is not None:
                diff = math.atan2(math.sin(target - s["yaw"]),
                                  math.cos(target - s["yaw"]))
                err = abs(math.degrees(diff))
                if err <= tol_deg:
                    return True, err
            time.sleep(0.1)
        return False, err

    def run_pattern(self, steps=None, takeoff_alt=1.0, tol=0.3, wp_timeout=40.0,
                    hold_s=1.0, do_takeoff=True, land_at_end=True):
        """Fly the hardcoded INDOOR_PATTERN (move/yaw steps) via OFFBOARD.

        Optionally arms + takes off `takeoff_alt` m first, then executes each
        step, waiting for arrival (position) or heading (yaw) before the next -
        which is what makes the relative composition match a manual REPL run.
        Optional land at the end.
        """
        steps = INDOOR_PATTERN if steps is None else steps

        if do_takeoff:
            ok, res = self.takeoff(takeoff_alt)
            if not ok:
                print(f"takeoff failed (result={res})"); return False
            s = self._wait_local_position()
            print(f"climbing to {takeoff_alt} m ...")
            self._wait_arrival(s["x"], s["y"], s["z"] - takeoff_alt, tol, wp_timeout)
        else:
            self.hold()   # ensure offboard is streaming before we command steps

        for i, step in enumerate(steps):
            if self._blocked:
                print(f"[AVOID] {self._block_reason} - pattern paused, holding. "
                      f"Take over in the REPL (move/yaw to a clear direction).")
                return False
            s = self._wait_local_position()
            yaw = s["yaw"]
            if step[0] == "move":
                dx, dy, dz = float(step[1]), float(step[2]), float(step[3])
                n = s["x"] + dx * math.cos(yaw) - dy * math.sin(yaw)
                e = s["y"] + dx * math.sin(yaw) + dy * math.cos(yaw)
                d = s["z"] + dz
                self._sp = {"type": "pos", "x": n, "y": e, "z": d, "yaw": yaw}
                print(f"[{i + 1}/{len(steps)}] move {dx} {dy} {dz} "
                      f"-> N={n:.2f} E={e:.2f} D={d:.2f}")
                reached, dist = self._wait_arrival(n, e, d, tol, wp_timeout)
                ds = f"{dist:.2f}" if dist is not None else "?"
                print(f"    {'reached' if reached else 'TIMEOUT'} (d={ds} m)")
            elif step[0] == "yaw":
                deg = float(step[1])
                target = yaw + math.radians(deg)
                self._sp = {"type": "pos", "x": s["x"], "y": s["y"], "z": s["z"],
                            "yaw": target}
                print(f"[{i + 1}/{len(steps)}] yaw {deg:+.0f}")
                reached, err = self._wait_yaw(target)
                es = f"{err:.1f}" if err is not None else "?"
                print(f"    {'settled' if reached else 'TIMEOUT'} (err={es} deg)")
            else:
                print(f"    skipping unknown step: {step!r}")
                continue
            time.sleep(hold_s)

        print("pattern complete")
        if land_at_end:
            print("landing"); self.land()
        return True

    def run_mission(self, waypoints, tol=0.3, wp_timeout=40.0, hold_s=1.0,
                    pass_tol=None,
                    land_at_end=True):
        """Fly a list of local-NED waypoints in sequence via OFFBOARD.

        waypoints: list of (N, E, D, yaw_deg), metres, relative to the EKF origin
        (takeoff spot); D negative = up. Uses LOCAL_POSITION_NED, which in the
        GPS-denied setup is fed by RTAB-Map (vision_pose -> EKF2). Arms + enters
        OFFBOARD on the first waypoint, waits for arrival at each, optional land.
        """
        if not waypoints:
            print("no waypoints"); return False
        s = self._wait_local_position()
        n0, e0, d0, y0 = waypoints[0]
        self._sp = {"type": "pos", "x": float(n0), "y": float(e0),
                    "z": float(d0), "yaw": math.radians(float(y0))}
        self._offboard.set()
        time.sleep(0.5)                     # prime the setpoint stream
        if not self.set_mode("OFFBOARD"):
            print("OFFBOARD rejected"); return False
        ok, res = self.arm()
        if not ok:
            print("  arm rejected - forcing"); ok, res = self.arm(force=True)
        if not ok:
            print(f"  arm failed (result={res})"); return False
        for i, (n, e, d, y) in enumerate(waypoints):
            print(f"-> wp {i + 1}/{len(waypoints)}: N={n} E={e} D={d} yaw={y}")
            # GATE EVERY WAYPOINT, exactly as goto() does.
            #
            # THIS PATH USED TO BYPASS THE AVOIDANCE LAYER ENTIRELY, and that is how the
            # 2026-08-20 flight ended up 9.2 cm inside a wall. _gate_travel was called
            # from goto() and move() - the interactive commands - and nowhere else, while
            # run_mission (which flies EVERY leg of an autonomous mission) wrote self._sp
            # straight through. So the obstacle ring was live and correct the whole time,
            # reporting walls at 0.20 m, and nothing ever asked it: first contact at
            # t=90.5 s, then 100 % contact for the remaining 6 minutes.
            #
            # Corridors stay flyable: at the +/-15 deg cone a 1 m corridor shows its side
            # walls at 0.50/sin15 = 1.93 m, a 1.55 m skin gap, comfortably over
            # OBST_CLEAR_M - see the arithmetic beside OBST_CONE_DEG. The gate refuses
            # walls AHEAD, not walls alongside.
            # DOES THIS WAYPOINT ACTUALLY TURN? The endpoint needs room for the yaw
            # circle only if a rotation is commanded there; otherwise the vehicle just
            # stops, and the strike radius is the right test.
            _s = self._wait_local_position()
            _yaw_cmd = math.radians(float(y))
            _turns = True
            if _s is not None and _s.get("yaw") is not None:
                _dy = abs((math.degrees(_s["yaw"]) - float(y) + 180.0) % 360.0 - 180.0)
                _turns = _dy > LEG_TURN_DEADBAND_DEG
            _final = (i == len(waypoints) - 1)
            if not self._leg_is_safe(float(n), float(e), final=_final, turns=_turns):
                # IF THE ONLY PROBLEM IS ROOM TO ROTATE, FLY IT WITHOUT ROTATING.
                #
                # An INTERMEDIATE waypoint's yaw is a planner artefact - global_planner
                # writes the path heading there, and it is cosmetic. The heading that
                # matters is the FINAL waypoint's, which is frontier_explorer's
                # raycast-chosen observation yaw. So when a mid-path waypoint is refused
                # only because there is no room to turn at it, the right answer is to pass
                # through holding the current heading, not to abandon the mission.
                #
                # THIS IS FORCED BY GEOMETRY, not preference. Measured 2026-08-20 17:35:
                # three consecutive legs refused with gaps of 0.450/0.449/0.462 m against a
                # 0.481 m stop-and-turn requirement - i.e. by 2-3 cm - while the vehicle's
                # MEDIAN clearance that flight was 0.428 m. In a 1 m corridor a perfectly
                # centred vehicle has 0.019 m of slack on that requirement, so requiring
                # rotation room at every waypoint makes refusal the normal case. Dropping
                # an unnecessary rotation costs nothing; refusing the leg costs the flight.
                if _turns and not _final and self._leg_is_safe(
                        float(n), float(e), final=False, turns=False, retry=True):
                    _yaw_cmd = _s["yaw"] if (_s and _s.get("yaw") is not None) else _yaw_cmd
                    print(f"   no room to turn at wp {i + 1} - flying through on the "
                          f"current heading instead (its yaw is path heading, not an "
                          f"observation requirement)")
                else:
                    print(f"   LEG REFUSED at wp {i + 1}/{len(waypoints)} - abandoning "
                          f"this mission. The planner will be given a fresh goal; flying "
                          f"into a refused bearing is what wedges the vehicle.")
                    return False
            self._sp = {"type": "pos", "x": float(n), "y": float(e),
                        "z": float(d), "yaw": _yaw_cmd}
            # ---- FLY THROUGH THE MIDDLE OF A LEG, STOP ONLY WHERE IT MATTERS.
            #
            # MEASURED 2026-09-01 on a live maze5 run: mean ground speed 0.26 m/s against
            # an MPC_XY_VEL_MAX of 1.0, and the vehicle was STATIONARY for 39% of a 70 s
            # window. The goals were fine (3.96 m hops every 10.6 s) - the time was going
            # into this loop. `time.sleep(hold_s)` ran after EVERY waypoint, so a 5-waypoint
            # leg spent five full seconds parked on purpose, and PX4 decelerated into each
            # 0.3 m tolerance sphere on the way.
            #
            # An INTERMEDIATE waypoint on a straight run is a via-point, not a destination:
            # nothing observes there, nothing rotates there. Only two kinds of waypoint
            # deserve a settle - the LAST one (the vehicle stops and the sensor collects)
            # and one that COMMANDS A YAW CHANGE (it has to rotate, and rotating while
            # translating is what costs a VIO its tracking). Everything else is passed
            # through at a looser radius, which also rounds the corner instead of stopping
            # dead on it.
            #
            # pass_tol is bounded by half the leg length so a via-point can never be
            # "reached" before the vehicle has actually set off toward it.
            _settle = _final or _turns
            _tol = tol
            if not _settle:
                _lead = pass_tol if pass_tol is not None else max(tol, 0.45)
                _prev = waypoints[i - 1][:3] if i else (n, e, d)
                _leglen = math.dist((float(n), float(e)), (float(_prev[0]), float(_prev[1])))
                _tol = max(tol, min(_lead, 0.5 * _leglen)) if _leglen > 1e-6 else tol
            reached, dist = self._wait_arrival(float(n), float(e), float(d),
                                               _tol, wp_timeout)
            ds = f"{dist:.2f}" if dist is not None else "?"
            print(f"   {'reached' if reached else 'TIMEOUT'} (d={ds} m"
                  f"{'' if _settle else f', passed through @{_tol:.2f} m'})")
            if _settle:
                time.sleep(hold_s)
        print("mission complete")
        if land_at_end:
            print("landing"); self.land()
        return True


class Fleet:
    def __init__(self, port, baud=None, source_system=255, obstacle_port=None):
        kw = {"source_system": source_system}
        if baud:
            kw["baud"] = baud
        self.master = mavutil.mavlink_connection(port, **kw)
        self.vehicles = {}
        self.active = None
        # optional OBSTACLE_DISTANCE endpoint for the avoidance safety layer
        self._obst_master = None
        if obstacle_port and obstacle_port.lower() != "none":
            try:
                self._obst_master = mavutil.mavlink_connection(obstacle_port)
                print(f"Obstacle-avoidance: listening for OBSTACLE_DISTANCE on "
                      f"{obstacle_port}")
            except Exception as e:
                print(f"Obstacle-avoidance disabled (can't open {obstacle_port}: {e})")

    def connect(self, timeout=15):
        # Plain wait_heartbeat() returns whichever HEARTBEAT arrives first, and reads
        # back self.master.target_system/target_component - both fine on a direct
        # SITL link with exactly one system on it, but wrong here: mavlink-router
        # mirrors the FULL bus onto every client, including QGroundControl's own
        # heartbeat (sysid=255, autopilot=MAV_AUTOPILOT_INVALID - it's a GCS, not a
        # vehicle). Racing that against the real FMU's (sysid=1, autopilot=PX4) is
        # nondeterministic, and target_system/target_component aren't reliably
        # populated by wait_heartbeat() on this connection type anyway - confirmed
        # live: it returned sysid=0 (an uninitialized default, not any real system).
        # Filter for an actual autopilot's heartbeat and read sysid/compid off the
        # message itself instead of trusting connection-level state.
        print(f"Waiting for heartbeat on {self.master.address} ...")
        deadline = time.time() + timeout
        hb = None
        while time.time() < deadline:
            remaining = deadline - time.time()
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True,
                                          timeout=max(0.0, remaining))
            if msg is None:
                break
            if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                continue  # a GCS (or other non-vehicle) heartbeat mirrored onto the link
            hb = msg
            break
        if hb is None:
            raise TimeoutError(
                "No MAVLink heartbeat from an autopilot. Is PX4 SITL up? Default port "
                "is udpin:0.0.0.0:14540 - try --port udp:127.0.0.1:14550 for the QGC "
                "stream.")
        sysid = hb.get_srcSystem()
        compid = hb.get_srcComponent()
        print(f"Heartbeat from system {sysid} component {compid} "
              f"(autopilot={mavutil.mavlink.enums['MAV_AUTOPILOT'][hb.autopilot].name})")
        v = Vehicle(self.master, sysid, compid, obst_master=self._obst_master)
        self.vehicles[sysid] = v
        self.active = v
        self._request_streams(v, rate_hz=10)
        return v

    def _request_streams(self, v, rate_hz=10):
        interval_us = int(1e6 / rate_hz)
        for msg_id in (
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
            mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,
        ):
            v.master.mav.command_long_send(
                v.sysid, v.compid, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, msg_id, interval_us, 0, 0, 0, 0, 0)

    def close(self):
        for v in self.vehicles.values():
            v.close()
        self.master.close()
        if self._obst_master is not None:
            self._obst_master.close()


def load_waypoints(path):
    """Read a mission file: 'N E D [yaw_deg]' per line, '#' comments. -> list of 4-tuples."""
    wps = []
    with open(path) as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            n, e, d = float(parts[0]), float(parts[1]), float(parts[2])
            y = float(parts[3]) if len(parts) > 3 else 0.0
            wps.append((n, e, d, y))
    return wps


def print_status(v):
    t = v.snapshot()
    age = time.time() - t["last_heartbeat"] if t["last_heartbeat"] else None
    link = f"{age:.1f}s ago" if age is not None else "no heartbeat"
    yaw_deg = math.degrees(t["yaw"]) if t["yaw"] is not None else None
    print("=" * 40)
    print(f" System id     : {v.sysid}")
    print(f" Link          : {link}")
    print(f" Mode          : {t['mode']}")
    print(f" Armed         : {t['armed']}")
    print(f" Local x,y,z   : {t['x']}, {t['y']}, {t['z']}  (NED m)")
    print(f" Vel x,y,z     : {t['vx']}, {t['vy']}, {t['vz']}  (m/s)")
    print(f" Yaw           : {yaw_deg} deg")
    print(f" Lidar (down)  : {t['dist_bottom']} m")
    print(f" Rel alt       : {t['rel_alt']} m")
    print(f" Battery       : {t['battery_v']} V ({t['battery_pct']}%)")
    print(f" Avoidance     : {v.obstacle_summary()}")
    print(f" Localization  : {v.ev_summary()}")
    print("=" * 40)


def repl(fleet):
    v = fleet.active
    print("PX4 interactive. Commands:")
    print("  status | arm [force] | disarm [force] | mode <NAME>")
    print("  offboard | hold | takeoff <m> | land | rtl")
    print("  goto <N> <E> <D> | vel <vx> <vy> <vz> | move <dx> <dy> [dz] | yaw <deg>")
    print("  mission <file> [noland] | pattern [noland] [notakeoff] | watch | quit")
    print(f"  modes: {', '.join(sorted(PX4_MODES))}")
    if v._obst_master is not None:
        print(f"  [avoidance ON] holds within {OBST_TRIGGER_M:.1f} m of travel; "
              f"move/goto need {OBST_CLEAR_M:.1f} m clearance (yaw always allowed)")
        print(f"  [localization failsafe ON] position-holds if external vision stops for "
              f"{EV_STALE_S:.1f}s (or {EV_STALE_PERIODS:.0f} heartbeat intervals, "
              f"whichever is longer - the gate learns the source's rate); "
              f"move/goto refused until it returns")
    while True:
        try:
            line = input("px4> ").strip()
        except (EOFError, KeyboardInterrupt):
            print(); break
        if not line:
            continue
        parts = line.split()
        cmd, args = parts[0].lower(), parts[1:]
        try:
            if cmd in ("quit", "exit"):
                break
            elif cmd == "status":
                print_status(v)
            elif cmd == "arm":
                ok, res = v.arm(force="force" in args)
                print("ARM ok" if ok else f"ARM rejected (result={res})")
            elif cmd == "disarm":
                ok, res = v.disarm(force="force" in args)
                print("DISARM ok" if ok else f"DISARM rejected (result={res})")
            elif cmd == "mode":
                print("mode set" if v.set_mode(args[0]) else "mode rejected")
            elif cmd == "offboard":
                print("OFFBOARD on" if v.start_offboard() else "OFFBOARD rejected")
            elif cmd == "hold":
                print("holding" if v.hold() else "hold rejected")
            elif cmd == "takeoff":
                ok, res = v.takeoff(float(args[0]))
                print("takeoff ok" if ok else f"takeoff rejected ({res})")
            elif cmd == "land":
                print("LAND" if v.land() else "land rejected")
            elif cmd == "rtl":
                print("RTL" if v.rtl() else "rtl rejected")
            elif cmd == "goto":
                print("goto sent" if v.goto(args[0], args[1], args[2])
                      else "goto refused (obstacle)")
            elif cmd == "vel":
                v.set_velocity(args[0], args[1], args[2]); print("velocity sent")
            elif cmd == "move":
                dz = float(args[2]) if len(args) > 2 else 0.0
                print("move sent" if v.move(float(args[0]), float(args[1]), dz)
                      else "move refused (obstacle)")
            elif cmd == "yaw":
                v.yaw(float(args[0])); print("yaw sent")
            elif cmd == "mission":
                if not args:
                    print("usage: mission <file> [noland]")
                else:
                    wps = load_waypoints(args[0])
                    print(f"loaded {len(wps)} waypoints from {args[0]}")
                    v.run_mission(wps, land_at_end=("noland" not in args))
            elif cmd == "pattern":
                print(f"running hardcoded pattern ({len(INDOOR_PATTERN)} steps)")
                v.run_pattern(land_at_end=("noland" not in args),
                              do_takeoff=("notakeoff" not in args))
            elif cmd == "watch":
                print("Ctrl-C to stop watching")
                try:
                    while True:
                        print_status(v); time.sleep(1)
                except KeyboardInterrupt:
                    print()
            else:
                print(f"unknown command: {cmd}")
        except Exception as e:
            print(f"error: {e}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT,
                   help=f"MAVLink endpoint (default {DEFAULT_PORT}). "
                        "For a serial handset use e.g. /dev/ttyACM0 with --baud.")
    p.add_argument("--baud", type=int, default=None)
    p.add_argument("--obstacle-port", default=DEFAULT_OBSTACLE_PORT,
                   help=f"OBSTACLE_DISTANCE endpoint for the avoidance safety "
                        f"layer (default {DEFAULT_OBSTACLE_PORT}; 'none' to disable)")
    p.add_argument("--no-avoid", action="store_true",
                   help="disable the obstacle-avoidance safety layer")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    a = sub.add_parser("arm"); a.add_argument("--force", action="store_true")
    d = sub.add_parser("disarm"); d.add_argument("--force", action="store_true")
    m = sub.add_parser("mode"); m.add_argument("name")
    t = sub.add_parser("takeoff"); t.add_argument("altitude", type=float)
    ms = sub.add_parser("mission"); ms.add_argument("file")
    ms.add_argument("--noland", action="store_true")
    pt = sub.add_parser("pattern")
    pt.add_argument("--noland", action="store_true")
    pt.add_argument("--notakeoff", action="store_true")
    sub.add_parser("land"); sub.add_parser("rtl"); sub.add_parser("repl")
    args = p.parse_args()

    fleet = Fleet(args.port, baud=args.baud,
                  obstacle_port=None if args.no_avoid else args.obstacle_port)
    try:
        v = fleet.connect()
        time.sleep(1.0)
        if args.cmd == "status":
            print_status(v)
        elif args.cmd == "arm":
            ok, res = v.arm(force=args.force)
            print("ARM ok" if ok else f"ARM rejected (result={res})")
        elif args.cmd == "disarm":
            ok, res = v.disarm(force=args.force)
            print("DISARM ok" if ok else f"DISARM rejected (result={res})")
        elif args.cmd == "mode":
            print("mode set" if v.set_mode(args.name) else "mode rejected")
        elif args.cmd == "takeoff":
            ok, res = v.takeoff(args.altitude)
            print("takeoff ok" if ok else f"takeoff rejected ({res})")
        elif args.cmd == "land":
            print("LAND" if v.land() else "land rejected")
        elif args.cmd == "rtl":
            print("RTL" if v.rtl() else "rtl rejected")
        elif args.cmd == "mission":
            wps = load_waypoints(args.file)
            print(f"loaded {len(wps)} waypoints from {args.file}")
            v.run_mission(wps, land_at_end=not args.noland)
        elif args.cmd == "pattern":
            print(f"running hardcoded pattern ({len(INDOOR_PATTERN)} steps)")
            v.run_pattern(land_at_end=not args.noland, do_takeoff=not args.notakeoff)
        elif args.cmd == "repl":
            repl(fleet)
    finally:
        fleet.close()


if __name__ == "__main__":
    sys.exit(main())
