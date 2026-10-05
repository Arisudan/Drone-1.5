#!/usr/bin/env python3
"""
================================================================================
SCRIPT: radxa_log_janitor.py
PURPOSE: Stop the Radxa's log files filling its disk
================================================================================

Runs daily from a systemd timer (deploy/drone-janitor.timer). Standard library
only. It looks at exactly two kinds of thing and nothing else:

  1. ROS 2 logs under ~/.ros/log: dated folders (2026-07-28-20-35-41-623938-host-pid)
     and per-node files (stereo_odometry_9234_1785252255520.log). Anything older
     than KEEP_DAYS goes, but the newest KEEP_NEWEST folders and KEEP_NEWEST_FILES
     files are always kept, and anything touched in the last 24 h is never removed.
  2. MAVLink telemetry logs (mav.tlog, mav.tlog.raw) in the Flop folder. Once one
     is bigger than ROTATE_MB it is compressed to a dated .gz and emptied in place
     (so a program still writing to it is not disturbed). Compressed copies older
     than KEEP_DAYS_TLOG days, beyond the newest KEEP_TLOG, are removed.

  If free space is below LOW_DISK_PCT it gets stricter about ROS logs (3 days).

  DRY RUN BY DEFAULT: without --apply it only prints what it would do. Every
  action is printed (and so lands in the journal).
================================================================================
"""

from __future__ import annotations

import argparse
import gzip
import os
import re
import shutil
import sys
import time
from dataclasses import dataclass
from typing import List, Optional

ROS_DIR_RE = re.compile(r"^\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}")
# Per-node log files that ROS writes straight into ~/.ros/log: <node>_<pid>_<unix ms>.log
# (stereo_odometry_9234_1785252255520.log). On this Radxa these, not the dated
# folders, are most of the space.
ROS_FILE_RE = re.compile(r"^[A-Za-z0-9_.\-]+_\d+_\d{12,}\.log$")
TLOG_NAMES = ("mav.tlog", "mav.tlog.raw")
DAY = 86400.0


@dataclass
class Policy:
    ros_log_dir: str = os.path.expanduser("~/.ros/log")
    flop_dir: str = os.path.expanduser("~/Flop")
    keep_days: float = 14.0
    keep_newest: int = 20
    keep_newest_files: int = 50
    never_touch_newer_than_s: float = DAY
    rotate_mb: float = 50.0
    keep_tlog: int = 10
    keep_days_tlog: float = 30.0
    low_disk_pct: float = 15.0
    low_disk_keep_days: float = 3.0


@dataclass
class Action:
    kind: str            # "delete" | "rotate"
    path: str
    reason: str
    size: int = 0


def tree_size(path: str) -> int:
    total = 0
    for root, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.lstat(os.path.join(root, f)).st_size
            except OSError:
                pass
    return total


def plan(pol: Policy, now: float, free_pct: Optional[float] = None) -> List[Action]:
    """What to do, without doing it."""
    acts: List[Action] = []
    keep_days = pol.keep_days
    low = free_pct is not None and free_pct < pol.low_disk_pct
    if low:
        keep_days = min(keep_days, pol.low_disk_keep_days)

    # 1. ROS log folders
    try:
        entries = []
        for name in os.listdir(pol.ros_log_dir):
            p = os.path.join(pol.ros_log_dir, name)
            if os.path.islink(p) or not os.path.isdir(p) or not ROS_DIR_RE.match(name):
                continue
            entries.append((os.lstat(p).st_mtime, p))
    except OSError:
        entries = []
    entries.sort(reverse=True)                                   # newest first
    for rank, (mtime, p) in enumerate(entries):
        age = now - mtime
        if rank < pol.keep_newest or age < pol.never_touch_newer_than_s or age < keep_days * DAY:
            continue
        why = f"ROS log folder, {age / DAY:.0f} days old" + (" (disk low)" if low else "")
        acts.append(Action("delete", p, why, tree_size(p)))

    # 1b. per-node log files written straight into the log folder
    files = []
    try:
        for name in os.listdir(pol.ros_log_dir):
            p = os.path.join(pol.ros_log_dir, name)
            if ROS_FILE_RE.match(name) and os.path.isfile(p) and not os.path.islink(p):
                files.append((os.lstat(p).st_mtime, p))
    except OSError:
        pass
    files.sort(reverse=True)
    for rank, (mtime, p) in enumerate(files):
        age = now - mtime
        if rank < pol.keep_newest_files or age < pol.never_touch_newer_than_s or age < keep_days * DAY:
            continue
        acts.append(Action("delete", p, f"ROS node log, {age / DAY:.0f} days old" + (" (disk low)" if low else ""),
                           os.lstat(p).st_size))

    # 2. telemetry logs
    for name in TLOG_NAMES:
        p = os.path.join(pol.flop_dir, name)
        try:
            size = os.lstat(p).st_size
        except OSError:
            continue
        if os.path.isfile(p) and not os.path.islink(p) and size > pol.rotate_mb * 1e6:
            acts.append(Action("rotate", p, f"{size / 1e6:.0f} MB is over {pol.rotate_mb:g} MB", size))
    rotated = []
    try:
        for name in os.listdir(pol.flop_dir):
            if re.match(r"^mav\.tlog(\.raw)?\.\d{8}-\d{6}\.gz$", name):
                p = os.path.join(pol.flop_dir, name)
                rotated.append((os.lstat(p).st_mtime, p))
    except OSError:
        pass
    rotated.sort(reverse=True)
    for rank, (mtime, p) in enumerate(rotated):
        if rank >= pol.keep_tlog and now - mtime > pol.keep_days_tlog * DAY:
            acts.append(Action("delete", p, f"old compressed telemetry log, {(now - mtime) / DAY:.0f} days",
                               os.lstat(p).st_size))
    return acts


def apply(acts: List[Action], now: float) -> int:
    """Carry the actions out. Returns bytes freed (approximate for rotations)."""
    freed = 0
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(now))
    for a in acts:
        try:
            if a.kind == "delete":
                if os.path.isdir(a.path) and not os.path.islink(a.path):
                    shutil.rmtree(a.path)
                else:
                    os.remove(a.path)
                freed += a.size
            elif a.kind == "rotate":
                gz = f"{a.path}.{stamp}.gz"
                with open(a.path, "rb") as src, gzip.open(gz, "wb", compresslevel=6) as dst:
                    shutil.copyfileobj(src, dst)
                with open(a.path, "r+b") as f:        # empty in place: a writer keeps its file
                    f.truncate(0)
                freed += max(0, a.size - os.path.getsize(gz))
        except OSError as exc:
            print(f"  FAILED {a.kind} {a.path}: {exc}", flush=True)
    return freed


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Radxa log janitor (dry run unless --apply)")
    ap.add_argument("--apply", action="store_true", help="actually delete / rotate")
    ap.add_argument("--keep-days", type=float, default=Policy.keep_days)
    ap.add_argument("--ros-log-dir", default=Policy.ros_log_dir)
    ap.add_argument("--flop-dir", default=Policy.flop_dir)
    args = ap.parse_args(argv)
    pol = Policy(ros_log_dir=args.ros_log_dir, flop_dir=args.flop_dir, keep_days=args.keep_days)
    now = time.time()
    u = shutil.disk_usage(os.path.expanduser("~"))
    free_pct = 100.0 * u.free / u.total
    acts = plan(pol, now, free_pct)
    print(f"disk: {u.free / 1e9:.1f} GB free ({free_pct:.0f} %)  mode: {'APPLY' if args.apply else 'dry run'}", flush=True)
    show = acts if len(acts) <= 25 else acts[:12]
    for a in show:
        print(f"  {'would ' if not args.apply else ''}{a.kind} {a.path}  [{a.reason}, {a.size / 1e6:.1f} MB]", flush=True)
    if len(acts) > len(show):
        print(f"  ... and {len(acts) - len(show)} more of the same kinds", flush=True)
    total = sum(a.size for a in acts)
    if not acts:
        print("  nothing to do", flush=True)
    elif args.apply:
        freed = apply(acts, now)
        print(f"  freed about {freed / 1e6:.0f} MB", flush=True)
    else:
        print(f"  {len(acts)} item(s), about {total / 1e6:.0f} MB - re-run with --apply to do it", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
