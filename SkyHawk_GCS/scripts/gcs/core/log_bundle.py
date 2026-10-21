"""
================================================================================
MODULE: log_bundle.py
PURPOSE: One-Click Post-Flight Diagnostics Bundle
================================================================================

WHY THIS EXISTS:
  After a bad flight the useful evidence lives in several places: the flight
  history, the operator's settings, and whatever log files the station wrote.
  Collecting them by hand is exactly the step that gets skipped. This zips them
  with a manifest into one file that can be attached to a bug report as-is.

WHAT IS INCLUDED:
  flights.jsonl, settings.json, every ``*.log`` / ``*.tlog`` in the data
  directory, and ``manifest.json`` (timestamp, host, python, file list).
  Missing files are skipped, never fatal: a bundle of what exists beats none.
================================================================================
"""

from __future__ import annotations

import json
import logging
import platform
import sys
import time
import zipfile
from pathlib import Path
from typing import List, Optional

from core.flight_log import log_dir

log = logging.getLogger("gcs.logbundle")

_FIXED = ("flights.jsonl", "settings.json")
_GLOBS = ("*.log", "*.tlog")


def collect_files(base: Optional[Path] = None) -> List[Path]:
    """Existing files that belong in a bundle, in a stable order."""
    base = Path(base) if base else log_dir()
    found: List[Path] = [base / n for n in _FIXED if (base / n).is_file()]
    for pat in _GLOBS:
        found.extend(sorted(p for p in base.glob(pat) if p.is_file()))
    return found


def default_bundle_name(now: Optional[float] = None) -> str:
    return time.strftime("gcs_bundle_%Y%m%d_%H%M%S.zip", time.localtime(now))


def create_bundle(dest: Path, base: Optional[Path] = None) -> List[str]:
    """Write ``dest`` and return the archive names it contains (manifest last)."""
    files = collect_files(base)
    manifest = {
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "host": platform.node(),
        "platform": platform.platform(),
        "python": sys.version.split()[0],
        "files": [p.name for p in files],
    }
    names: List[str] = []
    with zipfile.ZipFile(dest, "w", zipfile.ZIP_DEFLATED) as zf:
        for p in files:
            try:
                zf.write(p, p.name)
                names.append(p.name)
            except OSError:
                log.exception("could not add %s to bundle", p)
        manifest["files"] = list(names)
        zf.writestr("manifest.json", json.dumps(manifest, indent=2))
        names.append("manifest.json")
    return names
