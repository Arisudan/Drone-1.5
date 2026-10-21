"""
================================================================================
MODULE: param_meta.py
PURPOSE: PX4 parameter descriptions and QGroundControl-style value formatting
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (core layer - no Qt, no MAVLink)
  * Upstream:      data/px4_param_meta.json.xz, built by
                   scripts/diagnostics/build_param_metadata.py from PX4's own
                   generated metadata for one firmware version
  * Downstream:    ui/params_tab.py (group list, value text, detail panel)

WHY THIS EXISTS:
  The flight controller sends parameter names and values only. Group, units,
  default, range, description and the name of each choice / bit are PX4's
  metadata, which is what QGroundControl shows. This module reads the bundled
  copy so the Parameters tab can show the same thing, offline.

DESIGN RULES:
  * Missing description is normal (hardware-specific or newer parameters) and is
    never an error: every function works with `meta=None` and falls back to the
    plain number. A missing or corrupt data file gives an EMPTY catalog, not an
    exception, so the tab still lists every parameter.
  * Read-only. Nothing here can change a parameter on the vehicle.
  * PX4 sends floats as 32-bit values, so 0.1 arrives as 0.10000000149. Display
    rounds to the parameter's own decimal places; the 'modified' test uses a
    tolerance for the same reason.
================================================================================
"""

from __future__ import annotations

import json
import lzma
import math
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Dict, List, Optional, Tuple

DATA_FILE = Path(__file__).resolve().parents[1] / "data" / "px4_param_meta.json.xz"

#: Shown for a parameter with no description in the bundled data.
NO_DESCRIPTION = "No description in the bundled PX4 parameter reference."


@dataclass(frozen=True)
class ParamMeta:
    name: str
    group: str
    type: str                                   # "INT32" | "FLOAT"
    short: str = ""
    long: str = ""
    units: str = ""
    default: Optional[float] = None
    minimum: Optional[float] = None
    maximum: Optional[float] = None
    increment: Optional[float] = None
    decimals: Optional[int] = None
    reboot: bool = False
    values: Tuple[Tuple[int, str], ...] = ()    # enum: (value, label)
    bitmask: Tuple[Tuple[int, str], ...] = ()   # (bit index, label)
    source: str = ""                            # "json" | "doc"


class ParamCatalog:
    """Descriptions for one PX4 firmware version. Cheap lookups; build once."""

    def __init__(self, entries: Optional[Dict[str, ParamMeta]] = None,
                 version: str = "", revision: str = ""):
        self._entries: Dict[str, ParamMeta] = entries or {}
        self.version = version
        self.revision = revision

    @property
    def available(self) -> bool:
        return bool(self._entries)

    def __len__(self) -> int:
        return len(self._entries)

    def get(self, name: str) -> Optional[ParamMeta]:
        return self._entries.get(name)

    def label(self) -> str:
        """'PX4 v1.17.0 (d6f12ad1c4)' or 'no descriptions loaded'."""
        if not self.available:
            return "no descriptions loaded"
        rev = f" ({self.revision})" if self.revision else ""
        return f"PX4 {self.version}{rev}"


def _from_entry(name: str, e: dict) -> ParamMeta:
    return ParamMeta(
        name=name, group=e.get("g", "") or "", type=e.get("t", "") or "",
        short=e.get("s", "") or "", long=e.get("l", "") or "", units=e.get("u", "") or "",
        default=e.get("d"), minimum=e.get("mn"), maximum=e.get("mx"),
        increment=e.get("inc"), decimals=e.get("dp"), reboot=bool(e.get("rb")),
        values=tuple((int(v), str(t)) for v, t in e.get("v", ())),
        bitmask=tuple((int(b), str(t)) for b, t in e.get("bm", ())),
        source=e.get("src", ""))


def parse_catalog(payload: dict) -> ParamCatalog:
    px4 = payload.get("px4") or {}
    entries = {n: _from_entry(n, e) for n, e in (payload.get("params") or {}).items()}
    return ParamCatalog(entries, version=px4.get("version", ""), revision=px4.get("revision", ""))


@lru_cache(maxsize=1)
def load_catalog(path: Optional[str] = None) -> ParamCatalog:
    """The bundled catalog; an empty one if the file is missing or unreadable."""
    p = Path(path) if path else DATA_FILE
    try:
        return parse_catalog(json.loads(lzma.decompress(p.read_bytes()).decode("utf-8")))
    except (OSError, ValueError, lzma.LZMAError, KeyError, TypeError):
        return ParamCatalog()


# ── grouping ──────────────────────────────────────────────────────────

def group_of(name: str, meta: Optional[ParamMeta]) -> str:
    """QGC's group for a parameter; the name prefix ('EKF2', 'CAL') when unknown."""
    if meta and meta.group:
        return meta.group
    return name.split("_", 1)[0] or "Other"


# ── value formatting ──────────────────────────────────────────────────

def _trim(text: str) -> str:
    if "." in text:
        text = text.rstrip("0").rstrip(".")
    return "0" if text in ("-0", "") else text


def format_number(value, meta: Optional[ParamMeta], is_float: bool) -> str:
    """The number the way QGC prints it: no float32 noise, no trailing zeros."""
    if not is_float:
        return str(int(value))
    f = float(value)
    if not math.isfinite(f):
        return "NaN" if math.isnan(f) else ("inf" if f > 0 else "-inf")
    if meta and meta.decimals is not None and meta.decimals >= 0:
        # PX4's decimalPlaces is a display hint; never print fewer digits than the
        # value really has at that precision (0.055 must not become 0.06).
        for dp in (meta.decimals, meta.decimals + 2, meta.decimals + 4):
            shown = f"{f:.{dp}f}"
            if abs(float(shown) - f) <= 1e-6 * max(1.0, abs(f)):
                return _trim(shown)
    s = f"{f:.6g}"
    return s if "e" in s else _trim(s)


def enum_label(meta: Optional[ParamMeta], value) -> Optional[str]:
    if not meta or not meta.values:
        return None
    try:
        iv = int(round(float(value)))
    except (TypeError, ValueError):
        return None
    for v, label in meta.values:
        if v == iv:
            return label
    return None


def bit_states(meta: Optional[ParamMeta], value) -> List[Tuple[int, str, bool]]:
    """[(bit, label, is_set)] for a bitmask parameter, else []."""
    if not meta or not meta.bitmask:
        return []
    iv = int(round(float(value)))
    return [(bit, label, bool(iv >> bit & 1)) for bit, label in meta.bitmask]


def value_text(meta: Optional[ParamMeta], value, is_float: bool) -> str:
    """What the VALUE column shows: a named choice, the names of the set bits, or the
    number - always with the raw number kept visible, and the units appended."""
    num = format_number(value, meta, is_float)
    label = enum_label(meta, value)
    if label is not None:
        text = f"{label} ({num})"
    elif meta and meta.bitmask and not is_float:
        on = [lbl for _bit, lbl, is_on in bit_states(meta, value) if is_on]
        text = f"{', '.join(on)} ({num})" if on else num
    else:
        text = num
    if meta and meta.units and label is None and not meta.bitmask:
        text += f" {meta.units}"
    return text


def default_text(meta: Optional[ParamMeta]) -> str:
    if not meta or meta.default is None:
        return "-"
    is_float = meta.type == "FLOAT" or isinstance(meta.default, float)
    label = enum_label(meta, meta.default)
    num = format_number(meta.default, meta, is_float)
    return f"{label} ({num})" if label is not None else num


def range_text(meta: Optional[ParamMeta]) -> str:
    if not meta or (meta.minimum is None and meta.maximum is None):
        return "-"
    is_float = meta.type == "FLOAT"
    lo = format_number(meta.minimum, meta, is_float) if meta.minimum is not None else "..."
    hi = format_number(meta.maximum, meta, is_float) if meta.maximum is not None else "..."
    return f"{lo} to {hi}"


def is_modified(meta: Optional[ParamMeta], value) -> Optional[bool]:
    """Differs from PX4's default? None when the default is unknown."""
    if not meta or meta.default is None:
        return None
    try:
        a, b = float(value), float(meta.default)
    except (TypeError, ValueError):
        return None
    return not math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-9)
