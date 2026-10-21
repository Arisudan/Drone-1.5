#!/usr/bin/env python3
"""
================================================================================
TOOL: build_param_metadata.py
PURPOSE: Build the parameter-description file the GCS Parameters tab bundles
================================================================================

WHY THIS EXISTS:
  The flight controller only sends parameter NAMES and VALUES. Everything else
  QGroundControl shows (group, units, default, min/max, description, the name
  of each choice or bit) is PX4's own metadata. This tool turns PX4's generated
  metadata for ONE firmware version into a single compact file the GCS ships, so
  the Parameters tab can show the same thing QGC does - offline, in the field.

TWO SOURCES, MERGED (the first wins):
  1. build/px4_sitl_default/parameters.json   exact descriptions for the modules
     in a SITL build.
  2. docs/en/advanced_config/parameter_reference.md   PX4's generated reference.
     It covers EVERY board, so it supplies the hardware-specific parameters a
     simulator build leaves out (battery dividers, DShot, PWM outputs, serial
     port choices ...). Measured on this drone's 1137 parameters: JSON alone
     describes 885, JSON + reference describes 1128.

USAGE:
  ./build_param_metadata.py --px4 ~/px4-sitl/PX4-Autopilot
  ./build_param_metadata.py --px4 /path/to/PX4-Autopilot --out some/file.json.xz
  ./build_param_metadata.py --px4 ... --board-params Drone_1.5.params   # coverage

  Re-run it when the flight controller's firmware version changes, and commit the
  new file. The version it was built from is stored inside it and shown in the tab.
================================================================================
"""

from __future__ import annotations

import argparse
import json
import lzma
import re
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional

DEFAULT_OUT = Path(__file__).resolve().parents[1] / "gcs" / "data" / "px4_param_meta.json.xz"

_TYPES = {"INT32": "INT32", "FLOAT": "FLOAT", "INT": "INT32", "REAL32": "FLOAT"}


def _norm_type(t: str) -> str:
    return _TYPES.get(str(t).strip().upper(), str(t).strip().upper())


def _num(text):
    """'15' -> 15, '0.01' -> 0.01, '' -> None. Integers stay integers."""
    if text is None:
        return None
    s = str(text).strip().replace("&nbsp;", "").strip()
    if not s or s in ("-", "&check;"):
        return None
    try:
        f = float(s)
    except ValueError:
        return None
    return int(f) if f == int(f) and "." not in s and "e" not in s.lower() else f


def _clean(text: Optional[str]) -> str:
    return re.sub(r"\s+", " ", (text or "").replace("\n", " ")).strip()


def from_json(path: Path) -> Dict[str, dict]:
    """PX4's parameters.json -> {name: compact entry}."""
    raw = json.loads(path.read_text(encoding="utf-8"))
    out: Dict[str, dict] = {}
    for p in raw["parameters"]:
        e = {"g": p.get("group", ""), "t": _norm_type(p.get("type", "")),
             "s": _clean(p.get("shortDesc")), "l": _clean(p.get("longDesc")),
             "d": p.get("default")}
        for src, dst in (("min", "mn"), ("max", "mx"), ("increment", "inc"),
                         ("decimalPlaces", "dp"), ("units", "u")):
            if p.get(src) not in (None, ""):
                e[dst] = p[src]
        if p.get("rebootRequired"):
            e["rb"] = True
        if p.get("volatile"):
            e["vol"] = True
        if p.get("values"):
            e["v"] = [[v["value"], _clean(v.get("description"))] for v in p["values"]]
        if p.get("bitmask"):
            e["bm"] = [[b["index"], _clean(b.get("description"))] for b in p["bitmask"]]
        out[p["name"]] = e
    return out


_HEAD = re.compile(r"^### (\w+) \(`(\w+)`\)")
_ITEM = re.compile(r"^- `(-?\d+)`:\s*(.*)$")


def from_markdown(path: Path) -> Dict[str, dict]:
    """PX4's generated parameter_reference.md -> {name: compact entry}."""
    out: Dict[str, dict] = {}
    group = ""
    cur: Optional[dict] = None
    mode = None            # None | "values" | "bitmask"
    paras: List[str] = []
    para: List[str] = []

    def finish():
        nonlocal cur, paras, para
        if cur is None:
            return
        if para:
            paras.append(" ".join(para))
        paras = [_clean(p) for p in paras if _clean(p)]
        if paras:
            cur["s"] = paras[0]
            if len(paras) > 1:
                cur["l"] = " ".join(paras[1:])
        cur.setdefault("s", "")
        out[cur.pop("_name")] = cur
        cur, paras, para = None, [], []

    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("## "):
            finish()
            group = line[3:].strip()
            mode = None
            continue
        m = _HEAD.match(line)
        if m:
            finish()
            cur = {"_name": m.group(1), "g": group, "t": _norm_type(m.group(2))}
            mode = None
            paras, para = [], []
            continue
        if cur is None:
            continue
        stripped = line.strip()
        if stripped in ("**Values:**", "**Bitmask:**"):
            if para:
                paras.append(" ".join(para))
                para = []
            mode = "values" if "Values" in stripped else "bitmask"
            continue
        item = _ITEM.match(stripped)
        if item and mode:
            key = "v" if mode == "values" else "bm"
            cur.setdefault(key, []).append([int(item.group(1)), _clean(item.group(2))])
            continue
        if stripped.startswith("|"):
            cells = [c.strip() for c in stripped.strip("|").split("|")]
            if cells and cells[0].lower() == "reboot" or set("".join(cells)) <= set("-: "):
                continue            # header / separator rows
            cells += [""] * (6 - len(cells))
            reboot, mn, mx, inc, default, unit = cells[:6]
            if "check" in reboot:
                cur["rb"] = True
            for key, val in (("mn", mn), ("mx", mx), ("inc", inc), ("d", default)):
                n = _num(val)
                if n is not None:
                    cur[key] = n
            if unit.strip():
                cur["u"] = unit.strip()
            mode = None
            continue
        if not stripped:
            if para:
                paras.append(" ".join(para))
                para = []
            continue
        if mode is None and not stripped.startswith(("<!--", "::", ":::")):
            para.append(stripped)
    finish()
    return out


def px4_version(px4: Path) -> dict:
    def git(*args):
        try:
            return subprocess.run(["git", "-C", str(px4), *args], capture_output=True,
                                  text=True, timeout=20).stdout.strip()
        except Exception:
            return ""
    return {"version": git("describe", "--tags", "--always"),
            "revision": git("rev-parse", "--short=10", "HEAD"),
            "date": git("log", "-1", "--format=%cs")}


def merge(json_meta: Dict[str, dict], md_meta: Dict[str, dict]) -> Dict[str, dict]:
    merged = dict(md_meta)
    merged.update(json_meta)          # exact JSON descriptions win over the reference
    for name, e in merged.items():
        e["src"] = "json" if name in json_meta else "doc"
    return merged


def read_board_names(path: Path) -> List[str]:
    names = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("#") or not line.strip():
            continue
        parts = line.rstrip("\r").split("\t")
        if len(parts) >= 3:
            names.append(parts[2])
    return names


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("USAGE:")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--px4", required=True, type=Path, help="PX4-Autopilot source tree")
    ap.add_argument("--json", type=Path, help="parameters.json (default: build/px4_sitl_default/)")
    ap.add_argument("--markdown", type=Path, help="parameter_reference.md (default: docs/en/advanced_config/)")
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--board-params", type=Path, help="a .params file, to report coverage")
    args = ap.parse_args(argv)

    jpath = args.json or args.px4 / "build" / "px4_sitl_default" / "parameters.json"
    mpath = args.markdown or args.px4 / "docs" / "en" / "advanced_config" / "parameter_reference.md"
    for p in (jpath, mpath):
        if not p.is_file():
            print(f"missing input: {p}", file=sys.stderr)
            return 2

    j, d = from_json(jpath), from_markdown(mpath)
    params = merge(j, d)
    ver = px4_version(args.px4)
    payload = {"px4": ver, "count": len(params),
               "sources": {"json": len(j), "reference": len(d)}, "params": params}

    args.out.parent.mkdir(parents=True, exist_ok=True)
    blob = json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")
    args.out.write_bytes(lzma.compress(blob, preset=9))
    print(f"PX4 {ver['version']} ({ver['revision']}, {ver['date']})")
    print(f"  parameters.json: {len(j)}   reference.md: {len(d)}   merged: {len(params)}")
    print(f"  wrote {args.out}  ({args.out.stat().st_size // 1024} KB, {len(blob) // 1024} KB unpacked)")

    if args.board_params:
        names = read_board_names(args.board_params)
        have = [n for n in names if n in params]
        print(f"  coverage of {args.board_params.name}: {len(have)}/{len(names)} "
              f"({100 * len(have) / max(1, len(names)):.1f}%)")
        missing = [n for n in names if n not in params]
        if missing:
            print(f"  not described ({len(missing)}): {', '.join(missing[:12])}"
                  f"{' ...' if len(missing) > 12 else ''}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
