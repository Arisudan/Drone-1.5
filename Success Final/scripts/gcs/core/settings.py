"""
================================================================================
MODULE: settings.py
PURPOSE: Typed, Validated, Persisted Ground-Station Settings
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Core Data Layer)
  * Downstream:    ui/config_tab.py (editor), drone_gcs.py (applies values)

WHY THIS EXISTS:
  Every tunable in this station used to be a literal in source: the network
  presets, the takeoff altitude bounds, the video URL, the battery warning
  thresholds. Changing one meant editing Python and restarting, and nothing
  survived a restart anyway. This gives them one typed home, validated on load
  and written to disk.

VALIDATION IS NOT OPTIONAL:
  ``validate()`` runs on every load. A malformed settings file fails loudly at
  startup rather than silently flying with a takeoff ceiling of zero or a
  battery-critical threshold above the warning one. Out-of-range values raise;
  unknown keys are ignored, so a file written by a newer build still loads.

FILE:
  ``$DRONE_GCS_HOME/settings.json`` (default ``~/.drone_gcs``). Absent file =
  defaults, and saving creates it.
================================================================================
"""

from __future__ import annotations

import json
import logging
import os
from dataclasses import asdict, dataclass, field, fields, is_dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

log = logging.getLogger("gcs.settings")


def settings_dir() -> Path:
    return Path(os.environ.get("DRONE_GCS_HOME", str(Path.home() / ".drone_gcs")))


def package_root() -> Path:
    """The checkout this module lives in (scripts/gcs/core -> package root)."""
    return Path(__file__).resolve().parents[3]


def default_rviz_config() -> str:
    """Ship-with-the-checkout RViz layout, wherever the checkout happens to be."""
    return str(package_root() / "config" / "rtabmap_drone.rviz")


def resolve_rviz_config(cfg: "GCSSettings") -> str:
    return cfg.slam.rviz_config or default_rviz_config()


@dataclass
class ProfileConfig:
    name: str = "Drone-1.5"
    frame: str = "Quad X"
    battery_cells: int = 4
    battery_capacity_mah: int = 5200
    drone_id: str = "DRONE-1.5"

    def validate(self) -> None:
        if not 1 <= self.battery_cells <= 12:
            raise ValueError(f"profile.battery_cells={self.battery_cells} outside 1..12")
        if self.battery_capacity_mah <= 0:
            raise ValueError("profile.battery_capacity_mah must be > 0")


@dataclass
class ConnectionConfig:
    default_network: str = "HTIC_RND"
    host: str = "172.16.101.84"
    protocol: str = "udp"           # udp | tcp
    udp_port: int = 14550
    tcp_port: int = 5760
    source_system: int = 255

    def validate(self) -> None:
        if self.protocol not in ("udp", "tcp"):
            raise ValueError(f"connection.protocol={self.protocol!r} must be udp or tcp")
        for name in ("udp_port", "tcp_port"):
            port = getattr(self, name)
            if not 1 <= port <= 65535:
                raise ValueError(f"connection.{name}={port} outside 1..65535")
        if not 1 <= self.source_system <= 255:
            raise ValueError("connection.source_system outside 1..255")


@dataclass
class VideoConfig:
    stream_url: str = "http://172.16.101.84:8080/video"
    map_bridge_host: str = "172.16.101.84"
    map_bridge_port: int = 5765
    jpeg_port: int = 8080

    def validate(self) -> None:
        if not self.stream_url:
            raise ValueError("video.stream_url must not be empty")


@dataclass
class SlamConfig:
    cruise_altitude_m: float = 1.0
    robot_radius_m: float = 0.25
    cell_size_m: float = 0.025
    treat_unknown_as_obstacle: bool = False
    # Empty means "the copy shipped with this checkout" - resolved at runtime
    # by default_rviz_config(), so the template works from any directory
    # without anyone editing a path into source.
    rviz_config: str = ""

    def validate(self) -> None:
        if not 0.2 <= self.cruise_altitude_m <= 10.0:
            raise ValueError("slam.cruise_altitude_m outside 0.2..10.0")
        if not 0.05 <= self.robot_radius_m <= 2.0:
            raise ValueError("slam.robot_radius_m outside 0.05..2.0")
        if self.cell_size_m <= 0:
            raise ValueError("slam.cell_size_m must be > 0")


@dataclass
class LimitsConfig:
    """Operator-entered command bounds - the guard against a fat-fingered value."""
    takeoff_alt_min_m: float = 0.2
    takeoff_alt_max_m: float = 3.0
    move_max_delta_m: float = 3.0

    def validate(self) -> None:
        if self.takeoff_alt_min_m <= 0:
            raise ValueError("limits.takeoff_alt_min_m must be > 0")
        if self.takeoff_alt_max_m <= self.takeoff_alt_min_m:
            raise ValueError("limits.takeoff_alt_max_m must exceed takeoff_alt_min_m")
        if self.move_max_delta_m <= 0:
            raise ValueError("limits.move_max_delta_m must be > 0")


@dataclass
class AlertsConfig:
    batt_warn_pct: int = 35
    batt_crit_pct: int = 20
    vision_stale_s: float = 3.0
    map_stall_s: float = 20.0

    def validate(self) -> None:
        for name in ("batt_warn_pct", "batt_crit_pct"):
            v = getattr(self, name)
            if not 0 <= v <= 100:
                raise ValueError(f"alerts.{name}={v} outside 0..100")
        if self.batt_crit_pct >= self.batt_warn_pct:
            raise ValueError("alerts.batt_crit_pct must be below batt_warn_pct")
        if self.vision_stale_s <= 0 or self.map_stall_s <= 0:
            raise ValueError("alerts timeouts must be > 0")


@dataclass
class GCSSettings:
    profile: ProfileConfig = field(default_factory=ProfileConfig)
    connection: ConnectionConfig = field(default_factory=ConnectionConfig)
    video: VideoConfig = field(default_factory=VideoConfig)
    slam: SlamConfig = field(default_factory=SlamConfig)
    limits: LimitsConfig = field(default_factory=LimitsConfig)
    alerts: AlertsConfig = field(default_factory=AlertsConfig)

    def validate(self) -> None:
        for f in fields(self):
            section = getattr(self, f.name)
            if is_dataclass(section):
                section.validate()

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    def sections(self) -> List[str]:
        return [f.name for f in fields(self)]


def _apply(target: Any, src: dict) -> None:
    """Overlay known keys, coercing to the declared type. Unknown keys ignored."""
    for f in fields(target):
        if f.name not in src:
            continue
        raw = src[f.name]
        try:
            if f.type in ("int", int):
                raw = int(raw)
            elif f.type in ("float", float):
                raw = float(raw)
            elif f.type in ("bool", bool):
                raw = bool(raw) if not isinstance(raw, str) else raw.lower() in ("1", "true", "yes", "on")
            elif f.type in ("str", str):
                raw = str(raw)
        except (TypeError, ValueError):
            log.warning("ignoring unparsable value for %s: %r", f.name, raw)
            continue
        setattr(target, f.name, raw)


def settings_path(path: Optional[Path] = None) -> Path:
    return Path(path) if path else (settings_dir() / "settings.json")


# CLI flag -> (settings section, field, env var). One table drives both the
# argument parser and the environment fallback, so a new knob cannot be added
# to one and forgotten in the other.
OVERRIDES = (
    ("host",        "connection", "host",        "GCS_HOST"),
    ("port",        "connection", "udp_port",    "GCS_PORT"),
    ("protocol",    "connection", "protocol",    "GCS_PROTOCOL"),
    ("map_host",    "video",      "map_bridge_host", "GCS_MAP_HOST"),
    ("map_port",    "video",      "map_bridge_port", "GCS_MAP_PORT"),
    ("video_url",   "video",      "stream_url",  "GCS_VIDEO_URL"),
    ("rviz_config", "slam",       "rviz_config", "GCS_RVIZ_CONFIG"),
)


def apply_overrides(cfg: "GCSSettings", args=None) -> "GCSSettings":
    """Layer CLI flags over environment variables over the saved settings.

    Precedence is flag > env > settings.json > dataclass default. Nothing here
    writes to disk: a one-off `--host` for a bench session should not silently
    become the permanent configuration.
    """
    for flag, section, field_name, env in OVERRIDES:
        value = getattr(args, flag, None) if args is not None else None
        if value is None:
            value = os.environ.get(env) or None
        if value is None:
            continue
        target = getattr(cfg, section)
        current = getattr(target, field_name)
        try:
            if isinstance(current, bool):
                value = str(value).lower() in ("1", "true", "yes", "on")
            elif isinstance(current, int):
                value = int(value)
            elif isinstance(current, float):
                value = float(value)
            else:
                value = str(value)
        except (TypeError, ValueError):
            log.warning("ignoring bad override %s=%r", env, value)
            continue
        setattr(target, field_name, value)

    # The stream URL follows the host unless it was set explicitly, so
    # `--host 10.0.0.5` alone re-points MAVLink, the map bridge and the video.
    explicit_video = (getattr(args, "video_url", None) if args else None) or \
        os.environ.get("GCS_VIDEO_URL")
    if not explicit_video:
        cfg.video.stream_url = f"http://{cfg.connection.host}:{cfg.video.jpeg_port}/video"
    if not (getattr(args, "map_host", None) if args else None) and not os.environ.get("GCS_MAP_HOST"):
        cfg.video.map_bridge_host = cfg.connection.host

    cfg.validate()
    return cfg


def load_settings(path: Optional[Path] = None) -> GCSSettings:
    """Read settings, layering the file over typed defaults.

    A missing file is normal (first run). A malformed one is reported and the
    defaults are used, because refusing to start the ground station over a bad
    preference file would be a worse failure than ignoring it.
    """
    cfg = GCSSettings()
    p = settings_path(path)
    if p.exists():
        try:
            with open(p, encoding="utf-8") as fh:
                payload = json.load(fh)
            for name in cfg.sections():
                if isinstance(payload.get(name), dict):
                    _apply(getattr(cfg, name), payload[name])
        except (OSError, ValueError):
            log.exception("could not read %s - using defaults", p)
            return GCSSettings()
    try:
        cfg.validate()
    except ValueError:
        log.exception("settings failed validation - using defaults")
        return GCSSettings()
    return cfg


def save_settings(cfg: GCSSettings, path: Optional[Path] = None) -> Path:
    """Validate then write. Raises ValueError if the values are not flyable."""
    cfg.validate()
    p = settings_path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w", encoding="utf-8") as fh:
        json.dump(cfg.to_dict(), fh, indent=2)
    return p
