"""
================================================================================
MODULE: audio.py
PURPOSE: Spoken and Tonal Operator Alerts (non-blocking, degrades to silence)
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Core Data Layer)
  * Upstream:      drone_gcs.py - arm/disarm, mode changes, ACK rejections,
                   battery thresholds, link loss, VIO loss, collision holds,
                   worker stalls, and STATUSTEXT at severity <= ERROR
  * Downstream:    The operator's ears. Nothing in this module can command the
                   aircraft, and nothing downstream of it reads its output.

WHY THIS EXISTS:
  Every warning this station produces is visual. During a real indoor flight the
  operator is watching the aircraft, not the screen - which is precisely when
  "battery critical" and "vision lost" need to arrive. A red pill in a header
  nobody is looking at is not an alert.

IT MUST NEVER BLOCK THE GUI THREAD:
  Playback is a subprocess, and a subprocess on a machine with a wedged audio
  device can hang for seconds. All of it happens on one daemon worker thread fed
  by a bounded queue, with a hard timeout on every process. The GUI thread's
  only contact with audio is putting a small object on a queue. If the queue is
  full the alert is dropped rather than waited on - an operator who is already
  eight alerts behind gains nothing from the ninth.

BACKENDS ARE DETECTED, NOT ASSUMED:
  PyQt5.QtMultimedia is absent on this platform (the Qt5 multimedia package is
  not part of the ROS 2 Jazzy desktop install), so this does not use it. Tones
  are synthesised to WAV once with numpy and played through `aplay`; speech goes
  through `spd-say`. Where neither binary exists - CI, a headless Radxa, a
  container - the whole module becomes a silent no-op that still accepts every
  call. An alert system that raises on a machine without a sound card would take
  down the ground station over a speaker.

DE-DUPLICATION IS THE POINT, NOT AN OPTIMISATION:
  PX4 re-runs its preflight checks every ~2 s. Without a per-event floor, one
  unresolved fault becomes a continuous alarm, and a continuous alarm is one the
  operator mutes - losing every other alert with it.

USAGE:
  alerts = AudioAlerts(settings.audio)
  alerts.say("Battery critical", Severity.CRITICAL, key="batt_crit")
  alerts.alert(Severity.WARN, key="cmd_rejected")
  alerts.set_muted(True)
  alerts.shutdown()
================================================================================
"""

from __future__ import annotations

import logging
import math
import os
import queue
import shutil
import struct
import subprocess
import threading
import time
import wave
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
from typing import Dict, List, Optional, Tuple

log = logging.getLogger("gcs.audio")

SAMPLE_RATE = 22050
# Long enough to hang means the device is wedged; the alert is already stale by
# then and the worker is better off dropping it and staying responsive.
PLAYBACK_TIMEOUT_S = 6.0
QUEUE_LIMIT = 24


class Severity(IntEnum):
    """Ordered by urgency, and mapped from MAVLink severities by from_mavlink().

    Deliberately coarser than MAVLink's eight levels: an operator can reliably
    distinguish about four alert sounds under stress, and a fifth tone that
    means "slightly worse than the fourth" is one nobody learns.
    """
    INFO = 0
    OK = 1
    WARN = 2
    CRITICAL = 3
    ALARM = 4

    @staticmethod
    def from_mavlink(severity: int) -> "Severity":
        # MAV_SEVERITY: 0 EMERGENCY, 1 ALERT, 2 CRITICAL, 3 ERROR,
        #               4 WARNING, 5 NOTICE, 6 INFO, 7 DEBUG
        if severity <= 1:
            return Severity.ALARM
        if severity <= 3:
            return Severity.CRITICAL
        if severity == 4:
            return Severity.WARN
        return Severity.INFO


# Tone design: each severity is distinguishable with the screen not in view.
# (frequency Hz, duration s, gap-after s) repeated `repeats` times.
#   info     - one soft mid blip, easy to ignore
#   ok       - rising two-tone, the only ascending figure in the set
#   warn     - two flat mid beeps
#   critical - three urgent descending beeps
#   alarm    - fast alternating klaxon, unmistakable and unlike the rest
_TONE_SPECS: Dict[Severity, Tuple[List[Tuple[float, float, float]], float]] = {
    Severity.INFO:     ([(880.0, 0.08, 0.0)], 0.30),
    Severity.OK:       ([(660.0, 0.09, 0.02), (990.0, 0.13, 0.0)], 0.38),
    Severity.WARN:     ([(740.0, 0.11, 0.07), (740.0, 0.11, 0.0)], 0.45),
    Severity.CRITICAL: ([(1180.0, 0.10, 0.05), (940.0, 0.10, 0.05),
                         (700.0, 0.16, 0.0)], 0.55),
    Severity.ALARM:    ([(1320.0, 0.09, 0.03), (880.0, 0.09, 0.03),
                         (1320.0, 0.09, 0.03), (880.0, 0.09, 0.03),
                         (1320.0, 0.13, 0.0)], 0.60),
}


@dataclass
class _Cue:
    severity: Severity
    text: str = ""
    tone: bool = True


# ─── Tone synthesis ─────────────────────────────────────────────────

def _render_tone(spec: List[Tuple[float, float, float]],
                 amplitude: float = 0.35) -> bytes:
    """16-bit mono PCM for one tone sequence.

    Each segment is enveloped with a 6 ms raised-cosine fade at both ends. Not
    cosmetic: a square-edged sine starts and stops with a step discontinuity,
    which on small laptop speakers is an audible click loud enough to mask the
    tone it brackets.
    """
    frames = bytearray()
    fade_s = 0.006
    for freq, duration, gap in spec:
        n = int(SAMPLE_RATE * duration)
        fade_n = max(1, min(n // 2, int(SAMPLE_RATE * fade_s)))
        for i in range(n):
            env = 1.0
            if i < fade_n:
                env = 0.5 * (1.0 - math.cos(math.pi * i / fade_n))
            elif i > n - fade_n:
                j = n - i
                env = 0.5 * (1.0 - math.cos(math.pi * j / fade_n))
            sample = amplitude * env * math.sin(2.0 * math.pi * freq * i / SAMPLE_RATE)
            frames += struct.pack("<h", int(max(-1.0, min(1.0, sample)) * 32767))
        frames += b"\x00\x00" * int(SAMPLE_RATE * gap)
    return bytes(frames)


def write_tone_files(directory: Path) -> Dict[Severity, Path]:
    """Synthesise every tone to WAV once, returning severity -> path.

    Written to disk rather than piped to the player on each alert: `aplay` wants
    a file or a correctly-framed stream, and re-rendering a half-second of audio
    for every battery warning is work this station does not need to repeat.
    Existing non-empty files are reused, so a restart costs nothing.
    """
    directory.mkdir(parents=True, exist_ok=True)
    out: Dict[Severity, Path] = {}
    for severity, (spec, _floor) in _TONE_SPECS.items():
        path = directory / f"{severity.name.lower()}.wav"
        if not path.exists() or path.stat().st_size == 0:
            pcm = _render_tone(spec)
            with wave.open(str(path), "wb") as fh:
                fh.setnchannels(1)
                fh.setsampwidth(2)
                fh.setframerate(SAMPLE_RATE)
                fh.writeframes(pcm)
        out[severity] = path
    return out


# ─── Backend discovery ──────────────────────────────────────────────

def find_tone_player() -> Optional[List[str]]:
    """Argv prefix for a WAV player, or None. The file path is appended."""
    if shutil.which("aplay"):
        return ["aplay", "-q"]
    if shutil.which("paplay"):
        return ["paplay"]
    if shutil.which("ffplay"):
        return ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet"]
    return None


def find_speech_player() -> Optional[List[str]]:
    """Argv prefix for a text-to-speech command, or None. Text is appended.

    `-w` on spd-say waits for the utterance to finish. Without it, two alerts
    arriving together talk over each other and neither is intelligible - the
    queue is what serialises them, and it can only do that if the process
    actually represents the duration of the speech.
    """
    if shutil.which("spd-say"):
        return ["spd-say", "-w"]
    if shutil.which("espeak-ng"):
        return ["espeak-ng"]
    if shutil.which("espeak"):
        return ["espeak"]
    return None


# ─── The alert service ──────────────────────────────────────────────

class AudioAlerts:
    """Queued, rate-limited, non-blocking audio alerts.

    Safe to construct with no audio hardware, no settings object, and no
    `sound_dir` write permission: every failure path ends in silence, logged
    once at debug level, with the public API still fully callable.
    """

    def __init__(self, config=None, sound_dir: Optional[Path] = None,
                 enabled: Optional[bool] = None):
        self._cfg = config
        self._muted = False
        self._lock = threading.Lock()
        self._last_played: Dict[str, float] = {}
        self._queue: "queue.Queue[Optional[_Cue]]" = queue.Queue(maxsize=QUEUE_LIMIT)
        self._thread: Optional[threading.Thread] = None
        self._stopping = threading.Event()

        self._tone_cmd = find_tone_player()
        self._speech_cmd = find_speech_player()
        self._tones: Dict[Severity, Path] = {}

        want = self._cfg_bool("enabled", True) if enabled is None else enabled
        self._available = bool(want) and bool(self._tone_cmd or self._speech_cmd)

        if self._available and self._tone_cmd:
            base = sound_dir or (Path(os.environ.get(
                "DRONE_GCS_HOME", str(Path.home() / ".drone_gcs"))) / "sounds")
            try:
                self._tones = write_tone_files(base)
            except OSError:
                log.debug("could not write tone files to %s", base, exc_info=True)
                self._tones = {}

        if self._available:
            self._thread = threading.Thread(
                target=self._run, name="gcs-audio", daemon=True)
            self._thread.start()
        else:
            log.info("audio alerts disabled (no aplay/spd-say, or turned off)")

    # ── configuration ───────────────────────────────────────────────

    def _cfg_bool(self, name: str, default: bool) -> bool:
        return bool(getattr(self._cfg, name, default)) if self._cfg else default

    def _cfg_float(self, name: str, default: float) -> float:
        try:
            return float(getattr(self._cfg, name, default)) if self._cfg else default
        except (TypeError, ValueError):
            return default

    def set_config(self, config) -> None:
        """Adopt a newly saved settings object without a restart."""
        self._cfg = config

    @property
    def available(self) -> bool:
        """True when something could actually be heard. False is a normal,
        supported state - callers must not branch on it to decide whether to
        report an event some other way; they should already be doing that."""
        return self._available

    @property
    def muted(self) -> bool:
        return self._muted

    def set_muted(self, muted: bool) -> bool:
        self._muted = bool(muted)
        if self._muted:
            self._drain()
        return self._muted

    def toggle_muted(self) -> bool:
        return self.set_muted(not self._muted)

    def backend_description(self) -> str:
        """One line for the diagnostics/config UI."""
        if not self._available:
            return "unavailable (no audio backend)"
        parts = []
        if self._tone_cmd and self._tones:
            parts.append(self._tone_cmd[0])
        if self._speech_cmd:
            parts.append(self._speech_cmd[0])
        return " + ".join(parts) if parts else "unavailable"

    # ── public API ──────────────────────────────────────────────────

    def alert(self, severity: Severity, key: Optional[str] = None) -> bool:
        """Play the tone for `severity`. Returns True if it was queued."""
        return self._submit(_Cue(severity=severity, text="", tone=True), key)

    def say(self, text: str, severity: Severity = Severity.INFO,
            key: Optional[str] = None, tone: bool = True) -> bool:
        """Speak `text`, optionally preceded by the severity tone.

        The tone leads the speech on purpose: it arrives in under a tenth of a
        second and tells the operator how bad it is, while the words take a
        second or more to say. Turning to the screen on the tone and hearing
        the detail on the way is the intended behaviour.
        """
        return self._submit(_Cue(severity=severity, text=text, tone=tone), key)

    def shutdown(self, timeout: float = 1.5) -> None:
        """Stop the worker. Idempotent, and safe to call from closeEvent."""
        if self._thread is None:
            return
        self._stopping.set()
        self._drain()
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            pass
        self._thread.join(timeout=timeout)
        self._thread = None

    # ── internals ───────────────────────────────────────────────────

    def _should_play(self, key: Optional[str]) -> bool:
        """Per-key rate limit. An un-keyed alert is always allowed: the caller
        has told us this event is distinct, and inventing a key from the text
        would silently collapse two genuinely different messages."""
        if key is None:
            return True
        floor = self._cfg_float("min_repeat_s", 8.0)
        if floor <= 0:
            return True
        now = time.monotonic()
        with self._lock:
            last = self._last_played.get(key, 0.0)
            if now - last < floor:
                return False
            self._last_played[key] = now
        return True

    def _submit(self, cue: _Cue, key: Optional[str]) -> bool:
        if not self._available or self._muted:
            return False
        if not self._cfg_bool("enabled", True):
            return False
        if not self._should_play(key):
            return False
        try:
            self._queue.put_nowait(cue)
        except queue.Full:
            # Backed up behind a slow device. Dropping the newest keeps the
            # already-queued (older, and so more likely still relevant to the
            # thing that just happened) alerts intact.
            log.debug("audio queue full, dropped %s", cue.severity.name)
            return False
        return True

    def _drain(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                return

    def _run(self) -> None:
        while not self._stopping.is_set():
            try:
                cue = self._queue.get(timeout=0.25)
            except queue.Empty:
                continue
            if cue is None:
                return
            if self._muted:
                continue
            try:
                self._play(cue)
            except Exception:
                # A failing speaker must never take the worker down with it;
                # the next alert deserves an attempt.
                log.debug("audio playback failed", exc_info=True)

    def _play(self, cue: _Cue) -> None:
        if cue.tone and self._cfg_bool("tones_enabled", True):
            path = self._tones.get(cue.severity)
            if path is not None and self._tone_cmd:
                self._spawn(self._tone_cmd + [str(path)])
        if cue.text and self._cfg_bool("speech_enabled", True) and self._speech_cmd:
            self._spawn(self._speech_cmd + [cue.text])

    @staticmethod
    def _spawn(argv: List[str]) -> None:
        try:
            subprocess.run(argv, timeout=PLAYBACK_TIMEOUT_S,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           check=False)
        except (subprocess.TimeoutExpired, OSError):
            log.debug("audio command failed: %s", " ".join(argv), exc_info=True)


class NullAudioAlerts(AudioAlerts):
    """Explicitly silent. For tests and for `--no-audio`, so the call sites stay
    identical rather than growing `if self.audio is not None` everywhere."""

    def __init__(self):
        super().__init__(config=None, enabled=False)
