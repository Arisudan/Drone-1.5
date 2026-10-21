"""
================================================================================
MODULE: video_recorder.py
PURPOSE: Record the FPV feed to a file and save snapshots (no Qt)
================================================================================

WHY THIS EXISTS:
  The FPV tab could only look at the feed. After a flight the useful questions
  are "what did the camera see when it went wrong?" and "can I keep this frame?".
  This writes the decoded frames the capture thread already produces - it opens
  no second connection to the Radxa - as an MJPEG AVI (plays everywhere, needs no
  extra codec) and single frames as PNG.

RULES:
  * Only REAL frames are recorded. The capture thread substitutes a grey
    "no signal" placeholder when the stream dies; the caller must not pass those
    in (VideoFeedWidget checks the same flag the health monitor uses), so a
    recording never fills up with placeholder frames.
  * The first real frame fixes the video size; any later frame of another size
    (the source reconnected at a different resolution) is resized to match -
    VideoWriter silently drops frames of the wrong size otherwise.
  * Files go to <DRONE_GCS_HOME>/video/ with a timestamp name.
================================================================================
"""

from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Optional

import cv2

from core.flight_log import log_dir

log = logging.getLogger(__name__)

DEFAULT_FPS = 15.0


def video_dir() -> Path:
    d = log_dir() / "video"
    d.mkdir(parents=True, exist_ok=True)
    return d


def timestamped(prefix: str, ext: str, directory: Optional[Path] = None) -> Path:
    """<dir>/<prefix>_YYYYMMDD_HHMMSS.<ext>; a counter is appended if it exists."""
    base = Path(directory) if directory else video_dir()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    p = base / f"{prefix}_{stamp}.{ext}"
    n = 1
    while p.exists():
        p = base / f"{prefix}_{stamp}_{n}.{ext}"
        n += 1
    return p


def save_snapshot(frame, directory: Optional[Path] = None) -> Optional[Path]:
    """Write one frame as PNG. Returns the path, or None if it could not be written."""
    if frame is None:
        return None
    path = timestamped("snapshot", "png", directory)
    try:
        ok = cv2.imwrite(str(path), frame)
    except cv2.error:
        ok = False
    return path if ok else None


class VideoRecorder:
    """MJPEG AVI writer. start() -> write() per frame -> stop()."""

    def __init__(self, fps: float = DEFAULT_FPS):
        self.fps = fps
        self.path: Optional[Path] = None
        self.frames = 0
        self._writer = None
        self._size = None            # (w, h) of the first frame
        self._t0 = 0.0
        self._active = False

    @property
    def recording(self) -> bool:
        return self._active

    @property
    def elapsed(self) -> float:
        return time.monotonic() - self._t0 if self.recording else 0.0

    def start(self, directory: Optional[Path] = None, fps: Optional[float] = None) -> Path:
        """Begin a recording. The file is created on the first frame (its size is
        not known until then)."""
        if self.recording:
            self.stop()
        if fps:
            self.fps = fps
        self.path = timestamped("fpv", "avi", directory)
        self.frames = 0
        self._size = None
        self._writer = None
        self._t0 = time.monotonic()
        self._active = True
        return self.path

    def write(self, frame) -> bool:
        """Add one real frame. Returns False if it was not written."""
        if not self.recording or frame is None:
            return False
        h, w = frame.shape[:2]
        if self._writer is None:
            self._size = (w, h)
            self._writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*"MJPG"),
                                           float(self.fps), self._size)
            if not self._writer.isOpened():
                log.error("could not open %s for recording", self.path)
                self._writer = None
                self.path = None
                self._t0 = 0.0
                self._active = False
                return False
        elif (w, h) != self._size:
            frame = cv2.resize(frame, self._size)
        self._writer.write(frame)
        self.frames += 1
        return True

    def stop(self) -> Optional[Path]:
        """Finish the file. Returns its path, or None if nothing was written."""
        path = self.path if self.frames else None
        if self._writer is not None:
            self._writer.release()
        self._writer = None
        self._active = False
        self.path = None
        self._size = None
        self._t0 = 0.0
        return path
