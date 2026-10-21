"""
================================================================================
MODULE: fonts.py
PURPOSE: Bundled heading fonts (Red Hat Display for the brand, Ubuntu for headings)
================================================================================

WHY THE FONTS ARE SHIPPED, NOT ASSUMED:
  Naming a font in a stylesheet only works on a machine that has it installed.
  Red Hat Display is not part of any default Linux or Windows install, so a
  stylesheet that merely says ``font-family: 'Red Hat Display'`` silently shows
  Noto Sans on most machines - including the Radxa and the Windows laptop. The
  .ttf files live in ``assets/fonts/`` (with their licences: SIL OFL for Red Hat
  Display, Ubuntu Font Licence for Ubuntu) and are registered with Qt at startup.

  ``load_bundled_fonts()`` is idempotent and is called from build_stylesheet(),
  so every entry point (GCS, Radxa monitor, tests) registers them before the
  stylesheet is applied. A missing file is logged and skipped, never fatal: the
  stylesheet's fallback chain keeps the UI readable.

WHAT USES WHAT:
  BRAND_FAMILY    - the "DRONE-GCS" title only.
  HEADING_FAMILY  - every heading: group-box titles (NAVIGATION, FLIGHT MODE),
                    each tab's section/card headings, diagnostics group headers.
  Body text, numbers, buttons, the console and the sidebar tab buttons keep their
  existing fonts.
================================================================================
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import List

log = logging.getLogger("gcs.fonts")

BRAND_FAMILY = "Red Hat Display"
HEADING_FAMILY = "Ubuntu"

# Registered in this order. Static files (not variable fonts): Qt 5 picks the
# right weight from a static family reliably.
BUNDLED = (
    "RedHatDisplay-Regular.ttf", "RedHatDisplay-Bold.ttf",
    "RedHatDisplay-ExtraBold.ttf", "RedHatDisplay-Black.ttf",
    "Ubuntu-Regular.ttf", "Ubuntu-Medium.ttf", "Ubuntu-Bold.ttf",
)

_loaded: List[str] = []
_done = False


def fonts_dir() -> Path:
    """``assets/fonts`` in the checkout this module lives in (ui -> gcs ->
    scripts -> package root)."""
    return Path(__file__).resolve().parents[3] / "assets" / "fonts"


def load_bundled_fonts() -> List[str]:
    """Register the bundled fonts with Qt. Returns the families now available
    from them. Safe to call repeatedly and before a QApplication exists (it then
    does nothing and tries again next time)."""
    global _done
    if _done:
        return list(_loaded)
    try:
        from PyQt5.QtGui import QFontDatabase, QGuiApplication
    except Exception:                       # no Qt at all: nothing to register
        return []
    if QGuiApplication.instance() is None:
        return []                           # QFontDatabase needs an app instance

    base = fonts_dir()
    families: List[str] = []
    for name in BUNDLED:
        path = base / name
        if not path.is_file():
            log.warning("bundled font missing: %s", path)
            continue
        fid = QFontDatabase.addApplicationFont(str(path))
        if fid < 0:
            log.warning("Qt could not load font %s", path)
            continue
        for fam in QFontDatabase.applicationFontFamilies(fid):
            if fam not in families:
                families.append(fam)
    _loaded[:] = families
    _done = True
    return list(_loaded)


def resolved_family(family: str, weight: int = 50) -> str:
    """The family Qt would actually render ``family`` with - used by tests to
    prove a heading really got the font and not a silent fallback."""
    from PyQt5.QtGui import QFont, QFontInfo
    f = QFont(family)
    f.setWeight(weight)
    return QFontInfo(f).family()
