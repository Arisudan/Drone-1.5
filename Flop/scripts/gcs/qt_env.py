"""
================================================================================
MODULE: qt_env.py
PURPOSE: Point Qt at the system platform plugins - only when that is correct
================================================================================

OpenCV's pip wheels ship their own Qt plugins and set QT_QPA_PLATFORM_PLUGIN_PATH
to them, which breaks the system PyQt5 the station runs on. The fix is to pin
the path back to the distro's plugin directory.

That fix is only right for the distro PyQt5, though. A pip-installed PyQt5 (a
venv, or the CI runner) bundles its own Qt, and handing it the system plugins
aborts the process: "Could not load the Qt platform plugin ...". So the path is
pinned only when the PyQt5 that will be imported lives outside site-packages.
================================================================================
"""

import importlib.util
import os
import sys

_SYSTEM_PLUGIN_DIRS = ("/usr/lib/aarch64-linux-gnu/qt5/plugins",
                       "/usr/lib/x86_64-linux-gnu/qt5/plugins")


def _pyqt5_is_system() -> bool:
    try:
        spec = importlib.util.find_spec("PyQt5")
    except (ImportError, ValueError):
        return False
    origin = (spec.origin or "") if spec else ""
    if not origin and spec and spec.submodule_search_locations:
        origin = next(iter(spec.submodule_search_locations), "")
    # Debian/Ubuntu's python3-pyqt5 installs to dist-packages; pip's wheel
    # (which bundles Qt under PyQt5/Qt5) lands in site-packages.
    return "dist-packages" in origin and not os.path.isdir(
        os.path.join(os.path.dirname(origin), "Qt5"))


def pin_system_qt_plugins() -> None:
    """Set QT_QPA_PLATFORM_PLUGIN_PATH to the distro plugins if appropriate."""
    if not sys.platform.startswith("linux") or not _pyqt5_is_system():
        return
    for path in _SYSTEM_PLUGIN_DIRS:
        if os.path.exists(path):
            os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = path
            return
