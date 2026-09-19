"""Import-path and headless setup for this project's tests.

Import this FIRST in every test module::

    import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

Two jobs:

  * Put ``scripts/gcs`` and ``scripts/diagnostics`` on ``sys.path`` so tests
    import the real modules the GCS and the diagnostics tools use, not a copy.
  * Force Qt to the ``offscreen`` platform before any PyQt import, so a test
    that touches a widget runs on a headless CI box (and over SSH) instead of
    failing with "could not connect to display".

Nothing here imports PyQt itself - the hermetic tests (health, telemetry,
path planner, map eval) must stay runnable with numpy alone.
"""

import os
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parent.parent

for _sub in ("scripts/gcs", "scripts/diagnostics"):
    _path = str(_REPO / _sub)
    if _path not in sys.path:
        sys.path.insert(0, _path)

# Must be set before any PyQt5 import anywhere in the process.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO_ROOT = _REPO
