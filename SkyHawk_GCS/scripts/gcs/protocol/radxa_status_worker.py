"""
================================================================================
MODULE: protocol/radxa_status_worker.py
PURPOSE: Poll the Radxa watchdog every few seconds on its own thread
================================================================================

The fetch can block for up to its timeout, so it never runs on the UI thread. The
host is read through a callable on every poll, so a Network preset change (a new
Radxa address) is picked up without restarting anything.
================================================================================
"""

from __future__ import annotations

from typing import Callable

from PyQt5.QtCore import QThread, pyqtSignal

from core.radxa_status import fetch_status


class RadxaStatusWorker(QThread):
    # RadxaStatus when the watchdog answered, None when it did not.
    status_received = pyqtSignal(object)

    def __init__(self, host_provider: Callable[[], str], port_provider: Callable[[], int],
                 period_s: float = 5.0, parent=None):
        super().__init__(parent)
        self._host, self._port, self._period = host_provider, port_provider, period_s
        self._running = False

    def run(self) -> None:
        self._running = True
        while self._running:
            try:
                st = fetch_status(self._host(), int(self._port()))
            except Exception:                      # noqa: BLE001 - a bad setting must not kill the poller
                st = None
            if self._running:
                self.status_received.emit(st)
            # Sleep in small steps so stop() is prompt.
            waited = 0.0
            while self._running and waited < self._period:
                self.msleep(100)
                waited += 0.1

    def stop(self) -> None:
        self._running = False
        self.wait(3000)
