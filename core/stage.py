"""
core/stage.py
=============
Owns the Isaac Sim application lifecycle. EVERYTHING that touches Isaac Sim
must happen after Stage.boot() is called, because SimulationApp is what loads
the USD / pxr / isaaclab modules onto the path.

Usage:
    from core.stage import Stage
    stage = Stage(headless=True)
    stage.boot()          # boots SimulationApp; AFTER this, import sim modules
    ... build scene, run, record ...
    stage.shutdown()
"""

from __future__ import annotations


class Stage:
    """Wraps SimulationApp boot/shutdown. One per process."""

    def __init__(self, headless: bool = True, enable_cameras: bool = True,
                 width: int = 1280, height: int = 720):
        self.headless = headless
        self.enable_cameras = enable_cameras
        self.width = width
        self.height = height
        self._app = None
        self._booted = False

    def boot(self):
        """Boot SimulationApp. Must be called before any isaac/usd imports."""
        if self._booted:
            return self._app
        from isaacsim import SimulationApp
        self._app = SimulationApp({
            "headless": self.headless,
            "enable_cameras": self.enable_cameras,
            "width": self.width,
            "height": self.height,
        })
        self._booted = True
        return self._app

    @property
    def app(self):
        if not self._booted:
            raise RuntimeError("Stage.boot() must be called before accessing .app")
        return self._app

    def is_running(self) -> bool:
        return self._booted and self._app is not None

    def shutdown(self):
        """Cleanly close SimulationApp. The shutdown segfault some containers
        emit is harmless and happens after all real work is done."""
        if self._booted and self._app is not None:
            self._app.close()
            self._booted = False
