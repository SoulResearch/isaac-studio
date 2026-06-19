"""
core/recorder.py
================
Captures frames from a camera sensor and writes them to an MP4 in the
mounted output folder (/isaac-sim/Documents -> host ~/docker/isaac-sim/data).

Designed so a single SimulationApp boot can record multiple camera angles
(see runner.py), avoiding the ~2 min boot cost per angle.
"""

from __future__ import annotations
import os
import numpy as np


# Default output directory is the volume-mounted folder that bridges
# container -> Brev host. Override via OUTPUT_DIR env var if needed.
DEFAULT_OUTPUT_DIR = os.environ.get("OUTPUT_DIR", "/isaac-sim/Documents")


class Recorder:
    """Collects RGB frames during a sim run and encodes them to MP4."""

    def __init__(self, fps: int = 30, output_dir: str = DEFAULT_OUTPUT_DIR):
        self.fps = fps
        self.output_dir = output_dir
        self._frames: list[np.ndarray] = []
        os.makedirs(self.output_dir, exist_ok=True)

    def capture(self, camera) -> bool:
        """Grab one RGB frame from an isaacsim Camera. Returns True if a
        valid frame was captured."""
        rgba = camera.get_rgba()
        if rgba is not None and rgba.size > 0:
            self._frames.append(rgba[:, :, :3].astype(np.uint8))
            return True
        return False

    def frame_count(self) -> int:
        return len(self._frames)

    def reset(self):
        """Clear captured frames (e.g. between camera angles)."""
        self._frames = []

    def write(self, filename: str) -> str:
        """Encode collected frames to an MP4 and return the full path."""
        import imageio.v2 as imageio
        if not self._frames:
            raise RuntimeError("No frames captured; nothing to write.")
        if not filename.endswith(".mp4"):
            filename += ".mp4"
        out_path = os.path.join(self.output_dir, filename)
        imageio.mimsave(out_path, self._frames, fps=self.fps,
                        codec="libx264", quality=8)
        return out_path
