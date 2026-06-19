"""
cameras/rigs.py
==============
CameraRig wraps an isaacsim Camera with cinematic controls and named presets.
Replaces the ugly default isometric debug view with intentional framing.

A preset defines: position offset relative to a target, the look-at point,
focal length (controls field of view / compression), and resolution.

Presets included:
    low_three_quarter : low angle, front-three-quarter, subject walks toward cam
    orbit             : elevated orbiting feel (static placement here)
    tracking_dolly    : side-on dolly framing for a profile walk
    hero_front        : straight-on heroic low front
"""

from __future__ import annotations
import numpy as np


PRESETS = {
    "low_three_quarter": dict(offset=(2.5, -2.5, 0.8),  focal=24.0, res=(1280, 720)),
    "orbit":             dict(offset=(0.0, -4.0, 3.0),  focal=35.0, res=(1280, 720)),
    "tracking_dolly":    dict(offset=(0.0, -3.0, 1.0),  focal=50.0, res=(1280, 720)),
    "hero_front":        dict(offset=(3.5,  0.0, 0.6),  focal=28.0, res=(1280, 720)),
}


class CameraRig:
    def __init__(self, offset, focal=24.0, res=(1280, 720),
                 target=(0.0, 0.0, 1.0), prim_path="/World/CinematicCamera"):
        self.offset = np.array(offset, dtype=float)
        self.focal = focal
        self.res = res
        self.target = np.array(target, dtype=float)
        self.prim_path = prim_path
        self.sensor = None

    @classmethod
    def preset(cls, name, target=(0.0, 0.0, 1.0)):
        if name not in PRESETS:
            raise KeyError(f"Unknown camera preset '{name}'. "
                           f"Options: {list(PRESETS)}")
        p = PRESETS[name]
        return cls(offset=p["offset"], focal=p["focal"], res=p["res"],
                   target=target)

    def _look_at_quat(self, eye, target):
        """Compute an orientation quaternion so the camera looks at target."""
        import isaacsim.core.utils.numpy.rotations as rot_utils
        forward = np.array(target) - np.array(eye)
        forward = forward / (np.linalg.norm(forward) + 1e-9)
        yaw = np.degrees(np.arctan2(forward[1], forward[0]))
        pitch = np.degrees(np.arcsin(-forward[2]))
        # Camera convention: roll 0, then pitch, then yaw
        return rot_utils.euler_angles_to_quats(
            np.array([0.0, pitch, yaw]), degrees=True)

    def attach(self, world):
        """Create / place the camera sensor in the scene."""
        from isaacsim.sensors.camera import Camera
        eye = self.target + self.offset
        quat = self._look_at_quat(eye, self.target)
        self.sensor = Camera(
            prim_path=self.prim_path,
            position=eye,
            frequency=30,
            resolution=self.res,
            orientation=quat,
        )
        # Focal length controls FOV / compression for a cinematic look
        try:
            self.sensor.set_focal_length(self.focal / 10.0)
        except Exception:
            pass  # some versions set this post-initialize; safe to skip
        return self.sensor

    def initialize(self):
        if self.sensor is not None:
            self.sensor.initialize()
