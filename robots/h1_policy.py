"""
robots/h1_policy.py
===================
Wraps NVIDIA's BUILT-IN, validated H1 flat-terrain locomotion policy.

This is the key choice of the project: we do NOT train and we do NOT import a
random checkpoint. We use `H1FlatTerrainPolicy` from the
`isaacsim.robot.policy.examples` extension that ships inside Isaac Sim. The
robot asset and the policy file are already matched and validated by NVIDIA,
which sidesteps the observation-vector mismatch that breaks arbitrary
checkpoints.

The policy is velocity-commanded: we set [v_x, v_y, w_z] and the policy
produces a balanced gait that tracks it. "Walk 1m forward" therefore becomes
"command v_x for the time needed to cover 1m", computed in command().

NOTE: class/method names follow the isaacsim 5.1 example extension. If a future
Isaac Sim renames them, only this file changes.
"""

from __future__ import annotations
import inspect
import numpy as np
from robots.base_robot import BaseRobot


class H1Robot(BaseRobot):
    def __init__(self, prim_path: str = "/World/H1", position=(0.0, 0.0, 1.05),
                 orientation=None):
        super().__init__()
        self.prim_path = prim_path
        self.position = np.array(position, dtype=float)
        self.orientation = None if orientation is None else np.array(
            orientation, dtype=float)
        self._h1 = None
        self._target_velocity = np.array([0.0, 0.0, 0.0])  # [vx, vy, wz]
        self._elapsed = 0.0
        self._walk_distance = None
        self._walk_speed = 0.8  # m/s default forward speed
        self._walk_direction = np.array([1.0, 0.0, 0.0], dtype=float)

    def _parse_walk_direction(self, value):
        if isinstance(value, str):
            mapping = {
                "+x": np.array([1.0, 0.0, 0.0], dtype=float),
                "-x": np.array([-1.0, 0.0, 0.0], dtype=float),
                "+y": np.array([0.0, 1.0, 0.0], dtype=float),
                "-y": np.array([0.0, -1.0, 0.0], dtype=float),
            }
            if value not in mapping:
                raise ValueError(
                    f"Unsupported walk_dir '{value}'. Expected one of {list(mapping)}")
            return mapping[value]

        direction = np.array(value, dtype=float)
        if np.linalg.norm(direction[:2]) < 1e-9:
            raise ValueError("walk_dir must have a non-zero x/y component.")
        direction[2] = 0.0
        return direction / np.linalg.norm(direction[:2])

    # ---- high-level command interface -------------------------------------
    def command(self, **kwargs):
        """Supported commands:
            walk_forward = <meters>   -> walk that far forward then stop
            speed        = <m/s>      -> forward speed to use
            walk_dir     = +x|-x|+y|-y -> world-space heading for the walk
            velocity     = (vx,vy,wz) -> raw velocity command (advanced)
        """
        super().command(**kwargs)
        if "speed" in kwargs:
            self._walk_speed = float(kwargs["speed"])
        if "walk_dir" in kwargs:
            self._walk_direction = self._parse_walk_direction(kwargs["walk_dir"])
        if "walk_forward" in kwargs:
            self._walk_distance = float(kwargs["walk_forward"])
            self._target_velocity = self._walk_direction * self._walk_speed
        if "velocity" in kwargs:
            self._target_velocity = np.array(kwargs["velocity"], dtype=float)
            if np.linalg.norm(self._target_velocity[:2]) > 1e-9:
                self._walk_direction = self._target_velocity.copy()
                self._walk_direction[2] = 0.0
                self._walk_direction = self._walk_direction / np.linalg.norm(
                    self._walk_direction[:2])
        return self

    # ---- lifecycle --------------------------------------------------------
    def spawn(self, world):
        """Add the H1 articulation + load the validated flat-terrain policy."""
        from isaacsim.robot.policy.examples.robots.h1 import H1FlatTerrainPolicy
        spawn_kwargs = {
            "prim_path": self.prim_path,
            "name": "H1",
            "position": self.position,
        }
        signature = inspect.signature(H1FlatTerrainPolicy)
        if self.orientation is not None and "orientation" in signature.parameters:
            spawn_kwargs["orientation"] = self.orientation
        self._h1 = H1FlatTerrainPolicy(**spawn_kwargs)
        if self.orientation is not None and "orientation" not in signature.parameters:
            if hasattr(self._h1, "set_world_pose"):
                self._h1.set_world_pose(
                    position=self.position,
                    orientation=self.orientation,
                )
        return self._h1

    def on_reset(self):
        """(Re)initialize the policy after world.reset()."""
        self._elapsed = 0.0
        if self._h1 is not None:
            self._h1.initialize()
            self._h1.post_reset()

    def step(self, world):
        """One inference step. Stops when the walk distance is covered."""
        dt = world.get_physics_dt()
        self._elapsed += dt

        cmd = self._target_velocity.copy()
        if self._walk_distance is not None:
            travelled = self._walk_speed * self._elapsed
            if travelled >= self._walk_distance:
                cmd = np.array([0.0, 0.0, 0.0])  # reached target -> stop

        # The example policy's forward() takes the velocity command and the
        # physics dt, computes the observation internally, runs inference, and
        # applies joint targets to the articulation.
        self._h1.forward(dt, cmd)
