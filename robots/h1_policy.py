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
import numpy as np
from robots.base_robot import BaseRobot


class H1Robot(BaseRobot):
    def __init__(self, prim_path: str = "/World/H1", position=(0.0, 0.0, 1.05)):
        super().__init__()
        self.prim_path = prim_path
        self.position = np.array(position, dtype=float)
        self._h1 = None
        self._target_velocity = np.array([0.0, 0.0, 0.0])  # [vx, vy, wz]
        self._elapsed = 0.0
        self._walk_distance = None
        self._walk_speed = 0.8  # m/s default forward speed

    # ---- high-level command interface -------------------------------------
    def command(self, **kwargs):
        """Supported commands:
            walk_forward = <meters>   -> walk that far forward then stop
            speed        = <m/s>      -> forward speed to use
            velocity     = (vx,vy,wz) -> raw velocity command (advanced)
        """
        super().command(**kwargs)
        if "speed" in kwargs:
            self._walk_speed = float(kwargs["speed"])
        if "walk_forward" in kwargs:
            self._walk_distance = float(kwargs["walk_forward"])
            self._target_velocity = np.array([self._walk_speed, 0.0, 0.0])
        if "velocity" in kwargs:
            self._target_velocity = np.array(kwargs["velocity"], dtype=float)
        return self

    # ---- lifecycle --------------------------------------------------------
    def spawn(self, world):
        """Add the H1 articulation + load the validated flat-terrain policy."""
        from isaacsim.robot.policy.examples.robots.h1 import H1FlatTerrainPolicy
        self._h1 = H1FlatTerrainPolicy(
            prim_path=self.prim_path,
            name="H1",
            position=self.position,
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
