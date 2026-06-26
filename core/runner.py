"""
core/runner.py
==============
Orchestrates a single "shot": build environment, spawn robot+policy, place
camera(s), step the sim while driving the policy, capture frames, write MP4(s).

Key efficiency feature: render_multi() records several camera angles from ONE
SimulationApp boot, so iterating on cinematography doesn't cost a 2-min boot
per angle.
"""

from __future__ import annotations
from core.recorder import Recorder


class Runner:
    """Drives one shot from a booted Stage."""

    def __init__(self, stage, environment, robot, fps: int = 30):
        self.stage = stage
        self.environment = environment
        self.robot = robot
        self.fps = fps
        self._world = None

    def _build(self):
        """Construct the world, environment, and robot. Called once."""
        from isaacsim.core.api import World
        self._world = World(stage_units_in_meters=1.0)
        # Environment adds ground + lighting + props
        self.environment.build(self._world)
        # Some environments can suggest a better spawn point after they know
        # their world-space bounds.
        get_robot_spawn_position = getattr(
            self.environment, "get_robot_spawn_position", None)
        if callable(get_robot_spawn_position):
            spawn_position = get_robot_spawn_position()
            if spawn_position is not None and hasattr(self.robot, "position"):
                self.robot.position = spawn_position
        # Robot adds its articulation + loads its policy
        self.robot.spawn(self._world)
        return self._world

    def _warmup(self, steps: int = 10):
        for _ in range(steps):
            self._world.step(render=True)

    def record(self, camera, filename: str, duration_s: float = 8.0) -> str:
        """Single-camera shot. Builds, runs, writes one MP4."""
        results = self.render_multi([(camera, filename)], duration_s=duration_s)
        return results[0]

    def render_multi(self, camera_jobs, duration_s: float = 8.0) -> list[str]:
        """Record multiple (camera, filename) jobs.

        Strategy: build once, then for each camera re-run the policy from a
        reset so every angle shows the same motion. This trades a little extra
        sim time for not paying the SimulationApp boot cost per angle.
        """
        if self._world is None:
            self._build()

        n_steps = int(duration_s * self.fps)
        outputs: list[str] = []

        for camera, filename in camera_jobs:
            camera.attach(self._world)            # place camera in the scene
            self._world.reset()
            self.robot.on_reset()                 # re-init policy state
            camera.initialize()
            self._warmup(steps=10)

            recorder = Recorder(fps=self.fps)
            for i in range(n_steps):
                self.robot.step(self._world)      # policy inference -> joint targets
                self._world.step(render=True)
                recorder.capture(camera.sensor)
            path = recorder.write(filename)
            outputs.append(path)
            print(f"[runner] wrote {path} ({recorder.frame_count()} frames)")

        return outputs
