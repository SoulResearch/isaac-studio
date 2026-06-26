"""
shots/apartment_walk.py
=======================
Default shot for the photoreal apartment scene.

It boots Isaac Sim, references `photorealistic_scenes/Apartment/scene_04.usd`,
spawns the validated H1 policy-driven robot near the apartment floor center,
pulls the camera far enough back to show the whole scene, and records an MP4
into `/isaac-sim/Documents`.

Run:
    /isaac-sim/python.sh shots/apartment_walk.py

Use `--no-robot` to render the empty apartment while keeping the same framing.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.stage import Stage


class NullRobot:
    """No-op robot used for empty-scene renders."""

    def command(self, **kwargs):
        return self

    def spawn(self, world):
        return None

    def on_reset(self):
        return None

    def step(self, world):
        return None


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    robot_group = parser.add_mutually_exclusive_group()
    robot_group.add_argument(
        "--robot",
        dest="with_robot",
        action="store_true",
        help="Include the H1 robot walking in frame (default).",
    )
    robot_group.add_argument(
        "--no-robot",
        dest="with_robot",
        action="store_false",
        help="Render the apartment empty with the same camera framing.",
    )
    parser.set_defaults(with_robot=True)
    parser.add_argument(
        "--output",
        default=None,
        help="MP4 filename written into /isaac-sim/Documents.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=8.0,
        help="Recording duration in seconds.",
    )
    parser.add_argument(
        "--walk-distance",
        type=float,
        default=1.5,
        help="Distance in meters for the H1 to walk before stopping.",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.8,
        help="Forward walking speed in meters per second.",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    stage = Stage(headless=True, enable_cameras=True)
    stage.boot()

    from environments.usd_scene import (
        DEFAULT_SCENE,
        FramedCameraRig,
        USDSceneEnvironment,
    )
    from robots.h1_policy import H1Robot
    from core.runner import Runner

    scene_path = DEFAULT_SCENE
    env = USDSceneEnvironment(scene_path=scene_path)
    robot = H1Robot(position=(0.0, 0.0, 1.05)) if args.with_robot else NullRobot()
    if args.with_robot:
        robot.command(walk_forward=args.walk_distance, speed=args.speed)

    runner = Runner(stage, env, robot, fps=30)
    runner._build()

    scene_min, scene_max = env.get_scene_bounds()
    scene_center = env.get_scene_center()
    scene_diagonal = env.get_scene_diagonal()

    print(f"[shot] scene path: {scene_path}")
    print(f"[shot] scene center: {scene_center.tolist()}")
    print(f"[shot] scene diagonal: {scene_diagonal:.3f}")
    if args.with_robot:
        print(
            f"[shot] robot spawn: {np.array(robot.position, dtype=float).tolist()}"
        )
    else:
        print("[shot] robot disabled; rendering empty apartment")

    camera = FramedCameraRig(
        center=scene_center,
        bounds_min=scene_min,
        bounds_max=scene_max,
        prim_path="/World/ApartmentCamera",
        resolution=(1600, 900),
        direction=(1.0, -1.0, 0.0),
    )

    output_filename = args.output
    if output_filename is None:
        output_filename = "apartment_walk.mp4" if args.with_robot else "apartment_empty.mp4"

    out = runner.record(camera, output_filename, duration_s=args.duration)
    print(f"[shot] DONE -> {out}")

    stage.shutdown()


if __name__ == "__main__":
    main()
