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


WALK_DIR_VECTORS = {
    "+x": np.array([1.0, 0.0, 0.0], dtype=float),
    "-x": np.array([-1.0, 0.0, 0.0], dtype=float),
    "+y": np.array([0.0, 1.0, 0.0], dtype=float),
    "-y": np.array([0.0, -1.0, 0.0], dtype=float),
}

FOCUS_PRESETS = {
    "living_room": {
        "camera_position": np.array([-2.5, -1.5, 1.6], dtype=float),
        "camera_target": np.array([-2.5, 2.0, 1.0], dtype=float),
        "fov_degrees": 60.0,
    },
    "room_center": {
        "camera_position": np.array([0.5, -2.0, 1.7], dtype=float),
        "camera_target": np.array([0.5, 2.0, 1.0], dtype=float),
        "fov_degrees": 60.0,
    },
}


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
    parser.add_argument(
        "--inspect",
        action="store_true",
        help="Load the scene, print hierarchy/bounds diagnostics, and exit.",
    )
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
        "--walk-dir",
        choices=sorted(WALK_DIR_VECTORS),
        default="+y",
        help="World-space walk heading used by the robot and interior camera.",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.8,
        help="Forward walking speed in meters per second.",
    )
    parser.add_argument(
        "--scene-scale",
        type=float,
        default=1.0,
        help="Uniform scale applied to the referenced apartment scene root.",
    )
    parser.add_argument(
        "--units",
        choices=("m", "cm"),
        default="m",
        help="Convenience unit conversion for the scene root scale.",
    )
    parser.add_argument(
        "--focus",
        choices=tuple(FOCUS_PRESETS) + ("auto",),
        default="living_room",
        help="Named interior camera focus preset.",
    )
    parser.add_argument(
        "--floor-prim",
        default="SM_floor_01_0",
        help="Prim name used to locate the real apartment floor.",
    )
    dome_group = parser.add_mutually_exclusive_group()
    dome_group.add_argument(
        "--add-dome-light",
        dest="add_dome_light",
        action="store_true",
        help="Add a dome light to brighten the interior (default).",
    )
    dome_group.add_argument(
        "--no-add-dome-light",
        dest="add_dome_light",
        action="store_false",
        help="Disable the supplemental dome light.",
    )
    parser.set_defaults(add_dome_light=True)
    parser.add_argument(
        "--dome-intensity",
        type=float,
        default=1000.0,
        help="Intensity of the supplemental dome light.",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    stage = Stage(headless=True, enable_cameras=True)
    stage.boot()

    from environments.usd_scene import (
        DEFAULT_SCENE,
        FramedCameraRig,
        look_at_quat,
        USDSceneEnvironment,
    )
    from robots.h1_policy import H1Robot
    from core.runner import Runner

    scene_path = DEFAULT_SCENE
    scene_scale = float(args.scene_scale) * (0.01 if args.units == "cm" else 1.0)
    walk_direction = WALK_DIR_VECTORS[args.walk_dir]
    robot_orientation = look_at_quat((0.0, 0.0, 0.0), walk_direction)

    env = USDSceneEnvironment(
        scene_path=scene_path,
        scene_scale=scene_scale,
        floor_prim_name=args.floor_prim,
        add_dome_light=args.add_dome_light,
        dome_intensity=args.dome_intensity,
    )
    robot = (
        H1Robot(position=(-2.5, 0.5, 0.05), orientation=robot_orientation)
        if args.with_robot
        else NullRobot()
    )
    if args.with_robot:
        robot.command(
            walk_forward=args.walk_distance,
            speed=args.speed,
            walk_dir=args.walk_dir,
        )

    runner = Runner(stage, env, robot, fps=30)
    runner._build()

    if args.inspect:
        env.inspect(max_depth=2)
        stage.shutdown()
        return

    scene_min, scene_max = env.get_scene_bounds()
    raw_scene_min, raw_scene_max = env.get_raw_scene_bounds()
    scene_center = env.get_scene_center()
    scene_diagonal = env.get_scene_diagonal()
    floor_top_z = env.get_floor_top_z() if env.get_floor_top_z() is not None else float(scene_min[2])
    floor_prim_path = env.get_floor_prim_path()
    focus_preset = FOCUS_PRESETS.get(args.focus)
    if args.focus == "auto":
        camera_position = None
        camera_target = None
        camera_mode = "auto"
        fov_degrees = None
    else:
        camera_position = focus_preset["camera_position"]
        camera_target = focus_preset["camera_target"]
        camera_mode = "fixed"
        fov_degrees = focus_preset["fov_degrees"]

    print(f"[shot] scene path: {scene_path}")
    print(f"[shot] raw scene bounds min={raw_scene_min.tolist()} max={raw_scene_max.tolist()}")
    print(f"[shot] filtered scene bounds min={scene_min.tolist()} max={scene_max.tolist()}")
    print(f"[shot] filtered scene center: {scene_center.tolist()}")
    print(f"[shot] filtered scene diagonal: {scene_diagonal:.3f}")
    print(f"[shot] effective scene scale: {scene_scale:.4f} (units={args.units})")
    print(f"[shot] walk dir: {args.walk_dir} -> {walk_direction.tolist()}")
    print(f"[shot] floor prim used: {floor_prim_path or '<fallback scene bbox min-z>'}")
    print(f"[shot] floor top z: {floor_top_z:.3f}")
    print(f"[shot] focus: {args.focus}")
    if camera_position is not None:
        print(f"[shot] camera position: {camera_position.tolist()}")
        print(f"[shot] camera target: {camera_target.tolist()}")
        print(f"[shot] camera fov degrees: {fov_degrees:.1f}")
    print(f"[shot] final robot spawn: {env.get_robot_spawn_position().tolist()}")
    if args.with_robot:
        print(f"[shot] robot position passed to H1: {np.array(robot.position, dtype=float).tolist()}")
    else:
        print("[shot] robot disabled; rendering empty apartment")

    camera = FramedCameraRig(
        center=scene_center,
        bounds_min=scene_min,
        bounds_max=scene_max,
        prim_path="/World/ApartmentCamera",
        resolution=(1600, 900),
        direction=walk_direction,
        camera_mode=camera_mode,
        floor_top_z=floor_top_z,
        camera_position=camera_position,
        camera_target=camera_target,
        fov_degrees=fov_degrees,
    )

    output_filename = args.output
    if output_filename is None:
        output_filename = "apartment_walk.mp4" if args.with_robot else "apartment_empty.mp4"

    out = runner.record(camera, output_filename, duration_s=args.duration)
    print(f"[shot] DONE -> {out}")

    stage.shutdown()


if __name__ == "__main__":
    main()
