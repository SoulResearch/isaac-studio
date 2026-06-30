"""
shots/apartment_orbit.py
========================
Standalone shot: load the SimReady Apartment USD scene and ORBIT the camera
around the living room (sofa + coffee table), recording an MP4. No robot.

This file is intentionally self-contained — it does not depend on the rest of
the repo's core/ modules — so it can't be broken by other edits. It reuses the
interior coordinates confirmed from `apartment_walk.py --inspect`:
    floor SM_floor_01_0 at z ~= 0
    sofa center (-2.52, 2.80, 0.43), coffee table (-2.59, 1.62, 0.23)
    walls ~2.59 m tall; living-room zone x in [-4, 0], y in [0, 3.5]

Run inside the container, from the repo root:
    /isaac-sim/python.sh shots/apartment_orbit.py
    /isaac-sim/python.sh shots/apartment_orbit.py --degrees 360
    /isaac-sim/python.sh shots/apartment_orbit.py --radius 2.4 --dome-intensity 1600

Output: /isaac-sim/Documents/apartment_orbit.mp4  (brev cp to your Mac).

NOTE ON 360 vs 180: the apartment is an interior. A full 360 swings the camera
THROUGH the back/left walls for part of the arc (you'll get wall-backside or
black frames on that side). The default is a 180 sweep across the OPEN front of
the room, which stays inside and looks clean. Pass --degrees 360 for the full
spin anyway.
"""

from __future__ import annotations
import os
import sys
import math
import argparse

# argparse is stdlib, safe to use before SimulationApp boots.
parser = argparse.ArgumentParser(description="Orbit the camera around the apartment living room.")
parser.add_argument("--scene", default="photorealistic_scenes/Apartment/scene_04.usd",
                    help="repo-relative or absolute path to the scene .usd")
parser.add_argument("--out", default="apartment_orbit.mp4", help="output mp4 filename")
parser.add_argument("--duration", type=float, default=8.0, help="clip length seconds")
parser.add_argument("--fps", type=int, default=30, help="frames per second")
# Orbit geometry (defaults tuned to the confirmed living-room coordinates).
parser.add_argument("--pivot", type=float, nargs=3, default=[-2.5, 1.9, 0.8],
                    metavar=("X", "Y", "Z"), help="point the camera orbits around / looks at")
parser.add_argument("--radius", type=float, default=2.7, help="orbit radius (m) from pivot")
parser.add_argument("--height", type=float, default=1.5, help="camera height z (m)")
parser.add_argument("--degrees", type=float, default=180.0,
                    help="total arc swept over the clip (180 clean interior; 360 spins through walls)")
parser.add_argument("--start-deg", type=float, default=None,
                    help="starting angle (deg). Default centers the arc on the front-of-sofa view.")
parser.add_argument("--direction", choices=["cw", "ccw"], default="ccw", help="orbit direction")
parser.add_argument("--settle", type=int, default=3, help="render substeps per frame (more = less noise, slower)")
parser.add_argument("--dome-intensity", type=float, default=1200.0, help="fill dome light intensity")
parser.add_argument("--width", type=int, default=1280, help="render width (keep divisible by 16)")
parser.add_argument("--height-px", type=int, default=720, help="render height (keep divisible by 16)")
parser.add_argument("--focal", type=float, default=18.0, help="lens focal length (lower = wider FOV)")
args = parser.parse_args()

# Resolve scene path (repo-relative -> absolute) and stub-check before booting.
_repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
scene_path = args.scene if os.path.isabs(args.scene) else os.path.join(_repo_root, args.scene)
if not os.path.exists(scene_path):
    sys.exit(f"[orbit] ERROR: scene not found: {scene_path}")
try:
    if os.path.getsize(scene_path) < 1024:
        with open(scene_path, "rb") as _f:
            _head = _f.read(64)
        if b"git-lfs" in _head or b"version https://" in _head:
            sys.exit(f"[orbit] ERROR: '{scene_path}' is a Git LFS pointer stub.\n"
                     f"        Run: git lfs install && git lfs pull")
except OSError:
    pass

# ---------------------------------------------------------------------------
# Boot the simulator FIRST. Only after this can we import pxr/isaacsim/omni.
# ---------------------------------------------------------------------------
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "enable_cameras": True,
                     "width": args.width, "height": args.height_px})

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils
from pxr import UsdGeom, UsdLux, Gf

import imageio.v2 as imageio


def look_quat(eye, target):
    """Quaternion that points the camera from eye toward target.
    Same convention that produced the correct static interior shot."""
    f = np.array(target, dtype=float) - np.array(eye, dtype=float)
    f = f / (np.linalg.norm(f) + 1e-9)
    yaw = np.degrees(np.arctan2(f[1], f[0]))
    pitch = np.degrees(np.arcsin(-f[2]))
    return rot_utils.euler_angles_to_quats(np.array([0.0, pitch, yaw]), degrees=True)


def main():
    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()

    # Reference the scene onto /World/Apartment (identity transform, so the
    # world coordinates match what --inspect reported).
    scene_prim = stage.DefinePrim("/World/Apartment", "Xform")
    scene_prim.GetReferences().AddReference(scene_path)
    print(f"[orbit] referenced {scene_path}")

    # Fill dome light so the interior is lit (the scene's own HDRI cubemap
    # binding is broken; this guarantees visibility).
    dome = stage.DefinePrim("/World/OrbitDome", "DomeLight")
    d = UsdLux.DomeLight(dome)
    d.CreateIntensityAttr(float(args.dome_intensity))
    d.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.92))

    # Create the camera once; we move it each frame.
    pivot = np.array(args.pivot, dtype=float)
    start_eye = np.array([pivot[0] + args.radius, pivot[1], args.height])
    cam_path = "/World/OrbitCam"
    cam = Camera(prim_path=cam_path, position=start_eye, frequency=args.fps,
                 resolution=(args.width, args.height_px),
                 orientation=look_quat(start_eye, pivot))
    cam.initialize()

    # Wide lens via the USD camera prim (FOV depends on focal/aperture ratio,
    # so this is robust to Isaac Sim unit quirks). focal 18 + aperture 20.955
    # ~= 60 deg horizontal FOV.
    geom_cam = UsdGeom.Camera(stage.GetPrimAtPath(cam_path))
    geom_cam.GetFocalLengthAttr().Set(float(args.focal))
    geom_cam.GetHorizontalApertureAttr().Set(20.955)

    world.reset()

    # Let the scene load + RTX settle before the first captured frame.
    for _ in range(60):
        world.step(render=True)

    total_frames = max(int(args.duration * args.fps), 1)
    sweep = math.radians(args.degrees)
    sign = 1.0 if args.direction == "ccw" else -1.0
    # Default start angle centers the arc on the front-of-sofa view. The open
    # front of the room is on the -y side of the pivot, which is angle = -90deg
    # (270deg). Center a `degrees`-wide arc there.
    if args.start_deg is not None:
        start = math.radians(args.start_deg)
    else:
        start = math.radians(270.0) - sign * (sweep / 2.0)

    out_dir = os.environ.get("OUTPUT_DIR", "/isaac-sim/Documents")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, args.out if args.out.endswith(".mp4") else args.out + ".mp4")

    if args.degrees >= 359.0:
        print("[orbit] NOTE: 360 arc will pass through interior walls on the far "
              "side; expect some wall-backside/black frames. Use --degrees 180 "
              "for a clean sweep.")

    print(f"[orbit] pivot={pivot.tolist()} radius={args.radius} height={args.height} "
          f"degrees={args.degrees} dir={args.direction} frames={total_frames}")

    frames = []
    for i in range(total_frames):
        t = i / max(total_frames - 1, 1)
        angle = start + sign * t * sweep
        eye = np.array([pivot[0] + args.radius * math.cos(angle),
                        pivot[1] + args.radius * math.sin(angle),
                        args.height])
        cam.set_world_pose(position=eye, orientation=look_quat(eye, pivot))
        for _ in range(max(args.settle, 1)):
            world.step(render=True)
        rgba = cam.get_rgba()
        if rgba is not None and rgba.size > 0:
            frames.append(rgba[:, :, :3].astype(np.uint8))

    if not frames:
        print("[orbit] ERROR: no frames captured.")
        app.close()
        return

    imageio.mimsave(out_path, frames, fps=args.fps, codec="libx264", quality=8)
    print(f"[orbit] DONE -> {out_path} ({len(frames)} frames)")
    print("[orbit] brev cp it from ~/docker/isaac-sim/data/ to your Mac.")


main()
app.close()
