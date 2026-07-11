"""
shots/h1_walk_grid.py
=====================
The SAME living-room walk as h1_walk_home.py (same spawn, waypoints, speeds,
physics fixes), filmed by FOUR cameras SIMULTANEOUSLY from different corners
of the apartment, composited into a single 2x2 grid MP4.

All four views show the identical walk because they capture the same run.
Each camera is fixed in position but RE-AIMS at the robot every frame, so the
robot stays roughly centered and visible in every quadrant throughout.

Grid layout (1280x720 total, each cell 640x360):
    [ A: south-east view ][ B: north-east view ]
    [ C: north view      ][ D: south-west view ]

Run:
    /isaac-sim/python.sh shots/h1_walk_grid.py
Output:
    /isaac-sim/Documents/h1_walk_grid.mp4
"""

from __future__ import annotations
import os, sys, math, argparse

parser = argparse.ArgumentParser()
parser.add_argument("--scene", default="photorealistic_scenes/Apartment/scene_04.usd")
parser.add_argument("--out", default="h1_walk_grid.mp4")
parser.add_argument("--laps", type=int, default=2)
parser.add_argument("--speed", type=float, default=0.5)
parser.add_argument("--turn-gain", type=float, default=1.2)
parser.add_argument("--steer-sign", type=float, default=1.0)
parser.add_argument("--dome-intensity", type=float, default=1400.0)
args = parser.parse_args()

_repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
scene_path = args.scene if os.path.isabs(args.scene) else os.path.join(_repo, args.scene)
if not os.path.exists(scene_path):
    sys.exit(f"[grid] ERROR: scene not found: {scene_path}")
try:
    if os.path.getsize(scene_path) < 1024:
        with open(scene_path, "rb") as _f:
            if b"git-lfs" in _f.read(64):
                sys.exit("[grid] ERROR: scene is an LFS stub. Run: git lfs install && git lfs pull")
except OSError:
    pass

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "enable_cameras": True,
                     "width": 1280, "height": 720})

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils
from pxr import UsdGeom, UsdLux, Gf
import imageio.v2 as imageio

# --- identical walk geometry to h1_walk_home.py ------------------------------
SPAWN = np.array([-3.6, -0.9, 1.05])
WAYPOINTS = np.array([
    [-1.2, -0.9],
    [-1.2,  0.6],
    [-3.6,  0.6],
    [-3.6, -0.9],
])
STOP_RADIUS = 0.45
FALL_Z = 0.60
TIMEOUT_S = 60.0 * args.laps

PHYS_DT = 1.0 / 200.0
SUBSTEPS = 8
VIDEO_FPS = 25

CELL_W, CELL_H = 640, 360          # each quadrant; grid = 1280x720

# --- four camera positions, audited against wall/furniture bboxes ------------
# Each is a fixed eye position; orientation re-aims at the robot every frame.
#   A (0.5,-1.3):  open floor SE of the loop (the proven angle)
#   B (0.3, 1.6):  NE, in the dining-area opening, looking SW past the table
#   C (-1.6, 1.8): N, between sofa (y>=2.37) and table (x<=-2.15), looking S
#   D (-3.95,-1.2): SW corner nook (0.2m off west wall), looking NE
CAMERAS = {
    "A_southeast": np.array([ 0.50, -1.30, 2.15]),
    "B_northeast": np.array([ 0.30,  1.60, 2.10]),
    "C_north":     np.array([-1.60,  1.80, 2.10]),
    "D_southwest": np.array([-3.95, -1.20, 2.00]),
}
AIM_HEIGHT = 0.9                    # look at the robot's torso


def look_quat(eye, target):
    f = np.array(target, float) - np.array(eye, float)
    f /= (np.linalg.norm(f) + 1e-9)
    yaw = np.degrees(np.arctan2(f[1], f[0]))
    pitch = np.degrees(np.arcsin(-f[2]))
    return rot_utils.euler_angles_to_quats(np.array([0.0, pitch, yaw]), degrees=True)


def quat_to_yaw(q):
    w, x, y, z = (float(v) for v in q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main():
    world = World(stage_units_in_meters=1.0,
                  physics_dt=PHYS_DT, rendering_dt=SUBSTEPS * PHYS_DT)
    stage = omni.usd.get_context().get_stage()

    # Scene + the fixes proven in h1_walk_home.py
    prim = stage.DefinePrim("/World/Apartment", "Xform")
    prim.GetReferences().AddReference(scene_path)
    print(f"[grid] scene referenced: {scene_path}")
    world.scene.add_default_ground_plane(z_position=0.0)
    gp = stage.GetPrimAtPath("/World/defaultGroundPlane")
    if gp and gp.IsValid():
        UsdGeom.Imageable(gp).MakeInvisible()
        print("[grid] invisible physics ground plane added at z=0")
    try:
        from pxr import UsdPhysics
        for p in stage.Traverse():
            if p.IsA(UsdPhysics.Scene) and str(p.GetPath()).startswith("/World/Apartment"):
                p.SetActive(False)
                print(f"[grid] deactivated embedded PhysicsScene: {p.GetPath()}")
    except Exception as e:
        print(f"[grid] physics-scene scan skipped: {e}")

    dome = UsdLux.DomeLight(stage.DefinePrim("/World/Dome", "DomeLight"))
    dome.CreateIntensityAttr(float(args.dome_intensity))
    dome.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.92))

    from isaacsim.robot.policy.examples.robots.h1 import H1FlatTerrainPolicy
    h1 = H1FlatTerrainPolicy(prim_path="/World/H1", name="H1", position=SPAWN.copy())

    # Four tracking cameras
    cams = {}
    for name, eye in CAMERAS.items():
        c = Camera(prim_path=f"/World/GridCam_{name}", position=eye,
                   resolution=(CELL_W, CELL_H),
                   orientation=look_quat(eye, np.array([*SPAWN[:2], AIM_HEIGHT])))
        c.initialize()
        g = UsdGeom.Camera(stage.GetPrimAtPath(f"/World/GridCam_{name}"))
        g.GetFocalLengthAttr().Set(18.0)
        g.GetHorizontalApertureAttr().Set(20.955)
        cams[name] = (c, eye)
    print(f"[grid] {len(cams)} cameras placed")

    world.reset()
    for _ in range(2):
        world.step(render=False)
    h1.initialize()
    try:
        h1.post_reset()
    except Exception:
        pass
    try:
        dp = getattr(h1, "default_pos", None)
        if dp is not None and getattr(h1, "robot", None) is not None:
            h1.robot.set_joints_default_state(dp)
            h1.robot.post_reset()
            print("[grid] default joint stance applied")
    except Exception as e:
        print(f"[grid] WARNING: could not set default joint state: {e}")

    def base_pose():
        for obj in (getattr(h1, "robot", None), h1):
            if obj is None:
                continue
            fn = getattr(obj, "get_world_pose", None)
            if fn is None:
                continue
            try:
                pos, quat = fn()
                return np.array(pos).reshape(-1)[:3], np.array(quat).reshape(-1)[:4]
            except Exception:
                continue
        raise RuntimeError("cannot read H1 base pose")

    def aim_all(robot_xy):
        target = np.array([robot_xy[0], robot_xy[1], AIM_HEIGHT])
        for c, eye in cams.values():
            c.set_world_pose(position=eye, orientation=look_quat(eye, target))

    def grab_grid():
        """Capture all four cameras and tile into one 2x2 frame."""
        tiles = []
        for name in ("A_southeast", "B_northeast", "C_north", "D_southwest"):
            rgba = cams[name][0].get_rgba()
            if rgba is None or rgba.size == 0:
                tiles.append(np.zeros((CELL_H, CELL_W, 3), dtype=np.uint8))
            else:
                tiles.append(rgba[:, :, :3].astype(np.uint8))
        top = np.hstack([tiles[0], tiles[1]])
        bot = np.hstack([tiles[2], tiles[3]])
        return np.vstack([top, bot])

    # Settle 1.5 s, cameras already aimed at spawn
    for i in range(300):
        h1.forward(PHYS_DT, np.zeros(3))
        world.step(render=(i % SUBSTEPS == SUBSTEPS - 1))
    p0, _ = base_pose()
    print(f"[grid] settled. base z={p0[2]:.2f} (should be ~1.0)")
    if p0[2] < FALL_Z:
        print("[grid] ERROR: robot fell during settle. Aborting.")
        app.close(); return

    frames = []
    total_wp = list(WAYPOINTS) * args.laps
    wp_i, sim_t = 0, 0.0
    print(f"[grid] walking {args.laps} lap(s), filming 4 angles...")

    while sim_t < TIMEOUT_S and wp_i < len(total_wp):
        pos, quat = base_pose()
        if pos[2] < FALL_Z:
            print(f"[grid] robot FELL at t={sim_t:.1f}s")
            break
        target = total_wp[wp_i]
        to_t = target - pos[:2]
        if float(np.linalg.norm(to_t)) < STOP_RADIUS:
            wp_i += 1
            print(f"[grid] waypoint {wp_i}/{len(total_wp)} t={sim_t:.1f}s")
            continue
        yaw = quat_to_yaw(quat)
        desired = math.atan2(to_t[1], to_t[0])
        err = (desired - yaw + math.pi) % (2 * math.pi) - math.pi
        vx = args.speed * max(math.cos(err), 0.0)
        wz = args.steer_sign * float(np.clip(args.turn_gain * err, -0.7, 0.7))
        cmd = np.array([vx, 0.0, wz])

        aim_all(pos[:2])                       # keep robot centered, all views
        for i in range(SUBSTEPS):
            h1.forward(PHYS_DT, cmd)
            world.step(render=(i == SUBSTEPS - 1))
            sim_t += PHYS_DT
        frames.append(grab_grid())

    # Final standing beat, 3 s
    for i in range(75):
        h1.forward(PHYS_DT, np.zeros(3))
        world.step(render=(i % SUBSTEPS == SUBSTEPS - 1))
        if i % SUBSTEPS == SUBSTEPS - 1:
            try:
                pos, _ = base_pose()
                aim_all(pos[:2])
            except Exception:
                pass
            frames.append(grab_grid())

    out_dir = os.environ.get("OUTPUT_DIR", "/isaac-sim/Documents")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, args.out if args.out.endswith(".mp4") else args.out + ".mp4")
    if frames:
        imageio.mimsave(out_path, frames, fps=VIDEO_FPS, codec="libx264", quality=8)
        done = wp_i >= len(total_wp)
        print(f"[grid] {'COMPLETED' if done else 'PARTIAL'} -> {out_path} "
              f"({len(frames)} frames, 2x2 grid @ {CELL_W*2}x{CELL_H*2})")
    else:
        print("[grid] no frames captured")


main()
app.close()
