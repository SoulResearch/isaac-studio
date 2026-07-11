"""
shots/h1_walk_home.py
=====================
ONE JOB: the H1 walks around the living room of the apartment without falling,
filmed from a good interior angle, saved as an MP4. No learning, no logging,
no experiment -- just the robot walking in the house.

Path: a gentle rectangle in the open living area (audited against every wall
and furniture bounding box from --inspect), two laps, then stop and stand.

Run (inside the container, repo root):
    /isaac-sim/python.sh shots/h1_walk_home.py
Output:
    /isaac-sim/Documents/h1_walk_home.mp4

If the robot SPINS IN PLACE instead of walking toward its first corner, the
steering sign is inverted for this policy build -- rerun with:
    /isaac-sim/python.sh shots/h1_walk_home.py --steer-sign -1
"""

from __future__ import annotations
import os, sys, math, argparse

parser = argparse.ArgumentParser()
parser.add_argument("--scene", default="photorealistic_scenes/Apartment/scene_04.usd")
parser.add_argument("--out", default="h1_walk_home.mp4")
parser.add_argument("--laps", type=int, default=2)
parser.add_argument("--speed", type=float, default=0.5, help="walk speed m/s")
parser.add_argument("--turn-gain", type=float, default=1.2)
parser.add_argument("--steer-sign", type=float, default=1.0,
                    help="flip to -1 if the robot spins instead of steering")
parser.add_argument("--dome-intensity", type=float, default=1400.0)
parser.add_argument("--no-scene", action="store_true",
                    help="DIAGNOSTIC: skip the apartment; spawn on a default "
                         "ground plane (NVIDIA's known-good example condition)")
args = parser.parse_args()

_repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
scene_path = args.scene if os.path.isabs(args.scene) else os.path.join(_repo, args.scene)
if not os.path.exists(scene_path):
    sys.exit(f"[walk] ERROR: scene not found: {scene_path}")
try:
    if os.path.getsize(scene_path) < 1024:
        with open(scene_path, "rb") as _f:
            if b"git-lfs" in _f.read(64):
                sys.exit("[walk] ERROR: scene is an LFS stub. Run: git lfs install && git lfs pull")
except OSError:
    pass

# --- boot first ------------------------------------------------------------
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

# --- hardcoded, audited geometry (meters; from apartment --inspect) --------
# Open living-area box: west wall x=-4.15, mid wall east x=-0.74 (y~-0.26),
# south wall y=-1.5, coffee table y>=1.19. Rectangle sits inside with >=0.45m
# clearance everywhere. First segment heads +x = robot's default facing.
SPAWN = np.array([-3.6, -0.9, 1.05])
WAYPOINTS = np.array([
    [-1.2, -0.9],   # east along the south edge (straight ahead from spawn)
    [-1.2,  0.6],   # north along the east edge
    [-3.6,  0.6],   # west along the north edge (skirting the coffee table)
    [-3.6, -0.9],   # south back to start
])
STOP_RADIUS = 0.45
FALL_Z = 0.60
TIMEOUT_S = 60.0 * args.laps

PHYS_DT = 1.0 / 200.0     # the H1 policy's required physics rate
SUBSTEPS = 8              # render/control frame every 8 steps = 25 Hz
VIDEO_FPS = 25


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

    if args.no_scene:
        world.scene.add_default_ground_plane(z_position=0.0)
        print("[walk] DIAGNOSTIC MODE: default ground plane, no apartment.")
    else:
        prim = stage.DefinePrim("/World/Apartment", "Xform")
        prim.GetReferences().AddReference(scene_path)
        print(f"[walk] scene referenced: {scene_path}")
    dome = UsdLux.DomeLight(stage.DefinePrim("/World/Dome", "DomeLight"))
    dome.CreateIntensityAttr(float(args.dome_intensity))
    dome.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.92))

    from isaacsim.robot.policy.examples.robots.h1 import H1FlatTerrainPolicy
    h1 = H1FlatTerrainPolicy(prim_path="/World/H1", name="H1", position=SPAWN.copy())

    # Camera: audited interior view (clear of every wall bbox), sees the loop.
    cam_eye = np.array([0.5, -1.3, 2.15])
    cam_target = np.array([-2.8, 0.2, 0.5])
    cam = Camera(prim_path="/World/WalkCam", position=cam_eye,
                 resolution=(1280, 720),
                 orientation=look_quat(cam_eye, cam_target))
    cam.initialize()
    gc = UsdGeom.Camera(stage.GetPrimAtPath("/World/WalkCam"))
    gc.GetFocalLengthAttr().Set(16.0)
    gc.GetHorizontalApertureAttr().Set(20.955)

    world.reset()
    for _ in range(2):
        world.step(render=False)          # physics view must exist first
    h1.initialize()
    try:
        h1.post_reset()
    except Exception:
        pass
    # NVIDIA's H1 example also seeds the articulation's default joint state
    # from the policy's trained stance; without it the robot can start from a
    # zero pose the policy was never trained to recover from.
    try:
        dp = getattr(h1, "default_pos", None)
        if dp is not None and getattr(h1, "robot", None) is not None:
            h1.robot.set_joints_default_state(dp)
            h1.robot.post_reset()
            print(f"[walk] default joint stance applied ({np.array(dp).shape})")
        else:
            print("[walk] WARNING: h1.default_pos not found; stance not seeded")
    except Exception as e:
        print(f"[walk] WARNING: could not set default joint state: {e}")

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

    # Settle: stand in place for 1.5 s at 200 Hz before walking.
    for i in range(300):
        h1.forward(PHYS_DT, np.zeros(3))
        world.step(render=(i % SUBSTEPS == SUBSTEPS - 1))
    p0, _ = base_pose()
    print(f"[walk] settled. base z={p0[2]:.2f} (should be ~1.0)")
    if p0[2] < FALL_Z:
        print("[walk] ERROR: robot fell during settle. Aborting.")
        app.close(); return

    frames = []
    total_wp = list(WAYPOINTS) * args.laps
    wp_i, sim_t = 0, 0.0
    print(f"[walk] walking {args.laps} lap(s), {len(total_wp)} waypoints, "
          f"speed={args.speed} steer_sign={args.steer_sign}")

    while sim_t < TIMEOUT_S and wp_i < len(total_wp):
        pos, quat = base_pose()
        if pos[2] < FALL_Z:
            print(f"[walk] robot FELL at t={sim_t:.1f}s pos={pos[:2].round(2).tolist()}")
            break
        target = total_wp[wp_i]
        to_t = target - pos[:2]
        if float(np.linalg.norm(to_t)) < STOP_RADIUS:
            wp_i += 1
            print(f"[walk] waypoint {wp_i}/{len(total_wp)} reached t={sim_t:.1f}s")
            continue
        yaw = quat_to_yaw(quat)
        desired = math.atan2(to_t[1], to_t[0])
        err = (desired - yaw + math.pi) % (2 * math.pi) - math.pi
        vx = args.speed * max(math.cos(err), 0.0)
        wz = args.steer_sign * float(np.clip(args.turn_gain * err, -0.7, 0.7))
        cmd = np.array([vx, 0.0, wz])
        for i in range(SUBSTEPS):
            h1.forward(PHYS_DT, cmd)
            world.step(render=(i == SUBSTEPS - 1))
            sim_t += PHYS_DT
        rgba = cam.get_rgba()
        if rgba is not None and rgba.size > 0:
            frames.append(rgba[:, :, :3].astype(np.uint8))

    # Stop and stand for a beat at the end.
    for i in range(75):  # 3 s at 25 fps
        h1.forward(PHYS_DT, np.zeros(3))
        world.step(render=(i % SUBSTEPS == SUBSTEPS - 1))
        if i % SUBSTEPS == SUBSTEPS - 1:
            rgba = cam.get_rgba()
            if rgba is not None and rgba.size > 0:
                frames.append(rgba[:, :, :3].astype(np.uint8))

    out_dir = os.environ.get("OUTPUT_DIR", "/isaac-sim/Documents")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, args.out if args.out.endswith(".mp4") else args.out + ".mp4")
    if frames:
        imageio.mimsave(out_path, frames, fps=VIDEO_FPS, codec="libx264", quality=8)
        done = wp_i >= len(total_wp)
        print(f"[walk] {'COMPLETED' if done else 'PARTIAL'} -> {out_path} ({len(frames)} frames)")
    else:
        print("[walk] no frames captured")


main()
app.close()
