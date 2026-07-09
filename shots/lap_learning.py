"""
shots/lap_learning.py
=====================
PATH A EXPERIMENT for the ESD paper (Scale Infra @ IROS 2026 workshop):
continual command-level adaptation of an H1 humanoid walking a patrol lap in
the compact SimReady apartment living room, skirting the coffee table.

What this demonstrates (mapped to the paper):
  * One rollout per update (R3): each episode is a single lap; the adapter
    updates from that one episode's logs.
  * Typed feedback records (Sec IV.C): per-step velocity error, base pitch/
    roll, furniture proximity, fall events -> JSONL.
  * Continual improvement from own experience (R1): with --learning on, the
    command-layer parameters (segment speeds, path inset) adapt each episode;
    with --learning off, they stay frozen (control condition).
  * Cost metering: per-episode wall time, sim time, control steps -> CSV.

NOTE (honest scope, matches the paper's framing): the H1 locomotion network
stays frozen (NVIDIA's validated flat-terrain policy). Learning happens at the
command layer. Weight-level ESD (Algorithm 1) remains protocol.

GEOMETRY (from apartment_walk.py --inspect, meters):
  coffee table  x[-3.02,-2.15]  y[1.19,2.06]  top z=0.446
  sofa          x[-3.54,-1.50]  y[2.37,3.23]   (blocks the table's north side)
  south wall    y ~= -1.55 ; floor z ~= 0
  The lap is a rectangle in the open floor south of the table:
     NW(-4.4, 0.6+inset) -> NE(-0.9, 0.6+inset) -> SE(-0.9,-0.9) -> SW(-4.4,-0.9)
  The north edge skirts the table; `inset` moves it closer (learned).

RUN (inside the container, repo root):
  # learning condition, 10 episodes, record first + last:
  /isaac-sim/python.sh shots/lap_learning.py --learning on --episodes 10
  # control condition:
  /isaac-sim/python.sh shots/lap_learning.py --learning off --episodes 10

OUTPUTS (in /isaac-sim/Documents/lap_learning/<run-name>/):
  metrics.csv            per-episode: lap_time, path_len, vel_err, incidents...
  feedback_epN.jsonl     typed per-step feedback records
  params_epN.json        parameter snapshots (the thing that learns)
  lap_epN.mp4            videos of recorded episodes
"""

from __future__ import annotations
import os, sys, math, json, time, argparse

parser = argparse.ArgumentParser(description="H1 lap continual-adaptation experiment.")
parser.add_argument("--scene", default="photorealistic_scenes/Apartment/scene_04.usd")
parser.add_argument("--learning", choices=["on", "off"], default="on")
parser.add_argument("--episodes", type=int, default=10)
parser.add_argument("--run-name", default=None, help="output subfolder; default learn_on/learn_off")
parser.add_argument("--record", choices=["first_last", "all", "none"], default="first_last")
parser.add_argument("--fps", type=int, default=30)
parser.add_argument("--timeout", type=float, default=90.0, help="max seconds per episode")
parser.add_argument("--dome-intensity", type=float, default=1400.0)
parser.add_argument("--width", type=int, default=1280)
parser.add_argument("--height-px", type=int, default=720)
args = parser.parse_args()

_repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
scene_path = args.scene if os.path.isabs(args.scene) else os.path.join(_repo, args.scene)
if not os.path.exists(scene_path):
    sys.exit(f"[lap] ERROR: scene not found: {scene_path}")
try:
    if os.path.getsize(scene_path) < 1024:
        with open(scene_path, "rb") as _f:
            _head = _f.read(64)
        if b"git-lfs" in _head or b"version https://" in _head:
            sys.exit(f"[lap] ERROR: '{scene_path}' is a Git LFS pointer stub.\n"
                     f"      Run: git lfs install && git lfs pull")
except OSError:
    pass

run_name = args.run_name or ("learn_on" if args.learning == "on" else "learn_off")
OUT = os.path.join(os.environ.get("OUTPUT_DIR", "/isaac-sim/Documents"), "lap_learning", run_name)
os.makedirs(OUT, exist_ok=True)

# ---------------------------------------------------------------------------
# Boot FIRST; only then import pxr / isaacsim / omni.
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

# ------------------------- scene constants (from --inspect) ----------------
TABLE_MIN = np.array([-3.02, 1.19]); TABLE_MAX = np.array([-2.15, 2.06])
FLOOR_Z = 0.0
# SPAWN/lap audit vs wall bboxes: west wall SM_wall_16 = x[-4.58,-4.15];
# mid wall SM_wall_04 starts at x=-0.74 (y~-0.26); south wall SM_wall_13 at
# y~-1.55. The previous spawn x=-4.4 was INSIDE wall_16 -> robot ejected onto
# the terrace ("spawned outside the window"). New rectangle sits fully in the
# open living area with >=0.45m clearance on every side:
#   west edge x=-3.6 (0.55m from wall_16), east edge x=-1.2 (0.46m from
#   wall_04's west end), south edge y=-0.9 (0.65m from wall_13), north edge
#   y=0.6 (0.59m from the table).
SPAWN = np.array([-3.6, -0.9, 1.05])          # SW corner, inside the room
LAP_TIMEOUT = args.timeout

# ------------------------- learnable parameters ----------------------------
DEFAULT_PARAMS = {
    "seg_speed": [0.45, 0.45, 0.45, 0.45],    # m/s per segment (conservative)
    "inset": 0.00,                             # north edge pull toward table (m)
    "stop_radius": 0.45,                       # waypoint arrival radius (m)
    "turn_gain": 1.4,                          # heading P gain
}
SPEED_MAX, SPEED_MIN = 0.95, 0.30
INSET_MAX = 0.20                               # north edge max pull; keeps
                                               #   >=0.39m table clearance
PROX_THRESH = 0.35                             # incident if closer to table
VEL_ERR_THRESH = 0.35                          # incident if tracking error high
PITCH_THRESH = 0.35                            # rad; instability incident
FALL_Z = 0.60


def waypoints(params):
    """Lap rectangle, fully inside the living area. Ordered so the FIRST
    segment from the SW spawn heads +x (straight ahead of the robot's default
    facing) -- no aggressive 90-degree turn at t=0."""
    n = 0.60 + params["inset"]                # north edge y (toward table)
    return np.array([
        [-1.2, -0.9],                          # SE  (east along south edge)
        [-1.2,  n],                            # NE  (north along east edge)
        [-3.6,  n],                            # NW  (west along north edge)
        [-3.6, -0.9],                          # SW  (south back to start)
    ])


def dist_to_table(p):
    d = np.maximum(np.maximum(TABLE_MIN - p, p - TABLE_MAX), 0.0)
    return float(np.linalg.norm(d))


def quat_to_yaw(q):
    """q = [w,x,y,z] -> yaw (rad)."""
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def look_quat(eye, target):
    f = np.array(target, float) - np.array(eye, float)
    f = f / (np.linalg.norm(f) + 1e-9)
    yaw = np.degrees(np.arctan2(f[1], f[0]))
    pitch = np.degrees(np.arcsin(-f[2]))
    return rot_utils.euler_angles_to_quats(np.array([0.0, pitch, yaw]), degrees=True)


# ------------------------- adapter (the thing that learns) -----------------
def adapt(params, ep_summary):
    """Hill-climb the command parameters from ONE episode's typed feedback.
    Clean fast segments speed up and the line tightens; trouble backs off."""
    p = json.loads(json.dumps(params))  # deep copy
    for i in range(4):
        seg = ep_summary["segments"][i]
        if seg["incidents"] == 0 and seg["mean_vel_err"] < 0.20:
            p["seg_speed"][i] = min(p["seg_speed"][i] + 0.06, SPEED_MAX)
        elif seg["incidents"] > 0:
            p["seg_speed"][i] = max(p["seg_speed"][i] - 0.10, SPEED_MIN)
    if ep_summary["proximity_incidents"] == 0 and not ep_summary["fell"]:
        p["inset"] = min(p["inset"] + 0.05, INSET_MAX)   # tighter racing line
    else:
        p["inset"] = max(p["inset"] - 0.08, 0.0)          # back off
    if ep_summary["fell"]:
        p["seg_speed"] = [max(s - 0.15, SPEED_MIN) for s in p["seg_speed"]]
    return p


def main():
    # CRITICAL TIMING: the H1 flat-terrain policy is built for 200 Hz physics
    # with rendering every 8th step (NVIDIA's h1_standalone.py reference uses
    # exactly physics_dt=1/200, rendering_dt=8/200). Running it at the World
    # default of 60 Hz makes the balance controller mistimed -> instant fall.
    world = World(stage_units_in_meters=1.0,
                  physics_dt=1.0 / 200.0,
                  rendering_dt=8.0 / 200.0)
    stage = omni.usd.get_context().get_stage()

    # Scene
    prim = stage.DefinePrim("/World/Apartment", "Xform")
    prim.GetReferences().AddReference(scene_path)
    dome = UsdLux.DomeLight(stage.DefinePrim("/World/LapDome", "DomeLight"))
    dome.CreateIntensityAttr(float(args.dome_intensity))
    dome.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.92))
    print(f"[lap] scene referenced: {scene_path}")

    # Robot: NVIDIA's built-in validated H1 flat-terrain policy.
    from isaacsim.robot.policy.examples.robots.h1 import H1FlatTerrainPolicy
    h1 = H1FlatTerrainPolicy(prim_path="/World/H1", name="H1",
                             position=SPAWN.copy())

    # Camera: static wide interior view covering the whole lap.
    # Position audited against wall bboxes from --inspect: (0.5,-1.3) sits in
    # open floor, clear of SM_wall_04 (x[-0.74,3.25] @ y~-0.26) and SM_wall_13
    # (y~-1.55); sightline to the lap rectangle crosses no wall extent.
    cam_eye = np.array([0.5, -1.3, 2.15])
    cam_target = np.array([-2.8, 0.2, 0.5])
    cam = Camera(prim_path="/World/LapCam", position=cam_eye, frequency=25,
                 resolution=(args.width, args.height_px),
                 orientation=look_quat(cam_eye, cam_target))
    cam.initialize()
    gc = UsdGeom.Camera(stage.GetPrimAtPath("/World/LapCam"))
    gc.GetFocalLengthAttr().Set(16.0)
    gc.GetHorizontalApertureAttr().Set(20.955)

    world.reset()

    def init_robot():
        """(Re)initialize the H1 against a fresh physics view. Must run after
        every world.reset(), and only after the sim has stepped at least once
        (the physics simulation view does not exist until then)."""
        for _ in range(2):
            world.step(render=False)
        try:
            h1.initialize()
        except Exception as e:
            print(f"[lap] ERROR in h1.initialize(): {e}")
            raise
        try:
            h1.post_reset()
        except Exception:
            pass

    init_robot()

    # Robust base-pose getter across API variants.
    def base_pose():
        for obj in (getattr(h1, "robot", None), h1):
            if obj is None:
                continue
            for name in ("get_world_pose", "get_world_poses"):
                fn = getattr(obj, name, None)
                if fn is None:
                    continue
                try:
                    pos, quat = fn()
                    pos = np.array(pos).reshape(-1)[:3]
                    quat = np.array(quat).reshape(-1)[:4]
                    return pos, quat
                except Exception:
                    continue
        raise RuntimeError("Cannot read H1 base pose; paste this traceback to fix.")

    dt = 1.0 / 200.0                 # physics step (matches World above)
    steps_per_frame = 8              # control/render at 25 Hz (8/200)
    video_fps = 25

    params = json.loads(json.dumps(DEFAULT_PARAMS))
    csv_path = os.path.join(OUT, "metrics.csv")
    with open(csv_path, "w") as f:
        f.write("episode,learning,lap_time_s,path_len_m,mean_vel_err,"
                "incidents,proximity_incidents,fell,control_steps,"
                "wall_time_s,seg_speeds,inset\n")

    for ep in range(1, args.episodes + 1):
        record = (args.record == "all" or
                  (args.record == "first_last" and ep in (1, args.episodes)))
        wps = waypoints(params)
        with open(os.path.join(OUT, f"params_ep{ep}.json"), "w") as f:
            json.dump(params, f, indent=2)

        # Reset episode: world.reset() invalidates the robot's physics views,
        # so a FULL re-initialize is required each episode (post_reset alone
        # cannot rebuild the views -- this caused the ep-1 matmul crash).
        world.reset()
        init_robot()
        for i in range(200):                      # settle 1s at 200 Hz
            try:
                h1.forward(dt, np.zeros(3))
            except Exception as e:
                print(f"[lap] robot forward() failed at settle: {e}")
                try:
                    p, q = base_pose()
                    print(f"[lap] base pose at failure: pos={p.tolist()}")
                except Exception as e2:
                    print(f"[lap] base_pose also failed: {e2}")
                app.close(); return
            world.step(render=(i % 8 == 7))

        fb_path = os.path.join(OUT, f"feedback_ep{ep}.jsonl")
        fb = open(fb_path, "w")
        frames = []
        seg_stats = [{"incidents": 0, "vel_errs": []} for _ in range(4)]
        prox_incidents, fell = 0, False
        path_len, sim_t, steps = 0.0, 0.0, 0
        wp_i, prev_xy = 0, None
        wall_t0 = time.time()

        while sim_t < LAP_TIMEOUT:
            pos, quat = base_pose()
            xy = pos[:2]
            if prev_xy is not None:
                path_len += float(np.linalg.norm(xy - prev_xy))
            prev_xy = xy.copy()

            target = wps[wp_i]
            to_t = target - xy
            d = float(np.linalg.norm(to_t))
            if d < params["stop_radius"]:
                wp_i += 1
                if wp_i >= len(wps):
                    break                          # lap complete
                continue

            yaw = quat_to_yaw(quat)
            desired = math.atan2(to_t[1], to_t[0])
            err = (desired - yaw + math.pi) % (2 * math.pi) - math.pi
            speed = params["seg_speed"][wp_i]
            vx = speed * max(math.cos(err), 0.0)   # slow while misaligned
            wz = float(np.clip(params["turn_gain"] * err, -1.0, 1.0))
            cmd = np.array([vx, 0.0, wz])

            for i in range(steps_per_frame):
                h1.forward(dt, cmd)
                world.step(render=(i == steps_per_frame - 1))
                sim_t += dt; steps += 1

            # ---- typed feedback record (Sec IV.C) ----
            pos2, quat2 = base_pose()
            actual_v = float(np.linalg.norm(pos2[:2] - xy) / (steps_per_frame * dt))
            vel_err = abs(actual_v - vx)
            w2, x2, y2, z2 = quat2
            pitch = math.asin(max(-1.0, min(1.0, 2 * (w2 * y2 - z2 * x2))))
            prox = dist_to_table(pos2[:2])
            rec = {"t": round(sim_t, 3), "seg": wp_i,
                   "pos": [round(float(v), 3) for v in pos2],
                   "cmd_vx": round(vx, 3), "actual_v": round(actual_v, 3),
                   "vel_err": round(vel_err, 3), "pitch": round(pitch, 3),
                   "table_prox": round(prox, 3), "events": []}
            seg_stats[wp_i]["vel_errs"].append(vel_err)
            if vel_err > VEL_ERR_THRESH:
                rec["events"].append("vel_tracking_error")
                seg_stats[wp_i]["incidents"] += 1
            if abs(pitch) > PITCH_THRESH:
                rec["events"].append("base_pitch_excursion")
                seg_stats[wp_i]["incidents"] += 1
            if prox < PROX_THRESH:
                rec["events"].append("furniture_proximity")
                prox_incidents += 1
            if pos2[2] < FALL_Z:
                rec["events"].append("fall")
                fell = True
            fb.write(json.dumps(rec) + "\n")
            if record:
                rgba = cam.get_rgba()
                if rgba is not None and rgba.size > 0:
                    frames.append(rgba[:, :, :3].astype(np.uint8))
            if fell:
                break

        fb.close()
        wall = time.time() - wall_t0
        completed = wp_i >= len(wps)
        lap_time = sim_t if completed else float("nan")
        summary = {
            "segments": [{"incidents": s["incidents"],
                          "mean_vel_err": float(np.mean(s["vel_errs"])) if s["vel_errs"] else 0.0}
                         for s in seg_stats],
            "proximity_incidents": prox_incidents, "fell": fell,
        }
        mean_err = float(np.mean([e for s in seg_stats for e in s["vel_errs"]]) if any(s["vel_errs"] for s in seg_stats) else 0.0)
        with open(csv_path, "a") as f:
            f.write(f"{ep},{args.learning},{lap_time:.2f},{path_len:.2f},"
                    f"{mean_err:.3f},{sum(s['incidents'] for s in seg_stats)},"
                    f"{prox_incidents},{fell},{steps},{wall:.1f},"
                    f"\"{[round(s,2) for s in params['seg_speed']]}\",{params['inset']:.2f}\n")
        status = "COMPLETE" if completed else ("FELL" if fell else "TIMEOUT")
        print(f"[lap] ep{ep} [{status}] time={lap_time:.1f}s path={path_len:.1f}m "
              f"incidents={sum(s['incidents'] for s in seg_stats)} "
              f"prox={prox_incidents} speeds={[round(s,2) for s in params['seg_speed']]} "
              f"inset={params['inset']:.2f}")

        if record and frames:
            vp = os.path.join(OUT, f"lap_ep{ep}.mp4")
            imageio.mimsave(vp, frames, fps=video_fps, codec="libx264", quality=8)
            print(f"[lap]   video -> {vp}")

        if args.learning == "on":
            params = adapt(params, summary)

    print(f"[lap] DONE. All outputs in {OUT}")
    print(f"[lap] brev cp -r from ~/docker/isaac-sim/data/lap_learning/ to your Mac.")


main()
app.close()
