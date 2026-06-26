"""
environments/usd_scene.py
=========================
References a packaged photoreal USD scene into Isaac Sim and exposes the same
`build(world)` Lego-piece interface as the procedural environments.

This is the default environment path for the apartment asset pack in
`photorealistic_scenes/Apartment/scene_04.usd`. The file is tracked with Git
LFS, so cold-start scripts must run `git lfs pull` after cloning.

Two ways to use this file:

  A) As a Lego piece in a shot:
        from environments.usd_scene import USDSceneEnvironment
        env = USDSceneEnvironment()
        env.build(world)

  B) Standalone preview render:
        /isaac-sim/python.sh environments/usd_scene.py
     -> writes preview PNGs to /isaac-sim/Documents/apartment_preview_*.png
"""

from __future__ import annotations

from pathlib import Path
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = "photorealistic_scenes/Apartment/scene_04.usd"
DEFAULT_SCENE_PRIM_PATH = "/World/Apartment"
DEFAULT_CAMERA_PRIM_PATH = "/World/ApartmentPreviewCamera"
DEFAULT_FOCAL_LENGTH = 15.5
DEFAULT_HORIZONTAL_APERTURE = 20.955
DEFAULT_PULLBACK_SCALE = 1.15
DEFAULT_HEIGHT_SCALE = 0.20
DEFAULT_ROBOT_HEIGHT = 1.05


def _resolve_scene_path(scene_path: str | Path) -> Path:
    path = Path(scene_path)
    if not path.is_absolute():
        path = REPO_ROOT / path
    return path


def _is_git_lfs_pointer(scene_path: Path) -> bool:
    try:
        with scene_path.open("rb") as file_handle:
            head = file_handle.read(256)
    except OSError:
        return False
    return head.startswith(b"version https://git-lfs.github.com/spec/v1")


def _as_vec3(values) -> np.ndarray:
    return np.array([float(values[0]), float(values[1]), float(values[2])],
                    dtype=float)


def look_at_quat(eye, target):
    import isaacsim.core.utils.numpy.rotations as rot_utils

    eye_vec = _as_vec3(eye)
    target_vec = _as_vec3(target)
    forward = target_vec - eye_vec
    forward = forward / (np.linalg.norm(forward) + 1e-9)
    yaw = np.degrees(np.arctan2(forward[1], forward[0]))
    pitch = np.degrees(np.arcsin(-forward[2]))
    return rot_utils.euler_angles_to_quats(
        np.array([0.0, pitch, yaw]), degrees=True)


def scene_bounds_from_prim(prim):
    from pxr import UsdGeom, Usd

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        ["default", "render"],
        useExtentsHint=True,
    )
    world_bound = bbox_cache.ComputeWorldBound(prim)
    aligned_range = world_bound.ComputeAlignedRange()
    min_point = aligned_range.GetMin()
    max_point = aligned_range.GetMax()
    min_vec = np.array([min_point[0], min_point[1], min_point[2]], dtype=float)
    max_vec = np.array([max_point[0], max_point[1], max_point[2]], dtype=float)
    center = (min_vec + max_vec) / 2.0
    diagonal = float(np.linalg.norm(max_vec - min_vec))
    return min_vec, max_vec, center, diagonal


def camera_eye_from_bounds(center, bounds_min, bounds_max,
                           pullback_scale: float = DEFAULT_PULLBACK_SCALE,
                           height_scale: float = DEFAULT_HEIGHT_SCALE,
                           direction=(1.0, -1.0, 0.0)):
    center_vec = _as_vec3(center)
    min_vec = _as_vec3(bounds_min)
    max_vec = _as_vec3(bounds_max)
    diagonal = float(np.linalg.norm(max_vec - min_vec))

    direction_vec = _as_vec3(direction)
    direction_vec[2] = 0.0
    direction_norm = np.linalg.norm(direction_vec[:2])
    if direction_norm < 1e-9:
        direction_vec = np.array([1.0, -1.0, 0.0], dtype=float)
        direction_norm = np.linalg.norm(direction_vec[:2])
    direction_vec = direction_vec / direction_norm

    eye = center_vec + direction_vec * (diagonal * pullback_scale)
    eye[2] = center_vec[2] + diagonal * height_scale
    return eye


def set_usd_camera_lens(stage, prim_path: str,
                        focal_length: float = DEFAULT_FOCAL_LENGTH,
                        horizontal_aperture: float = DEFAULT_HORIZONTAL_APERTURE):
    from pxr import UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Camera prim does not exist yet: {prim_path}")
    usd_camera = UsdGeom.Camera(prim)
    usd_camera.GetFocalLengthAttr().Set(float(focal_length))
    usd_camera.GetHorizontalApertureAttr().Set(float(horizontal_aperture))
    return usd_camera


class FramedCameraRig:
    """A small camera wrapper that frames a scene from its bounding box."""

    def __init__(self, center, bounds_min, bounds_max,
                 prim_path: str = DEFAULT_CAMERA_PRIM_PATH,
                 focal_length: float = DEFAULT_FOCAL_LENGTH,
                 horizontal_aperture: float = DEFAULT_HORIZONTAL_APERTURE,
                 resolution=(1600, 900),
                 pullback_scale: float = DEFAULT_PULLBACK_SCALE,
                 height_scale: float = DEFAULT_HEIGHT_SCALE,
                 direction=(1.0, -1.0, 0.0)):
        self.center = _as_vec3(center)
        self.bounds_min = _as_vec3(bounds_min)
        self.bounds_max = _as_vec3(bounds_max)
        self.prim_path = prim_path
        self.focal_length = float(focal_length)
        self.horizontal_aperture = float(horizontal_aperture)
        self.resolution = resolution
        self.pullback_scale = float(pullback_scale)
        self.height_scale = float(height_scale)
        self.direction = direction
        self.sensor = None

    def attach(self, world):
        from isaacsim.sensors.camera import Camera
        import omni.usd

        eye = camera_eye_from_bounds(
            self.center,
            self.bounds_min,
            self.bounds_max,
            pullback_scale=self.pullback_scale,
            height_scale=self.height_scale,
            direction=self.direction,
        )
        quat = look_at_quat(eye, self.center)
        self.sensor = Camera(
            prim_path=self.prim_path,
            position=eye,
            frequency=30,
            resolution=self.resolution,
            orientation=quat,
        )
        stage = omni.usd.get_context().get_stage()
        set_usd_camera_lens(
            stage,
            self.prim_path,
            focal_length=self.focal_length,
            horizontal_aperture=self.horizontal_aperture,
        )
        return self.sensor

    def initialize(self):
        if self.sensor is not None:
            self.sensor.initialize()


class USDSceneEnvironment:
    """References a photoreal USD scene into the stage."""

    def __init__(self,
                 scene_path: str | Path = DEFAULT_SCENE,
                 scene_prim_path: str = DEFAULT_SCENE_PRIM_PATH,
                 robot_height: float = DEFAULT_ROBOT_HEIGHT):
        self.scene_path = str(scene_path)
        self.scene_prim_path = scene_prim_path
        self.robot_height = float(robot_height)
        self._stage = None
        self._scene_prim = None
        self.scene_bounds_min = None
        self.scene_bounds_max = None
        self.scene_center = None
        self.scene_diagonal = None
        self.robot_spawn_position = None

    def build(self, world):
        import omni.usd
        from pxr import UsdGeom

        self._stage = omni.usd.get_context().get_stage()
        scene_path = _resolve_scene_path(self.scene_path)
        if not scene_path.exists():
            raise FileNotFoundError(
                f"USD scene not found: {scene_path}. "
                "Run the cold-start scripts so the repo and assets are present.")
        if _is_git_lfs_pointer(scene_path):
            raise RuntimeError(
                f"Git LFS pointer detected at {scene_path}. "
                "Run `git lfs pull` inside the repo so the apartment asset is fully downloaded.")

        scene_xform = UsdGeom.Xform.Define(self._stage, self.scene_prim_path)
        scene_xform.GetPrim().GetReferences().AddReference(str(scene_path))
        self._scene_prim = scene_xform.GetPrim()

        self.scene_bounds_min, self.scene_bounds_max, self.scene_center, \
            self.scene_diagonal = scene_bounds_from_prim(self._scene_prim)
        # Add a hidden support plane just below the apartment floor so the
        # robot has reliable footing even if the imported scene lacks physics.
        world.scene.add_default_ground_plane(
            z_position=float(self.scene_bounds_min[2]) - 0.02)
        self.robot_spawn_position = np.array([
            float(self.scene_center[0]),
            float(self.scene_center[1]),
            float(self.scene_bounds_min[2]) + self.robot_height,
        ], dtype=float)

        print(f"[usd-scene] referenced {scene_path}")
        print(
            f"[usd-scene] bounds center={self.scene_center.tolist()} "
            f"diag={self.scene_diagonal:.3f}")
        print(
            f"[usd-scene] robot spawn={self.robot_spawn_position.tolist()}")

    def get_robot_spawn_position(self):
        return self.robot_spawn_position

    def get_scene_bounds(self):
        return self.scene_bounds_min, self.scene_bounds_max

    def get_scene_center(self):
        return self.scene_center

    def get_scene_diagonal(self):
        return self.scene_diagonal


if __name__ == "__main__":
    import os
    import sys

    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "enable_cameras": True,
                         "width": 1600, "height": 900})

    import numpy as np
    from isaacsim.core.api import World
    from PIL import Image

    world = World(stage_units_in_meters=1.0)
    env = USDSceneEnvironment()
    env.build(world)
    world.reset()

    output_dir = os.environ.get("OUTPUT_DIR", "/isaac-sim/Documents")
    os.makedirs(output_dir, exist_ok=True)

    preview_views = [
        ("wide", (1.0, -1.0, 0.0)),
        ("corner", (-1.0, -0.55, 0.0)),
        ("reverse", (0.25, 1.0, 0.0)),
    ]

    for name, direction in preview_views:
        camera = FramedCameraRig(
            center=env.get_scene_center(),
            bounds_min=env.get_scene_bounds()[0],
            bounds_max=env.get_scene_bounds()[1],
            prim_path=f"/World/PreviewCameras/{name}",
            resolution=(1600, 900),
            direction=direction,
        )
        camera.attach(world)
        camera.initialize()
        for _ in range(60):
            world.step(render=True)
        rgba = camera.sensor.get_rgba()
        if rgba is not None and rgba.size > 0:
            path = os.path.join(output_dir, f"apartment_preview_{name}.png")
            Image.fromarray(rgba[:, :, :3].astype(np.uint8)).save(path)
            print(f"[preview] wrote {path}")

    app.close()
    print("[preview] DONE. brev cp the PNGs from /isaac-sim/Documents.")
