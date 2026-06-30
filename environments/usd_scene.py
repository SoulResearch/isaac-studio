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
DEFAULT_SCENE_SCALE = 1.0
DEFAULT_DOME_INTENSITY = 1000.0
DEFAULT_INTERIOR_CAMERA_HEIGHT = 1.6

_INSPECT_KEYWORDS = ("floor", "tiles", "sofa", "wall", "carpet", "table")
_FLOOR_PRIORITIES = ("sm_floor_01", "sm_tiles_01")


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


def _vec3_to_list(values) -> list[float]:
    return [float(values[0]), float(values[1]), float(values[2])]


def _prim_name_matches_keywords(prim) -> bool:
    name = prim.GetName().lower()
    return any(keyword in name for keyword in _INSPECT_KEYWORDS)


def _is_mesh_prim(prim) -> bool:
    return prim.GetTypeName() == "Mesh"


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


def robust_scene_bounds_from_prim(scene_prim):
    """Compute raw and outlier-resistant bounds from all mesh prims."""

    mesh_entries = []

    def _walk(prim):
        if _is_mesh_prim(prim):
            min_vec, max_vec, center, diagonal = scene_bounds_from_prim(prim)
            mesh_entries.append({
                "prim": prim,
                "min_vec": min_vec,
                "max_vec": max_vec,
                "center": center,
                "diagonal": diagonal,
            })
        for child in prim.GetChildren():
            _walk(child)

    _walk(scene_prim)

    if not mesh_entries:
        min_vec, max_vec, center, diagonal = scene_bounds_from_prim(scene_prim)
        return {
            "raw_min": min_vec,
            "raw_max": max_vec,
            "raw_center": center,
            "raw_diagonal": diagonal,
            "filtered_min": min_vec,
            "filtered_max": max_vec,
            "filtered_center": center,
            "filtered_diagonal": diagonal,
            "strategy": "fallback_root",
            "mesh_count": 0,
            "filtered_count": 0,
        }

    raw_min = np.min([entry["min_vec"] for entry in mesh_entries], axis=0)
    raw_max = np.max([entry["max_vec"] for entry in mesh_entries], axis=0)
    raw_center = (raw_min + raw_max) / 2.0
    raw_diagonal = float(np.linalg.norm(raw_max - raw_min))

    centers = np.stack([entry["center"] for entry in mesh_entries], axis=0)
    percentile_min = np.percentile(centers, 5.0, axis=0)
    percentile_max = np.percentile(centers, 95.0, axis=0)
    percentile_mask = np.all(
        (centers >= percentile_min) & (centers <= percentile_max), axis=1)
    filtered_entries = [
        entry for entry, keep in zip(mesh_entries, percentile_mask) if keep
    ]
    strategy = "percentile_5_95"

    if len(filtered_entries) < max(3, int(len(mesh_entries) * 0.1)):
        median = np.median(centers, axis=0)
        sigma = np.std(centers, axis=0)
        sigma = np.where(sigma < 1e-6, 1e-6, sigma)
        sigma_mask = np.all(np.abs(centers - median) <= (3.0 * sigma), axis=1)
        sigma_entries = [
            entry for entry, keep in zip(mesh_entries, sigma_mask) if keep
        ]
        if sigma_entries:
            filtered_entries = sigma_entries
            strategy = "median_3sigma"

    if not filtered_entries:
        filtered_entries = mesh_entries
        strategy = "all_meshes"

    filtered_min = np.min([entry["min_vec"] for entry in filtered_entries], axis=0)
    filtered_max = np.max([entry["max_vec"] for entry in filtered_entries], axis=0)
    filtered_center = (filtered_min + filtered_max) / 2.0
    filtered_diagonal = float(np.linalg.norm(filtered_max - filtered_min))

    return {
        "raw_min": raw_min,
        "raw_max": raw_max,
        "raw_center": raw_center,
        "raw_diagonal": raw_diagonal,
        "filtered_min": filtered_min,
        "filtered_max": filtered_max,
        "filtered_center": filtered_center,
        "filtered_diagonal": filtered_diagonal,
        "strategy": strategy,
        "mesh_count": len(mesh_entries),
        "filtered_count": len(filtered_entries),
    }


def find_floor_prim(scene_prim, floor_prim_name: str = "SM_floor_01_0"):
    """Return the best floor-like prim under the referenced scene root."""

    candidates = []
    target_name = floor_prim_name.lower()

    def _walk(prim):
        prim_name = prim.GetName().lower()
        prim_path = str(prim.GetPath()).lower()
        exact_priority = 0 if (
            target_name == prim_name or target_name in prim_path
        ) else 1 if any(token in prim_name for token in _FLOOR_PRIORITIES) else 2 if any(
            keyword in prim_name for keyword in ("floor", "tiles")
        ) else None
        if exact_priority is not None:
            min_vec, max_vec, center, diagonal = scene_bounds_from_prim(prim)
            candidates.append({
                "prim": prim,
                "priority": exact_priority,
                "max_z": float(max_vec[2]),
                "min_vec": min_vec,
                "max_vec": max_vec,
                "center": center,
                "diagonal": diagonal,
            })
        for child in prim.GetChildren():
            _walk(child)

    _walk(scene_prim)

    if not candidates:
        return None

    candidates.sort(key=lambda item: (item["priority"], -item["max_z"]))
    return candidates[0]


def _print_prim_hierarchy(prim, max_depth: int = 2, depth: int = 0):
    indent = "  " * depth
    min_vec, max_vec, center, diagonal = scene_bounds_from_prim(prim)
    print(
        f"[inspect] {indent}{prim.GetPath()} | {prim.GetTypeName() or 'Unknown'} "
        f"| bbox min={_vec3_to_list(min_vec)} max={_vec3_to_list(max_vec)} "
        f"center={_vec3_to_list(center)} diag={diagonal:.3f}"
    )
    if depth >= max_depth:
        return
    for child in prim.GetChildren():
        _print_prim_hierarchy(child, max_depth=max_depth, depth=depth + 1)


def print_scene_diagnostics(stage, scene_prim_path: str, max_depth: int = 2):
    scene_prim = stage.GetPrimAtPath(scene_prim_path)
    if not scene_prim or not scene_prim.IsValid():
        raise RuntimeError(f"Scene prim does not exist: {scene_prim_path}")

    print(f"[inspect] scene root: {scene_prim_path}")
    _print_prim_hierarchy(scene_prim, max_depth=max_depth)
    bounds = robust_scene_bounds_from_prim(scene_prim)
    print(
        f"[inspect] raw mesh bounds min={_vec3_to_list(bounds['raw_min'])} "
        f"max={_vec3_to_list(bounds['raw_max'])} "
        f"center={_vec3_to_list(bounds['raw_center'])} "
        f"diag={bounds['raw_diagonal']:.3f} "
        f"meshes={bounds['mesh_count']}"
    )
    print(
        f"[inspect] filtered mesh bounds ({bounds['strategy']}) min={_vec3_to_list(bounds['filtered_min'])} "
        f"max={_vec3_to_list(bounds['filtered_max'])} "
        f"center={_vec3_to_list(bounds['filtered_center'])} "
        f"diag={bounds['filtered_diagonal']:.3f} "
        f"meshes={bounds['filtered_count']}"
    )

    print("[inspect] keyword matches:")
    found_match = False

    def _walk_for_matches(prim):
        nonlocal found_match
        if _prim_name_matches_keywords(prim):
            found_match = True
            min_vec, max_vec, center, diagonal = scene_bounds_from_prim(prim)
            print(
                f"[inspect] MATCH {prim.GetPath()} | {prim.GetTypeName() or 'Unknown'} "
                f"| bbox min={_vec3_to_list(min_vec)} max={_vec3_to_list(max_vec)} "
                f"center={_vec3_to_list(center)} diag={diagonal:.3f}"
            )
        for child in prim.GetChildren():
            _walk_for_matches(child)

    _walk_for_matches(scene_prim)
    if not found_match:
        print("[inspect] (no floor/tiles/sofa/wall/carpet/table matches found)")


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
                 direction=(1.0, -1.0, 0.0),
                 camera_mode: str = "auto",
                 floor_top_z: float | None = None,
                 camera_position=None,
                 camera_target=None,
                 fov_degrees: float | None = None):
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
        self.camera_mode = camera_mode
        self.floor_top_z = None if floor_top_z is None else float(floor_top_z)
        self.camera_position = None if camera_position is None else _as_vec3(
            camera_position)
        self.camera_target = None if camera_target is None else _as_vec3(
            camera_target)
        self.fov_degrees = None if fov_degrees is None else float(fov_degrees)
        if self.fov_degrees is not None:
            self.focal_length = float(
                self.horizontal_aperture /
                (2.0 * np.tan(np.radians(self.fov_degrees) / 2.0))
            )
        self.sensor = None

    def attach(self, world):
        from isaacsim.sensors.camera import Camera
        import omni.usd

        if self.camera_position is not None and self.camera_target is not None:
            eye = self.camera_position.copy()
            target = self.camera_target.copy()
        elif self.camera_mode == "interior":
            eye = self.center.copy()
            if self.floor_top_z is None:
                eye[2] = self.center[2]
            else:
                eye[2] = self.floor_top_z + DEFAULT_INTERIOR_CAMERA_HEIGHT
            target = eye + _as_vec3(self.direction)
            target[2] = eye[2]
        else:
            eye = camera_eye_from_bounds(
                self.center,
                self.bounds_min,
                self.bounds_max,
                pullback_scale=self.pullback_scale,
                height_scale=self.height_scale,
                direction=self.direction,
            )
            target = self.center

        quat = look_at_quat(eye, target)
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
                 robot_height: float = DEFAULT_ROBOT_HEIGHT,
                 scene_scale: float = DEFAULT_SCENE_SCALE,
                 floor_prim_name: str = "SM_floor_01_0",
                 add_ground_plane: bool = False,
                 add_dome_light: bool = True,
                 dome_intensity: float = DEFAULT_DOME_INTENSITY):
        self.scene_path = str(scene_path)
        self.scene_prim_path = scene_prim_path
        self.robot_height = float(robot_height)
        self.scene_scale = float(scene_scale)
        self.floor_prim_name = floor_prim_name
        self.add_ground_plane = bool(add_ground_plane)
        self.add_dome_light = bool(add_dome_light)
        self.dome_intensity = float(dome_intensity)
        self._stage = None
        self._scene_prim = None
        self.raw_scene_bounds_min = None
        self.raw_scene_bounds_max = None
        self.raw_scene_center = None
        self.raw_scene_diagonal = None
        self.scene_bounds_min = None
        self.scene_bounds_max = None
        self.scene_center = None
        self.scene_diagonal = None
        self.floor_prim_path = None
        self.floor_bounds_min = None
        self.floor_bounds_max = None
        self.floor_center = None
        self.floor_top_z = None
        self.robot_spawn_position = None

    def build(self, world):
        import omni.usd
        from pxr import UsdGeom, Gf, UsdLux

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

        if abs(self.scene_scale - 1.0) > 1e-9:
            UsdGeom.Xformable(self._scene_prim).AddScaleOp().Set(
                Gf.Vec3f(self.scene_scale, self.scene_scale, self.scene_scale))

        robust_bounds = robust_scene_bounds_from_prim(self._scene_prim)
        self.raw_scene_bounds_min = robust_bounds["raw_min"]
        self.raw_scene_bounds_max = robust_bounds["raw_max"]
        self.raw_scene_center = robust_bounds["raw_center"]
        self.raw_scene_diagonal = robust_bounds["raw_diagonal"]
        self.scene_bounds_min = robust_bounds["filtered_min"]
        self.scene_bounds_max = robust_bounds["filtered_max"]
        self.scene_center = robust_bounds["filtered_center"]
        self.scene_diagonal = robust_bounds["filtered_diagonal"]

        floor_info = find_floor_prim(self._scene_prim, self.floor_prim_name)
        if floor_info is not None:
            self.floor_prim_path = str(floor_info["prim"].GetPath())
            self.floor_bounds_min = floor_info["min_vec"]
            self.floor_bounds_max = floor_info["max_vec"]
            self.floor_center = floor_info["center"]
            self.floor_top_z = float(floor_info["max_z"])
        else:
            self.floor_prim_path = None
            self.floor_bounds_min = None
            self.floor_bounds_max = None
            self.floor_center = self.scene_center
            self.floor_top_z = float(self.scene_bounds_min[2])

        self.robot_spawn_position = np.array([
            -2.5,
            0.5,
            float(self.floor_top_z) + 0.05,
        ], dtype=float)

        if self.add_ground_plane:
            # Optional support plane for scenes that need a fallback collider.
            world.scene.add_default_ground_plane(
                z_position=float(self.scene_bounds_min[2]) - 0.02)

        if self.add_dome_light:
            dome_prim = self._stage.DefinePrim("/World/Lights/ApartmentDome",
                                               "DomeLight")
            dome = UsdLux.DomeLight(dome_prim)
            dome.CreateIntensityAttr(self.dome_intensity)
            dome.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))

        print(f"[usd-scene] referenced {scene_path}")
        print(
            f"[usd-scene] raw mesh bounds center={self.raw_scene_center.tolist()} "
            f"diag={self.raw_scene_diagonal:.3f}")
        print(
            f"[usd-scene] filtered mesh bounds center={self.scene_center.tolist()} "
            f"diag={self.scene_diagonal:.3f} scale={self.scene_scale:.4f}")
        if self.floor_prim_path is not None:
            print(
                f"[usd-scene] floor prim={self.floor_prim_path} "
                f"top_z={self.floor_top_z:.3f} "
                f"bbox min={self.floor_bounds_min.tolist()} "
                f"max={self.floor_bounds_max.tolist()}")
        else:
            print(
                f"[usd-scene] floor prim=<fallback scene bbox min-z> "
                f"top_z={self.floor_top_z:.3f}")
        print(
            f"[usd-scene] robot spawn={self.robot_spawn_position.tolist()}")

    def get_robot_spawn_position(self):
        return self.robot_spawn_position

    def get_scene_bounds(self):
        return self.scene_bounds_min, self.scene_bounds_max

    def get_raw_scene_bounds(self):
        return self.raw_scene_bounds_min, self.raw_scene_bounds_max

    def get_scene_center(self):
        return self.scene_center

    def get_scene_diagonal(self):
        return self.scene_diagonal

    def get_floor_prim_path(self):
        return self.floor_prim_path

    def get_floor_center(self):
        return self.floor_center

    def get_floor_top_z(self):
        return self.floor_top_z

    def inspect(self, max_depth: int = 2):
        if self._stage is None or self._scene_prim is None:
            raise RuntimeError("Environment must be built before inspect().")
        print_scene_diagnostics(self._stage, self.scene_prim_path,
                                max_depth=max_depth)


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
