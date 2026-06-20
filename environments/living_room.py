"""
environments/living_room.py
===========================
A warm, golden-hour, upper-middle-class American living room built procedurally
from primitives with carefully tuned PBR materials. The aesthetic target is the
reference: amber/honey tones, soft directional sun raking across wood and
fabric, lived-in but not cluttered. Lighting and material tuning do the heavy
lifting — no texture files, no heavy meshes — so it stays light on compute while
looking borderline-photoreal under Isaac Sim's RTX renderer.

Design decisions (matching the brief):
  * Geometry: procedural primitives (boxes/cylinders) — full control, low cost.
  * Materials: UsdPreviewSurface PBR with hand-tuned albedo/roughness/metallic.
    Richness comes from material + light interaction, not polygon count.
  * Physics: STATIC colliders only (CollisionAPI, no RigidBodyAPI). The robot
    can't walk through walls/furniture, but nothing tips over or shatters.
  * Layout: open center with ample clear floor for a robot to walk among the
    furniture (a ~3.5 x 2.5 m clear zone in the middle).

Two ways to use this file:

  A) As a Lego piece in a shot:
        from environments.living_room import LivingRoomEnvironment
        env = LivingRoomEnvironment()
        env.build(world)

  B) Standalone preview render (see the ACTUAL look before wiring a robot):
        /isaac-sim/python.sh environments/living_room.py
     -> writes preview PNGs to /isaac-sim/Documents/living_room_preview_*.png
        which you brev cp to your Mac.
"""

from __future__ import annotations
import numpy as np


# ----------------------------------------------------------------------------
# Palette — the warm golden-hour upper-middle-class living room.
# (linear-ish RGB 0..1; values chosen for warmth under amber key light)
# ----------------------------------------------------------------------------
PALETTE = {
    "oak_floor":     (0.34, 0.22, 0.12),   # warm honey oak planks
    "rug":           (0.42, 0.20, 0.16),   # deep terracotta/rust area rug
    "wall_warm":     (0.78, 0.70, 0.56),   # warm cream / muted ochre paint
    "ceiling":       (0.88, 0.85, 0.80),   # soft off-white
    "trim":          (0.92, 0.89, 0.83),   # baseboards / window trim
    "sofa":          (0.46, 0.43, 0.33),   # muted olive / sage upholstery
    "armchair":      (0.55, 0.46, 0.30),   # warm tan-gold floral stand-in
    "cushion":       (0.40, 0.26, 0.20),   # rust accent cushion
    "walnut":        (0.20, 0.12, 0.07),   # dark walnut wood (tables/shelves)
    "shelf_back":    (0.26, 0.18, 0.12),   # shelf interior, slightly lighter
    "brick":         (0.45, 0.28, 0.22),   # warm fireplace brick
    "mantel":        (0.30, 0.20, 0.13),   # wood mantel
    "lamp_ceramic":  (0.62, 0.42, 0.22),   # glazed amber ceramic lamp base
    "lamp_shade":    (0.90, 0.78, 0.55),   # warm linen shade (slightly emissive)
    "metal_warm":    (0.55, 0.45, 0.28),   # brushed brass accents
    "art_canvas":    (0.50, 0.34, 0.22),   # framed art, warm abstract
    "plant_pot":     (0.40, 0.30, 0.22),   # terracotta pot
    "plant_green":   (0.20, 0.30, 0.16),   # muted foliage
}


class LivingRoomEnvironment:
    """Procedural golden-hour living room. build(world) drops it into a scene."""

    def __init__(self,
                 room_size=(7.0, 6.0),     # interior floor (x, y) meters
                 wall_height=3.0,
                 mood="golden_hour"):
        self.rw, self.rd = room_size
        self.wh = wall_height
        self.mood = mood
        self._stage = None
        self._materials = {}

    # =====================================================================
    # PUBLIC: build the whole room into `world`
    # =====================================================================
    def build(self, world):
        import omni.usd
        self._stage = omni.usd.get_context().get_stage()

        # Physics ground (invisible collision plane the robot stands on);
        # the visible floor sits on top.
        world.scene.add_default_ground_plane(z_position=0.0)

        self._build_materials()
        self._build_shell()
        self._build_window()
        self._build_fireplace()
        self._build_bookshelf()
        self._build_seating()
        self._build_tables_and_rug()
        self._build_lamp()
        self._build_accents()
        self._build_lighting()

    # =====================================================================
    # MATERIALS — UsdPreviewSurface PBR, tuned per surface
    # =====================================================================
    def _build_materials(self):
        from pxr import UsdShade, Sdf, Gf

        # (name, roughness, metallic, [optional emissive color])
        specs = {
            "oak_floor":    (0.45, 0.0),
            "rug":          (0.95, 0.0),
            "wall_warm":    (0.92, 0.0),
            "ceiling":      (0.95, 0.0),
            "trim":         (0.70, 0.0),
            "sofa":         (0.88, 0.0),
            "armchair":     (0.85, 0.0),
            "cushion":      (0.90, 0.0),
            "walnut":       (0.32, 0.0),
            "shelf_back":   (0.55, 0.0),
            "brick":        (0.96, 0.0),
            "mantel":       (0.40, 0.0),
            "lamp_ceramic": (0.22, 0.0),
            "lamp_shade":   (0.80, 0.0),   # emissive added below
            "metal_warm":   (0.30, 0.85),
            "art_canvas":   (0.85, 0.0),
            "plant_pot":    (0.70, 0.0),
            "plant_green":  (0.80, 0.0),
        }
        mats_scope = "/World/LivingRoom/Looks"
        for name, (rough, metal) in specs.items():
            color = PALETTE[name]
            mat_path = f"{mats_scope}/{name}"
            mat = UsdShade.Material.Define(self._stage, mat_path)
            shader = UsdShade.Shader.Define(self._stage, f"{mat_path}/Shader")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(*color))
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
            shader.CreateInput("useSpecularWorkflow",
                               Sdf.ValueTypeNames.Int).Set(0)
            # The lampshade glows softly so the lamp reads as "on"
            if name == "lamp_shade":
                shader.CreateInput("emissiveColor",
                                   Sdf.ValueTypeNames.Color3f).Set(
                                   Gf.Vec3f(0.95, 0.78, 0.50))
            mat.CreateSurfaceOutput().ConnectToSource(
                shader.ConnectableAPI(), "surface")
            self._materials[name] = mat

    def _bind(self, prim_path, material_name):
        from pxr import UsdShade
        prim = self._stage.GetPrimAtPath(prim_path)
        UsdShade.MaterialBindingAPI(prim).Bind(self._materials[material_name])

    # =====================================================================
    # GEOMETRY HELPERS
    # =====================================================================
    def _box(self, path, center, size, material, collision=True):
        """Axis-aligned box via a unit Cube scaled to `size` (full extents)."""
        from pxr import UsdGeom, Gf, UsdPhysics
        cube = UsdGeom.Cube.Define(self._stage, path)
        cube.GetSizeAttr().Set(1.0)
        x = UsdGeom.Xformable(cube)
        x.ClearXformOpOrder()
        x.AddTranslateOp().Set(Gf.Vec3d(*center))
        x.AddScaleOp().Set(Gf.Vec3f(size[0], size[1], size[2]))
        if collision:
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())  # static collider
        self._bind(path, material)
        return cube

    def _cyl(self, path, center, radius, height, material, collision=True):
        from pxr import UsdGeom, Gf, UsdPhysics
        cyl = UsdGeom.Cylinder.Define(self._stage, path)
        cyl.GetRadiusAttr().Set(radius)
        cyl.GetHeightAttr().Set(height)
        cyl.GetAxisAttr().Set("Z")
        x = UsdGeom.Xformable(cyl)
        x.ClearXformOpOrder()
        x.AddTranslateOp().Set(Gf.Vec3d(*center))
        if collision:
            UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
        self._bind(path, material)
        return cyl

    # =====================================================================
    # ROOM PIECES
    # =====================================================================
    def _build_shell(self):
        R = "/World/LivingRoom"
        hw, hd = self.rw / 2, self.rd / 2
        t = 0.12  # wall thickness

        # Floor (thin slab) + rug on top
        self._box(f"{R}/Floor", (0, 0, -0.02), (self.rw, self.rd, 0.04),
                  "oak_floor")
        # Ceiling
        self._box(f"{R}/Ceiling", (0, 0, self.wh), (self.rw, self.rd, 0.1),
                  "ceiling", collision=False)
        # Walls: back (-y), left (-x), right (+x). Front (+y) left open for camera.
        self._box(f"{R}/Wall_Back", (0, -hd, self.wh / 2),
                  (self.rw, t, self.wh), "wall_warm")
        self._box(f"{R}/Wall_Left", (-hw, 0, self.wh / 2),
                  (t, self.rd, self.wh), "wall_warm")
        self._box(f"{R}/Wall_Right", (hw, 0, self.wh / 2),
                  (t, self.rd, self.wh), "wall_warm")
        # Baseboards (thin warm trim) along the three walls
        bb_h = 0.12
        self._box(f"{R}/Base_Back", (0, -hd + t / 2, bb_h / 2),
                  (self.rw, 0.03, bb_h), "trim", collision=False)
        self._box(f"{R}/Base_Left", (-hw + t / 2, 0, bb_h / 2),
                  (0.03, self.rd, bb_h), "trim", collision=False)
        self._box(f"{R}/Base_Right", (hw - t / 2, 0, bb_h / 2),
                  (0.03, self.rd, bb_h), "trim", collision=False)

    def _build_window(self):
        """A window opening in the right wall — the source of the warm sun.
        Modeled as a bright emissive panel + trim so light reads as streaming in."""
        from pxr import UsdShade, Sdf, Gf, UsdGeom
        R = "/World/LivingRoom"
        hw = self.rw / 2
        # Window glow panel (emissive, warm) just inside the right wall
        win_path = f"{R}/Window/Glow"
        cube = UsdGeom.Cube.Define(self._stage, win_path)
        cube.GetSizeAttr().Set(1.0)
        x = UsdGeom.Xformable(cube)
        x.ClearXformOpOrder()
        x.AddTranslateOp().Set(Gf.Vec3d(hw - 0.07, 0.8, 1.5))
        x.AddScaleOp().Set(Gf.Vec3f(0.02, 1.6, 1.4))
        # dedicated emissive material for the window
        mp = f"{R}/Looks/window_glow"
        mat = UsdShade.Material.Define(self._stage, mp)
        sh = UsdShade.Shader.Define(self._stage, f"{mp}/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(1.0, 0.92, 0.72))
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(1.0, 0.85, 0.55))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(mat)
        # Window trim frame
        self._box(f"{R}/Window/Trim_Top", (hw - 0.08, 0.8, 2.25),
                  (0.04, 1.8, 0.12), "trim", collision=False)
        self._box(f"{R}/Window/Trim_Bot", (hw - 0.08, 0.8, 0.75),
                  (0.04, 1.8, 0.12), "trim", collision=False)

    def _build_fireplace(self):
        """Brick fireplace with a wood mantel on the back wall (left of center)."""
        R = "/World/LivingRoom"
        hd = self.rd / 2
        cx = -1.8
        # Brick surround
        self._box(f"{R}/Fireplace/Surround", (cx, -hd + 0.2, 0.9),
                  (1.6, 0.4, 1.8), "brick")
        # Dark firebox recess (no collision, just visual depth)
        self._box(f"{R}/Fireplace/Firebox", (cx, -hd + 0.25, 0.6),
                  (0.9, 0.3, 0.9), "walnut", collision=False)
        # Wood mantel shelf
        self._box(f"{R}/Fireplace/Mantel", (cx, -hd + 0.28, 1.35),
                  (1.9, 0.45, 0.12), "mantel", collision=False)

    def _build_bookshelf(self):
        """A built-in shelving unit on the back wall (right of fireplace).
        Modeled as a frame + horizontal shelves; NO individual books (per brief)."""
        R = "/World/LivingRoom"
        hd = self.rd / 2
        cx = 0.8
        w, d, h = 2.4, 0.35, 2.6
        y = -hd + d / 2 + 0.02
        # Back panel + sides + top/bottom
        self._box(f"{R}/Shelf/Back", (cx, -hd + 0.05, h / 2),
                  (w, 0.05, h), "shelf_back", collision=False)
        self._box(f"{R}/Shelf/Side_L", (cx - w / 2, y, h / 2),
                  (0.06, d, h), "walnut")
        self._box(f"{R}/Shelf/Side_R", (cx + w / 2, y, h / 2),
                  (0.06, d, h), "walnut")
        self._box(f"{R}/Shelf/Top", (cx, y, h),
                  (w, d, 0.06), "walnut", collision=False)
        # Horizontal shelves (just slabs — the "rich but not cluttered" look)
        for i, z in enumerate(np.linspace(0.5, h - 0.4, 5)):
            self._box(f"{R}/Shelf/Shelf_{i}", (cx, y, float(z)),
                      (w - 0.12, d - 0.04, 0.04), "walnut", collision=False)

    def _build_seating(self):
        """A sofa facing the camera-open side, plus an armchair at an angle.
        Built from a base + back + arms + cushions for readable form."""
        R = "/World/LivingRoom"

        # ---- Sofa (centered-left, facing +y toward the open front) ----
        sx, sy = -0.6, -1.4
        self._box(f"{R}/Sofa/Base", (sx, sy, 0.28), (2.2, 0.95, 0.30), "sofa")
        self._box(f"{R}/Sofa/Back", (sx, sy - 0.42, 0.62),
                  (2.2, 0.18, 0.70), "sofa")
        self._box(f"{R}/Sofa/Arm_L", (sx - 1.05, sy, 0.45),
                  (0.20, 0.95, 0.55), "sofa")
        self._box(f"{R}/Sofa/Arm_R", (sx + 1.05, sy, 0.45),
                  (0.20, 0.95, 0.55), "sofa")
        # Seat + back cushions (accent color, no collision so they read soft)
        for i, dx in enumerate((-0.55, 0.55)):
            self._box(f"{R}/Sofa/Seat_{i}", (sx + dx, sy + 0.05, 0.46),
                      (0.95, 0.85, 0.12), "cushion", collision=False)
            self._box(f"{R}/Sofa/Cush_{i}", (sx + dx, sy - 0.30, 0.66),
                      (0.85, 0.14, 0.55), "cushion", collision=False)

        # ---- Armchair (right side, angled toward the coffee table) ----
        ax, ay = 1.9, -0.3
        self._box(f"{R}/Armchair/Base", (ax, ay, 0.26),
                  (0.95, 0.95, 0.28), "armchair")
        self._box(f"{R}/Armchair/Back", (ax, ay - 0.40, 0.62),
                  (0.95, 0.16, 0.70), "armchair")
        self._box(f"{R}/Armchair/Arm_L", (ax - 0.46, ay, 0.42),
                  (0.16, 0.95, 0.50), "armchair")
        self._box(f"{R}/Armchair/Arm_R", (ax + 0.46, ay, 0.42),
                  (0.16, 0.95, 0.50), "armchair")
        self._box(f"{R}/Armchair/Seat", (ax, ay + 0.04, 0.44),
                  (0.78, 0.80, 0.12), "cushion", collision=False)

    def _build_tables_and_rug(self):
        R = "/World/LivingRoom"
        # Area rug under the seating zone (very thin slab)
        self._box(f"{R}/Rug", (0.1, -0.6, 0.005), (3.4, 2.4, 0.01),
                  "rug", collision=False)

        # Coffee table (centered in front of the sofa) — walnut top + 4 legs
        cx, cy = -0.3, -0.4
        self._box(f"{R}/CoffeeTable/Top", (cx, cy, 0.42),
                  (1.3, 0.7, 0.08), "walnut")
        for i, (dx, dy) in enumerate([(-0.55, -0.28), (0.55, -0.28),
                                      (-0.55, 0.28), (0.55, 0.28)]):
            self._box(f"{R}/CoffeeTable/Leg_{i}",
                      (cx + dx, cy + dy, 0.19), (0.07, 0.07, 0.38),
                      "walnut", collision=False)

        # Side table next to the armchair (holds the lamp)
        self.side_table_top = (2.6, -0.9, 0.55)
        self._box(f"{R}/SideTable/Top", self.side_table_top,
                  (0.55, 0.55, 0.06), "walnut")
        self._cyl(f"{R}/SideTable/Post", (2.6, -0.9, 0.27),
                  0.05, 0.52, "walnut", collision=False)

    def _build_lamp(self):
        """Table lamp on the side table: ceramic base + brass neck + linen shade.
        The shade is softly emissive and an actual warm point light sits inside."""
        R = "/World/LivingRoom"
        from pxr import UsdLux, Gf
        bx, by, bz = 2.6, -0.9, 0.58  # top of side table
        self._cyl(f"{R}/Lamp/Base", (bx, by, bz + 0.10),
                  0.10, 0.20, "lamp_ceramic", collision=False)
        self._cyl(f"{R}/Lamp/Neck", (bx, by, bz + 0.32),
                  0.02, 0.24, "metal_warm", collision=False)
        # Shade (truncated look via a short wide cylinder)
        self._cyl(f"{R}/Lamp/Shade", (bx, by, bz + 0.52),
                  0.17, 0.22, "lamp_shade", collision=False)
        # Warm sphere light inside the shade for the cozy glow
        lamp_light = self._stage.DefinePrim(f"{R}/Lamp/Light", "SphereLight")
        sl = UsdLux.SphereLight(lamp_light)
        sl.CreateIntensityAttr(12000.0)
        sl.CreateRadiusAttr(0.12)
        sl.CreateColorAttr(Gf.Vec3f(1.0, 0.80, 0.52))
        from pxr import UsdGeom
        UsdGeom.Xformable(lamp_light).AddTranslateOp().Set(
            Gf.Vec3d(bx, by, bz + 0.52))

    def _build_accents(self):
        """A few large, low-effort accents for richness: framed art, a tall
        plant, a console. No tiny props."""
        R = "/World/LivingRoom"
        hd = self.rd / 2
        # Framed art above the fireplace mantel
        self._box(f"{R}/Art/Canvas", (-1.8, -hd + 0.12, 2.1),
                  (1.0, 0.05, 0.75), "art_canvas", collision=False)
        self._box(f"{R}/Art/Frame", (-1.8, -hd + 0.10, 2.1),
                  (1.12, 0.06, 0.87), "walnut", collision=False)
        # Tall potted plant in the far corner (pot + simple foliage mass)
        px, py = self.rw / 2 - 0.5, -hd + 0.5
        self._cyl(f"{R}/Plant/Pot", (px, py, 0.25), 0.22, 0.5, "plant_pot")
        self._box(f"{R}/Plant/Foliage", (px, py, 1.1),
                  (0.7, 0.7, 1.2), "plant_green", collision=False)

    # =====================================================================
    # LIGHTING — the soul of the golden-hour look
    # =====================================================================
    def _build_lighting(self):
        from pxr import UsdLux, UsdGeom, Gf
        R = "/World/LivingRoom"

        if self.mood == "golden_hour":
            # Warm ambient sky dome — low, so the room isn't flat
            dome = self._stage.DefinePrim(f"{R}/Light/Dome", "DomeLight")
            d = UsdLux.DomeLight(dome)
            d.CreateIntensityAttr(180.0)
            d.CreateColorAttr(Gf.Vec3f(0.95, 0.85, 0.72))

            # The KEY: a warm low-angle sun streaming through the right window
            sun = self._stage.DefinePrim(f"{R}/Light/Sun", "DistantLight")
            s = UsdLux.DistantLight(sun)
            s.CreateIntensityAttr(2600.0)
            s.CreateColorAttr(Gf.Vec3f(1.0, 0.82, 0.55))
            s.CreateAngleAttr(1.2)  # soft-edged shadows
            sx = UsdGeom.Xformable(sun)
            sx.ClearXformOpOrder()
            # Low from the right (+x), slightly from behind, raking across room
            sx.AddRotateXYZOp().Set(Gf.Vec3f(-18, -62, 0))

            # A big soft rect "window light" to fake bounced sun off the floor
            win = self._stage.DefinePrim(f"{R}/Light/WindowFill", "RectLight")
            w = UsdLux.RectLight(win)
            w.CreateIntensityAttr(900.0)
            w.CreateColorAttr(Gf.Vec3f(1.0, 0.88, 0.66))
            w.CreateWidthAttr(1.6)
            w.CreateHeightAttr(1.4)
            wx = UsdGeom.Xformable(win)
            wx.ClearXformOpOrder()
            wx.AddTranslateOp().Set(Gf.Vec3d(self.rw / 2 - 0.2, 0.8, 1.5))
            wx.AddRotateXYZOp().Set(Gf.Vec3f(0, 90, 0))  # face into room (-x)

            # Cool soft fill from the open front so shadows aren't black
            fill = self._stage.DefinePrim(f"{R}/Light/Fill", "DistantLight")
            f = UsdLux.DistantLight(fill)
            f.CreateIntensityAttr(350.0)
            f.CreateColorAttr(Gf.Vec3f(0.6, 0.68, 0.85))
            fx = UsdGeom.Xformable(fill)
            fx.ClearXformOpOrder()
            fx.AddRotateXYZOp().Set(Gf.Vec3f(-35, 120, 0))


# ============================================================================
# STANDALONE PREVIEW — see the ACTUAL RTX look before wiring up a robot.
#   /isaac-sim/python.sh environments/living_room.py
# Writes preview PNGs to /isaac-sim/Documents/ (brev cp them to your Mac).
# ============================================================================
if __name__ == "__main__":
    import os, sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    from isaacsim import SimulationApp
    # Higher resolution + cameras for a quality still
    app = SimulationApp({"headless": True, "enable_cameras": True,
                         "width": 1600, "height": 900})

    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.sensors.camera import Camera
    import isaacsim.core.utils.numpy.rotations as rot_utils

    world = World(stage_units_in_meters=1.0)
    env = LivingRoomEnvironment()
    env.build(world)

    # A couple of cinematic preview angles looking into the room from the
    # open (+y) front side.
    def look_quat(eye, target):
        f = np.array(target) - np.array(eye)
        f = f / (np.linalg.norm(f) + 1e-9)
        yaw = np.degrees(np.arctan2(f[1], f[0]))
        pitch = np.degrees(np.arcsin(-f[2]))
        return rot_utils.euler_angles_to_quats(
            np.array([0.0, pitch, yaw]), degrees=True)

    angles = [
        ("wide",   (0.2, 4.2, 1.5), (0.0, -1.0, 0.9)),
        ("cozy",   (2.6, 3.0, 1.2), (-0.6, -1.2, 0.8)),
        ("hearth", (-1.0, 3.4, 1.4), (-1.8, -2.6, 1.2)),
    ]

    world.reset()
    out_dir = os.environ.get("OUTPUT_DIR", "/isaac-sim/Documents")
    os.makedirs(out_dir, exist_ok=True)

    from PIL import Image
    for name, eye, target in angles:
        cam = Camera(prim_path=f"/World/PreviewCam_{name}",
                     position=np.array(eye), frequency=30,
                     resolution=(1600, 900),
                     orientation=look_quat(eye, target))
        cam.initialize()
        # Let RTX accumulate / lighting settle for a clean still
        for _ in range(60):
            world.step(render=True)
        rgba = cam.get_rgba()
        if rgba is not None and rgba.size > 0:
            path = os.path.join(out_dir, f"living_room_preview_{name}.png")
            Image.fromarray(rgba[:, :, :3].astype(np.uint8)).save(path)
            print(f"[preview] wrote {path}")

    app.close()
    print("[preview] DONE. brev cp the PNGs from ~/docker/isaac-sim/data/ "
          "to your Mac.")