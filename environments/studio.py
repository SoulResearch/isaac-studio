"""
environments/studio.py
======================
A clean, cinematic studio: a ground plane plus deliberate three-point lighting
(key / fill / rim) and a neutral dome. Loads instantly, costs almost nothing on
disk, and looks far better than a cluttered photoreal room filmed badly.

This is the first environment because great lighting + a good camera angle on a
clean stage is the fastest path to a beautiful video.

Everything here is plain Python building USD prims, so it is fully modifiable:
move a light, change intensity/color, swap the backdrop — all in code.
"""

from __future__ import annotations
import numpy as np


class StudioEnvironment:
    def __init__(self, lighting: str = "cinematic", ground_color=(0.18, 0.18, 0.2)):
        self.lighting = lighting
        self.ground_color = ground_color

    def build(self, world):
        import isaacsim.core.utils.prims as prim_utils
        from pxr import UsdLux, UsdGeom, Gf, UsdShade, Sdf
        import omni.usd

        # Ground plane (physics collision + visual)
        world.scene.add_default_ground_plane()
        stage = omni.usd.get_context().get_stage()

        # --- Lighting ------------------------------------------------------
        # Soft dome for ambient fill
        dome = stage.DefinePrim("/World/Lights/Dome", "DomeLight")
        UsdLux.DomeLight(dome).CreateIntensityAttr(
            300.0 if self.lighting == "cinematic" else 1000.0)

        if self.lighting == "cinematic":
            # Three-point setup. Key (bright, angled front-left), fill (soft,
            # front-right), rim (behind, separates subject from background).
            self._add_distant_light(stage, "/World/Lights/Key",
                                    intensity=3000.0, angle=(-35, 45, 0))
            self._add_distant_light(stage, "/World/Lights/Fill",
                                    intensity=900.0, angle=(-20, -50, 0))
            self._add_distant_light(stage, "/World/Lights/Rim",
                                    intensity=2200.0, angle=(-10, 170, 0))
        else:
            self._add_distant_light(stage, "/World/Lights/Sun",
                                    intensity=2500.0, angle=(-45, 0, 0))

    def _add_distant_light(self, stage, path, intensity, angle):
        from pxr import UsdLux, UsdGeom, Gf
        prim = stage.DefinePrim(path, "DistantLight")
        UsdLux.DistantLight(prim).CreateIntensityAttr(intensity)
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        xform.AddRotateXYZOp().Set(Gf.Vec3f(*angle))
