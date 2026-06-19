"""
environments/office.py
=====================
A procedurally-built office space: floor, perimeter walls, and a few desk
blocks, all composed in Python from primitives. This is the "modifiable in
code" environment you asked for — change the layout by editing build().

Kept deliberately light (primitive boxes, not heavy photoreal props) to respect
the cheapest-compute constraint. Swap primitives for furniture USDs later if
you want more detail.
"""

from __future__ import annotations
import numpy as np


class OfficeEnvironment:
    def __init__(self, size=(8.0, 6.0), wall_height=2.5, num_desks=3):
        self.size = size
        self.wall_height = wall_height
        self.num_desks = num_desks

    def build(self, world):
        import isaacsim.core.utils.prims as prim_utils
        from pxr import UsdLux
        import omni.usd

        world.scene.add_default_ground_plane()
        stage = omni.usd.get_context().get_stage()

        w, d = self.size
        h = self.wall_height
        t = 0.1  # wall thickness

        # Perimeter walls as thin boxes
        walls = [
            ("/World/Office/Wall_N", (0, d / 2, h / 2), (w, t, h)),
            ("/World/Office/Wall_S", (0, -d / 2, h / 2), (w, t, h)),
            ("/World/Office/Wall_E", (w / 2, 0, h / 2), (t, d, h)),
            ("/World/Office/Wall_W", (-w / 2, 0, h / 2), (t, d, h)),
        ]
        for path, pos, scale in walls:
            prim_utils.create_prim(path, "Cube", position=pos, scale=scale)

        # A row of desk blocks
        for i in range(self.num_desks):
            x = -w / 3 + i * (w / 3)
            prim_utils.create_prim(
                f"/World/Office/Desk_{i}", "Cube",
                position=(x, -d / 4, 0.4), scale=(1.2, 0.6, 0.75))

        # Overhead lighting
        ceiling = stage.DefinePrim("/World/Office/Light", "DomeLight")
        UsdLux.DomeLight(ceiling).CreateIntensityAttr(800.0)
