"""
shots/h1_walk_studio.py
=======================
THE LEGO LAYER. One shot file = one video. This composes existing pieces:
studio environment + H1 (NVIDIA validated policy) + low three-quarter camera,
and records the H1 walking 1 meter forward.

Run from the repo root inside the Brev container:
    /isaac-sim/python.sh shots/h1_walk_studio.py

Output MP4 lands in /isaac-sim/Documents (the mounted folder), which appears on
the Brev host at ~/docker/isaac-sim/data/ for brev cp to your Mac.

To make a NEW video: copy this file, change the env / camera / command lines.
"""

import sys
import os

# Make repo modules importable when run from anywhere
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.stage import Stage

# 1. Boot the simulator FIRST (loads usd / isaac / policy modules)
stage = Stage(headless=True, enable_cameras=True)
stage.boot()

# 2. Imports that require a booted app
from environments.studio import StudioEnvironment
from robots.h1_policy import H1Robot
from cameras.rigs import CameraRig
from core.runner import Runner

# 3. Compose the shot
env = StudioEnvironment(lighting="cinematic")
robot = H1Robot(position=(0.0, 0.0, 1.05))
robot.command(walk_forward=1.0, speed=0.8)      # the "prompt": 1 m forward
camera = CameraRig.preset("low_three_quarter", target=(0.5, 0.0, 1.0))

# 4. Run + record (single boot). Want multiple angles? Use render_multi().
runner = Runner(stage, env, robot, fps=30)
out = runner.record(camera, "h1_walk_studio.mp4", duration_s=8.0)
print(f"[shot] DONE -> {out}")

# 5. Clean shutdown (any segfault after this line is the harmless container one)
stage.shutdown()
