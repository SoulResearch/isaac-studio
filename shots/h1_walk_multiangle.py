"""
shots/h1_walk_multiangle.py
===========================
Demonstrates the efficiency feature: record the SAME walk from THREE camera
angles in a SINGLE SimulationApp boot (instead of paying the ~2 min boot cost
per angle). Great for iterating on cinematography.

Run:
    /isaac-sim/python.sh shots/h1_walk_multiangle.py
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.stage import Stage
stage = Stage(headless=True, enable_cameras=True)
stage.boot()

from environments.studio import StudioEnvironment
from robots.h1_policy import H1Robot
from cameras.rigs import CameraRig
from core.runner import Runner

env = StudioEnvironment(lighting="cinematic")
robot = H1Robot(position=(0.0, 0.0, 1.05))
robot.command(walk_forward=1.5, speed=0.8)

target = (0.75, 0.0, 1.0)
jobs = [
    (CameraRig.preset("low_three_quarter", target=target), "h1_three_quarter.mp4"),
    (CameraRig.preset("hero_front",        target=target), "h1_hero_front.mp4"),
    (CameraRig.preset("tracking_dolly",    target=target), "h1_dolly.mp4"),
]

runner = Runner(stage, env, robot, fps=30)
outs = runner.render_multi(jobs, duration_s=8.0)
print("[shot] DONE:")
for o in outs:
    print("   ", o)

stage.shutdown()
