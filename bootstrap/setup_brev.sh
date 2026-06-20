#!/bin/bash
# bootstrap/setup_brev.sh
# ========================
# Container-side setup. Runs INSIDE the isaac-sim container. Minimal now:
# Isaac Lab is GONE (we use NVIDIA's built-in H1 policy + procedural envs,
# both of which ship in the base Isaac Sim container).
#
# A fresh Isaac Sim container is a bare Ubuntu base: no git, curl, wget, unzip,
# vim. There is NO bare `python` or `pip` either -- Python is /isaac-sim/python.sh
# and pip is `/isaac-sim/python.sh -m pip`. This script installs the common
# system tools + the MP4 encoder, then verifies the two things we rely on.
#
# Usually invoked for you by bootstrap/coldstart.sh. To run by hand:
#     bash bootstrap/setup_brev.sh
set -e

echo "[1/4] Installing common system tools (git curl wget unzip vim ca-certificates)..."
apt-get update -qq
apt-get install -y -qq git curl wget unzip vim ca-certificates
update-ca-certificates 2>/dev/null || true
git --version

echo "[2/4] Installing MP4 encoder backend (note: pip is python.sh -m pip)..."
/isaac-sim/python.sh -m pip install --quiet imageio imageio-ffmpeg pillow || true

echo "[3/4] Verifying Isaac Sim boots + USD loads (one SimulationApp boot)..."
/isaac-sim/python.sh -c "
from isaacsim import SimulationApp
app = SimulationApp({'headless': True})
import torch
from pxr import Usd
print('ISAAC SIM OK | torch', torch.__version__, '| cuda', torch.cuda.is_available())
app.close()
" 2>/dev/null | tail -2

echo "[4/4] Verifying built-in H1 policy extension imports..."
/isaac-sim/python.sh -c "
from isaacsim import SimulationApp
app = SimulationApp({'headless': True})
try:
    from isaacsim.robot.policy.examples.robots.h1 import H1FlatTerrainPolicy
    print('H1 POLICY EXTENSION OK')
except Exception as e:
    print('H1 POLICY IMPORT FAILED:', e)
app.close()
" 2>/dev/null | tail -1

echo "DONE. Preview an env:  /isaac-sim/python.sh environments/living_room.py"
