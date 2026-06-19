#!/bin/bash
# bootstrap/setup_brev.sh
# ========================
# Codifies the one-time environment setup on a fresh Brev L40S instance.
# Assumes the Isaac Sim 5.1.0 container is already running as root and named
# "isaac-sim" (see README for the docker run command).
#
# Run INSIDE the container:
#     bash bootstrap/setup_brev.sh
set -e

echo "[1/4] Installing git (idempotent)..."
apt-get update -qq && apt-get install -y -qq git || true
git --version

echo "[2/4] Cloning Isaac Lab if missing..."
if [ ! -d /isaac-sim/IsaacLab ]; then
    cd /isaac-sim
    git clone https://github.com/isaac-sim/IsaacLab.git
    cd IsaacLab
    ln -sf /isaac-sim _isaac_sim
    ./isaaclab.sh --install
else
    echo "    IsaacLab already present, skipping."
fi

echo "[3/4] Verifying Isaac Lab + CUDA (boots SimulationApp)..."
/isaac-sim/python.sh -c "
from isaacsim import SimulationApp
app = SimulationApp({'headless': True})
import isaaclab, torch
from pxr import Usd
print('ISAAC LAB OK | torch', torch.__version__, '| cuda', torch.cuda.is_available())
app.close()
" 2>/dev/null | tail -2

echo "[4/4] Verifying built-in H1 policy extension is available..."
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

echo "DONE. Run a shot with: /isaac-sim/python.sh shots/h1_walk_studio.py"
