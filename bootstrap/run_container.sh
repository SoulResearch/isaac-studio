#!/bin/bash
# bootstrap/run_container.sh
# ==========================
# Starts the Isaac Sim 5.1.0 container on a fresh Brev HOST. Version-controlled
# so the exact docker run never has to be copy-pasted from a README again.
#
# Run on the Brev host (NOT inside a container):
#     bash bootstrap/run_container.sh
#
# Idempotent-ish: if a container named isaac-sim already exists, it is reused.
set -e

CONTAINER=isaac-sim
IMAGE=nvcr.io/nvidia/isaac-sim:5.1.0

if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    echo "Container '${CONTAINER}' already exists. Starting it if stopped..."
    docker start "${CONTAINER}" >/dev/null 2>&1 || true
    exit 0
fi

echo "Creating mount directories on host..."
mkdir -p ~/docker/isaac-sim/cache/main \
         ~/docker/isaac-sim/cache/computecache \
         ~/docker/isaac-sim/config \
         ~/docker/isaac-sim/data \
         ~/docker/isaac-sim/logs

echo "Pulling + starting Isaac Sim container (image pull ~12 min first time)..."
docker run --name "${CONTAINER}" -u root --entrypoint bash -d -it --gpus all \
  -e "ACCEPT_EULA=Y" -e "PRIVACY_CONSENT=Y" --network=host \
  -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
  -v ~/docker/isaac-sim/data:/isaac-sim/Documents:rw \
  "${IMAGE}"

echo "Container '${CONTAINER}' is up."
