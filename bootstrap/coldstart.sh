#!/bin/bash
# bootstrap/coldstart.sh
# ======================
# ONE-COMMAND cold start for a fresh, ephemeral Brev instance. Runs on the
# Brev HOST. Does the whole sequence: mount dirs -> container -> git + repo
# clone inside -> minimal setup + verify. After this, you only run renders.
#
# Usage (on a fresh Brev host), passing your repo URL as the first argument:
#
#   curl -sSL https://raw.githubusercontent.com/<you>/<repo>/main/bootstrap/coldstart.sh \
#     | bash -s -- https://github.com/<you>/<repo>.git
#
# Or, if you've already cloned the repo to the host somehow:
#   bash bootstrap/coldstart.sh https://github.com/<you>/<repo>.git
set -e

REPO_URL="${1:-${REPO_URL:-}}"
CONTAINER=isaac-sim
IMAGE=nvcr.io/nvidia/isaac-sim:5.1.0

if [ -z "${REPO_URL}" ]; then
    echo "ERROR: pass your GitHub repo URL as the first argument."
    echo "  bash coldstart.sh https://github.com/<you>/isaac-studio.git"
    exit 1
fi

echo "==> [1/4] Host mount directories"
mkdir -p ~/docker/isaac-sim/cache/main \
         ~/docker/isaac-sim/cache/computecache \
         ~/docker/isaac-sim/config \
         ~/docker/isaac-sim/data \
         ~/docker/isaac-sim/logs

echo "==> [2/4] Isaac Sim container (image pull ~12 min on a fresh instance)"
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    echo "    Container exists; (re)starting."
    docker start "${CONTAINER}" >/dev/null 2>&1 || true
else
    docker run --name "${CONTAINER}" -u root --entrypoint bash -d -it --gpus all \
      -e "ACCEPT_EULA=Y" -e "PRIVACY_CONSENT=Y" --network=host \
      -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
      -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
      -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
      -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
      -v ~/docker/isaac-sim/data:/isaac-sim/Documents:rw \
      "${IMAGE}"
fi

# Wait until the container accepts exec
echo "    Waiting for container to be ready..."
for i in $(seq 1 30); do
    if docker exec "${CONTAINER}" true 2>/dev/null; then break; fi
    sleep 2
done

echo "==> [3/4] Clone repo + run setup inside the container"
# git + ca-certificates are the minimum needed to clone over HTTPS. The full
# common tool set (curl, wget, unzip, vim, ...) is installed by setup_brev.sh.
docker exec "${CONTAINER}" bash -c "
set -e
apt-get update -qq && apt-get install -y -qq git git-lfs ca-certificates
update-ca-certificates 2>/dev/null || true
if [ ! -d /isaac-sim/isaac-studio ]; then
    cd /isaac-sim && git clone ${REPO_URL} isaac-studio
else
    cd /isaac-sim/isaac-studio && git pull --ff-only || true
fi
cd /isaac-sim/isaac-studio
git lfs install
git lfs pull
bash bootstrap/setup_brev.sh
"

echo "==> [4/4] Cold start complete."
echo ""
echo "Next, enter the container and render:"
echo "    docker exec -it ${CONTAINER} bash"
echo "    cd /isaac-sim/isaac-studio"
echo "    /isaac-sim/python.sh shots/apartment_walk.py"
