# isaac-studio

A composable harness for producing cinematic videos of humanoid robots in
NVIDIA Isaac Sim, running on cheap, ephemeral NVIDIA Brev GPU instances.
Inference only — no training. Uses NVIDIA's built-in, validated H1 locomotion
policy plus procedural, Python-defined environments.

---

## Design principle: everything is reproducible from scratch

Brev instances here are **ephemeral** — they get killed completely, not paused.
Every session starts from nothing. Therefore the **GitHub repo is the only
durable artifact**; the instance is disposable. The whole environment is
rebuilt on each fresh instance by one script. Nothing important lives only on
an instance.

What this means in practice:
- Code lives on GitHub (and your device). Never only on Brev.
- Big binaries (USD assets, policies, MP4 outputs) are NOT in git (see
  `.gitignore`); they're regenerated or pulled on demand.
- The apartment scene lives in `photorealistic_scenes/Apartment/scene_04.usd`
  and is fetched with Git LFS during cold start.
- A cold start is one command and ~14 minutes, mostly the Isaac Sim image pull.

---

## The four Lego layers

- **environments/** — the world (procedural, Python-defined; e.g. `living_room.py`)
- **robots/** — robot + policy (`h1_policy.py` wraps NVIDIA's validated H1 policy)
- **cameras/** — cinematic camera rigs with named presets
- **core/** — the engine: boots Isaac Sim, runs inference, records MP4

A **shot** (`shots/`) is one short Python file composing these into one video.

---

## Repository layout

```
isaac-studio/
├── bootstrap/
│   ├── coldstart.sh        # ONE-COMMAND cold start (run on the Brev host)
│   ├── run_container.sh    # just the docker run (host)
│   └── setup_brev.sh       # container-side: tools + deps + verify
├── core/
│   ├── stage.py            # boots SimulationApp, owns lifecycle
│   ├── recorder.py         # frames -> MP4 in the mounted folder
│   └── runner.py           # orchestrates a shot; multi-camera per boot
├── robots/
│   ├── base_robot.py       # interface for any robot/policy
│   └── h1_policy.py        # NVIDIA built-in validated H1 policy
├── environments/
│   ├── living_room.py      # golden-hour living room (+ standalone preview)
│   ├── studio.py           # clean stage + 3-point lighting
│   ├── office.py           # procedural office
│   ├── usd_scene.py        # default photoreal apartment environment
│   └── README.md           # how to preview an environment
├── photorealistic_scenes/
│   └── Apartment/
│       └── scene_04.usd    # default apartment USD scene (Git LFS)
├── cameras/rigs.py         # CameraRig + cinematic presets
└── shots/
    ├── apartment_walk.py
    ├── h1_walk_studio.py
    └── h1_walk_multiangle.py
```

---

## Prerequisites

- A Brev account and the `brev` CLI installed on your device (`brev login`).
- This repo pushed to GitHub (it is).
- A fresh Brev GPU instance (cheapest L40S is fine). Note its name from
  `brev ls` — used below as `<instance>`.

---

## Cold start (the whole thing, one command)

### 1. Get onto the fresh Brev host (from your device)

```bash
brev ls                        # find your instance name
brev shell <instance>          # land on the host as shadeform@shadecloud
```

### 2. Run the one-command cold start (on the Brev host)

This pulls the Isaac Sim image, starts the container, installs system tools,
clones the repo, pulls the Git LFS apartment assets, installs the MP4 encoder,
and verifies Isaac Sim + the H1 policy:

```bash
curl -sSL https://raw.githubusercontent.com/SoulResearch/isaac-studio/main/bootstrap/coldstart.sh \
  | bash -s -- https://github.com/SoulResearch/isaac-studio.git
```

Expect ~14 minutes the first time (≈12 is the 20GB image pull). You're looking
for `ISAAC SIM OK ...` and `H1 POLICY EXTENSION OK` near the end.

### 3. Enter the container and render

```bash
docker exec -it isaac-sim bash
cd /isaac-sim/isaac-studio
/isaac-sim/python.sh shots/apartment_walk.py
```

### 4. Pull outputs to your device (in a NEW local terminal tab on your device)

```bash
brev cp <instance>:~/docker/isaac-sim/data/apartment_walk.mp4 ~/Desktop/apartment_walk.mp4
open ~/Desktop/apartment_walk.mp4
```

---

## Manual cold start (fallback if you don't want the curl one-liner)

On the Brev host:

```bash
# 1. start the container (creates mounts + docker run)
bash <(curl -sSL https://raw.githubusercontent.com/SoulResearch/isaac-studio/main/bootstrap/run_container.sh)

# 2. enter it
docker exec -it isaac-sim bash

# 3. inside the container: install git, clone, pull LFS assets, run setup
apt-get update && apt-get install -y git git-lfs ca-certificates
cd /isaac-sim && git clone https://github.com/SoulResearch/isaac-studio.git isaac-studio
cd /isaac-sim/isaac-studio && git lfs install && git lfs pull
cd /isaac-sim/isaac-studio && bash bootstrap/setup_brev.sh
```

---

## Critical facts about this container (the gotchas that waste hours)

- **No bare `python`.** Python is `/isaac-sim/python.sh` (Python 3.11).
- **No bare `pip`.** Pip is `/isaac-sim/python.sh -m pip`.
- **`pxr` / USD / isaac modules import ONLY after `SimulationApp()` boots.**
  A bare `python.sh -c "from pxr import Usd"` will ALWAYS fail — that's normal,
  not a broken install. Boot the app first, then import.
- **A fresh container is a bare Ubuntu base** — no git/curl/wget/unzip/vim.
  `setup_brev.sh` installs them; don't assume they're present.
- **The mount bridge:** the container's `/isaac-sim/Documents` is the host's
  `~/docker/isaac-sim/data`. That mount is the ONLY way files cross from the
  container to where `brev cp` (run on your device) can reach them. Anything in
  container-only paths must be copied into `/isaac-sim/Documents` first.
- **Run the container as `-u root`** (apt needs root) and detached with
  `-d -it` and NO `--rm` (so `exit` doesn't delete it).
- **A segfault after work completes** (after the `DONE`/`[preview] DONE` line)
  is the harmless container shutdown crash. Ignore it; outputs are already
  written.

---

## What the bootstrap scripts do (and don't)

`coldstart.sh` + `setup_brev.sh` get you a **working Brev + Isaac Sim
configuration**: container running, system tools present, MP4 encoder
installed, Isaac Sim verified to boot, H1 policy verified to import, repo
cloned. That is the platform.

They do NOT guarantee every line of application code runs first try against
your exact Isaac Sim point release — the environment/robot code was written
against the documented APIs. If a render throws, it's almost always a small,
isolated API mismatch in one file; capture the traceback and fix that line.

Isaac Lab is intentionally NOT installed — it was only needed for training, and
we use NVIDIA's built-in H1 policy now. This is what trims the cold start.

---

## Making a new video / environment

- New environment: copy `environments/usd_scene.py` for USD scenes, or
  `environments/living_room.py` / `studio.py` / `office.py` for procedural
  scenes; keep the standalone preview block to render stills.
- New video: copy a file in `shots/`, swap the env / robot command / camera.
- Default video: `shots/apartment_walk.py`, which uses the apartment USD scene.
- Preview any environment: see `environments/README.md` for the full sequence.

---

## Cost note

Single robot + single camera inference is light; fine on the cheapest L40S.
The only meaningful per-run cost is the ~2 min SimulationApp boot — use
`Runner.render_multi()` to capture several camera angles in one boot.
