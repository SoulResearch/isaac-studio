# isaac-studio

A composable harness for producing cinematic videos of humanoid robots walking
in Isaac Sim, using NVIDIA's built-in validated locomotion policy. Inference
only — no training. Built to run on the cheapest Brev L40S instance.

## The idea

Four swappable layers snap together like Lego:

- **environments/** — the world (clean studio, procedural office, ...)
- **robots/** — robot + policy (H1 with NVIDIA's validated flat-terrain policy)
- **cameras/** — cinematic camera rigs with named presets
- **core/** — the engine that boots the sim, runs inference, records MP4

A **shot** (in `shots/`) is one short Python file that composes these into one
video. New video = new shot file. The engine in `core/` never changes.

## Layout

```
isaac-studio/
├── bootstrap/setup_brev.sh     # one-time container + Isaac Lab setup
├── core/
│   ├── stage.py                # boots SimulationApp, owns lifecycle
│   ├── recorder.py             # frames -> MP4 in the mounted folder
│   └── runner.py               # orchestrates a shot; multi-camera per boot
├── robots/
│   ├── base_robot.py           # interface for any robot/policy
│   └── h1_policy.py            # NVIDIA built-in validated H1 policy
├── environments/
│   ├── studio.py               # clean stage + cinematic 3-point lighting
│   ├── office.py               # procedural office (walls, desks)
│   └── living_room.py          # warm procedural living room set
├── cameras/rigs.py             # CameraRig + presets
└── shots/
    ├── h1_walk_studio.py       # H1 walks 1m, three-quarter cam
    └── h1_walk_multiangle.py   # same walk, 3 angles, one boot
```

## Workflow (three locations)

- **Local Mac** — edit shot files / modules, `git push`, receive MP4s.
- **GitHub** — holds the code (not the big USD/policy/MP4 binaries; see .gitignore).
- **Brev container** — `git pull`, run a shot, MP4 lands in the mounted folder.

### One-time Brev setup

Start the persistent container as root (so apt/git work, survives `exit`):

```bash
docker run --name isaac-sim -u root --entrypoint bash -d -it --gpus all \
  -e "ACCEPT_EULA=Y" -e "PRIVACY_CONSENT=Y" --network=host \
  -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
  -v ~/docker/isaac-sim/data:/isaac-sim/Documents:rw \
  nvcr.io/nvidia/isaac-sim:5.1.0

docker exec -it isaac-sim bash
cd /isaac-sim && git clone <your-fork-or-repo-url> isaac-studio
cd isaac-studio && bash bootstrap/setup_brev.sh
```

### Run a shot

```bash
cd /isaac-sim/isaac-studio
/isaac-sim/python.sh shots/h1_walk_studio.py
```

### Pull the video to your Mac

```bash
# from your LOCAL Mac terminal
brev cp isaac-sim-g1:~/docker/isaac-sim/data/h1_walk_studio.mp4 ~/Desktop/h1_walk_studio.mp4
open ~/Desktop/h1_walk_studio.mp4
```

## Making a new video

Copy a shot file and change the composition:

```python
env = OfficeEnvironment()                       # swap the world
robot = H1Robot(); robot.command(walk_forward=2.0, speed=1.0)
camera = CameraRig.preset("hero_front", target=(1.0, 0, 1.0))
Runner(stage, env, robot).record(camera, "my_new_shot.mp4", duration_s=10)
```

## Notes / honest caveats

- **Policy:** NVIDIA's built-in `H1FlatTerrainPolicy`. Validated, no training,
  no checkpoint-mismatch risk. It only does velocity-commanded flat walking;
  stairs/manipulation would need a different policy (a future Lego piece).
- **Class/method names** in `robots/h1_policy.py` follow the Isaac Sim 5.1
  example extension. If a future Isaac Sim release renames them, only that one
  file needs updating.
- **Boot cost:** every run boots SimulationApp (~2 min). Use `render_multi()`
  to record several camera angles per boot.
- **Compute:** single robot + single camera inference is light; fine on the
  cheapest L40S. Keep environments primitive to stay within disk limits.
