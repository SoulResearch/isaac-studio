# Previewing Environments in Isaac Sim on Brev

This guide gives you the complete, copy-paste command sequence to render and
view any environment in this folder (e.g. `living_room.py`) as still images
from Isaac Sim running on a Brev L40S instance.

The flow is always the same four phases:

1. Get onto the Brev host
2. Get inside the running Isaac Sim container
3. Run the environment's standalone preview (renders PNGs)
4. Pull the PNGs back to your Mac and open them

---

## Prerequisites (one-time, already done if you've been following along)

- A Brev instance named `isaac-sim-g1` with the persistent, root container
  named `isaac-sim` running (see the repo root README for the `docker run`
  command).
- Isaac Lab installed in the container (`bootstrap/setup_brev.sh`).
- This repo cloned at `/isaac-sim/isaac-studio` inside the container.

If the repo isn't on the container yet, get it there first:

```bash
# inside the container
cd /isaac-sim
git clone <your-repo-url> isaac-studio
```

To update the code after you push changes from your Mac:

```bash
# inside the container
cd /isaac-sim/isaac-studio && git pull
```

---

## Phase 1 — Get onto the Brev host (from your Mac)

```bash
brev shell isaac-sim-g1
```

You should land at a prompt like `shadeform@brev-...`.

---

## Phase 2 — Enter the running container

```bash
docker exec -it isaac-sim bash
```

You should now be at `root@brev-...:/isaac-sim#` (inside the container).

> If `docker exec` says `No such container: isaac-sim`, the container isn't
> running. Start it with the `docker run` command from the repo root README,
> then re-run the `docker exec` line.

---

## Phase 3 — Render the environment preview

```bash
cd /isaac-sim/isaac-studio
/isaac-sim/python.sh environments/living_room.py
```

What happens:
- Isaac Sim boots (~2 minutes the first time — lots of log output is normal).
- The scene is built and three preview angles are rendered: `wide`, `cozy`,
  `hearth`.
- You're looking for these lines near the end:
  ```
  [preview] wrote /isaac-sim/Documents/living_room_preview_wide.png
  [preview] wrote /isaac-sim/Documents/living_room_preview_cozy.png
  [preview] wrote /isaac-sim/Documents/living_room_preview_hearth.png
  [preview] DONE. ...
  ```

> A segfault AFTER the `[preview] DONE` line is the harmless container
> shutdown crash — ignore it. The PNGs are already written.

Confirm the files exist (they land in the volume-mounted folder, which bridges
the container to the host):

```bash
ls -lh /isaac-sim/Documents/living_room_preview_*.png
```

---

## Phase 4 — Pull the PNGs to your Mac and view

Open a **separate terminal tab on your Mac** (a real local prompt — not SSHed
into Brev, or `brev`/`open` won't exist). Then:

```bash
brev cp isaac-sim-g1:~/docker/isaac-sim/data/living_room_preview_wide.png ~/Desktop/living_room_preview_wide.png
brev cp isaac-sim-g1:~/docker/isaac-sim/data/living_room_preview_cozy.png ~/Desktop/living_room_preview_cozy.png
brev cp isaac-sim-g1:~/docker/isaac-sim/data/living_room_preview_hearth.png ~/Desktop/living_room_preview_hearth.png

open ~/Desktop/living_room_preview_wide.png
open ~/Desktop/living_room_preview_cozy.png
open ~/Desktop/living_room_preview_hearth.png
```

That's it — you're looking at the actual RTX-rendered environment.

---

## Why these paths line up

The container writes to `/isaac-sim/Documents`, which is volume-mounted to the
Brev host at `~/docker/isaac-sim/data`. That mount is the bridge: anything
written inside the container at `/isaac-sim/Documents` appears on the host at
`~/docker/isaac-sim/data`, where `brev cp` (run from your Mac) can reach it.
The container's own `logs/` paths are NOT on the host, which is why preview
output is deliberately written to `/isaac-sim/Documents`.

---

## Previewing a DIFFERENT environment

Any environment file with a `if __name__ == "__main__":` standalone preview
block works the same way — just point `python.sh` at it:

```bash
/isaac-sim/python.sh environments/<env_name>.py
```

Then pull `~/docker/isaac-sim/data/<whatever it printed>.png` to your Mac.

---

## Quick reference (the whole sequence, condensed)

```bash
# --- on your Mac ---
brev shell isaac-sim-g1

# --- on the Brev host ---
docker exec -it isaac-sim bash

# --- inside the container ---
cd /isaac-sim/isaac-studio
git pull                                            # if you pushed changes
/isaac-sim/python.sh environments/living_room.py
ls -lh /isaac-sim/Documents/living_room_preview_*.png

# --- in a NEW local Mac terminal tab ---
brev cp isaac-sim-g1:~/docker/isaac-sim/data/living_room_preview_wide.png ~/Desktop/
open ~/Desktop/living_room_preview_wide.png
```

---

## Troubleshooting

| Symptom | Cause / Fix |
|---|---|
| `No such container: isaac-sim` | Container not running. Re-run the `docker run` command from the repo root README. |
| `python.sh: No such file` | You're not in the container, or not at `/isaac-sim`. Re-run Phase 2. |
| Python error during build | Likely a small USD/light API mismatch for your Isaac Sim version. Copy the full traceback and we fix the one line in `living_room.py`. |
| No `.png` files written | Check the run reached `[preview] DONE`. If it errored before that, see the traceback. |
| `brev: command not found` | You're running `brev cp` on the Brev host, not your Mac. Open a fresh local Mac terminal. |
| PNG looks black / underexposed | Lighting intensities need tuning — tell me and I'll adjust the light values in `_build_lighting()`. |  