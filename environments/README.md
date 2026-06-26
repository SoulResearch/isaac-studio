# Previewing the default USD apartment scene in Isaac Sim on Brev

The default environment for this repo is the photoreal USD apartment scene
referenced by `environments/usd_scene.py`. It loads
`photorealistic_scenes/Apartment/scene_04.usd`, which is tracked with Git LFS.
That means a fresh clone or cold start must run `git lfs pull` before the
scene will be usable. The bootstrap scripts already do this automatically.

The flow is always the same four phases:

1. Get onto the Brev host
2. Get inside the running Isaac Sim container
3. Run the apartment preview script
4. Pull the PNGs back to your device and open them

---

## Prerequisites

- A Brev instance with the persistent, root container named `isaac-sim`
  running.
- This repo cloned at `/isaac-sim/isaac-studio` inside the container.
- Git LFS assets pulled inside the repo:

```bash
cd /isaac-sim/isaac-studio
git lfs install
git lfs pull
```

If the repo isn’t on the container yet, get it there first:

```bash
# inside the container
cd /isaac-sim
git clone <your-repo-url> isaac-studio
cd /isaac-sim/isaac-studio && git lfs install && git lfs pull
```

---

## Phase 1 — Get onto the Brev host

From your device:

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

> If `docker exec` says `No such container: isaac-sim`, the container isn’t
> running. Start it with the `docker run` command from the repo root README,
> then re-run the `docker exec` line.

---

## Phase 3 — Render the apartment preview

```bash
cd /isaac-sim/isaac-studio
/isaac-sim/python.sh environments/usd_scene.py
```

What happens:
- Isaac Sim boots.
- The apartment scene is referenced onto the stage.
- Three preview angles are rendered from the scene bounding box: `wide`,
  `corner`, and `reverse`.
- You’re looking for these lines near the end:
  ```
  [preview] wrote /isaac-sim/Documents/apartment_preview_wide.png
  [preview] wrote /isaac-sim/Documents/apartment_preview_corner.png
  [preview] wrote /isaac-sim/Documents/apartment_preview_reverse.png
  [preview] DONE. ...
  ```

> A segfault AFTER the `[preview] DONE` line is the harmless container
> shutdown crash — ignore it. The PNGs are already written.

Confirm the files exist:

```bash
ls -lh /isaac-sim/Documents/apartment_preview_*.png
```

---

## Phase 4 — Pull the PNGs to your device and view

Open a separate terminal on your device. Then:

```bash
brev cp isaac-sim-g1:~/docker/isaac-sim/data/apartment_preview_wide.png ~/Desktop/apartment_preview_wide.png
brev cp isaac-sim-g1:~/docker/isaac-sim/data/apartment_preview_corner.png ~/Desktop/apartment_preview_corner.png
brev cp isaac-sim-g1:~/docker/isaac-sim/data/apartment_preview_reverse.png ~/Desktop/apartment_preview_reverse.png

open ~/Desktop/apartment_preview_wide.png
open ~/Desktop/apartment_preview_corner.png
open ~/Desktop/apartment_preview_reverse.png
```

That’s it — you’re looking at the actual RTX-rendered apartment scene.

---

## Why these paths line up

The container writes to `/isaac-sim/Documents`, which is volume-mounted to the
Brev host at `~/docker/isaac-sim/data`. That mount is the bridge: anything
written inside the container at `/isaac-sim/Documents` appears on the host at
`~/docker/isaac-sim/data`, where `brev cp` can reach it.

---

## Previewing a different environment

The procedural environments still work the same way — just point `python.sh`
at their file:

```bash
/isaac-sim/python.sh environments/living_room.py
/isaac-sim/python.sh environments/studio.py
/isaac-sim/python.sh environments/office.py
```

The new default video shot for the apartment scene is:

```bash
/isaac-sim/python.sh shots/apartment_walk.py
```

