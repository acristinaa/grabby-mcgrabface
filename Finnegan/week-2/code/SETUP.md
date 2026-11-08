# SETUP.md — Phase 0 step by step (macOS / Apple Silicon)

> Goal of this document: from "nothing installed" to "the G1 moves in MuJoCo on my Mac".
> Every step has a **check** — only go on when the check has passed.

---

## Overview: what we do NOT build ourselves

A lot exists already. We build on it instead of starting from zero:

| What | Where from | Use |
|---|---|---|
| G1 robot model (MJCF) | `google-deepmind/mujoco_menagerie` | ready-made, physically tested model |
| Locomotion training recipes | **MuJoCo Playground** | ONLY locomotion. **No G1 manipulation environment** — we build that ourselves (Phase 1). |
| Pre-trained walking policies | `unitree_rl_gym` / `unitree_rl_lab` | can be played back right away, serves as baseline |
| Sim2Real bridge (DDS) | `unitreerobotics/unitree_mujoco` | identical interface sim <-> real robot |
| Terrain generator (stairs, rough ground) | included in `unitree_mujoco` | training environments |
| Teleoperation + data recording | `unitreerobotics/xr_teleoperate` | VLA demonstration data |
| VLA base model | UnifoLM-VLA-0 (Unitree) / OpenVLA | fine-tuning instead of training from scratch |

**The G1 model in Menagerie has been shown to transfer to real hardware with MuJoCo Playground**
— so the Sim2Real path is proven, not just theory.

---

## Step 1 — Basic tools

Check what is already there:

```bash
python3 --version     # 3.10 - 3.12 is ideal. Do NOT use 3.14 (torch/LeRobot/ROS2 are missing).
git --version
```

If Homebrew is missing (package manager for macOS), install it from brew.sh.

**Check:** Both commands print a version.

---

## Step 2 — Project folder + virtual environment

Always work in a venv, so nothing pollutes the system Python.

```bash
cd ~/"Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas"
git clone https://github.com/finneganadam-maker/UnitreeG1-Robot.git && cd UnitreeG1-Robot
/opt/homebrew/bin/python3.12 -m venv .venv.nosync && ln -s .venv.nosync .venv   # 3.12 explicitly! .nosync = not in iCloud
source .venv/bin/activate
python -m pip install --upgrade pip
```

Then put the project files (README.md, PLAN.md, PROGRESS.md, CLAUDE.md, SETUP.md) into this
folder and run `git init`.

**Check:** The terminal prompt shows `(.venv)`.

> **Remember:** In EVERY new terminal session, first run `source .venv/bin/activate`.
> A forgotten activation is the most common cause of "module not found".

---

## Step 3 — Install MuJoCo

```bash
pip install mujoco
```

**Check:**
```bash
python -c "import mujoco; print(mujoco.__version__)"
```

---

## Step 4 — IMPORTANT: the macOS viewer trick

On macOS, **`python -m mujoco.viewer` does not work** reliably. The Mac requires
that rendering runs in the main thread. MuJoCo ships its own launcher for this: **`mjpython`**.

`mjpython` is installed automatically with the `mujoco` package and is a
1:1 replacement for `python`.

First test with the built-in demo model:

```bash
mjpython -m mujoco.viewer
```

**Check:** A window opens. If yes: MuJoCo runs on your Mac.

> When you read `python -m mujoco.viewer ...` anywhere on the internet:
> on the Mac, always turn it into **`mjpython`**.

---

## Step 5 — Get the G1 model (the moment you are waiting for)

Two ways. **Way A is the fastest.**

### Way A — via the Menagerie package
```bash
pip install mujoco-menagerie
python -c "import mujoco_menagerie as mm; mm.prefetch(['unitree_g1']); print('ok')"
```

### Way B — clone the repository (more control, all files visible)
```bash
cd ~/dev/g1-vla-project
git clone https://github.com/google-deepmind/mujoco_menagerie.git
mjpython -m mujoco.viewer --mjcf mujoco_menagerie/unitree_g1/scene.xml
```

**Check:** The Unitree G1 stands in the viewer. You can rotate and zoom with the mouse.
**With this, the core of Phase 0 is reached.**

Things to try in the viewer: pause/start the simulation (space bar), show joints and
contact forces, switch the camera. Drag the robot with the mouse and watch
it fall over — this is the physics that training will use later.

---

## Step 6 — First own script (move the robot)

File `scripts/00_view_g1.py`:

```python
import time
import numpy as np
import mujoco
import mujoco.viewer
import mujoco_menagerie as mm

model = mm.load("unitree_g1")     # loads the scene incl. floor and light
data = mujoco.MjData(model)

print("Degrees of freedom (nq):", model.nq)
print("Actuators (nu):         ", model.nu)

with mujoco.viewer.launch_passive(model, data) as viewer:
    t0 = time.time()
    while viewer.is_running():
        # gentle sine control of ALL actuators, only to illustrate
        t = time.time() - t0
        data.ctrl[:] = 0.2 * np.sin(2.0 * t)
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.002)
```

Start it (always with `mjpython`):
```bash
mjpython scripts/00_view_g1.py
```

**Check:** The robot twitches and falls over. That is CORRECT and expected —
there is no policy yet, only random motion. This is exactly where training starts.

This script shows you the basic structure of EVERY MuJoCo application:
`model` (physics description) + `data` (current state) + `mj_step()` (one time step)
+ `data.ctrl` (motor commands = what the policy will output later).

---

## Step 7 — Look at the locomotion recipe: MuJoCo Playground

This is where the **ready-made training environments** for walking live (rewards, domain randomization,
G1 support). Base for Phase 1.

```bash
pip install playground
# or: git clone https://github.com/google-deepmind/mujoco_playground.git
```

On the Mac this runs only with CPU JAX, so it is **slow**. For **vision-based** RL it is not
slow but **impossible**: the batch renderer (Madrona / MJWarp) is CUDA-only,
and `jax-metal` has been dead since Dec 2025. The real training goes to the cloud.

> **Phase decision (see PLAN.md):** Steps 7-8 are **out of scope** for our goal.
> We cut out locomotion — the G1 already walks out of the box.

**What you should learn here (do not train!):**
- How is the reward function built?
- Which observations does the policy get?
- Which parameters are randomized?

---

## Step 8 — Clone the Unitree repos (baseline policies)

```bash
cd ~/dev/g1-vla-project
git clone https://github.com/unitreerobotics/unitree_mujoco.git
git clone https://github.com/unitreerobotics/unitree_rl_gym.git
```

`unitree_mujoco/config.yaml` says `robot: "go2"` — change it to **`g1`** for us.

> **G1 pitfall:** The G1 uses the message format **`unitree_hg`**, not `unitree_go`
> (that is for the Go2). Many examples on the web are written for the Go2. Adapt them when you copy them,
> or the robot will not react.

`unitree_rl_gym` has pre-trained policies under `deploy/pre_train/g1/motion.pt`.
Goal: **play this ready-made policy back in MuJoCo** — that is your baseline and
your first moment of success with a real, walking model.

**Warning:** The training part of `unitree_rl_gym` needs Isaac Gym → **does not run on macOS**.
Only the deploy/MuJoCo part can be used locally. That is exactly the plan.

---

## Step 9 — Reality check and moving on to Phase 1

After step 8 you have locally:
- a working MuJoCo installation
- the G1 model, visible and controllable
- an understanding of `model` / `data` / `ctrl`
- a ready-made baseline policy as a reference
- the Sim2Real bridge and training recipes at hand

**Only THEN** move on to Phase 1: own training on a cloud GPU, built on the
Playground recipe, with domain randomization from the start.

---

## Common errors on the Mac

| Symptom | Cause / fix |
|---|---|
| Viewer does not open / freezes | used `python` instead of `mjpython` |
| `ModuleNotFoundError` | venv not activated (`source .venv/bin/activate`) |
| Isaac Gym cannot be installed | Expected — it does not exist for macOS. Use the cloud |
| Robot does not react to SDK commands | used `unitree_go` instead of `unitree_hg` |
| Training extremely slow | Normal on CPU. Only test runs locally, training in the cloud |
| Meshes/assets missing | Menagerie not fully cloned (check LFS/prefetch) |

---

## Checklist Phase 0

- [ ] venv active, git initialized, project files in the folder
- [ ] `mujoco` installed, version is printed
- [ ] `mjpython -m mujoco.viewer` opens a window
- [ ] G1 visible in the viewer
- [ ] `00_view_g1.py` runs, robot moves (and falls)
- [ ] nq / nu values noted (base for the observation/action space)
- [ ] MuJoCo Playground G1 recipe read (reward + randomization understood)
- [ ] `unitree_mujoco` configured for `g1`
- [ ] baseline policy played back in MuJoCo
- [ ] entry made in `PROGRESS.md`


---

## Addendum — verified on 2026-09-07 on this Mac

| Check | Result |
|---|---|
| Python | System `python3` is **3.14.3** → not usable. venv built on **3.12.14**. |
| mujoco / menagerie | 3.12.0 / 2026.9.0, installed |
| `mm.load("unitree_g1")` | nq=36, nv=35, **nu=29**, **ncam=0** |
| Offscreen RGB + depth | works on the M4 (the base for Phase 2 is in place) |
| Depth accuracy | ⚠️ `ARB_clip_control unavailable` — limited on macOS |

**Two findings with consequences:**

1. **`ncam=0`** — the Menagerie G1 has **no cameras**. We must add the wrist and head cameras
   ourselves in Phase 1, or the VLM planner has no input.
2. **`nu=29`, all arm/leg/waist** — the hands in the render are **rigid decoration without
   actuators**. Confirmed: the BrainCo Revo2 integration is real work of our own.
