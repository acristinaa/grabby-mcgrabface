# SAP Embodied AI Project

Teaching a **Unitree G1** humanoid (29 DoF, BrainCo Revo2 hands) to **walk up to a table, pick up a volleyball and put it in a
basket** — first in simulation, then on the real robot. The robot's intelligence comes from a **Vision-Language-Action model
(SmolVLA)**: it looks through the head camera, gets an instruction in words, and outputs the arm, hand and waist movements.
Walking and navigation are done by a ready-made controller (GR00T in sim, BotBrain on the robot); the VLA does the grasping.

To train the VLA we need many demonstrations. This repo builds the **simulated demonstrator**: a scripted "teacher" that does
the task reliably under heavy variation, so its demos can teach the VLA.

**Status (2026-10-09), all in simulation:**

| Stage | Result |
|---|---|
| Pick and place at the table, fully randomized, with distractors | **50/50 + 50/50** |
| Walk to the table from random situations (behind it, turned away, obstacles, …) | **24/24 + 24/24**, then 12/12 per refinement |
| Walk + pick and place in **one continuous simulation** | **12/12** |
| Real robot | not yet — see [docs/REAL_ROBOT_AND_VLA.md](docs/REAL_ROBOT_AND_VLA.md) |

**Video:** [`media/videos/walk_pick_place_12of12.mp4`](media/videos/walk_pick_place_12of12.mp4) — 12 walk + pick-and-place
trials in one continuous run.

---

## How one trial works

```
 random start (behind the table, turned away, obstacles, ...)
        │  GR00T walks — our code plans the path (A*, smoothed) and steers (pure pursuit)
        ▼
 checkpoint 0.75 m before the table: align, raise the arms
        │  straight walk-in, forward steps only
        ▼
 dock (5 cm behind the grasp scene's spot: GR00T stands in a crouch)
        │  HANDOVER: GR00T switched off, pelvis held, every joint held
        ▼
 two-hand pick and place: turn to the ball (hands rise) → reach → touch → squeeze → lift
        → turn to the basket → release → arms back → face front → 3 s check: ball in the basket
        │  GR00T takes over again
        ▼
 next trial walks off from here
```

## How to run

Setup (once): [docs/SETUP.md](docs/SETUP.md). Environment `~/dev/g1-ext/.venv`; `PY=~/dev/g1-ext/.venv/bin/python`.

```bash
# Lab View: watch runs live and replay them  →  http://127.0.0.1:8030
$PY scripts/65_labview.py

# 1. Pick and place at the table
$PY scripts/67_volleyball_pick_place.py --trials 10                          # nominal
$PY scripts/67_volleyball_pick_place.py --trials 50 --jitter 0.02 --distractors
$PY scripts/67_volleyball_pick_place.py --trials 50 --randomize --seed 0     # everything varied
PATH=~/dev/g1-ext/.venv/bin:$PATH $PY scripts/67_volleyball_pick_place.py --trials 3 --watch 1   # live, real time

# 2. Walk to the table (live camera + top view at http://127.0.0.1:8020)
DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib PYTHONUNBUFFERED=1 \
  $PY scripts/68_walk_to_table.py --trials 12 --seed 0

# 3. Walk + pick and place in one sim (add --video for an mp4 of the whole run)
DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib PYTHONUNBUFFERED=1 \
  $PY scripts/69_walk_pick_place.py --trials 6 --seed 0

# 4. Replay the handover offline over a grid of dock positions (no walking, seconds per trial)
$PY scripts/70_handover_sweep.py --dx=-0.06,0,0.03 --dy=-0.04,0,0.04 --dyaw=-3,0,3

# Tests
PYTHONPATH=/tmp/stubs $PY -m pytest -q        # 144 passed
```

## Repository

| Path | What |
|---|---|
| `g1sim/ball_grasp.py` | the two-hand pick-and-place teacher (Finnegan's motion + random starts, hand lift, gravity offset) |
| `g1sim/table_env.py`, `table_scene.py` | grasp scene (fixed robot) and its environment: 27-D actions, distractors, randomization |
| `g1sim/randomization.py`, `configs/randomization.json` | what varies per trial and by how much |
| `g1sim/approach.py`, `approach_world.py` | walk-to-table: scenarios, path, checkpoint, straight walk-in; the walk world |
| `g1sim/walk_sim.py`, `navigation.py`, `occupancy.py`, `arm_swing.py` | LeRobot/GR00T walking sim wrapper, path following, map, arm swing |
| `g1sim/walk_grasp.py` | **the handover**: GR00T off, pelvis held, grasp on the same state; offline handover replay |
| `g1sim/labview.py`, `liveview.py`, `video.py` | browser views and videos |
| `scripts/51` | build the simulator (once) |
| `scripts/65` | Lab View server |
| `scripts/66` | volleyball scene check |
| `scripts/67` | pick and place (`--distractors`, `--randomize`, `--arrivals`, `--watch`) |
| `scripts/68` | walk to the table |
| `scripts/69` | walk + pick and place, one sim |
| `scripts/70` | offline handover sweep |
| `tests/` | 144 tests |
| `models/` | Unitree payload (backpack, head module + D455), BrainCo mount, Revo2 XML (meshes: see setup) |
| `docs/` | [RESULTS](docs/RESULTS.md) (every test), [HISTORY](docs/HISTORY.md) (what happened, in order), [SETUP](docs/SETUP.md), [REAL_ROBOT_AND_VLA](docs/REAL_ROBOT_AND_VLA.md) |
| `results/` | JSON of the main runs, named by experiment and result |
| `media/` | video of the 12/12 run, scene images |

Script numbers continue the numbering of the team repo, so results and notes can be matched. Some docstrings mention team-repo
files (`PLAN.md`, `PROGRESS.md`, `docs/LEARNING_PATH.md`); those live in the team repo.

## Next steps
1. Distractors in the continuous walk + pick and place.
2. One randomized demo recorder (start from the handover, clean arm targets, stance + backpack randomization) → SmolVLA.
3. First real-robot session: measure stance, table, camera; test waist control while balancing.

## Credits and licences
- Based on the team project **`finneganadam-maker/UnitreeG1-Robot`** — the simulator setup, the volleyball scene, the
  two-hand teacher and most of `g1sim/` are by **Finnegan Adam**; distractors, randomization in the volleyball scene,
  walking to the table, the continuous walk + grasp and the robustness work are by **Saad Sheikh**. Code written with the help
  of Claude Code.
- Third party: LeRobot (Apache-2.0), MuJoCo, mink, Unitree G1 payload and mount models (licences in `models/*/LICENSE`),
  BrainCo Revo2 models (download from BrainCo, not redistributed).
- Private repository; no licence granted.
