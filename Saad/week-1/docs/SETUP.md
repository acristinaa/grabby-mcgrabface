# Setup (macOS, Apple Silicon)

Everything runs on a MacBook (M-series, no CUDA). VLA training does **not** run here — it needs a cloud GPU (see
[REAL_ROBOT_AND_VLA.md](REAL_ROBOT_AND_VLA.md)).

## 1. Python environment: `~/dev/g1-ext/.venv`

Python 3.12. The code expects third-party software outside this repo, in `~/dev/g1-ext`:

| Package | Version used | Needed for |
|---|---|---|
| `mujoco` | 3.15.0 | everything |
| `mink` | 1.3.0 | arm IK |
| `numpy` | 2.2.6 | everything |
| `opencv-python`, `matplotlib`, `scipy` | current | images, plots |
| `huggingface_hub` | current | downloads the LeRobot G1 sim |
| `pytest` | 9.x | tests |
| `imageio-ffmpeg` | current | ffmpeg for videos (link its binary as `~/dev/g1-ext/.venv/bin/ffmpeg`, or `brew install ffmpeg`) |
| **walking only:** `lerobot` | 0.6.2 (commit `89236ea0`), installed from source in `~/dev/g1-ext/lerobot` | G1 sim + GR00T walking controller |
| **walking only:** `unitree_sdk2_python`, `cyclonedds` | 0.10.x, CycloneDDS built in `~/dev/g1-ext/cyclonedds/install` | DDS between controller and sim |
| **walking only:** `torch` | 2.11 | GR00T policy |

Watched runs record a replay video, so ffmpeg has to be on `PATH`:
`PATH=~/dev/g1-ext/.venv/bin:$PATH`.

## 2. Third-party models

- **BrainCo Revo2 hand meshes** (not in git): download `Revo2_xml.zip` from
  <https://www.brainco-hz.com/docs/revolimb-hand/en/revo2/download.html> and unpack the meshes to
  `models/revo2/xml_left/meshes/` and `models/revo2/xml_right/meshes/` (the XML files are in the repo). Two right-hand
  `_collision` STLs were missing in the download; the visual meshes of the same links stand in for them.
- **Unitree G1 payload** (backpack + head module with D455) and the **BrainCo mount**: in `models/`, with their licences.

## 3. Build the simulator (once)

```bash
~/dev/g1-ext/.venv/bin/python scripts/51_build_brainco_sim.py
```

Builds the LeRobot G1 sim with BrainCo hands and the payload into `~/dev/g1-ext/g1_brainco_sim` (`g1sim.SIM_DIR`).

## 4. Check

```bash
# empty stub packages are enough for the tests that only import lerobot / unitree_sdk2py
mkdir -p /tmp/stubs/lerobot /tmp/stubs/unitree_sdk2py && touch /tmp/stubs/{lerobot,unitree_sdk2py}/__init__.py
PYTHONPATH=/tmp/stubs ~/dev/g1-ext/.venv/bin/python -m pytest -q        # 144 passed (2026-10-09)
```

## 5. Run

See the README, "How to run". The walking scripts (68, 69) need the CycloneDDS library on the path and must never run
together with another DDS sim:

```bash
DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib PYTHONUNBUFFERED=1 \
  ~/dev/g1-ext/.venv/bin/python scripts/69_walk_pick_place.py --trials 6
```

Live views: walking camera + top view at <http://127.0.0.1:8020>, Lab View (all runs, replays) at <http://127.0.0.1:8030>
(`scripts/65_labview.py`). The native MuJoCo window freezes on this Mac, so everything is shown in the browser.
