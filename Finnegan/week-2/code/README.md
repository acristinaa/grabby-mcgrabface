# G1 + BrainCo Revo2 — Language-controlled manipulation

Unitree G1 with BrainCo Revo2 hands. Goal: "pick up the red bottle" — first in MuJoCo,
then on real hardware.

## Documents
| File | Purpose |
|---|---|
| `PLAN.md` | Strategy, phases, acceptance criteria. **Read this first.** |
| `PROGRESS.md` | Current status + log. Update after each sub-step. |
| `SETUP.md` | Phase 0 installation, macOS-specific |
| `CLAUDE.md` | Working instructions for Claude Code |

## Quickstart
```bash
cd ~/"Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot"
source .venv/bin/activate          # ALWAYS first
mjpython scripts/00_view_g1.py     # Viewer: mjpython, NOT python
python  scripts/01_offscreen_render.py   # headless, normal python is enough
```

## Environment (verified 2026-09-07)
- Python 3.12.14 (venv) — **not** 3.14, the robotics stack does not support it
- mujoco 3.12.0, mujoco-menagerie 2026.9.0
- Apple Silicon arm64, no CUDA → training runs in the cloud, not locally

## Architecture (short version)
- **Track A (deliverable):** VLM as planner → IK → scripted grasp. Runs on the Mac.
- **Track B (stretch):** end-to-end VLA. Needs our own teleop data, because **no
  published checkpoint fits the 26-DOF `g1_brainco` morphology.**
