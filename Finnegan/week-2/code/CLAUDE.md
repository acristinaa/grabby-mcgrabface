# CLAUDE.md — Working instructions for Claude Code / VS Code

> This file is read first in every session. It explains the project and the ground rules.

## What are we building?

> **Update 2026-10-07 (English):** The goal is now the user's Level ladder: 1 pick and place → 2 vision-based picking → 3 task-level
> picking with an SAP report → 4 (stretch) walk through rooms and fetch. The fetch task below is Level 4. Read `PLAN.md` →
> "SCOPE UPDATE 2026-10-07" first. Robot confirmed: G1 EDU 29 DoF, Thunderobot backpack (i7 + RTX). The focus heading in
> `PROGRESS.md` is now "NEXT STEP".

A **Unitree G1** (humanoid, BrainCo Revo2 hands) should do a fetch-and-bring task **fully autonomously**:
*"Go to the next room, fetch the red bottle from the table and bring it back."*
Goal and milestones: `docs/MASTERPLAN.md`. Stage plan: `PLAN.md` (SCOPE UPDATE 2026-09-14) + `PROGRESS.md`.

Layers — no human and no gamepad in the loop:
- **Orchestrator** (state machine) → decides *what* comes next: walking, grasping, carrying, placing
- **Navigation** (Fast-LIO + Nav2) → decides *where to go*, sends velocity commands (vx, vy, ω)
- **Walking controller** → turns the commands into steps + balance. **Taken over ready-made, not trained by us**
  (sim: LeRobot G1 sim with GR00T/Holosoma; real: the same code or the Unitree factory controller).
  "Controller" here never means remote control.
- **Grasping** → first VLM + geometry/IK (Track A, no training), then π₀.₅ via imitation learning from sim demos

**Order: everything in simulation first. The physical robot comes last.**

## Central repos
- `google-deepmind/mujoco` — physics simulator
- `google-deepmind/mujoco_menagerie` — G1 MJCF model (base of our table scene)
- `huggingface/lerobot` (v0.5+) — **G1 sim with walking controller** (environment `~/dev/g1-ext`)
- `lerobot/unitree-g1-mujoco` (Hugging Face) — the G1 MuJoCo sim for it: 29 DoF, DDS, torque limits
- `kevinzakka/mink` — IK on MuJoCo (template `examples/humanoid_g1.py`)
- `Physical-Intelligence/openpi` — **training stack, decided 2026-09-14:** π₀.₅ base + LoRA, JAX. Training and
  policy server on Ubuntu + NVIDIA GPU; datasets in LeRobot format **v2.1**; the Mac uses only `openpi-client`.
  Our link to it: `openpi_g1/` (training/server side), `g1sim/openpi_env.py` (sim side)
- `unitreerobotics/unitree_lerobot` — G1 + BrainCo data layout (`Unitree_G1_Brainco`, 26-D)
- `unitreerobotics/xr_teleoperate` — data recording via VR (Linux), supports BrainCo
- `unitreerobotics/unitree_mujoco` — Sim2Real bridge over DDS
- `unitreerobotics/unitree_sdk2` (+ `_python`) — real hardware (`LocoClient` for walking, `rt/arm_sdk` for the arms)
- `BrainCoTech/brainco-hand-sdk`, `BrainCoTech/unitree-g1-brainco-hand` — hand control
- Fast-LIO + Nav2 (ROS 2, Linux) — localization + path planning
- ~~`mujoco_playground`, `unitree_rl_gym`, `unitree_rl_lab`~~ — dropped: we do not train walking ourselves;
  Isaac Gym is deprecated. If RL is ever needed: `unitree_rl_mjlab`
- ~~UnifoLM-VLA-0~~ — CC BY-NC-SA (non-commercial), replaced by π₀.₅ (Apache-2.0)

## macOS specifics (always keep in mind!)
- **Viewer only with `mjpython`**, not `python`. `mjpython` comes with the `mujoco` package.
  Every example from the web with `python -m mujoco.viewer` must be rewritten to `mjpython ...`.
- **The G1 uses `unitree_hg` messages**, not `unitree_go` (that is the Go2). A common source of errors
  when taking over example code.
- `unitree_mujoco/config.yaml`: set `robot` from `go2` to `g1`.

## Non-negotiable ground rules

1. **Do not train a VLA from zero.** Always fine-tune an existing model with LoRA.
2. **Reproduce before you change.** First get the official baseline running, then adapt it.
3. **Domain randomization is mandatory** in every training and in every demo generation (object poses, light,
   colors, masses, friction, motor gains, sensor noise, latency). Without it, Sim2Real is hopeless.
4. **Do not skip a phase.** A phase is only done when its acceptance criterion is met.
5. **Sim2Sim is the mandatory safety test** before any thought of real hardware.
6. **Keep the observation and action format identical** between sim and real.
7. **Safety before progress.** On real hardware: secured, debug mode, slow, e-stop.

## Hardware reality (important for suggestions!)

The user works on a **MacBook Pro M4 (Apple Silicon, no CUDA)**.

- Possible locally: MuJoCo, scene building, policy inference, code, debugging, small test runs
- NOT possible locally: Isaac Gym (no macOS), large RL training, VLA fine-tuning

→ **Never suggest a local full training run.** Always separate clearly:
"develop/test locally" vs. "train on a cloud GPU".
→ For local tests: small `num_envs`, few iterations, only check that it runs.

## How we work in every session

1. Read `PLAN.md` (strategy) and read `PROGRESS.md` (current status).
2. Work only on the item named under "NEXT STEP".
3. Small steps that can be checked. After each step: does it run? Then add an entry to `PROGRESS.md`.
4. Change of strategy → update `PLAN.md`. Progress → `PROGRESS.md`.
5. When blocked: do not guess. Document it in `PROGRESS.md` under BLOCKERS and ask.
6. **Saving:** the main Claude session commits, one commit per passed sub-step, and shows the files + message
   (user's rule, 2026-10-18; replaces "only the user commits" from 2026-09-15). **Pushes only on the user's word.**
   Build/subagents never commit. At the end of the session, update `PROGRESS.md` ("Where we continue")
   and `docs/START_PROMPT.md`.
7. **Location (user's rule, 2026-09-15):** code only in `~/Documents/Projekt Tim and Lukas/Cursor/Software Engineering/
   SAP Robot Project/First Ideas/UnitreeG1-Robot`, saved only in the GitHub repo `finneganadam-maker/UnitreeG1-Robot`.
   No copies or repos anywhere else. Third-party software (LeRobot, openpi, CycloneDDS, their Python environments) stays in `~/dev/g1-ext`.
8. **Always keep the learning path up to date (user's rule, 2026-10-02):** `docs/LEARNING_PATH.md` is the complete learning path. In every session,
   add to it without being asked: status and result of each step (§6), **every explanation from the chat** as a short version in §10,
   corrected answers to the comprehension questions, and the logbook (§9). `docs/LEARNING_PATH.md` belongs in every commit proposal.

## Managing expectations (stay honest)

- Locomotion Sim2Real: works reliably, a solved technique.
- Manipulation Sim2Real: hard; contact-rich tasks often reach only 5-10% success.
- Purely synthetic VLA data is rarely enough — plan for real teleoperation data.
- The biggest effort in the project is **data collection**, not compute.

If something is unrealistic, **say so clearly** instead of promising it.
