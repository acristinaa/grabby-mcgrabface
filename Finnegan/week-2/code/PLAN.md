# PLAN — G1 + BrainCo Revo2, Sim-First, Language-Conditioned Manipulation

> **SCOPE UPDATE 2026-10-07 — read this first.** It sits on top of the 2026-09-14 block below. Where they disagree, this block wins.
>
> **(j) Update 2026-10-08 — walking up to the table moves to Saad (sim).** Finnegan no longer works on the walk up to the table.
> Saad builds it in sim on branch `saad/table-distractors`: `scripts/68_walk_to_table.py` puts the table (and 0–3 obstacles) at a random
> pose around the robot (table ahead, robot behind the table, robot turned away, table beside, obstacle course, anywhere), plans with
> A*, walks with GR00T and **docks at the volleyball grasp pose** (`g1sim/approach.py`). This is the "align" step from (i) and the sim
> stand-in for what BotBrain/Nav2 + an align step will do on the real robot. Acceptance = the stand range the Level 1 teacher handles
> (±10 cm, ±10°). Still missing for Level 4: walk + grasp in one world (the walking robot's pelvis is ~5 cm lower than the fixed one in
> the grasp scene), localization without sim knowledge (L7), carrying while walking.
>
> **(i) Update 2026-10-07 (evening) — supersedes the model choice in (h).** Model rule: **π₀.₅ (openpi) is the main model** when a
> ≥ 24 GB NVIDIA GPU (AWS credits or other) is available by ~2026-10-13; otherwise **SmolVLA**, measured to train overnight on this Mac
> (0.84 s/step at batch 8). Both use the same raw demos; compare later on the same 50 sim trials. The model always runs **locally on the
> robot** (backpack RTX), never in the cloud while the robot moves.
> **Big picture:** LLM planner (local or SAP's own AI service) → orchestrator (state machine, SAP report) → **BotBrain** (team approved:
> SLAM with RTAB-Map on the Mid-360 + Nav2 navigation + G1 driver; run on the G1's Jetson Orin NX; locally, WITHOUT the web dashboard
> (needs a Supabase login), ROSA (OpenAI) and YOLO (AGPL)) for walking to named places, **VLA** for pick and place → Unitree's walking
> controller. One owner per joint group: BotBrain = legs, VLA = arms + hands + waist. The handover is the risk: navigation arrives within
> ~10–20 cm, so the demos vary the start stance (±10 cm, ±10°), and an "align" step may follow later. Day-by-day plan: `PROGRESS.md` → NEXT STEP.
>
> **(h) Decision 2026-10-07 (later the same day): the task is done by a VLA, trained and run locally.** The user wants an AI ("intelligence")
> to run the whole process, not a script (another team already has a scripted program). Object: a **volleyball** (two-hand grasp, waist
> turn, drop into a basket). Training and inference **locally, not on AWS** (AWS maybe later). Consequences:
> - **Model: SmolVLA** (Hugging Face / LeRobot, ~450M parameters, Apache-2.0, fine-tunable on a consumer GPU such as an RTX 3060). It is in
>   LeRobot, which we already use. π₀.₅ (openpi) stays the upgrade path: LoRA fine-tuning needs > 22.5 GB GPU memory, which only a rented GPU
>   (or a 24 GB card) gives. Where SmolVLA trains (backpack RTX vs Mac) depends on the backpack's OS and GPU memory — still unknown.
> - **Data:** scripted demonstrations from a sim-only "teacher" (never run on the real robot) + later real teleop demos. Dataset in the
>   LeRobot format that SmolVLA expects (check v2.1 vs v3.0; an official converter exists).
> - **Action format 27-D:** 26-D + waist yaw, so the VLA controls the waist turn. This leaves the `unitree_lerobot` 26-D layout.
> - **Level 2 is learned:** randomized ball colour/texture/lighting teaches the VLA to see a real volleyball; no separate detector needed for Level 1.
> - **Campus run ~2026-10-21:** realistic target = first VLA attempts on the real robot + recording real data; a reliable real-robot VLA
>   usually needs real demos too (sim-only policies often fail at first). Plan and order: `PROGRESS.md` → NEXT STEP.
>
> **(a) The user's goal ladder (stated 2026-10-07) — this is now the build order.** Levels 1–3 are the goal. Level 4 is a stretch.
>
> | Level | What it means | Acceptance idea | Learning path (`docs/LEARNING_PATH.md`) |
> |---|---|---|---|
> | 1 — Pick and place, mastered | Scripted first, then record a dataset with the demo factory, then train (π₀.₅ LoRA, or ACT as a cheaper baseline) on a rented GPU. Sim first, then the real robot (needs hours of real teleop demos, not dozens). Next steps: second table + box in the sim scene → placing phase → 50 randomized trials → 100+ pick-and-place demos | Object "inside the box and still for 3 s" in ≥ 8/10 of 50 randomized trials | ≈ L3a (+ placing) + D4–D7 |
> | 2 — Vision-based picking | The robot knows what it picks: object class + colour (e.g. red vs yellow bottle). With a detector, the "item must not be red" rule goes away | Detector picks the right object by class + colour (metric to define) | ≈ part of L5 (Stage 1.3 object finder) |
> | 3 — Task-level picking | "Give me the yellow bottle" → LLM tool call (e.g. AWS Bedrock) → `pick(object, color)` → state machine (find → grasp → place → check → report) → report to SAP ("task completed" / "failed at step X because Y"). Suggested: one table, several objects, no walking | Task + report JSON defined; SAP stand-in web page gets a correct report | ≈ L4 (orchestrator, fixed table) + L5 (text task) + SAP |
> | 4 (stretch) — Walk, fetch, bring back | The old masterplan goal. Sim already has go-to-point + two-room walking (10/10). Missing: walk + grasp in one world, carrying while walking, real-robot localization (Mid-360) | Masterplan M9–M11 | ≈ L3 + L4 + L7 + masterplan |
>
> **(b) Team proposal — pending.** `docs/TEAM_GOALS_PROPOSAL.md` (for the team meeting 2026-10-07 with Cristina + Saad): SAP starts a task →
> robot inspects fire extinguisher + exit (report only) → finds a misplaced item → carries it into a box on a 2nd table → reports to SAP
> through a small bridge program. Success: full sequence ≥ 7/10 in sim, ≥ 3 filmed real attempts, pitch + business case.
> **Status: proposal, waiting for the team's decision.**
>
> **(c) Confirmed hardware (user, 2026-10-07):** Unitree G1 EDU **29 DoF** · RealSense D435i (built in) · Livox Mid-360 LiDAR ·
> Jetson Orin NX 16 GB · BrainCo Revo2 hands · 2-DoF head module with RealSense D455 · backpack = Thunderobot PC (Intel Core i7 +
> NVIDIA GeForce RTX). What it changes:
> - **29 DoF → D5 (mass demo recording) is no longer blocked.** The current grasp needs wrist pitch/yaw, and the real robot has them.
> - The RTX backpack *may* run π₀.₅ inference on the robot. That depends on its GPU memory (π₀.₅ needs > 8 GB) and on its OS
>   (openpi + BrainCo driver need Linux). Both still unknown.
> - The sim backpack (`models/unitree_g1_payload/payload.xml`, AGX backpack shape, 1.9 kg) is only a stand-in. Put in the real mass
>   once the Thunderobot weight is known.
>
> **(d) Perception decision (detector licences).** A *detector* is a model that draws boxes around objects and names them.
> - Ultralytics YOLO (v8/11/26) is **AGPL-3.0** for code, weights and self-trained models (copyleft: it can force us to publish our
>   code). Use it only with explicit approval or a paid Enterprise licence.
> - **Default: RF-DETR** (Roboflow, Apache-2.0 — only sizes N/S/M/L; the other sizes are PML-1.0). Runs on the Mac (Apple MPS).
> - Open vocabulary (find objects by any name, for Level 3): OWLv2 or Grounding DINO (Apache-2.0, via HF transformers).
> - Colour: HSV threshold inside the detected box. Avoid: YOLO-World (GPL-3.0), YOLOE / YOLOv10 (AGPL), DEIMv2 (non-commercial).
>
> **(e) Borrowed ideas — we adopt none of these projects, we only take ideas:**
> - **BotBrain** (MIT, ROS 2 + Docker, Jetson only): G1 Nav2 config + RTAB-Map with the Mid-360; e-stop order stop → squat → damp;
>   waypoint missions as a pattern for the SAP bridge. Caveat: its YOLO module is AGPL; x86 (our Thunderobot) needs an unmerged PR.
> - **g1-manipulation-challenge forks** (luckyrobots repo has **no licence** → ideas only, never copy code or weights): placing with
>   contact check and thumb released first, carry pose, strict success test (at rest, tilt ≤ 10°), failure list, seeded sweeps.
>   Warning from the supat-roong write-up: 94 % fell to 45 % with wide randomization, ~40 % drops while carrying.
> - **Reddit ACT example** (G1 + Dex3, can sorting by colour into 2 boxes, ACT on ~3 h of real demos, no success rate given):
>   a narrow camera view beats a wide one; boxes are forgiving targets; tape on objects against glare.
>
> **(f) Environment / Docker plan — proposed, waiting for the user's go.** (*Docker* packs the software and its settings into a
> container, so a teammate gets the same setup with one command.)
> - Docker Stage 0 (1–2 days): lock the environments (`pyproject.toml` + `uv.lock`), one `setup.sh`, three small fixes (loopback name `lo0`
>   hard-coded in 6 places → setting; live-view/teleop bind host as a flag; `SIM_DIR` overridable).
> - Docker Stage 1 (2–3 days): Dockerfile (arm64 + amd64) + docker compose (sim, teleop), CPU rendering, browser live view. Assets without a
>   licence are downloaded at build time, never pushed in an image. Acceptance: a teammate runs `docker compose up`, live view at
>   localhost:8020, 50 grasp trials within ±5 % of native.
> - Docker Stage 2 (later): openpi's own Docker setup on the AWS GPU / Thunderobot (if Linux).
> - Limits on the Mac: no GPU in containers, no `mjpython` window, no DDS multicast to the real robot → daily dev and real-robot work
>   stay native. Use **Colima** (MIT, free) instead of Docker Desktop, whose terms require a paid plan for our case.
>
> **(g) What the earlier blocks still govern, and what changes.**
> - **Still valid:** training stack openpi π₀.₅ + LoRA; datasets in LeRobot v2.1; sim first; the autonomy layers (orchestrator →
>   navigation → ready-made walking controller → motors); the 26-D action space; head camera only.
> - **Changes:** the build order now follows the Level ladder above (it replaces the L1→L5 vertical-slice order as the main order).
>   The masterplan goal "fetch from the next room" becomes the Level 4 stretch.
> - **Proposal, to confirm with the user:** dataset v1 changes from "pick up the red bottle" to "pick and place into a box".
>   This conflicts with the D3 decision below (placing goes into v2), so it is not decided yet.
>
> Current next steps: `PROGRESS.md`.

> **SCOPE UPDATE 2026-09-14 — supersedes parts of this document. Read this block first.**
>
> The team's masterplan (`docs/MASTERPLAN.md`) is now the **target**: the robot performs a fetch-and-return task
> **fully autonomously**, including walking into the next room, with π₀.₅ as the manipulation policy.
> It is built in stages, and each stage ends in a working demo:
>
> | Stage | Content | Masterplan milestone |
> |---|---|---|
> | 0 | Decision base: LeRobot G1 sim spike on the Mac, one shared MJCF, training stack, Linux+GPU access | – |
> | 1 | Grasp at the table without training (Track A: VLM → 3D → IK) + eval harness | M2 |
> | 2 | π₀.₅ learns grasping from sim demos generated by Stage 1 | M3–M7 |
> | 3 | Autonomous walking + navigation: known pose first, then simulated LiDAR + IMU → Fast-LIO + Nav2 | M8 |
> | 4 | Orchestrator (state machine, masterplan §13) | M9 |
> | 5 | Sim2real (Phases 3–5 below) | M10–M11 |

> **Build order since 2026-10-02 (confirmed by the user): vertical slice first,** see `docs/LEARNING_PATH.md`.
> The stages above stay; the order of work becomes L1 go-to-point (closed loop, pose from sim) → L2 two-room map + A* →
> L3a better grasp at the fixed table (16:10 camera, occlusion, tilt, placing) → L3 walk + grasp in one world →
> L4 orchestrator → L5 text task (LLM tool calling + VLM, Stage 1.3). π₀.₅ (Stage 2.1–2.3) runs in parallel as L6 once the GPU exists;
> LiDAR localization (3.3) follows as L7. Reason: L1–L5 need no GPU and give an end-to-end system early.
>
> **Dataset decisions (D3, 2026-10-02):** (1) **Head camera only** — the real G1 has no wrist cameras (confirmed by the user).
> Episodes, dataset and policy carry `head_image` (D455, 16:10, 320 × 200); π₀.₅'s two wrist inputs get a black image with
> image mask False, as openpi does for missing cameras (`libero_policy.py`). (2) 23 vs 29 DoF still open — blocks D5 (mass
> recording) only, because the grasp with locked wrist is 0/5 (update 2026-10-07: 29 DoF confirmed — D5 no longer blocked). (3) Dataset v1 = "pick up the red bottle" only; placing and the box
> go into dataset v2. (4) ~100 scripted + 20–30 teleop episodes.
>
> **Autonomy layers — no human, no gamepad in the loop:**
>
> ```
> instruction → orchestrator (what now: walk / grasp / carry / place)          ← we build
>             → navigation: Fast-LIO (where am I) + Nav2 (path) → (vx, vy, ω)   ← we build / integrate
>             → locomotion controller: steps + balance                         ← taken as-is, not trained
>             → motors
> ```
>
> **What changes versus the text below:**
> - **"Cut locomotion entirely" is replaced.** We still do not *train* the leg controller; walking and
>   balance are a solved problem. "Controller" means only the bottom layer above, not remote control.
>   Sim: LeRobot v0.5's G1 MuJoCo sim (`lerobot/unitree-g1-mujoco`, 29 DoF, DDS) with
>   `GrootLocomotionController` / `HolosomaLocomotionController`. Real robot: the same code
>   (`--robot.is_simulation=false`) or Unitree's factory controller (`LocoClient.Move(vx, vy, yaw_rate)`).
>   Autonomy, including localization, is tested in sim already (Stage 3). Rejected: `unitree_rl_gym`
>   `motion.pt` (trained on `g1_12dof.xml`, no arms).
> - **Track roles swap.** Track A is no longer the end product. It is the demo generator and the baseline that
>   π₀.₅ has to beat (`docs/DATA_STRATEGY.md`). Track B (π₀.₅) is the goal, no longer a stretch.
> - **Training stack — decided 2026-09-14 (Stage 0.4): openpi (Physical Intelligence, JAX) with π₀.₅ base weights
>   and LoRA.** Fixed by the user as part of the masterplan's tech stack. Consequences: training and policy serving run on an
>   Ubuntu machine with an NVIDIA GPU (LoRA > 22.5 GB); datasets are **LeRobot format v2.1** (openpi pins lerobot `0cf8648`),
>   written by our converter in openpi's own environment; the Mac runs sim, recording and the `openpi-client` side
>   (websocket, like openpi's LIBERO/ALOHA-sim examples). Weights: Gemma terms apply → ask SAP legal.
>   LeRobot π₀.₅ (same weights, dataset v3.0) stays the documented fallback (`docs/STACK_RESEARCH_2026-09-14.md`).
>   Never mix dataset versions.
> - **Action space settled:** 26-D `g1_brainco` = `unitree_lerobot` `Unitree_G1_Brainco` (7 arm joints per side
>   incl. wrist pitch/yaw + 6 hand joints per side), identical for 23 and 29 DoF. Hands normalized 0…1, 0 = open.
> - **Why grasping first:** only grasping is trained, its data comes from a working grasp pipeline, and the leg
>   controller already exists. Stage 0 comes first so the grasp scene is not built twice on two G1 models.
>
> Current checklist: `PROGRESS.md` → NEXT STEP. Everything below is kept as the history of the reasoning.

## Context

**Goal (as stated):** Build a MuJoCo simulation of a Unitree G1, train it there, add a
vision-language model so it understands what to pick up, then transfer to the real robot.

**Situation:** CODE Berlin student project / demo. Real G1 arrives in days-to-weeks.
BrainCo Revo2 hands. Dev machine is an M4 MacBook Pro (no CUDA). Existing docs
(`SETUP.md`, `PROGRESS.md`, `CLAUDE (2).md`) cover Phase 0 well but contain four factual
errors and reference a `PLAN.md` and `README.md` that do not exist.

**The finding that reshapes everything:**
[unifolm-world-model-action#46](https://github.com/unitreerobotics/unifolm-world-model-action/issues/46)
documents a real G1 + Revo2 bring-up. The morphology is `g1_brainco`, **26 DOF**
(14 arm joints incl. 4 phantom wrist slots + 12 hand DOF, 6 actuated per hand,
commands normalized 0.0–1.0). The thread ends on: *"A `g1_brainco` 26-DOF trained
checkpoint is needed."*

**No released VLA checkpoint matches your robot's action space.** Not UnifoLM-VLA-0,
not OpenVLA. Every end-to-end option requires collecting your own teleop data first.
This is the project's critical path, and it is a data problem, not a compute problem.

> **UPDATE 2026-09-09 (K1) — the tooling half of this claim is now wrong.**
> Still true: no *checkpoint* exists for the 26-DOF `g1_brainco` space. No longer true:
> that you would have to build the training path yourself.
> - [`unitreerobotics/unitree_lerobot`](https://github.com/unitreerobotics/unitree_lerobot)
>   lists **BrainCo** as a supported hand (alongside Dex3, Dex1, Inspire) and trains
>   **π₀, π₀.₅, GR00T, ACT and Diffusion Policy**.
> - [`unitreerobotics/xr_teleoperate`](https://github.com/unitreerobotics/xr_teleoperate)
>   supports the **BrainCo hand** and the G1 in **both 23 and 29 DOF**, recording
>   LeRobot-compatible episodes.
> - [LeRobot v0.5.0](https://huggingface.co/blog/lerobot-release-v050) (March 2026) ships
>   full G1 support incl. Real-Time Chunking.
>
> **Consequence:** the gap to Track B is now purely a **data** problem, not a tooling
> problem. Track B is more realistic than the rest of this document assumes — but the
> data path still gates it (see Open decisions).

**Intended outcome:** A demo where a person says "pick up the red bottle," the G1 does it,
first in MuJoCo and then on real hardware — built so the sim is genuinely load-bearing
(env + safety gate + eval), not decoration.

---

## Honest assessment: two corrections to the stated plan

### 1. Do not train perception in MuJoCo

A VLM already knows what a red bottle is from internet pretraining. MuJoCo's renderer is
not photorealistic and cannot teach it anything better — training visual understanding
there actively creates a sim2real gap instead of closing one. What sim genuinely cannot
give you is the *action* distribution for contact-rich grasping.

`CLAUDE (2).md` line 72 already says this. It is correct. It contradicts the "train a VLM
in sim" framing, and we resolve it in favour of line 72.

**Sim's real jobs here:** build and validate the `g1_brainco` model · geometric/IK
development · a repeatable evaluation harness · and the sim2sim safety gate that stops
you breaking a €40k robot. All four are essential. None is "training a VLM."

### 2. Cut locomotion entirely — *superseded 2026-09-14, see SCOPE UPDATE at the top*

`SETUP.md` builds toward locomotion RL (Playground, `unitree_rl_gym`). Drop it.

- The G1 ships with a working walking controller. You would spend your entire compute
  budget reproducing something you already have.
- MuJoCo Playground has **no G1 manipulation environment** — G1 there is locomotion-only
  (`G1JoystickFlatTerrain`, `G1JoystickRoughTerrain`); all manipulation envs are
  Franka / Leap / Aloha. `SETUP.md`'s table implies otherwise. So Playground buys you
  nothing for your actual task.
- Your differentiator is language-conditioned manipulation with the BrainCo hands.

**Scope: standing / fixed-base G1 upper-body manipulation.** Legs stay in the stock
controller or locked.

### 3. Two tracks — commit to A, treat B as stretch — *roles swapped 2026-09-14, see SCOPE UPDATE at the top*

| | Track A — VLM as planner | Track B — end-to-end VLA |
|---|---|---|
| VLM output | object identity + 2D/3D target pose | joint actions at 30 Hz |
| Execution | IK + scripted grasp primitives | learned policy |
| Data needed | none | 100–500+ real teleop episodes |
| Tooling | build it yourself | **exists**: `unitree_lerobot` + `xr_teleoperate` (K1) |
| Compute | Mac + a VLM API | LoRA fine-tune, **>22.5 GB VRAM** — one RTX 4090 suffices |
| OS | macOS fine | **Ubuntu only** — openpi and xr_teleoperate do not run on macOS (K3) |
| Works by demo day | **yes** | unlikely |
| Sim2real risk | low (geometry, not learned dynamics) | high |

**Track A is the deliverable. Track B is the stretch goal and the honest "future work"
slide.** Track A is also not a cop-out: open-vocabulary grounding driving IK primitives is
a legitimate, current architecture, and it is debuggable in a way an end-to-end policy is not.

---

## Direct answer: is the M4 MacBook Pro good enough?

**For everything before the robot arrives: yes. For training: no, and no Mac ever will be.**

| Task | M4 |
|---|---|
| MuJoCo CPU sim + viewer (`mjpython`) | ✅ fine |
| Building the `g1_brainco` MJCF, scene, objects | ✅ fine |
| Track A: VLM planner + IK + grasp primitives | ✅ fine |
| `unitree_mujoco` sim2sim over DDS (Python version) | ✅ likely — no C++ build needed |
| Playground / MJX GPU training | ❌ CUDA-only |
| `mjlab` / MuJoCo-Warp | ❌ CUDA-only |
| Vision-based RL (Madrona / MJWarp renderer) | ❌ CUDA-only |
| IsaacLab (`xr_teleoperate --sim`) | ❌ Linux + CUDA only |
| VLA fine-tuning (Qwen2.5-VL-7B) | ❌ needs ~40–80 GB VRAM |

CPU-only JAX works but is not a training path: a G1 run that takes ~20 min on an H100 is
days on M4 CPU. `jax-metal` is dead — JAX maintainers closed all Metal issues in Dec 2025.

**Recommendation: do not buy a better MacBook.** A faster Mac gives you zero CUDA and
solves none of the ❌ rows. Two better uses of the same money:

1. **Rent, don't buy.** RunPod / vast.ai / Lambda, roughly $0.5–2/hr. A student project
   needs tens of hours, not a laptop.
2. **You will need a Linux box anyway.** BrainCo hand control is dual RS-485 over
   `/dev/ttyUSB*` with ROS 2 Foxy. That is not happening on macOS. A cheap Linux mini-PC
   or the G1's onboard computer covers the hardware side.

Keep the M4 as the dev machine. It is genuinely good at that.

---

## Corrections to make in the existing docs

| File | Issue | Fix |
|---|---|---|
| `SETUP.md` | Table implies Playground gives G1 manipulation recipes | It gives locomotion only. Note that manipulation env is custom-built. |
| `SETUP.md` | "Training slow on CPU" | For **vision** it is impossible, not slow — Madrona/MJWarp are CUDA-only. |
| `CLAUDE (2).md` | Lists `unitree_rl_gym` (Isaac Gym) as a peer option | Isaac Gym is deprecated. Prefer `unitree_rl_mjlab` if RL is ever needed. |
| `CLAUDE (2).md` | "UnifoLM-VLA-0 — bevorzugte VLA-Basis" | Exists (released 2026-01-29), but weights are **CC BY-NC-SA 4.0**. Fine for a student project — record the constraint so nobody assumes commercial reuse later. **Superseded 2026-09-09: prefer π₀ / openpi (Apache-2.0)** — see below. |
| `PLAN.md` | UnifoLM-VLA-0 as the VLA base | **π₀ / π₀.₅ via [`openpi`](https://github.com/Physical-Intelligence/openpi) is Apache-2.0.** The SAP brief requires business relevance, so a non-commercial checkpoint is the wrong foundation. This was the model recommended in the 2026-09-09 team session. |
| both | Reference a non-existent `PLAN.md` / `README.md` | Create them (this file becomes `PLAN.md`). |
| folder | `CLAUDE (2).md` is not read as project memory | Rename to `CLAUDE.md`. |
| `docs/MASTERPLAN.md` | Misses the LeRobot v0.5 G1 integration; openpi YAML config; HDF5/Zarr; `29 + N_hand` action | Appendix A in `docs/MASTERPLAN.md` (2026-09-14) |

---

## Phases

Each phase has an acceptance criterion. Per `PROGRESS.md` rule 4, a phase is not DONE
until its criterion passes.

### Phase 0 — Local foundation (M4, ~1 day)

Follow existing `SETUP.md` steps 1–6 as written — they are correct. **Skip steps 7–8**
(Playground / `unitree_rl_gym`), per the locomotion cut.

- venv, `pip install mujoco mujoco-menagerie`
- `mjpython -m mujoco.viewer` opens
- `mm.load('unitree_g1')` renders; record `model.nq` / `model.nu`
- Fix docs per the correction table; create `PLAN.md`, rename `CLAUDE.md`

**Accept:** G1 visible and twitching under sinusoidal `data.ctrl` via `mjpython`.

### Phase 1 — Build the `g1_brainco` MuJoCo model ← *the asset nobody has*

This is original work and the foundation of everything after it.

- Download the **Revo2 MuJoCo XML** from the BrainCo download center
  (`brainco-hz.com/docs/revolimb-hand/en/revo2/download.html` — URDF ROS1/ROS2 + MuJoCo XML)
- Start from menagerie `unitree_g1/g1.xml` (29 DOF). Note `g1_with_hands.xml` uses
  **Unitree's** hand, not BrainCo — do not use it as-is
- Attach left + right Revo2 at the wrist frames (`mm.get('unitree_g1').spec('g1')` gives a
  `MjSpec` for programmatic attachment — cleaner than hand-editing XML)
- Define the **26-DOF `g1_brainco` action space** matching issue #46: 14 arm
  (incl. 4 phantom wrist slots) + 12 hand, hands normalized 0.0–1.0
- Build a tabletop scene: table, 3–5 graspable objects incl. a red bottle, wrist + head cameras

**Accept:** Both hands open/close under `data.ctrl`; the 26-DOF vector maps 1:1 to what
the real `g1_brainco` stack expects; a wrist camera renders the scene.

**Risk:** the Revo2 MJCF may be a bare hand with no G1 mount frame. Budget time for
manual frame alignment. Cross-check against `BrainCoTech/unitree-g1-brainco-hand`.

### Phase 2 — Track A: VLM planner in sim

Pipeline: `render camera → VLM → target object + 2D point → depth → 3D pose → IK → arm
trajectory → hand close`.

- Use an API VLM (Claude, or Qwen2.5-VL for a local/offline story). Runs fine on the M4.
- Prompt returns structured JSON: object label + pixel coords + suggested grasp type
- MuJoCo depth buffer → 3D point → grasp pose
- IK to the pose, then a scripted Revo2 close on the normalized hand DOFs
- Keep every interface **identical to what the real robot will use** (`CLAUDE.md` rule 6)
- **Closed-loop, not open-loop** (added 2026-09-09): re-perceive and re-correct at least
  once shortly before closing the hand. A humanoid never ends up in exactly the same pose
  twice, so a single snapshot → IK → blind execution breaks for the same reason scripted
  joint trajectories do. This is the strongest argument made in the 2026-09-09 session,
  and it applies to Track A as much as it argues for Track B.

**Accept:** "pick up the red bottle" succeeds in sim, and a distractor test
("pick up the green cup") proves the VLM is actually grounding, not memorizing a position.

### Phase 3 — Sim2sim safety gate (non-negotiable, before hardware)

- Clone `unitree_mujoco`, use the **Python** simulator (avoids the C++ build on macOS)
- `config.yaml`: `robot: "g1"`, IDL `unitree_hg` (**not** `unitree_go` — that is Go2)
- Run the Phase 2 controller through the DDS interface instead of direct `data.ctrl`

**Accept:** Identical behaviour via DDS as via direct control. Plus a hard-coded
joint-limit / velocity / workspace guard that trips before anything unsafe.

### Phase 4 — Real robot bring-up (day 1 with hardware) — **read-only**

Do not command the robot on day one.

- Follow `BrainCoTech/unitree-g1-brainco-hand` and `unitreerobotics/brainco_hand_service`
- Pre-empt the issue #46 failures: Python `>=3.10` relaxation ·
  `LD_PRELOAD=libgomp.so.1` on ARM · bind inference server to `0.0.0.0` ·
  define real initial poses (zeros cause **arms jerking at startup**)
- Stream and log real joint states + camera. Compare against sim.

**Accept:** Real observation vector matches the sim's format exactly. Nothing commanded yet.

### Phase 5 — Real execution (suspended, slow, e-stop in hand)

- Robot gantried/suspended, debug mode, velocity-capped, human on e-stop
- Hands alone first, then arms at reduced speed, then the full Phase 2 pipeline

**Accept:** One successful real "pick up the red bottle." **That is the demo.**

### Phase 6 — Stretch: teleop data + VLA fine-tune (Track B)

Only after Phase 5 works, and only if time and a GPU exist.

**Concrete path (updated 2026-09-09 — this is no longer hand-wavy):**

1. **Record** with `xr_teleoperate` — supports G1 (23 **and** 29 DOF) **and BrainCo hands**,
   writes LeRobot-compatible episodes. Hand-tracking *or* controller-tracking mode.
2. **Convert + train** with `unitree_lerobot`, which targets exactly this hardware and
   supports π₀, π₀.₅, GR00T, ACT and Diffusion Policy.
3. **Base model:** π₀ or π₀.₅ from `openpi` — **Apache-2.0**, unlike UnifoLM-VLA-0.
4. **Evaluate** against Track A in the Phase 2 harness (same success-rate number).

**Hard requirements — none of these are on the M4:**

| | Requirement |
|---|---|
| OS | Ubuntu 22.04 (`openpi`: *"we do not currently support other operating systems"*); xr_teleoperate: Ubuntu 20.04/22.04, Python 3.10 |
| π₀ inference | > 8 GB VRAM |
| π₀ LoRA fine-tune | > 22.5 GB VRAM (a single RTX 4090 is enough) |
| π₀ full fine-tune | > 70 GB VRAM (A100/H100) |
| Data | ~200–500 demos for single-task adaptation on a known embodiment |

- ⚠️ **VR headset: Apple Vision Pro, PICO 4 Ultra Enterprise, or Meta Quest 3.**
  `xr_teleoperate` also has a **simulation mode** (`unitree_sim_isaaclab`) and a
  controller-tracking mode — but both still require Linux. Open decision — see below.
- **Architecture reference:** if you go hierarchical (high-level VLM planner orchestrating
  low-level VLA specialists), that is **Hi Robot** — [arXiv 2502.19417](https://arxiv.org/abs/2502.19417),
  System 2 plans / System 1 (π₀) acts. Do not reinvent it; cite it.

Be honest in the write-up: contact-rich manipulation sim2real is often 5–10% success.
Track A existing is what makes Track B safe to attempt.

---

## Open decisions

1. **VR headset for Phase 6.** Do you have a Vision Pro / PICO 4 Ultra Enterprise /
   Quest 3? If not, Track B has no data path and Phase 6 is out of scope — which is a
   fine answer for a student demo, but should be decided now, not in week 6.
   Fallbacks exist (controller-tracking mode, `unitree_sim_isaaclab` sim teleop) but
   both need a Linux box, so they do not remove requirement 2.
2. **Linux + NVIDIA machine — now needed earlier than "only if Phase 6 happens."**
   `openpi`, `xr_teleoperate` and the BrainCo RS-485/ROS 2 hand stack are all
   Linux-only. Options: cloud (RunPod / vast.ai, an RTX 4090 covers LoRA at ~$0.5–2/hr),
   a cheap Linux mini-PC, or the G1's onboard computer.
   ⚠️ **Collision with the SAP constraint:** the team recorded that code may go *into*
   the SAP network but nothing comes back *out*. If the only GPU is SAP hardware on
   site, any fine-tuned model is created inside that network and can never leave — and
   is useless on the ~355 days you are not there. **Resolve before the first SAP day.**
3. **Compute placement on the robot.** Inference must sit close to the robot. The G1 EDU
   ships with a Jetson Orin NX (~100 TOPS); an official Jetson AGX Thor backplate
   expansion exists. Training never runs on the robot — only inference. Say so explicitly
   in the spec so nobody plans around compute that will not be there.
4. **Which arm/hand does the demo use** — single-arm is meaningfully simpler than bimanual.
   Recommend single-arm.
5. **VLM choice** — API (better, simpler) vs. local Qwen2.5-VL (offline story, slower on M4).

---

## Verification

- **Phase 0:** `mjpython scripts/00_view_g1.py` → robot moves and falls
- **Phase 1:** script driving all 12 hand DOFs 0→1 → both hands visibly close;
  assert `model.nu` matches the 26-DOF layout
- **Phase 2:** scripted eval over ≥5 object/instruction pairs incl. distractors; log
  success rate. This harness is reused for Phase 6 comparison
- **Phase 3:** run the same eval through DDS; results must match Phase 2 within noise
- **Phase 4:** diff real vs. sim observation vectors field by field
- **Phase 5:** manual, supervised, one task, logged

## Files

- `PLAN.md` — **create** (this document)
- `README.md` — **create** (referenced, missing)
- `CLAUDE (2).md` → **rename** `CLAUDE.md`, apply corrections
- `SETUP.md` — correct Playground/vision claims, mark steps 7–8 out of scope
- `PROGRESS.md` — structure is good, keep the logging discipline
- `models/g1_brainco/` — **new**, Phase 1 output
- `scripts/` — `00_view_g1.py`, `01_build_g1_brainco.py`, `02_vlm_planner.py`, `03_eval.py`
