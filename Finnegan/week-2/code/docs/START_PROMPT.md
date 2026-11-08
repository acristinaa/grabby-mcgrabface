# Start prompt for a new session

> Copy this into a new Claude Code conversation. Working folder in VS Code:
> `~/Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot`.
> Everything below the line is the prompt. At the end of every session, update "Status" and "Next session".
> Without a new conversation you can also just continue: `claude --continue` (terminal), or open the last conversation in VS Code.

---

We are working on the SAP / CODE robotics project **Unitree G1 + BrainCo Revo2**. The code lives **only** in
`~/Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot` and is backed up **only**
in the GitHub repo `finneganadam-maker/UnitreeG1-Robot`. No copies or repos anywhere else. Do not touch the drafts directly in
`First Ideas/`.
The long-term goal is the masterplan (`docs/MASTERPLAN.md`): the G1 fetches the red bottle from the next room fully on its own and
brings it back, ≥ 85 % success in simulation, then on the real robot. **My current build order is the goal ladder (2026-10-07):**
Level 1 pick and place → Level 2 vision-based picking → Level 3 task-level picking with a report to SAP; Level 4 (walk through rooms,
fetch, bring back) is a stretch. See `PLAN.md`, block "SCOPE UPDATE 2026-10-07".
The training stack is **fixed: openpi (π₀.₅ + LoRA, datasets in LeRobot v2.1 format)**. Do not reopen this. ACT is allowed as a
cheaper baseline.
All new text in the repo is plain, simple English (since 2026-10-05). Chat with me in English too (since 2026-10-09), even when I write German.

**Before you change anything:** read `CLAUDE.md`, `PLAN.md` (SCOPE UPDATE blocks at the top) and `PROGRESS.md` (section "where we
continue" and the log from 2026-09-22 on). Commands are in `COMMANDS.md`. The path and the learning are in `docs/LEARNING_PATH.md`
(learning path, steps L0–L8). The team proposal is in
`docs/TEAM_GOALS_PROPOSAL.md`. Always keep the learning path up to date (rule 8 in `CLAUDE.md`): status per step in §6, every
explanation from the chat in §10, go through the comprehension questions with me, and add to the logbook (§9).

## Status (2026-11-01 evening) - read this first, then PROGRESS.md → "Update 2026-11-01 (evening)"
- **v2 dataset recorded: 800 demos, 0 failed trials** (`g1-demos/level1_walkup_v2` on the T7). New sim setup: table with legs, natural light,
  textures, table height 70-75 cm, head camera +-2.5 cm/+-5 deg, latency 0-67 ms, 5 task phrasings, debug markers + camera housing hidden.
  Rulings R1/R2/R5 confirmed (2026-10-29). Commits `1696d77` .. `25fe907`, not pushed (push only on my word).
- **Next (at home, Mac on power, lid open, T7 plugged in directly):** `scripts/run2_start.sh` = convert (~20-25 min) -> validate (0 problems)
  -> SmolVLA run2 training detached (~21 h, 90k steps from smolvla_base). Watch: Lab View `#training`.
- **While run2 trains:** data round-trip test (79/80) + pipeline smoke e2e; fix the replay gate (frame-0 edge pixels after a handover
  restore); PC setup script on the T7 must rebuild the scene (`table_scene.write`) after cloning; checkpoint checks at ~20k/40k.
- **Then:** eval 82 (100-200 trials, new config + `--randomization configs/randomization_v1.json` vs run1 72 %), Sim2Sim via 85/86 before SAP (~11-06).
- **External PC:** Ubuntu 26.04, GTX 1060 6 GB, 16 GB RAM; `pc-setup/setup.sh` on the T7 (uv + Python 3.12, torch cu126), not run yet.

## Status (2026-10-27 evening) - read this first, then PROGRESS.md → "Update 2026-10-27 — START HERE"
- **Next robot access: SAP visit ~2026-11-06.** Robot day 10-19 was read-only only (no motion, no VLA attempt). Plan to the visit is in PROGRESS.md.
- **Done 10-27 (committed, not pushed; main is 6 commits ahead of origin):** status audit + plan (`59ac39a`) · Track C 85/87 review GO, torso-lean
  fix, `--arms-up-first` (way in clear, every way out not), runbook Part C, tests 0 failed in 3 envs (`f174f1f`) · teacher little-finger clearance
  0.05 → 3.18 cm, all teacher gates pass, plan + code review GO (`511544c`).
- **Next, only when I say "start" (needs the T7 mounted):** (1) R5 lift-failure diagnostic (~5 min, FINGER-PLAN.md section 8) · (2) quick recorder
  review · (3) v2 demos with the NEW teacher only, no mixing with v1 (proposed 450) · (4) convert 79 + validate 80 · (5) run2 fine-tune (Mac night
  ~12 h or rented GPU) · (6) eval 82 held-out 50, goal ≥ 43/50. Rulings R1/R2/R5 (FINGER-PLAN.md section 10): recommended yes, I have not confirmed yet.
- **Open:** real-table attempt is blocked by 87 (ways out hit table/basket) - decide pull-table-back + 86 wait-at-ready, padded sliding table, or
  shadow only · L2 definition + number · GPU · roles (Step 2 Saad?) · push on my word.

## Status (2026-10-18 afternoon) - read this first, then PROGRESS.md → NEXT STEP "Update 2026-10-18 14:50"
- **Robot day is Monday 2026-10-19.** Cristina + Saad run it from their laptops; I am in Bielefeld. Goal: the 40k SmolVLA attempts the grasp on
  the real G1, safely (gantry, slow, motion locked until I unlock it after all reviews are GO).
- Step 8 DONE (10-16): held-out sim 40k 36/50 (72 %), 45k 35/50, teacher 50/50 → generalization gap; robot candidate = 40k.
- **First task:** run the build workflow `~/dev/g1-ext/release/robot-day-build.workflow.js` with the Workflow tool (`scriptPath`), after checking
  the internet works and the laptop is plugged in (`caffeinate -dimsu` in the background). It writes plan rev 6, reviews it, builds tracks
  A/B1/B2/C/E with reviews, integrates in sim, dry-walks the runbook, and a judge decides. Write progress to `.planning/STATUS.md`.
- **Window 15:30 → Monday 09:00, runs overnight, two pushes** (I push myself): push 1 ~20:00 = read-only checklist + handover kit, push 2
  ~07:30 Monday = motion code only if every gate is GO. Also today: the GitHub Release of the 40k model in two steps (commands in PROGRESS.md), and tell Cristina + Saad.
- No new training before Monday. After Monday: harden Level 1 (finish line in PROGRESS.md), then Level 2; plus a testing track for my uni course.

## Status (2026-10-15 evening) - read this first, then PROGRESS.md → NEXT STEP "Update 2026-10-15 (end of day)"
- Level 1 Steps 1, 4, 5, 6, 7 DONE: teacher (shoulder lift), recorder, 300 demos, LeRobot dataset, policy server/eval/open-loop check, SmolVLA
  run1 (45k steps, checkpoints in `~/dev/g1-ext/smolvla-runs/run1/checkpoints/`, backups on the T7). First sign: 20k checkpoint grasped alone 1/2.
- **First task: Step 8** - closed-loop eval of the 40k and 45k checkpoints on MPS, 50 held-out no-walk trials each + 10 training variations +
  teacher comparison (`COMMANDS.md` → "VLA eval"). Then real-robot prep for Potsdam (10-18/19).
- Start long runs detached: `nohup … & disown` (a run started from a Claude session dies when the window closes). Lab View: http://127.0.0.1:8030.

## Status (2026-10-14) - history
- Level 1 (volleyball into the basket, done by a VLA) follows the 8-step list in `PROGRESS.md` → NEXT STEP. Steps 1 and 5 are DONE:
  teacher in the walk world, now **lifting from the shoulders** (user-approved 2026-10-14, 200/200), and the recorder
  `scripts/69_volleyball_demo_factory.py` (5/5 test demos on the T7, bit-exact replay, review GO).
- Step 6 DONE 2026-10-14: 300 demos on the T7 (`/Volumes/T7/Robot Project SAP/DATA Collection/g1-demos/level1_walkup_v1/`), LeRobot v3.0
  dataset on the internal disk `~/dev/g1-ext/datasets/g1_volleyball_walkup_v1` (never on the exFAT T7), `80` 0 problems, tar backup on the T7.
  Next: Step 7 - SmolVLA 200-step smoke, then night 1.
- Open from the review: eval needs a held-out no-walk mode (`.planning/level1-walkup/PLAN.md` section L). Steps 2 (Saad) and 4 open.
- Commands: `COMMANDS.md` → "Volleyball demo factory". Explanations: `docs/LEARNING_PATH.md` §10.49-10.56.

## Status (2026-10-07)
- **Goal ladder** (mine, 2026-10-07): Levels 1–3 are the goal, Level 4 is a stretch. Mapping to the learning path: Level 1 ≈ L3a +
  placing + D4–D7 · Level 2 ≈ part of L5 · Level 3 ≈ L4 + L5 + SAP · Level 4 ≈ L3 + L4 + L7. **What matters most to me: learning and
  understanding what happens, and what I build when and how.**
- ✅ Up to 2026-09-22: sim with BrainCo hands + payload, walking with GR00T (10 m), grasping without AI, randomization, openpi link, teleop
- ✅ **L1 go to point** (`g1sim/navigation.py`, `61_goto_check.py` 10/10) and **L2 map and path** (`62_rooms_check.py` 10/10 room changes,
  0 wall contacts). Live view in the browser (`g1sim/liveview.py`, port 8020+)
- ✅ **D1** head camera 16:10 like the D455 + "clear the view" phase → 95 % on reachable starts · **D3 (part)** head camera only in the
  data format
- ✅ **D2** (code done, **not committed, independent review gate OPEN**): hand rolled −30°, ring + little finger open, `LIFT` 0.10 +
  `LOAD_SAG` 0.04. Tilt ~41° → ~2.7°. `54 --randomize --trials 50`: seed 0 25/25 reachable (25/50), seed 1 35/35 (35/50); teleop check
  `58` passes. 100 % is not a robustness promise (`.planning/reviews/D2-measurement.md`). **2 known test failures:**
  `test_ik_erreicht_greifpose[23dof]`, `test_greifwert_erst_daumen_dann_finger` → fix or mark
- ✅ **D4 probe** (`scripts/64_demo_factory.py`, uncommitted): 2 demos, each 20.5 s sim, ~93 MB, ~16 s compute. Mass recording not started
- ✅ **Real robot confirmed (2026-10-07):** G1 EDU **29 DoF** · RealSense D435i · Livox Mid-360 · Jetson Orin NX 16 GB · BrainCo Revo2 ·
  2-DoF head with D455 · backpack = Thunderobot PC (Core i7 + RTX). 23/29 is closed → **D5 unblocked**
- 📝 Real-robot scripts written, **never run on the real robot** (uncommitted): `70_real_goto.py` (go to point via `LocoClient.SetVelocity`,
  `--dry-run`), `71_real_probe.py` (listen only; `runs/real_probe.json` was recorded against the sim on `lo0`), `72_real_check.py`
  (joints, one arm joint, hand, camera)
- 📝 Team proposal `docs/TEAM_GOALS_PROPOSAL.md` (meeting 2026-10-07 with Cristina + Saad): **proposal, pending team decision**
- ✅ **2026-10-08 (Saad, branch `saad/table-distractors`):** volleyball distractors · full randomization in the volleyball scene
  (`67 --randomize`, 100/100) · **walk up to the table from random start situations** (`68_walk_to_table.py`: table ahead, robot
  behind the table, turned away, beside, obstacles, anywhere → A* + docking at the grasp pose; 24/24, ≤ 3 cm / 3°; live view with a top
  view). Walking to the table moved from Finnegan to Saad (`PLAN.md` (j)). Not yet: grasp after the walk (L3 part 2)
- ⚠️ Limits: no placing yet · many random starts out of reach (fixed by L3 "align") · position and map come from the sim · sim backpack
  (`models/unitree_g1_payload/payload.xml`, 1.9 kg) is a stand-in with the wrong mass
- Open without code: GPU access, RTX model + GPU memory and OS of the Thunderobot, backpack weight, Gemma license (π₀.₅ weights),
  YOLO/AGPL approval, SAP test system

## Next session, in this order
At the start, propose a **short** table of deliverables and acceptance criteria and let me confirm it. Before each step, explain the
concept in 5 minutes, then ask comprehension questions; record everything in the learning path (rule 8 in `CLAUDE.md`).
0. **If not done yet: I commit the open work** (D2 `grasp.py`, `63`, `64`, `70`–`72`, `docs/TEAM_GOALS_PROPOSAL.md`, docs). Give me the
   file list and a commit message.
1. **L3 part 2 (Saad's track):** walk up to the table (`68`) and then run the volleyball teacher in the *walking* world – the walking
   pelvis is ~0.74 m, the teacher was tuned at a fixed 0.793 m; give the IK the real pelvis/waist pose. Acceptance: ≥ 8/10 walk + grasp.
1. **Level 1 build (start here):** second table + box in the sim table scene → placing phase in `g1sim/grasp.py` (lower over the box,
   release, check; borrow ideas from the supat-roong write-up, never copy code) → `54 --randomize --trials 50` with "inside the box and
   still for 3 s" ≥ 8/10 → then the demo factory (D4/D5) for 100+ pick-and-place demos
2. **Docker Stage 0** (lock environments + 3 small fixes: `lo0` setting, bind host flag, `SIM_DIR` overridable): only after my go; can
   run alongside step 1
3. **Level 2** in parallel (good for teammates): coloured objects in the sim, real D455 photos, RF-DETR (after the YOLO/AGPL question)
4. **Level 3 prep:** task + report JSON, SAP stand-in web page
5. Put the real Thunderobot mass into `payload.xml` once known
6. **Real robot:** first read-only connection with `71_real_probe.py`, then `70_real_goto.py` with the safety checklist (second person
   on the remote, harness/spotter, 3 m clear)
7. **Saving:** I commit and push. After each passed step, name the files and a commit message (including `docs/LEARNING_PATH.md`); at the
   end, update `PROGRESS.md` ("where we continue") and this file

Waiting on others: team meeting outcome (goal, which simulator, success definition), answers from the robot owner / SAP, GPU access.
**Open for the user:** run `scripts/run_tests.sh` (~2–3 min; 2 known failures expected, see the D2 log) · try teleop (`57_teleop.py --randomize`) · answer the L1 + L2 comprehension questions, fill logbook §9.

## How I want to work
- **Fast but clean:** short plans, few questions (a default with a recommendation instead of a questionnaire), clear steps, clean code
- **Tests only where code breaks silently** (interfaces, bugs we found, new calculation cores). The step's acceptance script is the proof
- Before each step, say briefly what you are working on ("Now: …"), afterwards the result
- Explain technical terms in one simple sentence the first time; for decisions, give a clear recommendation
- Announce long runs beforehand: what runs and how long
- **I run the unit, integration and E2E tests myself** – give me the command and the duration. You may run the acceptance script of the
  current step
- **I commit and push myself**
- Say honestly what is realistic and what is not finished yet

## Environments and pitfalls
- Path with spaces: always quote it in the shell (`cd ~/"Documents/…/First Ideas/UnitreeG1-Robot"`), run Python via `.venv/bin/python -m …`
- Three Python environments:
  - `.venv` → `.venv.nosync` (project, excluded from iCloud sync; without `mink`)
  - `~/dev/g1-ext/.venv` (sim, DDS, grasp scene, `mink`, openpi client; needs `DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib`)
  - `~/dev/g1-ext/openpi/.venv` (openpi; `datasets`, `pyarrow`, `huggingface-hub`, `fsspec`, `gcsfs` pinned to openpi's `uv.lock`
    versions – do not upgrade)
- Third-party software stays in `~/dev/g1-ext`. The table scene lives in `~/dev/g1-ext/g1_brainco_sim` and disappears every time the
  walking model is rebuilt (`51_build_brainco_sim.py`) → then run `53_build_table_scene.py` again (54 also writes it itself)
- No MuJoCo window (`mjpython`) – it freezes on this Mac (in both environments); to watch, use `--video` or the live view in the browser
  via `scripts/57_teleop.py` (physics and images in the main thread, the browser only displays)
- Never run two sim runs with DDS at the same time; the stand-in server and the dashboard share port 8000
- Holding `TableEnv` still: always send a fixed target (e.g. `action_space.READY_POSE`), never `env.state()`. The IK also works on
  commanded angles and subtracts the measured sag of the hand
- `head_camera` = D455 on the head module, no longer LeRobot's placeholder. The head joints have no motor (nu stays 41): change the angle
  with `payload.set_head(model, pan, tilt)`, start tilt at the table is `table_scene.HEAD_POSE`. Unitree's URDF `torso_link` sits 1 cm
  below LeRobot's – never attach Unitree poses directly to LeRobot's `torso_link` (`payload.ANCHOR_POS`)
- Revo2: motor `Thumb` turns the thumb in front of the palm, `ThumbAux` bends it. Turn first, then close
- The grasp geometry is measured, not guessed (`g1sim/grasp.py`: `BOTTLE_IN_HAND`, `GRASP_BELOW_TOP`, and since D2 the roll and the open
  ring/little finger) – do not change it without a new measurement run. `63_grasp_sweep.py` turns `ROT_DEG` relative to the current
  (already rolled) `GRASP_ROTATION`
- The grasp scene runs ~20× faster than real time without camera images; 50 grasp trials take ~50 s. With camera images it is much slower
- The warning "overflow/divide by zero in matmul" is a false alarm from numpy on macOS (Accelerate); the values are correct
- Walking scripts: `g1sim/walk_sim.WalkSim` (connect, rubber band, fall check, live view). Always `use_brainco_sim(cameras=False)` and
  **only one** renderer per process – three OpenGL image sources at the same time left the process unkillable in the graphics driver on
  2026-10-02 (it holds its port until logout). The sim computes in its own thread: read data like `data.contact` as a snapshot
  (`np.array(data.contact.geom)`)
- Head camera image size: `table_env.IMAGE_SIZES` (head 320 × 200 = 16:10). openpi pads images to 224 × 224 itself
- No local training (M4 without CUDA). `openpi_g1.run train` refuses on the Mac; training later on a rented Linux GPU
- Real robot: the Mac connects by cable with fixed IP 192.168.123.222/24 (interface e.g. `en5`). Hand and camera hang on the onboard
  computer (192.168.123.164), not the motion computer. Never use Damp or ZeroTorque while it stands, and never start LeRobot's
  `run_g1_server.py` on the robot – it switches off the factory walking controller
