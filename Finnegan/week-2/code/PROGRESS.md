# PROGRESS.md — Progress log

> This document tracks the CURRENT STATUS. PLAN.md = strategy, PROGRESS.md = execution.

---

## RULES (binding)

1. **When to save:** After each finished sub-step of a phase, add an entry here
   — not only at the end of a phase.
2. **Format of each entry:** `[YYYY-MM-DD] Phase X.Y — <what was done> — Status: DONE/WIP/BLOCKED`.
3. **When BLOCKED:** write down the reason + the decision/resource that is needed, so it is clear what is missing.
4. **Never skip a phase:** A phase only counts as DONE when its validation step has passed
   (e.g. Phase 1 is DONE only after a successful Sim2Sim test).
5. **Reproduce before you change:** Before our own code replaces a baseline, the baseline must be
   shown to run (entry with status DONE).
6. **Code + docs in sync:** Every relevant code change that affects the strategy is mirrored in
   PLAN.md; pure progress goes here.
7. **Current focus** is always at the very top under "NEXT STEP" (until 2026-10-07: "NÄCHSTER SCHRITT").

---

## NEXT STEP (NÄCHSTER SCHRITT)
**Current goal (set by the user on 2026-10-07): a ladder of four levels.** Levels 1–3 are the goal, Level 4 is a stretch.
- **Level 1 — Pick and place, mastered.** Scripted grasp first, then record a dataset with the demo factory, then train
  (π₀.₅ with LoRA, or ACT as a cheaper baseline) on a rented GPU. Sim first, then the real robot. (LoRA = a small add-on that is
  trained on top of a big model instead of retraining the whole model. ACT = a smaller imitation-learning model.)
- **Level 2 — Vision-based picking.** The robot knows *what* it picks (object class + colour, e.g. red vs. yellow bottle).
  Detector: RF-DETR (Apache-2.0), not YOLO (AGPL, see Blockers).
- **Level 3 — Task-level picking.** "Give me the yellow bottle" → LLM tool call → `pick(object, color)` → state machine
  (find → grasp → place → check → report) → report back to SAP. One table, several objects, no walking.
- **Level 4 (stretch) — Walk through rooms, fetch, bring back.** This is the old masterplan task (`docs/MASTERPLAN.md`).
- Map to the learning path (`docs/LEARNING_PATH.md`): Level 1 ≈ L3a + placing + D4–D7 · Level 2 ≈ part of L5 (Stage 1.3) ·
  Level 3 ≈ L4 at a fixed table + L5 + SAP · Level 4 ≈ L3 + L4 + L7 + masterplan.
- Team: `docs/TEAM_GOALS_PROPOSAL.md` (SAP task → safety round → move a misplaced item into a box → report to SAP) was written for
  the team meeting on 2026-10-07. **Proposal only — the team decision is still pending.**

**Robot visit milestones (decided by Finnegan, 2026-11-04):**
- **Visit 1 (SAP, ~2026-11-13):** the robot is placed at the table. Fixed, 87-checked waypoint paths raise the arms safely to the ready pose
  -> the trained VLA (run2) does the grasp and place on its own -> fixed safe paths bring the arms back (recover after any fault).
  Intelligence = the learned grasp; waypoints + safety layer = engineered, verified frame. Qwen/QR brain (Saad) is a bonus, read-only at first.
- **Visit 2 (SAP, date open):** full walk-up: the robot walks to the table, then grasps and places intelligently. Needs first: Step 2 (walk +
  grasp in one sim world) with the walk-mode eval >= 80 %, real walking (Unitree LocoClient; `70_real_goto.py` never run on hardware) +
  handover at the table, Sim2Sim of the whole chain, and the visit-1 lessons (real torso lean, real camera look, first real attempts).

**Update 2026-11-08 — START HERE (newest). run2 evaluated: undertrained; run3 training on the Hive PC; safety fence in build.**
- **run2 eval (Hive PC, `runs/eval_run2_pc.sh`, reports `runs/vla_eval_*.json` on the PC):** 90k on 100 held-out starts (seed 0, v2 ranges)
  **47/100 = 47 % (Wilson 95 % 38-57 %)**, training starts **3/10**, verdict **"undertrained"**. Failures: 33 ball lifted but not placed in time,
  16 forbidden contacts (table/stand/basket), 4 other. Teacher on the same 100 starts: **100/100**. Fair comparison on run1's conditions
  (v1 ranges, same 50 starts): run2 **27/50 = 54 %** vs run1 40k **36/50 = 72 %**. 80k/60k not evaluated (skipped).
- **Cause (planning error):** run2's 90k steps were sized for 800 demos (~1.2 epochs). v2 grew to 1,691 demos = 1,227,666 frames without
  re-sizing, so run2 saw 720k samples = **0.59 epochs** (run1: 1.65). Loss 0.012 hid it. Rule from now on: size training in epochs
  (target 1.5-2), re-compute whenever the dataset changes.
- **Decision (Finnegan, 2026-11-08):** continue on the Hive PC: **run3** = `scripts/train_smolvla.sh run3` (`5cef14b`), from run2's 90k
  weights, fresh schedule (warm-up 1k, cosine over 150k), 150k steps -> ~1.57 epochs in total, ~1.84 s/step -> **~77 h, started 2026-11-08
  14:38, expected end Wed 2026-11-11 ~20:00**. Checkpoints every 10k. If the Tuesday mid-check shows no clear gain, or it gets tight: a rented
  GPU. run1-under-v2-ranges eval (fallback C) later, not today (Finnegan).
- **Gotcha:** the policy server on the PC needs `HF_HUB_OFFLINE=1 HF_HUB_CACHE=~/.cache/huggingface/hub` (as training), otherwise it tries
  the Hub and dies. The `libavutil.so.60` traceback at training start is a harmless torchcodec probe (also 10x in run2's log).
- **Visit-1 goal re-stated (Finnegan, 2026-11-08):** the real robot runs the learned grasp, never hurts itself or hits table/stand/basket,
  no violent jerks (a bit jerky is fine), tries to pick up the ball; a miss is acceptable. 85 % sim stays the training target.
- **Safety build started 2026-11-08 (workflow, plan -> review -> build -> review):** virtual fence = live FK distance check of every policy
  command against table/stand/basket and the robot's own body, hold + soft exit on violation, `82 --fence` for sim; smoothness metrics +
  limiter only if needed. Plan `.planning/real-robot/FENCE-SMOOTH.md`, reviews `.planning/reviews/fence-smooth.md`.
- **Committed, not pushed:** `29e64a7` visit milestones, `8bc5c9c` safe arm paths (code review GO r3), `ae455d1` README, `5cef14b` run3.
  Team repo `acristinaa/grabby-mcgrabface` branch `week-2`: Finnegan/week-2 (status + code snapshot), pushed by Finnegan by hand.
- **Next steps:** Sun: run3 running; fence/smooth build; replay test of the PC data (offered). Mon: fence review + sim test against
  run2's contact cases. Tue: run3 mid-check (copy a 10k-step checkpoint, eval) -> stay on PC or rent GPU; smoothness in the 85 rehearsal;
  runbook Part C text. Wed ~20:00: run3 done -> final eval (100+ starts), pick checkpoint, sha256 release. Thu: Sim2Sim of the chosen
  model through 84 -> 86 -> 85 with fence; laptop self-test 89. Fri 13 Nov: SAP. Open rulings for Finnegan: F1-F5, S1-S6, r3-m1, r3-m3
  (`.planning/real-robot/SAFE-PATHS.md`) + fence margins. Qwen: after visit 1 basics; needs 86 to take the task sentence from a whitelist
  of the 5 trained phrasings (today `policy_wire.TASK` is fixed, `g1sim/real_run_policy.py:133`).

**Where we continue: see "Update 2026-11-08" above. Older notes (as of 2026-10-07):**
Repo: `~/Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot` (rule 7 in `CLAUDE.md`).
Start prompt for the next chat: `docs/START_PROMPT.md`. The user commits and pushes himself (rule 6). Keep the learning path
`docs/LEARNING_PATH.md` up to date in every session without being asked (rule 8). New text in the repo is plain English (since 2026-10-05).

**Robot specs (confirmed by the user, 2026-10-07):** Unitree G1 EDU **29 DoF** · RealSense D435i (built in) · Livox Mid-360 LiDAR ·
Jetson Orin NX 16 GB · BrainCo Revo2 hands · 2-DoF head module with RealSense D455 · backpack = Thunderobot PC (Intel Core i7 +
NVIDIA GeForce RTX). (DoF = degrees of freedom = number of motor joints.)

**Done since 2026-10-02** (details in the LOG):
- **D2 bottle tilt:** tilt 41° → ~2.7°; 25/25 and 35/35 on reachable starts (seed 0 / seed 1). Code done, review gate OPEN, uncommitted.
- **D4 probe:** 2 probe demos with `scripts/64_demo_factory.py` (~20.5 s sim, ~93 MB, ~16 s compute each). Mass recording not started.
- **Real-robot scripts** `scripts/70_real_goto.py`, `71_real_probe.py`, `72_real_check.py` written — **never run on the real robot**
  (the probe only ran against the sim).
- **Team goal proposal** `docs/TEAM_GOALS_PROPOSAL.md` + fact checks (`.planning/reviews/team-goals-*.md`) + research (compute, AWS,
  external projects).
- **29 DoF confirmed** → the old 23/29 question is closed, D5 is no longer blocked.
- **2026-10-08 (Saad, branch `saad/table-distractors`, merged 2026-10-09):** volleyball distractors (30/30) · full randomization in the
  volleyball scene, teacher follows the real stand pose, reach rule (**100/100** randomized) · **walk up to the table from random start
  situations** – table ahead / robot behind the table / turned away / beside / obstacles / anywhere – docking at the grasp pose
  (`scripts/68_walk_to_table.py`, **24/24**, ≤ 3 cm / 3°). Finnegan no longer works on the walk to the table (PLAN.md (j)).

**Update 2026-10-09 — Saad's branch merged and pushed (`main` = `82f7445`, plan `.planning/level1-walkup/PLAN.md` section J):**
our Stages 1+2 + natural arm motion and Saad's distractors, full randomization and walk to the table now live in one code base.
Gates after the merge: all tests green (3 envs) · 67 nominal 10/10 · ±2 cm 50/50 · fully randomized 50/50 + 50/50 · distractors 30/30
(267 placed, none moved) · 68 walk to the table 12/12 (≤ 2.1 cm, ≤ 2.6°) · quick independent review GO
(`.planning/reviews/merge-saad-distractors.md`). Nothing is on the T7 yet; no demos recorded; no VLA trained.

**Update 2026-10-10 — Step 1 DONE (teacher in the walk world, details in the LOG):** walk-world sweep **199/200** (gate ≥ 170/200),
reach map 52/65 (every failure explained), fixed scene unchanged (10/10, ±2 cm 50/50, randomized 50/50), all tests green (3 envs).
New: stop radius 0.458 (knee 4.1 cm from the table), walk sampler (layout + stop pose + arrival error), `67 --scene volleyball_walk
[--map]`, teacher phase `centre` (ball over the basket centre before the release: p95 3.8 → 0.3 cm).

**Update 2026-11-03 — v2 now built on the Hive PC (newest, read first).** PC `ubucode` (Ubuntu 26.04, Ryzen 7 2700X 8c/16t, 15 GB RAM,
GTX 1060 6 GB, 849 GB free) reached over Tailscale `grabby@<pc-tailscale-ip>` (Mac key, no password). Set up remotely: repo pushed over SSH (no
GitHub access on the PC), sim model + hand meshes rsynced (gitignored), uv + Python 3.12, sim env from `requirements-record.txt`, SmolVLA env
torch 2.11.0+cu126 (Pascal needs cu126) + `lerobot[smolvla,dataset]==0.6.1`. Measured: teacher 5/5, recording 7 s/demo/process (3x the Mac),
training probe 200 steps OK at **1.62 s/step**, 3.6 GB GPU memory (Mac 0.84 s/step). T7 not seen by the PC (cable/port) -> v2 re-recorded on
the PC (T7 copy = backup). User raised v2 to **1,600 demos**. 6 recorders froze the PC (17:13, no log line); a 69 process leaks ~1.8 -> 3-4 GB
over ~40 demos (on the Mac's RAM it never showed) -> RAM/temperature watchdog on the PC (`runs/pc_watchdog.sh`, stops recorders below 1.5 GB
free or above 90 C) and `scripts/run2_pc.sh` (`968f7c7`) records in 40-demo processes, 3 at a time, then 79 -> 80 -> run2 (CUDA, 90k steps,
~40 h). 18:12: 287/1,600; expected training start ~21:15, end ~Thu 11-05 ~14:00. Linger on for grabby (jobs survive logout); auto-suspend on
AC off. Security: PC hardening open (details offline). TODO: find the 69 memory leak.
Check from the Mac: `ssh grabby@<pc-tailscale-ip> 'tail -3 ~/UnitreeG1-Robot/runs/l1v2_pc_chain.log'`.

**Update 2026-11-01 (evening) — v2 recorded: 800 demos, 0 failed trials.** `g1-demos/level1_walkup_v2` on the T7 (84 GB): seeds 110-112 x 200,
s109 x 144 (crashed at 15:30 when the T7 briefly disconnected and remounted - PermissionError on /Volumes/T7), s113 x 56 as the replacement.
Before the restart at 14:27: the head image showed the D455 housing edge as a dark bar under camera pose randomization and the sim's red/green
debug sites at the feet -> all sites + the housing hidden from cameras (`c971523`, physics identical 10/10); self-view check over 1296 extreme
camera poses: 0 bar/marker pixels, max 27 px of the head servo in 6 corner poses (`runs/l1h4_selfview_check.log`). The 31 demos recorded before
the fix are in `level1_walkup_v2_discarded_markers` (not used). Smoke check: smoke_v2 10/10, validator 0 problems; the replay gate shows ~0.2 %
edge pixels (shadow/contour) differing on frame 0 after a handover restore - render-only, to be fixed in the gate. APFS image on the T7 was
damaged by the disconnect (`g1data-apfs-broken-20261101.sparsebundle`, held only smoke_v2) and recreated. Next: 79 -> 80 -> run2
(`scripts/train_smolvla.sh run2`, 90k steps from smolvla_base, ~21 h) started by a chain at 16:02 (`runs/l1v2_chain.log`).

**Update 2026-11-01 (afternoon) — table look, render look, table height.** Table = 3 cm top + 4 legs (visual only, `e2f02eb`); natural render
look (key + fill light, matte floor; physics bit-identical; v1 config keeps the old look); table height per demo in [-4, +1] cm (70-75 cm).
+-4 cm was tried first: above +1.4 cm the teacher brushes table/stand/basket (high bucket 70/73, p5 2.53 cm, `runs/l1h2_g5_g7_buckets.log`),
so Finnegan chose [-0.04, 0.01]. Gates on [-4, +1] cm: G1 10/10, G2 50/50, G3 50/50 + 50/50, G4 30/30, G5 200/200, G6 100/100, G8 50/50,
info seeds 101-104 300/300; G7 p5 3.69 cm, pinky p5 3.71 cm, **min 2.97 cm (0.3 mm under the 3.0 working bar, one trial, no contact - accepted
to keep moving, Finnegan 2026-11-01)**; G9 lift min 8.03 cm (`runs/l1h3_*`). Review of this step: not run (workflow stopped for the range change).
APFS sparse image on the T7 for converted datasets: `Robot Project SAP/g1data-apfs.sparsebundle` -> `/Volumes/G1DATA` (no `._` files).

**Update 2026-11-01 — v2 variation built (newest, read first).** Plan: `~/.claude/plans/humming-squishing-treasure.md` (v2 = Level 1 only,
built Level-2-ready). Built: latency FIFO (now up to 3 ticks), head camera +-2.5 cm / +-5 deg, 12 table/floor textures, `configs/randomization_v1.json`
(old ranges) + `82 --randomization`, 5 task phrasings per demo, teacher `target=` argument, provenance (config sha256 + versions) in 69's sidecar,
`requirements-record.txt`. Review: code GO; the plan's latency 0-100 ms failed G7 (p5 3.47 < 3.5 cm), so Finnegan chose **0-67 ms** -> G5+G6 300/300,
G7 p5 3.63 / min 3.18 / pinky p5 3.71 cm, G9 lift min 8.26 cm (`runs/l1v2b_*`); camera corners leave the image by up to 12.5 px in ~5 % of draws -
accepted by Finnegan (the real head camera can shift that much). Tests 300 / 515 (+1 skip: T7 not mounted) / 10 pass.
Next: data round-trip test (79/80) + pipeline smoke (e2e) -> smoke_v2 (10 demos) -> v2 800 demos (seeds 109-112) -> convert -> run2.
External PC = native Ubuntu 26.04, GTX 1060 6 GB, 16 GB RAM: setup script + sim model on the T7 (`Robot Project SAP/pc-setup/setup.sh`, not run yet).

**Update 2026-10-27 — START HERE (newest, read first). Status audit + plan to the SAP visit (~2026-11-06).**
- **Paused 2026-10-27 evening by Finnegan (no internet on the way).** Done today: Track C (`f174f1f`), finger clearance (`511544c`), tests green.
  **v2 recording and run2 wait for Finnegan's "start"** (T7 must be mounted). Order then: R5 diagnostic → recorder review → v2 (new teacher only,
  proposed 450) → 79 + 80 → run2 → 82 eval. Rulings R1/R2/R5 not confirmed yet (recommended yes).
- **Robot day 2026-10-19 (told by Finnegan, 10-27):** only the read-only part ran (checklist 84 / reading joints, hands, camera). No arm motion,
  no VLA attempt. No real data is in the repo (`runs/real/` does not exist). → "real ≥ 3/10" is still at 0 attempts.
- **Audit 10-27 (read-only workflow, 4 auditors + planner + independent critic, verdict OPEN = numbers fine, plan needs the decisions below):**
  all branches are merged into `main`; 3 commits not pushed (`85f3204`, `0c5dd13`, `510f54a`); tests: g1-ext 505 pass / 1 fail, project venv
  299 / 1, openpi 10 / 10 - the one red test is `tests/test_unit_runbook.py::test_runbook_clearance_table_matches_report` (runbook Part C still
  has the placeholder instead of 87's rows; Task E2 never done). Track C (85, 87, `g1sim/arm_clearance*.py`, `tests/helpers/sim_host.py`,
  3 tests) still uncommitted and unreviewed; its e2e ran 10-18: 6 pass, 2 fail on plan limits (waist sag 0.03 > 0.02, replay lag 0.255 > 0.15).
- **Level 1 finish line (PROGRESS.md "Agreed direction") - none met yet:** held-out sim ≥ 85 % → **72 %** (40k, 36/50, CI 58-83 %) · walk mode
  ≥ 80 % → never measured (Step 2 walk + grasp in one world is not in the repo) · real ≥ 3/10 → 0 attempts.
- **Blockers found:** (1) **table clearance**: `runs/real_c_clearance.log` - for every factory elbow 0.0-1.6 and waist 0/0.15/0.35/0.6 the
  replay exits and fault exits are NOT clear → no VLA attempt at the real table without a changed layout ("arms up first, then table in" variant
  not checked yet). (2) **storage**: T7 not mounted, 58 GiB free on the Mac → no v2 recording until a disk is back. (3) **GPU**: still open
  (rented GPU vs. RTX backpack). (4) no path for real teleop demos exists (57 is sim keyboard; 86 has no teleop mode).
- **Effort basis for the estimates:** Level 1 sim (10-07..10-16) = 10 calendar days / 7 active days / ~14 h compute; real-robot stack ~2.5 active
  days. Actual vs. plan was 2.3-4.5x (first VLA number planned 10-12, came 10-16; Level 2 was planned for 10-10..12 and never started).
  **Level 2** (VLA with a sentence per demo, several coloured balls, 2 boxes): 4-6 active dev days, 8-15 calendar days, 800-1500 demos,
  80-150 GB, training 15-25 h Mac or ~4-8 h GPU (derived); + RF-DETR detector 3-5 days (parallel, optional). The language path is already wired
  (episode.py → 79 → policy_wire.py → vla_eval.py), only the prompt is one constant (69:53). **Level 3** (sentence → LLM tool call → state
  machine → SAP report): 4-6 active days, almost all new code, no GPU; can start now with the teacher as the grasp skill. Realistic by 11-06:
  L3 in sim with an SAP stand-in page; a first L2 VLA number is more likely on or after the visit.
- **Plan to the SAP visit (focus now = finish Level 1 for real, then Level 2):**

  | Day | Goal | Done check |
  |---|---|---|
  | Tue 10-27 | Track C review (1 round) + fix + commit · runbook Part C rows from 87 → all tests green · Robot-day entry (this block) · T7 check · start little-finger clearance (teacher) | `real-robot-code-C.md` GO; `pytest -m "not e2e"` 0 failed in 3 envs |
  | Wed 10-28 | Little-finger fix + teacher review · L2 scene (several targets + 2nd box) · L3 task/report JSON contract | teacher 50/50 + 0 finger contacts on the 200 sweep |
  | Thu 10-29 | v2 demos re-recorded with the new teacher (overnight, needs a disk) · teacher with a target parameter · state machine against the teacher | validator 80: 0 problems |
  | Fri 10-30 | run2 training (fine-tune, GPU or a Mac night) · eval 82 target-aware · LLM tool call · table layout fix for 87 | 87 shows replay + fault exits clear, or documented alternative |
  | Sat 10-31 | run2 held-out eval → L1 sim number · L2 demos with sentences recorded | 82 seed 0, 50 trials: goal ≥ 43/50 |
  | Sun 11-01 | Buffer · L2 training start · SAP stand-in page | |
  | Mon 11-02 | Real-robot plan revision for the SAP site + plan review | review GO |
  | Tue 11-03 | Chosen model through 85/86 in sim (Sim2Sim) + final judge · L3 end-to-end in sim | judge GO; L3 report = run log 10/10 |
  | Wed 11-04 | L2 eval (stretch) · push 2 + motion unlock only on Finnegan's word | |
  | Thu 11-05 | Freeze · sim videos L1-L3 · self-test 89 on the operator laptops | |
  | Fri 11-06 SAP | Read-only A/B; arm motion only if clearance + unlock are GO, else shadow mode on real frames + data; L2/L3 shown in sim | result x/10 in PROGRESS.md the same day |

- **Decisions open for Finnegan (recommended default in brackets):** L2 definition (VLA with language until SAP, detector later) + L2 number
  (right object in the right box ≥ x/50) · GPU (rented GPU) · disk (T7 back or a new SSD) · roles (Step 2 Saad? L3 who? real track Cristina?) ·
  table layout at SAP ("arms up first, then table in" if 87 clears it) · loosening the 2 e2e plan limits only after an independent review.

**Update 2026-10-18 19:55 — PAUSED by Finnegan (read the 10-27 block first).** Everything stopped; no agent or sim running.
- **On GitHub (push 1, 733ade8):** A checklist 84 (+ segfault fix), E1 kit, B1 safety layer, runbook A+B+C. Monday read-only works.
- **Local, committed, not pushed:** B2 adapter 86 (review GO round 4, `0c5dd13`), handover `85f3204`.
- **Uncommitted, NOT reviewed yet:** Track C - `scripts/85_arm_sdk_sim.py`, `scripts/87_arm_path_clearance.py`,
  `g1sim/arm_clearance{,_fk,_report}.py`, `tests/helpers/sim_host.py`, 3 test files. C review round 1 was stopped mid-way.
- **Key finding from 87:** no factory elbow is cleared for table steps M4-M6 (raising hanging -> ready sweeps a hand through the
  basket/table even 15 cm back) -> Monday motion = release/ready/replay in the air only, unless the variant "arms up first, then push
  the table in" clears (check was stopped before a result). e2e: 7 pass, 2 fail on plan limits (waist pitch sag 0.03 vs 0.02; replay
  lag 0.255 vs 0.15 while the demo itself lags 0.264) - limits to re-judge, not code bugs.
- **Next when resuming:** (1) C review (one round; stop if nothing broken - user rule), (2) 87 variant check, (3) integration in sim
  with the 40k model, (4) final judge, (5) push 2 on Finnegan's word, (6) Finnegan unlocks motion himself.

**Update 2026-10-18 19:15 — START HERE (newest).** Push 1 is on GitHub (`733ade8`, 19:00): plan rev 7.x + reviews, E1 kit
(setup, self-test 89, test data), A read-only checklist 84 (+ segfault fix), B1 safety layer (logic only, sends nothing), runbook
Parts A+B+C. Push-1 check from a fresh clone passed (87 unit tests, SELFTEST PASS, 84 5/5 exit 0, own writers 0).
- **Open for push 2 (motion code):** B2 adapter 86 (extra review rounds running), C sim stand-in 85 + table clearance 87 (building),
  then integration in sim + final judge. Testing guide and the post-integration Part C dry walk were dropped (credits); runbook Part C
  is review-GO (E2). Status per gate: `.planning/STATUS.md`, findings: `.planning/reviews/real-robot-*.md`.
- **If the session dies:** Monday works read-only with push 1. To continue: start a session in `UnitreeG1-Robot`, read this block +
  the tail of `.planning/STATUS.md`; run the remaining gates (B2/C review GO -> integration -> judge) and commit B2/C only when GO.
  Motion stays locked: only Finnegan sets `docs/ROBOT_DAY_MOTION.json` after push 2.
- **Rules since today:** Claude commits per passed stage, Finnegan says "push" (CLAUDE.md rule 6). Robot PC login only on site
  from the owner. Head camera is motorized: owner sets it, operators only check (runbook B3).
- **Open questions for Finnegan:** shared folder for Monday's `runs/`; table stays on tape marks (recommended); D3 robot re-place
  within 1 cm vs. move the table.

**Update 2026-10-18 14:50 — START HERE. Robot day is MONDAY 2026-10-19; Cristina + Saad operate, Finnegan is in Bielefeld (remote).**
- **Goal for Monday (user):** the 40k SmolVLA attempts the volleyball grasp on the real G1, run from Cristina's/Saad's laptops; the robot is
  placed at the table by the owner's remote (walking with `70` stays out of Monday's runbook). Honest expectation: a sim-only VLA likely
  misses at first; success = safe motion, clean replay, first attempts, real data for dataset v2.
- **Nothing for the robot is built yet.** Real-robot plan `.planning/real-robot/PLAN.md` is at **revision 5** (written for a *Sunday,
  no-motion* day; plan review GO with condition r5-6, see `.planning/reviews/real-robot-plan.md`). It must become **revision 6** (Monday, motion
  allowed behind a gate). The build workflow failed twice: once stopped by the user, once on lost internet (ENOTFOUND) - 0 agents finished.
- **Build = workflow script `~/dev/g1-ext/release/robot-day-build.workflow.js`** (run it with the Workflow tool, `scriptPath`): planner rev 6 →
  plan review loop → tracks A (real_io, real_camera, 84 read-only checklist), B1 (real_safety), B2 (86 adapter: release/ready/replay/shadow/run/
  hands-open), C (85 sim stand-in + sim-to-sim/fault test), E (runbook, laptop setup macOS/Ubuntu with a Python 3.10 robot env, 89 self-test) →
  per-track review/fix → integration in sim → runbook dry walk by a fresh agent → judge. Agents never commit or push.
- **Window today 15:30 → Monday 09:00 (runs overnight), two pushes:** push 1 ~20:00 = Tier A + handover kit (+ the release) so Cristina and Saad can set up
  tonight (~30-60 min download/install); push 2 ~07:30 Monday = motion code ONLY if every review + the final gate are GO (operators `git pull` before 09:00), otherwise not pushed and Monday is read-only.
  **Motion stays locked** until Finnegan sets `docs/ROBOT_DAY_MOTION.json` (allowed + reviewed commit) himself.
- **User's standards for this build (2026-10-18 ~14:53):** clean, readable code he can explain line by line (small functions, named constants with
  their source, docstrings that say why); few but meaningful tests, each with a "Proves / Catches" docstring, unit + integration + e2e, safety
  checks seen failing first (mutation check); reviewers treat unreadable code as a major; every agent writes a plain-language start/done line to
  `.planning/STATUS.md`; at the end `docs/TESTING_GUIDE.md` explains every new test (for his testing course). Already built into the workflow
  script (dry run with stub agents passed, incl. a NO-GO → fix → re-review round).
- **Dead-code screening (2026-10-18 ~14:58, read-only):** 120 files / 426 top-level defs: 0 unused modules, 0 never-called functions, 1 unused
  import (`time` in `scripts/82_vla_eval.py:33`), 2 orphan scripts from the dropped RL-standing experiment (`scripts/31_eval_stand.py`,
  `32_record_stand.py`). Cleanup deferred until after Monday (user decides on 31/32). The final gate also screens all NEW files for dead code.
- **Two sessions ran at once on 10-18 ~14:52:** session "first-ideas-7b" launched the old script inline (run wf_3bf7af0f-761). Only ONE run may
  write PLAN.md and the track files - stop that run before launching the updated script.
- **Model release NOT on GitHub yet** (first try 10-17 failed mid-upload, gh rolled it back). Packed + sha256 OK in `~/dev/g1-ext/release/`.
  Upload in two steps: `gh release create robot-day-2026-10-18 --repo finneganadam-maker/UnitreeG1-Robot ~/dev/g1-ext/release/smolvla_run1_040000.tar.sha256
  --title "SmolVLA run1 40k" --notes "..."`, then `gh release upload robot-day-2026-10-18 ~/dev/g1-ext/release/smolvla_run1_040000.tar --repo
  finneganadam-maker/UnitreeG1-Robot --clobber` (repeat only this line if it fails). Teammates download ~4-7 GB in total (model + SmolVLA base
  + packages); the 60 GB dataset is not needed at the robot.
- **Not pushed since `47407f9`:** PROGRESS.md, docs/LEARNING_PATH.md, docs/START_PROMPT.md, .planning/real-robot/PLAN.md, .planning/reviews/
  real-robot-plan.md, .planning/STATUS.md.
- **v2 dataset: aborted.** The T7 dropped out during recording on 10-16 (~17 demos per seed 105-108 in `g1-demos/level1_walkup_v2add`,
  unverified, partial). No v2 training. Recommendation: no new training before Monday; the next round mixes in Monday's real data.
- **Agreed direction after Monday:** harden Level 1 first (more + more varied sim data, little-finger clearance, real demos, tests + evals,
  numeric finish line: held-out sim ≥ 85 %, walk mode ≥ 80 %, real ≥ 3/10), then Level 2 (colours, several objects, two boxes, sentence per demo).
  The user also wants an automated-testing track for his university course (writing tests himself with Claude as tutor; course requirements
  still to ask).
- **Open with the user:** teammates' laptop OS (macOS Apple chip / Ubuntu), Ethernet adapter, `gh auth login`; phone number for the runbook;
  the motion unlock decision after the final gate; rulings R2-R8 (motion details) - recommended defaults: yes.

**Update 2026-10-16 — Step 8 no-walk eval DONE (details in the LOG).** Held-out seed 0, 50 new starts + 10 training starts, MPS:
**40k 36/50 = 72 % [58-83 %]** (training 9/10) · **45k 35/50 = 70 % [56-81 %]** (training 10/10) · teacher 50/50 on the same starts (upper bound,
the scene is fine). 40k and 45k are statistically the same. Decision rule (walk-up PLAN section E): training starts ~90-100 %, new starts ~70 %
→ **generalization gap, not undertraining** → more varied demos or narrower ranges, not more steps. Main failure: a **little finger brushes the
ball stand / basket / table while reaching** (40k: 6, 45k: 12 of the failures; a forbidden contact ends the trial at once), then "ball not lifted"
(40k: 7, 45k: 3). No stalls. Latency 209-212 ms mean per chunk. Candidate for the robot: **40k** (half the contact failures; weak preference).
Open: walk-mode eval (needs Step 2) · one Lab View video · real-robot plan `.planning/real-robot/PLAN.md` (being written, review next).

**Update 2026-10-15 (end of day) — START HERE TOMORROW. Steps 4 + 7 DONE, Step 8 half done (details in the LOG):**
- **Training run1 finished 22:25** (45,000 steps, loss 4.0 → 0.011). 9 checkpoints in `~/dev/g1-ext/smolvla-runs/run1/checkpoints/<step>/pretrained_model`
  (5k … 45k); all 9 backed up to the T7 (one sha256-verified .tar each) in `/Volumes/T7/Robot Project SAP/DATA Collection/smolvla-runs/run1/`.
- **Step 4 DONE:** policy server `81`, sim eval `82`, open-loop check `83` (gate 10/10 replay, review GO). **Step 8 so far:** MPS latency **207 ms per
  chunk** (passes < 1 s); open-loop over all 9 checkpoints on MPS (`runs/l1w_s8_open_loop_all.log`): skill arms 0.41 → 0.80, waist 0.44 → 0.78,
  hands 0.79 → 0.93-0.97; **plateau from ~35k** (arms 0.785 / 0.801 / 0.801 at 35k / 40k / 45k). Hands "gate FAIL" at 45k is noise on tiny
  numbers (MAE 0.0008 → 0.0012). Preview 20k on CPU in the sim: 1/2 (one success alone, one pinky touch on the stand).
- **Tomorrow, in this order (Step 8, ~1-2 h):**
  1. Full eval of **40k and 45k** on MPS: `82 --no-walkup --seed 0 --trials 50` each (+ `--train-variations 10 --demos <level1_walkup_v1>`),
     server `81 --checkpoint .../0X0000/pretrained_model --device mps` (commands: `COMMANDS.md` → "VLA eval"). Then `82 --teacher` on the same
     seed 0 as the upper bound. Result = success rate with Wilson interval, failure table, decision rule (PLAN section E).
  2. Watch a few trials in the Lab View (Live) and keep one video.
  3. Then the real-robot prep for Potsdam (10-18/19): adapter VLA → `rt/arm_sdk` + BrainCo hands + waist, D455 image in 200×320, policy
     server on the backpack PC, safety layer, bring-up checklist. Not started yet - plan + review first, like Step 4.
- Open: Step 2 (walk + grasp in one world, Saad - not on GitHub yet). Uncommitted since `450fd4b`: see `git status` (Steps 4 + 7, backup
  script, Lab View Training tab, docs).
- Running in the background: only the Lab View server (http://127.0.0.1:8030). Nothing else.

**Update 2026-10-14 — Step 5 DONE, Step 6 recording (details in the LOG):** recorder `scripts/69_volleyball_demo_factory.py` (review gate GO),
5/5 smoke demos valid on the T7, bit-exact replay. The grasp now **lifts from the shoulders** (user-approved, walk sweep 200/200, all fixed-scene
gates green). **Step 6 DONE:** 300 demos on the T7 (99.7 % of trials), LeRobot v3.0 dataset `~/dev/g1-ext/datasets/g1_volleyball_walkup_v1`
(internal disk), validator 0 problems, tar backup on the T7. Next: Step 7 SmolVLA 200-step smoke, then night 1 (`--dataset.root` = that path).
Open from the review: the eval needs a held-out no-walk mode (PLAN section L, M1). Nothing of today is committed yet.

**24-hour plan (agreed 2026-10-09 evening, "let's go" for Step 1):** demos do **not** wait for Saad's walk-in (Step 2). They start at
the handover anyway; the walk-up only decides *where the robot stands*, so v1 is recorded in **no-walk mode**: stance = stop pose +
a random arrival error inside the range the reach map passed (back 0-6 cm, sideways ±4 cm, ±6°; Saad's measured walk arrivals are
≤ 4 cm / ≤ 3°). Not in the fixed scene: its pelvis is 4.3 cm higher (0.793 vs 0.7505 m) and closer to the table than the knee allows.
Order: Step 1 (done) → Step 5 recorder + 5 test demos on the T7 + one quick review → Step 6 record 300 + convert + validate →
Step 7 SmolVLA smoke + night 1. Steps 2 and 4 in parallel; Step 2 feeds the walk-mode eval and a later v2 dataset.
Plan B: Step 1 not green by Sat 14:00 → 50 fixed-scene demos only to prove record → convert → train (not the Level 1 dataset).
**T7:** plug it in right before the 5 test demos (Step 5) and keep it plugged in through recording, conversion and the training
night (training reads from it). Path `/Volumes/T7/Robot Project SAP/DATA Collection/g1-demos/level1_walkup_v1/`. ~30 GB raw npz +
30-40 GB lossless v3.0. If the smoke run reads much slower than 0.84 s/step from the T7, copy the dataset to the internal disk.
**Recommendations in use, not yet confirmed by the user:** ball always left of the basket in v1 (mirror = v2) · distractors in v1
only if moving them into the walk world takes < 1 h, else v2 · two training nights (night 1 = pipeline proof).

**Next steps (agreed 2026-10-09; Step 1 done 2026-10-10, Step 5 no longer waits for Step 2):**
| # | What | Who | Done when |
|---|---|---|---|
| 1 | ~~Stage 3 remainder: stop radius, teacher in `volleyball_walk` (67 `--scene volleyball_walk`), basket position varies too~~ **DONE 2026-10-10: 199/200** | Finnegan | 200 random trials in the walk scene ≥ 85 % |
| 2 | Join the two worlds: Saad's walk-in controller inside `volleyball_walk` → stop → weld on → teacher grasps | Saad | walk + grasp ≥ 8/10 ball in the basket |
| 3 | Basket detection with distractors (Stage 2 review: a tall object in front of the basket can give a wrong centre) | Finnegan or Saad, before the walk-up uses the camera | distractor in front of the basket → right centre or None, never wrong |
| 4 | ~~Policy server + eval script (Stage 5), proven by replaying a recorded demo~~ **DONE 2026-10-15: replay 10/10, review GO** | whoever is free | replay 5/5 |
| 5 | ~~Demo factory (Stage 6): record from the handover (head image + 27-D), successes only, to the T7~~ **DONE 2026-10-14: `69`, 5/5 on the T7** | Finnegan | 1 test demo recorded + validated |
| 6 | ~~Record 300 demos → convert to LeRobot v3.0 → validate (Stage 7)~~ **DONE 2026-10-14: 300 demos, 0 validation problems** | Finnegan (~1-2 h run) | 0 validation problems |
| 7 | ~~SmolVLA training (Stage 8)~~ **DONE 2026-10-15: run1 45k steps, loss 0.011, open-loop error falls to a plateau at ~35k** | overnight | checkpoint saved, open-loop error falls |
| 8 | Eval (Stage 9): scripted walk-up, the VLA grasps alone **— no-walk part DONE 2026-10-16: 40k 36/50 (72 %), 45k 35/50 (70 %), teacher 50/50; walk mode waits for Step 2** | Finnegan | success rate over 50 trials |
1, 2 and 4 run in parallel (different files); 5 needs 1 (no-walk mode; walk mode needs 2 as well); 6-8 in order. Timeline: 1 + 2 Thu/Fri → recording Fri/Sat → training Sat
night → first VLA number Sun 10-12. Decided 2026-10-09: the grasp motion is good enough for now ("good enough, not perfect", user) -
lock it for the recording; polish later means re-recording (~1-2 h) + retraining (one night). Review rule: one quick review before the
recorder (Stage 5/6) is used for the 300 demos.

**Status at the end of 2026-10-07 (all committed):** Lab View v1 (http://127.0.0.1:8030, `scripts/65_labview.py`) · volleyball scene
(`66_volleyball_scene_check.py`, 5/5 checks) · **sim teacher for the volleyball: 10/10 nominal, 50/50 with ±2 cm jitter**
(`g1sim/ball_grasp.py`, `scripts/67_volleyball_pick_place.py`, video `docs/videos/volleyball_demo_video.mp4`) · SmolVLA measured on this Mac
(trains: ~0.84 s/step at batch 8, ~5 h for 20k steps; inference ~200 ms per 50-action chunk on MPS). **No model is trained yet** — the
teacher is programmed; it only exists to record demonstrations for the VLA.

**Big picture (agreed 2026-10-07, details in `PLAN.md` → SCOPE UPDATE (i)):** "Hey Unitree, take the package from table A and put it in the
box" → LLM planner (local, or SAP's own AI service — no OpenAI) → orchestrator (our state machine, reports to SAP) → **BotBrain** for
SLAM + navigation (green light from the team; runs locally WITHOUT its web dashboard, Supabase, ROSA and YOLO) and the **VLA** (π₀.₅ or
SmolVLA) for pick and place → Unitree's built-in walking controller. Rules: BotBrain = legs only, VLA = arms + hands; BotBrain on the G1's
Jetson Orin NX, VLA + planner on the Thunderobot backpack.

**Model decision rule:** both share the same demos. **AWS credits (or another ≥ 24 GB NVIDIA GPU) by ~2026-10-13 → train π₀.₅ (openpi,
already integrated in `openpi_g1/`), ~$30–60 per LoRA run (30k steps).** Otherwise → SmolVLA trained overnight on this Mac. Run either
locally on the robot (never in the cloud while the robot moves). Later: compare both on the same 50 sim trials in the Lab View.
For the G1 specifically, **π₀.₅ is the better fit** (pre-trained on two-armed robots, stronger vision and language, already integrated);
SmolVLA's only advantage is training on the Mac, so it serves as the safety net and to get the pipeline working end to end first.
Third candidate, **not yet checked**: NVIDIA GR00T N1.x (a VLA built for humanoids; license, hardware needs and G1 support unverified).

**Day-by-day plan to Potsdam (real robot 2026-10-18/19):**
| Day | Work | Done when |
|---|---|---|
| **Wed 10-08** | (1) natural start pose for the volleyball scene (today's READY_POSE raises the hands to head height) · (2) vary the robot's start stance in the volleyball scene (±10 cm, ±10°) so the VLA later copes with imprecise arrival from navigation · (3) 27-D recorder (26-D + waist yaw) + volleyball demo factory · (4) record 200–500 demos to the external SSD (`--out /Volumes/<SSD>/g1-demos`) · (5) convert (LeRobot v3.0 for SmolVLA; v2.1 for openpi exists) + validate · (6) start SmolVLA training overnight (batch 8–16, 20k steps) | 67 still ≥ 40/50 with the new pose and stance variation; demos validated; training running |
| **Thu 10-09** | Evaluate the trained SmolVLA in the sim: new eval script, the VLA alone runs the whole sequence, logged to the Lab View · analyse failures · fix data, retrain overnight | **Level 1 trained**: success count of the VLA in sim |
| **Fri–Sun 10-10..12** | **Level 2:** visual variation (ball colour/pattern like a real volleyball, lighting, camera noise), two balls + language prompt ("put the blue ball in the basket") · retrain · evaluate | VLA picks the requested ball in sim |
| **Mon 10-13** | **Model decision gate** (AWS?) → π₀.₅ LoRA run on a rented GPU with the same data, compare in the Lab View · ask the backpack questions if still open | Decision written into PLAN |
| **Tue–Fri 10-14..17** | **Real-robot prep:** real adapter (VLA actions → `rt/arm_sdk` incl. waist yaw + BrainCo driver; D455 images in) · policy server on the backpack · sim-to-sim test of exactly this code path · safety layer (speed/range/torque limits, stop on lost camera) · tests: unit, integration, E2E, fault injection, regression gate (fixed seeds) · bring-up checklist script (PASS/FAIL per step) · in parallel (teammates): BotBrain installed on the G1 Orin NX, fully local, tested with the internet disconnected | All tests green; checklist ready; BotBrain walks to a goal without internet |
| **Sat/Sun 10-18/19 Potsdam** | Bring-up in small steps: read-only → arms in the air → hands → slow VLA attempts → full task · film everything · record real teleop demos (dataset v2) · lessons learned | ≥ 3 filmed attempts + lessons list |

**Parked:** Docker stage 0 · Level 3 SAP stand-in page · separate Level 2 detector (the VLA learns to see) · BotBrain dashboard (only with a
self-hosted Supabase) · YOLO (AGPL — only with SAP approval).
**Blocking questions (ask now):** AWS credits yes/no by ~10-13 · Thunderobot OS (Linux?), exact RTX model + GPU memory · how the BrainCo
hands are wired and which computer drives them · robot time + harness in Potsdam · is waist yaw allowed via `rt/arm_sdk` · VR headset for
real teleop demos · SAP legal: Gemma terms of the π₀.₅ weights · SSD name (is "BrainBot" the SSD?).

**Waiting on others:** team meeting outcome (goal, which simulator, what counts as success) · answers to the open questions in
BLOCKERS (robot owner / SAP) · GPU access.
**Open for the user:** run `scripts/run_tests.sh` (~2–3 min; 2 known failures expected, see the D2 log) · try teleop (`57_teleop.py --randomize`) · answer the L1 + L2 comprehension questions, fill logbook §9.
**Known:** a hung Python process from the 2nd L2 run (stuck in the graphics driver, state "UE", cannot be killed) still holds port 8020
until logout (checked 2026-10-07) — the live view automatically moves to port 8021+.

### Stage 0 — Basis for decisions (1–2 days)
- [x] 0.1 Status committed and pushed to GitHub (private, `finneganadam-maker/UnitreeG1-Robot`) — DONE
- [x] 0.2 **LeRobot G1 sim runs on the Mac — acceptance check passed.** GR00T walking controller, walk command from code:
      **10.0 m without a fall**, 0.51 m/s, pelvis constant at 0.75 m, sim at 0.54× real time (M4 Pro, no window).
      `scripts/50_spike_lerobot_walk.py`, result `runs/stufe0_2_lerobot_walk.json`.
      Limits: still Dex3 hands (→ 0.3) · without course correction 3.4 m sideways drift over 10 m
      (→ navigation must control the direction) · the MuJoCo window (`mjpython`) freezes when it opens →
      `--headless`, to watch use `--video` (mp4) — DONE
- [x] 0.3 **BrainCo hands in the LeRobot sim** — DONE (0.3a–d). The LeRobot sim ships with **Unitree Dex3 hands**
      (`g1_29dof_with_hand.xml`, 14 hand motors) — not our hands.
      - [x] 0.3a **Model comparison — DONE** (numbers in the log 2026-09-14): joint names, order, joint limits
        and wrist mount point are **identical** to our Menagerie model. Differences: torso 9.60 kg
        (LeRobot) vs. 7.82 kg (Menagerie), joint damping 0.05 vs. 0.5.
        → **Decision: base = LeRobot model**, the GR00T walking controller is tuned for it.
        Table scene, cameras and pixel→3D carry over, because the kinematics are the same.
      - [x] 0.3b **Model built — DONE (10/10).** `scripts/51_build_brainco_sim.py` builds `~/dev/g1-ext/g1_brainco_sim`
        (recipe `g1sim/build.py`): nu 41, all torque `<motor>`, mount = official Unitree URDF (0.0 mm / 0.0°),
        no hand contacts at rest, each hand motor alone closes to ≥ 96.7 % and opens to ≤ 0.2 %. Image `docs/stufe0_3b_brainco_haende.png`
      - [x] 0.3c **10 m with BrainCo hands without a fall — DONE.** 10.0 m, 0.51 m/s, arms track to 0.08 rad (Dex3 comparison 0.08).
        `50_spike_lerobot_walk.py --brainco`, video `docs/videos/stufe0_3c_g1_brainco_laeuft.mp4`
      - [x] 0.3d **Sim bridge `rt/brainco/*` — DONE.** All 12 hand motors over DDS in three phases (fingers closed + left half,
        rotate thumbs, open), DDS state = sim (0.000). Video `docs/videos/stufe0_3d_haende_dds.mp4`
      - **Where the adapted sim lives:** The LeRobot sim on Hugging Face has **no license statement**, and neither has the BrainCo download
        → no modified copy in the repo. Instead, a build script in the repo creates the sim under `~/dev/g1-ext`;
        the script redirects LeRobot's fixed load call (`make_env("lerobot/unitree-g1-mujoco")`) to this copy.
      - Do not confuse: `BrainCoTech/brainco-hand-sdk` is the **driver for the real hands**
        (RS-485/CAN/CANFD/EtherCAT), not a simulation model. The MuJoCo model comes from brainco-hz.com
- [x] 0.4 **Training stack decided: openpi** (π₀.₅ base + LoRA, JAX) — set by the user as the tech stack of the masterplan
      (2026-09-14). Consequences: training + policy server on Ubuntu with an NVIDIA GPU · datasets in LeRobot **v2.1** · Mac = sim +
      recording + `openpi-client` · weights under the Gemma license → SAP legal. Fallback: LeRobot π₀.₅ (research
      `docs/STACK_RESEARCH_2026-09-14.md`). Details `PLAN.md` → SCOPE UPDATE — DONE
- [ ] 0.5 Clarify Linux + GPU access (Blocker 3) · accept the PaliGemma license on Hugging Face

### Stage 1 — Grasping at the table, without training (masterplan M2, Track A)
- [x] 1a Cameras (head + 2x wrist) built in — DONE
- [x] 1b Table scene with red bottle + distractors — DONE
- [x] 1c BrainCo Revo2 attached (nu 29 → 41, acceptance check `16_hand_check.py`) — DONE
- [x] 1c2a Tool for fine-tuning the hand mount (web control panel) — DONE
- [x] 2a Pixel→3D grasp point verified (0.9 cm error) — DONE
      ⚠️ *The bounding boxes are still typed in by hand (`VLM_BOXES` in `13_grasp_target.py`).*
- [ ] 1.0 **Test scaffold** — *started 2026-09-14: `tests/` (unit/integration/E2E), `scripts/run_tests.sh`; 12/13 not yet converted* (`pytest`, folder `tests/`): turn `12_pixel_to_3d.py`, `13_grasp_target.py` and
      `16_hand_check.py` into tests + model checks (nu 41, 12 hand motors, both `grasp_site`,
      torque limits, bottle mass). Slow sim tests with `@pytest.mark.slow`. From 1.2 on, every new module
      comes with tests. Acceptance check: `pytest -m "not slow"` runs; only the known gaps from 1.1 may fail
- [x] 1.1 **Scene finished — DONE 2026-09-15** (`53_build_table_scene.py` 10/10, start pose `table_env.READY_POSE`, log 2026-09-15).
      Built on the walking model (`g1sim/table_scene.py`); the points below are done there. Original list (on the MJCF base from 0.3):
      - ~~(1c2) Fine-tuning the mount~~ → done 2026-09-14: official mount from Unitree's G1+BrainCo URDF incl. adapter flange
      - add a left `grasp_site` — today only `rh_grasp_site` exists
      - bottle 0.35 → 0.5 kg · `condim` 3 → 4 on objects and fingertips
      - set torque limits (today `forcerange` 0 = unlimited): legs 88/88/88/139/50/50 Nm,
        waist 88/50/50, arms 25, wrist pitch/yaw 5 Nm (values from `lerobot/unitree-g1-mujoco`)
- [x] 1.1b **Add-on parts of the real G1 — DONE 2026-09-16** (log 2026-09-16): AGX backpack + tiltable D455 head module from Unitree's URDF
      (`g1sim/payload.py`), `head_camera` = D455. 51 · 53 · 54 (5/5) · 50 (10 m) passed
- [x] 1.2 (1d) **Action space module 26-D `g1_brainco` — DONE 2026-09-14:** `g1sim/action_space.py` + `tests/test_unit_action_space.py`.
      ~~Action space module~~ as single source of truth + round-trip test.
      Layout and normalization are now known (log 2026-09-14); the DoF answer is no longer needed for this.
- [ ] 1.3 (2a2) **Close the VLM loop**: real API call → label + box → existing geometry path.
      Acceptance check: "take the green cup" does not return the bottle.
- [x] 1.4 (2b) **Grasping without AI — DONE 2026-09-15** (`54_grasp_scripted.py` 5/5, log 2026-09-15): IK with `mink` (`g1sim/grasp_ik.py`),
      the head camera finds the bottle (`g1sim/perception.py`), sequence with a re-correction before closing (`g1sim/grasp.py`).
      Open: placing · bottle hangs 41° tilted (→ D2: ~2.7°, 2026-10-02) · with a locked wrist (23 DoF) 0/5 —
      **resolved: the robot has 29 DoF (2026-10-07)**
- [x] 1.5a (2c) **Evaluation with random positions — DONE 2026-09-15:** `54_grasp_scripted.py --randomize --trials 50` → **50/50 (100 %)**
      with ±5 cm offset of all objects (log 2026-09-15). The same success definition will later measure π₀.₅ (2.3)
- [x] 1.5b **Domain randomization — DONE 2026-09-22** (log 2026-09-22): all groups from `configs/randomization.json`, evaluation per group.
      Reachable bottle: **90 %** with everything, 95–100 % per group. Out of reach: 30/50 → "align" step before grasping (Stage 4)
- [x] 1.5c **Learning path L3a part 1 / D1 — DONE 2026-10-02:** head camera 16:10 like the D455, hand out of the image before the first look →
      reachable bottle **95 %** (same start poses as 1.5b). Open (D2): tilt ~40°, placing

### Stage 2 — π₀.₅ learns grasping from sim demos (M3–M7)
- [x] 2.0 **openpi link — DONE 2026-09-15:** `pi05_g1_brainco_lora`, conversion, converter npz → LeRobot v2.1, stand-in server,
      loop sim ↔ server (`60_openpi_closed_loop.py` 8/8). Checks the wiring and formats, not grasping (log 2026-09-15)
- [x] 2.1a **Teleop tool — DONE 2026-09-22** (log 2026-09-22): show demos yourself in the browser (keyboard, gamepad, iPhone),
      same format as the demo factory. `57_teleop.py`, acceptance check `58_teleop_check.py` 16/16. Not yet used by a human
- [ ] 2.1 Recorder → LeRobot dataset + `validate_dataset.py` (masterplan §9.2 + units/order against 1.2).
      Acceptance check: 100 episodes pass the validation (M4)
- [ ] 2.2 Small ACT sanity run, then π₀.₅ on a rented GPU (M5)
- [ ] 2.3 Closed loop in the harness from 1.5: > 60 % at a fixed position (M6), > 85 % randomized (M7), comparison with Track A

### Stage 3 — Walk & navigate autonomously (M8)
No human, no gamepad. Navigation decides *where to go* and sends (vx, vy, ω) every ~50 ms.
The **walking controller** only turns this into steps and balance (sim: GR00T/Holosoma, real: the same code
or the Unitree factory controller). We do not train the walking controller ourselves. We build navigation and the orchestrator.
- [ ] 3.1 Walking with BrainCo hands in the model from 0.3, with a driven hand bridge.
      Legs = walking controller, arms + hands = our policy.
      Acceptance check: velocity command **from code**, not from the gamepad → G1 walks
- [x] 3.2 Two-room scene + autonomous navigation, **intermediate step with a known position**
      (the sim gives the pose; 2D map + A* → vx, vy, ω). Acceptance check: change rooms without help, without collisions
      — **DONE 2026-10-02 as learning path L1 + L2** (`61_goto_check.py` 10/10, `62_rooms_check.py` 10/10, 0 wall contacts, log 2026-10-02)
- [ ] 3.3 **Full autonomy incl. localization:** simulate Livox Mid360 + IMU in MuJoCo → Fast-LIO + Nav2
      on the Linux box against the sim, no ground-truth pose any more. Acceptance check M8: room change without collisions,
      target point ±10 cm (masterplan T2/T3). The same navigation stack later runs on the real robot
- [ ] 3.4 Carry test: bottle close to the body (the walking controller does not know the arm pose), held for 10 s while walking (T8)

### Stage 4 — Orchestrator (M9)
- [ ] State machine as in masterplan §13, fixed joint authority per state, hand-overs §13.1.
      Acceptance check: a complete fetch-and-bring cycle in the sim

### Stage 5 — Sim2Real (M10/M11)
- [ ] Safety guard · hardware day read-only · navigation stack from 3.3 with the real Mid360 · 50–150 real episodes + co-training

## MASTERPLAN COMPARISON (2026-09-14)
Status against the milestones from `docs/MASTERPLAN.md` §16. Numbers measured on 2026-09-14.
Corrections to the masterplan itself: Appendix A in `docs/MASTERPLAN.md`.

| M | Criterion | Status | Evidence / what is missing |
|---|---|---|---|
| M0 | Environment runs | ✅ | Phase 0 DONE (macOS instead of Ubuntu, on purpose) |
| M1 | 60 s stable | 🟡 | Fixed base passed (30,000 steps, 0 warnings, bottle drift 0.22 cm). Free standing not needed |
| M2 | Table + bottle + camera | ✅ | Stage 1.1 (2026-09-15): scene on the walking model, 3 cameras, 30 images per sim second, start pose without contacts · 1.1b (2026-09-16): head camera = D455 on the tiltable head module (pose + FOV); image format 16:10 still open |
| M3 | Recording pipeline | 🟡 | Recording + converter ready (2.0: episode valid, v2.1 round trip exact) · demo factory ready (1.4: 5/5, 1.5b: 90 % reachable) · teleop ready (2.1a: 58 16/16) · real demos missing → 2.1 |
| M4 | Dataset v1 | ❌ | Stage 2.1 |
| M5 | First fine-tuning | ❌ | Stage 2.2 |
| M6/M7 | Closed-loop grasping | ❌ | Stage 1.3–1.5, 2.3 |
| M8 | Navigation in sim | 🟡 | Walking by command from code ✅ (0.2) · BrainCo hands ✅ (0.3) · **two-room scene + A* with known position ✅ (L2, 2026-10-02: 10/10, 0 wall contacts)** · localization without sim knowledge missing → L7 / 3.3 |
| M9 | Full task | ❌ | Stage 4 |
| M10/M11 | Sim2Real | ❌ | Stage 5 |

## CURRENT STATUS (2026-09-14)
**Goal:** Masterplan — the G1 does the fetch-and-bring task **fully autonomously**, built in Stages 0–5.
Only grasping is trained (π₀.₅). Legs and balance come from a ready-made walking controller; *where* it walks is
decided by navigation (Fast-LIO + Nav2) and the orchestrator — we build those. Our own RL walking stays dropped.

**Built and proven:**

| What | Where | Evidence |
|---|---|---|
| MuJoCo + G1 on the Mac | `.venv`, `scripts/00_view_g1.py` | Phase 0 |
| Table scene: table, red bottle, 2 distractors, head + 2 wrist cameras | `models/g1_brainco/scene_table.xml`, `scripts/10_build_scene.py` | Phase 1a/1b |
| BrainCo Revo2 on both wrists (table scene) | `scene_table_hands.xml`, `scripts/15_attach_hands.py` | `16_hand_check.py`: all fingers close |
| Pixel → 3D grasp point | `scripts/13_grasp_target.py` | 0.9 cm mean error |
| Web control panel (41 sliders, hand mount live) | `dashboard/`, `scripts/40_dashboard.py` | 23.7 fps in operation |
| **LeRobot G1 sim + GR00T walking controller on the Mac** | `scripts/50_spike_lerobot_walk.py`, environment `~/dev/g1-ext` | 10 m twice without a fall, 0.52 m/s |
| Video of a walk instead of a window | `--video` → `docs/videos/stufe0_2_g1_laeuft.mp4` | 20.6 s; camera stream 16 images/s |
| Docs: masterplan comparison, stage plan, data strategy, commands | `docs/MASTERPLAN.md`, `PLAN.md`, `docs/DATA_STRATEGY.md`, `COMMANDS.md` | GitHub (private) |

**Not built yet:**
- ~~BrainCo hands in the walking model~~ ✅ 0.3b/c · DDS rerun 0.3d open
- Test scaffold started (`tests/`, `scripts/run_tests.sh`), E2E never run → 1.0
- Scene physics: left `grasp_site`, torque limits, bottle mass, `condim` → 1.1
- Action space module 26-D → 1.2 · VLM in the loop (boxes still typed by hand) → 1.3 · IK + grasping → 1.4 · eval harness → 1.5
- Data recording, dataset, training, π₀.₅ → Stage 2 (needs Linux + GPU)
- Navigation, two-room scene, carrying → Stage 3 · orchestrator → Stage 4 · real robot → Stage 5

**Known problems:**
- The MuJoCo window (`mjpython`) freezes in the LeRobot environment when it opens; two such processes are still running
  and only go away at logout → only `--headless`, to watch use `--video`
- Without course correction 3.4 m sideways drift over 10 m → navigation must control the direction
- The sim only reaches ~0.5× real time on the Mac
- If all hand motors close at the same time, the thumb hits the index finger (right 0.26 instead of 1) → grasp strategy in 1.4:
  rotate the thumb first
- Aborted sim runs leave the camera process on port 5555 → `g1sim/runtime.py::free_camera_port()` cleans it up
- Open hardware questions: 23 or 29 DoF · Linux computer with GPU (Blocker 3)

## LOG
- [2026-09-07] Phase 0.0 — Project scaffold (README, PLAN, PROGRESS, CLAUDE) created — Status: DONE
- [2026-09-07] Phase 0.1 — venv on Python 3.12.14 (system Python 3.14 cannot be used for the robotics stack) — Status: DONE
- [2026-09-07] Phase 0.2 — mujoco 3.12.0 + mujoco-menagerie 2026.9.0 installed, mjpython present — Status: DONE
- [2026-09-07] Phase 0.3 — G1 loaded: nq=36 nv=35 nu=29 ncam=0; all 29 actuators documented — Status: DONE
- [2026-09-07] Phase 0.4 — Offscreen RGB+depth rendering verified on the M4 (docs/render_check.png) — Status: DONE
- [2026-09-07] Phase 0.5 — Viewer visual check confirmed by the user (G1 twitches and falls) — Status: DONE
- [2026-09-07] **PHASE 0 COMPLETE** — acceptance criterion met.
- [2026-09-07] Phase 1.a — Head + 2 wrist cameras built in via MjSpec (before: ncam=0) — Status: DONE
- [2026-09-07] Phase 1.b — Table scene: table, red bottle, green cup, blue cube;
  base fixed (no freejoint), assets copied along → the model loads on its own.
  `models/g1_brainco/scene_table.xml`, nq=50 nv=47 nu=29 ncam=3 — Status: DONE
- [2026-09-07] Phase 2.a — **Pixel→3D pipeline verified.** Naive point approach: 6.6 cm error.
  Cause found: the depth buffer sees the front surface, not the center. Correction via
  bounding box width → radius → shift along the view ray. **New error: 0.9 cm
  mean / 1.2 cm max.** Pure geometry, no training. — Status: DONE
- [2026-09-07] Tools — `21_watch.py` (viewer) and `22_record_video.py` (mp4 without a window)
  created, so results can be seen at any time — Status: DONE
- [2026-09-07] **Benchmark measured on this Mac** (`20_benchmark.py`): G1 locomotion
  **3,863 env steps/s on 1 core** (12 cores available). — Status: DONE

- [2026-09-07] **RL learning experiment "standing" done** (for learning, not a deliverable).
  Environment `envs/g1_stand.py`, PPO over 10 CPU processes, ~7,500 steps/s.
  - Run 1: too much exploration (ACTION_SCALE 0.35, default log_std) → it barely learned,
    at 179k steps only 43/400 steps, worse than "do nothing" (169/400).
  - Run 2 after the fix (`log_std_init=-1.6`, ACTION_SCALE 0.22): at 179k steps
    already 95/400. After 1.9 million steps (4.5 min): **295/400 = 74% held,
    reward 110 → 1098.** Stopped on request, so there is no saved policy.
  - Curves: `runs/stand/progress.csv`, `runs/stand/lauf1_progress.csv`, `docs/lernkurve.png`
  - **Lesson: compute was never the problem — tuning is.** — Status: DONE

- [2026-09-09] **Team session evaluated + tools checked against the primary sources.**
  Three corrections to PLAN.md (entered there as K1-K3):
  - **K1:** `unitree_lerobot` supports the **BrainCo hand explicitly** (next to Dex3, Dex1,
    Inspire) and trains π₀, π₀.₅, GR00T, ACT, Diffusion Policy. `xr_teleoperate`
    supports BrainCo + G1 in 23 **and** 29 DOF. LeRobot v0.5.0 (March 2026) has full
    G1 support. → The gap to Track B is **no longer a tooling problem, only
    a data problem.** PLAN.md was out of date at this point.
  - **K2:** The 23/29 DOF blocker is defused (Blocker 1); it only blocks 1d now.
  - **K3:** The whole VLA stack is **Linux-only** — this clashes with the SAP one-way rule
    (Blocker 3). Must be clarified before the first SAP day.
  - **Model choice:** π₀ / `openpi` is **Apache-2.0** and so solves the license problem
    that UnifoLM-VLA-0 (CC BY-NC-SA) would have had for an SAP business case.
  - **Architecture:** "Orchestrator + VLA specialists" is Hi Robot (arXiv 2502.19417),
    System 2 plans / System 1 (π₀) acts. Do not design it ourselves, cite it.
  - **Most important point:** A humanoid never stands exactly the same way twice →
    grasping must be **closed-loop**. This also holds for our geometry path, not only
    as an argument for VLA. Entered in PLAN.md Phase 2. — Status: DONE

- [2026-09-09] **Phase 1.c — Revo2 hands attached.** `15_attach_hands.py` +
  `16_hand_check.py` new.
  - Model taken from brainco-hz.com (`Revo2_xml.zip`, no login) → `models/revo2/`.
  - Placeholder `*_rubber_hand` (visual geometry only) removed, real hands attached to
    `*_wrist_yaw_link` via `MjSpec.attach`. **nu 29 → 41**, nq 50 → 77.
  - **12 hand actuators** confirmed (6 per hand: thumb ×2, index/middle/ring/pinky).
    The other joints are coupled via `equality` constraints (underactuated).
    Matches the "12 hand DOF" from issue #46 exactly.
  - Acceptance check: all 10 fingertip links move 3.4–5.8 cm when closing.
    Image: `docs/hands_open_closed.png`. — Status: DONE

- [2026-09-09] **Two findings that affect 1d:**
  1. **The vendor models are not symmetric.** The right hand has extra
     `*_tip` bodies (17 bodies), the left one ends at `*_distal_link` (12 bodies).
     Not relevant for the action space (6 actuators per side), but a trap for any evaluation by
     body name — do not match on `"tip"`.
  2. **Unit mismatch sim ↔ real.** The MuJoCo model drives the hand in
     **radians** (ctrlrange 0…1.46 fingers, 0…1.51 / 0…1.04 thumb). Issue #46
     describes the real `g1_brainco` stack as **normalized 0.0…1.0**.
     → 1d needs an explicit conversion. Exactly the kind of mismatch that
     silently breaks Sim2Real. — Status: OPEN

- [2026-09-09] **Web control panel built (dashboard S1).** New: `dashboard/`
  (`scene.py`, `drivers.py`, `sim.py`, `server.py`, `static/`) + `scripts/40_dashboard.py`.
  - **Architecture chosen from measurements, not guessed:** the model loads in 148 ms,
    physics 15,079 steps/s (30× real time), rendering 56 fps at 640×480.
    → The server renders offscreen, the browser shows the stream. No WASM, no second
    physics stack. Measured in operation: **960×720 at 23.7 fps**.
  - **Driver seam** (`drivers.py`): `RobotDriver` as an abstract interface,
    `MujocoDriver` implements it. `DdsDriver` can be added next to it later without rework
    (`unitree_hg`, not `unitree_go`). This is the place where Johannes' dashboard
    can connect.
  - **No duplicate:** The build logic from `15_attach_hands.py` moved to
    `dashboard/scene.py::build_with_hands()`. Script and dashboard call
    the same function; `15` and `16` still run unchanged.
  - **Mount adjustable live:** slider → recompile (~180 ms) → visible in the image at once.
    The set values are kept; the robot does not jump to the zero pose.
    Verified: +5 cm z at the mount point → the hand base moves by exactly 5.0 cm.
  - **Grasp width live** (distance hand base→fingertips, cm) — the same value that
    `16_hand_check.py` measures as its acceptance check. In the test: 11.2 → 9.0 cm when closing.
  - Also needed: `<visual><global offwidth/offheight>` in the scene, otherwise
    MuJoCo limits the offscreen buffer to 640×480. — Status: DONE

- [2026-09-14] Stage 0.0 — **Masterplan comparison + stage plan** (docs only, no code).
  `G1_VLA_Masterplan.md` checked against the repo and copied to `docs/MASTERPLAN.md`
  (original unchanged + Appendix A "Status & corrections").
  - **Measured:** fixed-base scene stable for 60 s (30,000 steps, 0 warnings, bottle drift 0.22 cm).
    3 cameras at 224×224 = 18.5 fps live — not relevant for scripted recording, because the sim pauses while rendering.
  - **Findings in the scene:** only the right hand has a `grasp_site` · `forcerange` 0 = no torque limits ·
    `condim` 3 everywhere · bottle 0.35 kg.
  - **Action space clarified:** `unitree_lerobot` `Unitree_G1_Brainco` = 7 arm joints per side (incl. wrist
    pitch/yaw) + 6 hand joints per side = 26. **Holds for 23 and 29 DoF.** Hand according to `xr_teleoperate`:
    normalized 0…1, **0 = open, 1 = closed**, order thumb, thumb aux, index, middle, ring, pinky.
    Real ranges 1.52 / 1.05 / 1.47 rad, sim 1.51 / 1.04 / 1.46.
  - **New path for walking:** LeRobot v0.5 has a G1 MuJoCo sim (`lerobot/unitree-g1-mujoco`, 29 DoF, DDS,
    torque limits) with ready-made walking controllers (GR00T-WBC, Holosoma). The same code runs on the real robot.
    Hands there: Dex3, not BrainCo. Real interfaces: `LocoClient.Move(vx, vy, yaw_rate)` for walking,
    `rt/arm_sdk` for the arms at the same time.
  - **Rejected:** `unitree_rl_gym` `motion.pt` — trained on `g1_12dof.xml` without arms.
  - **Masterplan errors** (details in Appendix A): openpi has no YAML configs · the format is LeRobot, not HDF5/Zarr ·
    the action is 26-D, not "29 + hand" · gamepad teleop for 26 DoF is not practical.
  - Scope decision → `PLAN.md`, NEXT STEP rewritten. — Status: DONE

- [2026-09-14] Stage 0.1 — Status saved in two commits, private GitHub repo
  `finneganadam-maker/UnitreeG1-Robot` created and pushed. `MUJOCO_LOG.TXT` no longer versioned. — Status: DONE

- [2026-09-14] Stage 0.2 — **LeRobot G1 sim + GR00T walking controller run on the Mac. Acceptance check passed.**
  - **Result:** 10.0 m without a fall in 19.6 s sim time (0.51 m/s at `remote.ly = 0.5`), pelvis constant at 0.75 m,
    real-time factor 0.54 (35.9 s wall clock). Walk command from code, no gamepad. The controller only drives
    legs + waist (joints 0–14); the arms stay free.
  - Environment outside the repo: `~/dev/g1-ext` (LeRobot 0.6.2, torch 2.11, onnxruntime 1.30, mujoco 3.13,
    `unitree_sdk2py` + self-built CycloneDDS 0.10.5). Setup and start: `COMMANDS.md`.
  - **Mac pitfalls solved:** CycloneDDS is missing in Homebrew → built it ourselves · LeRobot/sim use a fixed `lo`
    → initialize `lo0` first (the channel factory is a singleton) · `scipy` is missing from the sim requirements ·
    the process hangs after the end (DDS threads) → `os._exit` · without a window, `mj_forward` must run before the first step,
    otherwise the elastic band crashes with a zero quaternion.
  - **Open / limits:**
    - The MuJoCo window under `mjpython` freezes in this environment already **when it opens** (process `UE`,
      cannot be killed, only goes away at logout). Reproduced twice. → `--headless`; to watch,
      `--video` renders an mp4 offscreen (the camera follows the pelvis). Offscreen rendering works.
    - The sim has **Unitree Dex3 hands**, not BrainCo → Stage 0.3
    - **Drift without course correction:** over 10 m, 3.4 m to the side (Δ = 9.4 / −3.4 m). Navigation must actively
      control the direction (Stage 3); a plain forward command is not enough
    - ~~Camera image transfer not checked~~ → checked in the repeat run with `--video`: head camera over
      ZMQ port 5555 at ~16 images/s, 700 sent, 0 dropped. Same run: 10.0 m without a fall, 0.52 m/s,
      0.51× real time, video `docs/videos/stufe0_2_g1_laeuft.mp4` (20.6 s)
  - Answers Blocker 6: **yes, it runs on the Mac.** — Status: DONE

- [2026-09-14] Stage 0.3a — **Model comparison LeRobot G1 ↔ Menagerie G1 + groundwork for BrainCo in the sim.**
  - **Same:** 29 body joints (names, order, limits — max. difference 0.0 rad), wrist mount point
    (`right_wrist_yaw_link` pos 0.046 / 0 / 0).
  - **Different:** `torso_link` 9.598 kg (LeRobot) vs. 7.818 kg (Menagerie), total −1.77 kg · joint damping
    0.05 vs. 0.5 · LeRobot has foot dummies and `*_rubber_hand` as separate bodies · actuators: LeRobot 29
    torque `<motor>` (the sim computes PD itself), Menagerie position controllers.
  - **Hand masses** (subtree from `right_wrist_yaw_link`): Revo2 0.66 kg, Dex3 0.78 kg, wrist alone 0.25 kg
    → Revo2 ≈ 0.41 kg, Dex3 ≈ 0.53 kg per hand.
  - **Revo2 MJCF:** 6 `<position>` actuators (kp 30, kv 10, no force limits), joints damping 0.5 /
    armature 0.01 / frictionloss 0.1, 5 joint couplings per hand.
  - **Real BrainCo hands over DDS** (`xr_teleoperate`): `unitree_go` `MotorCmds_`/`MotorStates_`,
    `rt/brainco/{left,right}/{cmd,state}`, 6 motors, `q` normalized 0…1.
  - LeRobot loads the sim only from Hugging Face (fixed in `UnitreeG1.connect()`); the HF repo has no license statement.
  - **Decision:** base = LeRobot model (the walking controller is tuned for it). — Status: DONE

- [2026-09-14] Stage 0.2 — **Cleaning up the sim environment.** Two MuJoCo window processes (PID 87491, 90784)
  hang in the kernel on exit (`UE`): `kill -9` has no effect, and neither has quitting `spindump_agent`.
  All other leftovers ended (old DDS test PID 86698, orphaned camera process, shells), port 5555 free.
  → The two processes only go away after logging out and in again, or after a restart. They use no CPU and
  do not block the run without a window. — Status: DONE (logout still pending)

- [2026-09-14] Stage 0.3b — **LeRobot G1 sim with BrainCo Revo2 built. Acceptance check passed (10/10).**
  - New package `g1sim/`: build recipe (`build.py`), Revo2 environment + DDS bridge (`brainco_env.py`), hand normalization (`brainco.py`).
    The LeRobot sim stays unchanged; `use_brainco_sim()` redirects LeRobot's fixed `make_env` call to `~/dev/g1-ext/g1_brainco_sim`.
  - **Finding 1 — index error in the original sim:** motor = joint index − 1. With the coupled Revo2 joints, the
    torques of the right arm landed on hand motors. The mapping now comes from the model (`actuator_trnid`).
  - **Finding 2 — the hand mount was wrong, also in the table scene:** fingers 90° upward, left hand twisted, left thumb 14 mm inside the
    wrist. Replaced by Unitree's official URDF (`unitree_ros` commit 43e9f3e, BSD-3): hand base 5.91 cm in front of the wrist,
    rotation left (−90°, 0, −90°) / right (90°, 0, 90°), plus an adapter flange of 23 g (`models/unitree_brainco_mount/`).
    Check: `dashboard/scene.py::mount_error()`.
  - **Finding 3:** The right Revo2 has 5 loose fingertip joints (no motor, coupling or limits) → removed, both hands now have 11 joints.
  - Hand actuators as `<motor>` (limits from the vendor model), PD kp 20 / kd 0.3: thumb rotation t90 1.7 s, fingers 0.66 s, no overshoot.
  - Table scene rebuilt (nq 77 → 72), `16_hand_check.py` still passes. — Status: DONE
- [2026-09-14] Stage 0.3c — **10 m with BrainCo hands without a fall. Acceptance check passed.** 10.0 m, 0.51 m/s, 0.51× real time. Arms track
  to 0.077 / 0.078 rad (Dex3 comparison run `runs/stufe0_3c_baseline_dex3.json`: 0.083 / 0.082) → motor mapping confirmed.
  - **Finding 4 — race condition in `UnitreeG1.connect()`:** The sim thread already computes while the main thread calls `reset()` →
    zero quaternion, the sim thread dies silently, the sim time stays 0 (several times, depends on chance). Fixed: a lock between step
    and `reset()`. A counter-check without the lock finds the error (`tests/helpers/sim_env_probe.py --ohne-sperre` (without lock)).
  - New `g1sim/runtime.py`: cleans up orphaned camera processes on port 5555, stall watchdog with thread stacks. — Status: DONE
- [2026-09-14] Stage 0.3d — DDS bridge `rt/brainco/*` built (`scripts/52_brainco_hand_dds.py`): DDS state = sim (0.000), left hand
  half closed to within 0.001, opening ≤ 0.002. The run only failed on a criterion that was too strict (thumb at the same time as the fingers). Script changed to three
  phases, rerun open. — Status: WIP
- [2026-09-14] Stage 1.0 (brought forward) — **Test scaffold:** `pytest.ini`, `tests/` in three levels, `scripts/run_tests.sh`. Unit + integration
  green (16 in `.venv`, 28 in `~/dev/g1-ext/.venv`). E2E written, not run yet. Regression tests with counter-checks (old mount,
  old index, without lock). — Status: WIP
- [2026-09-14] Stage 0.4 — Research on the training stack (raw report `docs/STACK_RESEARCH_2026-09-14.md`): LeRobot π₀.₅ = ported
  openpi weights, LoRA + expert-only, dataset v3.0, RTC; openpi reads only v2.1, PyTorch version without LoRA, Ubuntu only. Weights under the
  Gemma license (ask SAP legal). Risk: an open issue says LeRobot π₀.₅ runs less smoothly than openpi (#2559) → early A/B test.
  Costs according to the research: H100 ~2 $/h, expert-only run estimated at 5–8 h. — Status: WIP (to be decided together)
- [2026-09-14] Stage 0.3d — **Acceptance check passed.** Three phases, each of the 12 hand motors over DDS to within 0.1, DDS = sim. — Status: DONE
- [2026-09-14] Stage 0.4 — **openpi decided** (user: tech stack of the masterplan). `scripts/setup_openpi.sh`: openpi 215abfb to
  `~/dev/g1-ext/openpi`, macOS with CPU JAX (override `jax==0.5.3`), `openpi-client` without dependencies into the sim environment
  (numpy 2.2 there). Import checked. `uv` installed via Homebrew. — Status: DONE
- [2026-09-14] Stage 1.2 — 26-D action space `g1sim/action_space.py`, layout = unitree_lerobot `Unitree_G1_Brainco`, delta mask for the arms. — Status: DONE
- [2026-09-14] Stage 1.1 — Grasp scene on the walking model (`g1sim/table_scene.py`), environment `g1sim/table_env.py` (300 Hz, control rate 30 Hz,
  LeRobot PD), raw recording format `g1sim/episode.py`. Check `53_build_table_scene.py`: 6/8 — robot touches the table, bottle slides 10.6 cm. — Status: WIP
- [2026-09-15] Stage 1.1 — **Grasp scene repaired, acceptance check passed (10/10).** Measured first, then changed:
  - **Cause 1 — objects were stuck in the hands:** Already in the zero pose (before any physics) the right fingers were 2.1 cm inside the
    bottle and the left pinky 0.3 cm inside the cup. While settling in `reset()` the physics knocked the bottle over (tilted 35° after 0.3 s,
    then lying down) — the 10.6 cm happened before the hold.
  - **Cause 2 — the check script let the arms sag:** holding with `apply(env.state())` (target = the pose just reached) → elbow in 2 s
    0.20 / 0.29 rad, wrist pitch −0.38 rad, right pinky on the table after 0.17 s. With a fixed target: ≤ 0.06 rad, no table contact.
  - **Start pose measured, not guessed:** 504 arm poses checked against the table and the object zone, three candidates held for 2 s and looked at. Chosen:
    forearms raised (elbow −0.6, wrist pitch −0.3 rad), hands just in front of the table edge above the objects, ≥ 12 cm distance even with
    ±5 cm random offset, head camera clear. Shortest path to the bottle, no path under the table edge (alternative "arms hanging" rejected).
  - `g1sim/table_env.py`: `READY_POSE`, `reset()` sets the arms there directly, `robot_contacts()`. `53_build_table_scene.py` is stricter: contacts over
    the whole hold time, sag < 0.1 rad, 10 random starts without contact. Result: drift 0.0 mm, sag 0.05 rad, 0 contacts.
  - New `tests/test_integration_table_env.py` (5 tests, 1.5 s) incl. counter-checks: zero pose → fingers in the bottle; `state()` as target → sag.
  - **Open, not a blocker:** In the start pose the palms face inward; each wrist camera sees the other hand, not the objects.
    For π₀.₅ what counts is what the camera sees on the approach → check in 1.4 (IK/grasping). — Status: DONE
- [2026-09-15] Stage 2.0 — **openpi link built, acceptance check on the Mac passed.** `pi05_g1_brainco_lora` loads in the openpi environment,
  the loop sim ↔ stand-in server ran one episode (`scripts/60_openpi_closed_loop.py` 8/8).
  - `openpi_g1/policy.py`: 3 cameras → `base_0_rgb` / `left_wrist_0_rgb` / `right_wrist_0_rgb`, 26-D state, output cut to 26.
    `config.py`: π₀.₅, `action_horizon` 30 (= 1 s at 30 Hz), LoRA on Gemma 2B + 300M with a freeze filter, no EMA, arms as delta / hands absolute,
    base `pi05_base`; it is added to openpi's `_CONFIGS` at runtime, openpi itself stays unchanged. `convert.py`: npz → LeRobot v2.1
    (only `success=True`). `run.py`: check / stand-in / norm-stats / train / serve; `train` refuses to run on the Mac, `--dry-run` shows the configuration.
    `stand_in.py`: openpi's `Policy` + `WebsocketPolicyServer` with the real conversion chain, only the model is replaced (it answers "hold the start pose").
  - Sim side: `g1sim/openpi_env.py` (openpi-client environment + episode store), loop like openpi's ALOHA sim example.
  - **Loop result:** 300 control steps (10 s sim), 10 server calls, latency 9 ms (the first one 270 ms: JAX compiles), actions arrive with
    ≤ 6·10⁻⁸ difference, arms hold (0.05 rad), 0 contacts, episode valid, 0.52× real time.
  - **Dataset round trip:** the same episode → `convert.py` → openpi's data loader: 300 frames, images pixel-identical, state and delta actions exact.
  - **Finding — package versions of the openpi environment on the Mac:** installed without `uv.lock`, so `datasets` 5.0.1 instead of 3.6.0 → LeRobot 0.1.0
    cannot load a dataset (`torch.stack` on `Column`). This would only have shown up when computing the norm statistics or training. Set to the lockfile versions
    (datasets 3.6.0, pyarrow 20.0.0, huggingface-hub 0.32.3, fsspec + gcsfs 2025.3.0) and pinned in `setup_openpi.sh`.
    Linux with `uv sync` is not affected.
  - `READY_POSE` moved from `table_env.py` to `g1sim/action_space.py`: the sim, the server metadata and later the real robot share it.
  - Tests: `tests/test_unit_openpi_policy.py` (4), `tests/test_integration_openpi.py` (6), `tests/test_integration_openpi_env.py` (4);
    `scripts/run_tests.sh` has the openpi environment as a third block. The PaliGemma tokenizer (4 MB) is in `~/.cache/openpi`.
  - **Honest:** what is checked is the wiring and the formats, not grasping. π₀.₅ weights not loaded. `batch_size` 32 / `num_train_steps` 30,000 are
    openpi's defaults — measure real values in 2.2 on the GPU. — Status: DONE
- [2026-09-15] Organization — **Repo location as set by the user:** code only in `~/Documents/Projekt Tim and Lukas/Cursor/Software Engineering/
  SAP Robot Project/First Ideas/UnitreeG1-Robot`, saved only in the GitHub repo `finneganadam-maker/UnitreeG1-Robot` (rule in `CLAUDE.md`).
  - Moved from `~/dev/g1-brainco` with the Git history and all unversioned files (hand meshes, assets, images, videos, `runs/`);
    checksum comparison: 536 files identical. The 4 drafts in `First Ideas/` stay untouched next to it.
  - Project venv rebuilt as `.venv.nosync` (iCloud "Desktop & Documents" does not upload it) + link `.venv`, the same 63 packages.
    Path with spaces: the `mjpython` start line now uses a `/bin/sh` redirect; calls via `.venv/bin/python -m …` are not affected.
  - The openpi environment points to the new location via `g1_brainco.pth`. Third-party software stays in `~/dev/g1-ext`.
  - The earlier reason for `~/dev` (spaces break ROS 2 / CMake) applies to the Linux computer where ROS 2 will run — not to the Mac. — Status: DONE
- [2026-09-15] Stage 1.4 — **Grasping without AI, acceptance check passed (5/5).** Bottle lifted ≥ 5 cm from a fixed position and held for 5 s, no contact
  with table, cup or cube. `scripts/54_grasp_scripted.py --video`: lift min 8.7 cm, head camera 3.0 mm, hand before closing 1.2 mm / 0.2°.
  5 trials in 17 s compute time (19 s sim per trial; without camera images ~20× faster than real time).
  - `g1sim/grasp_ik.py`: mink 1.3.0 + solver daqp. Hand base to the target pose, all other joints locked via `DofFreezingTask`, joint limits and
    ≥ 2 cm forearm/hand ↔ tabletop as hard limits. Grasp pose to within 1.4 mm, with a locked wrist (23 DoF) to within 3.2 mm; 0.8 ms per step.
  - `g1sim/perception.py`: red mask + depth image of the head camera → lid center, 2.9 mm error. Placeholder for the image model from 1.3.
  - `g1sim/grasp.py`: look → approach → lower → look again → move in → rotate thumb → close → lift → hold. The IK works
    on commanded angles; before looking and before closing, the measured sag of the hand (position + rotation) is subtracted.
    Without the rotation the hand stayed tilted by 8.5°.
  - **Finding 1 — friction:** With MuJoCo's default (pyramidal cone, impratio 1) the bottle slipped through in 36/36 grasp variants. The grasp scene now uses
    `cone="elliptic"`, `impratio="10"` (`g1sim/table_scene.py`, recommended by the MuJoCo docs for grasping); `53_build_table_scene.py` still 10/10 afterwards.
  - **Finding 2 — grasp geometry measured:** palm 9.4 cm long, fingers start at 7.5 cm, bottle 6.6 cm diameter. If the bottle is in front of the
    finger base links, the fingers push it away (images in the chat). Chosen from ~150 sim variants: bottle axis 6.6 cm in front of the hand base,
    9 cm along the fingers, 6 cm below the lid; the rotated thumb holds from behind, the fingers from the front. Holds 9/9 with ±5 mm offset.
  - **Honest — limits:** The fingers do not reach around the bottle; it hangs 41° tilted (grasps with 8° let it fall in 1–3 of 9
    cases at ±5 mm; a grasp height of 4–6 cm below the lid changes nothing). With a locked wrist 0/5 (hand 10.2 mm off, lift 3.4 cm).
    Placing not built. Larger random offset not checked → 1.5. `rh_grasp_site` from the vendor model is 11 cm in front of the palm
    instead of between the fingers; 1.4 does not use it.
  - Wrist camera during the approach (`docs/stufe1_4_greifen_handkamera.png`): start 0 % bottle in the image (it sees the other hand), next to the bottle 25 %,
    before closing 41 %.
  - Test `tests/test_integration_grasp_ik.py`: the IK reaches the grasp pose with 29 and 23 DoF. False alarm "overflow in matmul" (numpy + Apple
    Accelerate) filtered in `pytest.ini` and in the script. — Status: DONE
- [2026-09-15] Stage 1.5a — **Evaluation with random positions: 50/50 (100 %).** `54_grasp_scripted.py` now has `--randomize` and `--seed`;
  one success definition for the acceptance check (1.4) and the evaluation (1.5, later π₀.₅ in 2.3). Failures are counted by the first criterion
  that is not met (order cause → effect: head camera, hand at grasp pose, forbidden contacts, lift). Output files are named after `--out`.
  - Result (seed 0, all objects ±5 cm): lift min 6.1–9.4 cm, head camera ≤ 11.4 mm (mostly 3 mm), hand ≤ 7.1 mm, tilt 40–42°, 0 forbidden contacts.
    50 trials in 50 s. Counter-check 1.4 (fixed position) still 5/5.
  - **Honest:** ±5 cm is the range from `TableEnv.reset`; the masterplan wants ±15 cm. No randomization of physics and image → 1.5b. — Status: DONE
- [2026-09-16] Stage 1.1b — **Add-on parts of the real G1 built in, all acceptance checks passed.** The user supplied four STL files
  (`backpack_link`, `head_servo_link`, `xl330_link`, `d455_link`). They are byte-identical to Unitree's `unitree_ros/robots/g1_description/meshes`
  (BSD-3). Poses, masses and inertias come from `g1_comp.urdf` (commit 7c40519, head module) and from the README "How to Add AGX Backpack?"
  (commit d6f13aa, backpack). The parts: **AGX backpack** 1.9 kg on the back of the upper body · **head base** 0.55 kg · **servo XL330** turns the camera
  ±50° (`xl330_joint`) · **RealSense D455** tilts −20…+90° (`d455_joint`). Together **+2.59 kg**, the robot now weighs 38.53 kg.
  - Model `models/unitree_g1_payload/` (meshes, LICENSE, `payload.xml`), module `g1sim/payload.py`, built into `g1sim/build.py` →
    the walking model and the grasp scene carry the same parts. nu stays 41, the 26-D action space stays the same: the head joints have no motor,
    one `equality` each holds the angle like a servo. `payload.set_head(model, pan, tilt)` changes it.
  - **Finding 1 — mounting frame:** Unitree's URDF `torso_link` sits 1 cm lower than LeRobot's (head mesh −0.044 instead of −0.054, both at the pelvis origin).
    `payload.ANCHOR_POS` makes up for this; the Unitree numbers stay unchanged in `payload.xml`. 51 recomputes the URDF chain independently
    (0.0 mm / 0.0°). Counter-check without the offset: 10.0 mm error, is detected.
  - **Finding 2 — head camera:** LeRobot's `head_camera` sat 3 cm above the real D435i, in the middle of the new head module. With the D455 tilted it only saw
    its housing. **`head_camera` is now the color camera of the D455** (masterplan §7.3: primary VLA input). Position from Intel's
    `realsense-ros`, fovy 65 (RGB 90×65°). The name stays, because LeRobot's sim and LeRobot's G1 server stream `head_camera`, and the grasp scene,
    perception, randomization and openpi format use it. New `d435i_camera` at Unitree's pose of the built-in D435i (fovy 42, unused).
    Grasp scene: `table_scene.HEAD_POSE` = tilt 0.85 rad (together with the servo's base tilt 51° downward): whole table, both hands, all objects in the image.
  - **Finding 3 — collision hull:** MuJoCo collides with the convex hull of the head mesh. From 0.45 rad tilt on, the D455 was stuck inside it, and the head stopped
    at 0.65 instead of 0.85. Contact D455 ↔ upper body excluded (per Unitree it tilts freely up to 90°). 51 now checks contacts over the whole tilt range.
  - **Finding 4 — MjSpec:** Without a prefix, the default class of the attached file is written without a name → the scene does not load. The class is now called
    `unitree_payload`. A `euler` rotation in the body ignores a set `quat` → servo in `payload.xml` as `quat`, otherwise `set_start_pose` aborts
    (found by the new test).
  - Acceptance checks: **51** 14/14 · **53** 12/12 (head holds 0.850 rad, follows a new target within 1 s to within 0.5 mrad) · **54** 5/5, head camera now
    2.8 mm (before 2.9 mm) → `runs/anbauteile_greifen.json` · **50 `--brainco`** 10.0 m without a fall, 0.50 m/s (before 0.51), pelvis 0.75 m, arms 0.08 rad
    → `runs/anbauteile_laufen.json`, video `docs/videos/anbauteile_g1_laeuft.mp4`. Test `tests/test_unit_payload.py` (6, both environments).
    53 now reads the bottle mass from the scene file (1.5b varies it in `env.model`).
  - **Honest — limits:** Without course correction now 0.65 m side drift over 10 m (before 0.16 m) → navigation handles that. The sim renders
    square images (224×224), the D455 delivers 16:10 → match the image format before 2.1 (masterplan §14.1). Lens height in the mesh ±2 mm, calibrate
    on the robot. The old Menagerie table scene (`models/g1_brainco/`, dashboard) has no add-on parts. — Status: DONE
- [2026-09-22] Stage 1.5b — **Randomization finished, evaluation per group.** Ranges in `configs/randomization.json`, sampling and writing into the
  model in `g1sim/randomization.py`, `TableEnv.reset(randomize=True)` or `reset(variation=...)` for a repeat. Varied: objects ±15 cm
  + rotation · stand pose ±10 cm / ±10° · mass and friction ±20 % · motor gains ±10 % · latency 0–30 ms · light (strength, direction, tint) ·
  table and distractor colors (never red) · camera pose ±5 mm / ±1.5° · image noise (RGB σ 3, depth σ 2 mm). Single groups: `54_grasp_scripted.py --vary stand`.
  - Result, 50 trials each, seed 0 (`runs/stufe1_5b_<run>.json`):

    | Run | passed | out of reach | **reachable: success** |
    |---|---|---|---|
    | Physics (mass, friction, gains, latency) | 50/50 | 0 | 100 % |
    | Image (light, colors, camera pose, noise) | 50/50 | 0 | 100 % |
    | Stand ±10 cm / ±10° | 36/50 | 12 | 36/38 = 95 % |
    | Objects ±15 cm | 24/50 | 25 | 24/25 = 96 % |
    | Everything | 18/50 | 30 | **18/20 = 90 %** |
    Counter-check without variation still 5/5, 53 still 12/12.
  - **Finding 1 — reach:** The right hand reaches the grasp pose only at about x 0.30–0.46 m in front of the pelvis and y −0.30 to +0.05 m. New first
    criterion `flasche_in_reichweite` (bottle in reach; IK computation without physics ≤ 1 cm). Consequences: Stage 4 needs an **"align"** state before grasping
    (bring the bottle into the grasp window with a small walk command, fixed rule, no language model); the demo factory (2.1) only takes reachable starts.
  - **Finding 2 — upper arm inside the torso:** For bottles near the body center the IK solution put the upper arm 14–21 mm into the torso; in the physics
    the hand got stuck 1–7 cm before the grasp pose. `g1sim/grasp_ik.py`: collision limit arm + hand base ↔ torso + pelvis (target 5 mm; it acts linearly on
    velocities, the promise is "no penetration", measured 0.4 mm left over). Without the fingers (21 of 27 parts), otherwise the IK was ~10× slower.
  - **Finding 3 — IK jumps:** With an unreachable bottle the IK jumped 9.6 cm deep through the torso in one step, two joints stayed at their limit,
    the solver found no solution any more and the evaluation aborted. Now: at most 3 rad/s per arm joint (`MAX_JOINT_SPEED`), and the joint angles
    are clamped into their range after every step (1e-10 rad over the limit was enough for the crash). Also important for teleop: no jerky arm.
  - **Finding 4 — head camera criterion:** The first look from the start pose is often off (up to 12 cm), because the robot's own right hand covers part
    of the lid. But the grasp pose only comes from the second look (2.5–3 mm). 13 of 17 "camera" failures had grasped
    perfectly. Now the second look is rated; the first one is in the result as `first_look_error_m`.
  - Also: `table_scene.write()` writes atomically (parallel runs read half a scene file); `54` no longer aborts the whole evaluation on a solver error,
    the trial counts as a failure.
  - Tests: new `tests/test_integration_randomization.py` (reset restores everything bit-exact, repeat, seed, only the bottle is red, objects
    fully on the table, wrong config key) · `test_integration_grasp_ik.py` +2 (distance to the torso, no jumps; counter-check without the
    limit: −32 mm → it fails as it should).
  - **Honest — limits:** 4 of the 5 remaining failures: first look > 3.5 cm off, the approach goes wrong. The fix (take the right hand out of the image before
    the first look) changes the sequence of all demos → decide with the user. The bottle still hangs ~40° tilted. — Status: DONE
- [2026-09-22] Stage 2.1a — **Teleop tool: show demos yourself, in the browser.** The user's question: "Can we teach the robot something in MuJoCo via
  teleoperation?" Yes. Unitree's `xr_teleoperate` needs Isaac Lab for this (Linux + NVIDIA), so we built our own small chain
  on the Mac: input → target pose of the right hand base + grasp value → limits → IK (`grasp_ik.py`) → 26-D action → `TableEnv`.
  - Building blocks: `g1sim/teleop.py` (TeleopArm, KeyPilot for keyboard + gamepad, PhonePilot, TeleopSession), `g1sim/phone.py` (iPhone receiver),
    `scripts/57_teleop.py` + `g1sim/teleop_page.html` (browser: overview over the shoulder/side/top, head and hand camera, status),
    acceptance check `scripts/58_teleop_check.py`. The MuJoCo window is not needed (it freezes on the Mac).
  - Format like the demo factory: observation before the action + 26-D joint targets, 3 cameras, 30 Hz sim time, next to it a JSON with variation and
    measurements. Success as in 54 (lifted ≥ 5 cm, no forbidden contacts); the converter only takes successful ones. Only scenes with a
    reachable bottle (`grasp.reach_error` ≤ 1 cm, now its own function). Recording starts with the first input.
  - Limits: target ≤ 0.25 m/s and ≤ 1.2 rad/s · workspace in front of the body · target at most 5 cm ahead of the IK hand (an unreachable target does not
    run away) · arm joints ≤ 3 rad/s · arm never in the table or torso (IK) · dead man's switch: 0.5 s without input from the browser → the hand stops.
  - **Finding 1 — grasp order:** The thumb physically needs ~1.8 s to rotate. With a fixed ramp (thumb "done" after 0.3 s) the fingers
    blocked it, and 1 of 3 grasps held. Now the grasp value only goes past "rotate thumb" when the **measured** thumb is ≥ 0.9
    (on the robot: from the hand's joint encoders). One key; the hand does the order itself.
  - **Finding 2 — our own bug:** The thumb lock lowered the grasp value as soon as the bottle pushed the thumb back to 0.87 while carrying → the hand
    opened, the bottle fell. Now the lock only stops further closing. Test `test_daumensperre_oeffnet_nie` (thumb lock never opens).
  - **Finding 3 — sluggish arm:** While moving, the hand lags up to 4 cm behind the target (PD without gravity compensation). At full speed you overshoot
    the target → fine control with the Shift key (¼ speed).
  - **Finding 4 — IK speed limit:** it applied per IK step, but the IK computes 2 steps per control step → actually 6 rad/s; when rotating
    into the grasp pose the wrist jumped 0.16 rad per control step. Now per control step (measured exactly 0.100 rad = 3 rad/s). 1.5b measured again
    afterwards: numbers unchanged, 54 still 5/5.
  - **iPhone:** We rebuilt the protocol of the app "MuJoCo AR" from the package `mujoco_ar` 1.4.0; the package is on purpose **not** installed: at
    start it kills every process on its port with `kill -9`. Clutch: hold the button = the hand follows the phone, switch = grasp, vibration when the hand touches
    the bottle. Checked with a simulated app client, **not with a real iPhone**.
  - Acceptance check `58_teleop_check.py` **16/16** (`runs/teleop_check.json`): a simulated operator who only presses keys as in the browser, 3/3 grasps with
    randomization (bottle lifted 11–13 cm, 12–15 s sim per episode, arms ≤ 3 rad/s, within the joint limits) · server: page, camera image,
    a key starts recording and moves the hand, discard resets · iPhone: phone 10 cm forward = hand 10 cm forward, letting go stops ·
    conversion to LeRobot v2.1 in the openpi environment. Quick test without camera images: 10/10.
  - Tests: `tests/test_integration_teleop.py` (8).
  - **Honest — limits:** During recording the sim runs at ~0.5× real time (three camera images cost ~66 ms per control step), otherwise ~0.9×. Only the
    right arm. Image format still 224×224 square → demos for the dataset only after the switch to 16:10. Not yet used
    by a human. `55_record_grasp.py` / `56_watch_grasp.py` (MuJoCo window, hangs) are replaced by the browser view. — Status: DONE (tool)

- [2026-10-02] Learning path L0 + L1 — **Learning path set** (`docs/LEARNING_PATH.md`, order confirmed by the user) · tests run by the user:
    all green (project 41, LeRobot environment 81, openpi 10; `runs/tests_unit_integration.log`) · **L1 go to a point:** tracking controller
    `g1sim/navigation.py` (pose from the sim → error into the body frame → P controller with limits, GR00T dead zone 0.05, ramp 0.5 m/s²),
    live view in the browser `g1sim/liveview.py` (port 8020, target as a floor marker), acceptance check `scripts/61_goto_check.py` **10/10**:
    mean 6.3 cm, max 8.7 cm, angle ≤ 6.3°, 10–15 s sim per target (1–4 m), no fall, sim 0.50× real time.
    Top view `docs/lernpfad_l1_wege.png`, video `docs/videos/lernpfad_l1_goto.mp4`, tests `tests/test_unit_navigation.py` (12).
    The first attempt aborted: the walking model only allows 640×480 pixels offscreen → live view at this size.
    Honest: the errors are just under the tolerance, because the controller stops when it enters the 10 cm circle and then coasts;
    more precise would only be possible with a slower start (GR00T dead zone). The position still comes from the sim (L7 replaces that). — Status: DONE

- [2026-10-02] Learning path L2 — **Map and path: 10/10 room changes, 0 wall contacts.** Two-room scene `g1sim/rooms.py` (room A + B, 1.2 m door,
    table as an obstacle; the geometry is used for both sim and map), occupancy + cost map + A* `g1sim/occupancy.py` (5 cm, impassable
    < 30 cm from a wall, more costly up to 80 cm; A* 20–60 ms), path following `navigation.PathFollower` (0.5 m look-ahead, uses the L1 controller),
    shared sim building block `g1sim/walk_sim.py`, scene via `use_brainco_sim(scene=…)`. Acceptance check `scripts/62_rooms_check.py`:
    mean 4.6 cm, max 6.3 cm, angle ≤ 8.4°, paths 1.8–5.6 m, all through the door center, path distance to the wall ≥ 0.6 m, no fall.
    Map `docs/lernpfad_l2_karte.png`, video `docs/videos/lernpfad_l2_rooms.mp4`, tests `tests/test_unit_occupancy.py` (8).
    **Finding — process hangs in the graphics driver and cannot be killed:** the 2nd attempt stopped in `mjr_render → settexture` (state UE,
    `kill -9` has no effect, holds port 8020 until logout). At that time three OpenGL contexts rendered at once (live view + video in the
    main thread, sim camera image in the sim thread). Fix: `use_brainco_sim(cameras=False)` for walking scripts (the sim renders no camera images)
    and **one** renderer for browser and video (`LiveView(video=…)`); the live view moves to the next free port. Side effect:
    sim 0.83× instead of 0.50× real time. For L3 (head camera needed) the rendering must stay in one thread. 1st attempt: IndexError while
    reading the contact list, because the sim keeps computing in its own thread → snapshot `np.array(data.contact.geom)`.
    Demo on the side: 2D LiDAR via `mj_multiRay` (720 rays in 1.35 ms), `docs/lernpfad_lidar_demo.png`. — Status: DONE

- [2026-10-02] Learning path L3a part 1 / D1 — **Head camera like the D455 + no blocked look.**
    - **16:10:** `table_env.IMAGE_SIZES`: head camera 320 × 200 (D455 1280 × 800 / 4), with fovy 65 → 91.1° wide (data sheet 90 × 65);
      before it was square = only 65° wide. Wrist cameras for now 224 × 224 (decision D3). openpi pads images to
      224 × 224 itself (`ResizeImages` = `resize_with_pad`, openpi `training/config.py`) → we store the real format. `convert.py`: image size per
      camera; the teleop page shows the head camera with padding like π₀.₅; `perception.Locator` 640 × 400. Comparison `docs/lernpfad_l3a_kamera.png`.
    - **Clearing the view:** the segmentation image showed ~5500 pixels of the robot's own right hand in the start pose. New phase `freimachen` (clear the view) before the first
      look: hand base 15 cm back, 10 cm outward (pelvis frame, `grasp.LOOK_OFFSET`), the shortest of 27 checked paths; over 30
      random starts 0 hand pixels, bottle ≥ 770 pixels, IK ≤ 1 mm, no contact. Test `test_freimachen_hand_aus_dem_bild` (with counter-check).
    - **Result** (`54 --randomize --trials 50 --seed 0`, the same start poses as 1.5b): first look max 13.3 cm → **0.3 cm**; reachable
      bottle 18/20 → **19/20 = 95 %**; failure `kopfkamera_findet_flasche` (head camera finds bottle) 1 → 0; out of reach unchanged at 30. Without variation 5/5.
      `58_teleop_check.py` passed with 16:10 (incl. LeRobot conversion). Open: 1 × hand not at the grasp pose, tilt ~40° (D2). — Status: DONE

- [2026-10-02] Learning path D3 (part) — **Only the head camera in the data format.** The user confirms: the real G1 has no wrist cameras.
    `episode.IMAGE_KEYS = ("head_image",)`; `TableEnv.observation()` now renders only this one (the sim hand cameras stay for watching:
    teleop page label "nur Anzeige" (display only), check images in 54). `openpi_g1/policy.py`: π₀.₅'s wrist inputs are black with image mask False,
    as openpi does for pi0/pi0.5 (`libero_policy.py`). Checked: openpi tests 10/10, sim tests 17/17 (openpi_env, teleop, table_env),
    `58_teleop_check.py` passed (recording + LeRobot conversion), `60_openpi_closed_loop.py` 8/8. Decisions D3 in `PLAN.md`.
    Open: 23 or 29 DoF (user unsure) — only blocks D5. — Status: DONE

- [2026-10-02] Learning path L3a part 2 / D2 — **Bottle tilt in the hand: 41° → ~2.7°.** Changes in `g1sim/grasp.py`: the hand is rolled
    −30° about the finger axis; ring + pinky stay open (the Revo2 model's thumb-rotation motor has only 0.5 Nm, closed ring/pinky pushed it
    back → ~40° tilt); `LIFT` 0.10 m + `LOAD_SAG` 0.04 m feed-forward (aim a bit higher to make up for the sag under load). New tool
    `scripts/63_grasp_sweep.py` (tries many grasp variants on the same start poses).
    Result `54_grasp_scripted.py --randomize --trials 50`: seed 0 → 25/50 passed, 25/25 reachable = 100 %, tilt median 2.71° / max 4.37°
    (`runs/d2_54_seed0.json`); seed 1 → 35/50, 35/35 reachable, 2.40° / 3.93° (`runs/d2_54_seed1.json`). Before: 19/20 reachable, ~41°.
    On the old set of reachable starts: 20/20 (seed 0) and 31/31 (seed 1). Teleop check 58: 16/16 (3/3 grasps).
    **Honest:** "100 %" is not a robustness promise — some edge starts are close to the 10 mm limits; run a 3rd seed before D5
    (`.planning/reviews/D2-measurement.md`, F4). Review: the measurement lens reproduced the numbers exactly but said NO-GO because of F1
    (23-DoF IK test); the rest of the review gate was stopped by the user → **gate OPEN**.
    Known test failures (2): `test_ik_erreicht_greifpose[23dof]` (rolled grasp is out of reach with a locked wrist: 40.7 mm vs. 10 mm
    limit — only matters for 23 DoF, which our robot does not have) and `test_greifwert_erst_daumen_dann_finger` (test still expects all
    6 fingers closed). Both tests need updating. — Status: DONE (code), review OPEN, uncommitted

- [2026-10-02] Learning path D4 — **Demo factory probe.** `scripts/64_demo_factory.py`: records successful scripted grasps (head camera
    image + state + 26-D action per control step) in the teleop format; only reachable starts; seeds from 100 up so the later π₀.₅ test
    (seeds 0/1) never sees training starts; output `runs/demos.nosync/`. Probe: 2/2 demos, 615 frames = 20.5 s sim time, ~93 MB and
    ~16 s compute per demo; scaled up: 100 demos ≈ 27 min in one process, ≈ 9.3 GB (`runs/d4_probe.log`).
    — Status: probe DONE, mass recording not started

- [2026-10-05] Real-robot track — **User priority: real G1 walking point-to-point first.** Three scripts written (uncommitted,
    **never run on the real robot**): `scripts/70_real_goto.py` (our sim go-to-point controller drives the factory walking controller
    via `LocoClient.SetVelocity`; pose from odometry, IMU or dead reckoning; max 3 m, 0.3 m/s, Ctrl+C stops, `--dry-run`),
    `71_real_probe.py` (listen only: which DDS topics exist, joint + IMU data, 23/29 DoF hint; DDS = the message system the robot
    uses), `72_real_check.py` (four basics one by one: read joints, move one arm joint a few degrees, move a hand finger, grab a head
    camera image). `runs/real_probe.json` was recorded on interface lo0 against the **sim**, not the robot.
    Also: decision that the repo becomes English; the translation waits until D2 is committed. — Status: WIP (written, not tested on hardware)

- [2026-10-06] Team + research — **Team goal proposal** `docs/TEAM_GOALS_PROPOSAL.md` (English) for the meeting on Wed 2026-10-07
    11:00 with Cristina + Saad: SAP starts a task → robot inspects fire extinguisher + exit (report only) → finds a non-red item → walks,
    grasps, carries, drops it into a box on a 2nd table → reports back to SAP. Success: full sequence ≥ 7/10 in sim, ≥ 3 filmed
    real-robot attempts + lessons learned, pitch + business case. Fact-checked twice (`.planning/reviews/team-goals-*.md`).
    Facts checked: big open models (~200–800 GB) → cloud only; AWS Bedrock ~$3–6 per 1,000 image+decide calls; EC2 Frankfurt
    g6.xlarge (L4 24 GB) $1.006/h, g6e.xlarge (L40S 48 GB) $2.327/h (safe for π₀.₅ LoRA > 22.5 GB); walking/balance always run on the
    robot (500 Hz motor commands, 50 Hz walking policy); fire extinguishers weigh ~7–11 kg vs. ~3 kg arm limit → inspect only, never carry.
    External resources (verdict: borrow ideas, adopt none): BotBrain (MIT, ROS 2, Jetson only; borrow Nav2 + RTAB-Map config for the
    Mid-360, e-stop sequence, waypoint missions; its YOLO module is AGPL) · luckyrobots g1-manipulation-challenge (no license → ideas
    only; supat-roong write-up: placing with contact check, carry pose, strict success test, failure list; ~3–6 days to borrow) ·
    Reddit "sorting cans" (G1 at a fixed table, ACT on ~3 h real demos; narrow camera view, forgiving boxes).
    — Status: DONE (proposal), team decision pending

- [2026-10-07] Robot specs — **Confirmed by the user:** G1 EDU **29 DoF**, D435i, Mid-360, Jetson Orin NX 16 GB, Revo2 hands, 2-DoF
    head with D455, backpack = Thunderobot PC (i7 + GeForce RTX). → 23/29 DoF question **resolved (29)**, D5 no longer blocked; the D2
    grasp needs wrist pitch/yaw, which the robot has. The sim backpack (`models/unitree_g1_payload/payload.xml`, Unitree AGX STL,
    1.9 kg) is only a stand-in with the wrong mass/shape → update when the real weight is known. Still unknown: RTX model + GPU
    memory, OS of the Thunderobot, backpack weight. — Status: DONE

- [2026-10-07] Goals + detector license — **User's Level 1–4 ladder** set as the build order (Levels 1–3 = goal, 4 = stretch; see NEXT
    STEP). Detector decision: Ultralytics YOLO is AGPL-3.0 for code, weights and self-trained models (copyleft: if you ship it, you must
    share your code) → needs approval or a paid license. Recommended: **RF-DETR** (Apache-2.0, sizes N/S/M/L only; other sizes are
    PML-1.0), colour via HSV inside the box; OWLv2 / Grounding DINO (Apache-2.0) as open-vocabulary fallback. Avoid YOLO-World (GPL-3.0),
    YOLOE/YOLOv10 (AGPL), DEIMv2 (non-commercial). — Status: recommendation (not yet confirmed by the user), YOLO approval question open

- [2026-10-07] Docker pitch — **Pitched, waiting for the user's go.** Stage 0 (1–2 days): lock environments (`pyproject.toml` +
    `uv.lock`), one `setup.sh`, 3 small fixes (hard-coded "lo0" in 6 places → setting; live-view/teleop bind host as a flag; `SIM_DIR`
    overridable). Stage 1 (2–3 days): Dockerfile (arm64 + amd64) + docker compose, CPU rendering, browser live view; assets downloaded
    at build time, never pushed in an image; acceptance: a teammate runs `docker compose up`, 50 grasp trials within ±5 % of native.
    Stage 2 (later): openpi's own Docker on the AWS GPU / Thunderobot. Limits on the Mac: no GPU, no mjpython window, no DDS multicast to
    the real robot → daily dev + real robot stay native. Use **Colima** (MIT, free) instead of Docker Desktop (paid plan needed for large companies and
    shared university-group projects, per the Docker Desktop terms updated 2026-08-26). — Status: proposal, waiting for go

- [2026-10-07] Baseline before Level 1 — User ran `scripts/run_tests.sh` after commit 27708e9: project env 49 passed, LeRobot env
    88 passed + the 2 known D2 failures, openpi env 10 passed. Then the 2 outdated tests were fixed: `test_ik_erreicht_greifpose`
    no longer tests the locked-wrist (23 DoF) case (our G1 has 29 DoF); `test_greifwert_erst_daumen_dann_finger` now expects
    `grasp.CLOSED` (ring + pinky open since D2). Both files: 12 passed. — Status: DONE

- [2026-10-07] Docs translated to English — All Markdown docs are now English. `docs/LERNPFAD.md` → `docs/LEARNING_PATH.md`,
    `.planning/reviews/D2-messung.md` → `.planning/reviews/D2-measurement.md`. User decision: the code stays untouched, so German
    identifiers, CLI flags, test names, JSON keys and output file names stay as they are, and the docs name them exactly as in the
    code. Baseline tests after commit ec59653 (code unchanged since): project env 49 passed, LeRobot env 89 passed, openpi env 10
    passed. — Status: DONE

- [2026-10-07] Lab View v1 — `g1sim/labview.py` (reporter inside the run, no extra renderer), `scripts/65_labview.py` (stdlib server on
    127.0.0.1:8030; tabs Live / History / Training placeholder), hook in `54_grasp_scripted.py` (`--watch SPEED`, `--no-labview`). Unit
    tests 10/10 in both envs; one watched live trial + History with 20 old runs checked by hand. Independent review skipped to save credits.
    — Status: DONE
- [2026-10-07] Volleyball scene — `table_scene.SCENES["volleyball"]`: red sim ball r 0.105 m / 0.27 kg (shell inertia, condim 6), tube
    stand (inner Ø 0.10, 0.08 high), basket 0.28 × 0.28 × 0.10 at (0.344, −0.241), table edge x = 0.20; `TableEnv(scene=...)`; bottle scene
    byte-identical. `66_volleyball_scene_check.py` 5/5; ball half hidden by the left hand in READY_POSE (accepted). — Status: DONE
- [2026-10-07] Volleyball teacher (sim only, records demos for the VLA) — `g1sim/ball_grasp.py` `TwoHandBallGrasp`: face ball (waist
    +35°) → approach → descend → touch → squeeze (palms cradle 20° below the equator, all five fingers curl gently, closure 0.12) → lift →
    waist turn to −35° → release → back off → settle; check "inside the basket, still 3 s, no robot contact". `67 --trials 10` 10/10,
    `--trials 50 --jitter 0.02 --seed 0` 50/50; tests 32 + 35 passed. Findings: stronger finger closure pushes the ball out; arm posture
    during the grasp is natural (elbows low, near the body) — the odd look is READY_POSE. Video `docs/videos/volleyball_demo_video.mp4`.
    — Status: DONE
- [2026-10-07] SmolVLA measured on the Mac (separate env `~/dev/g1-ext/smolvla-test`, lerobot 0.6.1, `lerobot/smolvla_base`): MPS works
    without fallback; 27-D state/action fits (padded to 32); inference ~200 ms per 50-action chunk (CPU 2.8 s); training 0.84 s/step at
    batch 8, 1.6 s at 16, 3.3 s at 32 (synthetic batch) → 20k steps ≈ 5 h / 9 h / 18 h. — Status: DONE (measurement)
- [2026-10-07] Decisions — Level 1 object = volleyball (two hands, waist turn); a VLA must run the task (not a script); local first
    (no AWS yet); model rule: π₀.₅ if a ≥ 24 GB GPU (AWS) is available by ~10-13, else SmolVLA; BotBrain approved by the team for navigation,
    used locally without dashboard/Supabase/ROSA/YOLO (code check: the ROS 2 part makes no internet calls; the dashboard requires a
    Supabase login). π₀.₅ cost estimate: ~15 H100-hours ≈ $30–60 per 30k-step LoRA run (from openpi's DROID numbers). — Status: DONE
- [2026-10-07] Team share — a `Finnegan/` folder (README for the team: today's results, architecture, VLA choice, plan to Potsdam, open
    questions; plus the volleyball video and 3 images) prepared for the team repo `acristinaa/grabby-mcgrabface` (Next.js app, push access
    confirmed, no `Finnegan/` folder existed). The user commits and pushes it himself, without a Claude co-author line. — Status: handed over
- [2026-10-08] Model fit for the G1 — π₀.₅ judged the better fit for the G1 (two arms, 27 control values, dexterous hands); SmolVLA stays the
    Mac-trainable safety net; GR00T N1.x noted as an unchecked third option. — Status: DONE (assessment)
- [2026-10-08] Level 1 walk-up plan — User's new scenario: robot starts 0.5-1.0 m back, walks up, finds ball + basket with the head camera, puts
    the ball in the basket; ball, basket and start pose random. User rulings: a script walks (BotBrain stand-in, GR00T), the VLA only grasps
    (arms + hands + waist yaw); camera check for the walk-up; the **real ball is grey** → detection by shape; distractors (teammate, own files,
    Stage D); demos to `/Volumes/T7/Robot Project SAP/DATA Collection/g1-demos/`. Plan `.planning/level1-walkup/PLAN.md` (9 stages, revision 2
    + section I), two reviews (both NO-GO → revised; the revision itself not re-reviewed). Timeline honest: training night 1 Fri 10-10 =
    pipeline proof, night 2 Sat, first real Level-1 VLA number Sun 10-12. — Status: DONE (plan)
- [2026-10-08] Level 1 walk-up Stage 1 — 27-D action space (index 26 = waist yaw; `set_waist_yaw` removed, the teacher drives `action[26]`),
    new free-floating scene `volleyball_walk` (pelvis free joint, mocap + weld `pelvis_hold`, `GROOT_STANCE`, `stance_clearance`,
    `snapshot_handover`/`restore_handover`, noise re-seeded per reset), grey ball (0.55) in both volleyball scenes, 66 check 5 colour-free
    (segmentation). Tests: project 89 passed, g1-ext 147 passed, openpi 10 passed (the 2 old D2 failures no longer appear); fixed-scene 67 10/10
    and 50/50 (±2 cm); bottle demo 27-D validates + converts. Review GO (2 rounds, `.planning/reviews/level1-walkup-stage1-code*.md`).
    **Found:** the nominal stop (R_STOP 0.45) has only 3.1 cm knee clearance to the table (plan assumed more) → Stage 3 re-derives R_STOP first
    (PLAN section I). Old 26-D demo files no longer validate. — Status: DONE
- [2026-10-08] Level 1 walk-up Stage 2 — `g1sim/perception.py`: `points_from_depth`, clustering, `find_ball` by sphere fit with the known radius
    (no colour; other-size spheres, cylinders, boxes rejected), `find_basket` by rim square; Locator bit-identical. 44 unit tests pass in both
    envs; 54 still 5/5 (2.8 mm). Real render of `volleyball_walk` from 0.5-1.0 m back: ball 25/28 views, basket 21/28, misses = object out of
    view or basket cut off (correct None). Review GO (2 rounds). **Open:** with a tall object in front of the basket `find_basket` can return a
    wrong centre (6-25 cm) instead of None → fix before distractors (Stage D). — Status: DONE
- [2026-10-08] Natural arm motion (user: "more like a human would grab the ball") — new start/end pose `action_space.BALL_READY_POSE` for the
    volleyball scenes (elbows low, palms facing each other, relaxed fingers; found by IK), teacher phases raise → turn → reach from above →
    ... → retract → turn back → lower (ends where it started). First try (turning with the hands low) knocked the ball off its stand in 13/50
    runs (fingertips 1 cm from the ball while turning) → the waist only turns with the hands raised (≥ 4.6 cm clearance at ±3 cm offset).
    Fixed-scene 67: 10/10, ±2 cm 50/50; volleyball tests 18 passed. Images `docs/volleyball_natural_before_after.png`,
    `docs/volleyball_natural_phases.png`. — Status: DONE
- [2026-10-08] Natural arm motion, round 2 (user screenshots: hands hung over the ball with bent wrists; elbows behind the torso in the start
    pose) — start pose now from a joint grid search with forward kinematics: elbows beside the body, palms facing each other, hands at chest
    height (≥ 4.9 cm from ball/table at the walk stop, ≥ 4.5 cm while turning ±40° with the ball ±3 cm off); the reach is one straight,
    smooth move to beside the ball (no "8 cm above, then down"); every move has a cosine speed profile; raise/lower phases removed.
    67: 10/10, ±2 cm 50/50; all tests green. Filmstrip `docs/volleyball_straight_reach.png`; replay video in the Lab View History. — Status: DONE
- [2026-10-08] Lab View replay button — a watched 67 run (`--watch`) saves a video of trial 0 to `runs/labview/videos/<run_id>.mp4`; the run
    record gets a `video` field; `65_labview.py` serves it (`/video/<run_id>.mp4`, HTTP byte ranges for Safari, only from `runs/labview/videos`
    and `docs/videos`) and shows "▶ Video" in History and on finished live cards: replay, loop, speed 0.25-2×. Checked: lab view unit tests
    10 passed, real run → video 1.0 MB, server 200/206/404 as expected, page JS syntax OK. Not yet clicked through in a browser by me.
- [2026-10-08] Volleyball distractors (Saad, branch `saad/table-distractors`) — 5 distractors in the volleyball scene
    (`table_scene.VOLLEYBALL_DISTRACTORS`: cardboard box, drink can, red cylinder (same red as the ball, on purpose), purple cylinder,
    yellow block). New randomization group `distractors` (`count` [1, 3]): `randomization.sample_distractors` puts 1–3 on the table, on
    the table top, apart from each other, clear of ball, basket and the carry arc (≥ 0.62 m from the waist axis,
    `DISTRACTOR_MIN_REACH`); the others are parked on the floor at x = −10 m. In the volleyball scene `reset(randomize=True)` now
    works and varies **only** the distractors (the other groups are still bottle-only). Touching a distractor is a forbidden contact
    in `ball_grasp`. `67 --distractors` draws them per trial.
  - **Result** `67 --trials 30 --jitter 0.02 --distractors --seed 0`: **30/30**, no forbidden contacts, no distractor moved (0.0 mm).
    Nominal `67 --trials 10` still 10/10. Tests: new `tests/test_integration_distractors.py` (placements valid over 300 draws, 1/2/3
    all occur, seed, switch off, parked at nominal, stand still, scene parts not robot, replay bit-exact); the volleyball "randomize is
    refused" test is replaced by "randomize varies only the distractors".
  - **Honest:** distractors only on the back of the table (the safe area for today's scripted teacher), so the VLA always sees them
    behind the ball; the red cylinder is sometimes hidden behind the ball. Next: let the demo factory call `reset(randomize=True)`,
    and later allow the middle strip for short objects once the teacher avoids them. — Status: DONE
- [2026-10-18 14:50] **Handover for a new session.** 10-16 evening/10-17: real-robot plan revisions 1-5 with 5 review rounds
    (`.planning/reviews/real-robot-plan.md`; final rev 5 GO for a no-motion Sunday). New facts: robot day moved to Monday 10-19, Finnegan away,
    the user wants a VLA attempt → plan rev 6 + build via `~/dev/g1-ext/release/robot-day-build.workflow.js`. Build runs on 10-17 failed (user
    stop; then no internet). v2 recording aborted (T7 dropped). Model packed for a GitHub Release (`~/dev/g1-ext/release/`), upload pending.
    Learning path 10.64-10.65 added. — Status: WIP
- [2026-10-16 15:58] Level 1 Step 8 (= plan Stage 9), no-walk part: **closed-loop eval of run1 40k + 45k + teacher.** `82 --no-walkup --seed 0
    --trials 50 --train-variations 10 --demos <T7>/level1_walkup_v1` with `81 --device mps` per checkpoint; teacher `82 --teacher` on the same starts
    in parallel (logs `runs/l1w_s8_eval_{teacher,045000,040000}.log`, reports `runs/vla_eval_{teacher_seed0,run1_040k_seed0,run1_045k_seed0}.json`,
    32 min in total). **Teacher 50/50 + 10/10. 45k: 35/50 = 70 % (Wilson 95 % 56-81 %), training 10/10, verdict "generalization gap". 40k: 36/50
    = 72 % (58-83 %), training 9/10, verdict "consistent".** Failures 45k: forbidden contact 12 (left pinky on the stand 6, others on basket/table),
    ball not lifted 3; 40k: contact 6, not lifted 7, placement not held 3 s 1. All contact failures end before the lift (furthest event = handover).
    9 starts fail with both checkpoints, 6 only with 45k, 5 only with 40k. Time to success ~24 s mean. Server 209-212 ms mean (max 2.3 s once, 45k).
    Reading: training starts ≈ teacher, new starts 70 % → the model learned the demos but generalizes only partly → v2 needs more varied demos
    (or narrower start ranges) and more clearance between the little fingers and the stand during the reach, not more steps. — Status: DONE
- [2026-10-15 22:31] **run1 finished (22:25:55, 45k steps, loss 0.011) + Step 8 part 1.** MPS self-test of 45k PASS (`runs/l1w_s8_mps_selftest.log`;
    first chunk 1.3 s incl. warm-up). Open-loop over all 9 checkpoints on MPS (`runs/l1w_s8_open_loop_all.log`, `runs/l1w_open_loop_run1_all_mps.json`):
    **207 ms per chunk** (latency check < 1 s: PASS); skill arms 0.414 / 0.560 / 0.638 / 0.677 / 0.715 / 0.751 / 0.785 / 0.801 / 0.801, waist 0.441 →
    0.766 (30k) → 0.770 / 0.775, hands 0.79 → 0.97 (35k) → 0.93 (45k; the diagnostic gate flags hands at 45k: MAE 0.0008 → 0.0012, noise level).
    Plateau from ~35k → candidates 40k and 45k for the closed-loop eval. All 9 checkpoints backed up on the T7 (45k at 22:31).
    Stopped for the day at the user's request; the full closed-loop eval (50 held-out + 10 training variations + teacher) is tomorrow's first task.
- [2026-10-15 17:04] **Open-loop check over run1 checkpoints 5k-20k** (`83 --run run1 --allow-during-training`, 20 training frames, CPU;
    `runs/l1w_s8_open_loop_20k.log`, `runs/l1w_open_loop_run1_upto20k.json`). Skill vs the hold-still baseline (0 = no better, 1 = perfect):
    arms 0.41 → 0.56 → 0.63 → 0.68, hands 0.80 → 0.83 → 0.87 → 0.94, waist 0.44 → 0.49 → 0.44 → 0.66; arm MAE (steps 1-25) 0.019 → 0.010 rad.
    Still improving at every checkpoint, no plateau → run1 continues to 45k. Diagnostic gate PASS (all groups). Training frames only.
- [2026-10-15 ~16:55] **First closed-loop preview of run1 (20k checkpoint, CPU, while run1 trains; user OK):** `82 --no-walkup --seed 0 --trials 2`
    (`runs/l1w_s8_preview20k.log`, `runs/vla_eval_run1_20k_cpu_preview.json`). **Trial 1: PASS - the VLA alone put the ball in the basket,
    latch 21.1 s** (26 chunks), in a held-out start. Trial 0: right pinky touched the ball stand at 8.1 s (forbidden contact, during the reach).
    1/2, Wilson 9-91 % - a first sign, not a number. 2,551 hand values clipped to [0, 1] (slightly below 0 = "more than open"). CPU latency
    ~5.3 s per chunk (MPS expected ~0.2 s). The Mac slept 16:03-16:47 with the lid closed; run1 continued after the wake (expected end ~23:35).
- [2026-10-15] Level 1 Step 4 (= plan Stage 5): **policy server + eval + open-loop check, gate PASSED.** Plan
    `.planning/level1-walkup/STEP4-PLAN.md` (rev 2 after review NO-GO → re-check GO), built by 3 parallel implementers, code review GO
    (`.planning/reviews/level1-step4-{plan,code}.md`). New: `g1sim/policy_wire.py` (stdlib TCP + JSON header + raw numpy, no pickle; the
    SmolVLA env has no websockets/msgpack and openpi-client pins numpy<2), `scripts/81_smolvla_server.py` (checkpoint with its processors or
    `--replay`; refuses MPS and run1 checkpoints while lerobot-train runs), `g1sim/vla_eval.py` + `scripts/82_vla_eval.py` (`--no-walkup`
    held-out seeds 0/1 = 69's handover, `--from-handover`, `--teacher`, events lifted/carried/placing/latched, Wilson intervals, stall +
    no-progress, Lab View task `vla_eval`), `scripts/83_open_loop_check.py` (chunk MAE per group vs a hold-still baseline).
  - Gate: replay 5/5 on smoke_v1 + 5/5 on training demos, every event tick equal to an in-process reference replay, state error 0.0,
    observation checks (state exact, tick-0 image 0 grey levels, later MAD ≤ 3.39) · wire check on the CPU smoke checkpoint max diff 0.0 ·
    3 mutations (stale observation, chunk off by one, hold 89 ticks) all caught · tests 95 + 90 passed (`runs/l1w_s4_*.log`).
  - Review fixes applied: oversized wire shapes refused before multiplying, a model RuntimeError gets an error reply (server keeps
    serving), **success needs the 6 cm lift first** (a ball pushed in is no success), labview test text updated for the Training tab.
  - Open: MPS latency check and the real eval (Step 8) wait for run1; R4 (open-loop check on run1 checkpoints on CPU while it trains):
    user said "later" (2026-10-15 ~13:55) - next session starts with it or with the finished run1. — Status: DONE
- [2026-10-15] Level 1 Step 7 (= plan Stage 8): **SmolVLA training on the 300 demos.** `scripts/train_smolvla.sh` (smoke | run1 | resume <name>;
    MPS, batch 8, `lerobot/smolvla_base` from `~/dev/g1-ext/smolvla-test/hf_cache/hub`, runs in `~/dev/g1-ext/smolvla-runs/<name>`).
  - **Smoke (200 steps): passed** - dataset loads (300 episodes, 217,800 frames), loss 4.59 → 1.02, 0.87 s/step, checkpoint 1.2 GB with the
    pre/post-processors (`runs/l1w_s7_smoke.log`).
  - **run1 started 10:15:** one schedule for **45,000 steps** (warm-up 1k, decay over 45k) instead of "20k + resume": lerobot scales the decay
    to `--steps`, so a 20k run would end at the minimum learning rate and a second night would barely learn. Checkpoint every 5k
    (~74 min); stop any time, `scripts/train_smolvla.sh resume run1`. ~11 h at 0.88 s/step. Log `runs/l1w_s7_run1.log`.
  - **Restarted 10:40:** closing the Claude Code window at 10:27 killed run1 at step 833 (child of the session; no checkpoint yet,
    `runs/l1w_s7_run1_killed.log`). Now started with `nohup caffeinate -i ... & disown` (parent = launchd) - survives closing the
    window. Same seed, same losses. Expected end ~22:15.
  - Checkpoints 5k (11:54) and 10k (13:07) saved; **backup to the T7** with `scripts/backup_checkpoints.sh run1 --watch` (detached):
    one sha256-verified .tar per checkpoint in `DATA Collection/smolvla-runs/run1/` (1.2 GB each, incl. training_state, so a run can resume from it).
  - **Lab View Training tab** (`scripts/65_labview.py`, `/api/training`): every lerobot-train log in `runs/` - exact step from the
    progress bar, loss curve, learning rate, passes through the data, s/step, time left, checkpoints. Live at http://127.0.0.1:8030/#training.
  - Checked for the user: Saad's walk to the table is on `main` (merged 10-09) and still runs (`68 --trials 2`: 2/2, ≤ 2.1 cm, ≤ 0.2°,
    `runs/l1w_s7_walk_check.log`). **Not on GitHub:** Step 2 (walk + grasp in one world) and Step 4 (policy server + eval) - not built yet.
    — Status: Step 7 smoke DONE, run1 RUNNING
- [2026-10-14] Level 1 Step 5 (= plan Stage 6): **volleyball demo recorder** `scripts/69_volleyball_demo_factory.py` + **shoulder lift**.
  - Recorder (no-walk mode, `volleyball_walk`): episode from the handover to face_front + 30 ticks = **726 frames (24.2 s)**, head image
    200x320 + 27-D state + 27-D action, prompt "put the volleyball in the basket", 30 fps; handover snapshot in the npz
    (`episode.handover_arrays` / `handover_snapshot`, JSON keeps the 128-bit noise RNG state); sidecar per demo; `factory_s<seed>.json`
    after every trial; lock per seed and folder; verified writes (`episode.write_verified`: temp name, fsync, size check, rename); invalid
    demos are not saved; Lab View card per process (recorded head frame | outside view, one image every 2 s). ~117 MB per demo.
  - Checks: restore of the saved snapshot in a fresh env + replay of the recorded actions → **bit-exact** (0 grey levels, 0.0 state) for
    every demo checked · new unit test `tests/test_unit_episode_handover.py` · tests 162 + 233 + 10 passed (`runs/l1w_s5_tests_v2.log`).
  - Found and fixed: (1) a 0-byte episode on the T7 - most likely the reviewer's run writing the same seed into the same folder at the same
    time (not the exFAT driver) → lock file + verified write; (2) the Lab View outside render changed the next head image by up to
    7 grey levels (state left in the GL context) → one throw-away head render, demos bit-exact with and without the Lab View;
    (3) the live image at 10 per second made 4 parallel recorders 3.5x slower (93 s instead of 26 s per demo) → one image every 2 s;
    (4) one of 4 recorders started at the same instant hung in the macOS Metal shader cache (`flock` in `MTLCompilerFSCache`, first
    render) → killed, restarted alone. Start parallel recorders a few seconds apart.
  - Review gate (`.planning/reviews/level1-step5-recorder-{contract,learning}.md`): contract **GO**, learning **OPEN → GO** (5/5 on the
    T7, reviewer's `guardcheck` folder removed). Fixes from it applied; M1 (eval needs a held-out no-walk mode) → PLAN section L.
  - **Shoulder lift** (user: "straighter pick-up, arms not so cramped"): a posture probe showed the 10-07 and the current teacher give the
    same arm posture in the same scene; the cramped look came from the walk-world stance (4.3 cm lower, 3.8 cm further back) and a lift that
    curls the elbows. `ball_grasp.py`: lift targets turned about the shoulder line (rigid pair, ball straight up), IK posture target = the
    squeeze arm during the lift. After the lift (walk world): forearm yaw -11 → -1.5°, forearm pitch 39 → 31°, upper arm 34 → 40° down.
    Tried and rejected: a stiff IK elbow (elbows flared 11 cm out). Gates: walk sweep **200/200** (seeds 10-13; was 199/200), fixed nominal
    10/10, ±2 cm 50/50, randomized 50/50 + 50/50, distractors 30/30. User approved it from `docs/videos/l1w_s5_lift_before_after.mp4`.
  - **Smoke (Step 5 done criterion): 5/5** demos on the T7 (`g1-demos/smoke_v1`, new motion), all valid, all replay bit-exactly.
  - **Step 6 DONE (17:56-18:58):** 4 processes × 75, seeds 101-104 → **300 demos** in `g1-demos/level1_walkup_v1` on the T7 (301 trials,
    1 rejected: forbidden contact, s104 t72; 726 frames each; 35.1 GB; 0 problems, 0 duplicate variations, `runs/l1w_s6_dataset_check.log`;
    8 sampled demos replay bit-exactly, `runs/l1w_s6_replay_sample.log`). ~33 s per demo per process with 4 processes.
  - **LeRobot v3.0 dataset:** `scripts/79_convert_smolvla.py` → `~/dev/g1-ext/datasets/g1_volleyball_walkup_v1` (**internal disk**, not the T7:
    exFAT `._` twins crash lerobot, reproduced) - 300 episodes, 217,800 frames, 30 GB, 7 min. `scripts/80_validate_dataset.py`: **0 problems**
    (every state/action value of every episode, stats, 300 first frames + 100 random images bit-equal; `runs/l1w_s6_validate.log`; counter-test
    with two swapped demos → 60 problems). Backup: one tar file on the T7 (`DATA Collection/lerobot/g1_volleyball_walkup_v1.tar`, sha256 checked).
    `lerobot[dataset]==0.6.1` installed into the SmolVLA env (U4). Loading builds a ~30 GB Arrow cache in `~/.cache/huggingface/datasets`.
  - Reviews: shoulder lift **GO** (`.planning/reviews/level1-step5-shoulder-lift.md`, minor F2/F3 applied); converter **NO-GO → fixed**
    (`.planning/reviews/level1-step6-converter.md`: F1 location, F3 full-value validation, F4/F5). A 2-step `lerobot-train` with
    `lerobot/smolvla_base` on the converted smoke data ran (reviewer, CPU).
    — Status: Step 5 DONE; Step 6 DONE
- [2026-10-10] Level 1 Step 1 (= Stage 3 remainder): **teacher in the walk world `volleyball_walk`** — first probe: the teacher works
    there unchanged (7/7 stances, 1.1 s per trial without images, `runs/l1w_s3_probe_walk.log`). Built:
    `navigation.table_stop_pose` (pelvis R from ball and basket, facing their middle, ball left) · `table_scene.R_STOP` 0.45 → **0.458**
    (knee clearance 3.1 → 4.1 cm; clearance grows ~1:1 with the distance, `runs/l1w_s1_fix_clearance.log`), `WALK_STANCE` computed from
    the rule (x -0.0455) · `randomization.sample_walk` / `walk_layout` / `walk_layout_ok` + config group `"walk"` (ball and basket ±8 cm,
    separation 0.40-0.56 m, stand ≥ 8 cm behind the edge, basket on the table, waist ≤ 45°, ball and basket ≥ 50° apart from the stance,
    arrival error back 0-6 cm / sideways ±4 cm / ±6°) · `TableEnv.reset(randomize=True)` for the free base: redraws when the FK clearance
    of the stop pose < 4 cm or of the stance < 2 cm · 67 `--scene volleyball_walk`, `--map`, a `walk` block per result (stop, stance,
    arrival, clearance, distance and bearing of stand and basket), p95 of the carry error · teacher phase **`centre`** (1 s: both hands move
    the ball horizontally over the basket centre before the release; Lab View label; `align` was taken by the walk-to-table phases).
  - **Reach map** (`runs/l1w_s3_map.json`, 65 grid points): 52/65. Stances up to 6 cm back / ±4 cm / ±6° all pass; 8 cm back with
    4 cm sideways puts the ball 0.547 m away → lift sags (3 fails, reach limit ~0.53-0.55 m); 1 start overlapped the stand (clearance
    -0.6 cm, never drawn by the sampler); all 9 layout failures are the **knee on the table** (clearance < 0), never the grasp - moving
    the objects deeper onto the table or further apart pulls the robot towards the edge. The sampler's clearance check excludes them.
  - **Sweep 1** (200, seed 0): 196/200, all 4 = right wrist on the basket during the reach, all with ball and basket < 50° apart as seen
    from the stance → layout rule `min_gap_deg` 50 (excludes the region instead of silently dropping its failures). Ball over the basket
    after the turn p95 **3.8 cm** (correlation 0.90 with the sideways arrival error: the turn keeps the ball's distance from the waist
    axis, the basket's differs) → `centre` phase.
  - **Final sweep** (200, seed 0, `runs/l1w_s3_sweep.json`): **199/200 (99.5 %)**, 1 = wrist on the basket at 50.4° (rule edge, left as
    is), ball over the basket p95 **0.3 cm** after centring, end in the basket p95 4.6 cm (the 21 cm ball rolls in the 28 cm basket),
    episode 29.2 s sim. Regression, fixed scene: nominal 10/10, ±2 cm 50/50, randomized 50/50 (centring there: 12.0 → 1.1 cm p95).
    Tests: 160 + 231 + 10 passed (`runs/l1w_s3_tests.log`), new unit tests for the stop pose, integration test for the walk sampler.
    — Status: DONE (teacher code changed: part of the one quick review before the recorder, Step 5)
- [2026-10-09] Merge of `saad/table-distractors` into `integrate/saad-distractors` — 7 conflicting files resolved by hand (teacher:
    our motion + Saad's stance geometry and distractor contacts; scene: walk scene + distractors; env: noise re-seed + randomization;
    67: both option sets; docs: both logs, Saad's learning-path sections renumbered 10.31-10.40). Found by the gates: random starts
    46/50 (sagging right wrist on the basket rim at the end of the straight reach) → `REACH_ABOVE` 0.04. Found by reading: 68 mixed
    26-D and 27-D arm vectors → 27-D, arms raised to `BALL_READY_POSE`. Results: see NEXT STEP update 2026-10-09. — Status: DONE
    (quick independent review: GO, `.planning/reviews/merge-saad-distractors.md`)
- [2026-10-08] Volleyball scene: full randomization switched on (Saad, branch `saad/table-distractors`) — `reset(randomize=True)`
    in the volleyball scene now varies **every** group, not only the distractors: ball + stand offset, robot stand pose (±10 cm,
    ±10°), physics (mass, friction, gains, latency), image (light, table colour, ball colour jitter, camera pose, noise), 1–3
    distractors. `randomization.sample` is scene-generic (objects, target and table from `table_scene.SCENES`; bottle draws
    bit-identical, checked by a hash over 204 draws). New config block `scenes` narrows ranges per scene (`for_scene`); volleyball:
    ball offset ±0.10 m (the bottle's ±0.15 m gave 17/20 – far-left starts out of reach). Distractor keep-out follows the drawn
    ball and stand offsets.
  - **Teacher** (`ball_grasp`): reads the pelvis pose and turns the waist by how far ball (stand) and basket moved around its waist
    axis; hand orientations use torso yaw = pelvis yaw + waist. Nominal start bit-identical to before (same JSON results).
  - **Reach rule:** each group alone 20/20, all together only 85/100 – all failures had the ball 0.53–0.62 m from the pelvis
    (stand back + ball out add up). `SCENES["volleyball"]["reach_m"] = (0.33, 0.52)`: `TableEnv._start_is_clear` redraws such
    starts.
  - **Result** `67 --trials 50 --randomize --seed 0` and `--seed 1`: **50/50 and 50/50**, no forbidden contacts, no distractor moved.
    Nominal and `--distractors` runs identical to before. New `67 --randomize [--vary GROUPS]`. Tests: volleyball scene (every group
    varies, ranges from `scenes`, ball in reach, bad overrides refused) + a teacher trial with the stand shifted and turned; 28/28 in
    the randomization/distractor/volleyball test files (run with stub `lerobot` modules, this Mac's g1-ext venv has no LeRobot).
  - **Honest:** the ball can only come 2 cm closer (it must stay on the table, front edge x = 0.20), so the near side is barely
    covered; the basket does not move yet; distractor colours are fixed. `tests/test_integration_table_scene.py` fails on this Mac
    without these changes too (`assets/pelvis.STL` missing – no project `.venv`). — Status: DONE
- [2026-10-08] Walk up to the table from random start situations (Saad, branch `saad/table-distractors`; Finnegan no longer works on
    this, PLAN.md (j)) — new `g1sim/approach.py` (scenarios, A* to a pre-dock point 0.40 m out, docking controller, table-frame error),
    `g1sim/approach_world.py` (walking model + mocap table with top and 4 legs + 3 mocap obstacles, contact report),
    `scripts/68_walk_to_table.py`. Six scenarios, relative to where the robot stands: `facing`, `behind_table` (walk all the way
    round), `turned_away` (turn round first), `beside`, `obstacle_course` (1–3 obstacles), `anywhere`. Trials chain (no reset).
    Goal = the volleyball grasp pose (pelvis 0.525 m from the table centre), acceptance = what the Level 1 teacher handles (±10 cm, ±10°).
  - **Environment:** LeRobot stack set up again on this Mac in `~/dev/g1-ext` (CycloneDDS 0.10.x built, `unitree_sdk2_python`, LeRobot
    pinned to `89236ea0` = 0.6.2 as of 2026-09-14, torch 2.11); numpy went 2.5 → 2.2.6. Baseline `61 --goals 2`: 2/2 (4.6 cm, 7.7–8.5°).
  - **Found on the way:** (1) at LeRobot's zero arm pose the pinky tips hit the table top while docking – the walking pelvis is at
    ~0.74 m; arms now held in `READY_POSE` while walking (13 cm clearance). (2) GR00T dead band: commanded vs. real forward speed
    0.05 → 0.009, 0.10 → 0.061, 0.20 → 0.154 m/s, ~2 cm settle-back after a stop; a P controller hovered 4–6 cm short for 30 s →
    `approach.Docker` (dead band compensation, aim 2 cm deeper, 0.6 s pose filter, re-dock if > 4 cm / 4°, max 3 attempts).
    (3) With live view + lab view the loop ran at ~8 Hz; the tick-counted filter grew to 1.6 s and docking oscillated ±20 cm
    (2/12) → Docker on sim time, lab view reuses the live view's JPEG (`Reporter(frame_getter=...)`): ~14 Hz. (4) Bus error in
    `mjr_render`: `LiveView._add_marker` called `mjv_connector` on an uninitialized scene slot – fixed (`mjv_initGeom` first);
    L1/L2 had been lucky. (5) Phase key "settle" collided with the ball teacher's label → `stand_still`.
  - **Result** `68 --trials 24 --seed 0` (live view + lab view open): **24/24**, every scenario 4/4, 0 contacts, 0 falls; arrival
    error mean 1.3 / 1.2 cm, max 2.9 / 2.7 cm, heading mean 1.3°, max 2.7°; 20 trials docked at the first attempt, 4 at the second;
    paths 1.2–4.7 m, 8 obstacles, start headings from −168° to +179° against the table. Second run `--seed 1`: **24/24**, mean 0.9 / 0.9 cm,
    max 3.3 / 3.4 cm, max 2.7°, 22 first-attempt docks; paths 1.2–4.8 m, 7 obstacles. Together **48/48**.
    Top view of all trials in the table frame: `docs/level4_walk_to_table.png`.
  - **Tests:** `tests/test_unit_approach.py` (dock geometry = volleyball scene, table-frame signs, every scenario valid, behind_table
    really on the far side + path round the table, turned_away really behind, obstacles avoided, seed, Docker with GR00T's measured dead
    band, re-dock limit), `tests/test_integration_approach_world.py` (table shape, place/park, contacts, marker regression),
    `tests/test_unit_labview.py` (+ unique phase keys, frame_getter). All unit tests 76 passed (3 skipped: openpi not installed).
  - **Honest:** position comes from the sim (no localization, L7); one table size, flat open floor, furniture appears by teleport between
    trials; walking only, **no grasp after the walk yet** (L3 part 2: the walking pelvis is ~5 cm lower than the fixed 0.793 m the
    teacher was tuned at); the real robot will use BotBrain/Nav2 + this kind of align step, not our A*. — Status: DONE (part 1 of L3)
- [2026-10-09] Walk to the table, step 1 of "walk calmer" (user: the walk looked funky and did not follow the path) — measured
    first: the robot strayed 9–10 cm from its path (max 28 cm; 68 seeds 1 + 3), the A* grid path changes direction at ~20 % of its
    steps. New: `occupancy.shortcut/smooth/smooth_path` (pull the path taut, round the corners, never closer to obstacles than A*,
    ≥ ~0.58 m clearance), `navigation.speed_profile` (0.35 m/s on straights, slower in curves, gentle start/stop, ≥ 0.12 m/s above
    GR00T's dead band) and `navigation.SmoothPathFollower` (pure pursuit: walk and turn together, turn on the spot only if the path
    starts > 60° off, and then step back to where the turn started – GR00T drifted 35 cm sideways during a 180° turn).
    `68 --follower smooth|grid` (smooth is the default) + per-trial walking metrics (path deviation, command calmness, speed).
  - **Result, seed 3, 12 trials each, live view on:** path deviation mean 9.5 → **4.4 cm**, p90 17.1 → **8.7 cm**, max 24.0 → **17.2 cm**,
    turned_away max 21–22 → 11 cm; 12/12 both. Slower on purpose: 0.27 → 0.21 m/s, walk 11.0 → 14.2 s per trial.
  - **Not improved:** command calmness (mean change of the turn rate 0.23 rad/s² both) – the pelvis heading sways with every step and
    the follower reacts to it. Tests: `tests/test_unit_smooth_path.py` (12). — Status: DONE (step 1); next: arms down while walking
- [2026-10-09] Walk to the table: **checkpoint before the table** (user's spec) — arms down while walking, a checkpoint 0.75 m
    (~3–4 steps) before the grasp pose where the robot aligns on the line into the table (`approach.Docker` with `ALIGN`, side steps
    allowed there), stands, raises the arms to `READY_POSE` in 2 s, then walks **straight in** (`approach.StraightIn`: forward steps
    only, vy = 0, heading only steers back onto the line and straightens over the last 0.3 m; redo only along the axis). The path
    ends with a 0.7 m lead-in on the line (`approach.lead_in_pose`, join rounded by `occupancy.smooth_path(tail=...)`; a sharp join of
    up to 136° made the robot cut it); obstacles keep the corridor lead-in → table free (`CORRIDOR_CLEAR` 0.75 m). Arms-down pose
    `ARM_DOWN` (pitch 0.2, roll ±0.3, elbow 1.2 rad: hands by the hips at ~0.47 m, 8 cm from the legs). New phases in the lab view:
    walk, align, raise_arms, walk_in, stand_still. Also fixed: `occupancy.resample` doubled the end point.
  - **Result, seed 3, 12 trials, live:** 12/12, 0 contacts. Walk-in: body at most **4°** off the table axis (before: up to **87°**
    with the side-stepping docker), commanded vy **0** (before 0.12 m/s), sideways share 0.17 (before 0.29, on the 0.6 s-averaged
    track). Checkpoint reached within 3.7 cm / 2.0° (11/12 first try). Arrival ≤ 1.9 / 2.7 cm, ≤ 2.9°. Trial time 22.5 → 33.5 s.
  - **Arms down vs. up while walking (same layouts, `--walk-arms up`):** path deviation mean 7.0 vs **4.1 cm**, p90 15.1 vs 7.8 cm,
    pelvis sway 1.20 vs 1.04 cm, 0 falls both. So in sim the arms-down pose does **not** help balance – GR00T follows the path worse
    with it. Arrival is equally good (the checkpoint absorbs the deviation). Decision for the user: natural look vs. tracking.
    — Status: DONE

- [2026-10-19] Robot day — only the read-only part ran (checklist 84); no arm motion, no VLA attempt, no real data in the repo (told by Finnegan on 10-27). — Status: DONE (read-only)
- [2026-10-27] Status audit — read-only workflow (history/effort, Level 1 gap, Level 2/3 readiness, test health) + planner + independent critic; plan to the SAP visit ~11-06 in the 10-27 block above. Tests: 814 pass, 1 fail (runbook Part C rows missing). — Status: DONE

- [2026-10-27] Robot day Track C — review NO-GO (87 assumed an upright torso; real lean 0.126 rad makes clearances 2-6 cm worse) → fix → re-check GO; `87 --arms-up-first`: way in clear (≥ 9.9 cm), every way out (soft exit leg 2, fault exit, release) 5-16 cm into table/basket → no VLA attempt at the real table yet; runbook Part C rows filled; tests 0 failed in 3 envs (commit f174f1f). — Status: DONE
- [2026-10-27] Level 1 hardening — little-finger clearance: teacher constants CRADLE_DEG 20→10, REACH_ABOVE 0.04→0.06, SQUEEZE 0.025→0.03; new `g1sim/hand_clearance.py` + `67 --clearance`. Hand-to-table/stand/basket min 0.05 → 3.18 cm (p5 3.65); all teacher gates pass (10/10, 50/50, 50/50+50/50, 30/30, walk 300/300, eval 50/50, lift ≥ 8.19 cm). Plan + code review GO (`.planning/level1-harden/FINGER-PLAN.md`, `.planning/reviews/level1-finger-*.md`). Note: `67 --map` crashes at trial 63 (pre-existing since 450fd4b, shoulder lift assert); 62 trials: 52 pass old = new. Next: R5 lift-failure diagnostic → v2 demos (new teacher only) → run2. — Status: DONE

- [2026-11-04] Level 2/3 brain track prepared for Saad — `docs/L2_BRAIN_TRACK.md` (QR via OpenCV, object recognition via local Qwen-VL, Qwen3 orchestrator, all on the laptop; steps + checks + start prompt), `scripts/90_head_frames.py` (full-resolution 1280 x 800 head frames + ground-truth labels), 12 sample frames in `tests/data/l2_brain/`; branch `l2-brain`. — Status: DONE

## WHY RL DOES NOT WORK FOR "TAKE THE BOTTLE"

Standing can be learned because the reward is **dense**: almost every action gives feedback
("a bit straighter = a few more points"). The algorithm always has a direction.

Grasping has a **sparse reward**: there are points only once the bottle is in the hand.
A randomly twitching 26-DOF arm practically never gets there → reward always 0 →
no learning signal. Rough math: ~13 joints would have to be about right at the same time;
with 10% tolerance per joint that is ~1 in 10 trillion. At 7,500 steps/s that would be
on the order of 40,000 years.

**That is why nobody trains manipulation with pure RL from zero.** The three real paths:
1. **Give the geometry (Track A)** — the VLM says WHERE, math computes HOW. No training.
   Core already verified: 0.9 cm accuracy.
2. **Show it (imitation/VLA, Track B)** — 100-500 teleop demos, the model copies them.
   This is how π₀, UnifoLM-VLA-0 and all working robot VLAs are made.
   Concrete path (as of 2026-09-09): record with `xr_teleoperate` → convert + train with `unitree_lerobot`
   → base π₀/π₀.₅ from `openpi`. Needs a VR headset
   (or controller mode) **and** a Linux computer with an NVIDIA GPU.
3. **RL with a hand-written guiding reward** — works for ONE narrow task,
   must be rewritten for each task. This is where months disappear.

## CORRECTION TO THE PLAN ASSUMPTION (important)

PLAN.md says, roughly, "training is impossible on the Mac". **That was too general.**
Measured: ~3,900 env steps/s per core, in practice ~25-35k/s over several processes.

- **Learning standing/balance (~5-20 million steps): doable on this Mac in ~20-60 min.**
- Full walking (~150 million steps): doable overnight, but not needed.

**The real blocker is NOT compute, but:** MuJoCo Playground and mjlab
are CUDA-only. On the Mac you would have to write the reward function, observation space and domain
randomization yourself — that is weeks of tuning work, and that is where humanoid RL projects
fail, not on FLOPs. On top of that: **the G1 can stand and walk out of the box.**

## BLOCKERS / OPEN DECISIONS

### Update 2026-10-07 (English)
**Resolved:** 23 vs. 29 DoF → **29 DoF** (user confirmed the spec list, 2026-10-07). Blocker 1 below is closed; D5 is unblocked.

**New / open — questions to send (who to ask):**
- **Backpack PC (robot owner):** which RTX card + how much GPU memory (π₀.₅ needs > 8 GB to run on the robot)? Which OS (BrainCo
  driver + openpi need Linux)? How heavy is the backpack (for the sim payload)?
- **Hands + access (robot owner):** how are the BrainCo hands wired and which Linux computer drives them? Developer access: login,
  IPs, `arm_sdk` permission?
- **Robot time + safety (robot owner):** access hours, harness/gantry, a second person on the remote; is a VR headset there for teleop?
- **SAP system (SAP):** a test system (S/4HANA with EHS, or BTP) for the report-back step?
- **Data rule (SAP):** does "code goes into SAP's network, nothing comes out" still hold? If yes, a model trained on SAP hardware
  stays trapped there (see blocker 3).
- **Licenses (SAP legal):** π₀.₅ weights (Gemma terms); YOLO AGPL — or stay with RF-DETR (Apache-2.0).
- **Cloud (SAP / course):** AWS account + budget for GPU training and Bedrock calls.
- **Course + room (team / supervisors):** pitch date + grading; room changes (2nd table, box, markers, filming); where the teammates'
  safety check runs.
- **Team (Cristina + Saad):** outcome of the 2026-10-07 meeting — goal, simulator, success definition.

1. **[RESOLVED 2026-10-07: 29 DoF] Does our G1 have 23 or 29 DOF?**
   Menagerie model = **29 DOF** (7 joints per arm, incl. wrist pitch+yaw).
   Issue #46 describes `g1_brainco` with *"14 arm joints incl. 4 phantom wrist slots"* —
   that only makes sense for a **23-DOF G1 (Edu)**, which has only 5 real joints per arm.
   Consequence with 23 DOF: no wrist pitch/yaw → **5-DOF arm → IK much harder**,
   grasp orientation strongly limited, and the Menagerie model does not match the hardware.
   → Check this first when unpacking. Until then, do not guess.
   **Defused 2026-09-09:** `xr_teleoperate` supports **both** variants natively.
   So for the VLA path the question is not critical; it only blocks our
   MJCF model and the IK — so step 1d, not the overall progress.
   **Update 2026-09-14:** The 26-D action vector is the same for both variants (with 23 DoF the
   slots for wrist pitch/yaw stay empty). So 1d is no longer blocked. The only open point is whether the IK and the
   sim model may use wrist pitch/yaw.
   **10-second test on the robot:** Can the hand nod up/down at the wrist? Yes = 29.

2. **[Phase 6] VR headset available?** `xr_teleoperate` needs Apple Vision Pro,
   PICO 4 Ultra Enterprise or Meta Quest 3.
   **Addition 2026-09-09:** There is also a **controller tracking mode** and
   a **simulation mode** (`unitree_sim_isaaclab`). But both still need
   Linux — so they do not replace Blocker 3, only the headset.

3. **[Stage 0.5 — needed before Stage 2 at the latest] Linux computer with an NVIDIA GPU.**
   No longer only "from Track B on": `openpi` (*"tested with Ubuntu 22.04, we do not currently
   support other operating systems"*), `xr_teleoperate` (Ubuntu 20.04/22.04, Python 3.10)
   and the BrainCo hand control (RS-485 + ROS 2) all do **not run on macOS**.
   Needs: π₀ inference >8 GB VRAM · LoRA fine-tuning >22.5 GB (one RTX 4090 is enough) ·
   full fine-tuning >70 GB.
   ⚠️ **Clash with the SAP constraint:** code may go into the SAP network, but nothing may come back out.
   If the only GPU is the SAP hardware on site, the fine-tuned model is created
   *inside* the network and we never get it out. **Clarify before the first SAP day.**
   **Update 2026-09-14:** With LeRobot π₀.₅ instead of openpi, openpi's Ubuntu requirement no longer applies,
   but an NVIDIA GPU (`--policy.device=cuda`) is still needed. The LeRobot docs size full fine-tuning
   for an 80 GB GPU; `train_expert_only=true` needs less. Measure the real need in Stage 2.2.

4. **[Phase 2] VLM choice** — API (simpler) vs. local Qwen2.5-VL (offline story).

5. ~~**Target task**~~ → decided 2026-09-14, see below. Grasp with one arm, the **right** one
   (the only hand with a `grasp_site`), distractors stay.

6. ~~**[Stage 0.2] Does the LeRobot G1 sim run on the Mac?**~~ → **Yes** (2026-09-14): 10 m without a fall,
   0.54× real time, without a window. Only open point: the MuJoCo window under `mjpython` freezes when it opens
   (substitute for watching: `--video`).

### Settled decisions
- ~~VLA base~~ → Track A (VLM planner) stays the deliverable. **For Track B the base is
  now π₀ / π₀.₅ via `openpi` (Apache-2.0)**, not UnifoLM-VLA-0 (CC BY-NC-SA 4.0 =
  non-commercial, and so the wrong basis for an SAP business case).
  Recommendation from the team session on 2026-09-09.
- ~~Isaac Gym~~ → dropped, deprecated. If RL is ever needed: `unitree_rl_mjlab`.
- ~~Scope / target task~~ → **masterplan** (fetch-and-bring task with navigation), built in Stages 0–5,
  grasping first. Track A = demo generator + baseline, π₀.₅ = goal. Decided 2026-09-14.
- ~~Walking in the sim~~ → **LeRobot G1 sim + ready-made walking controller** (GR00T/Holosoma) for leg movement
  and balance, not trained by us. "Controller" does **not** mean remote control: the commands come autonomously
  from navigation + orchestrator. Full autonomy incl. localization is already tested in the sim (Stage 3.3).
  Decided 2026-09-14.
