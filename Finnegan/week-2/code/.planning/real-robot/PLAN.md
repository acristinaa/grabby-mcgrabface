---
phase: real-robot
plan: 01-robot-day-2026-10-19
revision: "7.1 (2026-10-18 18:56, planner; revises rev 7 after plan review rev 6 round 5 - R5-1 87 picks its start poses by arm movement (<= MAX_SAMPLE_STEP_M, max_start_step_m asserted, --frame-step dropped), R5-2 push 1 = the local commits + a reviewed push-1 fix set (Track A follow-up, runbook A+B + RV fixes, E2 after its fix round), one runbook writer at a time, 84 x5 from the clean clone, R5-3..R5-5 minors folded in, plus Finnegan's facts: motorized head set by the owner, robot-PC SSH login only on site, table stays on its tape marks; see 'Review response (round 5) -> revision 7.1'). Rev 7 (2026-10-18 18:28, planner; revises rev 6.3 after plan review rev 6 round 4 - R4-1 87 samples every path by how far the arm moves (MAX_SAMPLE_STEP_M), R4-2 every split file in the frontmatter and push 1/push 2 proven from a clean local clone, R4-3..R4-7 minors folded in, plus the open planner items of the A and B1 code reviews; see 'Review response (rev 6 round 4) -> revision 7'). Rev 6.3 (2026-10-18 ~18:10, planner; revises rev 6.2 after plan review rev 6 round 3 - R3-1 the policy check uses the prefix's own HF cache, R3-2 87 and the lookup per waist limit, R3-3..R3-11 minors folded in; see 'Review response (rev 6 round 3)'). Rev 6.2 (2026-10-18 17:29, planner; revised rev 6.1 after plan review rev 6 round 2 - R2-1 arm exits from a policy pose at the table, R2-2 factory-pose overwrite after a clean exit + early run marker, R2-3 lookup bounds, R2-4 re-timed from an ~18:05 build start, R2-5 'hangs in the gantry' defined, R2-6 E1 install detached + CPU self-test allowed). Rev 6.1 (2026-10-18 ~15:45, planner) revised rev 6 after plan review rev 6 round 1 - RV6-1 table clearance, RV6-2 factory-pose record in the Part C mode, RV6-3 honest push-1 timing, minors m6-1..m6-13; rev 6 = scope change by the user: the VLA attempts the grasp on Monday 10-19, operated by Cristina and Saad; Tier B returns behind a motion gate)"
depends_on:
  - Level 1 Step 4 (policy server 81, wire g1sim/policy_wire.py, review GO)                      # done 2026-10-15
  - "Step 8 sim eval: robot candidate = run1 40k (36/50 held-out starts)"   # done 2026-10-16
  - DDS sim with BrainCo bridge ~/dev/g1-ext/g1_brainco_sim (built by scripts/51, Stage 0.3d)      # exists
  - model release packed: ~/dev/g1-ext/release/smolvla_run1_040000.tar + .sha256                   # exists, upload by Finnegan
files_modified:
  # Push sets (R4-2): a track's push set = its files below + any file the track's STATUS line or review reports as a split.
  # Every push is proven from a clean local clone of the commit (6.2, 6.4), never from this working tree.
  # Push 1 (R5-2) = the local commits c0898cf (A) + f897b0c (E1) + e914654 (B1, inert, ruling R18) + ONE push-1 fix-set commit:
  #   Track AF (A follow-up: real_io.py, 84, test_unit_real_io.py), Track RB (runbook A+B + RV-1..RV-8 fixes, loopback_lowstate_pub.py
  #   one line) and E2's files after E2's fix round - each behind its own review gate (section 5, 6.2).
  # Track A - read-only bring-up (push 1; committed locally 18:15 as c0898cf, not pushed yet; AF edits real_io.py, 84, test_unit_real_io.py)
  - g1sim/real_io.py                          # new: motor layout, 27-D mapping, LowState/hand READERS, writer-sample check, publisher guard
  - g1sim/real_factory_pose.py                # new (split out of real_io by Track A, ~300-line rule): factory-pose record (+ mode note), run-log dir, arms_may_be_held_by_us; re-exported by real_io
  - g1sim/real_bringup.py                     # new (split out of 84 by Track A, ~300-line rule): step verdicts, factory-pose record step, Part B lines; 84 imports it (R4-2)
  - g1sim/real_camera.py                      # new: D455 grabber (runs on the robot PC over SSH), frame reader, framing checks
  - scripts/84_real_bringup.py                # new: read-only checklist, owner questions, factory-pose record, no-motion recording
  - tests/helpers/fake_camera.py              # new: streams recorded frames in the grabber's byte format
  - tests/helpers/real_fakes.py               # new (A fix round 1): shared fake readers/writers; four A test files import it (R4-2)
  - tests/test_unit_real_io.py                # new
  - tests/test_unit_real_camera.py            # new
  - tests/test_unit_no_publisher.py           # new: static ban + import check + start-up guard (seen failing first)
  - tests/test_unit_real_factory_pose.py      # new (A fix round 1, ~300-line rule): factory-pose record and sidecar rules (R4-2)
  - tests/test_unit_real_bringup.py           # new (A fix round 1, ~300-line rule): 84 step verdicts (R4-2)
  # Track A2 - read-only follow-ups after push 1 (push 2, if its review and the final gate are GO): edits of the Track A files above
  #   plus g1sim/real_camera.py's message and scripts/setup_robot_laptop.sh's gh check (walk proposals 4, 5; R5-4)
  # Track B1 - safety layer, pure logic (review GO round 5; committed locally 18:44 as e914654, so it rides in push 1 as inert code - R18)
  - g1sim/real_safety.py                      # new
  - g1sim/real_timing.py                      # new (split out of real_safety by Track B1, ~300-line rule): chunk timing + staleness; re-exported by real_safety, loaded by 86 (motion file)
  - tests/test_unit_real_safety.py            # new
  - tests/test_unit_real_safety_filter.py     # new (split out of test_unit_real_safety, ~300-line rule): the per-message SafetyFilter
  - tests/test_unit_real_timing.py            # new: chunk timing + staleness
  # Track B2 - adapter 86 and motion gate (push 2, only if every gate is GO)
  - scripts/86_real_run.py                    # new: CLI, modes gate/release/hands-open/ready/replay/shadow/run
  - g1sim/real_run.py                         # new: the 50 Hz phase runner with injected I/O (testable with fakes)
  - g1sim/real_send.py                        # new: the ONLY publishing module (rt/arm_sdk, rt/brainco/<side>/cmd)
  - g1sim/motion_gate.py                      # new: unlock JSON + pinned-commit check + operator confirmations
  - g1sim/real_preflight.py                   # new (split out of real_run by Track B2, ~300-line rule): rule-2 preflight checks (R4-2)
  - g1sim/real_run_log.py                     # new (B2 split): npz log + two-step sidecar (R4-2)
  - g1sim/real_run_modes.py                   # new (B2 split): ready/release/hands-open/exit paths (R4-2)
  - g1sim/real_run_policy.py                  # new (B2 split): policy phase, worker thread, exit requests (R4-2)
  - tests/helpers/fake_real_run.py            # new (B2 split): fake RunIO pieces for B2's tests (R4-2)
  - tests/test_unit_real_run.py               # new
  - tests/test_unit_motion_gate.py            # new
  - tests/test_unit_real_preflight.py         # new (B2 split, STATUS 18:34; R5-5): preflight refusals
  - tests/test_unit_real_send.py              # new (B2 split, STATUS 18:34; R5-5): fill_arm_sdk / senders
  # Track C - sim stand-in and sim-to-sim tests (push 2 with B1/B2)
  - scripts/85_arm_sdk_sim.py                 # new: DDS sim + GR00T that obeys rt/arm_sdk (weighted blend into a COPY of LowCmd, latch)
  - tests/helpers/sim_host.py                 # new: loopback-only sim host library + loopback arm_sdk test publisher
  - tests/test_integration_arm_sdk_sim.py     # new: 85 alone (blend, latch, refusals)
  - tests/test_e2e_real_adapter_sim.py        # new: 84/86 -> 85 chain incl. fault injection; replay server; 40k run
  - scripts/87_arm_path_clearance.py          # new (RV6-1, R2-1): FK-only clearance of the scripted arm paths and of the exits from policy poses against table, stand, basket
  - tests/test_integration_arm_path_clearance.py   # new (RV6-1)
  # Track E1 - handover kit (push 1; committed locally 18:14 as f897b0c, not pushed yet); Track E2 - runbook Part C + motion extras
  # (after E1 is GO; same owner, never parallel to E1; started 18:15; its files join push 1's fix set after E2's fix round GO, R5-2);
  # Track RB - runbook A+B + RV fixes (push-1 fix set; one runbook writer at a time: RB, then E2's fix round, then nobody until the commit)
  - docs/ROBOT_DAY_RUNBOOK.md                 # new: Part A Sunday setup, Part B Monday read-only (E1, RB); Part C Monday motion, only if UNLOCKED (E2)
  - scripts/setup_robot_laptop.sh             # new: robot env (Python 3.10) + policy env (Python 3.12) + release download; macOS, Ubuntu
  - scripts/89_robot_day_selftest.py          # new: robot-free self-test (publishes nothing itself)
  - tests/helpers/loopback_lowstate_pub.py    # new: the only LowState/hand-state test publisher, loopback only (RB: hand deadline line)
  - tests/helpers/make_robot_day_data.py      # new: builds the three data files below from the internal-disk dataset
  - tests/data/selftest_episode.npz           # new: 3 frames of dataset episode 0 (< 1 MB)
  - tests/data/replay_demo.npz                # new: all actions/states of dataset episode 0, head images blank (< 1 MB)
  - docs/robot_day/reference_head_frame.png   # new: frame 0 of dataset episode 0, for the camera overlay
  - tests/test_unit_runbook.py                # new
  - scripts/81_smolvla_server.py              # one-line change: "cuda" added to --device
  - docs/robot_day/volleyball_natural_phases.png   # E2: copy of the git-ignored docs/volleyball_natural_phases.png (M2 comparison, m6-1)
  # set ONLY by Finnegan at push 2 (never by an agent): docs/ROBOT_DAY_MOTION.json
  # integration / docs (main session + user): COMMANDS.md, PROGRESS.md, docs/START_PROMPT.md, docs/LEARNING_PATH.md;
  # docs/TESTING_GUIDE.md is written by the workflow's last phase
---

# Robot day Monday 2026-10-19: read-only bring-up (A), handover kit (E), and a gated VLA grasp attempt on the real G1 (B1, B2, C)

Repo root (all paths below are relative to it):
`/Users/finneganadam/Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot`.
`PY=~/dev/g1-ext/.venv/bin/python` (sim env, Python 3.12), `SV=~/dev/g1-ext/smolvla-test/.venv/bin/python` (SmolVLA env),
`DDS="DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib"`, `CKPT=~/dev/g1-ext/smolvla-runs/run1/checkpoints/040000/pretrained_model`,
`DATASET=~/dev/g1-ext/datasets/g1_volleyball_walkup_v1` (LeRobot v3.0, internal disk), `P=~/dev/g1-ext/robot-day-test` (fresh test prefix),
`REL=~/dev/g1-ext/release` (packed model tar + sha256).

## The situation
- **Robot day: Monday 2026-10-19 from 09:00.** Cristina (`acristinaa`) and Saad (`saadsheikh-tech`) operate from their laptops. Neither wrote
  this code. Finnegan is in Bielefeld; assume he cannot be reached during the day.
- **Build: Sunday evening to Monday morning on Finnegan's laptop, unattended overnight; Finnegan is reachable overnight for both pushes
  (STATUS 17:18).** Where the build stands (STATUS, read 18:19; R4-3): E1 started 17:38, A 17:43 (rev 6.2); **both are review GO at 18:13
  and committed locally at 18:14/18:15 (f897b0c, c0898cf; `main` is ahead of `origin` by 5, nothing pushed)**. B2 started 17:47 and B1
  17:49, both on **rev 6.2** (rev 6.3 was written at 18:07), so their rev-6.3 items reach them as review findings (section 5, seam 3).
  E2 started 18:15. C has not started; it starts on rev 7.1 with the R4-1 and R5-1 sampling rules in its task text, and its code
  review checks them against this revision. One workflow run at a time writes this plan, and each track file has exactly
  one writer.
- **Where the build stands at rev 7.1 (STATUS read 18:45; R5-2):** B1 review GO round 5, committed locally as `e914654` (18:44, on top
  of `c0898cf`; `main` ahead of `origin` by 6). B2 round 2 NO-GO (18:43), fix round 2 running (incl. the `MAX LEAD` line). C has not
  started. After A's GO, the main session's push-1 DDS run (18:16-18:20) found an **intermittent 84 crash** (a reader freed by Python's
  cyclic GC while cyclonedds 0.10.2 still calls it; `runs/real_a_fu_drop_reader_why.log`) and a **false "slow hand state"** (rate over
  the full window): Track AF (A follow-up) fixed both in the working tree (18:37), its review started 18:37. Cristina's paper walk of
  Parts A+B was NO-GO (18:22, F1-F5 + 15 minors); the fixes (18:33, incl. F21: the table height formula ~10 cm too low) were re-reviewed
  NO-GO at 18:39 (RV-1..RV-4 major, RV-5..RV-8 minor: hip-height state, foot marks, motorized head, head re-check); Track RB (the RV fixer,
  also the loopback publisher's hand-deadline line) started 18:40. E2 was GO round 1 with 6 minors (18:22), fix round not yet run.
  Push 1 is therefore redefined (section 6.2): the local commits plus one reviewed **push-1 fix-set** commit.
- **Facts from Finnegan (18:3x-18:45, via the coordinator):** (1) the D455 sits on a **motorized head** that the robot can move: the owner
  sets it with the head's own control (straight ahead = pan 0, 50 deg down); operators never touch it and only verify it through 84's
  overlay (rule 4.1 (3)); (2) the **robot-PC SSH login** (user + password) exists only on site at SAP, the owner gives it; it is never
  written into a file, 84's answers or the sheet; (3) the **table stays on its tape marks** and only the camera is corrected within
  49-51 deg (recommended default; Finnegan's confirmation is still pending, section 10 R19).
- **Power: the laptop ran on battery at 17:36, 17:43, 17:47, 18:13, 18:19 and 18:38 (`pmset -g batt`: "Now drawing from 'Battery
  Power'", 94-100 %, discharging); B1/B2 were launched on battery.** The main session must see `Now drawing from 'AC Power'` before it launches C
  (MuJoCo + DDS), the integration (40k on MPS) and the final gate; they do not last the night on battery.
- **Two pushes, both by Finnegan:**
  - **Push 1 (expected ~19:30-20:15, R5-2; after every fix-set gate is GO and the clean-clone check of 6.2 passes):** the local commits
    `f897b0c` (E1), `c0898cf` (A), `e914654` (B1, inert: nothing in push 1 imports it, ruling R18) and one **push-1 fix-set commit**
    made by the main session (CLAUDE.md rule 6) = Track AF's edits + Track RB's runbook/publisher edits + E2's files after E2's fix
    round (section 5, 6.2). The GitHub Release `robot-day-2026-10-18` already exists (verified 15:09). After the push, Finnegan messages
    Cristina and Saad that **Part A (setup + 89) can be done tonight** - the safer path than a Monday 08:00 install over venue Wi-Fi
    (R4-3) - and names the shared folder for `runs/` (section 10, R19 (b)). The push-1a/1b fallback of 6.1 stays only for the case that
    the clean-clone check fails for A's files.
  - **Push 2 (expected overnight, as soon as the final gate is GO and Finnegan is awake; target no later than ~08:00 Monday, may slip to
    ~08:30; latest 09:30 only if Finnegan confirms he is reachable until 09:30 - else the latest push is when he goes offline):**
    (i) Track A2, E2's later edits (integration's lookup fill) and every read-only fix that passed the final gate;
    (ii) the motion code (B1, B2, C) **only if every track review, the integration run and the final gate are GO**, followed by Finnegan's
    unlock commit (section 4.3). The unlock commit carries the date 2026-10-19 and the gate compares it with the local date, so it is
    made **after 00:00** (R4-3); a final-gate GO before midnight means the motion code may be pushed locked first and the unlock follows
    after 00:00. Otherwise motion files are not pushed and Monday stays read-only (the revision-5 day). Part B (read-only,
    about 1 h) comes first on Monday anyway, so Part C only needs push 2 pulled before M0; a push after 09:30 is not made (Finnegan is
    then not reachable) and Monday stays read-only.
  - Operators `git pull` at 08:00, at 08:30 if Finnegan's STATUS message says push 2 slipped, and once more before Part C if he says it
    landed after 08:30.
- **The user wants the VLA to really attempt the grasp.** Honest expectation (section 11): the sim-only 40k most likely misses at first.
  Success on Monday = safe motion, a clean slow replay, first VLA attempts that end in clean exits, and real data.

## 0. Hard constraints
- No robot is connected during the build. Everything is verified against fakes and the DDS sim on loopback; robot steps stay "not run".
- **No code reaches the operators unless it passed its review gate** (one findings file per gate, m6-12:
  `.planning/reviews/real-robot-code-<track>.md` for A, B1, B2, C, E1, E2; `real-robot-integration.md`; `real-robot-final.md`) and, for motion
  code, the final gate. **The push-1 fix set has its own gates (R5-2):** Track AF = section "Track A - follow-up review" in
  `real-robot-code-A.md`; Track RB = "Review of dry-walk A+B fixes" (written 18:39, NO-GO) plus a diff check of the RV-1..RV-8 lines,
  both in `real-robot-runbook-walk-AB.md`; E2's fix round = "Track E2 - round 2" in `real-robot-code-E2.md`. Build and review agents
  never commit, push, create the release or write `docs/ROBOT_DAY_MOTION.json`; the main session commits a stage after its gate is GO
  (CLAUDE.md rule 6, 2026-10-18); only Finnegan pushes.
- **Operators run only runbook commands and never edit files.** `72_real_check.py arm` and `72 hand --bewegen` are in the runbook's "never" box
  (F-72a: 72 `arm` leaves waist roll/pitch at kp 0, `72_real_check.py:34, 113-119`). **The only allowed 72 command is
  `python scripts/72_real_check.py kamera --suchen`** (R5-4; read-only camera lookup over SSH: `lsusb`, `v4l2-ctl --list-formats` per
  `/dev/video*`, the opencv version; lazy imports, no DDS, no publisher, nothing written - `72_real_check.py:101-111, 152-155, 186-206`,
  re-derived by the round-5 and the walk reviewer); every other 72 hint a FAIL line prints is not run. `70_real_goto.py` stays untouched
  and is not a runbook step (section 7, owner-optional note).
- **Head camera (Finnegan, R5 facts):** the head module is motorized and the robot can move it. Nothing in our code commands it (the walk
  reviewer's grep of `g1sim/real_*.py`, `motion_gate.py`, 84, 86 for head/xl330/dynamixel/set_head finds nothing; head control exists
  only in the sim). The owner sets it with the head's own control; operators never move it by hand and only verify it (rule 4.1 (3)).
- **Robot-PC login:** the SSH user and password exist only on site at SAP; the owner gives them. No file, no 84 answer, no sheet and no
  commit ever contains the password.
- Gate and test output goes to `runs/<name>.log` in full, never through `head`/`tail`. Background processes a gate starts are killed in the
  same block (`kill $PID; wait $PID`).
- **During the Build phase only Track C starts MuJoCo/DDS sims or GPU work.** The loopback LowState publisher runs in the main session's
  push-1 check (section 6.2), not inside track builds. One exception (R2-6): Track E1 may run the 81 policy self-test **on CPU** from the
  fresh prefix once its policy env exists (no MuJoCo, no DDS, no GPU); the push-1 check runs it again.
- **One loopback DDS run at a time (R3-4).** The sim, LeRobot, 71, 72, 52, 84, 85, 86, 89 and the loopback helpers all use DDS domain 0
  on lo0, so two runs at once see each other's samples (false FAILs, or a PASS for the wrong reason). Before any of them starts, this
  check must print nothing (the **one** pattern; 6.2 and Track C use exactly this text):
  `pgrep -fl "85_arm_sdk_sim|run_sim|g1_brainco_sim|sim_host|loopback_lowstate_pub|84_real_bringup|86_real_run|89_robot_day_selftest|pytest.*(test_e2e_real_adapter_sim|test_integration_arm_sdk_sim)"`
  (wait and re-check; never kill another agent's process). The `pytest.*(...)` part (R4-7) catches C's DDS tests: `sim_host` is a
  library, so `publish_test_arm_sdk` and `start_standing_g1` run inside a pytest process whose command line names only the test file
  (checked 18:19: macOS `pgrep` accepts this extended regex and does not match itself). Unit tests of refusals and guards fake `ChannelFactoryInitialize` and the
  writer classes, so a mutated (broken) copy fails without creating a real DDS writer (R3-11).
- Third-party code only under `~/dev/g1-ext`. The T7 is not used: test data comes from `$DATASET` on the internal disk.
- New code and text in plain English; existing German code stays untouched.
- **Clean code (the user's top priority):** small single-job functions with English names, type hints, a short module and public-function
  docstring saying what and why (safety reason, robot fact); every limit is a named constant with a one-line source comment; files stay under
  ~300 lines (split, do not grow); fail loudly with a message that tells the operator what to do.
- **Meaningful tests:** few; each named `test_<what>_<expected>` with the docstring `Proves: ... / Catches: ...`; Arrange-Act-Assert; one
  behaviour per test; fakes, never a robot. Checks marked **[mutation]** below must first be seen failing on a deliberately broken copy; the
  implementer records each in `runs/<track>_mutation.log` and in its STATUS line.
- **Test collection in every env (m6-3):** `scripts/run_tests.sh` also runs pytest in the project `.venv` (no cyclonedds, unitree_sdk2py or
  cv2). Every new test file that imports `real_io`, `real_camera`, `real_send`, `real_run`, `motion_gate` or 84/86/89 at module level starts
  with `pytest.importorskip("unitree_sdk2py")` (and `pytest.importorskip("cv2")` where it needs cv2), so it is skipped there with a reason,
  not a collection error. Pure-numpy files (`test_unit_real_safety.py`) need no skip.

## 1. Assumptions
- Assuming: one operator laptop ("robot laptop") is macOS or Ubuntu x86_64, cabled to the robot network at `192.168.123.222/24`
  (`scripts/72_real_check.py:16-19`), and runs 84 and 86 in the robot env (Python 3.10). Linux on ARM is not supported (no cyclonedds 0.10.2
  wheel). Windows only as a policy-server host, untested.
- Assuming: the policy server (81, 40k) runs on the robot laptop itself if it is an Apple-silicon Mac, else on the second laptop (macOS MPS or
  Linux CUDA) reachable over TCP (`81 --host 0.0.0.0`). Decided Sunday evening by which laptop passes 89's policy check (ruling R11).
- Assuming: BrainCo hands via `brainco_hand_service` on PC2 (`rt/brainco/*`, Q4); D455 on a Linux computer with `python3` + `opencv` (Q2).
- Assuming (Finnegan, R5 facts): the D455 sits on a motorized 2-DoF head (pan XL330 +-50 deg, tilt below it; `g1sim/payload.py:5-6`;
  training pose pan 0, `table_scene.HEAD_POSE`) that the owner can set and hold at pan 0 / 50 deg down with the head's own control, and
  that keeps that pose until something moves it (a mode change or reboot may re-home it - unknown, so 84 `camera` is re-run, rule 4.1 (3)).
  Whether the head angle is visible on DDS is unknown; our check is 84's overlay by eye.
- Assuming: the robot-PC SSH login is given by the owner on site (Q5); Parts A and the non-camera Part B steps do not need it.
- Assuming (recommended default, Finnegan's confirmation pending, R19 (a)): the table stays on its Part B tape marks (24.5 cm from the
  hip axis, 87's nominal layout) and only the camera is corrected within 49-51 deg; a remaining overlay offset is written down, not
  fixed by moving the table, because the M0 lookup (87) is computed for the tape-mark position.
- Assuming: Unitree's controller blends `rt/arm_sdk` linearly by the weight in `motor_cmd[29].q` and latches the last message when messages
  stop. Neither is published; 85 emulates exactly that, so the sim gate proves our code path, timing and safety behaviour, not Unitree's
  controller. The factory-pose check + `release` cover both latch and timeout (section 4.2, rule 12).
- Assuming: the owner places the robot at the table with their own remote and hangs it in a gantry/harness for every motion step.
- Assuming (RV6-1): the real factory arm hold pose is unknown until Monday M0 and may have the forearms angled down (elbow > 0). The scripted
  paths between that pose and `BALL_READY_POSE` are straight joint-space lines, so whether they clear the table edge depends mostly on the
  factory elbow angle. Track C computes the clearance by forward kinematics (FK = joint angles -> link positions, no physics) for a grid
  of elbow angles; the runbook turns it into a lookup the operators apply at M0. The sim kinematics stand in for the real ones.
- Assuming (R2-1, corrected by R3-7): `86 release` after a crash moves our own straight line at weight 1 (case 2), so its path is known.
  The fault ramp-out (w 1 -> 0, case 1) follows the straight joint line from the pose we hold to the controller's hold pose **only if the
  controller blends the target angles q alone**. If it blends at torque level (q, kp and kd weighted, as 85 does), each joint progresses at
  its own rate (`w*kp_ours / (w*kp_ours + (1-w)*kp_factory)`, factory gains unknown) and the hand path bows away from the line. Neither
  blend type is published. 87 therefore checks the straight line **and** the two extreme orderings ("shoulder joints first, then elbow
  and wrist" and the reverse) and reports the worst; this brackets the bow, it does not prove it (stated in R15). The 5 cm margin is meant
  to absorb the tracking lag and sag of a 2 s ramp, which nobody has measured on the real arm. So 87's own sampling must not eat that
  margin (R4-1): every path is sampled densely enough that no point of an arm or hand collision geom moves more than 2 cm between two
  samples, which bounds the sampling error near the 5 cm line to about 1 mm (section 3.5).
- Assuming (R3-2): the factory waist yaw is close to 0 (bounds line, rule 16 (a)), so 87's hanging poses with waist 0 stand in for it, and
  `--waist-hold` (motor 12 held at q0) is checked as waist limit 0.0.
- Assuming (R2-1): the arm poses of dataset episode 0 (`tests/data/replay_demo.npz`) stand in for the poses the 40k policy reaches at the
  table. They are exact for M4 (the replay plays them) and an approximation for M6. They were recorded with that episode's randomized ball
  and stop pose, while 87 uses the nominal layout, so a frame can start a few cm closer to the stand than in its own episode.
- Assuming (R4-5): 87 measures the arm and hand collision geoms only, not the ball. The size-5 volleyball (r 0.105 m, 0.27 kg, soft) is
  held between both hands after the lift (`table_scene.py:52-53, 114`), so a fault while it is lifted may press the ball onto the table
  edge between the descending hands. Accepted as a residual risk (light, soft object; no person in reach) and named in rule 16 (f), R15
  and the runbook; modelling a ball sphere between the palms is not built tonight.
- Assuming (RV6-2): the controller's arm hold pose can change with the robot mode (issues #156/#170), so the factory pose is recorded only in
  the mode Part C uses, right before the first 86 command (M0).
- Assuming: dataset episode 0 of `$DATASET` is a successful level-1 demo (its source stem is read from `meta/episodes` if present). E checks
  that its head image key is `(200, 320, 3)`.
- `.planning/REVIEW-LENSES.md` is not in this repo; lenses walked from the harness template (section 12).

## 2. Findings the plan rests on (evidence; re-checked 2026-10-18 where marked)
### 2.1 `rt/arm_sdk`
- Official example `unitree_sdk2_python/example/g1/high_level/g1_arm7_sdk_dds_example.py` (local clone `65691c8`, byte-identical to GitHub
  master per review round 1): `WaistYaw = 12`, `WaistRoll = 13`, `WaistPitch = 14`, arms 15-28, `kNotUsedJoint = 29 # NOTE: Weight`
  (lines 40-64); `arm_joints` = 14 arm joints + waist yaw/roll/pitch (91-103); weight 1 on the **first** message at the measured angles (135);
  ramp-out (164-168); kp 60 / kd 1.5, 50 Hz (69, 74-75). C++ example: same 17 joints, `max_joint_velocity = 0.5` rad/s (line 114).
- Issue #156: waist yaw honoured via arm_sdk on one firmware, arms not; issue #170: arm_sdk works in R2+A, R1+X rejected on 1.5.3. Mode
  depends on firmware -> Q8/Q10 with the owner.
- With weight > 0 we must also command 13 and 14 (F-72a; `unitree_sdk2py/idl/default.py:210, 219`: every MotorCmd defaults to kp 0).
### 2.2 BrainCo hands
`brainco_hand_service` `main.cpp`: `rt/brainco/{left,right}/cmd|state` (`unitree_go MotorCmds_/MotorStates_`), q in [0, 1] x1000, 0 = open,
order [Thumb, Thumb_aux, Index, Middle, Ring, Pinky] = ours (`g1sim/brainco.py:16`), 100 Hz, **no timeout** (the hand keeps the last command
if we die). Mapping = identity.
### 2.3 Joint order and gains
27-D 0-6 = motors 15-21, 7-13 = 22-28, 14-25 = hands, 26 = motor 12 (+ = turn left) (`action_space.py:5-9, 34`). Gains (re-checked today,
`table_env.py:42-43`): waist 12-14 kp 250 / kd 5; each arm kp 50, 50, 80, 80, 40, 40, 40 / kd 3, 3, 3, 3, 1.5, 1.5, 1.5. Signs: same enum in
LeRobot for sim and real, no flips (`lerobot/robots/unitree_g1/g1_utils.py:102-139`) - inference; checked on Monday visually at `ready`
and during the slow replay (section 7). `action_space.BALL_READY_POSE` (line 61) is the ready pose.
### 2.4 Head camera
fovy 65, 200 x 320 = 1280 x 800 / 4 (`payload.xml:51`, `table_env.py:35-40`); optical axis about **50.0 deg below horizontal** at the handover
(tilt 48.70 + mount 2.29 - pelvis 0.96), checked with a phone flat on the D455 top face (the axis lies in that plane). The head is
motorized (Finnegan): the owner sets pan 0 / 50 deg down with the head's own control, operators only check it. Capture 1280 x 800,
downscale by 4 (INTER_AREA), default 15 fps. Dataset image mean RGB (0.378, 0.479, 0.551), std (0.272, 0.252, 0.230).
### 2.5 DDS sim and LeRobot seams (re-checked today)
- `g1_brainco_sim` publishes `rt/lowstate` at **127.8 Hz** on lo0 (`runs/real_probe.json:38`); `g1sim/brainco_env.py` adds `rt/brainco/*`
  (`use_brainco_sim(..., cameras=False)`, line 223).
- LeRobot `unitree_g1.py` (sim env clone): `publish_lowcmd` rewrites the persistent `self.msg` (lines 250-265) and writes it with
  `self.lowcmd_publisher.Write(self.msg)` (line 266; publisher created at line 370). 85 therefore blends into a **copy** (review m6).
- LeRobot initialises DDS on `"lo"`, which does not exist on macOS: initialise on `lo0` before the LeRobot import, as
  `scripts/52_brainco_hand_dds.py:28-29` does (review r5-1).
### 2.6 How `unitree_sdk2py` publishes (what the no-publisher guard covers)
`core/channel.py:8` imports `cyclonedds.pub.DataWriter` at module level; `ChannelPublisher` (line 256) builds its writer with that name
(line 127). So `cyclonedds.pub` is always loaded; the guard patches `unitree_sdk2py.core.channel.ChannelPublisher`,
`unitree_sdk2py.core.channel.DataWriter` and `cyclonedds.pub.DataWriter` (review rev 5 confirmed).
### 2.7 Environments
- Robot env (84, 86, 89): **Python 3.10** (cyclonedds 0.10.2 wheels exist for cp37-cp310 only), `--only-binary cyclonedds`; pins numpy 2.2.6,
  opencv-python-headless 4.13.0.92, cyclonedds 0.10.2, unitree_sdk2py at `65691c8`. `uv python install 3.10` is the default on Ubuntu too
  (24.04 has no `python3.10` apt package, r5-5). 86's imports are numpy-only repo modules (`policy_wire` -> `episode` -> `action_space` ->
  `brainco`, re-checked today) plus the SDK.
- Policy env (81): Python 3.12, lerobot 0.6.1, torch 2.11.0 (Ubuntu + NVIDIA: CUDA wheel index), backbone cache 2.7 GB, `HF_HUB_OFFLINE=1`.
  **The backbone cache must live in the prefix (R3-1):** 81 sets `HF_HUB_CACHE` only by `setdefault` to Finnegan's
  `~/dev/g1-ext/smolvla-test/hf_cache/hub` (`81_smolvla_server.py:37, 67-68`), which holds SmolVLM2-500M-Video-Instruct + `smolvla_base`
  on this Mac only (the checkpoint's `config.json` has `load_vlm_weights: true`). So every operator-path run of 81 sets
  `HF_HUB_CACHE=<prefix>/hf_cache/hub` explicitly (`policy-env.sh` exports it; 89 passes it), and tonight's checks assert that cache, so
  they fail here exactly when they would fail on an operator laptop.
- 81 accepts `--device mps|cpu` only today (`81_smolvla_server.py:159`, re-checked) -> E adds `cuda`. `policy_wire.device_problem` only
  inspects `mps` (line 343-348), so no other change is needed.
### 2.7b Table geometry at the walk stop (RV6-1; `g1sim/table_scene.py`, re-read 2026-10-18)
`TABLE_TOP = 0.74`; the volleyball table's front edge is at x = 0.20 in the fixed-pelvis scene; at the nominal walk stop the pelvis is 0.2455 m
from the front edge (`R_STOP = 0.458`, lines 78-82) at pelvis z 0.7505 (`GROOT_STANCE`). Ball at (0.344, 0.241) on a tube stand (inner radius
0.05, wall 0.006, height 0.08); basket at (0.344, -0.241). `BALL_READY_POSE` (forearms raised, elbow -0.8) keeps >= 4.9 cm to ball, stand,
basket and table at the nominal stop (`action_space.py` comment above `BALL_READY_RIGHT_ARM`). Elbow axis +y: elbow > 0 points the forearm
down; at elbow 0 the forearm points forward. Demo episode 0 starts and ends at `BALL_READY_POSE` (review rev 6 round 1), so no demo ever moved
between a hanging pose and the ready pose near the table. Policy poses at the ball put the hand above the table top about 0.15 m behind
the front edge, so an exit from there down to a hanging pose crosses the plane of the front edge (review round 2, R2-1; an estimate from
this geometry, computed by 87 tonight). Training reach: the ball must stay 0.33-0.52 m from the pelvis (`table_scene.py:101`, `reach_m`), so
a large setback puts the ball out of the policy's reach.
### 2.8 Demo statistics (envelope)
`$DATASET/meta/stats.json`: shoulder pitch -1.62..-0.25 both sides, waist -0.71..0.68, hands 0..0.3 (thumbs) / 0..0.15 (fingers). Envelope =
action min/max +- 0.10 rad. A hanging factory arm (shoulder pitch around 0) is **outside** the envelope, so the envelope applies to the
`policy` phase only (review B1). `BALL_READY_POSE` right shoulder pitch -0.25 is inside only through the margin (test-pinned).

## 3. Contracts (fixed here so the five tracks A, B1, B2, C, E1 build in parallel; E2 follows E1)
All new modules: Python 3.10 clean, numpy-only unless stated. "17-vector" = values for motors 12..28 in that order (position i = motor 12 + i).

### 3.1 `g1sim/real_io.py` (Track A; readers only, NO publisher)
```
LOOPBACK = ("lo0", "lo")
SDK_MOTORS = tuple(range(12, 29)); WAIST_YAW = 12; WAIST_HOLD = (13, 14); ARM_MOTORS = tuple(range(15, 29))
MOTOR_NAMES: tuple[str, ...]  (29, equal to 72's JOINTS)
JOINT_LIMITS: dict[int, tuple[float, float]]           # 12..28 rad, literal, test-pinned to the MJCF
LOWSTATE_HZ = {"loopback": {"refuse": 100, "warn": 100}, "robot": {"refuse": 200, "warn": 400}}   # review M3, n1
lowstate_hz_check(iface: str, hz: float) -> str        # "ok" | "warn" | "refuse"
is_loopback(iface: str) -> bool
robot_net_present() -> bool                            # any interface with a 192.168.123.x address
state27(lowstate, hand_left, hand_right) -> np.ndarray # (27,) float32; hand_* None -> NaN in its 6 slots
sdk_q(lowstate) -> np.ndarray                          # (17,) measured q of motors 12..28
class LowStateReader(iface)                            # .latest() -> (msg | None, age_s); .rate_hz(window_s); KEEP_LAST 1 QoS (r5-3)
    # (Track AF, R5-2) .rate_hz = gaps between the samples in the window / time from the first of them until now (< 2 samples -> 0),
    # so a reader that listened 50 ms reports the stream rate, not samples / full window (the false "4 Hz" of runs/push1_84_rep1.log)
class HandReader(iface, side)                          # same interface for rt/brainco/<side>/state
# (Track AF, R5-2) every reader channel stays referenced in a module list until close_dds(): a reader dropped by its caller and freed
# by the cyclic GC skips dds_delete in cyclonedds 0.10.2 and the next sample calls freed memory (SIGSEGV, runs/real_a_fu_drop_reader_why.log)
close_dds() -> None                                    # deletes the participant (and every reader) via dds_delete; registered with
                                                       # atexit by init_dds, so it runs after 86's `finally` ramp-out and before teardown
ARM_CMD_TOPICS = ("rt/arm_sdk", "rt/lowcmd")          # LowCmd_ type; weight read from slot 29
HAND_CMD_TOPICS = ("rt/brainco/left/cmd", "rt/brainco/right/cmd")   # MotorCmds_ type (R3-10 (c): an owner's hand tool would fight ours)
active_writers(iface, topics=ARM_CMD_TOPICS + HAND_CMD_TOPICS, seconds=2.0) -> dict[str, dict]
    # {topic: {"samples": int, "max_weight": float | None}}; counts SAMPLES; message type chosen per topic; max_weight None on hand topics
own_writers(participant) -> int                        # DCPSPublication entries owned by this participant (must be 0 in 84)
install_publisher_guard() -> None                      # section 2.6; idempotent
joint_rows(lowstate) -> list[dict]; mirror_gap(state27) -> np.ndarray (7,)
# factory-pose record (review N4)
FACTORY_POSE_TOL = 0.10                                # rad; larger gap = arms not where Unitree's controller keeps them
REPO = pathlib.Path(__file__).resolve().parent.parent  # (R3-5) every safety file below is anchored here, never to the cwd
factory_pose_path(iface: str) -> pathlib.Path          # REPO/runs/factory_pose_robot.json, or REPO/runs/factory_pose_sim.json on loopback
save_factory_pose(path, q17, iface, mode_note: str, overwrite=False) -> None
    # writes {"q": [17 floats], "iface", "mode_note" (owner's Q8 answer, free text, RV6-2), "recorded_at": ISO time, "date": YYYY-MM-DD};
    # raises FactoryPoseConflict when a record from the same date exists and differs by more than FACTORY_POSE_TOL, unless overwrite=True;
    # an empty mode_note on a non-loopback iface raises ValueError ("ask the owner which mode the robot is in (Q8) and pass --mode-note")
load_factory_pose(path) -> np.ndarray | None           # (17,) or None when missing or from another date
load_factory_record(path) -> dict | None               # the whole record (for messages: recorded_at, mode_note)
RUN_LOG_DIR = REPO / "runs" / "real"                   # 86's default --out (section 3.4); absolute (R3-5): a relative "runs/real"
                                                       # made 86 started from another cwd write sidecars 84 never finds (fails open)
BRINGUP_REPORT_DIR = REPO / "runs"                     # 84's runs/real_bringup_<stamp>.json reports (R3-5, R3-9)
earliest_factory_arms_today(report_dir=BRINGUP_REPORT_DIR, robot: bool = True) -> tuple[np.ndarray, str] | None
    # (R3-9) q17 and HH:MM of today's first 84 `joints` reading on a robot (or loopback) iface - Part B step 0, before any owner motion
arms_may_be_held_by_us(log_dir=RUN_LOG_DIR) -> bool    # (R2-2, replaces rev 6.1's motion_ran_today) reads today's 86 sidecars (3.4);
    # True when the newest one with "arm_sdk_writer": true is unfinished ("exit_code": null: killed, crashed, power lost) or ended with
    # "last_weight_sent" > 0, or (A2, R5-3) ended with "exit_reason" "lowstate_stale": with the link down our weight-0 ramp-out may
    # never have reached the robot, so the controller may still latch our last pose. False when there is none today or the newest
    # ended with weight 0 sent for any other reason (our last message gave the controller full control back). Refused runs (exit 2)
    # never set arm_sdk_writer and do not count.
factory_arm_summary(q17) -> str                        # (RV6-1, R2-3, R3-2) one line, both sides, plus the waist yaw:
    # "left/right elbow 0.95/0.97, shoulder pitch 0.21/0.20, shoulder roll 0.25/-0.24, shoulder yaw 0.01/-0.02, largest wrist 0.05,
    #  waist yaw 0.01 rad"
```
`real_factory_pose.py` holds the factory-pose and run-log names (Track A's split); `real_io` re-exports them, so every other track
imports them from `real_io` as listed here. `g1sim/real_bringup.py` holds 84's step verdicts (Track A's split of 84).
**Per-laptop safety state (R4-4):** the factory record, 86's sidecars and the Part B reading live in `runs/` of one clone, but the hazard
is per robot. So 84's M0 step and every 86 command run on **one laptop all day** (runbook page 1); if that laptop dies while 86 runs:
owner's remote, the owner re-stands the robot, then M0 on the new laptop. When `earliest_factory_arms_today()` is None, 84 prints
(A2) `PART B ARMS: no Part B reading found on this laptop - M0 cannot compare; if Part B or an 86 run happened on another laptop, stop
and ask the owner to re-stand the robot first`.
Overwrite rule (RV6-2, changed by R2-2): 84 refuses `--overwrite-factory-pose` when `arms_may_be_held_by_us()` is True ("our last 86 run
may still hold the arms (it did not end with weight 0): run `86 release` first, then re-record; if release is refused, ask the owner to
use the remote; do not overwrite"). When it is False nothing of ours can be latched, so the runbook allows the overwrite after the owner
confirms the arms are in the controller's hold pose - also after a mode change later in the day, once the last run ended cleanly.
### 3.2 `g1sim/real_camera.py` and `tests/helpers/fake_camera.py` (Track A)
`to_policy_frame(bgr) -> (rgb (200,320,3) uint8, cropped)`; `GRABBER` (python3 >= 3.8 on the robot PC, numpy + cv2
only; V4L2 `/dev/video<N>`, YUYV 1280x800, `--fps 15`, `--count N`; one JSON info line, then frames as `b"FRM1" + >I len + >d t + >H w + >H h
+ >B cropped + RGB`); `FrameStream(cmd, stdin_text=None)` with `.info`, `.latest() -> (frame | None, age_s)`, `.stats()`, `.close()`;
`grab(cmd, stdin_text, count=45, timeout_s=20)`; `ssh_cmd(host, *remote)`; `image_stats`; `DATASET_IMAGE_STATS`; `overlay(real, ref)`.
`fake_camera.py --npz FILE [--fps 15] [--stop-after S]` loops the file's `head_image` frames in the grabber byte format.

### 3.3 `g1sim/real_safety.py` (Track B1; pure logic, no DDS, no I/O, no repo imports except `g1sim.action_space` for `BALL_READY_POSE`)
```
SDK_MOTORS = tuple(range(12, 29))       # Unitree arm7 example lines 40-64 (same literal as real_io; a B2 test pins them equal)
WAIST_YAW_POS, WAIST_HOLD_POS = 0, (1, 2)   # positions in a 17-vector
SEND_HZ = 50                            # Unitree example rate, line 69
RAMP_SECONDS = 2.0                      # weight 0<->1 smoothstep (72_real_check.py:76-89)
MAX_WEIGHT_STEP = 0.02                  # per message in ramp_in/ramp_out; smoothstep over 100 ticks peaks at 0.015 (m6-8: B2's runner
                                        # checks it on every message it sends; release's first message, a jump to w = 1 at the measured
                                        # angles, is the one documented exception - Unitree arm7 example line 135)
SCRIPTED_SPEED = 0.3                    # rad/s for every scripted move (review N2)
ARM_SPEED_MAX, WAIST_SPEED_MAX = 1.0, 0.3   # rad/s command rate limit in every phase (rule 6)
LIMIT_MARGIN = 0.05                     # rad inside JOINT_LIMITS
ENVELOPE_MARGIN = 0.10; ENVELOPE_STREAK_TICKS = 30
LEAD_LIMIT = 0.25; LEAD_TICKS = 13      # 0.25 s at 50 Hz, rounded up (review M2/n2)
HAND_CAP = 0.6; HAND_SPEED = 0.5; HANDS_OPEN_SPEED = 0.3       # ruling R6
ROBOT_MAX_SPEED_FACTOR = 0.5            # motion gate (e)
ROBOT_WAIST_MAX_DEFAULT, ROBOT_WAIST_MAX_CEILING = 0.15, 0.6   # rad, ruling R10
STALE_S = {"lowstate": 0.1, "camera": 1.0, "hand_state": 0.5, "chunk_ran_out": 1.0}
POLICY_TIMEOUT_S = 2.0
# (m6-9) no time cap on the soft-exit path: a cap could cut the path short and the ramp-out would then swing the rest of the gap
EXIT_PATH_MARGIN_S = 1.0                # the runner faults if executing the exit path takes > planned duration + this (a stuck loop)
SIM_GAINS_KP, SIM_GAINS_KD: np.ndarray (17,)     # table_env.KP/KD[12:29] literal, test-pinned
ENVELOPE_LO, ENVELOPE_HI: np.ndarray (27,)       # stats.json action min/max -/+ 0.10, literal, test-pinned
PHASES = ("release", "ramp_in", "to_ready", "hold", "shadow", "policy", "to_start", "to_factory", "ramp_out")
ENVELOPE_PHASES = ("policy",)           # replay and VLA both run in "policy" (review B1)
EXIT_CODES = {"done": 0, "refused": 2, "soft": 3, "fault": 4}
class InvalidTarget(ValueError); class ChunkTooLate(RuntimeError)

smoothstep_weights(start: float, end: float, seconds: float = RAMP_SECONDS) -> np.ndarray   # one value per 50 Hz tick, monotone, ends at `end`
ramp_in(q0) -> list[tuple[float, np.ndarray]]          # (weight 0->1, q0) - q0 is frozen for the whole ramp (review N1)
ramp_out(q_hold, weight_now) -> list[tuple[float, np.ndarray]]   # weight_now -> 0 at q_hold
move(start, goal, speed=SCRIPTED_SPEED) -> list[np.ndarray]      # straight line; every joint |step| <= speed / SEND_HZ; last == goal
release_plan(measured, factory) -> list[tuple[float, np.ndarray]]
    # [(1.0, measured)] + [(1.0, q) for q in move(measured, factory)] + ramp_out(factory, 1.0)   (reviews M1, N2)
    # (B1 review P2) its 13/14 values are the factory record's; every caller sends release messages through SafetyFilter with
    # q0 = measured, which holds 13/14 at the measured angles (rule 4). B2's release() does; a caller that skips the filter must not exist.
soft_exit_plan(current17, q0, ready17, from_policy: bool) -> list[tuple[str, np.ndarray]]
    # (RV6-1) phase-tagged targets. from_policy: ("to_ready", q) for q in move(current, ready) - the pose the demos start and end in and
    # that clears ball, stand, basket and table - then ("to_start", q) for q in move(ready, q0). Otherwise only the to_start segment.
    # Hands are opened by the runner at the end of to_ready (a held ball drops from chest height onto the table, harmless) or, without
    # to_ready, before to_start. 13/14 stay q0 in every target.
exit_path_seconds(plan) -> float                       # len(plan) / SEND_HZ, logged and used with EXIT_PATH_MARGIN_S
sdk_targets(action27, q0) -> np.ndarray (17,)          # 12 <- action[26]; 13/14 <- q0 (held); 15-28 <- action[0:14]
hand_targets(action27) -> tuple[np.ndarray, np.ndarray]  # (left6, right6) clipped to [0, HAND_CAP]; (m6-7) hands get HAND_CAP only:
                                                       # never envelope-clipped and never counted towards envelope_streak
factory_pose_gap(measured17, factory17) -> float       # max |difference|
speed_refusal(speed: float, on_loopback: bool) -> str | None
waist_max_refusal(waist_max: float, on_loopback: bool) -> str | None
class SafetyFilter(limits_lo17, limits_hi17, q0, waist_max: float, waist_hold: bool)
    .step(target17, measured17, phase, weight) -> tuple[np.ndarray, list[str]]
    # order: NaN/Inf -> InvalidTarget; waist hold (12 := q0) or waist clip +-waist_max; envelope clip if phase in ENVELOPE_PHASES;
    # hard clip to limits -/+ LIMIT_MARGIN; 13/14 := q0; rate limit against the last command (arms 1.0, waist 0.3 rad/s);
    # lead monitor only while weight == 1.0. Events: "limit_clip", "envelope_clip", "envelope_streak", "contact_fault".
    .reset_last(command17)                             # used after a scripted move hands over to the policy phase
class ChunkScheduler(speed, chunk_size=50, exec_steps=25, action_hz=30)
    .position -> float                                 # action units since the policy phase began
    .advance(dt)                                       # position += dt * action_hz * speed
    .add_chunk(chunk (50, 27), requested_at: float)    # action j of the chunk plays at position requested_at + j (latency skip);
                                                       # raises ChunkTooLate when position - requested_at >= chunk_size
    .target() -> np.ndarray | None                     # linear interpolation at position; None before the first chunk or after it ran out
    .needs_request() -> bool                           # >= exec_steps actions of the newest chunk used and no request pending
    .mark_requested() -> float                         # returns position (pass back as requested_at); the infer request's
                                                       # tick = int(that position) (m6-5: the replay server then advances through the demo)
    .seconds_since_ran_out() -> float | None
stale_reason(lowstate_age, camera_age, hand_age, ran_out_s, mode) -> str | None
    # "lowstate_stale" | "camera_stale" | "hand_state_stale" | "policy_stale" | None; camera and hand-state only stop mode shadow/run
exit_kind(reason) -> str                               # "fault" for lowstate_stale, contact_fault, invalid_target, second_signal; else "soft"
fault_targets(reason, measured17, last_command17) -> np.ndarray   # measured, except last_command for lowstate_stale (review M2)
```

### 3.4 Track B2 modules and the 86 CLI
`g1sim/real_send.py` (the only module that publishes; topics `rt/arm_sdk` and `rt/brainco/<side>/cmd` only):
```
WEIGHT_SLOT = 29                                       # kNotUsedJoint, Unitree example line 64
fill_arm_sdk(cmd, weight, q17, kp17, kd17) -> cmd      # 17 motors q/dq=0/tau=0/kp/kd + motor_cmd[29].q = weight; ValueError on weight
                                                       # outside [0, 1], wrong length, non-finite, kp/kd above SIM_GAINS
class ArmSdkSender(iface)  .send(weight, q17, kp17, kd17)   # sets the CRC
fill_hand_cmd(msg, targets6, speed) -> msg; class HandSender(side)  .send(targets6, speed)
```
`g1sim/motion_gate.py` (decides whether 86 may publish at all; section 4.3):
```
MOTION_FILE = "docs/ROBOT_DAY_MOTION.json"; OPERATORS = ("acristinaa", "saadsheikh-tech")
loaded_repo_files(repo, modules=sys.modules) -> list[pathlib.Path]   # every module file under the repo + the __main__ script
gate_problems(repo, loaded_files) -> list[str]         # (a) + (b); empty list = unlocked
confirm_operators(ask=input) -> str | None             # (c); returns a problem or None
confirm_hanging(ask=input) -> str | None               # (d)
sim_bypass_problem(iface, yes_sim) -> str | None       # sim runs need a loopback iface, --yes-sim and robot_net_present() False
```
`g1sim/real_run.py`: the phase runner. `RunIO` (dataclass: lowstate reader, hand readers, frame stream or None, arm sender, hand senders,
policy client or None, clock, sleep) is injected, so unit tests drive it with fakes. Public: `run_mode(mode, io, args) -> RunResult`
(`exit_code`, `reason`, `log_path`). 86 = argparse + building real `RunIO` + printing.

**86 CLI (Tracks E1/E2's runbook and Track C's e2e code against this):**
```
86_real_run.py gate                                    # no DDS; prints "MOTION: UNLOCKED" or "MOTION: LOCKED" + one line per problem; exit 0 / 2
86_real_run.py release    --iface IF
86_real_run.py hands-open --iface IF
86_real_run.py ready      --iface IF [--hold S (5)]
86_real_run.py replay     --iface IF --server HOST:PORT --episode STEM [--speed F (0.25)] [--max-seconds S (120)] [camera opts | --no-camera]
                          # 120 s default (m6-5): the 726-frame demo lasts 726 / (30 x 0.25) = 97 s at speed 0.25
86_real_run.py shadow     --iface IF --server HOST:PORT [--max-seconds S (30)] camera opts
86_real_run.py run        --iface IF --server HOST:PORT [--speed F (0.25)] [--max-seconds S (60)] camera opts
camera opts: --robot-pc USER@HOST --camera-device N | --camera-cmd "CMD"
common: --waist-max RAD (robot default 0.15, ceiling 0.6) | --waist-hold; --no-hands; --out DIR (real_io.RUN_LOG_DIR); --label TEXT; --yes-sim
```
- `--out` (R3-5): a relative DIR is resolved against the repo root, never the cwd; off loopback a DIR outside `real_io.RUN_LOG_DIR` is
  refused (exit 2, "robot runs log only to runs/real, so 84 can see whether our last run may still hold the arms"). On loopback any DIR
  is allowed (C's e2e uses `--out <tmp>`).
- `replay` requires a server hello with `mode == "replay"`; `shadow`/`run` require a checkpoint server (catches a swapped server).
- Every infer request sends `tick = int(scheduler.mark_requested())` (m6-5).
- `replay` without hand state (m6-6): the missing hand slots of `state27` are filled with the last hand command and the event
  `hand_state_synthetic` is logged (as with `--no-hands`), because `check_observation` refuses NaN (`policy_wire.py:130`).
- `replay`/`shadow`/`run` print `real_io.factory_arm_summary(<recorded factory q17>)` and "table steps: check the runbook M4 lookup" before the gate prompts
  (RV6-1; 86 cannot see the table, the operator applies the lookup).
- Refusal for a factory-pose gap (RV6-2, R2-2 (iii)) offers only the way out that works now: it starts "arms are <gap> rad from the
  factory pose recorded at <HH:MM> (mode: <mode_note>)." and goes on, when `real_io.arms_may_be_held_by_us(out)` is True, "our last run
  may still hold the arms: run `86 release` first"; when it is False, "our code does not hold the arms. If the owner changed the robot
  mode, re-record the factory pose (runbook M0); otherwise ask the owner".
- Last stdout lines: `86 <MODE>: DONE | REFUSED <reason> | SOFT EXIT <reason> | FAULT EXIT <reason>` and `LOG <path to .npz>`. Exit codes 0/2/3/4.
- Log per run, never overwritten: `<out>/<YYYYmmdd_HHMMSS>_<mode>.npz` in the `g1sim/episode.py` layout (head_image, state, actions = the
  filtered command sent as 27-D, timestamp, task, fps 30, success False) + `policy_actions` (raw) + `sent_t (N,)`, `sent_phase (N,) str`,
  `sent_weight (N,)`, `sent_q / sent_kp / sent_kd (N, 17)`, `measured_q (N, 17)`; sidecar `.json` with `source` ("real" or "sim85"),
  `mode`, `exit_code`, `exit_reason`, `events`, `args`, `git` (commit, dirty), `gate` (checks, operators, `loaded_files`), `loaded_files_at_exit`,
  `arm_sdk_writer` (bool), `last_weight_sent` (float | null).
- Sidecar timing (R2-2 (ii)): 86 writes the sidecar **twice**. First right before it creates its first writer: `"exit_code": null`,
  `"arm_sdk_writer"`: true when an `ArmSdkSender` is about to be created (false for `hands-open`, which only creates `HandSender`s),
  `"last_weight_sent": null` (written to a temp file, then renamed, so a kill never leaves half a file). Then at exit with the final
  fields. A run refused before any writer writes it once at exit with `"arm_sdk_writer": false`. So a run that is killed or loses power
  after its first publish leaves `exit_code: null`, which `arms_may_be_held_by_us` reads as "may be held".

### 3.5 Track C: `scripts/85_arm_sdk_sim.py` and `tests/helpers/sim_host.py`
- `sim_host.py` (library): `refuse_unless_safe_loopback(iface)` (loopback iface and `robot_net_present()` False, re-checked every second
  by the publishing helpers, r5-2); `start_standing_g1(band: bool)` (lo0 init before the LeRobot import, `use_brainco_sim(cameras=False)`,
  `UnitreeG1(is_simulation=True, controller="GrootLocomotionController")`); `publish_test_arm_sdk(weight, q17, seconds)` (loopback-only test
  publisher used by C's tests to stand in for a foreign writer).
- `85_arm_sdk_sim.py --seconds S [--band] [--no-blend] [--pause-lowstate-at T --pause-for D] --report PATH` (sim env, lo0 only): latches the last
  `rt/arm_sdk` message; wraps `robot.lowcmd_publisher.Write` so a **copy** of LowCmd gets motors 12-28 `= w * sdk + (1 - w) * own` for q, kp,
  kd (w = `motor_cmd[29].q` clipped to [0, 1]), recomputes the CRC and writes the copy; `--no-blend` only counts samples (84's read-only check).
- Report JSON: `arm_sdk` = list of `{t, weight, q[17], kp[17], kd[17]}` as received; `arm_sdk_samples`; `lowcmd_foreign_samples`; `max_gap_s`
  between arm_sdk messages; `measured` = `{t, q[29]}` at 50 Hz; `pelvis_z_min`; `hand_cmd` received per side; `run_s`.
- **`scripts/87_arm_path_clearance.py` (RV6-1, R2-1, R3-2, R3-7, R3-8; sim env; `mj_forward` only - no stepping, no DDS, no GPU):**
  `87 --report runs/arm_path_clearance.json [--elbows 0.0:1.6:0.1] [--shoulder-pitches -0.1,0.1,0.3] [--setbacks 0.0:0.15:0.01]
  [--waist-limits 0.0,0.15,0.35,0.6] [--demo tests/data/replay_demo.npz]` and `87 --merge OUT.json IN1.json IN2.json ...` (fallback
  only, see runtime). There is **no `--frame-step`** (R5-1): start poses are chosen by arm movement.
  - Model: `TableEnv(scene="volleyball_walk").reset(randomize=False)` (nominal layout: pelvis at `WALK_STANCE` in `GROOT_STANCE`), waist 0;
    87 then only writes arm (and, for demo frames, waist yaw) qpos and calls `mj_forward` (MuJoCo 3.13.0 in the sim env has
    `mj_geomDistance`, checked 2026-10-18; called with `distmax` 0.10, so pairs farther apart than 10 cm are cut early and reported as 0.10).
  - Hanging pose h(e, s): every arm joint 0 except both elbows = e and both shoulder pitches = s (left/right equal, as the mirror leaves
    elbow and shoulder pitch unsigned), waist yaw 0. `GRID_BOUNDS` in the report: elbow 0.0..1.6, shoulder pitch -0.1..0.3, and
    `FACTORY_LOOKUP_TOL = 0.15` rad (named constant: the largest shoulder roll, shoulder yaw, wrist angle or waist yaw of a real factory
    pose for which the grid's "all other joints 0" still stands in, R2-3, R3-2).
  - Every pose is a 17-vector (motors 12..28) built with `real_safety.sdk_targets(pose27, q0=zeros(17))`, so 87 moves along exactly the
    joint lines 86 commands; 87 writes those 17 values into the matching qpos entries.
  - **Torso lean (Track C code review B1, 2026-10-27):** the waist roll/pitch of the 17-vector is not where the torso is. 86 holds
    13/14 at the measured q0 and the torso sags forward under the arms: e2e G2 (sim) held q0 pitch 0.041 and measured up to
    0.126 rad (roll <= 0.001). So 87's FK writes roll 0 and pitch `WAIST_PITCH_CHECKED` = 0.126 over 13/14 for every pose. Re-derived
    evidence (FK over all 726 demo frames, waist limit 0.15, runs/real_c_b1_waist_probe.txt): min table clearance 9.9 cm at pitch
    -0.1, 5.2 cm at 0, 3.3 cm at 0.0405, 1.2 cm at 0.085, -0.8 cm at 0.126, -2.0 cm at 0.15 - it falls monotonically with the pitch,
    so the largest measured pitch is the worst case; roll +-0.05 at pitch 0.126 costs another 0.4-0.6 cm. The upright torso (pitch
    0) the earlier sentence implied read the whole 5 cm margin too optimistic. `WAIST_PITCH_SAG` = 0.085 (= 0.126 - 0.041, same run)
    is the sag below the hold; it is a sim number - the real sag is not measured yet.
  - **Sampling by arm movement, not by tick count (R4-1).** A coarse even sampling can step over a close pass: with 10 samples per leg a
    fingertip moves 8-10 cm between samples, and a true 1.5 cm pass past the table edge then reads ~5.2 cm (sampled minimum
    ~ sqrt(d^2 + (step/2)^2)). So every path in blocks 1 and 2 is built in two steps:
    - the **even samples** that define the path's shape (the `move` line every 5th tick; for a corner ordering its two legs, each
      a joint-space line);
    - then `densify(path, step_of, max_step=MAX_SAMPLE_STEP_M)` (pure function, no MuJoCo inside): for every pair of consecutive
      samples whose `step_of(qa, qb)` exceeds `max_step`, insert the joint-space midpoint, and repeat until every pair is within
      `max_step`; more than `MAX_HALVINGS` halvings of one interval raises `RuntimeError("87: path could not be densified - a joint
      jumps; check the path inputs")` (fail loudly, never skip).
    - `MAX_SAMPLE_STEP_M = 0.02` (named constant; source: round-4 review R4-1 - for a convex edge the sampled minimum overstates the
      true one by sqrt(d^2 + 0.01^2) - d, about 1 mm at d = 5 cm and at most 1 cm as d -> 0, so the 5 cm margin stays for pose error
      and lag). `MAX_HALVINGS = 10` (named constant: 2^10 = 1024 sub-steps per interval; a smooth arm never needs more, so more means a
      bug).
    - `step_of(qa, qb)` in 87 = the largest movement of any arm/hand **collision** geom between the two poses, bounded per geom as
      `|xpos_b - xpos_a| + geom_rbound * rotation_angle(xmat_a, xmat_b)`: the centre shift plus how far the geom's far end can swing
      (a forearm capsule turning 0.4 rad moves its tip more than its centre). FK results are cached per pose, so each sample costs one
      `mj_forward`, and every inserted pose is also a distance sample.
    - The report's `summary` carries `MAX_SAMPLE_STEP_M`, `max_sample_step_m` (the largest step actually used, must be <= the constant),
      `max_start_step_m` (R5-1, below; must be <= the constant), `start_poses` per waist limit and `samples` per block; 87 prints them.
  - **Start poses by arm movement, not by frame count (R5-1).** Picking every 10th demo frame left up to 11.8 cm of hand travel between
    two checked start poses (p90 5.5 cm, all fast windows with the hand over the table; round-5 FK on `replay_demo.npz`), so a fault
    from an unchecked pose between them could pass the edge several cm closer than any checked row, and `frames_inside` could miss a
    frame that starts inside the table top. So block 2's start poses come from a pure function (no MuJoCo inside, same `step_of` as
    `densify`):
    `select_start_poses(frames, step_of, max_step=MAX_SAMPLE_STEP_M) -> (poses, max_start_step_m)` walks the demo frames in order and
    keeps frame 0; for each next frame it keeps the previous frame whenever `step_of(last_kept, frame)` would exceed `max_step` (so
    every skipped frame lies within `max_step` of the pose kept before it); where two consecutive raw frames are already farther apart
    than `max_step` (up to 2.6 cm per frame measured), it inserts joint-space interpolated poses between them (`densify` of that pair);
    the last frame is always kept. `max_start_step_m` = the largest `step_of` between consecutive kept poses and between any skipped
    frame and the pose kept before it. It runs **per waist limit L on the clipped 17-vectors** (clipping changes the poses). Each start
    pose carries `frame` = its demo frame index (fractional for an interpolated pose). Expected ~86 start poses (whole-demo hand path
    1.70 m at 2 cm) instead of 73, so the runtime estimate stays.
  - **Block 1, `ready_paths` (covers ready <-> hanging only, R2-1 (a)):** `real_safety.move(h, BALL_READY_POSE arms)`, the exact `to_ready`
    line of `ready`; `to_start` and `release` from the ready pose run the same segment backwards. It does **not** cover exits that start
    from a policy pose; block 2 does.
  - **Block 2, `policy_exits` (R2-1 (b), R3-2):** for every waist limit L of `--waist-limits` (`WAIST_LIMITS`, named constant: the values
    Monday's M3 escalation can reach - 0.15, 0.35, 0.6 - plus 0.0 for `--waist-hold`, whose motor 12 stays at q0, about 0 by the bounds
    line), start poses = `select_start_poses` over the frames of `--demo` (arms = `state[0:14]`, waist yaw = `state[26]` clipped to
    +-L, as `SafetyFilter` clips it, rule 6b; R5-1). Two exits per start pose, each densified as above (R4-1):
    - `to_ready`: `real_safety.move(start, ready)` with waist -> 0 (the soft exit's first leg, case 3 of R2-1; our own command at weight 1,
      so the straight line is the path);
    - `to_hanging` (case 1, the fault ramp-out; `86 release` after a crash, case 2, moves our own straight line at 0.3 rad/s and is the
      `straight` ordering): three paths from start to h(e, s) for every (e, s) of the grid, and the worst of the three counts (R3-7):
      `straight` (the joint line; even samples = the ramp-out's 100 ticks, every 5th); `proximal_first` (waist yaw and the three
      shoulder joints of each arm, motors 12, 15-17, 22-24, reach their goal first while elbow and wrists stay, then elbow and wrists
      move; even samples 10 + 10); `distal_first` (the reverse). All three are then densified to `MAX_SAMPLE_STEP_M` (R4-1: the even
      corner legs alone move the fingertip 8-10 cm per sample). The two corner paths bracket how a torque-level blend with unknown
      factory gains bends the path (assumption, section 1).
    - Setbacks for block 2: 0 and each distinct `setback_needed_m` of block 1 (at most a handful), not the full setback grid (runtime).
    - A start pose already computed for a smaller L is reused when clipping to the larger L leaves it unchanged (plain dict keyed by the
      rounded 17-vector; most demo frames turn less than 0.6 rad, so the 0.6 column costs little).
    - Pass rule per object (table, stand, basket): along the exit the clearance never drops below `min(CLEARANCE_MIN_M, clearance of that
      object at the start pose)` - "the exit never brings the arm closer to an object than 5 cm, or than it already was". Needed because a
      grasp pose may start 1-2 cm from the stand; a plain 5 cm rule would fail every grasp frame.
    - `frames_inside` (R3-10 (a), R5-1: counted over the movement-spaced start poses, so a pose between two old 10-frame checks
      counts too): per (setback, L) and object, the number of start poses with negative clearance (the replay itself
      would press into that object - the nominal-vs-episode layout difference). M4 at the table needs the **table** count 0 (rule 16 (c));
      stand and basket counts are printed (a light-stand brush is not an abort, section 7 M4).
  - Distance (R3-8): only **collision** geoms (`contype` or `conaffinity` non-zero) of the arm and hand bodies (15-28 chain + Revo2) and of
    the table, the stand and the basket (the planner's probe found 53 and 22). Per sample: `mj_forward`, then a numpy bounding-sphere test
    (`data.geom_xpos`, `model.geom_rbound`: centre distance minus both radii) over all pairs, and `mujoco.mj_geomDistance` only for pairs
    within `distmax`; minimum per object. Setback = the table, stand and basket moved together by +x.
  - Runtime is a **reported number, not a done criterion** (R3-8: a slow 87 costs time, a fast but wrong one costs safety). 87 prints
    the number of samples per block before it starts and the elapsed time at the end. Planner's timing probe on this Mac at the ready
    pose (`runs/planner_fk_bench.log`, 2026-10-18 17:49): `mj_forward` 70 us, forward + pre-filter + distance 182 us with 108 distance
    calls per sample. Before densifying, block 2 was about 73 frames x (51 grid lines x 60 samples + ~30 `to_ready` samples) = ~226k
    samples per (setback, L). Densifying (R4-1) adds samples only where the arm moves fast: a fault drop moves the fingertip up to
    ~0.8-1.0 m, i.e. ~40-50 samples per path at 2 cm instead of 20, so roughly 2-3x, ~0.5-0.7M samples per (setback, L), times 4 L (less
    the reuse) and up to ~4 setbacks: expected 10-25 min (an estimate from the probe, not measured); ~86 movement-spaced start poses
    instead of 73 (R5-1) add ~15 %, inside that range. **Fallback if a run exceeds 30 min (R5-1):** fewer setbacks per run, or one
    waist limit per run (`--waist-limits 0.15` etc., each to its own report), then `87 --merge runs/arm_path_clearance.json <parts>`
    into one report, recorded in `real-robot-integration.md`. `merge_reports` (pure function) refuses parts whose `MAX_SAMPLE_STEP_M`,
    `CLEARANCE_MIN_M`, `GRID_BOUNDS`, elbow/shoulder grid or demo file differ, and recomputes the summary from the merged rows.
    **Never** coarser start spacing, never a raised `MAX_SAMPLE_STEP_M`.
  - Report: `ready_paths.rows` = `{elbow, shoulder_pitch, setback_m, min_clearance_m, worst_pair, worst_tick}`; `policy_exits.rows` =
    `{frame, waist_limit, exit ("to_ready" | "to_hanging"), ordering ("straight" | "proximal_first" | "distal_first", the worst one for
    to_hanging), elbow, shoulder_pitch, setback_m, start_clearance_m {object: m}, min_clearance_m {object: m}, passed, worst_pair}`
    (`policy_exits` is `null` with a `reason` when `--demo` is absent); `summary` = `CLEARANCE_MIN_M` (0.05, named constant: 5 cm margin
    for real-vs-sim pose error and tracking lag), `MAX_SAMPLE_STEP_M`, `max_sample_step_m`, `samples` (R4-1), `max_start_step_m`,
    `start_poses` (R5-1), `GRID_BOUNDS`,
    `WAIST_LIMITS`, `BLEND_NOTE` ("to_hanging = worst of straight and two
    corner orderings; a bracket of the factory blend, not a proof"), `max_elbow_clear_at_nominal` (largest e that keeps >= 5 cm with
    setback 0 for every s in the grid, or null), `setback_needed_m` per elbow (smallest setback with >= 5 cm for every s, or null),
    `setback_max_keep_layout_m` (largest setback that keeps the whole stand on the table top), `setback_max_policy_m` (largest setback
    that keeps the nominal ball within the training `reach_m` maximum 0.52 m of the pelvis, R2-3; `reach_m` is read from
    `table_scene.SCENES["volleyball"]`, because the `volleyball_walk` scene does not carry it), `replay_exits_clear[setback][L]` (every
    `to_ready` exit passed), `frames_inside[setback][L]` = `{table, stand, basket}`, and per elbow, in the list `summary["elbows"]`
    (the container name E2's lookup test reads; E2 review r1 minor 2), rows `{elbow, table_steps ("nominal" | "setback" | "skip"),
    setback_m, fault_exits_clear {L: bool}}` (every `to_hanging` exit, worst ordering, for that e and every s passed, at that elbow's
    setback). `table_steps` is "setback" only when `setback_m <= setback_max_keep_layout_m`, else "skip" (E2 review r1 minor 1, rule
    16 (c)).
  - Prints one line per elbow and waist limit: `elbow 0.9 waist 0.35: table at nominal | move the table back 3 cm | skip table steps;
    replay exits clear | NOT clear; fault exits clear | NOT clear; frames inside the table 0`, then the elapsed time.

### 3.6 Track E1: helpers, 89, setup
- `tests/helpers/loopback_lowstate_pub.py --iface lo0|lo --seconds S [--hz 200] [--hands]`: publishes LowState_ with every q = 0 (inside
  `JOINT_LIMITS`) and, with `--hands`, `rt/brainco/*/state` at 100 Hz; refuses a non-loopback iface and a present 192.168.123.x address,
  re-checked every second; after `ChannelFactoryInitialize` asserts the bound interface is loopback.
  Its refusal test fakes `ChannelFactoryInitialize` and the publisher class (R3-11).
- `scripts/setup_robot_laptop.sh [--robot-env] [--policy-env] [--release TAG | --release-file PATH] [--prefix DIR] [--show-ip]`
  (written out here because revision 5 was never committed, R3-6). Default prefix `~/g1-robot-day`. Each block prints
  `SETUP <block>: OK <info>` or `SETUP <block>: FAILED <what to do>`, is safe to re-run, and talks to no robot. **Outputs other parts
  rely on:**
  - `--robot-env`: Python 3.10 venv `<prefix>/robot-env` (uv; pins of 2.7, `unitree_sdk2py` at `65691c8`) and `<prefix>/robot-env.sh`
    (exports `ROBOT_DAY_PREFIX=<prefix>`, activates the venv). Used by 84, 86, 89 and the loopback helper.
  - `--release TAG` / `--release-file PATH`: sha256 check against `<tar>.sha256`, unpack into a temp folder, check
    `pretrained_model/{config.json, model.safetensors, policy_preprocessor.json, policy_postprocessor.json}`, then move it to
    `<prefix>/smolvla-runs/run1/checkpoints/040000/` (the tar's top level is `pretrained_model/`, so the setup creates this folder).
  - `--policy-env`: Python 3.12 venv `<prefix>/policy-env` (lerobot 0.6.1, torch 2.11.0; Ubuntu `--torch-backend auto`); downloads
    `lerobot/smolvla_base` and `HuggingFaceTB/SmolVLM2-500M-Video-Instruct` at the revisions in Finnegan's cache into
    **`<prefix>/hf_cache/hub`** (R3-1); writes `<prefix>/policy-env.sh` (exports `ROBOT_DAY_PREFIX`, `HF_HUB_CACHE=<prefix>/hf_cache/hub`,
    `HF_HUB_OFFLINE=1`, activates the venv).
  - `--show-ip`: lists IPv4 addresses per adapter (the robot adapter must show 192.168.123.222).
- `scripts/89_robot_day_selftest.py [--policy-device mps|cpu|cuda] [--skip-policy]` (robot env; prefix from `ROBOT_DAY_PREFIX`):
  installs the publisher guard first, then the checks `versions` (Python 3.10 and the pins), `dds` (refused while a 192.168.123.x address
  exists; starts the loopback helper with `--hands` and reads LowState + hand state through `real_io`'s readers), `camera`
  (`fake_camera.py` through `real_camera.FrameStream`), `bringup` (`84 --dry-run`), `imports` (no motion module loaded), `policy`
  (unless `--skip-policy`: 81 `--self-test tests/data/selftest_episode.npz` from `<prefix>/policy-env` with
  `HF_HUB_CACHE=<prefix>/hf_cache/hub`, `HF_HUB_OFFLINE=1`; **FAIL when `<prefix>/hf_cache/hub/models--HuggingFaceTB--SmolVLM2-500M-Video-Instruct`
  is missing** ("backbone not in the prefix - re-run setup_robot_laptop.sh --policy-env"), and the PASS line names that cache path, R3-1).
  Prints `CHECK <name>: PASS|FAIL|SKIP <reason>` per check, writes `runs/robot_day_selftest_<stamp>.json`, ends with `SELFTEST: PASS|FAIL`.

## 4. Safety rules (B1 implements the logic, B2 applies it, C proves it in sim)
### 4.1 Read-only rules (84, 89, 71)
1. **No publisher in anything Track A/E ships:** (a) static test over `real_io`, `real_camera`, 84, 89, 71 for `ChannelPublisher`,
   `DataWriter`, `Publisher(`, `.Write(`, `Damp`, `ZeroTorque`, `ReleaseMode`, `MotionSwitcher`, `run_g1_server`, `LocoClient` (ast + text)
   **[mutation]**; (b) import check: `84 --dry-run` in a subprocess loads no `real_send`, `real_run`, `motion_gate`, `brainco_env`, `lerobot`,
   `g1sim.teleop`, and (R4-2, B2's splits) no `real_preflight`, `real_run_log`, `real_run_modes`, `real_run_policy` - the same list in
   A's `MOTION_MODULES` (`test_unit_no_publisher.py`, Track A2) and 89's `FORBIDDEN_MODULES` (Track E2); (c) start-up guard in 84 and 89 **[mutation]**; (d) `own_writers == 0` at the end of every 84 run, and 85 `--no-blend`
   reports 0 `rt/arm_sdk` samples while 84 runs (integration).
2. 84 preflight: LowState older than 0.1 s or below `refuse` -> FAIL; below `warn` -> warning; foreign samples are always recorded;
   on `rt/arm_sdk` they FAIL the `probe` step (Part B abort) unless the label starts with `owner_motion`, on the hand-command topics
   they warn (R3-10 (c)); `--record-factory-pose` refuses while `rt/arm_sdk` samples arrive (R3-9).
3. **Head camera (Finnegan's facts, walk review RV-3/RV-4):** nothing we ship commands the head (no head/xl330/dynamixel code in 84, 86,
   `real_*.py`, `motion_gate.py`). The owner sets the head with its own control to straight ahead (pan 0) and 50 deg down; operators
   never move it by hand (a powered servo can strip, or returns to its own setpoint) and only verify it: phone flat on the D455 top
   face reads 49-51 deg and 84's overlay shows the table centred left-right. **After Part B 3 nothing may move the head:** the owner
   turns off whatever can move it (head tracking, behaviours, apps). `84 --steps camera` (the Part B 3 line) is **re-run before M4,
   before M5, and after any mode change or reboot**; if the overlay differs from the Part B one, the owner re-sets the head and the
   check is repeated; still different -> note it, no M4-M6 data is recorded as "nominal view" (the policy would act on an unseen view).
   A frame-vs-overlay check inside 86 is not built tonight (next session).
### 4.2 Motion rules (86)
1. **Never** Damp, ZeroTorque, ReleaseMode, `rt/lowcmd` as a publisher topic, `run_g1_server`, LocoClient. Static test over `real_safety`,
   `real_send`, `real_run`, `motion_gate`, 86 **[mutation]**. `real_send` is the only file allowed `ChannelPublisher`.
2. **Preflight refuses (exit 2)** before 86 creates any writer: gate problems (4.3); LowState older than 0.1 s or below `refuse`
   (200 Hz robot / 100 Hz loopback); **samples** on `rt/arm_sdk` within 2 s (`active_writers`) - samples on `rt/lowcmd` only warn and are
   logged (review M4, n3); samples on `rt/brainco/<side>/cmd` warn and are logged as `foreign_hand_cmd` ("another program sends hand
   commands - ask the owner to stop their hand tool, or run with --no-hands", R3-10 (c)); any measured SDK motor outside `JOINT_LIMITS`; no factory record (`load_factory_pose` None) -> "record the
   factory pose first (runbook M0)", or `factory_pose_gap > 0.10` -> the message of section 3.4 (`86 release` while our last run may
   hold the arms, otherwise M0 re-record; R2-2) (except mode `release`); (replay/shadow/run) server hello not `chunk_size 50`,
   `action_dim 27`, `image_shape [200, 320, 3]` or wrong server mode; (shadow/run) no camera frame younger than 0.5 s or hand state missing.
3. **Ramp in:** q0 = measured angles read **once** at the first ramp tick, commanded for the whole ramp, weight 0 -> 1 in 2 s (review N1).
4. **13/14 held** at q0 with `SIM_GAINS` in every message while weight > 0.
5. **Limits by phase:** hard clip (limits - 0.05) + rate limit in every phase except the fault ramp-out (it holds the measured angles
   unfiltered so a blocked arm stops pushing, rule 12; 13/14 stay at q0 there too, B2 review r1 minor 2); demo envelope only in `policy` (review B1); 30 clipped ticks in
   a row -> soft exit `envelope_streak`; in `shadow` the envelope is only evaluated on the logged policy actions.
6. **Speed:** scripted moves 0.3 rad/s; command rate <= 1.0 rad/s arms, 0.3 rad/s waist; speed factor <= 0.5 off loopback (gate e).
   6b. **Waist:** `--waist-max` (robot default 0.15, refused above 0.6) or `--waist-hold` (motor 12 at q0, policy waist logged, not sent).
7. **Lead limit:** |command - measured| > 0.25 rad for 13 ticks -> contact fault, **only while weight == 1.0** (review N2).
8. **Gains** = `SIM_GAINS`; `fill_arm_sdk` refuses anything higher.
9. **Hands:** targets clipped to [0, 0.6], speed 0.5; `hands-open` sends 0 at speed 0.3 for 2 s (no arm_sdk).
10. **Stale:** LowState > 0.1 s -> fault exit; camera > 1.0 s, hand state > 0.5 s (shadow/run only; replay warns and fills the hand slots
    with the last hand command, `hand_state_synthetic`, m6-6), no new chunk 1.0 s after the current one ran out, policy error or socket
    timeout 2 s -> soft exit.
11. **NaN/Inf** in a chunk -> fault exit before anything is sent.
12. **Exits.** *Soft* (normal end, first Ctrl+C/SIGTERM/SIGHUP, stale camera/policy/hand, envelope streak, chunk too late):
    `soft_exit_plan` - from the `policy` phase the arms first move to `BALL_READY_POSE` at 0.3 rad/s (RV6-1: never straight from a pose
    near the ball/table down to q0), **hands opened actively** there (0 at speed 0.3), then down to q0 at 0.3 rad/s, then ramp out; from
    `to_ready`/`hold`/`shadow` they go to q0 directly. The path always completes (no time cap, m6-9); a second signal turns it into a fault exit. *Fault* (contact, NaN, second signal, stale LowState):
    ramp out at once on `fault_targets` (measured angles; last command only for stale LowState); hands hold. Signals set a flag; a third
    signal does not skip the ramp; a `finally` block ramps out on any exception. No exit path depends on the terminal: after
    SIGHUP (terminal closed) or a broken pipe every print raises OSError, so 86 sends every operator message through one `say()`
    that keeps an unprintable message in the sidecar (`unprinted_messages`) instead of raising (B2 review r1 #2). **A fault in the `policy` phase at the table is not
    steered away from the table** (R2-1): the ramp-out hands the arm to the factory controller at the pose the policy left it in, and the
    arm follows roughly the straight line to the factory pose (assumption, section 1). 87's `policy_exits` block checks that line from
    demo poses; rule 16 and ruling R15 decide what Monday does with the result. A "fault hold" (keep w = 1 at the measured angles until
    the operator confirms, then leave through the ready pose) would remove this case; it is new motion logic and goes to the next
    session (section 10, R15).
13. **Release** (`86 release`, the first action after any crash, cable pull or kill): `release_plan(measured, factory)`: weight 1 at the
    measured angles (jump-free whether or not Unitree latched), move to the recorded factory pose at 0.3 rad/s, then ramp out. **No factory
    record -> refused** (ruling R12); the runbook box sends the operators to the owner's remote. Release from a policy pose (a crash, kill
    or cable pull during M4-M6) follows the straight line to the factory pose, which may cross the table edge (R2-1 case 2), so the runbook
    pulls the table back first (rule 16 (e)).
14. **Chunk timing:** 30 Hz x speed, linear interpolation at 50 Hz, request after 25 actions, latency skip, d >= 50 -> soft exit.
15. **Process:** run with the charger on and the lid open (never close the lid); macOS: `caffeinate -i python ...`; Ubuntu (m6-10):
    `systemd-inhibit --what=sleep:idle:handle-lid-switch python ...`; 85's report gate: largest gap between arm_sdk messages < 60 ms.
16. **Table clearance (RV6-1, R2-1, R2-3):** the M0 FACTORY ARMS line is looked up in the runbook table built from
    `runs/arm_path_clearance.json`:
    - (a) **Bounds:** if any shoulder roll, shoulder yaw, wrist angle or the waist yaw is more than `FACTORY_LOOKUP_TOL` (0.15 rad) from
      0, a shoulder pitch is outside -0.1..0.3 or an elbow outside 0.0..1.6 (`GRID_BOUNDS`), 87's grid does not describe this robot: skip
      M4-M6 at the table. (Track C code review B1, 2026-10-27) The same holds if the M0 q0 waist pitch is above
      `GRID_BOUNDS["waist_pitch_q0_max"]` = 0.041 rad (= `WAIST_PITCH_CHECKED` 0.126 - `WAIST_PITCH_SAG` 0.085: a larger q0 plus the
      sim's sag leans the torso further forward than 87 checked, and clearance falls with the pitch) or the M0 q0 waist roll is more
      than `WAIST_ROLL_TOL` = 0.02 rad from 0 (87 checks roll 0; FK probe: roll +-0.05 costs 0.4-0.6 cm, so +-0.02 costs <= ~0.25 cm
      of the 5 cm margin). Evidence: runs/real_c_b1_waist_probe.txt. A smaller or negative q0 pitch only adds clearance (probe down to
      -0.1 rad).
    - (b) **Waist column (R3-2):** the lookup has one column per checked waist limit (`--waist-hold`, 0.15, 0.35, 0.6). M4 and M6 at
      the table run only with one of these four settings, never above the limit reached in M3, and read that column. If that column says
      "no", a lower checked limit whose column says "yes" may be used (if M3 did not run that setting, one in-air replay with it comes
      first); otherwise skip.
    - (b2) **Two rows, not one (R4-6):** nothing pins that clearance falls monotonically with the elbow (a fault drop is a 3-D sweep
      past an edge), so the larger factory elbow is read in **both neighbouring rows** of the 0.1 grid (floor and ceil, e.g. 0.93 ->
      rows 0.9 and 1.0; exactly on a row -> that row only), and a cell counts as "yes" only if both rows say yes; the table position is
      the larger setback of the two rows.
    - (c) **M4 (replay at the table)** needs, in that column: the larger factory elbow's `table_steps` = "nominal" or "setback" (with the
      table moved back by `setback_m` <= `setback_max_keep_layout_m`), `replay_exits_clear` at that setback (every soft exit from a demo
      frame reaches the ready pose with clearance), `frames_inside.table` = 0 at that setback (no demo frame starts inside the table top,
      R3-10 (a)) **and**, by default (ruling R15, stricter since rev 6.3, R3-10 (b)), `fault_exits_clear` for that elbow **and** (B1
      review P1) the largest lead per joint that 86 logged during the M3 in-air replay at that waist setting stays below
      `M3_LEAD_MAX` = 0.15 rad (runbook constant; source: B1 review P1 - well under `LEAD_LIMIT` 0.25, so a sagging real arm is found in
      the air, not as a contact fault at the table; the 5 cm margin assumes small lag). The lead comes from the M3 log
      (`sent_q` vs `measured_q`, policy phase, weight 1); 86 prints it as `MAX LEAD <rad> (<joint name>)` before its final line in
      replay/run (Track B2, arrives as a review finding). If the build B2 ships lacks that line, the runbook uses the M2 `hold` lead
      (printed once per second, abort > 0.10) as the stand-in and says so.
    - (d) **M6 (VLA live at the table)** needs (c) **and** `setback_m` <= `setback_max_policy_m` (else the ball is out of the policy's
      reach and M6 is not a meaningful attempt). Without `fault_exits_clear`, M4 and M6 at the table are skipped (ruling R15 default);
      M5 `shadow`, M3 and M6 in the air are unaffected.
    - (e) **After a crash, kill or cable pull during M4-M6** (86 printed no final `86 <MODE>:` line **or** printed `86 <MODE>: FAULT
      EXIT lowstate_stale` (R5-3: the typical signature of a pulled or flaky cable - the weight-0 ramp-out may never have reached the
      robot), so our last command may still hold the arms over the table): before `86 release`, the owner decides whether the table
      (with stand and basket) can be pulled straight back away from the robot; if yes, one person pulls it back by its far edge (never
      reaching between the arms and the table), then `86 release`. If not, the owner's remote (existing box). After any **other** exit 4
      the ramp-out has already handed the arms to the factory controller (weight 0 sent), so `86 release` should move nothing; after an
      exit 3 the arms are already at q0. In every case note whether the arm touched the table on the way down (that is the R15 finding)
      and run `86 hands-open` if the fingers are closed.
    - (f) **The ball (R4-5):** a fault while the ball is lifted may press the ball onto the table edge between the descending hands; 87
      does not model the ball. The runbook names it, and it counts as "contact with the table" (abort, note it as an R15 finding).
    - Until 87 has run with `--demo` and its numbers are in the runbook, M4-M6 at the table are not cleared.
### 4.3 Motion gate for non-author operators (B2, `motion_gate.py`; every 86 mode except `gate` runs it after all imports)
- (a) `docs/ROBOT_DAY_MOTION.json` is **committed in HEAD** (`git cat-file -e HEAD:docs/ROBOT_DAY_MOTION.json`; R3-3: `git diff --quiet`
  alone exits 0 for an untracked, hand-made file) and unmodified (`git diff --quiet HEAD -- docs/ROBOT_DAY_MOTION.json`); the gate parses
  the JSON **from `git show HEAD:docs/ROBOT_DAY_MOTION.json`**, never from the working tree. Content:
  `{"motion_allowed": true, "commit": "<40-hex>", "unlocked_by": "finneganadam", "date": ...}`; the pinned commit is an ancestor of HEAD
  (`git merge-base --is-ancestor`), and its `date` equals today's local date (m6-11, ruling R13: an unlock never carries over to a later
  session; the message says "the unlock is for <date>; only Finnegan can unlock today"). Only Finnegan writes it, after the final gate is GO.
- (b) every file from `loaded_repo_files()` exists in the pinned commit (`git cat-file -e <commit>:<path>`) and is unchanged
  (`git diff --quiet <commit> -- <path>`). 86 imports every module at the top (no lazy imports); the e2e checks `loaded_files_at_exit` ==
  `gate.loaded_files` (review R4-1). This guards against accidents, not misuse (said in the runbook).
- (c) both operators type their GitHub names (two distinct names from `OPERATORS`) and `START`.
- (d) the operator types `HANGING` to confirm the robot is secured in the gantry and the owner holds the remote - for **every** 86 motion
  command (review R4-2). "Hangs in the gantry" is defined (R2-5) as: the robot **stands on its own feet** under the owner's controller in
  the agreed arm_sdk mode, and its harness is attached to the gantry with only a few cm of slack, so a damp or collapse is caught within
  a few cm. The operators type HANGING only for that state (not for a robot lifted off the floor, and not for a harness with a long slack).
- (e) `--speed` <= 0.5 and `--waist-max` <= 0.6 off loopback.
- (f) no kill test by non-authors: 86 has no kill mode; the runbook says "press Ctrl+C once; never kill -9 or close the terminal".
- Sim bypass: (a)-(d) are skipped only when `sim_bypass_problem()` is None (loopback iface, `--yes-sim`, no 192.168.123.x address); the
  sidecar records `gate: "sim"`.
- **Finnegan's unlock (after 00:00 on 2026-10-19 local time, because gate (a) compares the date; only after a final-gate GO):**
  ```
  git add <motion files listed by the final gate>; git commit -m "Robot day motion code (reviewed, final gate GO)"
  git status --porcelain -- g1sim scripts tests   # must print nothing (m6-11, R4-2, R5-5: every loaded code file, split and test file is committed)
  MOTION=$(git rev-parse HEAD)
  printf '{"motion_allowed": true, "commit": "%s", "unlocked_by": "finneganadam", "date": "2026-10-19"}\n' "$MOTION" > docs/ROBOT_DAY_MOTION.json
  git add docs/ROBOT_DAY_MOTION.json; git commit -m "Unlock motion for the robot day 2026-10-19"
  # R4-2: prove the COMMIT, not this working tree - a clean local clone has no untracked file to hide a missing one
  CLONE=$(mktemp -d)/repo; git clone -q . "$CLONE"
  (cd "$CLONE" && ~/dev/g1-ext/.venv/bin/python scripts/86_real_run.py gate) > runs/push2_clean_gate.log 2>&1; echo "exit $?"   # must be 0 and print "MOTION: UNLOCKED"
  (cd "$CLONE" && source ~/dev/g1-ext/robot-day-test/robot-env.sh && python scripts/86_real_run.py gate) > runs/push2_clean_gate_py310.log 2>&1; echo "exit $?"   # same, operators' Python 3.10
  git push
  ```
  The final gate's motion-file list is the frontmatter's B1, B2 and C files plus every split the tracks reported (push-set rule,
  frontmatter). 86 loads Track A's `real_io`, so an uncommitted Track A2 edit makes `git status` print and gate (b) refuse: A2 (and E2)
  must be GO and committed before the unlock, or Finnegan sets their edits aside on a side branch first (his call, never an agent's).
  Any later commit that changes a loaded file locks motion again until he re-pins.

## 5. Build tracks
Five tracks build in parallel: A, B1, B2, C, E1. Their files are disjoint. Serialised after them (rev 7.1): AF and RB (push-1 fix
set), E2 (after E1; fix round after RB), A2 (after the fix-set commit). Actual starts (STATUS, R4-3): E1 17:38 and A 17:43 (rev
6.2; rev-6.3 items came in their fix rounds; both GO 18:13); B2 17:47 and B1 17:49, **also on rev 6.2** - B2's rev-6.3 items (R3-3
gate (a) via `cat-file -e` + `git show HEAD:`, R3-5 `--out` rule, R3-10 (c) `foreign_hand_cmd`) and B1's (none) reach them as
review findings (seam 3 below); C starts on rev 7.1 (R4-1 and R5-1 in its task text) and is reviewed against rev 7.1.
**E2 starts only after E1's review is GO** (started 18:15) and edits only files E1 owns (the runbook, 89) plus one new image, so it never
runs in parallel with E1 (RV6-3); its rev-7 items (R4-4 page-1 rule, R4-5 ball line, R4-6 two rows, B1-P1 lead line, 89's forbidden
list) reach it as review findings. **Push-1 fix set (R5-2), serialised, one writer per file at a time:** Track AF (A follow-up,
the only writer of `real_io.py`, 84 and `test_unit_real_io.py` since A's GO; built 18:20-18:37) and Track RB (runbook A+B fixes, then
the RV-1..RV-8 fixes and the loopback publisher's hand-deadline line; started 18:40) each pass their own gate (section 0); **the runbook
has one writer at a time: RB, then E2's fix round, then nobody until the push-1 commit** (afterwards E2, then the integration phase,
in that order). A's files are otherwise frozen until push 1. **A2 starts only after the push-1 fix-set commit exists** (so the pushed
state is the reviewed one) and never runs in parallel with another writer of A's files. **Push-set rule (R4-2):** a track that splits a file (the
~300-line rule) names the new file in its STATUS line; that file joins the track's push set and the final gate's list, and the planner
adds it to the frontmatter at the next revision. Each track
codes against section 3 and uses fakes for other tracks' files; a verify step that needs another track's file is marked "pending
integration" and runs in section 6.2/6.3. Each track's reviewer writes its own findings file `.planning/reviews/real-robot-code-<track>.md`
(m6-12). Every track writes a start and a finish line to `.planning/STATUS.md` (clock from `date "+%Y-%m-%d %H:%M"`).

<task id="A" track="A" push="1" parallel_with="B1,B2,C,E1">
<name>Track A: read-only bring-up - real_io readers + factory-pose record, real_camera, 84 checklist, no-publisher enforcement</name>
<files>g1sim/real_io.py, g1sim/real_factory_pose.py, g1sim/real_bringup.py, g1sim/real_camera.py, scripts/84_real_bringup.py, tests/helpers/fake_camera.py, tests/helpers/real_fakes.py, tests/test_unit_real_io.py, tests/test_unit_real_camera.py, tests/test_unit_no_publisher.py, tests/test_unit_real_factory_pose.py, tests/test_unit_real_bringup.py</files>
<action>
Interface: sections 3.1, 3.2, 4.1. Write `real_io.py` first (other tracks import its constants and `robot_net_present`).
Started 17:43 on rev 6.2; the rev-6.3 items marked (R3-x) below arrived through A's review and fix round. **Status: review GO round 2
(18:13), committed locally as c0898cf (18:15).** The crash and hand-rate fixes found by the push-1 DDS run are Track AF (push 1);
remaining A findings go to Track A2.
1. `real_io.py` (+ its split `real_factory_pose.py`), `real_camera.py`, `fake_camera.py` as the contracts. Readers use KEEP_LAST 1 QoS.
   (R3-5) `RUN_LOG_DIR`, `BRINGUP_REPORT_DIR` and `factory_pose_path` are absolute paths under `REPO`; (R3-10 (c)) `active_writers` also
   counts `HAND_CMD_TOPICS`; (R3-2) `factory_arm_summary` adds the waist yaw.
2. `scripts/84_real_bringup.py` (robot env): `install_publisher_guard()` first; steps write PASS / FAIL / SKIP / NOT RUN into
   `BRINGUP_REPORT_DIR/real_bringup_<stamp>.json` and print `STEP <name>: PASS|FAIL ... <reason>`; exit 0 only if no step FAILs. Flags
   `--iface`, `--steps`, `--dry-run`, `--sim`, `--yes-sim` (loopback only), `--camera-cmd`, `--robot-pc`, `--camera-device`,
   `--policy HOST:PORT`, `--seconds`, `--label`, `--record-factory-pose`, `--mode-note TEXT`, `--overwrite-factory-pose`. Steps (written
   out here because revision 5 was never committed, R3-6):
   - `net`: the `--iface` adapter has 192.168.123.222/24 (else "set the static IP, runbook Part B step 0") and PC2 192.168.123.164
     answers one ping (`72_real_check.py:10`; a ping failure is a warning, not a FAIL - the motion computer's LowState in `probe` is the
     proof that matters).
   - `probe`: 71 as a subprocess (LowState rate, `lowstate_hz_check`) + `active_writers` (prints samples per topic; foreign samples on
     `rt/arm_sdk` FAIL unless the label starts with `owner_motion`; hand-command samples warn, R3-10 (c)) + `own_writers == 0`.
   - `joints`: `joint_rows` with limits, `mirror_gap`; always prints `FACTORY ARMS: <factory_arm_summary>` and stores q17 in the report.
     With `--record-factory-pose --mode-note "<Q8 answer>"`: (R3-9) first refuses while `active_writers` sees `rt/arm_sdk` samples within 2 s
     ("another program steers the arms right now - its pose is not the factory pose; ask the owner to stop it"); then compares q17 with
     `earliest_factory_arms_today()` and prints `PART B ARMS: gap <x> rad to the reading at HH:MM`, plus `WARNING: differs from the Part
     B reading by more than 0.10 rad - if the owner did not change the robot mode, the owner's tool may still hold the arms: the owner
     re-sets the robot (or confirms their tool ended at weight 0), then record again` when the gap exceeds `FACTORY_POSE_TOL` (a warning,
     because a mode change may legitimately move the hold pose, RV6-2); then `save_factory_pose(factory_pose_path(iface), ..., mode_note)`.
     A `FactoryPoseConflict` prints "a different factory pose was recorded today at HH:MM (mode: <old note>)." followed by only the way
     out that works now (R2-2 (iii)): when `arms_may_be_held_by_us()` is False, "if the owner changed the robot mode, re-run with
     --overwrite-factory-pose after the owner confirms the arms are in the controller's hold pose (runbook M0); otherwise ask the owner";
     when it is True, "our last 86 run may still hold the arms: run `86 release` first"; `--overwrite-factory-pose` is refused while
     `arms_may_be_held_by_us()` is True (section 3.1, R2-2).
   - `questions`: asks Q1-Q12 (section 9) one by one and stores the answers in the report.
   - `hand-state`: `HandReader` for both sides, rate and 6 values each, NaN-free.
   - `camera`: `real_camera.grab` via `--robot-pc USER@HOST --camera-device N` or `--camera-cmd`; framing (`cropped` flag), image stats
     vs `DATASET_IMAGE_STATS`, overlay with `docs/robot_day/reference_head_frame.png` saved to `runs/`; no FOV intrinsics sub-check (cut
     item 1 applied).
   - `record`: `--seconds` of LowState + hands + frames into a `g1sim.episode` npz (`commanded_by_us: false`, `source: real_observe`,
     `label`); `--policy` optional (cut item 2); a label that starts with `owner_motion` (e.g. `"owner_motion arm7 example, R2+A"`, r5-4)
     expects foreign samples.
3. Tests (each with Proves/Catches): `test_state27_maps_motors_into_policy_order`, `test_state27_marks_missing_hands_as_nan`,
   `test_joint_limits_match_the_mjcf`, `test_lowstate_hz_check_refuses_below_200_on_robot_and_100_on_loopback` [mutation],
   `test_active_writers_counts_samples_not_discovered_writers` [mutation], `test_factory_pose_record_refuses_silent_overwrite` [mutation],
   `test_factory_pose_overwrite_refused_while_our_last_run_may_hold_the_arms` [mutation: ignore arms_may_be_held_by_us] (RV6-2, R2-2;
   fake sidecars: unfinished `exit_code: null`, and finished with `last_weight_sent` 0.4),
   `test_factory_pose_overwrite_allowed_after_a_run_that_ended_at_weight_0` (R2-2 (iv); Catches the rev 6.1 dead end after a mode change),
   `test_sidecar_found_when_86_ran_from_another_cwd` [mutation: relative RUN_LOG_DIR] (R3-5; a sidecar with `exit_code: null` under
   `RUN_LOG_DIR` while the test's cwd is a temp folder -> `arms_may_be_held_by_us()` True; Catches the fail-open path where 84 finds no
   sidecar and allows an overwrite while our run holds the arms),
   `test_factory_pose_record_refused_while_a_foreign_arm_sdk_writer_sends` [mutation] (R3-9; fake `active_writers`; Catches recording an
   owner tool's pose as the factory pose), `test_factory_pose_record_warns_when_it_differs_from_the_part_b_reading` (R3-9; fake report
   with an earlier `joints` reading 0.3 rad away),
   `test_factory_arm_summary_lists_roll_yaw_wrist_and_waist` (R2-3, R3-2; Catches a summary that hides a factory shoulder roll or waist
   yaw the lookup must see),
   `test_factory_pose_record_stores_mode_note` (RV6-2), `test_factory_pose_paths_differ_for_robot_and_sim`, `test_to_policy_frame_flags_640x480_as_cropped`, `test_to_policy_frame_converts_bgr_to_rgb`
   [mutation], `test_grabber_parses_as_python_3_8`, `test_frame_bytes_round_trip_through_fake_camera`,
   `test_bringup_files_contain_no_publisher` [mutation], `test_bringup_dry_run_imports_no_motion_modules`,
   `test_publisher_guard_blocks_channel_publisher` [mutation] (R3-11: the test replaces `ChannelFactoryInitialize` and the real writer
   classes with fakes before installing the guard, so the mutated copy without the guard fails on the fake, never on a real DDS writer),
   `test_yes_sim_refused_off_loopback`, `test_record_output_reads_back_with_episode_load` (fake readers), `test_new_files_parse_as_python_3_10`.
   Added in A's fix round 1 (as built, names from the test files): `test_active_writers_listens_on_hand_command_topics_with_their_own_type`,
   `test_bringup_installs_publisher_guard_before_any_step`, `test_net_passes_with_a_warning_when_pc2_is_silent`,
   `test_probe_fails_on_stale_lowstate`, `test_probe_fails_on_foreign_arm_sdk_samples_unless_owner_motion`,
   `test_probe_warns_on_foreign_hand_commands`, `test_joints_fail_when_a_motor_is_outside_its_limit`, `test_hand_state_fails_on_nan_values`,
   `test_record_stops_when_lowstate_goes_stale`, `test_factory_pose_first_record_refused_while_our_last_run_may_hold_the_arms`,
   `test_newest_arm_writer_run_decides_and_refused_runs_are_skipped`, `test_factory_pose_record_refuses_an_empty_mode_note_on_the_robot`,
   `test_earliest_reading_ignores_sim_reports_on_the_robot` (in `test_unit_real_factory_pose.py` / `test_unit_real_bringup.py`, sharing
   `tests/helpers/real_fakes.py`).
4. Each test file starts with the `pytest.importorskip` lines of section 0 (m6-3).
</action>
<verify>
  $PY -m pytest tests/test_unit_real_io.py tests/test_unit_real_camera.py tests/test_unit_no_publisher.py tests/test_unit_real_factory_pose.py tests/test_unit_real_bringup.py -q > runs/real_a_unit.log 2>&1; echo "exit $?"
  $DDS $PY scripts/84_real_bringup.py --dry-run > runs/real_a_dryrun.log 2>&1; echo "exit $?"
  # pending integration (section 6.2 push-1 check): 84 against loopback_lowstate_pub from the fresh Python 3.10 robot env
</verify>
<done>Unit tests exit 0 with every [mutation] check seen failing first (runs/real_a_mutation.log); dry run exit 0 listing the steps; review GO - reached 18:13 (round 2).</done>
<effort>planned 4.5 h agent time; actual ~30 min build + two review/fix rounds (17:43-18:13)</effort>
</task>

<task id="AF" track="AF" push="1 (fix set)" depends_on="Track A GO; push-1 DDS run of 18:16-18:20">
<name>Track AF: Track A follow-up for push 1 - readers kept alive until close_dds at exit, 84 crash trail, rate_hz over listening time</name>
<files>g1sim/real_io.py, scripts/84_real_bringup.py, tests/test_unit_real_io.py</files>
<action>
Built 18:20-18:37 by the follow-up implementer (notes: `real-robot-code-A.md` "Track A - follow-up (push-1 check)"); this task pins
what it must contain so its reviewer has a contract (R5-2):
1. `real_io`: every reader channel stays referenced until `close_dds()`; `init_dds` registers `close_dds` with `atexit` (section 3.1).
   Why: a reader dropped by `step_hands`/`active_writers` and freed by the cyclic GC skips `dds_delete` in cyclonedds 0.10.2 and the next
   sample jumps into freed memory (SIGSEGV, 3 of 3 isolated runs). This also protects 86 (preflight counters on `rt/arm_sdk`) and 89.
2. `LatestReader.rate_hz` = gaps in the window / time from the first sample in the window until now (< 2 samples -> 0); `step_hands`
   waits for both hands, listens `HAND_RATE_LISTEN_S` = 1 s (named constant), then reads.
3. 84 `__main__`: `faulthandler.enable()` and line-buffered stdout, so a crash leaves every STEP line and a trace even into a file; the
   report is written before `close_dds` runs.
4. Tests: `test_a_dropped_reader_stays_subscribed` [mutation: no `_open_channels` list] and
   `test_rate_hz_of_a_young_reader_is_the_stream_rate` [mutation: divide by the full window] - both seen failing on the old code
   (`runs/real_a_fu_new_tests_on_old_code.log`); `test_joint_limits_match_the_mjcf` skips without mujoco (A review r2 minor 4).
Reviewer checks beyond the diff: `close_dds` is safe when DDS was never initialised (unit tests fake `ChannelFactoryInitialize`);
in 86 the `atexit` `close_dds` runs only after the `finally` ramp-out and after the policy worker thread is joined (B2 re-review
confirms against the working-tree `real_io`).
</action>
<verify>
  $PY -m pytest tests/test_unit_real_io.py tests/test_unit_real_camera.py tests/test_unit_no_publisher.py tests/test_unit_real_factory_pose.py tests/test_unit_real_bringup.py -q > runs/real_af_unit.log 2>&1; echo "exit $?"
  (source $P/robot-env.sh && python -m pytest tests/test_unit_real_io.py tests/test_unit_real_factory_pose.py tests/test_unit_real_bringup.py -q) > runs/real_af_unit_py310.log 2>&1; echo "exit $?"
  # DDS proof = 6.2's five 84 runs from the clean clone of the fix-set commit (after the section-0 DDS check)
</verify>
<done>Both unit runs exit 0; the two new tests seen failing on the old code; "Track A - follow-up review" in `real-robot-code-A.md` is GO; 6.2's five clean-clone 84 runs all exit 0 with `OWN WRITERS: 0` and no "slow hand state".</done>
<effort>built (17 min); review ~10 min</effort>
</task>

<task id="RB" track="RB" push="1 (fix set)" depends_on="E1 GO; Cristina's paper walk (18:22)">
<name>Track RB: runbook Parts A+B fixes for push 1 - dry-walk F1-F21 (done 18:33) and the re-review's RV-1..RV-8 (from 18:40)</name>
<files>docs/ROBOT_DAY_RUNBOOK.md (Part A, Part B, end of day only), tests/helpers/loopback_lowstate_pub.py (one line)</files>
<action>
The only runbook writer until its gate is GO; Part C and page 1 belong to E2's fix round (next writer). What RB must contain:
1. F1-F21 as listed in `real-robot-runbook-walk-AB.md` "Dry-walk A+B fixes" (done 18:33, fidelity GO by the re-review), incl. F21:
   table top = 0.74 m + (hip height - 0.648 m), hip height = floor to the hip pitch axis (0.7505 - 0.1027, re-derived twice).
2. RV-1: hip height measured with the robot standing in the mode it will have at the table in Part C (owner sets it now and keeps the
   stand height; a change -> measure again, re-set the table); plausible 0.60-0.72 m (else measure again, then ask the owner and write
   it down); never a table above 0.84 m; landmark = centre of the upper-most hip motor cap seen from the side.
3. RV-2: B3 also tapes both feet (toe and heel).
4. RV-3 + RV-4 (Finnegan's facts, rule 4.1 (3)): the head is motorized; the owner sets it (pan 0, 50 deg down) with its own control;
   operators never move it, only check (phone 49-51 deg, 84 overlay centred); after B3 nothing may move the head. The table stays on
   its tape marks; camera corrected only within 49-51 deg; a remaining offset is written down (R19 (a) default). The paper Q3 becomes
   the RV-3 head question (section 9); 84's own Q3 text follows with A2.
5. SSH: login details only on site from the owner, never written anywhere (already in F2); RV-7: `ssh -o ConnectTimeout=5 ...`.
6. RV-5, RV-6, RV-8 wording as the re-review proposes; the only 72 command named is `72_real_check.py kamera --suchen` (R5-4).
7. `loopback_lowstate_pub.py` `run_loop`: `next_hands += 1.0 / HAND_HZ` (deadline scheduling; the A follow-up measured 78 Hz instead
   of the printed 100 Hz).
</action>
<verify>
  $PY -m pytest tests/test_unit_runbook.py -q > runs/real_rb_unit.log 2>&1; echo "exit $?"
  git diff -- docs/ROBOT_DAY_RUNBOOK.md tests/helpers/loopback_lowstate_pub.py > runs/real_rb_diff.log 2>&1; echo "exit $?"   # input of the diff check
</verify>
<done>Runbook tests exit 0; the diff check of RV-1..RV-8 (appended to `real-robot-runbook-walk-AB.md` under "Review of dry-walk A+B fixes") is GO; E1's loopback tests still pass (`tests/test_unit_runbook.py` covers them).</done>
<effort>~10 min fix + ~5 min diff check</effort>
</task>

<task id="A2" track="A2" push="2-if-GO" depends_on="push-1 fix-set commit made (A's files frozen until then)">
<name>Track A2: read-only follow-ups to Track A - per-laptop Part B message, owner_motion label prefix, import-ban list, four test gaps, lowstate_stale as 'may be held', the walk's script proposals</name>
<files>g1sim/real_bringup.py, scripts/84_real_bringup.py, g1sim/real_factory_pose.py, g1sim/real_camera.py, scripts/setup_robot_laptop.sh, tests/test_unit_real_io.py, tests/test_unit_no_publisher.py, tests/test_unit_real_factory_pose.py, tests/test_unit_real_bringup.py, tests/test_unit_real_camera.py</files>
<action>
Small, read-only edits of Track A's files (no new module, no motion code). Each change from a named finding:
1. (R4-4) `part_b_lines(q17, None)` returns the 3.1 text "no Part B reading found on this laptop - M0 cannot compare; if Part B or an
   86 run happened on another laptop, stop and ask the owner to re-stand the robot first" (replaces "Part B step 0 was skipped").
2. (A review r2 minor 3) the `owner_motion` label is accepted when the label **starts with** `OWNER_MOTION_LABEL` (4.1 rule 2, section 7
   uses `"owner_motion <tool>, <mode>"`), in `real_bringup.py` and 84.
3. (R4-2) `MOTION_MODULES` in `test_unit_no_publisher.py` adds `g1sim.real_preflight`, `g1sim.real_run_log`, `g1sim.real_run_modes`,
   `g1sim.real_run_policy` (rule 4.1 (b)).
4. (A review r2 minor 4) `test_joint_limits_match_the_mjcf` starts with `pytest.importorskip("mujoco")` (the robot env has no mujoco).
5. (A review r2 minor 7) `test_sidecar_found_when_86_ran_from_another_cwd` never writes into the real `runs/real/` (a killed test would
   leave an `exit_code: null` sidecar that blocks M0 all day): it asserts `RUN_LOG_DIR.is_absolute()` and `== REPO / "runs" / "real"`,
   and any cwd case uses `tmp_path` [mutation: relative RUN_LOG_DIR].
6. (R5-3) `arms_may_be_held_by_us`: the newest arm-writer sidecar with `"exit_reason": "lowstate_stale"` counts as "may be held", like
   an unfinished run (3.1): with the link down our weight-0 ramp-out may never have reached the robot.
7. (R5-4, walk "needs code" 1, 2; RV-3) `real_bringup.QUESTIONS` and section 9 in step: Q2 adds "and which /dev/videoN is its colour
   stream?"; Q3 becomes "Head module: how is it set (app/remote), can it be set to pan 0 / 50 deg down and left there, and what else can
   move it (behaviours, apps)?"; Q5 adds "(never type the password here: answers are saved)". `check_camera`'s FAIL text names
   `python scripts/72_real_check.py kamera --suchen (runbook B3)`; `check_hands`' FAIL text drops the `72 hand --pruefen` hint (not a
   runbook command, rule 0).
8. (walk "needs code" 4) `setup_robot_laptop.sh` release download: check `gh --version` first and fail with "gh <ver> is too old (needs
   2.21+ for --clobber) - install gh from cli.github.com (runbook A1)".
9. (walk "needs code" 5) `real_camera`/84: when ssh's stderr contains `Permission denied`, `camera did not start` names the cause
   ("SSH login failed: password wrong or typed after 30 s").
   Not built: walk proposal 3 (`--robot-pc` for `72 kamera`): 72 is existing German code that section 0 keeps untouched; the runbook's
   fallback (try N = 0..7 with 84 `--robot-pc <user@IP>`) covers another user@IP.
Tests (each with Proves/Catches): `test_part_b_line_says_no_reading_on_this_laptop` [mutation: old text],
`test_lowstate_stale_exit_counts_as_arms_may_be_held` (R5-3; fake sidecar `exit_code` 4, `exit_reason` "lowstate_stale",
`last_weight_sent` 0.0 -> True; Catches the cable-pull case read as "free") [mutation: decide by `last_weight_sent` only],
`test_questions_ask_for_video_node_head_control_and_no_password` (R5-4),
`test_probe_accepts_owner_motion_label_with_tool_and_mode` (the plan's example label) [mutation: exact match],
`test_joints_step_refuses_record_while_foreign_arm_sdk_samples_arrive` (A review r2 minor 2: 84's `step_joints` with `active_writers`
monkeypatched to 40 rt/arm_sdk samples -> FAIL "another program steers the arms", no file written) [mutation: `writers = {}`],
`test_unreadable_newest_sidecar_counts_as_arms_may_be_held` (A review r2 minor 5, R2) [mutation: `continue` on broken JSON].
Clean code as section 0; files stay under ~300 lines.
</action>
<verify>
  $PY -m pytest tests/test_unit_real_io.py tests/test_unit_real_camera.py tests/test_unit_no_publisher.py tests/test_unit_real_factory_pose.py tests/test_unit_real_bringup.py -q > runs/real_a2_unit.log 2>&1; echo "exit $?"
  (source $P/robot-env.sh && python -m pytest tests/test_unit_real_io.py tests/test_unit_real_factory_pose.py tests/test_unit_real_bringup.py -q) > runs/real_a2_unit_py310.log 2>&1; echo "exit $?"
  $DDS $PY scripts/84_real_bringup.py --dry-run > runs/real_a2_dryrun.log 2>&1; echo "exit $?"
  bash -n scripts/setup_robot_laptop.sh > runs/real_a2_setup_syntax.log 2>&1; echo "exit $?"
  $PY -m pytest tests/test_unit_runbook.py -q > runs/real_a2_runbook.log 2>&1; echo "exit $?"   # the setup script's runbook tests
</verify>
<done>Unit tests exit 0 in the sim env and the Python 3.10 robot env; each [mutation] seen failing first (runs/real_a2_mutation.log); 84 dry run exit 0; review GO (findings file `.planning/reviews/real-robot-code-A2.md`); in push 2 only if the final gate is GO, else push 1's A stays.</done>
<effort>about 45-60 min agent time (rev 7.1 adds items 6-9)</effort>
</task>

<task id="B1" track="B1" push="1 as inert code (committed e914654, ruling R18); motion use only with push 2 if every gate is GO" parallel_with="A,B2,C,E1">
<name>Track B1: safety layer real_safety.py - pure logic for ramps, release, limits, envelope by phase, lead fault, staleness, chunk timing</name>
<files>g1sim/real_safety.py, g1sim/real_timing.py, tests/test_unit_real_safety.py, tests/test_unit_real_safety_filter.py, tests/test_unit_real_timing.py</files>
<action>
Interface: section 3.3 exactly (B2 codes against it in parallel). No DDS, no file or network I/O. Split helpers rather than exceed ~300 lines.
Tests (each with Proves/Catches): `test_ramp_in_holds_q0_and_raises_weight_in_steps_below_0_02`,
`test_release_moves_to_factory_pose_at_0_3_rad_s_before_weight_drops` [mutation: ramp straight from measured],
`test_release_first_message_is_full_weight_at_measured_angles`, `test_envelope_clips_only_in_policy_phase` [mutation: always clip],
`test_ball_ready_pose_lies_inside_envelope`, `test_hard_limits_clip_in_every_phase` [mutation], `test_rate_limit_caps_arm_and_waist_steps_in_every_phase` [mutation: skipped in policy],
`test_waist_roll_pitch_always_equal_q0` [mutation], `test_lead_fault_fires_after_13_ticks_only_at_full_weight` [mutation: count at w < 1],
`test_fault_targets_measured_on_contact_last_command_on_stale_lowstate` [mutation], `test_nan_target_raises_invalid_target_in_safety_filter` / `_in_chunk_scheduler` / `_in_hand_targets` (split in code review round 2),
`test_scheduler_starts_chunk_at_the_action_matching_its_latency` [mutation: start at arrival], `test_scheduler_refuses_chunk_that_lies_fully_in_the_past`, `test_stale_reason_thresholds` [mutation: 0.1 -> 1.0],
`test_speed_above_0_5_and_waist_above_0_6_refused_off_loopback` [mutation], `test_envelope_and_gains_pinned_to_stats_and_table_env`
(skipped with a reason if `$DATASET/meta/stats.json` is absent), `test_hand_targets_capped_at_0_6`,
`test_soft_exit_from_policy_passes_through_ready_pose` [mutation: straight to q0] (RV6-1),
`test_soft_exit_path_is_never_truncated` (m6-9: a 2.0 rad gap still ends exactly at q0), `test_hands_never_envelope_clipped` (m6-7).
Added in code review round 1: `test_ramp_in_target_stays_frozen_when_the_measured_buffer_changes` [mutation: alias q0],
`test_envelope_streak_fires_on_the_30th_clipped_policy_tick` [mutation], `test_filter_with_nan_waist_max_raises_instead_of_sending_nan`
[mutation], `test_fault_targets_fall_back_to_last_command_when_measured_is_nan` [mutation], `test_nan_speed_and_waist_max_refused` [mutation].
Added in code review round 2: `test_rate_limit_caps_arm_and_waist_steps_in_every_phase` now steps 5 messages per phase against the
literal 1.0/50 and 0.3/50 [mutation: limit only the first message; ARM_SPEED_MAX 5.0], `test_hard_limits_clip_in_every_phase` now
also downwards against the literal -/+0.45 [mutation: lower clip removed; lower margin outwards; LIMIT_MARGIN 0],
`test_waist_hold_keeps_motor_12_at_q0_in_every_phase` [mutation], `test_waist_max_clips_waist_yaw_target` [mutation],
`test_safety_numbers_equal_the_plan` (the 3.3 numbers as literals) [mutation: SCRIPTED_SPEED 3.0, LEAD_TICKS 50, LEAD_LIMIT 0.5],
`test_ramp_out_holds_target_and_lowers_weight_in_steps_below_0_02` [mutation], `test_nan_measured_angle_counts_as_a_lead_over_the_limit`
[mutation]; `test_stale_reason_thresholds` probes 0.01 s above each threshold and `test_envelope_clips_only_in_policy_phase` also
below ENVELOPE_LO [mutation each].
Write `real_safety.py` early: Track C's 87 imports `move` (pending integration until it exists).
</action>
<verify>
  $PY -m pytest tests/test_unit_real_safety.py tests/test_unit_real_safety_filter.py tests/test_unit_real_timing.py -q > runs/real_b1_unit.log 2>&1; echo "exit $?"
</verify>
<done>Unit tests exit 0 in seconds; every [mutation] seen failing first (runs/real_b1_mutation.log); review GO - reached round 5 (18:43), committed e914654 (18:44).</done>
<effort>planned 2.75 h agent time; actual: started 17:49 on rev 6.2 (no rev-6.3 item touches B1), built by 17:54, fix round 2 from 18:12</effort>
</task>

<task id="B2" track="B2" push="2-if-GO" parallel_with="A,B1,C,E1">
<name>Track B2: adapter 86 - modes gate/release/hands-open/ready/replay/shadow/run, the only publisher module, the motion gate</name>
<files>scripts/86_real_run.py, g1sim/real_run.py, g1sim/real_send.py, g1sim/motion_gate.py, g1sim/real_preflight.py, g1sim/real_run_log.py, g1sim/real_run_modes.py, g1sim/real_run_policy.py, tests/helpers/fake_real_run.py, tests/test_unit_real_run.py, tests/test_unit_motion_gate.py, tests/test_unit_real_preflight.py, tests/test_unit_real_send.py</files>
<action>
Interface: uses 3.1 (A), 3.2 (A), 3.3 (B1), `g1sim/policy_wire.py` (`PolicyClient(host, port, timeout_s=2, connect_wait_s=10)`, `TASK`,
`IMAGE_KEY`, `STATE_KEY`, `check_observation`); provides 3.4. Rules 4.2 and 4.3.
Started 17:47 on **rev 6.2**, built by 18:11 (the five split files above came from its ~300-line rule, R4-2). Its review and fix rounds
hold it to rev 7: steps 2a/2b (R3-3, R3-5, R3-10 (c)) were not in its task text at build time (R4-3: the 18:08 code read the unlock JSON
from the working tree and had no `--out` refusal and no `foreign_hand_cmd`), plus rev 7's `MAX LEAD` line (step 1b) and A review r1
item 4 (`real_run.py` types `out` as `str`, but `real_io.RUN_LOG_DIR` is a `pathlib.Path`). **Rev 7.1 (R5-2 (5)):** B2's round-2+
re-reviews run against the working-tree `real_io` that contains Track AF (`_open_channels`, `close_dds` at `atexit`); the reviewer
confirms that `close_dds` runs only after 86's `finally` ramp-out and after the policy worker thread is joined, and records
`git hash-object g1sim/real_io.py` in its log. Round 2 (18:43) was NO-GO; fix round 2 builds the `MAX LEAD` line (step 1b).
1. Modes: `ready` = preflight -> gate -> ramp_in -> to_ready (`BALL_READY_POSE` arms, waist 0, hands `BALL_READY_HAND`) -> hold (lead printed
   once per second) -> to_start -> hands open -> ramp_out. `replay`/`run` = the ready path, then the policy phase with `ChunkScheduler` and a
   worker thread for the client, and every exit through `soft_exit_plan(..., from_policy=True)` (rule 12, RV6-1); `shadow` = ready pose
   held, policy actions filtered and logged, never sent; `release`, `hands-open` per 4.2. The runner checks `MAX_WEIGHT_STEP` on every
   message it sends, with release's first message as the one exception (m6-8).
1b. (B1 review P1, rule 16 (c)) `replay` and `run` print `MAX LEAD <rad> (<joint name>)` before the final `86 <MODE>:` line: the largest
   |sent_q - measured_q| over the arm and waist motors while phase == "policy" and weight == 1 (the runbook's M3 -> M4 check). One test
   `test_max_lead_line_reports_the_largest_policy_phase_lead` (fake sagging arm; Catches a lead taken over the ramp, where w < 1).
2. Observation: `{IMAGE_KEY: frame, STATE_KEY: real_io.state27(...), "prompt": TASK}`, request `tick = int(mark_requested())` (m6-5);
   `--no-hands`, and `replay` whenever hand state is missing (m6-6), fill hand slots with the last hand command and log
   `hand_state_synthetic`. `replay --no-camera` sends a zero image (81 replay ignores it). Gate (a) includes the date check (m6-11, R13).
2a. Preflight (rule 2) also warns on `rt/brainco/<side>/cmd` samples (`foreign_hand_cmd`, R3-10 (c)); `--out` is resolved against the
   repo root and refused off loopback when it lies outside `real_io.RUN_LOG_DIR` (3.4, R3-5). Gate (a) uses `git cat-file -e` and
   `git show HEAD:docs/ROBOT_DAY_MOTION.json` (4.3, R3-3).
2b. Sidecar written twice (3.4, R2-2 (ii)): before the first writer is created and at exit (temp file + rename); the factory-gap refusal
   asks `real_io.arms_may_be_held_by_us(args.out)` and prints only the way out that works (3.4, R2-2 (iii)).
3. Tests with fakes (FakeReader with a scripted, sagging arm; FakeSender recording messages; FakeClock; FakePolicyClient; FakeFrames):
   `test_ready_sends_frozen_q0_during_ramp_in_while_arm_sags` [mutation: re-read per tick], `test_every_arm_sdk_message_holds_waist_roll_pitch`
   [mutation], `test_contact_fault_ramps_out_on_measured_angles`, `test_soft_exit_returns_to_start_and_opens_hands`,
   `test_stale_lowstate_gives_fault_exit_code_4` [mutation], `test_foreign_arm_sdk_samples_refuse_before_own_writer_exists` [mutation],
   `test_start_refused_when_arms_off_factory_pose`, `test_release_refused_without_factory_record`, `test_second_ctrl_c_turns_soft_exit_into_fault_exit`,
   `test_fill_arm_sdk_writes_17_motors_and_weight_slot_29`, `test_sdk_motor_layout_equal_in_real_io_and_real_safety`,
   `test_motion_code_never_uses_damp_or_lowcmd` [mutation]; gate (temp git repo fixture): `test_gate_locked_without_motion_json`,
   `test_gate_refuses_file_changed_since_pinned_commit` [mutation], `test_gate_refuses_file_missing_from_pinned_commit`,
   `test_gate_refuses_untracked_motion_json` [mutation: drop the `cat-file -e HEAD:` check] (R3-3; an **untracked** hand-made JSON with
   `commit` = HEAD and today's date must give LOCKED; Catches the "one helpful edit" unlock), `test_gate_reads_motion_json_from_head_not_worktree`
   (R3-3; committed JSON says `motion_allowed: false`, working copy edited to true -> LOCKED), `test_gate_needs_two_distinct_operators`, `test_gate_needs_hanging_confirmation`,
   `test_sim_bypass_refused_with_robot_network_present` [mutation], `test_gate_locked_when_unlock_date_is_not_today` [mutation] (m6-11);
   runner: `test_policy_exit_goes_through_ready_pose_before_q0` (RV6-1), `test_replay_requests_advance_tick_with_scheduler` (m6-5),
   `test_replay_without_hand_state_fills_last_command` (m6-6), `test_weight_step_above_0_02_refused_except_release_first_message` (m6-8),
   `test_factory_gap_refusal_offers_m0_only_when_arms_are_free` (RV6-2, R2-2 (iii); replaces rev 6.1's `..._names_m0_and_release`),
   `test_sidecar_marks_arm_writer_before_first_publish` [mutation: write the sidecar only at exit] (R2-2 (ii): the FakeSender asserts on
   its first `send` that the sidecar exists with `exit_code: null` and `arm_sdk_writer: true`; Catches a killed run that leaves no trace),
   `test_out_dir_outside_run_log_dir_refused_off_loopback` (R3-5; Catches a robot run whose sidecar 84 never reads).
4. Test files start with the `pytest.importorskip` lines of section 0 (m6-3).
</action>
<verify>
  $PY -m pytest tests/test_unit_real_run.py tests/test_unit_motion_gate.py tests/test_unit_real_preflight.py tests/test_unit_real_send.py -q > runs/real_b2_unit.log 2>&1; echo "exit $?"
  $PY scripts/86_real_run.py gate > runs/real_b2_gate.log 2>&1; echo "exit $?"     # expected: MOTION: LOCKED, exit 2
  # pending integration: e2e against 85 (Track C file) and from the Python 3.10 robot env
</verify>
<done>Unit tests exit 0, every [mutation] seen failing first (runs/real_b2_mutation.log); `86 gate` prints LOCKED with "docs/ROBOT_DAY_MOTION.json not found - only Finnegan can unlock"; review GO.</done>
<effort>planned 5.25 h agent time; actual build 24 min (17:47-18:11); review/fix rounds from 18:12 (rev 6.3/7 items add ~15-30 min)</effort>
</task>

<task id="C" track="C" push="2-if-GO" parallel_with="A,B1,B2,E1">
<name>Track C: 85 makes the DDS sim obey rt/arm_sdk; sim-to-sim and fault-injection tests of exactly the 86 code path</name>
<files>scripts/85_arm_sdk_sim.py, tests/helpers/sim_host.py, tests/test_integration_arm_sdk_sim.py, tests/test_e2e_real_adapter_sim.py, scripts/87_arm_path_clearance.py, tests/test_integration_arm_path_clearance.py</files>
<action>
Interface: section 3.5; reads 86's log contract (3.4) and A's `fake_camera.py`, E1's `tests/data/*` (pending integration until present);
87 imports B1's `real_safety.move` and `real_safety.sdk_targets`, and reads E1's `tests/data/replay_demo.npz` for its `policy_exits` block.
Starts on rev 7.1 (not started at 18:45); its code review checks 87 against rev 7.1 section 3.5. **Build note R5-1 (major, plan review
round 5):** 87 picks block 2's start poses by arm movement with `select_start_poses` (3.5): keep a pose whenever the step from the last
kept pose would exceed `MAX_SAMPLE_STEP_M`, with joint-space interpolated poses where two raw frames are farther apart; report and
assert `max_start_step_m`; there is no `--frame-step` and no every-20th-frame fallback (the runtime fallback is fewer setbacks or one
waist limit per run + `87 --merge`). **Build note R5-2 (5):** C's e2e runs against the working-tree `real_io` with Track AF's
`close_dds` at `atexit`; the e2e log prints `git hash-object g1sim/real_io.py`, and F6/K check that 86's last arm_sdk message (weight 0)
arrives before the process ends.
Before any sim or DDS start (builder, reviewer and fixer alike), both checks must print nothing (wait and re-check, never kill a foreign
process): `pgrep -fl "82_vla_eval|81_smolvla_server|lerobot-train"` and the loopback DDS check of section 0, copied exactly (R3-4, R4-7:
it covers `loopback_lowstate_pub|84_real_bringup|89_robot_day_selftest` of the main session's push-1 check and C's own DDS pytest runs).
Only one sim at a time. 87 is FK only (no DDS) and needs neither check.
1. `sim_host.py`, `85_arm_sdk_sim.py` per 3.5.
2. `tests/test_integration_arm_sdk_sim.py` (marker integration; runs during the build): `test_85_blends_weighted_arm_sdk_into_a_lowcmd_copy`
   (Catches an in-place blend that drifts GR00T's own arm targets, review m6), `test_85_latches_last_message_after_sender_stops`,
   `test_85_refuses_a_non_loopback_iface`, `test_85_no_blend_counts_samples_without_obeying`.
2b. (RV6-1) `87_arm_path_clearance.py` per 3.5 and `tests/test_integration_arm_path_clearance.py` (marker integration, FK only, seconds):
   `test_ready_pose_clears_table_stand_and_basket_at_nominal_stop` (Proves the scene and distance code agree with the measured >= 4.9 cm of
   `BALL_READY_POSE`; Catches a wrong scene, stance or geom set), `test_clearance_detects_a_path_through_the_table` (a planted path whose
   hand ends 3 cm inside the table top reports a negative clearance; Catches a distance call that never sees the table) [mutation: skip the
   table geoms], `test_setback_increases_clearance` (Catches a setback applied with the wrong sign),
   `test_policy_exit_pass_rule_compares_with_start_clearance` (R2-1; a planted exit that starts 1 cm from the stand and moves away passes,
   a planted exit that starts clear and dips to 1 cm from the table edge fails; Catches a plain 5 cm rule that would fail every grasp
   frame, and an "only the start matters" rule that misses the dip) [mutation: drop the start-clearance term],
   `test_fault_exit_counts_the_worst_of_straight_and_corner_orderings` (R3-7; a planted start/goal pair whose straight line clears the
   table edge but whose `distal_first` corner dips below 5 cm must fail; Catches an 87 that assumes a q-only blend) [mutation: straight
   line only], `test_policy_exits_are_reported_per_waist_limit` (R3-2; a planted demo frame turned 0.6 rad gives different start poses
   and rows for L = 0.15 and 0.6; Catches a lookup that reuses the 0.6 numbers for a run at 0.15),
   `test_fault_exit_sampling_cannot_skip_a_close_pass` (R4-1; pure numpy, no MuJoCo: `densify` with a planted one-joint "arm" - a point
   0.5 m from its axis, as shoulder to fingertip, sweeping 2.4 rad in one leg of 10 even samples, 13.3 cm apart - and a planted edge
   point 1 cm outside the arc, midway between samples 4 and 5; the even samples alone read ~6.8 cm and would pass the 5 cm rule, the
   densified path reads <= 1.5 cm and fails it, and every consecutive step is <= `MAX_SAMPLE_STEP_M`; Catches an 87 that keeps the coarse
   corner sampling) [mutation: `densify` returns the even samples unchanged],
   `test_start_poses_follow_arm_movement_not_frame_count` (R5-1; pure numpy, no MuJoCo, like the test above: a planted demo of 21
   frames of the same 0.5 m one-joint arm sweeping 2.4 rad (6 cm per frame, so interpolation is exercised) and a planted edge point
   1 cm outside the arc midway between frames 14 and 15; `select_start_poses` gives a minimum start clearance <= 1.5 cm, which fails
   the 5 cm rule, and `max_start_step_m` <= `MAX_SAMPLE_STEP_M`; every-10th-frame selection (frames 0, 10, 20) would read ~27 cm and
   pass; Catches a return to fixed frame steps) [mutation: fixed frame step 10],
   `test_merged_report_keeps_every_waist_limit_and_refuses_mixed_settings` (R5-1 fallback; pure, planted per-limit report dicts: the
   merge has the rows and `fault_exits_clear` of both limits, and parts with different `MAX_SAMPLE_STEP_M` raise; Catches a merge that
   silently mixes runs).
   Run 87 once with the defaults (C may run it during Build; FK only) and keep `runs/arm_path_clearance.json` + its printed table in
   `runs/real_c_clearance.log` for E2's runbook lookup. If `tests/data/replay_demo.npz` does not exist yet, 87 writes `policy_exits: null`
   with the reason and C marks the `--demo` run "pending integration" (6.3 re-runs 87).
3. `tests/test_e2e_real_adapter_sim.py` (marker e2e; each test starts what it needs and kills it; 85 latches, so test K gets a fresh 85;
   every test records its own sim factory pose in its fixture and passes `--out <tmp dir>` to 86, m6-4, so a record from before midnight
   or a log in `runs/real` never changes a later run):
   - `test_ready_ramps_without_motion_and_holds_waist` (G1, `86 ready --hold 3`): weight starts 0, steps <= 0.02, ends 0; motors 12-14 move
     < 0.03 rad during `ramp_in` (N1); 13/14 within 0.02 while w > 0; kp/kd <= SIM_GAINS; max gap < 60 ms.
   - `test_replay_tracks_demo_inside_envelope` (G2, 81 `--replay tests/data/replay_demo.npz`, `86 replay --speed 0.5 --max-seconds 40`):
     at w = 1 tracking median < 0.05 rad, max lead < 0.15 rad (n2); every `policy` target inside the envelope; pelvis z > 0.5;
     `loaded_files_at_exit == gate.loaded_files`; every received message is in 86's sent log (matched by content); the policy-phase
     `policy_actions` equal `replay_demo.npz` actions at the requested ticks and the ticks increase (m6-5, Catches a replay stuck at tick 0);
     the exit passes through `BALL_READY_POSE` (phase `to_ready` after `policy`, RV6-1).
   - `test_release_after_kill_moves_slowly_to_factory_pose` (K): `86 ready --hold 30`, SIGKILL during hold (author test in sim only), next
     `86 ready` exits 2 (factory gap), `86 release`: first message weight 1 at measured angles, peak joint speed < 0.35 rad/s, ends within
     0.03 of the factory pose, weight 0.
   - Faults: `test_camera_stop_gives_soft_exit_and_opens_hands` (F1, exit 3 `camera_stale`, last hand command 0),
     `test_policy_server_death_gives_soft_exit` (F2, exit 3 `policy_stale`), `test_ctrl_c_once_soft_twice_fault` (F6, exit 3 then exit 4 with
     measured-angle targets), `test_lowstate_pause_gives_fault_exit` (F7, `85 --pause-lowstate-at 10 --pause-for 1`, exit 4),
     `test_foreign_arm_sdk_writer_refuses_start` (F8; its fixture records the sim factory pose **before** starting the foreign
     writer, because 84 refuses to record while `rt/arm_sdk` samples arrive, R3-9; `publish_test_arm_sdk` running, exit 2, 85 saw no message from 86).
   - `test_84_sends_nothing_to_the_sim` (85 `--no-blend`, 84 all steps with `--yes-sim`, 0 `rt/arm_sdk` samples, `own_writers == 0`).
   - `test_vla_40k_shadow_then_live_runs_clean` (only when `G1_E2E_40K=1`; integration phase): 81 `--checkpoint $CKPT --device mps`;
     `86 shadow` 30 s exit 0; `86 run --speed 0.5 --max-seconds 60` exit 0, or 3 with `envelope_streak` reported as a policy finding.
</action>
<verify>
  $DDS $PY -m pytest -m integration tests/test_integration_arm_sdk_sim.py -v > runs/real_c_integration.log 2>&1; echo "exit $?"
  $PY -m pytest -m integration tests/test_integration_arm_path_clearance.py -v > runs/real_c_clearance_tests.log 2>&1; echo "exit $?"   # needs B1's real_safety.py
  $PY scripts/87_arm_path_clearance.py --report runs/arm_path_clearance.json --demo tests/data/replay_demo.npz > runs/real_c_clearance.log 2>&1; echo "exit $?"
  $PY -c "import json; s = json.load(open('runs/arm_path_clearance.json'))['summary']; assert s['max_sample_step_m'] <= s['MAX_SAMPLE_STEP_M'], s; assert s['max_start_step_m'] <= s['MAX_SAMPLE_STEP_M'], s; print('max sample step', s['max_sample_step_m'], 'max start step', s['max_start_step_m'], 'start poses', s['start_poses'], 'samples', s['samples'])" > runs/real_c_clearance_step.log 2>&1; echo "exit $? (R4-1, R5-1)"
  # (E1 wrote tests/data/replay_demo.npz at 17:42, so C runs block 2 during its build; without the file, drop --demo and mark it pending)
  # pending integration (needs 86, fake_camera, tests/data): $DDS $PY -m pytest -m e2e tests/test_e2e_real_adapter_sim.py -v > runs/real_c_e2e.log 2>&1
</verify>
<done>Integration tests exit 0; `runs/arm_path_clearance.json` written with a summary line per elbow and waist limit (incl. `replay_exits_clear`, `frames_inside` and `fault_exits_clear`, or `policy_exits: null` with "pending integration"), 87's sample count and elapsed time printed (a reported number, not a pass criterion, R3-8); the report's `max_sample_step_m` and `max_start_step_m` <= `MAX_SAMPLE_STEP_M` (0.02, R4-1, R5-1); a planted corner dip fails the ordering test first, the coarse-sampling mutation fails the sampling test first and the fixed-frame-step mutation fails the start-pose test first; e2e file collects and every test whose inputs exist passes; a planted in-place blend fails the blend test first, a planted table-blind distance and a planted plain-5-cm pass rule fail their clearance tests first; review GO.</done>
<effort>planned 5.5 h agent time (85 1.75, sim_host 0.5, integration tests 0.5, e2e file 1.25, 87 + its tests 1.5 - rev 6.3 adds the
waist limits, the corner orderings, the pre-filter and two tests; rev 7 adds `densify` + one pure test, ~15 min; rev 7.1 adds
`select_start_poses`, `--merge` and two pure tests, ~15 min). Observed pace of the
other tracks is 5-25 min per build, so C (the largest track, with sims) is expected to take 30-60 min plus 87's 10-25 min run.</effort>
</task>

<task id="E1" track="E1" push="1" parallel_with="A,B1,B2,C">
<name>Track E1: handover kit for push 1 - runbook Parts A+B, laptop setup, robot-free self-test 89, loopback helper, test data, 81 cuda line</name>
<files>docs/ROBOT_DAY_RUNBOOK.md, scripts/setup_robot_laptop.sh, scripts/89_robot_day_selftest.py, tests/helpers/loopback_lowstate_pub.py, tests/helpers/make_robot_day_data.py, tests/data/selftest_episode.npz, tests/data/replay_demo.npz, docs/robot_day/reference_head_frame.png, tests/test_unit_runbook.py, scripts/81_smolvla_server.py</files>
<action>
Interface: 3.1 (`robot_net_present`, `LowStateReader`), 3.2 (`FrameStream`, `fake_camera.py`), 3.4 (86 CLI), 3.6.
Only push-1 items here (RV6-3); runbook Part C, the 89 `86 gate` line and the phases image are E2.
Started 17:38 on rev 6.2; the rev-6.3 items marked (R3-x) below arrive through E1's review and fix round.
1. `make_robot_day_data.py` runs in the **SmolVLA env `$SV`** (m6-2: the sim env has no pyarrow/datasets; `$SV` has pyarrow, PIL, cv2;
   the script puts the repo root on `sys.path` like `scripts/81_smolvla_server.py:33`). It reads dataset episode 0 from the parquet files under `$DATASET`, decodes `observation.images.camera1` (PNG
   bytes, `(200, 320, 3)`) into `head_image`, maps `observation.state` -> `state` and `action` -> `actions`, and writes, with
   `g1sim.episode` (numpy-only): `selftest_episode.npz` (frames 0-2), `replay_demo.npz` (all 726 actions and states, task, fps; `head_image`
   all zeros because 81's replay only uses images for diagnostics - stated in the file's sidecar note and in the runbook), and
   `reference_head_frame.png` (frame 0). Kept in E1 although `replay_demo.npz` is a motion input: it is the same one-minute run and C's
   e2e needs it early.
2. `setup_robot_laptop.sh` exactly as section 3.6 (R3-6: written out there; blocks print `SETUP <block>: OK|FAILED <hint>`;
   re-runnable; `uv python install 3.10` default on Ubuntu; `--release-file` and `--release` share the checksum + unpack code). Its
   outputs are what the other parts use: `<prefix>/robot-env.sh`, `<prefix>/policy-env.sh` (exports `HF_HUB_CACHE=<prefix>/hf_cache/hub`,
   R3-1), the unpack target `<prefix>/smolvla-runs/run1/checkpoints/040000/pretrained_model`, and the backbone cache
   `<prefix>/hf_cache/hub` with SmolVLM2-500M-Video-Instruct and `smolvla_base`.
3. `89_robot_day_selftest.py` exactly as section 3.6 (guard first; `versions`, `dds`, `camera`, `bringup`, `imports`, `policy`; ends
   with `SELFTEST: PASS|FAIL`). (R3-1) The `policy` check FAILs when the SmolVLM2 folder is missing from `<prefix>/hf_cache/hub` and its
   PASS line names the cache path it used.
4. `81_smolvla_server.py` line 159: `choices=("mps", "cpu", "cuda")`.
5. `docs/ROBOT_DAY_RUNBOOK.md` (plain English, every command copy-paste, "you should see / if you see, do"): page 1 rules (section 7, incl.
   "charger on, lid open", with the macOS `caffeinate -i` and Ubuntu `systemd-inhibit` prefixes, m6-10), the "never" box (72 `arm`,
   72 `hand --bewegen`, `kill -9`, editing files, anything not in the runbook), the definition of "hangs in the gantry" (section 4.3 (d),
   R2-5: stands on its own feet in the owner's arm_sdk mode, harness on the gantry with only a few cm of slack; type HANGING only then),
   and the box **"86 release refused or crashed -> tell the owner to use the remote (the robot hangs in the gantry, so damping is safe);
   we do nothing more; note the time and the last 20 lines"**.
   (R3-6, from rev-5 findings m11 and r5-4, now in the task text:) Part A has a **firewall step**: macOS
   `/usr/libexec/ApplicationFirewall/socketfilterfw --getglobalstate`; if it says enabled, allow incoming connections for the robot-env
   python (`socketfilterfw --add <prefix>/robot-env/bin/python` and `--unblockapp` the same path, with the admin password) - otherwise DDS
   samples from the robot never arrive, and 89's loopback test cannot see this; Ubuntu: `sudo ufw status`, and if active, allow the
   robot network (`sudo ufw allow from 192.168.123.0/24`). Part B step 4 (`--label owner_motion`, Q12): **both operators stand outside
   the arms' reach (1.5 m) while the owner's tool moves the arms**, and the label records the owner's tool name and mode.
   Part A Sunday evening, Part B Monday read-only (each step with its abort rule; table/camera numbers; R11 server placement; who to call -
   Finnegan's phone filled in by him, reply not guaranteed). Part C is a one-line stub "Part C (motion) arrives with push 2 on Monday
   morning, only if `86 gate` prints UNLOCKED".
6. Tests: `test_runbook_commands_match_script_flags` (each `scripts/NN_*.py` command's flags exist in that script's argparse; a command whose
   script is absent - 84, 86, 87 while the tracks build in parallel - is skipped with the reason "<script> not built yet", m6-13),
   `test_never_box_lists_72_moving_commands`, `test_loopback_pub_refuses_robot_network` [mutation] (R3-11: `ChannelFactoryInitialize`
   and the publisher class are faked, so the mutated copy without the refusal fails on the fake and never publishes fake `rt/lowstate`
   on a real interface), `test_81_accepts_cuda_device`, `test_selftest_policy_fails_without_backbone_in_prefix` (R3-1; temp prefix with
   policy env stub and model but no `hf_cache/hub/models--HuggingFaceTB--SmolVLM2-500M-Video-Instruct` -> FAIL naming the setup command;
   Catches a self-test that passes on Finnegan's Mac from his old cache).
   Test files start with the `pytest.importorskip` lines of section 0 where they import DDS modules (m6-3).
</action>
<verify>
  $SV tests/helpers/make_robot_day_data.py > runs/real_e_data.log 2>&1; echo "exit $?"
  bash scripts/setup_robot_laptop.sh --robot-env --prefix $P > runs/real_e_setup_robot.log 2>&1; echo "exit $?"
  bash scripts/setup_robot_laptop.sh --release-file $REL/smolvla_run1_040000.tar --prefix $P > runs/real_e_setup_release.log 2>&1; echo "exit $?"
  # R2-6: the policy env (torch + 2.7 GB) is started DETACHED after the release step (never at the same time against the same prefix),
  # so it survives the end of the agent's Bash call; its PID goes to a file so anyone can check or stop it later
  nohup bash scripts/setup_robot_laptop.sh --policy-env --prefix $P > runs/real_e_setup_policy.log 2>&1 < /dev/null & echo $! > runs/real_e_setup_policy.pid
  $PY -m pytest tests/test_unit_runbook.py tests/test_unit_policy_wire.py -q > runs/real_e_unit.log 2>&1; echo "exit $?"
  # finished when `kill -0 $(cat runs/real_e_setup_policy.pid)` fails and the log ends with "SETUP policy-env: OK"; then (CPU only, allowed
  # by section 0, R2-6):
  # R3-1: the self-test must use the prefix's own backbone cache, never Finnegan's ~/dev/g1-ext/smolvla-test/hf_cache
  (source $P/policy-env.sh && echo "HF_HUB_CACHE=$HF_HUB_CACHE" && test "$HF_HUB_CACHE" = "$P/hf_cache/hub" \
      && test -d "$P/hf_cache/hub/models--HuggingFaceTB--SmolVLM2-500M-Video-Instruct" && test -d "$P/hf_cache/hub/models--lerobot--smolvla_base" \
      && python scripts/81_smolvla_server.py --checkpoint $P/smolvla-runs/run1/checkpoints/040000/pretrained_model \
      --self-test tests/data/selftest_episode.npz --device cpu) > runs/real_e_policy_selftest.log 2>&1; echo "exit $?"
  grep -n "HF_HUB_CACHE=$P/hf_cache/hub" runs/real_e_policy_selftest.log > runs/real_e_policy_cache.log 2>&1; echo "exit $? (0 = prefix cache used)"
  # pending integration (section 6.2): full 89 SELFTEST from the fresh prefix
</verify>
<done>`SETUP robot-env: OK`, `SETUP release: OK` (checksum + unpack), policy self-test exit 0 on CPU from the fresh prefix **with `HF_HUB_CACHE` = `$P/hf_cache/hub` and both backbone folders present there** (R3-1; or "still installing, PID in runs/real_e_setup_policy.pid" noted), unit tests exit 0 with the [mutation] seen failing; runbook Parts A and B complete, Part C stub; review GO (expected ~22:45-23:15, section 6.1).</done>
<effort>about 3.5 h agent time (runbook A+B 1.0, setup 1, 89 + helper 0.75, data 0.25, tests 0.25; rev 6.3 +0.25 h: cache check, firewall
step, one test); policy-env install in the background</effort>
</task>

<task id="E2" track="E2" push="1 (fix set, after its fix round GO; R5-2)" depends_on="E1 review GO; Track RB's gate GO before its fix round; C's runs/arm_path_clearance.json (pending integration until present)">
<name>Track E2: runbook Part C (M0-M6) with the factory-pose step and the table-clearance lookup, 89's motion line, the M2 comparison image</name>
<files>docs/ROBOT_DAY_RUNBOOK.md (Part C only), scripts/89_robot_day_selftest.py (motion line + forbidden list), docs/robot_day/volleyball_natural_phases.png, tests/test_unit_runbook.py (three tests)</files>
<action>
Same owner as E1; starts only after E1's review is GO, so no two agents write these files at once. Started 18:15 on rev 6.3; the rev-7
items marked (R4-x) / (B1-P1) below reach it as review findings.
1. Runbook Part C exactly in the order of section 7 (M0 first), each step with its command, "you should see", abort rule and the boxes.
   - M0 (RV6-2): the owner sets the agreed arm_sdk mode (Q8/Q10) and leaves the arms where the controller holds them (the owner's own
     arm tool, if used in Part B step 4, has been stopped); then `84 --steps joints --record-factory-pose --mode-note "<owner's Q8
     answer>"`; on a conflict the overwrite rule of section 3.1. (R3-9) If 84 prints the Part B WARNING (the new pose differs from the
     Part B step 0 reading by more than 0.10 rad) and the owner did **not** change the mode: the owner re-sets the robot or confirms
     their tool ended at weight 0, then M0 is recorded again; if 84 refuses because `rt/arm_sdk` samples arrive, the owner stops their
     tool first.
   - M0 lookup (RV6-1, R2-1, R2-3, R3-2, R3-10; rule 16), copied from `runs/arm_path_clearance.json`:
     - first the bounds line: "if any shoulder roll, shoulder yaw, wrist or the waist yaw on the FACTORY ARMS line is more than 0.15 rad
       from 0, a shoulder pitch is outside -0.1..0.3 or an elbow outside 0.0..1.6: skip M4-M6 at the table";
     - then one row per elbow (0.0-1.6 in 0.1 steps; (R4-6) the operator reads **both** rows around the larger of the two factory
       elbows - floor and ceil, e.g. 0.93 -> 0.9 and 1.0; exactly on a row -> that row - and writes "yes" only where both rows say yes,
       with the larger setback of the two; rule 16 (b2)): `elbow | table position
       (Part B tape mark / move back N cm to the second mark / skip) | M4 at the table: hold / 0.15 / 0.35 / 0.6 | M6 at the table:
       hold / 0.15 / 0.35 / 0.6` (yes / no per waist column), with, per waist limit L, M4 = `table_steps` != skip and
       `replay_exits_clear[setback][L]` and `frames_inside[setback][L].table` == 0 and `fault_exits_clear[L]` (ruling R15 default), and
       M6 = M4 and `setback_m` <= `setback_max_policy_m`.
     - the waist rule (rule 16 (b)): M4/M6 at the table use exactly one of `--waist-hold`, `--waist-max 0.15`, `0.35`, `0.6`, never
       above the limit reached in M3, and read that column; a "no" allows a lower checked limit whose column says "yes".
   - A box for 86's `foreign_hand_cmd` warning (R3-10 (c)): "another program sends hand commands - ask the owner to stop their hand tool
     and run again; if it cannot be stopped, add `--no-hands`".
     - If the JSON or its `policy_exits` block is not there yet, Part C says in bold "M4-M6 at the table are NOT cleared" and the
       integration phase fills the table.
   - The rule 16 (e) box "crash, kill or cable pull during M4-M6": the owner decides about pulling the table back before `86 release`;
     otherwise the owner's remote.
   - (R4-4) Page 1 gets the one-laptop rule: "84 M0 and every 86 command run on one laptop all day. If it dies while 86 runs: owner's
     remote, the owner re-stands the robot, then M0 on the new laptop."
   - (B1-P1, rule 16 (c)) M3: note the `MAX LEAD` line of each in-air replay on the sheet; M4/M6 at the table need it below 0.15 rad at
     the waist setting used (stand-in if 86 does not print it: the M2 hold lead, abort > 0.10).
   - (R4-5, rule 16 (f)) at M4/M6 and in the exits paragraph: "a fault while the ball is lifted may press the ball onto the table edge -
     that is contact with the table: abort and note it".
   - (R16, R17 defaults) the M0 line "84 `joints` FAILs on one joint within about 0.02 rad past its limit while it rests on its stop: do
     not record M0, note joint and value, Parts A+B continue", and the "never" box line "never run 86 with `--yes-sim` on the robot
     laptop".
1b. **Fix round (rev 7.1; starts only after Track RB's diff check is GO, so the runbook has one writer; R5-2 (3)):** E2 review r1
   minors 1-6 (`real-robot-code-E2.md`: the `setback_max_keep_layout_m` clause in the M4 cell and its oracle, `summary["elbows"]` as
   pinned in 3.5, 89's `TimeoutExpired` -> INFO, 89 shows 86's first reason line, "rad" + a bounds-vs-`GRID_BOUNDS` test, a split
   `m4_allowed`), plus:
   - (RV-2) before M4: the robot's feet are on their Part B tape marks and the 24.5 cm (+ setback) is measured once more;
   - (RV-4, rule 4.1 (3)) page 1 and Part C: nothing may move the head after Part B; the Part B 3 `84 --steps camera` line is re-run
     before M4, before M5 and after any mode change or reboot; a different overlay -> the owner re-sets the head, check again;
   - (R5-3, rule 16 (e)) the crash box opens with "86 printed no final `86 <MODE>:` line, or printed `FAULT EXIT lowstate_stale`";
   - (RV-1) M0: if the owner changed the stand height or mode since Part B 3, re-measure the hip height and re-set the table.
   Its gate: "Track E2 - round 2" in `real-robot-code-E2.md` (a diff check of the fix round). After it is GO, E2's files (runbook incl.
   Part C, 89, `tests/test_unit_runbook.py`, the phases image) join the push-1 fix-set commit; Part C is inert in push 1 (`86 gate`
   prints LOCKED or "no such file", and 89 runs `86 gate` only when `scripts/86_real_run.py` exists). **Fallback (R5-2 (1)):** if the
   fix round is not GO twice, E2 (the only writer) parks its versions under `runs/e2_parked/`, restores Part C to E1's stub and 89 +
   `tests/test_unit_runbook.py` to their `f897b0c` content (keeping what Track RB's runbook tests need), re-runs
   `tests/test_unit_runbook.py`, and E2's files go to push 2.
2. `89`: if `scripts/86_real_run.py` exists, run `86 gate` and print its MOTION line (information, not a FAIL). (R4-2) 89's
   `FORBIDDEN_MODULES` adds `g1sim.real_preflight`, `g1sim.real_run_log`, `g1sim.real_run_modes`, `g1sim.real_run_policy` (rule 4.1 (b)).
3. Copy `docs/volleyball_natural_phases.png` (git-ignored by `.gitignore:7`) to `docs/robot_day/volleyball_natural_phases.png` (not ignored;
   m6-1) and point M2 there.
4. Tests: `test_runbook_part_c_starts_with_factory_pose_record` (Catches a Part C that lets 86 run before M0),
   `test_runbook_clearance_table_matches_report` (every row incl. the M4 and M6 cells of all four waist columns; skipped with a reason
   while `runs/arm_path_clearance.json` or its `policy_exits` block is absent; Catches stale numbers, a waist column copied from 0.6,
   and an M4/M6 "yes" the report does not support), `test_runbook_lookup_reads_both_neighbouring_rows` (R4-6: the runbook's reading
   instruction names floor and ceil; Catches a return to "round up" that trusts monotonic clearance) [mutation: text "rounded up"].
</action>
<verify>
  $PY -m pytest tests/test_unit_runbook.py -q > runs/real_e2_unit.log 2>&1; echo "exit $?"
  git check-ignore -v docs/robot_day/volleyball_natural_phases.png > runs/real_e2_ignore.log 2>&1; echo "exit $? (1 = not ignored, wanted)"
</verify>
<done>(Rev 7.1) the fix-round items above are in and "Track E2 - round 2" is GO; Part C complete with M0 (incl. the Part B comparison), the bounds line, the clearance lookup with M4/M6 cells per waist column read from both neighbouring rows (or the bold NOT CLEARED line), the waist rule, the M3 lead line, the ball line, the one-laptop rule, the hand-tool box and the crash box; 89's forbidden list matches rule 4.1 (b); image tracked-able; unit tests exit 0; review GO before C3.</done>
<effort>planned 1.5 h agent time (rev 6.3: waist columns, Part B comparison, hand-tool box; rev 7 +15 min: two rows, lead, ball, one laptop, forbidden list)</effort>
</task>

## 6. Timeline (Sunday ~17:20 - Monday 09:00), checkpoints, push contents
### 6.1 Clock plan (re-timed in rev 7 from STATUS, R4-3; push-1 rows re-timed in rev 7.1, R5-2; the workflow runs build -> review -> fix per track; integration waits for all tracks)
Observed pace (STATUS 17:38-18:19): builds 5-25 min (B1 5, A 8, E1 13, B2 24), reviews 5-6 min, fix rounds 5-8 min; A and E1 were GO at
18:13. The rev-6.3 clock (push 1 ~00:00, push 2 ~08:00 with about 0 slack) assumed 3-5 h builds and is replaced. The times below are
expectations from that pace, not promises; the **cutoffs** are the old late times and stay as the fail-safe limits. The user should hear
plainly: push 1 can land this evening, so the operators can do Part A (setup + 89) tonight, which is safer than a Monday 08:00 install
over venue Wi-Fi.
| Time (expected) | A / E1 / E2 / A2 | B1 / B2 / C | Main session (and Finnegan) |
|---|---|---|---|
| 18:13-18:15 | A and E1 review GO; committed locally (c0898cf, f897b0c) | B1 fix round 2, B2 review round 1 running | **charger on** (`pmset` said Battery at 18:19) |
| 18:15-18:45 (done) | E2 build + review GO r1 (18:22); paper walk A+B NO-GO (18:22), RB fixes (18:33), re-review NO-GO (18:39); AF built (18:37) | B1 GO r5, committed e914654 (18:44); B2 r2 NO-GO (18:43), fix round 2 | push-1 DDS run (18:16-18:20) found the 84 crash; planner rev 7, rev 7.1 |
| 18:45-19:30 | AF review; RB's RV fixes (from 18:40) -> diff check; then E2's fix round -> "Track E2 - round 2" (one runbook writer at a time) | **C starts on rev 7.1** (after `AC Power`); B2 fix round 2 + re-review (new `real_io`) | main session commits the push-1 fix set once AF, RB and E2-r2 are GO |
| **C2 ~19:30-20:15** | - | C build | **6.2 from the clean clone** (84 x5); **push 1** by Finnegan; message to Cristina and Saad: "Part A (setup + 89) can be done tonight" + the shared folder name |
| ~20:15-21:00 | A2 build + review (after the fix-set commit) | C build ends, review + fix to ~21:00-21:30 | - |
| **C3 ~21:30 (cutoff 03:00)** | E2, A2 have a verdict | B1, B2, C have a verdict | **main session launches the finish stage as soon as the build runs return** (not automatic; if the session is not re-invoked, push 2 carries no motion) |
| ~21:30-23:30 | integration (6.3), incl. 87 with `--demo` (10-25 min expected, R4-1) and its numbers into runbook Part C if E2 left the placeholder | | - |
| **C4 ~23:30 (cutoff 05:45)** | e2e chain green incl. replay, faults, 40k on MPS, Python 3.10 env | | - |
| ~23:30-00:00 | dry walk of Part C only + fixes | | - |
| ~00:00-01:45 | final gate (judge, budget 1.5-1.75 h) | | - |
| **C5 ~01:45 (cutoff 07:30)** | final gate verdict written | | STATUS summary with commit lists |
| after C5 **and** after 00:00 | - | - | **push 2** by Finnegan when he is awake (unlock commit only if every gate is GO; its date 2026-10-19 needs the local date to be the 19th, 4.3); target no later than ~08:00 |
| 08:00-09:30 | - | - | operators: `git pull` (again at 08:30 if Finnegan's STATUS message says push 2 slipped), `89`, `86 gate`; Part B first (about 1 h) |

Checkpoint misses:
- **C1** (A/E1 progress) passed at 18:13.
- **C2 / push 1 fallback (based on what is actually GO, RV6-3):**
  - The clean-clone check (6.2) passes -> full push 1.
  - (R5-2) A fix-set gate (AF, RB's diff check, E2 round 2) is not GO -> push 1 waits for that gate's next fix round, because the
    committed 84 crashes intermittently (6 of 11 runs, `runs/real_a_fu_diag_repeat.log`) and the committed runbook failed the paper
    walk; E2 alone not GO -> its fallback (Part C stub, E2 files to push 2). Still not GO at 23:30 -> push 1a below.
  - Any of 6.2's five 84 runs fails -> no push 1 with that 84; AF gets a fix round and 6.2 is re-run in full.
  - It fails because a file is missing from the commit (R4-2) -> add the file to the commit and re-run the check; that is a commit
    problem, not a code problem, and needs no review.
  - It fails in Track A's code or 84 and E1's own checks pass -> push 1a = `setup_robot_laptop.sh`, `tests/helpers/*` (E1's),
    `tests/data/*`, `docs/robot_day/reference_head_frame.png` and the runbook (Part A only marked "today"), so the operators can start the
    long install tonight; Track A + 89 follow as push 1b after the fix and a re-run of 6.2 (Finnegan is reachable overnight), else with
    push 2 (89 calls 84, so it cannot pass without A), and the operators run 89 at 08:00 Monday after the pull.
  - Nothing passes -> no push 1; everything goes into push 2; the operators run the setup on Monday at 08:00 (budget about 45 min for
    torch + the 2.7 GB backbone + the 865 MB release over the venue network), so Part B starts about 09:30.
- **C3 missed for B1/B2/C** (no GO by 03:00): motion stays locked on Monday; integration still runs the read-only parts.
- **C4 missed** (chain not green by 05:45, or 40k run not clean): motion locked, or (if replay + faults are green and only the 40k run
  fails) the final gate may allow Part C up to `shadow` only (cut item 5).
- **C5 missed** (no final-gate verdict by 07:30): no motion push unless the verdict lands before Finnegan's last reachable time (09:30
  only if he confirmed that window), because Part B comes first; otherwise push 2 carries only the read-only fixes that passed review.
### 6.2 Push-1 check (main session, after the push-1 fix-set commit exists - AF, RB and E2 round 2 GO, R5-2; from a clean local clone of that commit, R4-2; light DDS only, no MuJoCo, policy on CPU)
Why a clean clone: this working tree holds B2/C files that are not committed, so a check run here cannot see a file that is missing
from the commit (the round-4 review reproduced exactly that: a push that follows the rev-6.3 frontmatter ships an 84 whose
`g1sim/real_bringup.py` import fails). A local clone contains only what the commit contains. Why five 84 runs (R5-2 (4)): the crash
Track AF fixes was intermittent (6 of 11 runs on the old code), so one passing run proves nothing.
```
REPO="$HOME/Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot"
P=~/dev/g1-ext/robot-day-test
cd "$REPO"
# R3-4, R4-7: the one loopback DDS check of section 0 (copy its pattern exactly) - wait until it prints nothing; never kill
pgrep -fl "85_arm_sdk_sim|run_sim|g1_brainco_sim|sim_host|loopback_lowstate_pub|84_real_bringup|86_real_run|89_robot_day_selftest|pytest.*(test_e2e_real_adapter_sim|test_integration_arm_sdk_sim)" > runs/push1_pgrep.log 2>&1; echo "exit $? (1 = nothing running, wanted)"
CLONE=$(mktemp -d)/repo; git clone -q "$REPO" "$CLONE"; git -C "$CLONE" log --oneline -5 > runs/push1_clean_head.log 2>&1   # top = the fix-set commit
grep -n "def close_dds\|_open_channels" "$CLONE/g1sim/real_io.py" > runs/push1_clean_af.log 2>&1; echo "exit $? (0 = Track AF is in the commit)"
(cd "$CLONE" && $PY -m pytest tests/test_unit_real_io.py tests/test_unit_real_camera.py tests/test_unit_no_publisher.py tests/test_unit_real_factory_pose.py tests/test_unit_real_bringup.py tests/test_unit_runbook.py tests/test_unit_policy_wire.py tests/test_unit_real_safety.py tests/test_unit_real_safety_filter.py tests/test_unit_real_timing.py -q -p no:cacheprovider) > runs/push1_clean_unit.log 2>&1; echo "exit $?"
(cd "$CLONE" && source $P/robot-env.sh && python scripts/89_robot_day_selftest.py --policy-device cpu) > runs/push1_clean_selftest.log 2>&1; echo "exit $?"
(cd "$CLONE" && source $P/robot-env.sh && exec python tests/helpers/loopback_lowstate_pub.py --iface lo0 --seconds 600 --hands) > runs/push1_clean_pub.log 2>&1 & PUB=$!
: > runs/push1_clean_84_exits.log   # fresh exit list for this check
for i in 1 2 3 4 5; do
  (cd "$CLONE" && source $P/robot-env.sh && python scripts/84_real_bringup.py --iface lo0 --sim --yes-sim --steps probe,joints,hand-state,camera,record --seconds 10 \
      --camera-cmd "python tests/helpers/fake_camera.py --npz tests/data/selftest_episode.npz") > runs/push1_clean_84_run$i.log 2>&1; echo "run $i exit $?" >> runs/push1_clean_84_exits.log
done
kill $PUB; wait $PUB
grep -v " exit 0$" runs/push1_clean_84_exits.log > runs/push1_clean_84_bad_exits.log 2>&1; echo "exit $? (1 = all five runs exit 0, wanted)"
# lens 4: `grep -c` over five files exits 0 if ANY file matches, so check each run on its own
for i in 1 2 3 4 5; do grep -q "OWN WRITERS: 0 (must be 0)" runs/push1_clean_84_run$i.log || echo "run $i: no 'OWN WRITERS: 0'"; done > runs/push1_clean_84_writers.log 2>&1
test ! -s runs/push1_clean_84_writers.log; echo "exit $? (0 = every run printed OWN WRITERS: 0)"
grep -n "slow hand state" runs/push1_clean_84_run1.log runs/push1_clean_84_run2.log runs/push1_clean_84_run3.log runs/push1_clean_84_run4.log runs/push1_clean_84_run5.log > runs/push1_clean_84_slowhand.log 2>&1; echo "exit $? (1 = no slow-hand warning, wanted)"
# R3-1: the policy check used the prefix's own backbone cache
grep -n "hf_cache/hub" runs/push1_clean_selftest.log > runs/push1_cache.log 2>&1; echo "exit $? (0 = cache path printed)"
test -d $P/hf_cache/hub/models--HuggingFaceTB--SmolVLM2-500M-Video-Instruct; echo "exit $? (0 = backbone in the prefix)"
```
(`exec` makes `$PUB` the publisher's own PID, so `kill $PUB` stops it rather than only its subshell.) **Pass** = Track AF found in the
clone; unit tests exit 0 in the clone (A, E1/E2, B1 - `e914654` rides along, R18); `SELFTEST: PASS` with the `policy` line naming
`$P/hf_cache/hub` (R3-1) and `CHECK motion: INFO` (no 86 in push 1); **all five 84 runs exit 0** (`runs/push1_clean_84_exits.log`),
each log has `OWN WRITERS: 0 (must be 0)` (`push1_clean_84_writers.log` empty) and no log has "slow hand state" against
the helper (its hands now on deadline scheduling, Track RB). Logs go to this repo's `runs/` (the redirects run before `cd`); the clone's
own `runs/` is thrown away with the clone.
Earlier evidence: planner probe (18:21, scratchpad clone of `c0898cf`, no DDS): `84 --dry-run` exit 0, A + E1 unit tests 55 passed, 1
skipped. The DDS part, run by the main session 18:16-18:20 on `c0898cf`, found the intermittent crash (`runs/push1_84.log` 0 bytes) and
the false "4 Hz" hand rate (`runs/push1_84_rep1.log`) that Track AF fixes; AF's own repeats: 12 + 6 of 6 runs exit 0.
**Push 1 files (R5-2 (1)):** the frontmatter's Track A and E1 files (incl. A's four split files, R4-2) as committed in `c0898cf` and
`f897b0c`; B1's `e914654` (inert, R18); the fix-set commit = Track AF's three files, Track RB's runbook + publisher lines, E2's files
after "Track E2 - round 2" is GO (runbook Part C, 89, `tests/test_unit_runbook.py`, `docs/robot_day/volleyball_natural_phases.png`;
else E2's fallback); plus `COMMANDS.md`, `PROGRESS.md`, `docs/START_PROMPT.md`, `docs/LEARNING_PATH.md` as the main session prepares
them. The release exists already (15:09). In the push-1a fallback (6.1) the check is the three `setup_robot_laptop.sh` lines of Track
E1's verify (robot env, release, then the detached policy env) from a fresh prefix plus `tests/test_unit_runbook.py`, run from a clean
clone; push 1b re-runs this full check.
### 6.3 Integration (workflow phase, after all tracks)
Every DDS line below starts only after the section-0 DDS check prints nothing. (R5-2 (5)) The integration runs against the pushed
Track AF `real_io`: the first line records its hash, and `real-robot-integration.md` states that it equals the push-1 commit's.
```
(git hash-object g1sim/real_io.py; git rev-parse HEAD:g1sim/real_io.py) > runs/real_int_real_io_hash.log 2>&1; echo "exit $? (two equal hashes wanted)"
bash scripts/run_tests.sh > runs/real_int_run_tests.log 2>&1; echo "exit $?"   # all three envs; new DDS tests skip in .venv (m6-3)
$PY -m pytest -m "not e2e" -q > runs/real_int_suite.log 2>&1; echo "exit $?"
$DDS $PY -m pytest -m e2e tests/test_e2e_real_adapter_sim.py -v > runs/real_int_e2e.log 2>&1; echo "exit $?"
G1_E2E_40K=1 $DDS $PY -m pytest -m e2e tests/test_e2e_real_adapter_sim.py -k vla_40k -v > runs/real_int_40k.log 2>&1; echo "exit $?"
# 86 from the operators' env: 85 in the sim env, 86 replay from the fresh Python 3.10 robot env against it
(source $P/robot-env.sh && python scripts/86_real_run.py gate) > runs/real_int_gate_py310.log 2>&1        # LOCKED, exit 2
# plus one G2-style replay run with 86 started from $P/robot-env.sh (command written by Track C in the e2e file's docstring)
$PY scripts/87_arm_path_clearance.py --report runs/arm_path_clearance.json --demo tests/data/replay_demo.npz > runs/real_int_clearance.log 2>&1; echo "exit $?"   # RV6-1, R2-1, R3-2: policy_exits not null, all four waist limits present; runtime reported (R3-8; > 30 min -> fewer setbacks or one waist limit per run + 87 --merge, noted; never coarser start spacing, R5-1)
$PY -c "import json; s = json.load(open('runs/arm_path_clearance.json'))['summary']; assert s['max_sample_step_m'] <= s['MAX_SAMPLE_STEP_M'], s; assert s['max_start_step_m'] <= s['MAX_SAMPLE_STEP_M'], s; print('max sample step', s['max_sample_step_m'], 'max start step', s['max_start_step_m'])" > runs/real_int_clearance_step.log 2>&1; echo "exit $? (R4-1, R5-1)"
$PY -m pytest tests/test_unit_runbook.py -q > runs/real_int_runbook.log 2>&1; echo "exit $?"   # clearance table in Part C == the report (both-rows rule, R4-6)
```
If E2 left the "NOT cleared" placeholder, the integration phase fills runbook Part C's lookup from `runs/arm_path_clearance.json`
(recorded under "Integration fixes" in `real-robot-integration.md`, so the final gate re-checks it). The integration result states in
plain words, for the user's ruling R15, per waist limit (hold / 0.15 / 0.35 / 0.6): "replay exits clear at nominal: yes/no; frames
inside the table: N; fault exits clear (worst of straight and the two corner orderings; paths sampled and start poses spaced to 2 cm of arm movement)
up to factory elbow X rad".
### 6.4 Push 2 (expected overnight after C5 and after 00:00; target no later than ~08:00, latest 09:30 if Finnegan confirms that window)
- Always (if the final gate passed them): Track A2's edits, E2's edits after push 1 (the integration's lookup fill; all of E2's files
  if E2's fallback applied), other read-only fixes made after push 1 (Part C dry-walk fixes) and `docs/TESTING_GUIDE.md`; in the
  push-1a fallback also the Track A files and 89.
- Only if every track review, the integration run and the final gate are GO: B1, B2, C files incl. every reported split (push-set rule),
  then the unlock commit (4.3). Otherwise the motion files stay local and Monday is Parts A+B only.
- **Proven from a clean local clone (R4-2):** before `git push`, the 4.3 recipe's clone lines (`86 gate` UNLOCKED in the sim env and in
  the Python 3.10 robot env), and `89 --skip-policy` from that clone with `$P/robot-env.sh` (`runs/push2_clean_selftest.log`, after the
  section-0 DDS check), so a motion or split file missing from the commit shows up here, not on Monday; plus (R5-5) B1's and B2's unit
  tests from that clone (`$PY -m pytest tests/test_unit_real_safety.py tests/test_unit_real_safety_filter.py tests/test_unit_real_timing.py
  tests/test_unit_real_run.py tests/test_unit_motion_gate.py tests/test_unit_real_preflight.py tests/test_unit_real_send.py -q -p
  no:cacheprovider > runs/push2_clean_unit.log`), so a test file missing from the commit fails here too.
### 6.5 Cut list (in this order) and never-cut list
1. FOV intrinsics sub-check in 84 `camera` (applied already).
2. `84 record --policy` (86 `shadow` gives real-frame predictions when motion is unlocked).
3. 84 `questions` step (the runbook carries Q1-Q12 as a paper list).
4. e2e F8 and the 84-sends-nothing e2e (unit tests stay).
5. Monday `run` (VLA live): Part C stops after `shadow`.
6. Monday `replay` with the ball at the table (in-air replay stays).
(Not a cut but a rule: M4-M6 at the table are skipped whenever the lookup says so, rule 16; M6 at the table is skipped without
`fault_exits_clear`, ruling R15.)
Never cut: no-publisher rules for 84/89/71, frozen q0, release N2, protected factory record (incl. M0 in the Part C mode), limits,
envelope by phase, staleness, signals, 13/14 hold, the sample-based writer check, the motion gate, `SELFTEST: PASS`, the runbook boxes,
87's clearance check incl. the `policy_exits` block, its four waist columns, the corner orderings, the 2 cm sampling and the
movement-spaced start poses (R3-2, R3-7, R4-1, R5-1), the prefix-cache check of the policy self-test (R3-1), the policy exit through the ready pose (RV6-1, R2-1), the early sidecar
marker, the "release only after the table is pulled back" box (R2-1, R2-2), the clean-clone push checks incl. the five 84 runs
(R4-2, R5-2), the reviewed push-1 fix set (R5-2) and the head re-check before M4/M5 (rule 4.1 (3)).
### 6.6 Effort
Planned agent time stays as listed per track (A 4.5 h, B1 2.75 h, B2 5.25 h, C 5.5 h, E1 3.5 h, E2 1.5 h, A2 0.75 h), but the observed
pace is about 10x faster (6.1): A, B1, E1 and B2 built in 5-25 min each. The critical path is now C (largest track, sims, 87's 10-25 min
run) -> integration -> final gate: expected C5 ~01:45 (rev 7.1: push 1 and C start ~1 h later, R5-2), cutoff 07:30, so there are hours of slack before the ~08:00 push-2 target. The
real risks to the timeline are power (battery at 18:19) and the main session being re-invoked for the finish stage.

## 7. Monday for Cristina and Saad (the runbook follows this order)
**Rules on page 1:** run only runbook commands, exactly as written; never edit a file; the owner and the remote always win; on a FAIL read
that step's line, try only what it lists, once, then skip the step and its dependents; **every motion step: the robot hangs in the
gantry/harness (defined, R2-5: it stands on its own feet in the owner's arm_sdk mode and the harness is on the gantry with only a few cm of
slack), the owner holds the remote, 1.5 m clear around the arms, video on, one operator's hand on Ctrl+C**; press Ctrl+C once to
stop (twice only if an arm keeps moving towards a person); never `kill -9` or close the terminal while 86 runs. **One laptop all day
(R4-4):** 84's M0 step and every 86 command run on the same laptop (its `runs/` holds the factory record and 86's run markers); if it
dies while 86 runs: owner's remote, the owner re-stands the robot, then M0 on the new laptop. **The only 72 command is
`72_real_check.py kamera --suchen`** (R5-4); a `72 ...` hint in a FAIL line is not run. **Head (rule 4.1 (3)):** never move the
head by hand; the owner sets it; after Part B 3 nothing may move it. **The robot-PC password** comes from the owner on site and is
never written down or typed into 84.
- **Sunday evening after push 1 (expected ~19:30; the safer time, R4-3) or Monday 08:00 (Part A):** `gh auth login`, clone, `setup_robot_laptop.sh --robot-env
  --policy-env --release robot-day-2026-10-18`, `--show-ip`, the firewall step (macOS `socketfilterfw`, Ubuntu `ufw`; R3-6/m11), `89`
  until `SELFTEST: PASS` (robot cable unplugged; after a push 1a, 89 arrives with push 1b or push 2). Decide R11: which laptop runs 81.
- **Monday 08:00:** `git pull`; `89` again; `python scripts/86_real_run.py gate` -> UNLOCKED means Part C is allowed today; LOCKED or "no such
  file" means Parts A+B only.
- **Part B (read-only):** 0. static IP; `84 --steps net,probe,joints` (no factory record here, RV6-2: the record is taken in M0, in the mode
  Part C uses; note the FACTORY ARMS line). Abort: no LowState / below 200 Hz; samples on `rt/arm_sdk`; joint out of range. 1. `84 --steps questions` (Q1-Q12, firmware + mode).
  2. `84 --steps hand-state`. 3. table + camera (R5-2 (6), F21, RV-1..RV-4, Finnegan's facts):
     - robot: the owner sets the mode and stand height it will have at the table in Part C and keeps them; tape both feet (toe and
       heel). Hip height = floor to the hip pitch axis (centre of the upper-most hip motor cap, seen from the side), expected near
       0.65 m (re-measure outside 0.60-0.72 m, then ask the owner); **table top = 0.74 m + (hip height - 0.648 m)** (0.648 = pelvis
       origin 0.7505 - hip offset 0.1027, re-derived twice), never above 0.84 m.
     - table: front edge 24.5 cm in front of the hip axis (measured along the floor from a plumb point), centred, ball stand centre
       14.4 cm back / 24.1 cm left, basket centre 14.4 cm back / 24.1 cm right; tape the table's front corners (the nominal
       position; M0 adds a second mark if the M4 lookup gives a setback). **The table stays on its tape marks** (R19 (a) default).
     - head: the owner sets it with its own control to pan 0, 50 deg down; operators check only (phone flat on the D455 top face 49-51
       deg). Robot-PC login from the owner (ssh once by hand, `-o ConnectTimeout=5`); camera number via `72_real_check.py kamera
       --suchen`; then `84 --steps camera` and open the saved overlay: table centred left-right, front edge where the reference shows
       it. Off: re-check 24.5 cm and the 49-51 deg once (the owner corrects the head); still off -> write down what is off, do not move
       the table off its marks. From now on nothing moves the head.
  4. `84 --steps record` for 3-5 layouts (and `--label "owner_motion <tool>, <mode>"` if the owner offers, Q12; both operators stay
  outside the arms' reach (1.5 m) while the owner's tool moves them, r5-4; the owner stops the tool afterwards).
- **Part C (only if UNLOCKED; robot in the gantry; the arm_sdk mode agreed with the owner, Q8/Q10):**
  - M0 (RV6-2) the owner switches to the agreed arm_sdk mode and leaves the arms where the controller holds them; then
    `84 --steps joints --record-factory-pose --mode-note "<owner's Q8 answer>"` **before the first 86 command of the day**. On a
    "different factory pose recorded today" message: the owner confirms the arms are in the controller's hold pose and the operator
    re-runs once with `--overwrite-factory-pose`; 84 refuses that while our last 86 run may still hold the arms (it did not end with
    weight 0) and then says "run `86 release` first" (R2-2). The same M0 step is repeated whenever the owner changes the robot mode later
    in the day, and after a mode change or reboot the Part B 3 `84 --steps camera` line is re-run (head check, rule 4.1 (3)) and, if the
    stand height changed, the hip height is re-measured and the table re-set (RV-1). (R3-9) 84 also prints `PART B ARMS: gap ...` against the Part B step 0 reading; on its WARNING, if the owner did not
    change the mode, the owner re-sets the robot (or confirms their tool ended at weight 0) and M0 is recorded again. Then apply the
    lookup (rule 16) to the FACTORY ARMS line: first the bounds line (roll, yaw, wrist, waist yaw, pitch, elbow), then **both** elbow
    rows around the larger factory elbow (floor and ceil; "yes" only where both say yes, the larger setback, R4-6), and write the result (table position nominal / setback N cm / skip; M4 and M6 at the table yes/no for each waist column hold / 0.15 /
    0.35 / 0.6) on the paper sheet.
  - M1 `86 release` - expected: nothing visible moves. Abort: any jerk or swing -> no more motion today.
  - M2 `86 ready --hold 5` (arms in the air, table moved away) - arms rise slowly to the ready pose; compare with
    `docs/robot_day/volleyball_natural_phases.png` (m6-1). Abort: jerk at ramp in/out; pose mirrored or a joint moving the wrong way; printed lead > 0.10 rad
    at hold; the robot sways.
  - M3 `86 replay --server ... --episode replay_demo --speed 0.25 --waist-max 0.15` in the air, then `--speed 0.5`, then `--waist-max 0.35`,
    then (owner says yes) `0.6`. Abort: exit 3 or 4; the motion differs from the demo pictures; hands head into the body; the robot sways at
    a waist step (stay at the previous limit or use `--waist-hold`). Write the waist limit reached on the sheet (`hold`, 0.15, 0.35 or 0.6)
    and, per waist setting, the `MAX LEAD` line 86 prints (B1 review P1); the table steps need it below 0.15 rad (rule 16 (c)).
    Never use another waist value at the table: 87 checked only these four (R3-2).
  - M4 **precondition (RV6-1, R2-1, R3-2, rule 16 (a)-(c)):** the M0 sheet says "M4 at the table: yes" **in the column of the waist
    limit reached in M3** (or of a lower checked limit that says yes; table at the listed tape mark); "no", "skip" or no lookup numbers
    in the runbook -> skip M4-M6 at the table (Part C ends after M3; M5 `shadow` and M6 may still run in the air). **Before M4
    (RV-2, RV-4):** the robot's feet are on their tape marks, the 24.5 cm (+ setback) is measured once more, and the Part B 3
    `84 --steps camera` line is re-run (overlay as in Part B; else the owner re-sets the head, check again). Then the table in
    place, ball on its light stand: the same replay at 0.5 with exactly that waist setting (real data);
    the full demo takes 49 s at 0.5 and 97 s at 0.25 (default `--max-seconds 120`, m6-5). Abort: contact with the table (top or edge),
    including while the arms rise or lower; the stand knocked over or a finger snags. A brush that moves the stand < 2 cm is logged, not
    an abort.
  - M5 **before it:** the Part B 3 `84 --steps camera` line once more (head check, rule 4.1 (3)). 81 with the 40k on the R11 laptop;
    `86 shadow --server ... --max-seconds 30`. Abort: > 5 % of logged policy actions envelope-clipped,
    or NaN.
  - M6 **precondition (R2-1, R3-2, rule 16 (d), ruling R15):** at the table only if the M0 sheet says "M6 at the table: yes" in the
    column of the waist setting used; otherwise M6 at the table is skipped; M6 in the air is optional (no table to hit; it only shows whether the policy moves sensibly on real
    frames).
    `86 run --server ... --speed 0.25`, then `0.5`, same waist setting as M4, 60 s max per attempt, >= 3 filmed attempts. Abort: two exits 3/4
    in a row -> no more VLA today.
  - Every **soft** exit from replay/run (normal end, one Ctrl+C, exit 3) first lifts the arms back to the ready pose, opens the hands there
    (a held ball drops onto the table), then lowers them (rule 12). A **fault** exit (exit 4) instead lowers the arms from where they are
    within about 2 s and keeps the fingers as they are; if the ball is lifted, it may be pressed onto the table edge between the
    descending hands (R4-5: 87 does not model the ball) - that counts as contact with the table. Say all of it in the runbook so nobody
    reaches in.
  - After any crash, cable pull or abort: if it happened during M4-M6 (no final `86 <MODE>:` line, or `FAULT EXIT lowstate_stale`,
    R5-3), the rule 16 (e) box first (owner decides whether to pull the table back); then `86 release`, then `86 hands-open` if the fingers are closed. **Box: release refused or crashed -> owner's remote; we do
    nothing more.**
- **Optional, owner only:** walking the robot to the table is the owner's remote. `70_real_goto.py` is not run by the operators; mention it
  to the owner only if they ask for a demo of it.
- **Pack up:** copy `runs/` to the shared folder Finnegan names in his push-1 message (R19 (b)); none named: leave `runs/` on the robot
  laptop and delete nothing; send the JSON/npz names to Finnegan.

## 8. Real-data capture
84 `record` episodes (`commanded_by_us: false`, `source: real_observe`) and every 86 run (`source: real`, `actions` = filtered command,
`policy_actions` raw) -> real frames, real 27-D state, real replay-as-demo attempts for dataset v2.

## 9. Owner questions (asked with `84 --steps questions`)
Written out here because revision 5 was never committed (R3-6). Q1 is the backpack PC (Thunderobot) in use, and by whom? Q2 which computer
does the D455 hang on, over which USB port, is it busy with another program, and which `/dev/videoN` is its colour stream? Q3 (rev 7.1,
motorized head) how is the head module set (app/remote), can it be set to pan 0 / 50 deg down and left there, and what else can move it
(behaviours, apps)? Q4 does `brainco_hand_service` run on PC2 (`rt/brainco/*`)? Q5 SSH user/IP of PC2 (192.168.123.164) and may we use
it? (the password is given on site and never typed into 84 - answers are saved) Q6 is the
waist locked (roll/pitch) or free? Q7 does a built-in arm service run that writes `rt/arm_sdk`? Q8 is arm_sdk allowed, and in which mode
on their firmware (R2+A, R1+X, ...)? Q9 the robot's clock vs ours (log times)? Q10 the firmware version? Q11 what does their controller do
when arm_sdk messages stop (hold the last pose or ramp back)? Q12 will the owner run their own arm motion (tool, mode) for a recording?
Q8 and Q10 must be answered before Part C. Until A2 lands, 84's own Q2/Q3/Q5 texts are the older ones; the runbook's paper
list carries the rev-7.1 texts (Track RB).

## 10. Rulings (pending user ruling; the recommendation is the working default)
- **R2 gains:** sim gains (`SIM_GAINS`). *Recommend yes* (the policy learned with them).
- **R4/(e) slow factors:** replay 0.25 then 0.5, VLA <= 0.5, enforced off loopback. *Recommend yes.*
- **R5 envelope in the policy phase:** *recommend on.*
- **R6 hand cap 0.6:** *recommend yes* (demos never exceed 0.3).
- **R7 `--waist-hold` fallback:** *recommend yes* if the waist misbehaves.
- **R8 "debug mode" in CLAUDE.md rule 7:** read as "gantry, slow, owner at the remote, in the owner's arm_sdk mode". *Recommend yes.*
- **R10 (new) waist ceiling 0.6 rad on Monday**, escalation 0.15 -> 0.35 -> 0.6 only after a clean replay and the owner's yes. The ball sits at
  +35 deg (0.61 rad), so 0.35 makes a grasp unlikely; 0.6 is the smallest ceiling that gives the VLA a real chance. *Recommend 0.6.*
- **R11 (new) where 81 runs:** on the robot laptop if it is an Apple-silicon Mac, else the teammate laptop that passed 89's policy check.
- **R12 (new) release without a factory record:** refuse and hand over to the owner's remote. *Recommend refuse* (a blind ramp-out can swing
  the arms at > 0.3 rad/s, review N2).
- **R13 (new, m6-11) unlock only for its date:** the gate checks `date == today`, so an unlock never carries over to a later session.
  *Recommend yes.*
- **R14 (new, RV6-1) table steps when the straight path does not clear:** Monday either moves the table back within the layout or skips
  M4-M6 at the table (rule 16); a waypoint path that avoids the table edge is not built tonight (new motion logic would need its own
  review round, which the overnight window does not have) and goes to the next session with Finnegan present. *Recommend this.*
  Consequence the user should know: if the real factory elbows hang well down, the VLA attempt at the table will not happen on Monday.
- **R15 (new, review round 2 R2-1) VLA live at the table when a fault exit may touch the table:** a fault in the policy phase (contact,
  NaN, second Ctrl+C, stale LowState) hands the arm to Unitree's controller where the policy left it, and it drops towards its hold pose
  in about 2 s. If 87 shows that this drop from demo poses does not clear the table for the M0 factory elbow (`fault_exits_clear` false),
  Monday either (i) accepts that a fault may tap the fingers on the table edge (hardware risk on the owner's robot, not a risk to people:
  gantry, 1.5 m rule), or (ii) skips M4 **and** M6 at the table (M3 and M6 in the air stay). *Recommend (ii)*, the default the plan
  builds since rev 6.3 (R3-10 (b)): there is no evidence that faults are rare during a replay at the table - a real ball offset by the
  episode's randomization is the likeliest trigger of a lead (contact) fault, and that fault has the same drop. The cost: without
  `fault_exits_clear` Monday collects no replay data at the table. The less strict option the user may choose instead: M4 at the table
  with `replay_exits_clear` and `frames_inside.table == 0` only (accepting the drop risk during M4), M6 still skipped.
  What 87 can and cannot show (R3-7): the fault drop is checked along the straight line **and** the two extreme joint orderings, because
  Unitree's blend type (angles only vs torque-level) is unknown; that brackets the path but does not prove it, so the "contact with the
  table edge" abort stays. 87's numbers are per waist limit (R3-2), so the answer can differ between 0.15 and 0.6; every exit path is
  sampled to 2 cm of arm movement (R4-1) **and** the start poses along the demo are spaced by at most 2 cm of arm movement (R5-1), so
  a "yes" is not an artefact of coarse sampling of either the paths or the poses they start from. It remains a check of demo poses
  only (exact for M4; M6's policy may wander further). Residual risk the user should know
  (R4-5): 87 does not model the ball; a fault while the ball is lifted may press it onto the table edge between the descending hands
  (light, soft ball; counted as table contact, abort). The real fix, a "fault hold" (keep w = 1 at the measured angles until the operator confirms, then leave through
  the ready pose, then `release_plan`), is new motion logic and goes to the next session with its own review.
- **R16 (new, A code review minor 8) a joint resting a hair past its MJCF limit:** 84 and 86 both refuse a measured joint outside
  `JOINT_LIMITS`, so a real joint resting on its mechanical stop 0.01-0.02 rad past the sim limit blocks M0 and Part C. A tolerance only
  in 84 would move the dead end to 86; a shared tolerance would be a change to safety code tonight. *Recommend:* no tolerance tonight;
  the runbook (E2) says "84 `joints` FAILs on one joint within about 0.02 rad past its limit while it rests on its stop: do not record
  M0, note the joint and the value, Parts A+B continue"; the limit question goes to the next session with the real numbers.
- **R17 (new, A code review minor 6) sim and robot sidecars in one `runs/real`:** `arms_may_be_held_by_us` ignores the sidecar's
  `source`, so a loopback 86 run on the robot laptop on robot day could mask or fake a robot run's state. No runbook step runs 86 on
  loopback, and C's e2e uses `--out <tmp>`. *Recommend:* no code change tonight (it would touch A's reader and B2's caller together);
  the runbook's "never" box adds "never run 86 with `--yes-sim` on the robot laptop"; filter by `source` in the next session.
- **R18 (new, rev 7.1) B1 rides in push 1:** the main session committed B1 locally as `e914654` (18:44) on top of `c0898cf`, so a
  plain `git push` of `main` carries `real_safety.py`/`real_timing.py` with push 1. They are pure logic (no DDS, no publisher, no I/O),
  nothing in push 1 imports them (84/89 do not; 86 is not in push 1), and motion still needs 86 + Finnegan's unlock commit. *Recommend:*
  accept (6.2 runs B1's unit tests in the clean clone). The alternative - pushing only up to the fix-set commit without B1 - needs a
  rebase by Finnegan tonight and buys no safety.
- **R19 (new, rev 7.1; open user questions from the Parts A+B walk, R5-4):** (a) may the table move to match the camera picture, and
  by how much? *Recommended default (Finnegan's confirmation pending):* no - the table stays on its Part B tape marks and only the
  camera is corrected within 49-51 deg, because 87's lookup is computed for the tape-mark position; a remaining overlay offset is
  written down. (b) the shared folder for `runs/` at the end of the day: *recommend* Finnegan names it in the push-1 message; none
  named -> `runs/` stays on the robot laptop, nothing is deleted.
- **Facts, not rulings (Finnegan, rev 7.1):** the head is motorized and the owner sets it (rule 4.1 (3)); the robot-PC SSH login is
  given on site by the owner and never written down.
- **Unlock (Finnegan, after 00:00 on 2026-10-19 and after a final-gate GO):** unlock only if the final gate says GO for every motion
  file; otherwise Monday is read-only.

## 11. Honest expectations
- Likely: Part B fully done; release, ready and a slow replay run clean; shadow numbers on real frames; a few VLA attempts that reach towards
  the ball and end in clean exits. The sim-only 40k (36/50 in sim) will most likely **not** grasp on the first real attempts.
- Real risks: the factory arm pose hangs the forearms low enough that the straight path to the ready pose crosses the table edge, so
  the table steps are skipped (RV6-1, 87's lookup decides on the day); the drop after a fault from a grasp pose does not clear the table,
  so the replay and the VLA attempt at the table are skipped (R2-1, R15 stricter default since rev 6.3) - then Monday's VLA result is
  in the air and on real frames only; a waist limit below 0.6 (owner says no, or the robot sways) changes 87's answer (R3-2); the firmware mode gives no arm control (issue #156); the owner cannot set or hold the motorized head at pan 0 / 50 deg, or something moves it after Part B (the head re-check before M4/M5 catches it; then M4-M6 data is not from the training view); the policy laptop is slow over Wi-Fi
  (latency skip + 2 s timeout handle it, with soft exits); Ubuntu setup is untested here (Sunday's `SELFTEST` catches it); the motion gate
  stays locked because a review is not GO (then Monday is still useful: Part B + data).

## 12. Review lenses (harness template)
1. *Guard coverage:* the gate runs in every publishing mode incl. `release` (integrity matters most in recovery code); the sim bypass needs
   loopback + `--yes-sim` + no robot address; `finally` + signal flags cover in-process exits; `kill -9`/crash/cable pull are covered by the
   factory-pose check + `release`; `release` itself failing is covered by the gantry + owner's remote box.
2. *Newly reachable:* non-authors running publishing code (gate a-f); a policy server on another laptop (timeout 2 s, latency skip, soft
   exit); 86 under Python 3.10 (integration runs it there); 84 writing a factory record that 86 now trusts (date + tolerance + no silent
   overwrite + separate sim file); waist yaw up to 0.6 (escalation + rate limit 0.3 rad/s).
3. *Meaning changes:* the envelope means "policy targets only"; 86's `actions` are the filtered command (raw in `policy_actions`);
   `replay_demo.npz` has blank images (81 uses images only for diagnostics; stated in its sidecar); `runs/factory_pose_*.json` is a safety
   input, not a log.
4. *Primitives:* `git diff --quiet <commit> -- <path>` cannot see a file missing from the commit (or an untracked file), so the gate
   also runs `git cat-file -e` - for (b) on the pinned commit and for (a) on `HEAD:docs/ROBOT_DAY_MOTION.json`, and (a) reads the JSON
   through `git show` (R3-3); `os.getcwd()`-relative paths break when 86 runs from another folder, so every safety file is anchored to
   the repo root (R3-5); `HF_HUB_CACHE` set by `setdefault` silently uses Finnegan's cache, so the operator path sets it explicitly (R3-1);
   `cyclonedds.pub` is always imported by the SDK, so the guard patches names; smoothstep over 100 ticks peaks at 0.015 < 0.02 per message;
   cyclonedds 0.10.2 wheels need Python <= 3.10; 81 has no `cuda` today.
5. *Shared consumers:* `real_io` is imported by 84, 89, 86, `sim_host` and the loopback helper - its contract is frozen here (changed
   once, by Track AF in rev 7.1: readers kept until `close_dds` at exit, `rate_hz` over listening time; lens 10 (b)); 81 gets one
   choice added (policy_wire tests re-run); LeRobot's persistent `self.msg` is only blended through a copy in 85.
6. *Rev 6.1 second-order checks:* the exit through the ready pose changes every replay/run exit path, and the e2e G2 + B1/B2 tests pin
   it; `motion_ran_today` (rev 6.2: `arms_may_be_held_by_us`) reads 86's sidecars, so 86's log-name contract (3.4) is now a safety input for 84 (A tests it with a fake
   sidecar, C's e2e uses `--out <tmp>` so sim runs never block an overwrite); 87's lookup is only as good as the sim kinematics and a
   straight-line approximation of the factory controller's path, hence the 5 cm margin and the "contact with the table edge" abort; the
   gate's date check makes a 07:30 unlock fail if pinned with yesterday's date (the recipe writes 2026-10-19).
7. *Rev 6.2 second-order checks:* 87 is now the only evidence for every exit that starts at the table, so its pass rule (per object,
   against the start clearance) is test-pinned with a mutation, and its inputs (`replay_demo.npz` from E1, `move`/`sdk_targets` from B1)
   are integration-checked (6.3 requires `policy_exits` not null). The overwrite rule now trusts 86's sidecar fields, so the sidecar is
   written before the first publish (B2 mutation test) and read by A with fake sidecars; a run in the sim with `--out <tmp>` still never
   touches `runs/real`. Allowing the overwrite after any run that ended at weight 0 relies on the linear-blend assumption (weight 0 =
   controller in full control); the owner's confirmation that the arms are in the hold pose stays the human check. Defining "hangs" as
   "stands, harness on the gantry with a few cm of slack" keeps the table-height formula (Part B 3) valid. The detached policy-env install
   runs after the release unpack, so the two never write the same prefix at once.
8. *Rev 6.3 second-order checks:* the waist dimension multiplies 87's work by up to 4 (reuse of unchanged start poses + pre-filter keep
   it at an expected 5-12 min; the runtime is reported, not gated) and turns the runbook lookup into a 4-column table, so E2's
   report-vs-runbook test checks every column and Part C forbids waist values 87 did not check. The stricter R15 default makes M4 at the
   table depend on `fault_exits_clear` too, so Monday may collect less table data; the less strict option is named for the user. 84's new
   refusal to record a factory pose while `rt/arm_sdk` samples arrive interacts with C's F8 fixture (it records first, then starts the
   foreign writer) and with Q7 (a built-in arm service that writes `rt/arm_sdk` also blocks 86's preflight, so Part C would not run
   anyway). Anchoring `RUN_LOG_DIR` to the repo makes 86's robot sidecars land where 84 looks, and the `--out` refusal off loopback keeps
   it that way; on loopback `--out <tmp>` stays allowed for C's e2e. Reading the unlock JSON through `git show HEAD:` means Finnegan's
   recipe must commit it before `86 gate` (it does: the recipe commits, then runs `86 gate`). The prefix-cache assertion makes E1's
   policy self-test fail here if the setup's download failed, which is the point (R3-1). The loopback DDS check of section 0 serialises
   the main session's push-1 check with C's sims; it can delay push 1 by up to one C test run (minutes), which is cheaper than a false FAIL.
9. *Rev 7 second-order checks:* densifying 87's paths (R4-1) raises its sample count only where the arm moves fast (expected 2-3x,
   10-25 min; rev 7.1 replaced the `--frame-step 20` fallback, which coarsened the start-pose spacing (R5-1), by fewer setbacks or one
   waist limit per run + `87 --merge`), and its step bound counts a geom's
   swing (`rbound x angle`), not only its centre, so long forearm capsules are not under-sampled; the pass rule itself is unchanged, so
   E2's lookup and its report-vs-runbook test keep their shape. Proving pushes from a clean clone (R4-2) makes the check independent of
   what is untracked here; it needs the robot-env prefix `$P` (exists) and the section-0 DDS check, and it writes logs here and scratch
   output in the throw-away clone. The unlock's date check now forces the unlock commit after 00:00, so an early final gate means a
   locked motion push first - fail-safe, not a dead end. A2 edits A's files after push 1, and 86 loads `real_io`, so gate (b) refuses
   while an A2 edit is uncommitted; the unlock recipe's `git status` line (now also over `tests/helpers`) stops the unlock first.
   Reading two lookup rows (R4-6) can only turn a "yes" into a "no". The one-laptop rule (R4-4) and the R16/R17 runbook lines are text
   only. B2's new `MAX LEAD` line arrives as a review finding to a track built on rev 6.2; the runbook names a stand-in if it is missing.
10. *Rev 7.1 second-order checks:* (a) Movement-spaced start poses (R5-1) can only add start poses (~86 instead of 73) and so can
   only turn a lookup "yes" into a "no" - including `frames_inside.table`, which now also counts a pose between two old 10-frame checks
   that starts inside the table top; an interpolated start pose is a joint-space blend of two consecutive 30 fps demo frames, which the
   replay passes within `MAX_SAMPLE_STEP_M`, so it adds no pose the arm never reaches. The report's row field `frame` turns fractional;
   E2's lookup test reads only `summary["elbows"]` and the per-(setback, L) summaries, so it is unaffected. Dropping `--frame-step`
   touches no runbook command (87 is not an operator command; E1's flag test checks runbook commands only). (b) Track AF's
   `close_dds` at `atexit` reaches every `real_io` consumer (84, 86, 89, `sim_host`, the loopback helper): it must be safe without an
   initialised participant (unit tests), and in 86 it must run only after the `finally` ramp-out and the joined policy worker - B2's
   re-review and C's e2e (F6, K: last weight-0 message before process end) check it against the working-tree `real_io`; motion gate
   (b) pins the AF `real_io` because AF is committed before any unlock. `rate_hz`'s new meaning changes the number 84 prints and the
   `warn`/`refuse` decisions only for young readers (a stopped stream still reads ever lower). (c) Moving E2 into push 1 puts Part C
   text in the operators' hands tonight; it is inert (`86 gate` LOCKED or absent) and says "NOT cleared" until integration fills the
   lookup, but operators may read it early - Finnegan's push-1 message says "Part C is for Monday and only if UNLOCKED". (d) The head
   rule adds three `84 --steps camera` runs on Monday (each asks for the robot-PC password within 30 s; about 1 min each) and makes a
   head move visible but not impossible (no DDS view of the head; 86 has no framing check). (e) `lowstate_stale` read as "may be held"
   (A2) blocks a factory-pose overwrite after a cable-pull fault until `86 release` ran (or the owner re-stood the robot) - the safe
   direction; until A2 is pushed the runbook box wording (E2) covers it by hand. (f) B1 in push 1 (R18) changes no operator path.
   (g) The five clean-clone 84 runs add ~3 min to 6.2 and need the publisher for 600 s (killed in the same block).
- Habits: every [mutation] check is seen failing first; e2e failures are compared by name.

## Review response and revision history
Revision 5 was GO (rev-5 review) for the read-only Sunday. Revision 6 brings motion back, so every earlier motion finding is re-applied:
| Finding | Status in rev 6 | Where |
|---|---|---|
| B1 envelope on scripted moves | Fixed | 4.2 rule 5, `ENVELOPE_PHASES`, B1 test `test_envelope_clips_only_in_policy_phase`, e2e G2 |
| M1 no recovery after death with w > 0 | Fixed | 4.2 rules 2, 13; `release_plan`; e2e K; runbook box |
| M2 fault keeps pushing | Fixed | 4.2 rule 12, `fault_targets`, B1 + B2 tests |
| M3 200 Hz vs 127.8 Hz sim | Fixed | `LOWSTATE_HZ` loopback 100 / robot refuse 200, warn 400 (n1) |
| M4 discovery-based writer check | Fixed | `active_writers` counts samples; `rt/lowcmd` samples warn (n3); B2 + e2e F8 |
| M5 waist escalation / R7 switch | Fixed | rule 6b, `--waist-max` / `--waist-hold`, Part C M3, R10 |
| M6 tilt target | Fixed | 2.4, 50.0 deg world pitch |
| M7 / N3 / R4-7 schedule | Fixed | section 6: five parallel tracks, clock checkpoints C1-C5, push contents, cut list |
| m1 sign check | Changed | sweep + sheet dropped (84 is read-only); ready-pose comparison + slow replay with Ctrl+C at 0.25 (Part C M2-M3); residual risk: joints that sit at 0 in the ready pose are only checked during the replay |
| m2 hands on exit | Fixed, changed as asked | soft exit opens the hands actively; fault exit holds; `86 hands-open` |
| m3-m5, m10, m11 | Fixed | GRABBER 3.8 test, 15 fps, colour test, light stand rule, firewall note in the runbook |
| m6 in-place blend | Fixed | 85 blends into a copy; C test `test_85_blends_weighted_arm_sdk_into_a_lowcmd_copy` |
| m7 slow factors in code | Fixed | gate (e), `speed_refusal` |
| m8 process robustness | Fixed | SIGTERM/SIGHUP = Ctrl+C, `caffeinate -i`, gap < 60 ms |
| N1 per-tick re-read at ramp-in | Fixed | rule 3, `ramp_in(q0)`, B2 sagging-arm test [mutation], e2e G1 (< 0.03 rad) |
| N2 release swing + lead check in blend | Fixed | rule 13, `release_plan`, lead only at w == 1, e2e K peak < 0.35 rad/s |
| N4 factory record stale/overwritten | Fixed | 3.1 factory-pose functions (date, tolerance, conflict, separate sim file), `--record-factory-pose` before any motion |
| n2 lead margin | Fixed | e2e G2 max lead < 0.15; Monday abort at lead > 0.10 during `ready` |
| n4, n5, n6 | Fixed | waist limit carried into M4; brush definition (< 2 cm); wording kept |
| R4-1 gate covers too few files | Fixed | 4.3 (b): every loaded repo file + JSON vs the pinned commit, `loaded_files_at_exit` check |
| R4-2 no on-site recovery | Fixed | gantry for every motion step (gate d), no kill test (f), "release refused/crashed" box |
| R4-3, R4-4, R4-5, R4-6 | Kept from rev 5 | 4.1, loopback helpers, Python 3.10 robot env, never box |
| r5-1 lo0 before LeRobot import | Folded in | 2.5, 3.5 |
| r5-2 re-check robot address every second, bound iface | Folded in | 3.5, 3.6 |
| r5-3 KEEP_LAST readers | Folded in | 3.1 |
| r5-4 operators outside arm reach during owner motion | Folded in | runbook Part B step 4 |
| r5-5 uv for Python 3.10 on Ubuntu | Folded in | 2.7, Track E |
| r5-6 staffing | Superseded | one agent per track; A's sim_host moved to C; push-1 check by the main session |
Refuted: none. Revision 6 history: Monday instead of Sunday, overnight window, two pushes, Tier B back behind the motion gate, five build
tracks, test data from the internal-disk dataset instead of the T7.

## Review response (rev 6 round 1) -> revision 6.1
Every finding of "Plan review rev 6 - round 1" (`.planning/reviews/real-robot-plan.md`) is fixed in the plan text; none is refuted.
| Finding | How it is fixed | Where |
|---|---|---|
| **RV6-1** arm paths vs. the table at M4-M6 | (a) Track C adds `87_arm_path_clearance.py`: FK only (`mj_forward` + `mj_geomDistance`) in the `volleyball_walk` nominal layout, the exact `real_safety.move` line from hanging poses (elbow 0-1.6, shoulder pitch -0.1/0.1/0.3) to `BALL_READY_POSE`, min clearance to table/stand/basket, setback needed for 5 cm, largest setback that keeps the layout; 3 tests incl. a planted path through the table [mutation]. (b) Every soft exit from the `policy` phase goes through `BALL_READY_POSE`, opens the hands there, then lowers to q0 (`soft_exit_plan`, B1 + B2 + e2e G2 tests). (c) Rule 16 + runbook M4 precondition: the M0 FACTORY ARMS line is looked up in 87's table -> nominal / setback / skip the table steps; without 87's numbers M4-M6 are not cleared. Ruling R14 states that a waypoint path is not built tonight. | 1, 2.7b, 3.1 (`factory_arm_summary`), 3.3 (`soft_exit_plan`), 3.4, 3.5 (87), 4.2 rules 12 + 16, Tasks A, B1, B2, C, E2, 6.3, 6.5, 7 (M0, M4), 10 (R14), 11, 12 |
| **RV6-2** factory pose recorded in the wrong mode | Part B no longer records; new Part C step M0 records it after the owner sets the agreed arm_sdk mode and before the first 86 command, with the owner's Q8 answer as `mode_note`. A conflict allows `--overwrite-factory-pose` once only while no 86 command has run today (`motion_ran_today`, enforced in 84, tested [mutation]); 84's conflict message and 86's gap refusal both name M0 and `86 release`. | 1, 3.1, 3.4, 4.2 rule 2, Task A (flags + 2 tests), Task B2 (1 test), E2 (M0 + test), 7 (Part B 0, M0) |
| **RV6-3** push-1 timing and fallback | Clock re-derived from 16:37 (when this revision was finished) and the workflow's serial build -> review -> fix: build from 17:15, push 1 "when A and E1 are GO", expected about 22:30 (22:00-23:00), latest 23:30. Track E is split: E1 = push-1 items only (3.25 h), E2 = runbook Part C, 89's motion line, phases image (after E1 GO). The "E only" fallback is replaced by one based on what is actually GO (E1 only -> setup + runbook Part A; nothing GO -> setup on Monday 08:00). The read-only dry walk (Parts A+B) moves to right after push 1, so only Part C waits for integration; slack before C5 stated as 30-45 min (about 0 without that move); push 2 may slip to ~08:00 with the pull at 08:30. | Situation, 5 (intro, E1, E2), 6.1, 6.2, 6.4, 6.6 |
| m6-1 phases image ignored by git | E2 copies it to `docs/robot_day/` and M2 points there; verify runs `git check-ignore` | frontmatter, E2, 7 M2 |
| m6-2 data script env + image key | runs in `$SV`; `observation.images.camera1` (PNG bytes) -> `head_image` | E1 step 1 + verify |
| m6-3 collection in `.venv` | `pytest.importorskip` at the top of every DDS/cv2 test file; integration runs `scripts/run_tests.sh` | 0, A, B2, E1, 6.3 |
| m6-4 factory record expires at midnight | every e2e test records its own sim factory pose and uses `--out <tmp>` | C step 3 |
| m6-5 replay stuck at tick 0; 60 s too short | `tick = int(mark_requested())`; G2 checks the replayed actions and rising ticks; replay `--max-seconds` default 120 | 3.3, 3.4, B2, C, 7 M4 |
| m6-6 replay without hand state | fill with the last hand command, `hand_state_synthetic` | 3.4, rule 10, B2 |
| m6-7 hands and the envelope | hands: `HAND_CAP` only, never envelope-clipped or counted | 3.3, B1 test |
| m6-8 `MAX_WEIGHT_STEP` owner | checked by B2's runner on every message; release's first message is the documented exception | 3.3, B2 |
| m6-9 6 s soft-exit cap | removed; the exit path always completes; `EXIT_PATH_MARGIN_S` only catches a stuck loop | 3.3, rule 12, B1 test |
| m6-10 `caffeinate` is macOS-only | Ubuntu `systemd-inhibit --what=sleep:idle:handle-lid-switch`, "never close the lid" | rule 15, E1 runbook |
| m6-11 unlock recipe unchecked | `git status --porcelain -- g1sim scripts` empty before pinning, `86 gate` must print UNLOCKED before the push; gate checks `date == today` (R13) | 4.3, B2 test, 10 |
| m6-12 one review file for five reviewers | one findings file per gate: `real-robot-code-<track>.md`, `real-robot-integration.md`, `real-robot-final.md` | 0, 5 intro |
| m6-13 runbook flag test vs. absent 84 | the skip applies to every missing script | E1 step 6 |
| Environment: laptop on battery | not plan content; STATUS 15:34 confirms AC power | - |

**Seams the main session must carry into the workflow script (not plan files):** (1) Track E is now E1 (parallel) + E2 (after E1's GO);
(2) the review prompts write `real-robot-code-<track>.md`, integration `real-robot-integration.md`, the final gate `real-robot-final.md`;
(3) the push-1 trigger is "A and E1 GO"; (4) the runbook dry walk splits: Parts A+B right after push 1, Part C after integration.
Status at rev 6.2 (checked in `.planning/robot-day-build.workflow.js`, 17:05 copy): (2) done (lines 115, 118, 155, 176), (4) done
(line 163 walks Part C + changed A/B lines; Parts A+B by the main session before push 1, STATUS 17:05), (1) done (TRACKS default
A, B1, B2, C, E1, line 109; E2 chained after E1). Still open for the main session: (3) a push-1 signal while E2 and the B tracks still run
(the A+E1 build run must log "track A: GO" / "track E1: GO" as soon as each is GO, or A and E1 run as their own stage, R2-4 (b)), and
(5, R2-6) E1's detached policy-env install with its PID file.

## Review response (rev 6 round 2) -> revision 6.2
Every finding of "Plan review rev 6 - round 2" (`.planning/reviews/real-robot-plan.md`) is fixed in the plan text or handed to the main
session where it is not plan content; none is refuted.
| Finding | How it is fixed | Where |
|---|---|---|
| **R2-1** (major) exits from a policy pose at the table not covered by 87 | (a) 3.5 corrected: block 1 `ready_paths` covers ready <-> hanging only. (b) 87 block 2 `policy_exits`: every 10th demo frame (arms + waist from `replay_demo.npz`), `to_ready` exit (case 3) and the straight line to h(e, s) for the grid (cases 1 and 2), every 5th tick, setbacks 0 + each needed setback, per-object pass rule "never closer than 5 cm or than at the start", summary `replay_exits_clear` and per elbow `fault_exits_clear`; one more C test with a mutation; integration requires the block. New assumption: under the linear blend the ramp-out target lies on that straight line. (c) Rule 16 (d) + runbook box: after a crash, kill or cable pull during M4-M6 the owner decides whether to pull the table back before `86 release`, else the owner's remote; after exit 4 nothing should move. (d) Ruling R15: without `fault_exits_clear`, M6 at the table is skipped (default); M4 residual risk stated with the stricter option. Rule 12 and R15 name the "fault hold" for the next session (no new motion logic tonight). | 1, 2.7b, 3.5, 4.2 rules 12, 13, 16, Track C, E2, 6.3, 6.5, 7 (M0, M4, M6, exits), 10 (R15), 11, 12 (lens 7) |
| R2-2 (minor) overwrite dead end; marker written too late | (i) `motion_ran_today` replaced by `arms_may_be_held_by_us`: the overwrite is allowed after any run that ended with weight 0 sent (broader than the reviewer's "release with exit 0": soft and fault exits also end with weight 0 sent, and under the linear-blend assumption weight 0 hands the arms fully back to the controller; what matters is the last weight we sent, not the mode), with the owner's confirmation. (ii) 86 writes the sidecar with `exit_code: null`, `arm_sdk_writer` before its first writer (temp file + rename) and again at exit with `last_weight_sent`. (iii) the gap refusal offers M0 only when our code does not hold the arms, else "run `86 release` first". (iv) A: two tests (refused while held [mutation], allowed after weight 0); B2: two tests (marker before first publish [mutation], gap message). | 3.1, 3.4, 4.2 rule 2, Tasks A, B2, 7 M0 |
| R2-3 (minor) lookup keys only on the elbow | `factory_arm_summary` adds shoulder roll/yaw and the largest wrist angle; 87 reports `GRID_BOUNDS` and `FACTORY_LOOKUP_TOL` 0.15 rad; the runbook's bounds line skips the table steps outside them; `setback_max_policy_m` added and used for M6. | 2.7b, 3.1, 3.5, 4.2 rule 16 (a), (c), Task A test, E2, 7 M0 |
| R2-4 (minor) timeline and seams | (a) re-timed from an ~18:05 build start with a fix round for every track: push 1 expected ~23:45 (23:15-00:15), push 1a/1b split when only E1 is GO, C3 cutoff 02:30, C4 cutoff 05:15, C5 07:15, push 2 target ~07:45 and latest 09:30 because Part B fills Monday's first hour. (b) script seams re-checked: (1), (2), (4) are in the 17:05 script copy; (3) the push-1 signal and (5) the detached install stay with the main session (seams paragraph above). (c) Part A wording: the main session's Parts A+B dry walk before push 1 (STATUS 17:05) reads it. | Situation, 6.1, 6.2, 6.4, 6.6, seams paragraph |
| R2-5 (minor) "hangs in the gantry" undefined | Defined: stands on its own feet in the owner's arm_sdk mode, harness on the gantry with only a few cm of slack; HANGING typed only for that state; on runbook page 1 (E1) and in gate (d). | 4.3 (d), E1 step 5, 7 page 1 |
| R2-6 (minor) E1 verify vs section 0; fragile background install | Section 0 allows E1's CPU-only policy self-test; E1 runs robot env, then `--release-file`, then the policy env detached with `nohup ... & echo $! > runs/real_e_setup_policy.pid`; done names the PID file if still installing. | 0, Track E1 verify + done, 6.2 |
| Environment: battery | Not plan content. `pmset -g batt` at 17:20 shows "AC Power, 100 %"; the main session re-checks before launching the build. | Situation |

## Review response (rev 6 round 3) -> revision 6.3
Every finding of "Plan review rev 6 - round 3" (`.planning/reviews/real-robot-plan.md`) is fixed in the plan text or handed to the main
session where it is not plan content; none is refuted. Rule 16's items are renumbered: (b) is the new waist column, old (b) -> (c),
old (c) -> (d), old (d) -> (e) (the history rows above keep the old letters).
| Finding | How it is fixed | Where |
|---|---|---|
| **R3-1** (major) the fresh-prefix policy check cannot fail on this Mac | Contract: `--policy-env` downloads `smolvla_base` + SmolVLM2 into `<prefix>/hf_cache/hub`; `policy-env.sh` exports `HF_HUB_CACHE` to it; 89's `policy` check passes that cache explicitly, FAILs when the SmolVLM2 folder is missing and names the cache on PASS. E1's verify asserts `HF_HUB_CACHE == $P/hf_cache/hub` and both backbone folders before 81 runs, logs the variable and greps it; the push-1 check greps 89's log for the cache path and tests the folder; one E1 test (`test_selftest_policy_fails_without_backbone_in_prefix`). E1's in-progress setup script and 89 already do most of this (re-read 17:47: `setup_robot_laptop.sh:96-106`, `89_robot_day_selftest.py:144`); the plan now pins it so the reviewer can hold E1 to it. | 2.7, 3.6, E1 steps 2-3 + 6 + verify + done, 6.2, 6.5 |
| **R3-2** (major) 87 checks exits only at waist 0.6 | 87 `--waist-limits 0.0,0.15,0.35,0.6` (`WAIST_LIMITS`; 0.0 = `--waist-hold`): block 2 clips `state[26]` to each limit; `replay_exits_clear`, `frames_inside` per (setback, limit), `fault_exits_clear` per (elbow, limit). Rule 16 (b): one lookup column per limit; M4/M6 at the table use exactly one checked setting, never above the M3 limit; a lower checked "yes" column may be used. E2's lookup has per-column cells and the test checks every column; Part C M3 records the limit reached and forbids other values at the table. Bounds line and `factory_arm_summary` add the waist yaw (planner addition: 87's hanging poses assume waist 0). One C test (per-waist rows). Restores rev-2 n4. | 1, 3.1, 3.5, 4.2 rule 16 (a)-(d), A step 1 + test, C test, E2, 6.3, 7 (M0, M3, M4, M6), 10 (R15), 11 |
| R3-3 gate (a) passes an untracked JSON | (a) requires `git cat-file -e HEAD:docs/ROBOT_DAY_MOTION.json` and parses the JSON from `git show HEAD:...`; B2 tests `test_gate_refuses_untracked_motion_json` [mutation: drop the cat-file check] (untracked file, as the reviewer showed) and `test_gate_reads_motion_json_from_head_not_worktree`; lens 4 updated. | 4.3 (a), B2 step 2a + tests, 12 lens 4 |
| R3-4 push-1 check and C's sims share DDS domain 0 on lo0 | Section 0 rule: one loopback DDS run at a time, with one `pgrep` pattern every DDS user checks first (wait, never kill). The push-1 check starts with that `pgrep`; Track C (builder, reviewer, fixer) runs it before every sim start. | 0, C action, 6.2, 12 lens 8 |
| R3-5 cwd-relative safety paths, one fails open | `REPO` anchor in the contract: `RUN_LOG_DIR`, `BRINGUP_REPORT_DIR`, `factory_pose_path` absolute (A's in-progress `real_factory_pose.py` already anchors `factory_pose_path` and resolves a relative `log_dir` against `REPO`, but `RUN_LOG_DIR` is still the string "runs/real", re-read 17:50). 86: relative `--out` resolved against the repo; off loopback an `--out` outside `RUN_LOG_DIR` is refused. A test `test_sidecar_found_when_86_ran_from_another_cwd` [mutation]; B2 test `test_out_dir_outside_run_log_dir_refused_off_loopback`. | 3.1, 3.4, A step 1 + test, B2 step 2a + test, 12 lens 4 |
| R3-6 fixes that lived only in "as revision 5" | Every "as revision 5" is replaced by text: 3.2 (contract was already complete), 3.6 (setup outputs + 89 checks written out), A step 2 (all eight 84 steps), E1 steps 2-3, section 9 (Q1-Q12 written out). m11 (firewall step: macOS `socketfilterfw --getglobalstate`, allow the robot-env python; Ubuntu `ufw`) and r5-4 (operators outside arm reach during owner motion; label records tool and mode) are now in E1 step 5 and section 7. R4-6 (`START_PROMPT.md:100` still names 70 as the next real-robot step) is main-session docs: seam below. | 3.2, 3.6, A, E1, 7, 9, seams |
| R3-7 straight-line stand-in assumes a q-only blend | 87's `to_hanging` takes the worst of `straight`, `proximal_first` and `distal_first` (waist yaw + shoulders vs elbow + wrists); the assumption in section 1 says why and that this brackets, not proves; R15 tells the user; summary carries `BLEND_NOTE`. `release` after a crash and the `to_ready` leg are our own weight-1 commands, so they keep the straight line. One C test [mutation: straight line only]. | 1, 3.5, C test, 10 (R15) |
| R3-8 87's runtime | Collision geoms only; per sample a numpy bounding-sphere pre-filter, `mj_geomDistance` only within `distmax`; start poses reused across waist limits when clipping leaves them unchanged; 87 prints its sample count and elapsed time; the runtime is reported, not a done criterion; > 30 min -> integration re-runs with `--frame-step 20` and records it (superseded in rev 7.1 by R5-1: no `--frame-step`). Planner's timing probe (FK only, no stepping, `runs/planner_fk_bench.log`): 53 arm/hand and 22 object collision geoms, `mj_forward` 70 us, ~182 us per sample with the pre-filter (108 distance calls at the ready pose); about 226k samples per (setback, limit) -> expected 5-12 min. | 3.5, C done, 6.3 |
| R3-9 M0 can record a pose the owner's tool still holds | 84 `--record-factory-pose` refuses while `rt/arm_sdk` samples arrive, and compares with today's first `joints` reading (`earliest_factory_arms_today`, Part B step 0) and prints a WARNING above `FACTORY_POSE_TOL`; runbook M0: on the warning without a mode change, the owner re-sets the robot (or confirms the tool ended at weight 0) and M0 is recorded again. A tests: refusal [mutation] + warning. C's F8 fixture records before starting its foreign writer. | 3.1, A step 2 + tests, C F8, E2 M0, 7 M0, 12 lens 8 |
| R3-10 (a) frames inside the table | 87 reports `frames_inside` per (setback, limit, object); M4 at the table needs the table count 0. | 3.5, 4.2 rule 16 (c), E2 |
| R3-10 (b) "faults there are rare" unsupported | R15's default is now the stricter one: M4 at the table also needs `fault_exits_clear`; the less strict option (M4 with replay exits + no frame inside the table) is named for the user; the unsupported sentence is removed. | 4.2 rule 16 (c)-(d), 10 (R15), 11 |
| R3-10 (c) foreign hand-command writer | `active_writers` also counts `rt/brainco/{left,right}/cmd`; 84 `probe` and 86's preflight warn (`foreign_hand_cmd`) with "ask the owner to stop their hand tool, or `--no-hands`"; runbook box in E2. | 3.1, 4.2 rule 2, A steps 1-2, B2 step 2a, E2 |
| R3-11 mutation runs could create real DDS writers | Section 0 rule; A's `test_publisher_guard_blocks_channel_publisher` and E1's `test_loopback_pub_refuses_robot_network` fake `ChannelFactoryInitialize` and the writer classes. | 0, 3.6, A test, E1 test |
| Environment: battery | `pmset -g batt` at 17:47 still "Battery Power, discharging" (A and E1 already run on battery). Situation now says so and requires "AC Power" before B1/B2/C launch. | Situation, 6.1 |
| Timeline (Verdict 3) | Re-timed from the real starts: push 1 ~00:00 (23:45-00:30), B2's last re-review ~02:40, C3 cutoff 03:00, integration to ~05:15, final gate to ~07:30, push 2 ~08:00 with about 0 slack; the later-than-target clock is said plainly; "latest 09:30" now needs Finnegan's confirmation; the finish-stage launch after both build runs is named as a main-session step. | Situation, 6.1, 6.4, 6.6 |

**Seams for the main session (not plan files):** (1) plug in the charger and see `AC Power` before launching B1/B2/C; (2) launch B1 and
B2 on rev 6.3 now, and C only after the round-4 review of the R3-2/R3-7/R3-8 items (87 contract, rule 16 (b)-(d), E2's lookup) is GO;
(3) A's and E1's reviewers judge against rev 6.3, so the rev-6.3 items in their task text arrive as findings in their fix round - say so
in the review prompt, so they are not read as a build failure; (4) launch the finish stage as soon as both build runs return (~02:40), or
chain it automatically; an unattended session that is not re-invoked means push 2 carries no motion; (5) ask Finnegan to confirm he is
reachable until 09:30 on Monday, else the latest push is when he goes offline; (6) before push 1, mark 70, 72 `arm` and 72
`hand --bewegen` as "never on the robot day" in `docs/START_PROMPT.md` (line 100 still names 70 as the next real-robot step) and
`PROGRESS.md` (R4-6, R3-6).

## Review response (rev 6 round 4) -> revision 7
Every finding of "Plan review rev 6 - round 4" (`.planning/reviews/real-robot-plan.md`, lines 994-1141) is fixed in the plan text or
handed to the main session where it is not plan content; none is refuted. Also folded in: the planner items of the Track A and B1 code
reviews (`real-robot-code-A.md` "For the planner" + round-2 majors/minors, `real-robot-code-B1.md` P1/P2).
| Finding | How it is fixed | Where |
|---|---|---|
| **R4-1** (major) 87's corner orderings sampled too coarsely | Every path in blocks 1 and 2 is densified: `densify(path, step_of, MAX_SAMPLE_STEP_M = 0.02)` inserts joint-space midpoints until no arm/hand collision geom moves more than 2 cm between samples (`step_of` = centre shift + `rbound x rotation angle`, so a swinging capsule end counts); `MAX_HALVINGS = 10` fails loudly; the report carries `MAX_SAMPLE_STEP_M`, `max_sample_step_m`, `samples`; C test `test_fault_exit_sampling_cannot_skip_a_close_pass` (planted 0.5 m arm, 2.4 rad leg, edge 1 cm off the arc: even samples read ~6.8 cm, densified <= 1.5 cm) [mutation: even samples only]; C's verify and 6.3 assert `max_sample_step_m <= MAX_SAMPLE_STEP_M`; runtime estimate re-derived (10-25 min, fallback halves frames, never the density). Planner addition beyond the reviewer's "geom centre": the `rbound x angle` term, because a forearm capsule's tip moves more than its centre. | 1, 3.5, C test/verify/done/effort, 6.3, 6.5, 12 lens 9 |
| **R4-2** (major) split files missing; push check sees the working tree | Frontmatter adds A's `real_bringup.py`, `real_fakes.py`, `test_unit_real_factory_pose.py`, `test_unit_real_bringup.py` and B2's `real_preflight.py`, `real_run_log.py`, `real_run_modes.py`, `real_run_policy.py`, `fake_real_run.py`; push-set rule (frontmatter + reported splits); A and B2 `<files>` updated; 6.2 runs the whole push-1 check from `git clone` of the commit; 4.3/6.4 prove push 2 (`86 gate` UNLOCKED in both envs, 89) from a clean clone; the unlock recipe's `git status` covers `tests/helpers`; rule 4.1 (b), A's `MOTION_MODULES` (A2) and 89's `FORBIDDEN_MODULES` (E2) add B2's splits. Re-derived evidence: the 18:14/18:15 local commits already contain A's splits - planner probe 18:21 on a scratch clone of `c0898cf`: `84 --dry-run` exit 0, 55 A+E1 unit tests pass, 1 skip (logs `clone_84_dryrun.log`, `clone_unit.log` in the planner's scratchpad); so the defect was in the plan text and the check, not yet in the commit. | frontmatter, 3.1, 4.1, 4.3, 5 intro, A, A2, B2, E2, 6.2, 6.4, 6.5 |
| R4-3 stale clock, false statements | 6.1 re-timed from STATUS (builds 5-25 min, reviews 5-6, fixes 5-8): push 1 ~18:45-19:30, C3 ~20:30, C5 ~00:45, cutoffs kept as fail-safe limits; Situation and section 5 state that B1/B2 started on rev 6.2 and get their rev-6.3/7 items as review findings; Finnegan's message "Part A tonight"; unlock after 00:00 because of the date check (new consequence of the earlier clock). | Situation, 5 intro, B1/B2 notes, 6.1, 6.4, 6.6, 7 |
| R4-4 safety state per laptop, hazard per robot | 3.1 "per-laptop safety state" paragraph; runbook page 1 one-laptop rule (section 7, E2); A2 changes 84's `PART B ARMS` line for a missing reading to "no Part B reading found on this laptop - M0 cannot compare ..." with a test [mutation]. | 3.1, A2, E2, 7 |
| R4-5 ball between the hands | Named as a residual risk (assumption in 1, rule 16 (f), R15, section 7 exits paragraph, runbook via E2); no ball geometry tonight. | 1, 4.2 rule 16 (f), 7, 10 R15, E2 |
| R4-6 lookup rounds up, assumes monotonicity | Rule 16 (b2): read floor and ceil rows, "yes" only if both, larger setback; E2 action + `test_runbook_lookup_reads_both_neighbouring_rows` [mutation]; section 7 M0. | 4.2 rule 16 (b2), E2, 7 |
| R4-7 pgrep misses DDS pytest runs | Section 0's one pattern adds `pytest.*(test_e2e_real_adapter_sim\|test_integration_arm_sdk_sim)`; 6.2 and Track C use the same text (6.2's shorter copy is gone). Checked 18:19: macOS `pgrep` accepts the regex and does not match itself. | 0, C action, 6.2, 6.3 |
| Environment: battery | Not plan content. `pmset -g batt` 18:19: "Battery Power, 99 %, discharging". Situation requires AC before C, integration and final gate. | Situation, 6.1 |
| A review r2 major 1 (= R4-2 for A) | as R4-2 | frontmatter, A |
| A review "for the planner" 2 | A's test list names the fix-round tests as built | A step 3 |
| A review r2 minors 2, 3, 4, 5 (R2), 7 | Track A2 (read-only, after push 1) | A2 |
| A review r1 item 4 (`out` typed `str`) | B2 note (arrives as review finding) | B2 |
| A review minor 6 / minor 8 | Rulings R17 / R16 with recommendations; runbook lines via E2 | 10, E2 |
| B1 review P1 (real arm lead before table steps) | Rule 16 (c) `M3_LEAD_MAX` 0.15 rad; B2 step 1b `MAX LEAD` line + test; section 7 M3; E2 runbook line; stand-in if missing | 4.2, B2, E2, 7 |
| B1 review P2 (release_plan 13/14) | One contract line under `release_plan` | 3.3 |

**Seams for the main session (not plan files):** (3, extended by R4-3) rev-6.3 and rev-7 items reach B1, B2 and E2 as review findings -
their reviewers must be pointed at rev 7 (B2: steps 1b, 2a, 2b and the `out` type; E2: the R4-4/R4-5/R4-6/B1-P1/R16/R17 lines and 89's
forbidden list); (6) from rev 6.3 still stands (START_PROMPT/PROGRESS "never" items before push 1); (7) launch Track A2 only after
push 1 is made; (8) run 6.2 from a clean clone before push 1, and 4.3's clone lines before push 2; (9) C may start now on rev 6.3
with the R4-1 build note (round-4 fast path), its code reviewer checks 87 against rev 7 section 3.5; (10) charger before C.

## Review response (round 5) -> revision 7.1
Every finding of "Plan review rev 6 - round 5" (`.planning/reviews/real-robot-plan.md`, lines 1143-1310) is fixed in the plan text as
the reviewer proposes or handed to the main session where it is not plan content; one sub-item is refuted with evidence (walk
proposal 3, row R5-4). Also folded in: Finnegan's three facts (motorized head, SSH login on site, table on its tape marks), the walk
re-review's RV-1..RV-8 (as Track RB and section 7), and E2 review r1 minors 1-2 (contract items in 3.5).
| Finding | How it is fixed | Where |
|---|---|---|
| **R5-1** (major) 87 picks start poses by frame count (up to 11.8 cm hand travel between checked poses) | Block 2's start poses come from `select_start_poses` (pure): keep a pose whenever the step from the last kept pose would exceed `MAX_SAMPLE_STEP_M`, joint-space interpolated poses where raw frames are farther apart, per waist limit on the clipped poses; `max_start_step_m` + `start_poses` in the summary, asserted in C's verify and 6.3; `frames_inside` counts the dense poses; `--frame-step` and the every-20th-frame fallback removed, runtime fallback = fewer setbacks or one waist limit per run + `87 --merge` (pure `merge_reports`, refuses mixed settings); R15 sentence corrected (paths **and** start poses at 2 cm; still demo poses only); C test `test_start_poses_follow_arm_movement_not_frame_count` (planted 21-frame demo, edge 1 cm off the arc between frames 14 and 15: movement selection <= 1.5 cm fails, frames 0/10/20 read ~27 cm - re-computed here: 26.96 cm) [mutation: fixed frame step 10] + one merge test. Put into Track C's task text as a build note (C had not started at 18:45). | 3.5, C (action, tests, verify, done, effort), 6.3, 6.5, 10 R15, 12 lens 9-10 |
| **R5-2** (major) push 1 stale; unreviewed fixes; three runbook writers | (1) Push 1 = `c0898cf` + `f897b0c` + `e914654` (R18) + one fix-set commit: Track AF (real_io readers kept until `close_dds` at exit, 84 faulthandler + line-buffered output, `rate_hz` over listening time, two tests) and Track RB (F1-F21 incl. F21's hip formula, RV-1..RV-8, the publisher's deadline line), plus E2's files after its fix round (Part C inert while LOCKED; fallback = E1 stub, E2 to push 2). (2) Own gates: "Track A - follow-up review" in `real-robot-code-A.md`; "Review of dry-walk A+B fixes" + a diff check of the RV lines in `real-robot-runbook-walk-AB.md`; "Track E2 - round 2". (3) One runbook writer at a time: RB, then E2's fix round, then nobody until the commit; A2 starts after the fix-set commit. (4) 6.2 runs 84 five times from the clean clone (every run exit 0, `OWN WRITERS: 0` in every log, no "slow hand state"), plus a grep that AF is in the clone. (5) B2's re-review and C's e2e run against the AF `real_io` (`close_dds` after the `finally` ramp-out; hash in their logs; integration records `git hash-object` = `HEAD:`). (6) Section 7 Part B 3: table top = 0.74 + (hip height - 0.648 m), with RV-1's measurement state and bounds. Timeline: push 1 ~19:30-20:15, C3 ~21:30, C5 ~01:45 (cutoffs unchanged). | frontmatter, Situation, 0, 3.1, 5 (intro, AF, RB, A2, B2, C, E2), 6.1, 6.2, 6.3, 6.4, 7, 12 lens 10 |
| R5-3 (minor) `lowstate_stale` exit read as "weight 0 delivered" | A2 item 6: the newest sidecar with `exit_reason` `lowstate_stale` counts as "may be held" + test [mutation: `last_weight_sent` only]; 3.1 contract; rule 16 (e) box "no final line **or** `FAULT EXIT lowstate_stale`", "after any **other** exit 4"; E2 fix round and section 7 carry the wording. | 3.1, 4.2 rule 16 (e), A2, E2, 7 |
| R5-4 (minor) `72 kamera --suchen` not sanctioned; walk proposals and questions homeless | Section 0 and section 7 page 1: "the only allowed 72 command is `72_real_check.py kamera --suchen`" (read-only, evidence cited). A2 items 7-9 take walk proposals 1, 2, 4, 5 (+ RV-3's Q3 text); A2's files add `real_camera.py`, `setup_robot_laptop.sh`. **Refuted, proposal 3** (`--robot-pc` flag for `72 kamera`): `scripts/72_real_check.py` is existing German code (docstrings and messages in German, committed 2026-10-07 in `27708e9`) that section 0 keeps untouched; the runbook's fallback "try N = 0..7 with 84 `--robot-pc <user@IP>`" already covers another user@IP, so no code change is needed. Open questions to section 10 as R19 (a) table vs camera picture (recommended default: table stays on its marks) and (b) the shared folder. | 0, 7, A2, 9, 10 R19 |
| R5-5 (minor) push-2 proof misses test files | Frontmatter + B2 `<files>`/verify add `tests/test_unit_real_preflight.py`, `tests/test_unit_real_send.py`; the unlock recipe's status check covers `tests`; 6.4 also runs B1/B2's unit tests in the clean clone. | frontmatter, 4.3, B2, 6.4 |
| Environment: battery | Not plan content. 18:38 "Battery Power, 94 %"; Situation keeps "AC Power before C, integration, final gate". | Situation |
| Finnegan: motorized head | 2.4, assumption (1), rule 4.1 (3) (owner sets pan 0 / 50 deg down with its own control; operators only verify via phone + 84 overlay; nothing moves the head after Part B; `84 --steps camera` re-run before M4, before M5, after any mode change/reboot), RB item 4, E2 fix round, section 7, Q3, section 11 risk. | 0, 1, 2.4, 4.1, RB, E2, 7, 9, 11 |
| Finnegan: SSH login only on site | Section 0, assumption (1), RB item 5 (`ConnectTimeout=5`), section 7, Q5 ("never type the password here"). | 0, 1, RB, 7, 9 |
| Finnegan: table on its tape marks | Recommended default R19 (a), pending his confirmation; assumption (1), RB item 4, section 7 Part B 3. | 1, RB, 7, 10 |
| E2 review r1 minors 1, 2 (contract gaps) | 3.5 pins `summary["elbows"]` and "`table_steps` = setback only within `setback_max_keep_layout_m`, else skip"; E2's fix round aligns its oracle. | 3.5, E2 |

**Seams for the main session (not plan files):** (11) run the gates in this order and never two runbook writers at once: AF review
(running since 18:37) and RB's RV fixes (since 18:40) -> RB diff check -> E2 fix round -> "Track E2 - round 2" -> commit the push-1 fix
set -> 6.2 from the clean clone -> Finnegan pushes; (12) start C on rev 7.1 (R5-1 build note is in its task text) only after `AC Power`;
(13) tell B2's fixer/reviewer and C that `real_io` now has `close_dds` at `atexit` (rev 7.1 B2 note, C build note); (14) Finnegan's push-1
message: "Part A tonight", the shared folder name (R19 (b)), "Part C is for Monday and only if `86 gate` says UNLOCKED", and his answer to
R19 (a); (15) launch A2 only after the fix-set commit; (16) seam (6) from rev 6.3 (START_PROMPT/PROGRESS "never" items) still stands.

**Unresolved user rulings (recommendation = working default):** R19 (a) table stays on its tape marks, camera only 49-51 deg (recommend
yes; Finnegan's confirmation pending), R19 (b) shared folder (recommend: named in the push-1 message, else `runs/` stays on the robot
laptop), R18 (B1 rides in push 1; recommend accept), R15 (M4 at the table also needs `fault_exits_clear`; recommend keep), R16 (no
joint-limit tolerance tonight; recommend the runbook line), R17 (no sidecar `source` filter tonight; recommend the never-box line), and
the earlier R2-R14 as listed in section 10.
