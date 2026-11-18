# Track C code review, round 1 (independent reviewer, 2026-10-27)

Reviewer: independent; did not write Track C and had no author reasoning. Model: Opus.
Scope: the uncommitted files scripts/85_arm_sdk_sim.py, scripts/87_arm_path_clearance.py, g1sim/arm_clearance.py,
g1sim/arm_clearance_fk.py, g1sim/arm_clearance_report.py, tests/helpers/sim_host.py, tests/test_e2e_real_adapter_sim.py,
tests/test_integration_arm_path_clearance.py, tests/test_integration_arm_sdk_sim.py. Checked against .planning/real-robot/PLAN.md
(section 0 rules, 1 assumptions, 3.5, 4.2 rules 4/7/16, task C) and the review lenses (harness template
`_harness/templates/project/.planning/REVIEW-LENSES.md`; the project has no copy of its own).

## Verdicts

- **Implementation fidelity (did the code do what the plan said): YES, with minor deviations.** Every item in task C is built. The
  checks the plan asks for pass, except two e2e limits (below). The assertions those two failures hide pass when I re-run them by hand.
- **Plan correctness (was the plan right): NO.** Three plan values describe physics the sim does not have:
  (P1) 87 models waist pitch 0, but the robot holds it at q0 and it sags further. (P2) The lead limits sit at or below the gravity
  sag of the demo itself. (P3) The waist-hold and ramp-still tolerances test gravity, not code.
- **Overall: NO-GO.** One behaviour bug: 87 gives clearances that are too optimistic, by about the whole 5 cm margin. Two e2e
  limits need a plan decision before the e2e can pass. 85, sim_host and the 85 integration tests are fine on their own.

## Commands run (all logs kept in full)

| Command | Result | Log |
|---|---|---|
| `$PY -m pytest tests/test_integration_arm_sdk_sim.py tests/test_integration_arm_path_clearance.py -v -rA` (g1-ext env, DDS env) | 13 passed, 49 s | /tmp/l1day1/c_review_integration.log |
| `$PY -m pytest -m e2e tests/test_e2e_real_adapter_sim.py -v -rA` | 3 failed (G1, G2, K), 5 passed, 2 skipped (40k), 329 s | /tmp/l1day1/c_review_e2e.log; artifacts /tmp/l1day1/c_review_e2e_artifacts |
| same, `-k release_after_kill`, twice | 2 x passed | /tmp/l1day1/c_review_e2e_K_rerun1.log, ..._rerun2.log |
| project `.venv` pytest of the three C test files | 5 passed, 6 skipped (importorskip works) | /tmp/l1day1/c_review_projvenv.log |
| G1/G2 assertions hidden by the first failing assert, re-run on the artifacts | all pass | /tmp/l1day1/c_review_masked_asserts.txt |
| FK probe: 87 clearances against waist pitch | see B1 | /tmp/l1day1/c_review_waistpitch_fk.txt |
| demo lag (action - state) of replay_demo.npz | max 0.264 rad, left shoulder pitch, frame 151 | /tmp/l1day1/c_review_demo_lag.txt |
| analysis of G1 sag and G2 lead per phase and joint | see L1, L2 | /tmp/l1day1/c_review_e2e_analysis.txt |
| analysis of the K failure | see m1 | /tmp/l1day1/c_review_release_analysis.txt |

Before every DDS run, the section-0 pgrep check and the `82_vla_eval|81_smolvla_server|lerobot-train` check printed nothing.
Mutation logs from the build: runs/real_c_mutation.log (85: 3/3 caught) and runs/real_c_mutation_87.log (87: 8/8 caught). I read
them but did not re-run them.

## Behaviour bugs

### B1 (major, safety): 87 checks an upright torso that the robot does not hold, so its clearances are 2-6 cm too optimistic
- **Trigger:** every 87 run. `arm_clearance.demo_frames`, `hanging_pose` and `READY17` all build 17-vectors with
  `real_safety.sdk_targets(..., q0=zeros)`, so motor 14 (waist pitch) is 0. `ClearanceModel._compute` writes those values over
  the scene's own stance (`g1sim/arm_clearance_fk.py:115`). But 86 holds 13/14 at the **measured** q0 (rule 4,
  `real_safety.sdk_targets`).
- **Evidence (live):**
  - After `TableEnv.reset`, the scene's waist pitch is 0.0405 rad: the GR00T stance, which 87 overwrites with 0.
  - In e2e G1, the sim holds q0 pitch = 0.0406. Measured pitch rises to 0.072 in ready/hold.
  - In e2e G2 (replay), measured pitch reaches **0.126 rad** while the sent pitch is 0.041.
  - FK over all 726 demo frames at L = 0.15 (min table clearance by waist pitch):

    | Waist pitch (rad) | Min table clearance | Demo frames inside the table |
    |---|---|---|
    | 0 (what 87 models) | 5.18 cm | 0 |
    | 0.0405 | 3.26 cm | 0 |
    | 0.072 | 1.76 cm | 0 |
    | 0.126 | -0.82 cm | 26 |

    87 reports `frames_inside.table = 0`.
  - On the ready path h(e, s) -> ready at setback 0.15, clearance drops by about 1-1.6 cm going from pitch 0 to 0.072.
- **Impact:** every number of the M0 lookup is computed for a torso about 0.04-0.13 rad more upright than the robot that 86
  commands. That is the whole 5 cm margin, which the plan meant for pose error and lag. Rule 16 (a) has no bound on waist
  roll/pitch either. **Masked today:** every elbow reads "skip table steps" (runs/real_c_clearance.log), so nothing gets unlocked
  right now. But any re-run with another layout or basket position can produce "nominal"/"setback" rows that are wrong in the
  unsafe direction.
- **Root cause:** the plan, section 3.5. "Every pose is a 17-vector built with `sdk_targets(pose27, q0=zeros(17))`" contradicts
  "87 then only writes arm (and waist yaw) qpos". The code followed the first sentence.
- **Fix direction (plan + code):**
  - 87 evaluates waist roll/pitch at the held value plus the measured sag: at least the sim's 0.126 rad from G2, or a small
    waist-pitch grid with the worst value counting.
  - The bounds line of rule 16 (a) gets waist roll/pitch: the M0 record has them.
  - The e2e reports the sag that 87 then assumes.

## Judgement of the two e2e limits (re-derived, not taken from the author)

Both runs, 10-18 and today, fail the same way: G1 0.0316 (10-18: 0.0317) > 0.02, and G2 0.256 (10-18: 0.255) > 0.15. The
values reproduce, so this is not a flake.

### L1: `WAIST_HOLD_TOL = 0.02` (G1, "13/14 within 0.02 while w > 0"). The code is right. The limit tests gravity, not code.
- **Code property of rule 4 (13/14 held at q0 in every message while w > 0):** holds exactly. The peak-to-peak of sent 13/14 over
  all w > 0 ticks is [0, 0]. kp/kd stay <= SIM_GAINS, and the max gap is 37 ms.
- **The error is a pitch sag that starts during `ramp_in` already** (0.0275 rad with the arms not moving yet). It stays at about
  0.031 through to_ready, hold, to_start and ramp_out. Roll stays <= 0.001.
- **Mechanism:** GR00T commands the waist at kp 250 (lerobot `config_unitree_g1.py:29`, the same as SIM_GAINS). GR00T's own
  target pre-loads the torso against gravity. 86 replaces that target with the *measured* angle, so the pre-load is gone. The
  joint then sags until 250 x 0.031 = about 7.8 Nm balances the upper-body and arm load. Any PD hold at a measured angle does this.
- **Verdict:** 0.02 has no source and fails for physics, not code. But **do not simply raise it.** The sag is safety-relevant
  (B1).
  - Replace the check with the code property: sent 13/14 == q0 in every w > 0 message.
  - Report the measured sag as a number.
  - Bound it by the waist pitch that 87 assumes, so a larger sag fails the e2e instead of silently eating the margin.
- **Same cause, second check:** `RAMP_STILL_MAX = 0.03` (N1) passes at 0.0275, which is 92 % of the limit (both runs). It is one
  noisy run from a false FAIL.

### L2: `LEAD_MAX = 0.15` (G2, "max lead < 0.15", review n2). The code is right. The limit is below the demo's own lag.
- **Code properties of G2:** all hold.
  - Envelope OK; pelvis z min 0.746.
  - `loaded_files_at_exit == gate.loaded_files`.
  - Every received message is in the sent log, in order.
  - 600 policy actions match demo rows, increasing, span 598.
  - The exit goes `policy -> to_ready`, and the exit code is 0.
  - Median lead 0.0148 < 0.05.
- **Where the lead comes from:** the max lead is 0.256 on **both shoulder pitches**. Measured stays at -0.87 while the command
  goes to -1.13 (arms raised forward). Lead > 0.15 for 773 ticks, the longest run 269 ticks (5.4 s). That is a static gravity
  sag at kp 50, not timing.
- **The training demo has the same signature:** recorded in the same sim with the same gains, it has |action - state| max
  0.264 on left shoulder pitch at frame 151, and 0.253 on the right one.
- **Verdict:** 0.15 is not a property 86 can meet with SIM_GAINS. It is contradicted by the data the policy was trained on.
  Raising it alone is wrong, because of the plan finding P2 below.

## Plan findings (was the plan right)

- **P1 = B1** (waist pitch in 87).
- **P2 (major): the contact-fault limit has no headroom over the nominal gravity sag.**
  - `LEAD_LIMIT` is 0.25 rad for 13 ticks (rule 7). The nominal free-air replay in sim reaches 0.256 for 3 ticks, which leaves
    0.006 rad of margin.
  - With a heavier real hand or slower tracking, the in-air M3 replay will trip `contact_fault`, a fault ramp-out from a policy
    pose.
  - `M3_LEAD_MAX = 0.15` (rule 16 (c)) cannot be met even in sim, so M4/M6 at the table are unreachable on that rule alone.
    That is conservative, but it is not what the plan intended.
  - The lead monitor needs a definition that separates contact from static sag. For example: lead relative to the hold offset at
    the same pose, or a gravity feed-forward `tau`, or an explicitly measured limit with a recorded source.
  - Until then, G2 should assert the code property: no run above `LEAD_LIMIT` as long as `LEAD_TICKS`. It should report the
    max lead, not gate on 0.15.
- **P3 (minor): the basket dominates block 1.**
  - At the nominal layout, the right hand of h(0, -0.1) is already 0.36 mm inside the basket (worst_tick 0, the hanging pose
    itself).
  - At setback 0.15, elbows 1.0-1.6 fail only on the basket (`rh_right_*|basket/183`).
  - So "no factory elbow is cleared" depends heavily on where the sim puts the basket, and the setback moves the basket together
    with the table.
  - The runbook must pin the real basket to the sim position, or 87 should handle the basket separately. Conservative, so not a
    safety bug.
- **P4 (minor): no fault-exit row is computed on the real scene.** `setbacks2 = [0.0]` and no elbow is cleared, so
  `policy_exits` has 349 `to_ready` rows and 0 `to_hanging` rows. `fault_exits_clear` is False everywhere because the rows are
  empty, not because exits failed. The R3-7 machinery is exercised only by the planted pure test. That is correct by the plan,
  but worth knowing.

## Minors (style, test gaps, paperwork)

- **m1 (e2e flake, not a C code bug):** K failed once in 4 runs (10-18 pass, today fail, 2 reruns pass).
  - `86 release` ended with exit 4 `lowstate_stale` 2.7 s in (to_factory was cut short; the last sent pose was 0.035 from
    factory; w ended at 0).
  - At the same time, 85's measured loop stalled 0.24 s, its arm_sdk receive gap reached 0.158 s, and sim qpos froze twice for
    about 0.3 s. The Mac load average was about 6.
  - 86 reacted per rule 10.
  - Unattributed: 86's stdout also shows `[Reader] catch DDSException error ... while taking data` after the LOG line. That is
    for the real_io owners (Track A) to check.
  - Test gap: 85 does not log its own rt/lowstate publish times, so a stale fault cannot be blamed on the sim or on 86 from the
    report.
- **m2:** `ClearanceModel.stand_on_table` ends with `self.set_setback(self.setback)` after `self.setback` was already
  overwritten, so the "restore" does nothing (`arm_clearance_fk.py:82-90`). It is harmless today because every caller sets the
  setback first, but it is wrong as written.
- **m3:** `exits_from` writes `worst_pair: None` for `to_hanging` rows. The plan's row format has `worst_pair`.
- **m4:** `TRAINING_REACH_MAX_M = 0.52` is a copied number. The plan says it should be read from
  `table_scene.SCENES["volleyball"]["reach_m"]` (currently (0.33, 0.52), so it matches).
- **m5:** `print_summary` prints "replay exits NOT clear" and "frames inside the table None" for elbows that have no setback.
  The numbers were not computed; the wording suggests they failed.
- **m6:** fault exits are computed only at an elbow's own setback. Rule 16 (b2) reads the larger setback of two neighbouring
  rows, so the smaller row's `fault_exits_clear` belongs to a different table position than the one used. Moot while every
  elbow is "skip".
- **m7:** the build note R5-2's `git hash-object g1sim/real_io.py` line comes from the builder's shell command, not from the e2e
  file. Today's log does not have it.
- **m8:** `sim_host.start_85` checks its READY deadline only when a line arrives, so a silent hang blocks pytest forever (there is
  no pytest timeout).
- **m9:** K asserts weight 0 from 86's own sent log. F6 and F7 check what 85 received. The R5-2 "last weight-0 message arrives"
  check is missing for K.
- **m10 (lens 2):** `scripts/run_tests.sh` without `all` runs `-m "not e2e"`, so it now starts four 85 DDS sims. The section-0
  pgrep pattern relies on the test file name appearing in the pytest command line, which run_tests.sh does not have. Only the 85
  child processes match while they run.
- **m11:** one line in tests/helpers/sim_host.py is over 130 characters. All files are under 300 lines; the docstrings say why;
  the tests carry Proves/Catches.

## What would turn this into GO

1. **B1/P1:**
   - 87 evaluates waist roll/pitch at the held q0 plus a sag allowance with a source (sim G2: 0.126 rad), or the worst of a
     small pitch grid.
   - Rule 16 (a) gets a waist roll/pitch bound.
   - Re-run 87 and refresh the runbook lookup.
2. **L1/L2/P2 (plan decision, Finnegan or planner):**
   - G1 asserts sent 13/14 == q0 and reports the sag, bounded by 87's allowance.
   - G2 asserts no lead run of `LEAD_TICKS` above `LEAD_LIMIT`, and reports max lead and the joint.
   - Decide how `LEAD_LIMIT` and `M3_LEAD_MAX` separate contact from gravity sag.
3. One full e2e pass after that. K may need a rerun on a loaded Mac (m1).

---

# Track C code review, Round 2 (independent re-check of fix B1, 2026-10-27)

Reviewer: independent; did not write Track C or the B1 fix and had no author reasoning. Model: Opus. Scope: only B1. The
fix changes g1sim/arm_clearance.py (WAIST_PITCH_CHECKED 0.126, WAIST_PITCH_SAG 0.085, WAIST_ROLL_TOL 0.02, two GRID_BOUNDS keys),
g1sim/arm_clearance_fk.py (`_compute` writes roll 0, pitch WAIST_PITCH_CHECKED after the 17-vector), one new test, PLAN.md 3.5
and rule 16 (a), and regenerates runs/arm_path_clearance.json. Only 87, its report module and its test import arm_clearance
(grep), so 85, 86 and the e2e cannot be affected. I did not re-run the e2e.

## Verdicts

- **Implementation fidelity: YES.** Each FK pose now has the torso at roll 0 and pitch 0.126. The step bound and the clearance
  both come from `_compute`, so both see the lean. The cache key needs no pitch because the pitch is a constant.
- **Plan correctness: YES for the table, which is what B1 was about.** Two residual caveats follow (r1, r2). Neither one flips a
  verdict today.
- **B1: FIXED. Verdict for this round: GO.** Track C as a whole is still blocked by L1/L2/P2 from round 1 (the e2e limits G1/G2).
  Those were not in this round, and the code they test is unchanged.

## Commands run (logs in full)

| Command | Result | Log |
|---|---|---|
| g1-ext pytest tests/test_integration_arm_path_clearance.py | 10 passed | /tmp/l1day1/c_review_r2_integration.log |
| project .venv pytest of the three C test files | 5 passed, 7 skipped (importorskip) | /tmp/l1day1/c_review_r2_projvenv.log |
| mutation: pre-fix upright `_compute` patched in memory, same test file | new test FAILS, 9 others pass | /tmp/l1day1/c_review_r2_mutation.log |
| 87 re-run (`--demo tests/data/replay_demo.npz`, own report) | exit 0, printed table identical to runs/real_c_clearance_b1fix.log, summary equal to runs/arm_path_clearance.json; every elbow "skip table steps" | /tmp/l1day1/c_review_r2_87.log, .json |
| report check | frames_inside.table = 1 per waist limit (was 0); GRID_BOUNDS carries waist_pitch_q0_max 0.041, waist_roll_q0_tol 0.02 | /tmp/l1day1/c_review_r2_json.txt |
| FK probe, block 2 soft exits (start -> ready) over pitch -0.05/0/0.041/0.085/0.126 | see E1 | /tmp/l1day1/c_review_r2_pitch_block2.txt |
| FK probe, block 1 ready paths h(e,s) -> ready, elbows 0..1.6, s -0.1/0.1/0.3, setback 0 and 0.15, same pitches | see r1 | /tmp/l1day1/c_review_r2_pitch_block1.txt |

Before the runs, the pgrep check for 85/86/82/81/lerobot-train/test_e2e printed nothing.

## Evidence that B1 is fixed

- **E1, block 2 exits at waist limit 0.15** (77 start poses), passed / worst margin:

  | Waist pitch (rad) | Passed | Worst margin |
  |---|---|---|
  | -0.05 | 76 | -0.03 cm |
  | 0 (old behaviour) | 71 | -0.35 cm |
  | 0.041 | 62 | -0.69 cm |
  | 0.085 | 53 | -1.19 cm |
  | 0.126 (now) | 41 | -1.67 cm |

  - At waist limit 0.6 the counts go 109, 100, 92, 79, 67 of 117.
  - **No exit passes at 0.126 and fails at a smaller pitch**, so for block 2 the largest pitch is the worst case. The 30
    soft exits that read "pass" with the upright torso now correctly read "fail".
- **E2:** 87's frames_inside.table goes from 0 to 1 on the sampled start poses. That agrees with round 1's 26 of 726 raw
  frames inside the table at 0.126.
- **E3, test quality:** `test_clearance_is_computed_with_the_torso_sagged_forward` fails against the pre-fix code (mutation
  above). It has a Proves/Catches docstring. The files are under 200 lines each, no line is over 130 characters, and the
  constants carry their source.

## Residual findings (minor, none flips a verdict today)

- **r1 (plan text over-generalises):** "clearance falls as the pitch grows, so the largest pitch counts" holds for the
  **table** and for the block-2 verdicts. It does not hold for every object on block-1 ready paths.
  - In the block-1 probe, a pitch other than 0.126 gives a lower path minimum in 34 of 153 (path, object) cases at setback 0,
    and in 17 at 0.15. These are almost all basket cases, plus the stand at 0.1 mm and one table case at 0.9 mm.
  - The largest gaps are on the basket at setback 0:

    | Path | Worst pitch | Clearance there | Clearance at 0.126 |
    |---|---|---|---|
    | h(0.2, 0.3) | 0 | -5.33 cm | -4.51 cm |
    | h(0.3, 0.3) | -0.05 | -5.02 cm | -4.35 cm |
    | h(0.1, 0.3) | 0.085 | -5.22 cm | -4.75 cm |

  - So PLAN.md rule 16 (a)'s "a smaller or negative q0 pitch only adds clearance" is shown only for the table minimum over the
    demo frames, not for block 1.
  - No ready path passes at any pitch, so no verdict changes, and the error (<= 0.8 cm) is far smaller than B1's 5 cm.
  - **Fix when a table row is ever unlocked:** take the worst of a small pitch grid, [smallest allowed q0 .. 0.126], as
    round 1 suggested, or bound q0 pitch from below too.
- **r2 (open, already named by the fixer):** rule 16 (a) now reads the M0 q0 waist roll/pitch, but nothing shows them to the
  operator yet.
  - `real_io.factory_arm_summary` (g1sim/real_factory_pose.py:132) prints elbow, shoulder, wrist and waist yaw, but no waist
    roll/pitch.
  - The runbook bounds line (docs/ROBOT_DAY_RUNBOOK.md:407) has no waist roll/pitch either.
  - Not reachable today: the runbook says M4-M6 at the table are NOT cleared (line 420), and every elbow is "skip". It must be
    done before any lookup row is filled.
- **r3 (open, part of round 1's L1 decision):** WAIST_PITCH_SAG 0.085 is a sim number at SIM_GAINS. Nothing on the robot checks
  that the real sag stays within it.
  - Proposal: M3's in-air replay records the max measured waist pitch, and rule 16 skips the table if it exceeds
    WAIST_PITCH_CHECKED.
  - The plan text says "the real sag is not measured yet", which is honest. Same masking as r2.

## What stays open for Track C (not this round)

- L1, L2 and P2 from round 1: the G1/G2 e2e limits and LEAD_LIMIT/M3_LEAD_MAX need a plan decision. Then one full e2e pass.
- The minors m1-m11 from round 1.
- r2 and r3 above, both before any table row of the M0 lookup is unlocked.
