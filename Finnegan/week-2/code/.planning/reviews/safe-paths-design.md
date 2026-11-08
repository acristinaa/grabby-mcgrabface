# Review: safe arm paths (SAFE-PATHS.md, g1sim/arm_paths*.py, 87 --waypoint-paths), 2026-11-04

Reviewer: independent (did not write the work, no author reasoning seen). Scope: `.planning/real-robot/SAFE-PATHS.md`,
`g1sim/arm_paths.py`, `g1sim/arm_paths_clearance.py`, the diffs to `g1sim/arm_clearance_fk.py` and
`scripts/87_arm_path_clearance.py`, `tests/test_integration_arm_paths.py`, read against `real_safety.py`, `real_run.py`,
`real_run_exits.py`, PLAN.md rules 12, 13 and 16, and review C (r1, P4, m6, m8).
All my output: `/tmp/safepaths/review/` (probe scripts `probe_review.py`, `probe_p3b.py`, `run_fine.py`, `mutants.py`).

## Verdicts

1. **Implementation fidelity: YES.** 87 `--waypoint-paths` checks what SAFE-PATHS.md says, and the numbers are right:
   - My re-run of all four commands gives **row-for-row identical JSON** to the builder's `runs/safe_paths_waist{0.6,0.35,0.15,0.0}.json`
     (way in 11016 rows, lift home 3159 / 2781 / 2079 / 1593 rows).
   - waist 0.6 lift home 3159/3159. Factory-clear rows 7047/7049; the 2 failures are elbow 0.0, pitch -0.1, setback 0.20 (4.4 / 4.5 cm,
     basket). The per-(elbow, pitch) table in SAFE-PATHS matches my recomputation from the JSON.
   - The 4 tests pass (`tests_arm_paths.log`). My own in-process mutants are caught: cache key without lean/height, SIDE_UP = ready
     arm, and waist turned before the lift (`mutant_*.log`).
   - All waypoints (swing back, side up, ready) lie inside 86's clipped joint limits for every grid factory pose, so the
     SafetyFilter does not change the line 87 checked.
2. **Plan correctness: NO.** The paths are a real improvement for the policy-phase way out and for the way in from a clear
   factory pose. The design still has four errors that would make the robot hit something or make the check lie (B1-B4).
3. **Overall: NO-GO** for building S4 as written and for using the S1 lookup. Nothing changes on the robot today: 86 is
   untouched and the runbook's "skip table steps" stands.

## Behaviour bugs

**B1. Recover from a pose on the way in (or on the second half of the way out) drives the hand into the table.**
- Where: SAFE-PATHS "Per path and case" rows "fault exit (proposed hold + recover) = way out" and "recover / 86 release after a crash =
  way out, from the measured pose"; fault table F1 (after LowState returns), F2, F3. `arm_paths.way_out` (lift -> waist home -> ready ->
  ...) is prescribed from *any* measured pose, with no phase condition.
- Trigger (reachable): a contact fault, a NaN chunk, LowState coming back, or a crash + `86 release` while the arms are on way-in leg 1
  or early leg 2. These legs are the 10-15 s of every table run where the hands are low beside the table edge, so this is the most likely
  place for a contact fault. The same holds on the mirrored legs of the way out.
- Evidence (`probe_P4.txt`, worst of 27 cases, first half of `way_out` from points on the way-in legs):

  | Factory pose | Leg 1 at 10 % | Leg 1 at 90 % | Leg 2 at 10 % | From leg 2 at 30 % on |
  |---|---|---|---|---|
  | h(1.6, 0.1) | -9.3 cm table | -4.8 cm basket | -4.8 cm basket | >= 2.7 cm |
  | h(1.5, -0.1) | -11.5 cm table | -4.9 cm basket | -4.4 cm basket | >= 4.1 cm |
  | h(0.8, 0.3) | -6.9 cm table | +0.6 cm basket | +5.9 cm | ok |

  Every start pose in that probe is >= 9 cm clear.
- Why: the lift (shoulder 0.1 forward, elbow bend) followed by the straight line to the ready pose is exactly the forward sweep the
  waypoints were built to avoid. SAFE-PATHS names "a crash in the middle of the way in" only under "Still not clear", for release.
  The fault behaviours and the per-path table present recover as clear.
- Fix direction: recover must depend on where the arm is. On a way-in or way-out leg, go back along the legs already travelled
  (towards the factory pose, or on to side-up). Use lift-home only from policy / hold / shadow. `86 release` must find the leg from the
  measured pose, or 86 must log it; otherwise it refuses. 87 needs rows for recover from mid-leg poses.

**B2. The lean grid starts at the wrong end, and the factory pose is worst at the smallest lean.**
- Where: `arm_paths_clearance.LEAN_PITCHES = (0.041, 0.0835, 0.126)`. 0.041 is the *largest* q0 that rule 16 (a) allows. Review C r1
  asked for "[smallest allowed q0 .. 0.126], or bound q0 pitch from below too". Rule 16 (a) has no lower bound ("a smaller or negative q0
  pitch only adds clearance").
- Physics: at the factory pose the arms hang, so there is no forward sag and the torso sits at about q0 itself. For the
  factory-pose rows, clearance *rises* with pitch (`probe_P3b.txt`, factory pose only, roll 0, nominal table):

  | Pose | q0 -0.05 | 0.0 | 0.02 | 0.041 | 0.126 |
  |---|---|---|---|---|---|
  | h(1.1, 0.1) | **4.00** | **4.88** | 5.25 | 5.65 | 7.33 |
  | h(0.8, 0.3) | 5.52 | 6.27 | 6.59 | 6.92 | 8.38 |
  | h(1.5, -0.1) | 5.37 | 6.42 | 6.85 | 7.31 | 9.25 |

- Impact: with a real q0 pitch below about 0.03, S1's "elbow 1.1 at shoulder pitch 0.1: clear at the nominal table (5.6 cm)" is false.
  The check lies.
- Lift home down to pitch -0.05 still passes (`probe_P1.txt`, worst +0.12 cm).
- Fix: give rule 16 (a) a lower q0 pitch bound and extend `LEAN_PITCHES` down to it, or check the measured q0 itself (see B3 fix).

**B3. The S1 lookup turns boundary grid rows into "clear" for real poses that are not.**
- (a) **Rule 16 (a) tolerance.** A grid row stands in for factory poses with shoulder roll/yaw, wrists and waist yaw up to 0.15 rad
  from 0. On the S1 boundary rows that tolerance costs up to 1.8 cm (`probe_P3.txt`, `probe_P3b.txt`, worst of 27 cases, way in):
  - h(1.1, 0.1) 5.64 cm -> **4.21** (wrist pitch -0.15), **3.87** (waist yaw +0.15);
  - h(0.8, 0.3) 6.92 -> **4.95** (waist yaw +0.15);
  - h(1.5, -0.1) 7.31 -> 5.40 (still passes);
  - h(1.6, 0.1) is unchanged at 8.92.
- (b) **Shoulder pitch between grid rows.** The pitch grid steps 0.2 rad. Elbow 1.1 is 5.64 cm at pitch 0.1, **1.55 cm at 0.0** and
  -2.3 cm at -0.1. S1 "keys on (elbow, shoulder pitch)" without saying which pitch row is read. Reading the nearest row lies.
  In the probe, clearance rose with pitch at every elbow, so reading the floor (more forward) row would be conservative. Rule 16 (b2)
  covers neighbouring rows for the elbow only.
- Fix: on site, run 87 FK on the actual M0 pose (seconds), same paths, same cases, lower lean included. Or require the row margin to
  cover the tolerance cost, and state the floor-row rule for pitch.

**B4. The fault table covers 4 of the 8 fault reasons 86 can raise.** The other 4 still ramp out at once, through the table.
- `real_run.py` / `real_run_exits.py` also raise these reasons:
  - `internal_error` (any exception, `_drive` and `soft_exit`);
  - `exit_path_overrun` (`real_run.py:206`, the exit clock = planned + 1 s);
  - `weight_step` (`:204`);
  - `signal_in_release` (`:233`).
- Each goes to `fault_exit` -> ramp out at once -> the factory controller's own path. That path is 15.7-16.2 cm into the table from a
  policy pose (SAFE-PATHS' own number).
- `exit_path_overrun` gets more likely with the new paths. They run up to 23.8 s, and F2's half-speed recover doubles a leg, so the
  exit clock must be planned with the real speed.
- Fix: rule on these four too (F5). At least: hold at weight 1 on the last command, then the operator decides.

## Minors

- **m1 (sampling near contact).** The 2 cm rule's "~1 mm error at 5 cm" does not hold where the pass line is the start clearance
  (< 1 cm).
  - With 0.5 cm sampling (`wp_fine_waist0.6.log`), lift home at waist 0.6 is **3157/3159**, not 3159/3159. The 2 failing rows miss by
    -0.08 / -0.03 cm on the stand, both at frame 127, table +1 cm.
  - Both of those start poses have the left hand 1.6-1.8 cm *inside* the table, which is not physical. So no real failure, but the
    count is a sampling artefact.
  - Coarse minus fine margin: up to 1.11 cm; 493 rows > 2 mm.
  - S3's thin margin is about **+0.01 cm**, not +0.06 cm (right elbow, basket).
  - The way-in rows are not affected: coarse minus fine <= 0.49 cm, no verdict flips.
- **m2 (2-7 cm setback rows).** S1 offers these rows as usable. Lift home was computed at setback 0 only. SAFE-PATHS says so under
  "Still not clear", but S1 should not list these rows until they are checked.
- **m3 (contact-fault start pose).** F2 recovers from the *measured* pose, which can be up to `LEAD_LIMIT` 0.25 rad from the
  commanded demo pose. 87 starts only from demo poses. The lift may also push into whatever blocks the arm. The sim table is a solid
  block, so "hand under the real table top" cannot be checked.
- **m4 (single source).**
  - 87's summary groups by elbow over all pitches, so it prints "elbow 1.1 NOT clear" while SAFE-PATHS/S1 read 1.1 as clear at
    pitch 0.1.
  - The per-(elbow, pitch) table comes from `runs/safe_paths_tables.txt`, which no script in the repo produces. I recomputed it from
    the JSON and it matches.
  - The lookup Monday reads should come from 87's summary.
  - `arm_paths` has no function for "soft exit back along the way-in legs from mid-leg", so S4 would have to invent it in 86. That is
    a drift risk; add it to `arm_paths` and let 87 check it.
  - When S4 is built, add a test that 86's planned ticks == `arm_paths.path_ticks(...)`.
- **m5 (waist hold).** `SafetyFilter._limit_waist` replaces waist yaw with q0's yaw under `--waist-hold`. `side_up` (yaw 0) and the
  waist-home leg (ready yaw) would then be sent with q0's yaw, up to 0.15 rad. That is a different line from the one 87 checked.
  S4 must pass q0's yaw, or 87 must check that line.
- **m6 (wording).** "Every case where a waypoint path is not clear is a case where the factory pose itself is already closer than
  5 cm" is contradicted by the 2 setback-0.20 rows. The doc does list them further down.
- **m7 (layout tolerance).** The table-height cases come from the randomization range, not from a real-table tolerance. A table up to
  1 cm closer than the tape marks (runbook "24.5 cm within 1 cm") is a negative setback, which is not in the grid. With a margin of
  about 0.01 cm (m1), lift home would not survive that unchecked.
- **m8 (rule text).** The F1-F4 changes to rule 12 are clearly marked as rulings. The soft-exit change (rule 12: ready -> straight
  to q0) and the release change (rule 13) are only covered by S4's "put these paths into 86". S4 should name both rule texts it
  rewrites.
- Review C status: r1 is handled only in `--waypoint-paths`, and from the wrong end (B2). P4: the old straight rows are quoted; the
  new design has no mid-leg fault rows (B1). m6 and m8 are untouched, as the doc says.

## Rulings Finnegan needs (in addition to F1-F4, S1-S4)

- F5: what happens for `internal_error`, `exit_path_overrun`, `weight_step`, `signal_in_release` at the table (B4).
- A lower bound for the M0 q0 waist pitch in rule 16 (a), or an on-site FK check of the measured pose (B2, B3).
- Recover must depend on the phase and leg; 86 logs the leg (B1).

## Commands and results

| Command | Result | File |
|---|---|---|
| 87 `--waypoint-paths`, waist 0.6, 8 setbacks | 119.7 s, identical to builder | /tmp/safepaths/review/wp_waist0.6.{json,log} |
| same, waist 0.35 / 0.15 / 0.0, setback 0 | identical to builder | /tmp/safepaths/review/wp_waist{0.35,0.15,0.0}.* |
| 87 `--waypoint-paths` at 0.5 cm sampling, waist 0.6 | lift home 3157/3159 (m1) | /tmp/safepaths/review/wp_fine_waist0.6.* |
| pytest tests/test_integration_arm_paths.py | 4 passed | /tmp/safepaths/review/tests_arm_paths.log |
| 3 in-process mutants | all caught | /tmp/safepaths/review/mutant_*.log |
| probes P1 (lean < 0.041), P2 (fine sampling), P3/P3b (off-grid poses), P4 (recover mid-leg) | B1-B3 | /tmp/safepaths/review/probe_P*.txt |

Not run: the 85 sim gate and anything on 86 (S4 is not built); own-body clearance (only the builder's probe exists).

---

## Round 2 (2026-11-04): re-check of B1-B4 after the fix round

Reviewer: independent (did not write the fix round, no author reasoning seen). Scope: `g1sim/arm_paths.py`,
`g1sim/arm_paths_clearance.py`, `scripts/87_arm_path_clearance.py` (`--factory-pose`), `tests/test_integration_arm_paths.py`,
SAFE-PATHS.md, read against `real_run.py`, `real_run_modes.py`, `real_safety.py`, `real_preflight.py`, `real_factory_pose.py`.
86 and PLAN.md are unchanged (`git status`). All my output: `/tmp/safepaths/review2/`.

### Verdicts

1. **Implementation fidelity: YES.** The code does what the fix round and SAFE-PATHS say.
   - Every SAFE-PATHS count re-derived from `runs/safe_paths_waist0.6.json` (`verify_json_waist0.6.txt`): 45 cases
     (pitch -0.05 / 0.0 / 0.041 / 0.0835 / 0.126), way in 18360 rows, recover 275400 rows, factory-clear rows 11499 / 11522
     (23 failures: elbow 0.0 x20, 0.1 x3 at pitch -0.1, -2.45 to +4.90 cm), recover where the way in passed 172470 / 172485
     (the 15 are the one h(0.6, 0.3) rounding row), lift home 5265 / 5265 (worst +0.06 cm). Clear-at-nominal lists per pitch
     match the table (min 5.36 / 6.32 / 5.51 cm). h(1.1, 0.1) worst 3.99 cm at lean -0.05. Waist 0.35 / 0.15 / 0.0:
     4611/4635, 3399/3465, 2519/2655 (`lift_other_waists.txt`).
   - My re-run of the on-site command on the wrist-pitch -0.15 record: rows and summary **identical** to the builder's
     (`onsite_wristpitch.{json,log}`, `onsite_compare.txt`), 42 s.
   - 87 `--factory-pose` refuses a record with q0 pitch -0.06 or roll 0.03 (exit 1, "skip table steps", no report written:
     `refuse_*.log`).
   - Tests: 7 passed (`tests_arm_paths.log`). My mutants (`mutants2.py`, `mutant_*.log`): recover = way_out, old lean grid,
     refusal on the upper bound only, refusal reading the wrong index - all caught. One survives (n2).
2. **Plan correctness: YES, with rulings pending.**
   - **B1 fixed.** `arm_paths.recover` goes back along the travelled way-in legs, or on along the way out; way_out only from
     policy/hold/shadow. 87 has recover rows from 5 points on every way-in leg; they pass wherever the way in passes. The
     way-out legs 4-6 are the way-in lines backwards, legs 1-3 are parts of lift-home lines. Logging the plan and leg is
     specified in S4 (86 not built yet, which is correct for this stage).
   - **B2 fixed.** `LEAN_PITCHES` starts at -0.05. The lower bound is ruling S5, and `measured_factory` refuses outside it.
     h(1.1, 0.1) now reads 4.0 cm, NOT clear.
   - **B3 fixed.** S1 is now an on-site 87 run on the recorded pose. The three off-grid poses read 2.8 / 2.2 / 0.1 cm.
     h(1.6, 0.1) still reads 8.9 cm. Residual tolerance: n1.
   - **B4 fixed** as ruling F5. All 8 fault reasons in `real_run.py` / `real_run_exits.py` now have a row:
     `lowstate_stale`, `contact_fault`, `invalid_target`, `second_signal`, `internal_error`, `weight_step`,
     `exit_path_overrun`, `signal_in_release`. `envelope_streak`, `operator_stop` and `policy_error` are soft exits.
3. **Overall: GO** for the design as the basis for S4, conditional on Finnegan's rulings F1-F5 and S1-S5. Nothing changes on
   the robot until S4 is built and reviewed. The runbook's "skip table steps" stands.

### New findings (minors, none blocking)

- **n1 (start-pose tolerance; same class as B3).**
  - The problem: 86 starts the way in from the *measured* q0 (`take_over`). `real_preflight.factory_problem` accepts any q0
    within `FACTORY_POSE_TOL` 0.10 rad per joint of the M0 record. S1's on-site 87 check covers the record only.
  - Probe (`probe_gap.txt`, worst of 45 cases): records that pass on site at 5.36 / 6.32 / 5.51 cm drop when one joint is
    moved by 0.10 rad:
    - shoulder pitch -0.10: to **1.2 / 2.2 / 1.8 cm**;
    - elbow -0.10: to 2.8-4.0 cm;
    - waist yaw +-0.10: to 4.2-5.2 cm.
    - h(1.6, 0.1) is unchanged at 8.9 cm.
  - In every one of these cases the closest point is the start pose itself (`probe_gap2.txt`). The path then stays 0.2 to
    1.0 cm further away than the start. So our motion adds no contact. The problem is that the hanging hand can already be
    closer than the 5 cm that S1 reported.
  - Fix direction (S4/S1): for table steps, either tighten the gap (e.g. 0.02 rad), or FK-check the measured q0 at run
    start (one pose), or run 87 with the record +- the tolerance.
- **n2 (test gap).** The mutant `recover` on the way in = "on to ready" (`plan[leg:]`) passes all 7 tests and would pass 87's
  recover rows. Its line is the clear way in, but it ends at the ready pose, and the ramp-out there is the 15.7-16.2 cm
  factory sweep. Fix: assert that `recover(...)[-1]` is the factory pose for both plans, and let 87's recover rows check the
  end pose. This is a test-only gap; the code is right.
- **n3 (S4 spec).** A fault or crash *during* a recover needs its own logged plan and leg. `recover()` accepts only
  `WAY_IN` / `WAY_OUT`.
  - With a stale "way_in leg k" log, a pose on leg j < k - 1 would get a corner-cutting line to `plan[k-1]` that nobody
    checked.
  - S4 should log recover legs too (a recover is "on along" its own waypoint list).
- **n4 (CLI).** `87 --factory-pose` without `--waypoint-paths` is ignored: the old grid check runs and exits 0. That should
  be a `parser.error`. The S1 command includes both flags, so it only matters for a mistyped command.
- **n5 (wording).** "Recover ... worst margin +0.36 cm" is the worst at the nominal setback. Over all setbacks the worst
  passing margin is 0.00 cm (the h(0.6, 0.3) rows at a 2 cm setback). The way-in pass test still uses rounded minima;
  this is documented and costs 0.05 mm.
- Round-1 minors m1-m8 and review C m6/m8 remain open, as SAFE-PATHS says. Not re-litigated.

### Commands and results (round 2)

| Command | Result | File |
|---|---|---|
| pytest tests/test_integration_arm_paths.py -m integration | 7 passed | /tmp/safepaths/review2/tests_arm_paths.log |
| 5 in-process mutants | 4 caught, 1 survives (n2) | /tmp/safepaths/review2/mutant_*.log |
| recount from runs/safe_paths_waist{0.6,0.35,0.15,0.0}.json | all SAFE-PATHS numbers match | verify_json_waist0.6.txt, lift_other_waists.txt, recover_margin.txt |
| 87 --waypoint-paths --factory-pose (wrist pitch -0.15 record) | identical to builder, 2.8 cm NOT clear | onsite_wristpitch.*, onsite_compare.txt |
| 87 --factory-pose with q0 pitch -0.06 / roll 0.03 | refused, exit 1 | refuse_*.log |
| probe: 0.10 rad start-pose tolerance | n1 | probe_gap.txt, probe_gap2.txt |

Not run: the full 8-setback grid re-run (I recounted the builder's JSON and re-ran one on-site case instead); anything on
86 (S4 not built); the 85 sim gate.
