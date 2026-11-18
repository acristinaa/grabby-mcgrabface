# Code review: safe arm paths built into 86 (SAFE-PATHS.md S4), 2026-11-06

Reviewer: independent. I did not write the build and saw no author reasoning. Scope: `git diff` (17 files) and the
untracked `g1sim/arm_paths.py`, `g1sim/arm_paths_clearance.py`, `tests/test_unit_real_run_paths.py`,
`tests/test_integration_arm_paths.py`, read against `.planning/real-robot/SAFE-PATHS.md` (S4, F1-F5) and
`.planning/reviews/safe-paths-design.md` (rounds 1-2). I also read the callers and consumers: `real_run.py`,
`real_run_policy.py`, `real_preflight.py`, `real_factory_pose.py`, `real_run_log.py`, `86_real_run.py`, and the runbook's
exit text. All my output is in `/tmp/safepaths/codereview/`. An earlier review attempt left files there (11-04 19:55-20:00)
but no review file; I did not read or use them.

## Verdicts

1. **Implementation fidelity: YES, with one deviation (m4).**
   - 86 now uses the waypoints from `arm_paths` for the way in, the soft exit and the fault recover. Every leg is a
     `real_safety.move` line (`move_through`). The unit test pins that 86 sends exactly `path_ticks(way_in)` and the way
     in backwards, and my mutant M3 (a straight line in `86 recover`) is caught.
   - The plan name and leg number are written to the sidecar at every leg start. The write is atomic (temp file + rename).
   - Faults follow `FAULT_ACTIONS`, which are the F1-F5 proposals as named constants. The recover drives the path first
     and ramps out afterwards. Mutants M2 (ramp out instead of recover) and M7 (ramp out before the path) are caught.
   - Exit clocks are planned from the tick count at the speed actually used (0.3 rad/s, or 0.15 for F2).
   - 87 `--waypoint-paths` (waist 0.6, setback 0) gives **row-for-row identical** way_in / recover / lift_home rows to the
     builder's fix-round report, so the S4 changes to `arm_paths.py` did not change what 87 checks. The on-site
     `--factory-pose` run on the h(1.6, 0.1) record is also identical.
   - Tests: 301 + 531 + 10 pass, the same as the builder's run.
   - Deviation: S4 said `86 release` would use the logged leg and refuse without one. Instead the build adds a new
     `86 recover` and leaves `release` as the straight line (m4).
2. **Plan correctness: NO, one gap (B1).** `86 recover` assumes the arms are still where the last sidecar says. Neither
   the design nor the code checks that.
3. **Overall: NO-GO**, because of B1 only. The fix is small: a preflight check plus one test. The rest of the build is
   sound, given the pending rulings F1-F5. The robot is not affected today: nothing is committed, and the runbook still
   says "skip table steps".

## Behaviour bugs

**B1. `86 recover` drives the logged plan from wherever the arm is, without checking that the arm is still on it.**
- **Where:** `real_run_modes.recover` → `arm_paths.home(measured, last, ...)`. Its only preflight is
  `real_preflight.arm_path_problem`, which checks that a log *exists*. `recover` is not in `ARM_MODES`, so there is no
  factory-gap check and no `arms_may_be_held_by_us` check.
- **Trigger (reachable):** the arms left the logged leg before `86 recover` runs. Three ways this happens:
  - (a) A F4/F5 freeze ended by the next Ctrl+C. It ramps out in place, and Unitree's controller takes the arms home. The
    sidecar still holds the leg or phase of the freeze. 86 prints FAULT EXIT, and the runbook's "after any crash" box
    applies.
  - (b) A kill or cable pull where Unitree does not latch our last message. F1 notes that latching is an unpublished
    assumption.
  - (c) The owner moved the arms with the remote.
- **Evidence, through the real runner** (test fakes, no DDS; `repro_recover_after_handback.{py,txt}`):
  - Sequence: `ready` from h(1.6, 0.1), the clearest grid pose, then 3 Ctrl+C (soft exit, freeze, hand back). The fake arm
    is then at the factory pose. `86 recover` follows.
  - Result: exit 0 DONE. It sends factory → side up → swing back → factory, because the sidecar says "recover_path leg 1".
  - That path comes **3.6 cm** from the basket in 87's case (roll +0.02, pitch -0.05, table +1 cm), and 4.5 cm in the
    default scene. The bar is 5 cm.
  - The factory → side-up line is the one SAFE-PATHS rejected (probe4/5).
- **Evidence, `arm_paths.home` from the factory pose with other logged states** (`probe_recover_offleg.txt`, worst of the
  45 cases, setback 0):

  | Logged state | h(1.6, 0.1) | h(1.5, -0.1) | h(0.8, 0.3) |
  |---|---|---|---|
  | phase policy / hold (way out from here) | **-9.7 cm** table | **-11.7** | **-7.2** |
  | way out leg 2 (waist home) | **-13.9** | **-14.3** | **-11.6** |
  | way out leg 1 (lift, from the grasp frame 153) | **-14.8** | **-15.1** | **-12.2** |
  | way in leg 3 | 3.6 basket | **-2.0** table | 0.0 basket |

- **Why:** the logged leg is valid only while the arm sits on that leg. `recover()` then joins "where the arm is" to
  `plan[leg-1]` or `plan[leg]` with a straight line that nobody has checked.
- **Fix direction (smallest):** in `arm_path_problem`, refuse `86 recover` in two cases:
  - (a) The measured pose is within `FACTORY_POSE_TOL` of the logged q0 or the factory record. The message: "the arms are
    already home, nothing to recover".
  - (b) The logged plan is a waypoint plan, and the measured pose is more than `LEAD_LIMIT` (per joint) from the segment
    `plan[leg-1]`..`plan[leg]`.
  - For policy/hold/shadow logs there is no logged pose. Either log the last command at those phase changes, or accept
    only when (a) does not hold and `arms_may_be_held_by_us()` is true.
  - Add one unit test, "recover after a hand-back refuses", and see it fail on today's code first.

## Minors

- **m1. Four safety checks were never seen failing.** In each case the code is right, but no test catches its mutant
  (`mutants.py`, `mutant_M*.log`; 59 relevant unit and integration tests run on a copy):
  - M1: the fault recover keeps steering on stale LowState (`"recover"` added to the stale pass-through in
    `measured_now`). **Survives.**
  - M4: `_lowstate_back` ignores the `LEAD_LIMIT` condition. **Survives.**
  - M5: no exit clock in the fault recover. **Survives.**
  - M6: a Ctrl+C during a fault recover is ignored. **Survives.**
  - Caught: M2, M3, M7 (path before ramp-out), M8 (stale hold on the measured angles instead of the last command).
  - The builder reported "10 mutants all caught"; these four were not among them. Each needs one short test. M1 is the
    "exit path with stale feedback" case this review was asked to look at.
- **m2. F1 when LowState comes back but the arm is not where we sent it.**
  - Where: `_recover` → timeout → `_fault_target(runner, "lowstate_stale")`.
  - What happens: after `STALE_RETURN_WAIT_S`, with fresh LowState but lead > 0.25 rad, 86 holds the *last command*. The
    hold is unfiltered, at weight 1, without the lead monitor, and lasts until a signal.
  - Why it matters: if the arm was pushed, or Unitree moved it while the link was down, this pulls it back by more than
    0.25 rad at PD speed and keeps pushing (the M2 situation).
  - Proposal: once LowState is fresh again, hold the fresh measured angles, as for a contact. This is part of ruling F1.
- **m3. Timing margin.**
  - `EXIT_PATH_MARGIN_S` stays at a fixed 1.0 s. The paths are now up to 24.0 s, plus 2 s pause and 2 s ramp-out, for the
    soft exit. The F2 recover runs up to 47.9 s. So the margin is 3.6 % / 2.1 % of the path (`probe_timing.txt`).
  - `Runner.wait` drops any lateness above 20 ms for good, so a few dozen hiccups of 40 ms use up the margin.
  - The result is `exit_path_overrun` → HOLD (F5) in the middle of the path at the table. Only the remote or a Ctrl+C
    (Unitree's own path) ends it.
  - Proposal: 1 s + 10 % of the planned time.
  - The builder's e2e had a max gap of 0.034 s and no overrun.
- **m4. Fidelity deviation from S4.**
  - `86 release` is unchanged: a straight line, rule 13. The 84 / `factory_problem` message still says "run `86 release`
    first", and so does the runbook (l.398, 565, 572).
  - Nothing points the operators to `86 recover`.
  - Covered for now by "skip table steps"; it needs the runbook step when the rulings come.
- **m5. The docs no longer match the code. Must be fixed before this 86 goes to the robot.**
  - Runbook l.373-377 says "Fault exit: the arms lower from wherever they are within about 2 s", and that Ctrl+C twice
    gives a fault exit.
  - The code now holds and then recovers (10-48 s), or freezes until a 3rd Ctrl+C.
  - PLAN.md rules 12/13 are unchanged, and the `FAULT_ACTIONS` values are proposals nobody has ruled on.
  - This affects every M1-M6 step, not only the table steps: every `ready` now swings the arms back and out before it
    raises them.
  - The operators are told "so nobody reaches in", so the text must match what the arms do.
- **m6. Open items carried over from the design review (unchanged by the build):**
  - n1: 86 accepts a start pose within 0.10 rad of the record, and the hanging hand can then be 1.2-4 cm from the table.
  - n4: 87 `--factory-pose` without `--waypoint-paths` is still silently ignored. I confirmed this in `main()`.
  - m5: under `--waist-hold`, the filter sends q0's yaw on the side-up and ready legs, not the yaw 0 that 87 checked.
  - m3: a contact recover starts at the measured pose, up to 0.25 rad off the checked line.
- **m7. Robustness.** In `_recover`, `arm_paths.home` and `move_through` run before the `try`. An exception there would
  leave `fault_exit` with no hold and no ramp-out, and 86 would end at weight 1. I found no trigger today: every
  weight-1 phase logs an arm path first. Moving both lines into the `try` makes such a case fall back to HOLD.

## Checked and fine

- **Stale feedback:**
  - During the way in, a soft exit or `release`, stale feedback → hold the last command → recover once LowState is fresh
    and within `LEAD_LIMIT`, otherwise keep holding.
  - During a fault recover, it stops the recover and holds.
  - During ramp-in or ramp-out (weight < 1, arms at q0), it ramps out.
  - At the start of `86 recover`, before the first message, it faults with nothing sent.
- **Recover ordering:** in-run and in `86 recover`, the path runs first, then the ramp-out at q0 / waypoints[-1] (M7
  caught).
- **Leg indexing:** `recover` on the way in returns plan[leg-1]..plan[0]. On the way out or the recover path it returns
  plan[leg]..plan[-1]. Soft-exit legs keep counting across to_ready / to_start, and the hands-open pause sits at way-out
  leg 3.
- **Straight lines left:**
  - Only `86 release` (rule 13, m4).
  - The ramp-out in place after a freeze plus Ctrl+C (Unitree's path, the operator's choice).
  - Nothing else calls `rs.move` toward the factory pose.

## Commands and results

| Command | Result | File |
|---|---|---|
| `scripts/run_tests.sh` | 301 + 531 + 10 passed, exit 0 | /tmp/safepaths/codereview/tests_all.log |
| 87 `--waypoint-paths --waist-limits 0.6 --setbacks 0.0` | 169 s; elbows 1.5/1.6 clear, lift home 5265/5265; rows identical to runs/safe_paths_waist0.6.json at setback 0 | wp_waist0.6_sb0.{json,log}, compare.txt |
| 87 `--waypoint-paths --factory-pose record_h1.6_0.1.json` | 42.5 s, 8.9 cm clear, identical to the builder's | onsite_h1.6.{json,log}, compare_onsite.txt |
| 8 mutants on a copy (59 tests) | 4 caught (M2, M3, M7, M8), 4 survive (M1, M4, M5, M6) | mutants.py, mutants_summary.txt, mutant_M*.log |
| repro: ready + 3 Ctrl+C, then `86 recover` | exit 0, 3.6 cm (B1) | repro_recover_after_handback.{py,txt} |
| probe: `arm_paths.home` from the factory pose | -15.1 to +3.6 cm (B1) | probe_recover_offleg.{py,txt} |
| probe: exit path durations | soft exit up to 24.0 s, F2 up to 47.9 s vs 1 s margin (m3) | probe_timing.{py,txt} |
| e2e (exit tests: release after kill, server death, Ctrl+C x3, LowState pause) | 4 passed, 218 s, no exit overrun | e2e_exits.log |

Builder's full e2e (`/tmp/safepaths/build/e2e.log`, run after the last edit): 6 passed, 2 failed. The 2 failures are the
known gravity limits from review C (L1 sag 0.0298 vs 0.02, L2 lead 0.255 vs 0.15), the same class and numbers as before.
I take that as conclusive for the other tests.

## Rulings for Finnegan

- B1 guard: should `86 recover` refuse when the arms are already home or off the logged leg? (Proposed: yes.)
- F1 after LowState returns with the arm more than 0.25 rad off: hold the measured angles, not the last command (m2).
- Exit margin: 1 s + 10 % instead of a fixed 1 s (m3).
- F1-F5 themselves, and then runbook l.373-377 and PLAN rules 12/13 rewritten to match. Until then, do not take this 86
  build to the robot (m5).
- Where the runbook sends operators after a crash at the table: `86 recover` or `86 release` (m4).

---

## Round 2 (2026-11-06): re-check of the B1 fix

Reviewer: independent. I did not write the fix and saw no author reasoning. Scope: the new `real_preflight.arm_path_problem`
(already-home and off-leg checks, `leg_distance`), its wiring in `run_preflight`, the new test
`test_recover_refuses_when_the_arms_are_not_where_the_log_says`, and the callers (`real_run.Runner.run`, `log_arm_path`,
`real_run_modes.recover`/`release`/`hold`, `real_run_policy` shadow, `arm_paths.home`). All my output is in
`/tmp/safepaths/b1review/`.

### Verdicts

1. **Implementation fidelity: YES.** The fix does what the task said. `86 recover` refuses with nothing sent when the
   measured pose is within `FACTORY_POSE_TOL` of the logged q0 or the record. It also refuses when a waypoint-plan log
   names a leg and the arm is more than `LEAD_LIMIT` from every message of that leg. The refusal runs in preflight,
   before any writer exists. My re-run of round 1's repro (ready, 3 Ctrl+C, then `86 recover` from h(1.6, 0.1)) now
   exits 2 "already home" with 0 messages; round 1 had exit 0 and 3.6 cm. The 3 builder mutants (old code, no home
   check, no leg check) each fail the new test.
2. **Plan correctness: NO, one route is still open (B1-r).** The off-leg check skips the free phases (`hold`, `shadow`,
   `policy`). Round 1's fix direction covered these phases separately, and the fix leaves them out. Route (c) from
   round 1 (the owner moves the arms with the remote) is still open for these logs. So is a recover started while the
   arms are not yet settled.
3. **Overall: NO-GO**, for B1-r only. Nothing else broke: tests 301 + 532 + 10 pass. The robot is not affected today,
   because nothing is committed and the runbook still says "skip table steps".

### Behaviour bug

**B1-r. With a `hold`, `shadow` or `policy` log, `86 recover` still drives home from wherever the arms are.** The only
condition is that the arms are more than 0.10 rad from home.
- **Where:** `real_preflight.py` returns None for `FREE_PHASES` right after the home check, with the comment "no logged
  leg to compare with". `arm_paths.home` then plans `way_out(measured, ...)` from that pose.
- **Trigger (reachable):**
  - A run is killed, or the cable is pulled, in the hold at the ready pose. The sidecar keeps phase `hold` and
    `exit_code` null.
  - Then the owner moves the arms with the remote. The refusal texts themselves tell operators to call the owner and
    the remote. The owner leaves them somewhere other than within 0.10 rad of home.
  - Then `86 recover` runs.
  - The same happens after a HOLD fault (F5) in the policy phase plus Ctrl+C, if `86 recover` starts before Unitree's
    controller has settled the arms, or after the owner moved them.
- **Evidence through the real runner** (test fakes; `repro_free_phase.{py,txt}`):
  - Setup: `ready` from h(1.6, 0.1), killed at 22 s in the hold. The log reads phase hold, way_in leg 3. Arms moved:
    both elbows +0.15 rad.
  - Result: `86 recover` exits 0 DONE and sends 1297 messages over 6 legs.
  - Clearance: **-9.9 cm** into the table worst case (roll -0.02, pitch 0.126, table +1 cm). **-9.4 cm** in the default
    scene.
- **Evidence from the probe** (`probe_free_phase_offhome.{py,txt}`):
  - Poses: h(1.6, 0.1), h(1.5, -0.1) and h(0.8, 0.3), each with one of 5 offsets of 0.15-0.30 rad on the elbows or
    shoulder pitches.
  - `arm_path_problem` accepted all 15 cases.
  - The paths go **-5.0 to -14.1 cm** into the table, worst over 45 cases.
- **Why the leg check does not cover it:** the logged pose is not needed for `hold` and `shadow`. Both phases send the
  logged `ready17` (`hold()`, and shadow "always holds the ready pose"). Only `policy` has no logged pose.
- **Fix direction (small):**
  - (a) For `hold` and `shadow`, refuse when the measured pose is more than `LEAD_LIMIT` from the logged `ready17`. This
    is the same rule as the leg check.
  - (b) For `policy`, there is no logged pose. Options for Finnegan:
    - refuse `86 recover` from a policy log altogether: owner's remote, or table back and `86 release`;
    - or log the last sent command to the sidecar about once a second and compare against it with `LEAD_LIMIT` plus
      the policy's speed cap times that interval.
  - (c) Also refuse when `real_io.arms_may_be_held_by_us(out_dir)` is False. That means the last run finished and
    handed back at weight 0, so the arms belong to Unitree and no sidecar describes them. This one line covers the
    hand-back route for every phase. It does not cover the kill route (exit_code null), which is why (a) and (b) are
    needed.
  - Add one test case each and see it fail first.

### Minors

- **r2-m1. The free-phase "already home" refusal is never seen failing.** In my mutant, the `FREE_PHASES` early return
  moves above the home check, so a hold or policy log skips it. The 4 real-run unit files still pass (46 passed,
  `mutants_summary.txt`). This case is round 1's worst row: -7.2 to -11.7 cm into the table from a policy/hold log at
  the factory pose. The new test only uses a `to_start`/`recover_path` log. One more `run("recover", ...)` with a hold
  log and the arm at home closes it. A second mutant also survives: dropping the logged q0 from `homes` and keeping only
  the record. This weakens the check less, because the record is today's factory pose too.
- **r2-m2. Wrong refusal text after a crashed `86 release`.**
  - Cause: `release()` sets `runner.q0 = measured`, the crash pose, and logs it as the arm path's q0.
  - Effect: if a release is killed within 0.10 rad of its start, a following `86 recover` refuses with "the arms are
    already home (factory pose)". The arms are at the crash pose near the table.
  - Safe, because nothing moves, but the text misleads the operator. It should say to run `86 release` again.
- **r2-m3. The "already home" text gives no next step.** It says "do not run 86 recover now". When an arm is within
  0.10 rad of home but still latched at weight 1 (kill with latch during ramp-in or way-in leg 1), the next step is
  `86 release`, a short straight line near home. The text could say that.

### Not run

- The e2e suite (85 sim). The fix is a preflight refusal for `mode == "recover"` only, and no e2e test drives
  `86 recover`. I take the unit runs through the real runner as conclusive for this change.

### Commands and results (round 2)

| Command | Result | File (/tmp/safepaths/b1review/) |
|---|---|---|
| `scripts/run_tests.sh` | 301 + 532 + 10 passed, all exit 0 | tests_all.log |
| round 1 repro on the fixed code | recover exit 2 "already home", 0 messages (round 1: exit 0, 3.6 cm). The script's own clearance print fails on 0 messages, as expected. | repro_after_fix.txt |
| builder's 3 mutants (`/tmp/safepaths/b1fix/mutants_summary.txt`) | all 3 caught by the new test | (builder's) |
| my 2 mutants, 4 real-run unit files | both survive (r2-m1) | mutants.sh, mutants_summary.txt, mutant_*.{diff,log} |
| repro: kill in hold, arms +0.15 elbow, `86 recover` | exit 0 DONE, -9.9 cm worst, -9.4 cm default scene (B1-r) | repro_free_phase.{py,txt} |
| probe: free-phase log, 3 poses x 5 offsets | 15/15 accepted, -5.0 to -14.1 cm (B1-r) | probe_free_phase_offhome.{py,txt} |

### Rulings for Finnegan

- `86 recover` after a policy-phase log: refuse altogether, or log the last command about once a second (B1-r (b)).
- Refuse `86 recover` whenever the last run already handed back at weight 0 (B1-r (c))? Proposed: yes.

---

## Round 3 (2026-11-06): re-check of the B1-r fix

Reviewer: independent. I did not write the fix and saw no author reasoning. Scope: `real_preflight.arm_path_problem`
(new: hand-back check via `real_io.arms_may_be_held_by_us`, hold/shadow vs logged `ready17`, `RECOVER_FROM_POLICY_LOG`,
new refusal texts), its wiring in `run_preflight`, the new tests in `tests/test_unit_real_run_paths.py`, and the callers
(`real_run_modes.recover`/`release`/`hold`/`policy`, `real_run_exits` soft/fault paths, `real_run_policy` shadow,
`real_run_log.note_arm_path`, `real_factory_pose.last_arm_path`). All my output is in `/tmp/safepaths/r3review/`.

### Verdicts

1. **Implementation fidelity: YES.** Round 2's repro (kill in hold, elbows +0.15, `86 recover`) now exits 2 with
   0 messages ("2.55 rad from ... the ready pose of its hold phase"). Round 2's 15-case probe, re-run with the new
   signature: 30/30 refused (hold and policy), both with `held_by_us` True and False. Boundaries: hold and shadow accept
   0.24 rad off `ready17` and refuse 0.26; policy refuses always; `held_by_us` False refuses every phase. r2-m2 and
   r2-m3 are fixed in the texts (q0 may be the crash pose of a release; next step `86 release`).
2. **Plan correctness: YES, with one ruling (r3-m1).** No route found where recover drives into the table. I hunted
   other phases (ramp_in, to_ready, to_start, to_factory from release/recover/fault recover, fault hold, soft-exit pause
   and ramp-out), stale logs (other day, other `--out`, refused runs: none counts as an arm-writer sidecar), NaN readings
   (refused by `joint_limit_problem`), missing fields (only reachable with hand-edited sidecars; KeyError before any
   writer) and partial legs. What remains is by design: a pose accepted within `LEAD_LIMIT` of the logged leg or ready
   pose starts with a line 87 never checked (87's recover rows sit exactly on the legs).
3. **Overall: GO.** No behaviour bug. Tests 301 + 535 + 10 pass.

### Behaviour bugs

None.

### Minors

- **r3-m1 (ruling). The 0.25 rad acceptance is wider than 87's evidence.** 87 checks recover only from poses exactly
  on the way-in legs; `86 recover` accepts anything within `LEAD_LIMIT` (0.25 rad, max over joints).
  - Single-joint offsets of 0.24 rad (elbows or shoulder pitches, 3 factory poses, hold + 9 leg points, 150 accepted):
    all at least 4.68 cm clear; 3 fail 87's rule by 0.2 mm (basket 4.70 -> 4.68 cm). `probe_within_tol.txt`,
    `probe_fail_detail.txt`.
  - Corners of the box (all 14 arm joints +-0.24 at once, 112 samples, 2 worst scene cases): no path that starts clear
    goes into an object, but 10 physically possible starts fail 87's rule, worst 5.03 -> 1.91 cm (table) and
    1.23 -> 0.18 cm (stand). `probe_random_corners.txt`.
  - Same probe at 0.10 rad: worst 2.05 cm absolute, 1 rule fail. `probe_random_corners_010.txt`.
  - After a kill the arm is either latched at the last command (on the leg) or dropped far (refused), so the wide corner
    is unlikely. The in-run fault recover has the same tolerance class. Options: use `FACTORY_POSE_TOL` (0.10) for
    `86 recover`, or add off-leg start rows to 87.
- **r3-m2 (test only). Three checks are never seen failing** (my mutants, 4 real-run unit files, `mutants_summary.txt`):
  shadow skipping the ready-pose check (no shadow-log test); the hold bound halved (`off / 2`; the test's offset is far
  beyond the limit, so the 0.25 bound is not pinned); dropping the logged q0 from `homes` (r2-m1's second mutant, still
  surviving). The other 5 of my mutants are caught (hand-back check removed or not wired, hold/shadow check removed,
  policy accepted, free-phase return above the home check).
- **r3-m3 (ruling). After a killed `86 release`, `86 recover` goes on along release's straight line.** `release` logs
  `RECOVER_PATH [crash pose, factory]` leg 1. If the arm is more than 0.10 rad past the crash pose and on that line,
  recover drives the rest of it, a line 87 never checked. This is safe under release's own rule (table pulled back
  first). But an operator may take `86 recover` as the safe-at-the-table choice. Option: when the newest sidecar's mode
  is `release`, refuse and say "run `86 release` again, table pulled back".

### Not run

- The e2e suite (85 sim). The change is a preflight refusal for `mode == "recover"` only, and no e2e test drives
  `86 recover`.

### Commands and results (round 3)

| Command | Result | File (/tmp/safepaths/r3review/) |
|---|---|---|
| `scripts/run_tests.sh` | 301 + 535 + 10 passed, all exit 0 | tests_all.log |
| round 2 repro (kill in hold, elbows +0.15) | exit 2, 0 messages (round 2: exit 0, -9.9 cm). The script's clearance print fails on 0 messages, as expected. | repro_free_phase.txt |
| round 2 probe, 3 poses x 5 offsets x hold/policy | 30/30 refused, held and handed back | probe_held.txt, probe_handedback.txt |
| boundaries + missing fields | hold/shadow 0.24 accept, 0.26 refuse; policy and held False refuse | check_boundaries.txt |
| within-tolerance probe (single joint 0.24) | 150 accepted, worst 4.68 cm, 3 fail by 0.2 mm (r3-m1) | probe_within_tol.txt |
| box-corner probe 0.24 / 0.10 | worst from a clear start 0.18 cm / 2.05 cm (r3-m1) | probe_random_corners*.txt |
| my 8 mutants on a scratch copy | 5 caught, 3 survive (r3-m2) | mutants.py, mutants_summary.txt, mutant_*.log |

### Rulings for Finnegan

- `86 recover` tolerance: keep 0.25 rad (`LEAD_LIMIT`), or 0.10 rad, or 87 off-leg rows (r3-m1).
- `86 recover` after a killed `86 release`: refuse and send to `86 release` again (r3-m3)? Proposed: yes.
- Still open from round 2: policy-log recover (S6, now refused).
