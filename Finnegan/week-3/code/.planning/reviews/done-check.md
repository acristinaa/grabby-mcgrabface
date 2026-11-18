# Review: 86 done check (uncommitted diff, 2026-11-16)

Reviewer: fresh context, did not author the change, no author reasoning seen.
Scope: `git diff` (g1sim/real_run.py, g1sim/real_run_policy.py, scripts/86_real_run.py, tests/test_unit_real_run.py,
tests/test_unit_real_run_fence.py) + untracked g1sim/done_check.py, tests/test_unit_done_check.py,
tests/data/done_check_hands.npz. Docs (runbook, solo page) reviewed as notes only (being edited in parallel).
`.planning/REVIEW-LENSES.md` does not exist in this repo; the "what becomes reachable" lens was applied by hand.

## Verdicts

1. Did the code do what the spec said: **YES.**
2. Was the spec right: **YES**, with one plan-level note (F6: at --speed 0.25 the 60 s cap leaves too little policy time
   for the check to ever fire; that predates the change).
3. Overall: **GO.** No blocking findings. Not re-derived: the 50-trial calibration numbers (data not on this Mac, see
   "Not run"); the check can only end the phase earlier through the existing soft exit, so those numbers are not
   safety-bearing.

## Evidence

- Unit suite (command from the brief): `477 passed, 2 skipped, 148 deselected in 62.54s`, exit 0.
- Mutation runs on scratch copies (repo files untouched), 6 real_run/done_check test files, 62 tests:
  | mutant | result |
  |---|---|
  | `_check_done` not called | caught (`test_done_check_ends_run_after_one_place[True]`) |
  | de-dup removed (feed every 50 Hz tick) | caught (`[True]`: fires at ~10.6 s instead of 11-13 s) |
  | `--no-done-check` flag ignored | caught (`[False]`) |
  | wrong exit reason | caught (`[True]`) |
  | relax stage dropped | caught (4 tests in test_unit_done_check.py + `[True]`) |
  | **mode gate removed (`runner.mode == "run"` dropped)** | **survives** (F1) |
  | `_check_done` moved before `r.tick` | survives (F3, not a spec violation) |
- Probe with the real runner and fakes (scratch script, grip -> release -> relax schedule): run 0.25 -> DONE after
  23.9 s wall = 5.975 s policy time; run 0.5 -> 12.02 s wall = 6.01 s policy time (same policy time: the de-dup is right
  at both speeds); replay 0.5 and shadow with the same actions -> no done_check, run to --max-seconds.
- Fixtures re-run through DoneCheck: placed -> done at action 601 (20.03 s; gripping 304, released 530);
  put_on_table -> 457 (15.23 s); never_released -> none (gripping 313 only); released_not_relaxed -> none (released 1286
  of 1350).
- Motion gate: after importing scripts/86_real_run.py, `motion_gate.loaded_repo_files(REPO)` contains
  `g1sim/done_check.py` (import at top of real_run_policy.py, which real_run_modes imports). The unlock pins it - once it is
  committed; while untracked, gate_problems reports "not in the unlocked commit".
- CLI: `parse_args(run ...)` -> `run_args().done_check` True; with `--no-done-check` False; shadow True (mode-gated in
  PolicyPhase); `replay --no-done-check` is rejected by argparse (flag exists only on `run`).

## Questions from the brief, answered

- **Fire on held / stale actions?** No. `scheduler.target()` returns None after a chunk ran out and before the first one
  (real_timing.py:76-97); `_check_done` returns on `action is None` (real_run_policy.py:114) and never sees `hold17`.
- **Blend crossfade?** The fed value is the blended target, i.e. exactly what is commanded. BLEND_ACTIONS = 5 is shorter
  than every stage duration (grip 15, release 9, relax 30 actions), so a crossfade alone cannot complete a stage; it can
  only move a stage boundary by a few actions.
- **Mid-grasp?** Release needs all 12 hand values <= 0.05 for 9 actions after a 15-action grip, i.e. the policy fully opens
  both hands; a ball cannot still be held by policy command at that point. A drop / put-down followed by 1 s of ready hands
  does end the phase (spec-accepted "attempt over").
- **Raise location.** `_check_done` runs after `r.tick` and `_log_frame` (real_run_policy.py:105-107) and raises inside
  `run()`'s try, so `finally` still prints the shadow summary (n/a in run) and the FENCE SUMMARY + sidecar fence block,
  then `_drive` -> `soft_exit(self, "done")` -> exit 0 "done", identical to the --max-seconds path. Only
  `scheduler.advance` of the last tick is skipped (irrelevant, the phase ends). It never alters target17/hands and does
  no I/O beyond one print + one sidecar event: no blocking.
- **De-dup at 0.25/0.5.** Position advances 0.15 / 0.30 actions per tick (< 1; --speed is capped at 0.5 by
  real_safety.speed_refusal), so every integer index is visited and fed once; confirmed by the probe above.

## Findings

**F1 - Low (test gap). The "run only" gate is not tested.** real_run_policy.py:77. Mutant dropping
`runner.mode == "run"` passes all 62 tests: no replay/shadow test feeds a grip -> release -> relax schedule (demo_actions
hold constant ready hands). Impact if it regressed: replay (M3/M4 demos contain exactly that schedule) and shadow would end
early - not unsafe, but the replay gate would see a shortened run. Current code is correct (probe). Fix if wanted: add
replay and shadow cases to `test_done_check_ends_run_after_one_place` asserting no `done_check` event and policy_s at the
cap.

**F2 - Low (test gap). `--no-done-check` -> `RunArgs.done_check` in 86 is not tested.** scripts/86_real_run.py:83-84,199.
The integration test sets `RunArgs(done_check=...)` directly; inverting the `not` in `run_args` would pass. Checked by hand:
mapping is correct.

**F3 - Info. Order inside `_step` is not pinned.** Moving `_check_done` before `r.tick` survives; harmless (one tick less),
listed only because the brief asked whether tests would catch skipped steps.

**F4 - Info. The check reads the policy's commanded hands, not what reached the fingers.** real_run.py tick: hand targets are
only commanded when the fence did not hold that message. If the fence held through the whole release window the check
could fire while the fingers are still closed. Consequence: soft exit opens the hands at the ready pose (ball drops on the
table) - the same as --max-seconds at that moment. No spec violation; the runbook could say "the policy's hand commands"
(it does).

**F5 - Info. Ticks with no action do not reset a stage's count.** A chunk gap inside the relax stage pauses the count rather
than restarting it. It cannot shorten the wall time to DONE (gaps add time) and a gap > 1 s is a policy_stale exit anyway.
DoneCheck.events (stage transitions with action index) is not written to the sidecar; logging it would help read robot-day
runs (optional).

**F6 - Medium (plan/doc note, predates this change). At --speed 0.25 the default --max-seconds 60 gives only 15 s of
policy time.** Policy time = wall x speed. The recorded success reaches DONE at 20.0 s policy time (latch 22.5 s); even the
put-on-table trial needs 15.2 s. So at 0.25 a place essentially cannot finish within 60 s: the check will not fire, the run
ends on the cap, likely with the ball in hand, and the soft exit drops it at the ready pose. At 0.5 (30 s policy time) the
check is reachable. The solo page line "then 0.5, 60 s each at most; the done check ends an attempt right after a place"
reads as if it applies to 0.25 too. Owner decision: either say the 0.25 attempts are motion checks that will be cut at 60 s,
or give 0.25 a larger --max-seconds (e.g. ~100 s) now that the done check ends finished attempts early.

**Doc notes (runbook, being edited in parallel):**
- ROBOT_DAY_RUNBOOK.md:376 "Soft exit (normal end, one Ctrl+C, exit code 3)" - a normal end (cap or done check) exits 0
  `86 RUN: DONE` (EXIT_CODES done=0); only Ctrl+C/stale inputs give 3. Predates the change, but the new M6 paragraph says
  "exit 0", so the two now contradict.
- New M6 paragraph: between the DONE CHECK line and `86 RUN: DONE` come `FENCE SUMMARY ...`, `soft exit (done): planned ...`
  and `MAX LEAD ...` (seen in the probe minus the fence line). "then" is fine; "a few lines later" would avoid confusion.
- "60 s each at most" is still true (cap unchanged), see F6 for 0.25.

## Not run

- The 50-trial calibration (30/30 successes, 1 fence_exit, 1 put-on-table, 18 none, 1.4-3.6 s before latch): the trial data
  (runs/states_run3) is not on this Mac; only the four fixtures were re-derived (above). Not a gate condition for the code:
  the check can only end the phase earlier via the existing soft exit.
- No DDS, sim or policy server started (as instructed).

## Applied (main session, 2026-11-16)
- F1: test_done_check_stays_off_in_shadow (mutation: drop the mode check -> fails). F2: test_no_done_check_option_reaches_the_runner.
- F6: M6 lines --max-seconds 180 (0.25) / 90 (0.5); solo page wording. Doc note: runbook soft exit line now says exit 0 for a normal end, 3 after Ctrl+C / stop rule.
- F3-F5: info, not changed.
