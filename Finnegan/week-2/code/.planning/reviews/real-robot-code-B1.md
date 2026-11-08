# Code review: Track B1 (safety layer)

## Track B1 - round 1

Reviewer: independent code reviewer (Claude Opus 5.5). I did not write this track and saw none of the author's reasoning.
Date: 2026-10-18, started 17:54.
Scope: `g1sim/real_safety.py` (272 lines), `g1sim/real_timing.py` (108 lines, split out by the author), `tests/test_unit_real_safety.py`
(320 lines), checked against PLAN.md rev 6.3 sections 0, 3.3, 4.2 and the B1 task. Consumers looked at: `g1sim/real_preflight.py`,
`g1sim/real_send.py`, `g1sim/real_run_policy.py` (B2, still being written), `g1sim/real_io.py` / `g1sim/real_camera.py` `latest()`.

### Runs (full output in runs/)
- `runs/real_b1_review_r1_unit.log`: sim env (`~/dev/g1-ext/.venv`), 24 passed in 0.16 s, nothing skipped (stats.json present, so the
  envelope and gains pin test really ran).
- `runs/real_b1_review_r1_unit_project.log`: project `.venv`, 23 passed and 1 skipped with a reason (table_env needs the sim env). This
  is what the plan expects.
- `runs/real_b1_review_r1_py310.log`: `import g1sim.real_safety` under the operators' Python 3.10 robot env
  (`~/dev/g1-ext/robot-day-test/robot-env`) exits 0. `str | None` and `tuple[...]` hints work on 3.10, and `g1sim/__init__.py` imports
  only `pathlib`, so there are no heavy imports.
- `runs/real_b1_mutation.log` (the author's): all 10 mutations caught. I read the script: each one runs on a temp copy and makes the
  named test fail. I accept it.
- `runs/real_b1_review_r1_mutation.log` (mine, temp copies, repo untouched): 6 more mutations. **4 survived** (no test failed):
  latency skip removed, rate limit skipped in the policy phase, ramp_in no longer frozen, envelope_streak never raised. 2 were caught:
  camera staleness ignored, release speed raised to 1.0 rad/s.
- `runs/real_b1_review_r1_probes.log`: edge-case probes. The code does the latency skip correctly (chunk requested at 0 and arrived
  after 10 actions: first target 10.0). NaN `--waist-max` / `--speed` pass the refusal functions. A filter built with NaN waist_max
  outputs a NaN waist command. `fault_targets` passes NaN measured angles through unchanged.

### Verdict 1: did the code do what the plan says?
Mostly yes. Every constant, function, class and event in contract 3.3 is there with the contract's signature and meaning. The filter
order is exactly as section 3.3 says: NaN, waist, envelope (policy only), hard clip, 13/14 := q0, rate limit, lead at w == 1 only.
`release_plan` has the one allowed jump, then a 0.3 rad/s line, then a ramp-out at the factory pose. `soft_exit_plan` goes through the
ready pose and has no time cap. `fault_targets` and `exit_kind` follow rule 12. The stale thresholds follow rule 10, and the scheduler
follows rule 14. All 19 tests in the plan's list are present (plus 3 extra), and each has a Proves/Catches docstring and an AAA layout.
The code is short and readable, every safety constant has a named source comment, and the error messages tell the operator what
happened.

The gaps: three tests claim to catch a bug they cannot catch (mutation shown surviving), there are a few unexplained numbers, and a
new file is not in the plan's file list.

### Verdict 2: is anything in the plan wrong for real hardware?
Nothing blocks. One unmeasured number is worth adding to the plan (finding 8).

### Findings

**1. MAJOR: `test_scheduler_skips_latency_and_raises_when_chunk_too_late` does not prove the latency skip, though its Catches line
says so.**
- File: `tests/test_unit_real_safety.py:279-293`, `g1sim/real_timing.py:61`.
- Trigger: mutate `self._chunk, self._start = chunk, requested_at` to `... = chunk, self.position`. This is the exact bug the docstring
  names: the chunk is replayed from action 0 when it arrives. The whole test file still passes (my mutation 1).
- Why: the test never reads `target()` right after `add_chunk`. It only checks "ran out" and "ChunkTooLate". Both still hold when the
  chunk starts at its arrival position. The interpolation test requests and adds at the same position, so it cannot tell either.
- Impact: with real network/inference latency the arm would lag behind what the policy saw, and no unit test would notice. Rule 14
  ("latency skip") is unproven.
- Fix: in the Act/Assert of this test (or a split-out `test_scheduler_starts_chunk_at_the_action_matching_its_latency`), assert
  `sched.target()` equals 5.5 right after `add_chunk(chunk, requested_at)` while position is 5.5. Then show the mutation above failing
  in the mutation log.

**2. MAJOR: the rate limit is proven only in phase `hold`, but the test's Catches line is about policy jumps.**
- File: `tests/test_unit_real_safety.py:145-154`.
- Trigger: wrap the rate-limit line in `if phase != "policy":`. Every test still passes (my mutation 2).
- Impact: rule 5/6 says "rate limit in every phase". The policy phase is where a jump from a new chunk or a bad output actually
  happens. A regression there would send a full policy jump to the motors, and the test that says it "catches a policy jump passed
  straight to the motors" stays green.
- Fix: run the same check over every phase in `rs.PHASES` (the hard-limit test already loops over phases), or at least over `"policy"`
  as well as `"hold"`. Record the mutation.

**3. MAJOR: "frozen q0 in the ramp-in" is in the task's safety-critical list, but no test can fail on an unfrozen q0.**
- File: `g1sim/real_safety.py:98-103`, `tests/test_unit_real_safety.py:38-48`.
- Trigger: replace `np.array(q0)` + `setflags(write=False)` with `np.asarray(q0)`, so the plan aliases the caller's buffer. The whole
  file passes (my mutation 3).
- Impact: if B2 reads LowState into a reused buffer (a common pattern), every ramp tick would command the newest measured angle. That
  is the "chase the sagging arm" failure review N1 was about. The code is correct today, but the safety property is unguarded.
- Fix: in the ramp-in test (or a new `test_ramp_in_target_stays_frozen_when_the_measured_buffer_changes`), change the input array after
  calling `ramp_in` (`q0[:] = 0.9`) and assert every planned target still equals the original values. Record the mutation.

**4. MAJOR (clean-code rule: unexplained constants): a bare `1e-9` and unnamed scheduler defaults.**
- `g1sim/real_safety.py:121`: `math.ceil(... / max_step - 1e-9)`. A second-year student cannot tell why it is there. It is a float
  tolerance so that an exact multiple of the step does not add an extra tick. Make it a named constant with a one-line comment (the test
  file already does this with `EPS`).
- `g1sim/real_timing.py:41`: `chunk_size=50, exec_steps=25, action_hz=30` are limits with no source comment. Name them (e.g.
  `CHUNK_SIZE = 50  # server hello chunk_size, rule 4.2.2`, `EXEC_STEPS = 25  # request after 25 actions, rule 14`,
  `ACTION_HZ = 30  # dataset fps, rule 14`) and use them as the defaults.

**5. MINOR: non-finite values can get through two places that should refuse them.**
- `speed_refusal(nan, False)` and `waist_max_refusal(nan, False)` return None (probe 1/1b). That is because `nan > 0.6` is False.
  `real_preflight.argument_problem` uses exactly these functions, and argparse `type=float` accepts `--waist-max nan`. A filter built
  with that value then outputs a NaN waist yaw command (probe 2). `require_finite` checks the filter's input, not its output.
- `fault_targets` returns the measured angles unchecked (probe 4). If LowState ever carried a NaN, the fault ramp-out would hold NaN.
- Trigger: an operator typing `nan` is contrived, and so is a NaN LowState, so this is minor. But the consequence would be a NaN on a
  real motor, which rule 11 exists to prevent.
- Fix: refuse `not math.isfinite(value)` in both refusal functions. In `fault_targets`, fall back to `last_command17` when `measured17`
  is not finite. Optionally run `require_finite(cmd, "filtered command")` as the last line of `SafetyFilter.step`.

**6. MINOR: `envelope_streak` (30 clipped policy ticks -> soft exit, rule 5) has no unit test.**
- Trigger: remove `events.append("envelope_streak")`. Every test still passes (my mutation 4).
- The plan's B1 test list does not ask for this test, and the e2e G2 run may hit it. So this is minor. Still, it is a safety stop that
  sits in this file.
- Fix: one test that steps 30 out-of-envelope policy targets and asserts the event appears on tick 30 and not before.

**7. MINOR: `g1sim/real_timing.py` is a new motion-path file that is not in the plan's `files_modified` list or B1's `<files>`.**
- 86 will load it, so motion gate (b) requires it in the pinned commit. Finnegan's unlock step is "git add <motion files listed by the
  final gate>". If the list is copied from PLAN.md lines 20-22, it misses this file and the gate reports MOTION LOCKED on Monday morning.
  That failure is safe (it fails closed) but costly. The `git status --porcelain -- g1sim scripts` line would flag it, so it is caught,
  but only at 07:30.
- Fix: add `g1sim/real_timing.py` to PLAN.md's B1 file list (main session / planner), and the final gate lists it.

**8. MINOR (plan, real hardware; inference, not measured): `LEAD_LIMIT` 0.25 rad has not been checked against the real arm's tracking
error at the rate cap.**
- With SIM_GAINS (shoulder pitch kp 50, kd 3) and a target velocity of 0, steady tracking at 1.0 rad/s gives roughly kd*v/kp = 0.06 rad
  of lag. A gravity load of about 5-6 Nm on an outstretched arm with a Revo2 hand adds roughly 0.1-0.12 rad of sag. That is about 0.18
  rad, near 0.25. This estimate is mine, from the gains in `real_safety.py:39-40` and an assumed arm mass. It is not measured.
- Impact: false contact faults in M4/M6. A fault at the table is exactly the case R2-1/R15 tries to avoid (no steering away from the
  table).
- Fix (plan): in M3 (in the air), log the largest lead per joint during the policy phase. Before M4 at the table, confirm it stays
  clearly under 0.25. The runbook should name that number.

**Notes (no action needed, checked):**
- I checked whether `stale_reason` treating `camera_age=None` as "camera not used" in `run` mode is a fail-open hole. It is not
  reachable: `real_camera.FrameReader.latest()` and `real_io` readers return `inf` (not None) before the first frame, so a lost camera
  is caught.
- `reset_last` does not force 13/14 to q0. B2 passes the last filtered command (13/14 already q0), so it is not reachable today.
- `tests/test_unit_real_safety.py` is 320 lines, a little over the ~300 guideline. The dict-of-lambdas in
  `test_nan_target_raises_invalid_target` is the least readable test. A plain `pytest.param` per entry point would be easier to explain.

### Overall: NO-GO
The safety logic itself looks correct in every probe. The NO-GO is because three safety tests (latency skip, rate limit in the policy
phase, frozen ramp-in q0) are shown to pass on code broken in exactly the way their Catches lines name, and there are unexplained
constants. Both are the user's stated top priorities. All fixes are test-only or one-line edits (findings 1-4, about 30 min). Findings
5-7 are cheap and worth folding in. Finding 8 goes to the plan/runbook.

## Track B1 - response (fix round 1)

Implementer: Track B1 implementer (Claude Opus 5.5), 2026-10-18 17:59-18:05. Nothing refuted; every finding taken or handed on.

| # | Severity | Result | What changed |
|---|----------|--------|--------------|
| 1 | major | Fixed | Split into `test_scheduler_starts_chunk_at_the_action_matching_its_latency` (asserts target 5.5 right after `add_chunk`) and `test_scheduler_refuses_chunk_that_lies_fully_in_the_past`. Your mutation 1 now fails (mutation 11 in the log). |
| 2 | major | Fixed | `test_rate_limit_caps_arm_and_waist_steps_in_every_phase` loops over `rs.PHASES` and checks the absolute step (in `policy` the envelope pulls some joints negative). Your mutation 2 now fails (mutation 12). |
| 3 | major | Fixed | New `test_ramp_in_target_stays_frozen_when_the_measured_buffer_changes`: `q0[:] = 0.9` after `ramp_in`, every planned target still equals the original. Your mutation 3 now fails (mutation 13). |
| 4 | major | Fixed | `real_safety.FLOAT_TOLERANCE = 1e-9` with a comment replaces the bare number in `move()`. `real_timing.CHUNK_SIZE / EXEC_STEPS / ACTION_HZ` (sources: policy_wire.CHUNK + server hello, rule 14, dataset fps 30) are now the `ChunkScheduler` defaults. |
| 5 | minor | Fixed | `speed_refusal` and `waist_max_refusal` refuse non-finite values first. `fault_targets` uses the last command when the measured angles are not finite. `SafetyFilter.step` now calls `require_finite` on the filtered command (NaN `waist_max` -> InvalidTarget, fault exit). 3 new tests, each with a mutation (15-18). |
| 6 | minor | Fixed | `test_envelope_streak_fires_on_the_30th_clipped_policy_tick`. Your mutation 4 now fails (mutation 14). |
| 7 | minor | Fixed in PLAN.md | `g1sim/real_timing.py` added to `files_modified` and to B1's `<files>`. **The final gate must list it as a motion file** (86 loads it via real_safety). |
| 8 | minor | Handed on (not B1's files) | This changes the Monday M3 procedure and the runbook (planner + E2). Proposed text: "M3 (in the air): log the largest lead per joint during the policy phase; before M4/M6 at the table it must stay below 0.15 rad (well under LEAD_LIMIT 0.25), else stop and report." I did not measure the real tracking error. |

Other changes: test files split for the ~300-line rule. `tests/test_unit_real_safety.py` (plans, exits, refusals, hands, pinned constants; 214 lines), `tests/test_unit_real_safety_filter.py` (the SafetyFilter; 139 lines), `tests/test_unit_real_timing.py` (chunk timing + staleness; 66 lines). The PLAN.md B1 test list, `<files>` and the verify command now name all three. `real_safety.py` is 279 lines.

Runs (full output in runs/):
- `runs/real_b1_unit.log`: B1 verify command (all three files, sim env): 30 passed.
- `runs/real_b1_unit_project_venv.log`: project `.venv`: 29 passed, 1 skipped (table_env needs the sim env).
- `runs/real_b1_py310.log`: import under the robot env, Python 3.10.21: exit 0.
- `runs/real_b1_r1_all_unit.log`: every `-m unit` test in the repo (sim env): 276 passed, 2 skipped. Nothing B2 imports broke.
- `runs/real_b1_mutation.log` (rewritten): control green, then 18 mutations on temp copies, each run against its named test: **all 18 caught**. These are the 10 from the first round (mutation 6 re-anchored to the new `fault_targets` line), your 4 survivors, and 4 for the NaN fixes. The script text is at the end of the log.

## Track B1 - round 2

Reviewer: independent code reviewer (Claude Opus 5.5). I did not write this track or the round-1 review, and I saw none of the
author's reasoning. Date: 2026-10-18, 18:05-18:12.
Scope: the files as of the fix round (mtimes 18:00/18:02, unchanged during this review): `g1sim/real_safety.py` (279 lines),
`g1sim/real_timing.py` (112), `tests/test_unit_real_safety.py` (214), `tests/test_unit_real_safety_filter.py` (139),
`tests/test_unit_real_timing.py` (64). Checked against PLAN.md rev 6.3 sections 0, 3.3, 4.2 and the B1 task. Consumers looked at:
`g1sim/real_run.py` (`tick`, `new_filter`, `_soft_exit`, `_fault_exit`, `measured_now`), `g1sim/real_run_modes.py` (`release`,
`take_over`, `move_to_ready`), `g1sim/real_run_policy.py`, `g1sim/real_preflight.py`, `g1sim/real_send.py`.

### Runs (full output in runs/)
- `runs/real_b1_review_r2_unit.log`: the B1 verify command in the sim env, `-v`: 30 passed in 0.18 s.
- `runs/real_b1_review_r2_unit_project.log`: project `.venv`: 29 passed, 1 skipped with a reason (table_env needs the sim env).
- `runs/real_b1_review_r2_py310.log`: `import g1sim.real_safety, g1sim.real_timing` under the operators' robot env
  (`~/dev/g1-ext/robot-day-test/robot-env`, Python 3.10.21): OK.
- `runs/real_b1_mutation.log` (author's): 18 mutations, each on a fresh temp copy, all caught. I read the script; it makes a new copy per
  mutation, so a stale `.pyc` cannot leak between mutations. Accepted.
- `runs/real_b1_review_r2_mutation.log` (mine): 17 new mutations on temp copies, run with `python -B` so no stale bytecode (my first
  attempt reused a `.pyc` between same-size edits; that attempt is discarded). For every survivor I also ran B2's
  `tests/test_unit_real_run.py` to see whether another track's tests guard it. **11 survive the B1 tests; 8 of those also survive B2's.**
- `runs/real_b1_review_r2_probes.log`: NaN probes (finding 6).

### Round-1 findings re-checked
1-3 (latency skip, rate limit in policy, frozen ramp-in q0): fixed. The new tests fail on the round-1 mutations (author's mutations
11-13), and the asserts are correct (`target()` == 5.5 right after `add_chunk`; `q0[:] = 0.9` after `ramp_in`). 4 (bare numbers): fixed
(`FLOAT_TOLERANCE`, `CHUNK_SIZE/EXEC_STEPS/ACTION_HZ`, each with a source). 5 (NaN): fixed (finite checks in both refusals, NaN fallback
in `fault_targets`, `require_finite` on the filter output). 6 (`envelope_streak`): fixed, and the test checks "tick 30 and not before".
7 (`real_timing.py` in the plan): fixed (PLAN.md lines 22, 692, 712). 8 (lead limit not measured on the real arm): handed on, but **not
yet in PLAN.md or the runbook** (grep: no "largest lead" or "lead per joint"). It stays open as plan finding P1 below.

### Verdict 1: did the code do what the plan says?
Yes for the logic. Every constant, function, class and event of contract 3.3 is present with the contract's meaning. The filter order is
exactly section 3.3. Every probe I ran on the real code behaved correctly. All 25 tests of the B1 list are present, each with a
Proves/Catches docstring and an AAA layout. The code is short and readable, and every constant has a source comment.
**No for "safety checks must be shown to fail on a broken version".** Four safety-critical behaviours in the plan's list (limits, rate
limit, waist rule 6b, the safety numbers themselves) still pass on code broken in exactly that way, in B1's tests and in B2's.

### Verdict 2: is anything in the plan wrong for real hardware?
Nothing blocks. Two plan points: P1 (round-1 finding 8, still open) and P2 (a contract inconsistency that B2 already absorbs).

### Findings

**R2-F1. MAJOR: the rate-limit test only proves the first message of a fresh filter. A filter that limits only the first message, or
allows 5 rad/s, passes every B1 and B2 test.**
- File: `tests/test_unit_real_safety_filter.py:87-96`; code `g1sim/real_safety.py:241-243`.
- Trigger 1 (mutation R2-1): store the unlimited target as `self.last` (`wanted = cmd.copy()` before line 241, `self.last = wanted`).
  The first message is still limited. From the second message on `cmd - self.last` is about 0, so the full jump goes to the motor. B1:
  29 passed. B2 `test_unit_real_run.py`: 20 passed.
- Trigger 2 (mutation R2-15): `ARM_SPEED_MAX = 5.0`. The test asserts `== rs.ARM_SPEED_MAX / rs.SEND_HZ`, so it moves along with the
  constant. B1 and B2 both pass.
- Impact: rule 6 (1.0 rad/s arms, 0.3 rad/s waist, every phase) is the last line between a policy jump and the motors. It is
  unguarded across ticks, and its number is unguarded altogether.
- Fix (test only): make one filter, step it 5 times towards a far target, and assert each consecutive step `np.diff` is <= 0.02 rad for
  the arms and <= 0.006 rad for the waist. Write the plan's numbers as literals (`1.0 / 50`, `0.3 / 50`) with a comment "rule 6", not
  `rs.ARM_SPEED_MAX`. Show R2-1 and R2-15 failing in the mutation log.

**R2-F2. MAJOR: only the upper hard limit is tested. A filter that clips only upwards, or puts the lower margin outside the joint
limit, passes every B1 and B2 test.**
- File: `tests/test_unit_real_safety_filter.py:75-84`; code `g1sim/real_safety.py:219, 237-239`.
- Trigger: mutation R2-5 (`np.clip(cmd, self.lo, self.hi)` -> `np.minimum(cmd, self.hi)`), mutation R2-4 (`self.lo = lo +
  LIMIT_MARGIN` -> `lo - LIMIT_MARGIN`), and mutation R2-16 (`LIMIT_MARGIN = 0.0`; the test asserts `<= hi - rs.LIMIT_MARGIN`, so it
  moves with the constant). All three: B1 29 passed, B2 20 passed.
- Impact: "limits" is first in the task's safety-critical list. Half of every joint range (shoulder pitch down, elbow, wrist) could be
  driven into, or 0.05 rad past, the end stop, and no test would notice.
- Fix (test only): extend `test_hard_limits_clip_in_every_phase` with a target of -2.0 from q0 = -0.44, and assert against the
  literals 0.45 / -0.45 (limits +-0.5 minus the plan's 0.05). Record R2-4, R2-5 and R2-16 failing.

**R2-F3. MAJOR: rule 6b (`--waist-hold` and `--waist-max`) has no test anywhere. Ignoring `--waist-hold` or doubling `--waist-max`
passes every B1 and B2 test.**
- File: `g1sim/real_safety.py:251-255` (`_limit_waist`). No test builds a filter with `waist_hold=True`. No test sends a waist target
  above `waist_max` (the rate-limit test uses 0.5 under waist_max 0.6).
- Trigger: mutation R2-2 (`if self.waist_hold:` -> `if False:`; with the CLI default `--waist-max 0.15`, the waist then turns +-0.15 rad
  under `--waist-hold`), and mutation R2-3 (clip at `+-2 * waist_max`). B1 29 passed, B2 20 passed. (Removing the clip completely is only
  caught by accident, through the NaN test.)
- Impact: rule 16 (b) / R3-2 make the waist setting a column of the Monday table-clearance lookup. `--waist-hold` is checked by 87 as
  "waist limit 0.0". If the hold silently fails, the M4/M6 clearance basis is wrong at exactly the table steps where it matters.
- Fix (test only): `test_waist_hold_keeps_motor_12_at_q0_in_every_phase` (waist target 0.5, q0 yaw 0.02, every phase: motor 12 ==
  0.02 on every tick) and `test_waist_max_clips_waist_yaw_target` (waist_max 0.15, target 0.5, enough ticks for the rate limit: it
  settles at 0.15). Both [mutation].

**R2-F4. MAJOR: tests whose names promise a plan number assert against the module's own constant, so changing the number itself is
not caught.**
- Examples: `test_release_moves_to_factory_pose_at_0_3_rad_s_before_weight_drops` asserts `<= rs.SCRIPTED_SPEED / rs.SEND_HZ`.
  Mutation R2-13 (`SCRIPTED_SPEED = 3.0`, release and soft exits 10x faster) passes all B1 tests; only 2 B2 runner tests catch it.
  `test_lead_fault_fires_after_13_ticks_only_at_full_weight` loops `range(rs.LEAD_TICKS)` and offsets by `rs.LEAD_LIMIT + 0.05`.
  Mutation R2-17 (`LEAD_TICKS = 50`, contact fault after 1 s) and mutation R2-9 (`LEAD_LIMIT = 0.50`) pass B1 and B2. ARM_SPEED_MAX and
  LIMIT_MARGIN are covered in F1/F2. (`HAND_CAP = 1.0` is caught, but only because the test input 0.9 lies between the two values.)
- Impact: the release speed and the contact-fault timing are in the task's safety-critical list. A "small tuning" edit to a constant
  overnight would keep every test green, and the test names would then be false.
- Fix (test only): one `test_safety_numbers_equal_the_plan` that pins, as literals with their plan source, SCRIPTED_SPEED 0.3,
  ARM/WAIST_SPEED_MAX 1.0/0.3, LIMIT_MARGIN 0.05, LEAD_LIMIT 0.25, LEAD_TICKS 13, MAX_WEIGHT_STEP 0.02, RAMP_SECONDS 2.0, HAND_CAP 0.6,
  ENVELOPE_STREAK_TICKS 30, STALE_S, ROBOT_MAX_SPEED_FACTOR 0.5, ROBOT_WAIST_MAX_DEFAULT/CEILING 0.15/0.6. A student can read this as
  "the contract table, checked". Alternatively, write the literals into the existing asserts. Record R2-13 and R2-17 failing.

**R2-F5. MINOR: weaker boundaries that survive.**
- Mutation R2-8 (LowState stale threshold 0.1 -> 0.14 s) survives B1 and B2: `test_stale_reason_thresholds` probes at 0.15. Probe
  at 0.11 (and 1.1 / 0.6 -> 1.01 / 0.51) so the test pins the number its docstring names.
- Mutation R2-6 (envelope lower bound dropped) survives B1 and B2: the envelope test only goes above the top. The hard limits still
  apply, so this is minor. Add a second joint below `ENVELOPE_LO` to `test_envelope_clips_only_in_policy_phase`.
- Mutation R2-7 (`ramp_out` 10x faster, weight step 0.15) survives B1. It is caught by 11 B2 runner tests through the runner's
  MAX_WEIGHT_STEP check, so it is guarded across tracks. A three-line `ramp_out` assert (weight steps <= 0.02, target frozen) next to
  the ramp-in test would keep B1 self-contained.

**R2-F6. MINOR: a NaN measured angle silently switches the contact-fault monitor off.**
- `g1sim/real_safety.py:276-277`: `lead = max(|cmd - NaN|)` is NaN, and `NaN > 0.25` is False, so `lead_ticks` resets every tick.
  Probe: 20 ticks at w = 1 with one NaN measured joint, no `contact_fault`, `lead_ticks` 0 (`runs/real_b1_review_r2_probes.log`).
  `stale_reason(nan, ...)` also returns None.
- Trigger: needs a NaN in LowState `q`, which is contrived. The rest of the module treats NaN as "refuse" (`factory_pose_gap`,
  `fault_targets`, `require_finite`), so this is the one fail-open spot.
- Fix: `if not np.all(np.isfinite(measured)) -> count as over the limit` (or raise InvalidTarget, which is a fault exit). One test.

**R2-F7. MINOR (readability, carried over from the round-1 notes, not addressed):** `test_nan_target_raises_invalid_target`
(`tests/test_unit_real_safety_filter.py:127-139`) still builds a dict of three lambdas, with the hand case built by
`np.where(np.arange(27) == 15, ...)`. It is the one test a second-year student would struggle to explain. Three short
`pytest.param`s, or three plain tests, would read top to bottom.

### Plan findings (real hardware)
**P1. MINOR (open from round 1, finding 8): measure the real arm's lead before the table steps.** The B2 runner already prints the lead
once per second in `hold` ("runbook M2 aborts above 0.10", `real_run_modes.py`). But PLAN.md section 7 / rule 16 and the runbook still
have no step that checks the largest lead per joint during the M3 policy phase in the air against LEAD_LIMIT before M4/M6. Planner or
E2 should add the round-1 proposed text. It is an estimate (kd*v/kp + gravity sag of about 0.18 rad vs 0.25), not measured.

**P2. NOTE (contract, no action for B1):** contract 3.3 `release_plan` moves all 17 values, including 13/14, to the factory record.
Rule 4 says 13/14 stay at q0. B2's `release()` runs every release message through a filter built with q0 = measured, which forces
13/14 back to the measured angles. So the robot gets rule 4, and the release_plan's own 13/14 values are never sent. Fine as built;
worth one line in the contract so a future caller that skips the filter does not drive the waist roll/pitch.

### What I checked and found correct (no action)
- Frozen q0 in `ramp_in`/`ramp_out` (read-only copies). Release: first message at w = 1 at measured, then <= 0.006 rad/tick, ramp-out
  only at the factory pose. Soft exit through the ready pose with no time cap. `fault_targets` falls back on stale or NaN data. The lead
  monitor works only at w == 1.0 exactly; the smoothstep ends at exactly 1.0 and the runner passes the literal 1.0.
- The scheduler latency skip, ChunkTooLate at exactly 50 actions late, interpolation, and the run-out timing.
- `stale_reason` with `camera_age=None` in run mode is not a fail-open hole: the readers return inf before the first frame (round-1
  note, still true).
- The runner's rate-limit reference: `SafetyFilter.last` starts at q0, `reset_last(command)` runs at the policy hand-over, and the
  release filter is rebuilt at q0 = measured, so the first release message is not rate-limited away from measured.
- Python 3.10 import from the operators' robot env works.
- Files are under 300 lines. Every test has a Proves/Catches docstring, a descriptive name and an AAA layout. Constants have source
  comments.

### Overall: NO-GO
The safety logic is correct in every probe, and all round-1 findings are fixed or handed on. The NO-GO is only about the tests, the
user's stated priority: 8 mutations of safety-critical checks survive both B1's and B2's tests. They are rate limit across ticks and its
number, lower hard limit and margin, `--waist-hold` / `--waist-max`, and the lead-fault timing. SCRIPTED_SPEED survives B1. Findings
F1-F4 are test-only (about 30-40 min): one multi-tick rate test, a lower-limit case, two waist tests, and one "safety numbers equal the
plan" pin test, each shown failing on its mutation. F5-F7 are cheap and worth folding in. P1 goes to the planner/E2.

## Track B1 - response (fix round 2)

Implementer: Track B1 implementer (Claude Opus 5.5), 2026-10-18 18:12-18:16. Nothing refuted; every finding taken except P1, which is
handed on (not B1's files). Only one code line changed in `g1sim/real_safety.py` (R2-F6); everything else is tests.

| # | Severity | Result | What changed |
|---|----------|--------|--------------|
| R2-F1 | major | Fixed | `test_rate_limit_caps_arm_and_waist_steps_in_every_phase` now steps one filter 5 times per phase towards the far target and asserts every consecutive step equals the literal `1.0 / 50` (arms) and `0.3 / 50` (waist yaw), module constants not used. R2-1 and R2-15 fail (mutations 19, 20). |
| R2-F2 | major | Fixed | `test_hard_limits_clip_in_every_phase` runs upwards (q0 0.44 -> 2.0) and downwards (q0 -0.44 -> -2.0) in every phase and asserts against the literals `SAFE_LO, SAFE_HI = -0.45, 0.45`. R2-4, R2-5, R2-16 fail (mutations 21-23). |
| R2-F3 | major | Fixed | New `test_waist_hold_keeps_motor_12_at_q0_in_every_phase` (q0 yaw 0.02, target 0.5, waist_max 0.15, 5 messages per phase, every yaw == 0.02) and `test_waist_max_clips_waist_yaw_target` (waist_max 0.15, target 0.5, 40 messages: never above 0.15, settles at 0.15). R2-2 and R2-3 fail (mutations 24, 25). |
| R2-F4 | major | Fixed | New `test_safety_numbers_equal_the_plan`: a `PLAN_NUMBERS` dict copies the PLAN.md 3.3 table as literals with one source comment each (SEND_HZ, RAMP_SECONDS, MAX_WEIGHT_STEP, SCRIPTED_SPEED, ARM/WAIST_SPEED_MAX, LIMIT_MARGIN, ENVELOPE_STREAK_TICKS, LEAD_LIMIT, LEAD_TICKS, HAND_CAP, ROBOT_MAX_SPEED_FACTOR, ROBOT_WAIST_MAX_DEFAULT/CEILING, POLICY_TIMEOUT_S, STALE_S) and compares it with the module in one assert. R2-13, R2-17, R2-9 fail (mutations 26-28). |
| R2-F5 | minor | Fixed | Stale thresholds probed 0.01 s above each value (0.11, 1.01, 0.51, 1.01): R2-8 fails (mutation 29). Envelope test adds left shoulder roll at -0.5 (below ENVELOPE_LO -0.2105): R2-6 fails (mutation 30). New `test_ramp_out_holds_target_and_lowers_weight_in_steps_below_0_02` (literal 0.02, frozen hold pose): R2-7 fails (mutation 31). |
| R2-F6 | minor | Fixed | `_monitor_lead`: a non-finite gap gives `lead = math.inf`, so a NaN angle counts as over LEAD_LIMIT and fires `contact_fault` after 13 ticks (fault exit; `fault_targets` then holds the last command because the measured angle is NaN). New `test_nan_measured_angle_counts_as_a_lead_over_the_limit`; reverting the line fails it (mutation 32). `stale_reason(nan, ...)` left as is: the age comes from a monotonic clock difference and cannot be NaN; noted here, not changed. |
| R2-F7 | minor | Fixed | `test_nan_target_raises_invalid_target` split into three plain tests that read top to bottom: `..._in_safety_filter`, `..._in_chunk_scheduler`, `..._in_hand_targets`, sharing one small `nan_action27()` helper. No lambdas, no `np.where`. |
| P1 | minor (plan) | Handed on (planner / E2) | Not B1's files (PLAN.md section 7 / rule 16 and the runbook Part C). Proposed text unchanged from round 1: "M3 (in the air): log the largest lead per joint during the policy phase; before M4/M6 at the table it must stay below 0.15 rad (well under LEAD_LIMIT 0.25), else stop and report." Still not measured. |
| P2 | note | No action | Agreed; contract wording is the planner's. |

PLAN.md B1 task block: the test list names the new and split tests with their [mutation] marks (nothing else in PLAN.md touched).

Runs (full output in runs/):
- `runs/real_b1_unit.log`: B1 verify command (sim env): 35 passed.
- `runs/real_b1_unit_project_venv.log`: project `.venv`: 34 passed, 1 skipped (table_env needs the sim env).
- `runs/real_b1_py310.log`: import under the robot env, Python 3.10.21: OK.
- `runs/real_b1_r2_all_unit.log`: every `-m unit` test in the repo (sim env): 333 passed, 2 skipped. B2's tests still pass with the
  NaN-lead change.
- `runs/real_b1_mutation.log` (rewritten): control green, then 32 mutations, each on a fresh temp copy run with `python -B` against its
  named test: **all 32 caught**. 1-18 are the earlier ones, 19-31 the reviewer's round-2 survivors (R2-1 to R2-9, R2-13, R2-15 to
  R2-17), 32 the NaN-lead fix. The script text is at the end of the log.

File sizes: `real_safety.py` 281, `test_unit_real_safety.py` 255, `test_unit_real_safety_filter.py` 214, `test_unit_real_timing.py` 64.

## Track B1 - round 3

Reviewer: independent code reviewer (Claude Opus 5.5). I did not write this track or the round-1/round-2 reviews, and I saw none of
the author's reasoning. Date: 2026-10-18, 18:17-18:23.
Scope: the files of fix round 2 (mtimes 18:00-18:14; they did not change during this review): `g1sim/real_safety.py` (281 lines),
`g1sim/real_timing.py` (112), `tests/test_unit_real_safety.py` (255), `tests/test_unit_real_safety_filter.py` (214),
`tests/test_unit_real_timing.py` (64). Checked against PLAN.md section 3.3, rules 4.2 and the B1 task block. Consumers read: `g1sim/real_run.py`
(`tick`, `measured_now`, `_soft_exit`, `_fault_exit`), `g1sim/real_run_modes.py`, `g1sim/real_run_policy.py`, `g1sim/real_send.py`
(`fill_hand_cmd`), `g1sim/real_io.py` (`latest()`).

### Runs (full output in runs/)
- `runs/real_b1_review_r3_unit.log`: the B1 verify command in the sim env (`~/dev/g1-ext/.venv`), `-v`: **35 passed** in 0.25 s.
- `runs/real_b1_review_r3_unit_project.log`: project `.venv`: 34 passed, 1 skipped with a reason (table_env needs the sim env).
- `runs/real_b1_review_r3_py310.log`: `import g1sim.real_safety, g1sim.real_timing` under the operators' robot env (Python 3.10.21): OK.
- `runs/real_b1_mutation.log` (the author's): 32 mutations, each on a fresh temp copy with `python -B`. All caught. I read the list:
  it covers every round-1 and round-2 survivor. Accepted.
- `runs/real_b1_review_r3_mutation.log` (mine): 19 new mutations on fresh temp copies (repo untouched), run with `python -B` against all three
  B1 test files. Every survivor was then also run against B2's `tests/test_unit_real_run.py` and `tests/test_unit_motion_gate.py`. Control
  green. **1 caught by B1, 18 survive B1. 2 of those are caught by B2 (N2, N5). 16 survive both.** I sorted these by whether the broken
  path can be reached from 86 today (below). Most cannot, or only in the safe direction. Two can, and they are the majors.
- `runs/real_b1_review_r3_probes.log`: speed factor probe, negative `--waist-max` probe, and the list of 3.3 constants that the pin
  test leaves out. (The first try at probe 3 failed on an import path. The retry is in the same file.)

### Round-2 findings re-checked
R2-F1 to R2-F4 are fixed, and the new tests read well. The 5-message rate test uses the literals `1.0 / 50` and `0.3 / 50`. The limit test
runs both directions against the literals ±0.45. The two waist tests are there. `PLAN_NUMBERS` uses literals. The author's mutations 19-28
fail as they should. R2-F5 (0.01 s probes, envelope bottom, ramp_out test), R2-F6 (NaN lead counts as `inf`, with a test) and R2-F7 (three
plain NaN tests) are fixed too. P1 (measure the real arm's lead in M3) is **still not in PLAN.md or the runbook**: grep finds only the
M2 "lead above 0.10" abort at runbook line 278. P2 is unchanged (a note).

### Verdict 1: did the code do what the plan says?
Yes for the logic. Every name in contract 3.3 is present with the contract's meaning. The filter order matches 3.3, and all 35 tests in the
B1 list are present, each with a Proves/Catches docstring and an AAA layout. The code is short, and a second-year student can explain
every function. Every constant has a source comment, and every refusal message tells the operator what to type instead.
**Not fully for "every safety check shown to fail on a broken version".** Two safety behaviours that 86 reaches on Monday still pass on a
broken version, in B1 and in B2: the runtime effect of the `--speed` factor (motion gate (e)), and the hand speed cap and three other
constants that the "every safety number" test leaves out.

### Verdict 2: is anything in the plan wrong for real hardware?
Nothing blocks. P1 is still open (see above). There is one new contract note, P3.

### Findings

**R3-F1. MAJOR: the `--speed` factor (motion gate (e): at most 0.5 on the robot) is tested at the refusal but nowhere at the point
where it takes effect. A scheduler that ignores it passes every B1 and B2 test.**
- File: `g1sim/real_timing.py:54` (`advance`), and `:93` (`seconds_since_ran_out`). The tests: `tests/test_unit_real_timing.py` uses only
  `speed=1.0`.
- Trigger: mutation N1 (`self.position += dt * self.action_hz`, speed dropped). B1: 34 passed, 1 skipped. B2: 29 passed (its runs use
  `speed=0.5` but never check the playback rate). Reachable: `86 replay/run --speed 0.25|0.5` builds
  `ChunkScheduler(runner.args.speed)` (`real_run_policy.py:70`). Probe 1: today 1 s of wall time at speed 0.5 plays action 15. With
  the mutation it plays action 30.
- Impact: the operator types `--speed 0.25` for M3 "in the air, slow", and the policy plays at full demo speed. The 1.0 rad/s rate limit still
  applies, but the reason for gate (e) ("slow is what makes Ctrl+C in time") silently stops holding. Mutation N15 (the run-out timer ignores
  speed, so `policy_stale` fires at a different time) also survives. It is the same gap.
- Fix (test only, about 5 min): `test_scheduler_plays_at_speed_times_30_actions_per_second`: speed 0.5, the counting chunk, 50 ×
  `advance(1/50)`, then assert `target() == 15.0` (the literal: 30 Hz × 0.5 × 1 s). Optionally add `seconds_since_ran_out()` after the run-out
  at speed 0.5. Show N1 (and N15) failing in the mutation log.

**R3-F2. MAJOR: `test_safety_numbers_equal_the_plan` says "every safety number in real_safety equals the PLAN.md 3.3 contract table",
but it leaves out 4 numbers of that table. One of them is the hand speed cap, and changing it passes every B1 and B2 test.**
- File: `tests/test_unit_real_safety.py:218-243`. Probe 3: the 3.3 scalars missing from `PLAN_NUMBERS` are `ENVELOPE_MARGIN`, `HAND_SPEED`,
  `HANDS_OPEN_SPEED` and `EXIT_PATH_MARGIN_S`.
- Trigger: mutation N4 (`HAND_SPEED = 1.0`) survives B1 and B2. `real_send.fill_hand_cmd` uses `HAND_SPEED` as the **cap**
  ("refuses speeds above HAND_SPEED (ruling R6)"), and the policy phase sends hand targets at that speed (`real_run_policy.py:115`). So
  a quiet edit doubles the BrainCo closing speed on Monday, and the cap moves along with it. Mutation N6 (`EXIT_PATH_MARGIN_S = 100.0`) also
  survives both. It disables the stuck-exit watchdog (`real_run.py:255`) for 100 s. N5 (`HANDS_OPEN_SPEED`) is caught only by B2.
  `ENVELOPE_MARGIN` is pinned indirectly by the stats test, but only in the sim env with the dataset present.
- Impact: this is the same class as R2-F4, which round 2 rated major: a docstring that promises more than the test checks, plus a
  safety number that can be changed overnight with every test green.
- Fix (test only, about 3 min): add the four entries as literals with their source (`"HAND_SPEED": 0.5  # ruling R6`, `"HANDS_OPEN_SPEED":
  0.3  # ruling R6`, `"ENVELOPE_MARGIN": 0.10  # PLAN.md 2.8`, `"EXIT_PATH_MARGIN_S": 1.0  # m6-9`). The timing values `CHUNK_SIZE 50`,
  `EXEC_STEPS 25` and `ACTION_HZ 30` (rule 14) also belong there, because N2 (requesting only after the whole chunk) is caught only by B2.
  Show N4 and N6 failing.

**R3-F3. MINOR: the lead boundary is checked 0.05 rad above the limit, so a 0.04 rad looser comparison survives.**
- `tests/test_unit_real_safety_filter.py:157` uses `measured[ARM] -= rs.LEAD_LIMIT + 0.05`. Mutation N8 (`lead > LEAD_LIMIT + 0.04`, constant
  unchanged) survives B1 and B2. The contact fault would then fire at 0.29 instead of 0.25 rad. The fix is an offset of 0.26, the same
  "0.01 above" style as the stale test.

**R3-F4. MINOR: "30 clipped policy ticks *in a row*" is not proven to need the ticks in a row.**
- Mutation N7 (no reset on an unclipped tick) survives B1 and B2. That is the safe direction (the exit comes sooner, never later), so it is
  minor. Add one unclipped tick at tick 15 to `test_envelope_streak_fires_on_the_30th_clipped_policy_tick` and assert no streak at tick 30.

**R3-F5. MINOR: two refusals have no test.**
- Mutation N10 (a negative `--waist-max` is no longer refused) and mutation N11 (`--speed 0` or below is no longer refused) survive B1 and B2.
  Probe 2: a filter built with `waist_max = -0.15` drives the waist yaw to -0.15 rad from a target of 0. `np.clip` with lo > hi returns hi.
  argparse accepts `--waist-max -0.15`. The code refuses it correctly today. Fix: two asserts in
  `test_speed_above_0_5_and_waist_above_0_6_refused_off_loopback` (or in the NaN refusal test).

**R3-F6. MINOR: camera staleness is proven in `run` only, not in `shadow`.**
- Mutation N16 survives B1 and B2. In `shadow` the arm holds the ready pose, so the impact is small. Fix: one more assert in
  `test_stale_reason_thresholds` (`camera_age 1.01, mode "shadow"` -> `"camera_stale"`).

**R3-F7. MINOR (readability):** `tests/test_unit_real_safety.py:244` has one blank line before `def
test_envelope_and_gains_pinned_to_stats_and_table_env` (PEP 8 wants two, as everywhere else in the file). The `make_filter` and
`ready17` helpers have no one-line docstring, and `ready17` is defined twice, once in each test file. These are cosmetic.

**Mutations that survive, but the broken path cannot be reached from 86 today (no action, recorded so nobody re-opens them):**
- N3 (LowState `None` no longer stale): `real_io` readers return `(None, inf)` before the first message (`real_io.py:195`), and
  `measured_now` checks `msg is None` itself.
- N9 (`soft_exit_plan` no longer holds 13/14): every soft-exit tick goes through the filter (`filtered=True`), and the filter forces 13/14
  to q0 (that is tested).
- N13 (`reset_last` is a no-op): the policy phase uses the same filter as `move_to_ready`, so `last` already equals `r.command`.
  This is worth a B2 note if B2 ever builds a new filter for the policy phase.
- N17 (`move` ignores its `speed` argument): every caller uses the default.
- N18 (the lead is measured on the target instead of the sent command): only false faults would follow (the safe direction).
- N19 (`ramp_out` aliases its input): callers pass `self.command` (rebound each tick, never written in place) or `fault_targets(...)` (a
  fresh array).
- N14 (smoothstep range check removed): this is a caller-bug guard, and every caller passes 0/1/current weight.

### Plan findings (real hardware)
**P1. MINOR (open since round 1): measure the real arm's lead before the table steps.** It is still not in PLAN.md section 7 / rule 16 or
the runbook Part C. The planner (rev 7 is running now) or E2 should add the proposed M3 line: "log the largest lead per joint in the policy
phase; before M4/M6 at the table it must stay below 0.15 rad, else stop and report". The 0.18 rad estimate against 0.25 is still not measured.

**P2. NOTE (carried):** `release_plan` moves 13/14 to the factory record. Only B2's filter keeps them at q0.

**P3. NOTE (new, contract wording):** the fault ramp-out sends `fault_targets(...)` with `filtered=False` (`real_run.py:300-301`). So
for a contact fault, motors 13/14 are commanded to their *measured* angles, not q0. Rule 4 says "13/14 held at q0 ... in every message while
weight > 0". Physically this is harmless, because holding the measured angle moves nothing, and the contract text of `fault_targets`
("measured") is what B1 built. One sentence in rule 4 ("except the fault ramp-out, which holds the measured angles") removes the
contradiction.

### Overall: NO-GO
The safety logic is correct in every probe, all round-2 findings are fixed, and the code is clean enough for a student to explain. The
NO-GO is about tests only, the user's stated top priority. Two safety settings that 86 uses on Monday can be broken with every B1 and B2 test
still green. R3-F1 is the `--speed` factor in the scheduler (gate (e)). R3-F2 is the hand speed cap and the exit watchdog margin, which the
"every safety number" test leaves out. Both fixes are test-only, about 10 minutes together, and each must be shown failing on N1/N4/N6.
R3-F3 to F7 are cheap and worth folding in. P1 stays with the planner/E2.

## Track B1 - response (round 3)

Implementer: Track B1 implementer (Claude Opus 5.5), 2026-10-18 18:23-18:26. Nothing refuted; every blocker, major and minor taken.
Tests only: `g1sim/real_safety.py` and `g1sim/real_timing.py` are unchanged.

| # | Severity | Result | What changed |
|---|----------|--------|--------------|
| R3-F1 | major | Fixed | `tests/test_unit_real_timing.py`: new `test_scheduler_plays_at_speed_times_30_actions_per_second` (speed 0.5, counting chunk, 50 x `advance(1/50)`, `target() == 15.0` against the literal `HALF_SPEED_ACTIONS_PER_S = 30 * 0.5`) and `test_run_out_timer_counts_wall_seconds_at_speed_0_5` (play to action 49, advance 1.0 s, `seconds_since_ran_out() == 1.0`). N1 and N15 fail (mutations 33, 34). |
| R3-F2 | major | Fixed | `PLAN_NUMBERS` gains `HAND_SPEED 0.5`, `HANDS_OPEN_SPEED 0.3` (ruling R6), `ENVELOPE_MARGIN 0.10` (PLAN.md 2.8), `EXIT_PATH_MARGIN_S 1.0` (m6-9) as literals; the docstring now says where the rest is pinned. The timing numbers live in `real_timing` (not re-exported by `real_safety`), so they are pinned next to their module: new `test_timing_numbers_equal_the_plan` with `TIMING_PLAN_NUMBERS` = `CHUNK_SIZE 50`, `EXEC_STEPS 25`, `ACTION_HZ 30` (rule 14). N4, N5, N6, ENVELOPE_MARGIN, N2, CHUNK_SIZE and ACTION_HZ fail (mutations 35-41). |
| R3-F3 | minor | Fixed | Lead test offset is the literal `LEAD_JUST_OVER = 0.26` (0.01 above 0.25). N8 fails (mutation 42). |
| R3-F4 | minor | Fixed | The streak test adds a second filter fed 14 clipped, 1 unclipped, 30 clipped ticks: no streak before the last tick, streak on it. The reviewer's shape (gap at tick 15, check at tick 30) does not catch N7 (only 29 clips in total), so the run continues to the 30th clip after the gap; without the reset the streak fires at tick 31. N7 fails (mutation 43). |
| R3-F5 | minor | Fixed | New `test_zero_or_negative_speed_and_negative_waist_max_refused` (speed 0.0 and -0.5, waist-max -0.15 on and off loopback refused, waist-max 0.0 allowed). N10 and N11 fail (mutations 44, 45). |
| R3-F6 | minor | Fixed | `test_stale_reason_thresholds` adds `camera_age 1.01, mode "shadow"` -> `"camera_stale"`. N16 fails (mutation 46). |
| R3-F7 | minor | Fixed (partly) | Two blank lines before `test_envelope_and_gains_pinned_to_stats_and_table_env`; one-line docstrings on `make_filter`, both `ready17` and `largest_joint_step`. The duplicate `ready17` stays: sharing it needs a helper file outside B1's five files (or a test-to-test import). |
| P1 | minor (plan) | Planner / E2 | Not B1's files. PLAN.md rev 7 now carries an `M3_LEAD_MAX` = 0.15 rad line citing B1 review P1 (PLAN.md section 4); `grep M3_LEAD_MAX docs/ROBOT_DAY_RUNBOOK.md` finds nothing yet - E2's to add. |
| P2, P3 | note | No action | Contract wording, the planner's. |

Not changed (outside the five files): the PLAN.md B1 task block test list does not yet name the four new tests
(`test_scheduler_plays_at_speed_times_30_actions_per_second`, `test_run_out_timer_counts_wall_seconds_at_speed_0_5`,
`test_timing_numbers_equal_the_plan`, `test_zero_or_negative_speed_and_negative_waist_max_refused`, all [mutation]) - the planner should add them.

Runs (full output in runs/):
- `runs/real_b1_unit.log`: B1 verify command (sim env): 39 passed.
- `runs/real_b1_unit_project_venv.log`: project `.venv`: 38 passed, 1 skipped (table_env needs the sim env).
- `runs/real_b1_mutation.log` (rewritten): control green, then 46 mutations, each on a fresh temp copy run with `python -B` against its
  named test: **all 46 caught**. 1-32 are the earlier ones, 33-46 the round-3 set (N1, N15, N4, N5, N6, ENVELOPE_MARGIN, N2, CHUNK_SIZE,
  ACTION_HZ, N8, N7, N10, N11, N16). A first run had N7 not caught (the 29-clip gap test described under R3-F4); the test was fixed and the whole run repeated.
  The script text is at the end of the log.

File sizes: `real_safety.py` 281, `real_timing.py` 112, `test_unit_real_safety.py` 274, `test_unit_real_safety_filter.py` 223,
`test_unit_real_timing.py` 109.

## Track B1 - round 4

Reviewer: independent code reviewer (Claude Opus 5.5). I did not write this track or any earlier review or response, and I saw none of
the author's reasoning. Date: 2026-10-18, 18:28-18:36.
Scope: `g1sim/real_safety.py` (281 lines, mtime 18:13), `g1sim/real_timing.py` (112, 18:00), `tests/test_unit_real_safety.py` (274, 18:25),
`tests/test_unit_real_safety_filter.py` (223, 18:26), `tests/test_unit_real_timing.py` (109, 18:24). Checked against PLAN.md 3.3, rules
4.2 and the B1 task block. Consumers read: `g1sim/real_run.py` (`tick`, `_soft_exit`, `_fault_exit`, `weight_step_problem`),
`g1sim/real_run_policy.py`, `g1sim/real_run_modes.py`, `g1sim/real_preflight.py:133`, `g1sim/real_io.py` (`state27`, `sdk_q`).

### Runs (full output in runs/)
- `runs/real_b1_review_r4_unit.log`: B1 verify command in the sim env, `-v`: **39 passed** in 0.19 s.
- `runs/real_b1_review_r4_unit_project.log`: project `.venv`: 38 passed, 1 skipped with a reason (table_env needs the sim env).
- `runs/real_b1_review_r4_py310.log`: `import g1sim.real_safety, g1sim.real_timing` under the robot env (Python 3.10.21): OK.
- `runs/real_b1_mutation.log` (the author's, 18:26): control green, 46 mutations on fresh temp copies with `python -B`, each against its
  named test, all caught. I read the script and the round-3 entries 33-46. Accepted.
- `runs/real_b1_review_r4_mutation.log` (mine): 22 new mutations in two batches, each on a fresh temp copy (g1sim, scripts, tests,
  pytest.ini; repo untouched). Every one ran against all three B1 test files **and** B2's `test_unit_real_run.py` + `test_unit_motion_gate.py`.
  Controls are green. Results: **11 caught by B1, 3 caught by B2 only (M19-M21, request logic), and 8 survive both.** The copy run shows 1 B1
  test skipped (the stats/table_env pin; no mutation touches it). B2's test count changed from 23 to 25 between mutations: B2's tests were
  being edited while I ran. The survivors were re-run after that, so the result holds for the 25-test version.
- `runs/real_b1_review_r4_probes.log`: the current code is correct on every survivor's path. The waist clip is symmetric (min -0.15), the
  mapping is right, the gap is a max, replay gets policy_stale, run gets hand_state_stale, and ramp_out starts at the given weight.
- `runs/real_b1_review_r4_ruff.log`: **not run**. No ruff or flake8 is installed in either env. Cosmetic only; it does not move the gate.

### Round-3 findings re-checked
All fixed and read in the code. R3-F1: `test_scheduler_plays_at_speed_times_30_actions_per_second` (literal 30 x 0.5) and
`test_run_out_timer_counts_wall_seconds_at_speed_0_5`. R3-F2: `PLAN_NUMBERS` has HAND_SPEED, HANDS_OPEN_SPEED, ENVELOPE_MARGIN and
EXIT_PATH_MARGIN_S as literals, and `TIMING_PLAN_NUMBERS` pins CHUNK_SIZE/EXEC_STEPS/ACTION_HZ. The docstring no longer over-promises.
R3-F3: `LEAD_JUST_OVER = 0.26`. R3-F4: the gap scenario (14 + 1 + 30), and the author is right that my round-3 shape could not catch
N7. R3-F5: `test_zero_or_negative_speed_and_negative_waist_max_refused`. R3-F6: the shadow camera probe. R3-F7: blank lines and helper
docstrings done. The duplicate `ready17` stays (accepted, outside the 5 files). P1: PLAN.md rev 7 carries `M3_LEAD_MAX` 0.15 (rule 16 (c)).
`grep M3_LEAD_MAX\|MAX LEAD docs/ROBOT_DAY_RUNBOOK.md` still finds nothing (E2's). The PLAN.md B1 task block still does not name the 4
round-3 tests (planner's).

### Verdict 1: did the code do what the plan says?
Yes for the logic: every 3.3 name has the contract's meaning, the filter order matches, and every probe is correct. **Not fully for "every
safety check shown to fail on a broken version"**, which this gate has held B1 to since round 2. Three contract behaviours that 86 reaches on
Monday have no direct test. B1 and B2 both stay green when they are broken: the waist yaw limit in the negative direction, the
27-D -> motor mapping, and the factory-pose gap as a maximum.

### Verdict 2: is anything in the plan wrong for real hardware?
Nothing new. P1 is now in the plan; the runbook line is E2's. P2/P3 are carried as notes.

### Findings

**R4-F1. MAJOR: `--waist-max` is only proven in the positive direction. A one-sided clip lets the waist turn to the envelope bottom
(-0.81 rad) under `--waist-max 0.15`, and every B1 and B2 test passes.**
- File: `g1sim/real_safety.py:255` (`_limit_waist`). Test: `test_waist_max_clips_waist_yaw_target` (`test_unit_real_safety_filter.py:148`) uses
  only target +0.5.
- Trigger: mutation R4-M9b `np.clip(yaw, -wm, wm)` -> `np.minimum(yaw, wm)`: B1 38 passed/1 skipped, B2 25 passed. (The `min()` variant
  R4-M9 was caught only by accident: Python's `min(x, nan)` hides the NaN, so the NaN-waist test fails. That catch is not real.)
- Reachable: `86 replay/run` with the default `--waist-max 0.15`. The demos use waist yaw down to -0.71 rad (`ENVELOPE_LO[26]` = -0.8137), so the
  policy sends negative yaw, and in the `policy` phase only the envelope (-0.81) and the joint limit would stop it.
- Impact: rule 16 (b) reads the table-clearance column for the waist limit reached in M3. A torso turning to -0.8 rad while the lookup assumed
  0.15 voids that clearance at the table.
- Fix (test only, about 2 min): in `test_waist_max_clips_waist_yaw_target` also drive target -0.5 for 40 messages and assert
  `min(yaws) >= -0.15 - EPS` and `yaws[-1] == approx(-0.15)`. Show R4-M9b failing.

**R4-F2. MAJOR: the 27-D action -> rt/arm_sdk motor mapping (`sdk_targets`, contract "12 <- action[26]; 15-28 <- action[0:14]") has no
test. Swapping the two arms, or reading the waist from a hand slot, passes every B1 and B2 test.**
- File: `g1sim/real_safety.py:60-67`. Every test builds target and measured with `sdk_targets` itself (`ready17`), so a wrong mapping cancels
  out. `_envelope17` also maps the envelope through `sdk_targets`, so the envelope moves along with the bug and never clips it.
- Trigger: R4-M1 (`target[ARMS] = np.r_[action27[7:14], action27[0:7]]`) and R4-M2 (`action27[25]` for the waist) survive B1 and B2.
- Reachable: every policy message (`real_run_policy.py:116`, `:190`) and the ready pose (`real_run_modes.py:74`). Probe 1: on a swap the left
  arm gets the right arm's mirrored values (shoulder roll, shoulder yaw, elbow-roll signs flip). The ready pose is nearly symmetric, so the
  error is small there, but the grasp poses are not. R4-M2 would turn the waist by a hand closing value (0-0.6).
- Impact: a policy driving the wrong arm into the other's workspace. The hard limits still hold, but the envelope does not, and 87's
  clearance does not either.
- Fix (test only, about 3 min): `test_sdk_targets_maps_action_slots_to_motors`: `sdk_targets(np.arange(27.0), np.arange(100.0, 117.0))`
  equals the literal `[26, 101, 102, 0, 1, ..., 13]` (Proves: motor 12 gets action 26, 13/14 get q0, 15-28 get actions 0-13 in order).
  Show R4-M1 and R4-M2 failing.

**R4-F3. MAJOR: `factory_pose_gap` is only tested with a NaN. Returning the smallest gap instead of the largest passes every B1 and B2
test, which switches off the preflight refusal "arms not at the factory pose" (rule 2, R2-2).**
- File: `g1sim/real_safety.py:165-169`, consumed at `g1sim/real_preflight.py:133`. Test: only `test_factory_pose_gap_treats_nan_as_infinite`.
- Trigger: R4-M3 (`np.max` -> `np.min`) survives. B2's `test_start_refused_when_arms_off_factory_pose` evidently moves every joint, so it
  cannot tell max from min.
- Impact: the refusal exists for the case "our last run may still hold the arms" (3.4, R2-2). There, a new run's ramp-in starts at weight 0,
  and Unitree's controller swings the held arm back to factory fast. With `min`, one joint 0.5 rad off (the others at factory) gives
  gap 0.0, and the start is allowed. Probe 4: today one joint 0.3 off gives 0.3 (correct).
- Fix (test only, about 1 min): `test_factory_pose_gap_is_the_largest_joint_difference`: one joint 0.3 off, the others equal -> `== 0.3`.
  Show R4-M3 failing.

**R4-F4. MINOR: two staleness rules are tested in only one mode.**
- R4-M4 (`policy_stale` only in shadow/run) and R4-M11 (`hand_state_stale` only in shadow) survive B1 and B2. `test_stale_reason_thresholds`
  probes `ran_out_s` only in `run` and the hand only in `shadow`. Impact is limited: in replay the 2 s socket timeout still ends a dead
  server, and a stale hand only feeds the policy, not the arm. Fix: add `{"ran_out_s": 1.01, "mode": "replay"} -> "policy_stale"` and
  `{"hand_age": 0.51} (mode run) -> "hand_state_stale"`.

**R4-F5. MINOR: `ramp_out` is only tested from weight 1.0.**
- R4-M10 (`smoothstep_weights(1.0, 0.0)`, which ignores `weight_now`) survives. The path is reachable: an exit or fault during ramp-in, where
  `real_run.py:279` keeps the partial weight. B2's `weight_step_problem` guard (`real_run.py:179`) refuses the 0.4 -> 1.0 jump before it is
  sent, so nothing moves, but the exit then ends in `weight_step` faults instead of a ramp-out, and the arms stay held. Fix: one assert,
  `rs.ramp_out(q, 0.4)` starts at 0.4, ends at 0.0, steps <= 0.02.

**R4-F6. MINOR: the scheduler's guard logic is only proven through B2.**
- R4-M5 (`needs_request` ignores a pending request) survives B1 and B2: every tick after 25 actions would queue another infer call.
  R4-M15 (chunk shape check removed) survives both. R4-M19-M21 (exec_steps, pending flag) are caught only by B2's
  `test_replay_requests_advance_tick_with_scheduler`. Fix: one B1 test, `test_scheduler_requests_after_25_actions_once_until_the_answer`
  (no request at 24.9, request at 25, `needs_request()` False after `mark_requested()` until `add_chunk`), plus a `pytest.raises(ValueError)`
  for a (50, 26) chunk.

**Mutations that survive but do no harm (no action, recorded):** R4-M8 (lead count not reset at w < 1) and R4-M16 (lead count never reset
by a small lead) only fault earlier, which is the safe direction. Also, w = 1 -> w < 1 -> w = 1 never happens in one filter's life.

**Clean code lens.** A second-year student can explain every function. The only "clever" line is `_envelope17` (mapping the bounds
through `sdk_targets` with ±inf as q0), and its docstring says why. Note R4-F2: that trick is also why a mapping bug stays invisible.
Every test reads Arrange-Act-Assert, with a Proves/Catches docstring and literal plan numbers where a constant would hide a change. Nothing
to fix beyond the carried `ready17` duplicate.

### Plan findings
- P1 (carried, now E2's): the runbook needs the `M3_LEAD_MAX` 0.15 / `MAX LEAD` line from rule 16 (c). It is not in
  `docs/ROBOT_DAY_RUNBOOK.md` yet.
- Planner: the B1 task block test list should name the 4 round-3 tests (the author noted this) and the round-4 ones, once they are added.

### Overall: NO-GO
The safety logic is correct in every probe, all round-3 findings are fixed, and the code is clean. The NO-GO uses the bar this gate has
applied since round 2 ("every safety check shown to fail on a broken version"). Three behaviours that 86 uses on Monday can be broken with
every B1 and B2 test green: the negative waist limit under `--waist-max` (R4-F1), the action -> motor mapping (R4-F2), and the factory-gap
refusal (R4-F3). All three fixes are test-only, about 6 minutes together, and each must be shown failing on R4-M9b, R4-M1/M2 and R4-M3 in
the mutation log. R4-F4 to F6 are cheap (about 5 min) and worth folding in. With F1-F3 done and shown failing, I expect GO with no further
code change.

## Track B1 - response (round 4)

Implementer: Track B1 implementer (Claude Opus 5.5), 2026-10-18 18:34-18:37. Nothing refuted; every major and minor taken.
Tests only: `g1sim/real_safety.py` (mtime 18:13) and `g1sim/real_timing.py` (18:00) are unchanged.

| # | Severity | Result | What changed |
|---|----------|--------|--------------|
| R4-F1 | major | Fixed | `test_waist_max_clips_waist_yaw_target` adds a second filter driven to target -0.5 in the `policy` phase (inside the envelope bottom -0.81, so only `--waist-max` can stop it) for 40 messages: `min >= -0.15 - EPS`, last `== approx(-0.15)`. R4-M9b (`np.minimum`) fails at -0.24 (mutation 47). |
| R4-F2 | major | Fixed | New `test_sdk_targets_maps_action_slots_to_motors`: `sdk_targets(arange(27), arange(100, 117))` equals the literal `[26, 101, 102, 0..13]`. R4-M1 (arms swapped), R4-M2 (waist from action 25) and a waist roll/pitch-from-action variant fail (mutations 48-50). |
| R4-F3 | major | Fixed | New `test_factory_pose_gap_is_the_largest_joint_difference`: one joint +0.3, another -0.1, rest equal -> `approx(0.3)`. R4-M3 (`np.min`, gives 0.0) and a `np.sum` variant (gives 0.4) fail (mutations 51, 52). |
| R4-F4 | minor | Fixed | `test_stale_reason_thresholds` adds `hand_age 0.51` in `run` -> `hand_state_stale` and `ran_out_s 1.01` in `replay` -> `policy_stale`; docstring updated. R4-M11 and R4-M4 fail (mutations 53, 54). |
| R4-F5 | minor | Fixed | New `test_ramp_out_from_weight_0_4_starts_at_0_4`: first weight 0.4, last 0.0, steps <= 0.02. R4-M10 fails (mutation 55). |
| R4-F6 | minor | Fixed | New `test_scheduler_requests_after_25_actions_once_until_the_answer` (False at 24.9, True at exactly 25.0, False while pending, True again 25 actions into the answer chunk) and `test_scheduler_refuses_chunk_of_the_wrong_shape` (`(50, 26)` -> `ValueError` matching "shape"). R4-M5, R4-M19 (`>`), R4-M20 (+1), request 0.5 early, R4-M21 (pending flag not cleared) and R4-M15 (shape check removed) fail (mutations 56-61). R4-M21 is caught by the first chunk already (the answer to the very first request never clears the flag). |
| R4-M8, R4-M16 | note | No action | Agreed with the reviewer: they only fault earlier (safe direction). |
| P1 | plan | E2 | Carried: the runbook `M3_LEAD_MAX` / `MAX LEAD` line is E2's. |

Planner (outside the five files): the PLAN.md B1 task block test list should also name the 4 round-3 tests (listed in the round-3
response) and the 5 new round-4 tests: `test_sdk_targets_maps_action_slots_to_motors`, `test_factory_pose_gap_is_the_largest_joint_difference`,
`test_ramp_out_from_weight_0_4_starts_at_0_4`, `test_scheduler_requests_after_25_actions_once_until_the_answer`,
`test_scheduler_refuses_chunk_of_the_wrong_shape` (all [mutation]).

The task text also named the round-3 items "--speed 0.5 halves the playback rate" and "plan numbers cover HAND_SPEED, HANDS_OPEN_SPEED,
ENVELOPE_MARGIN and the exit watchdog margin": both were done in round 3 (R3-F1, R3-F2) and stay shown failing as mutations 33-41 in
the rerun below.

Runs (full output in runs/):
- `runs/real_b1_unit.log`: B1 verify command (sim env): **44 passed**.
- `runs/real_b1_unit_project_venv.log`: project `.venv`: 43 passed, 1 skipped (table_env needs the sim env).
- `runs/real_b1_mutation.log` (rewritten, round-3 copy kept in the implementer's scratchpad): control green, then **61 mutations**, each on
  a fresh temp copy run with `python -B` against its named test: **all 61 caught**. 1-46 are rounds 1-3 (rerun unchanged), 47-61 the round-4
  set listed above. Script text at the end of the log.
- ruff/flake8: not run (not installed in either env, as the reviewer found).

File sizes: `real_safety.py` 281, `real_timing.py` 112, `test_unit_real_safety.py` 314, `test_unit_real_safety_filter.py` 227,
`test_unit_real_timing.py` 142.

## Track B1 - round 5

Reviewer: independent code reviewer (Claude Opus 5.5). I did not write this track or any earlier review or response, and I saw none of
the author's reasoning. Date: 2026-10-18, 18:38-18:45.
Scope: `g1sim/real_safety.py` (281 lines, mtime 18:13, unchanged since round 4), `g1sim/real_timing.py` (112, 18:00, unchanged),
`tests/test_unit_real_safety.py` (314, 18:35), `tests/test_unit_real_safety_filter.py` (227, 18:35), `tests/test_unit_real_timing.py`
(142, 18:35). Checked against PLAN.md 3.3, rules 4.2 and the B1 task block. Consumers re-read: `g1sim/real_run.py` (`new_filter`, `tick`,
`measured_now`, `_soft_exit`), `g1sim/real_run_policy.py` (`run`, `_step`, `_target`, `_observe`, `_shadow_evaluate`),
`g1sim/real_run_modes.py` (tick phases), `g1sim/real_io.py` (`latest()`).

### Runs (full output in runs/)
- `runs/real_b1_review_r5_unit.log`: B1 verify command in the sim env (`~/dev/g1-ext/.venv`, Python 3.12), `-v`: **44 passed**, exit 0.
- `runs/real_b1_review_r5_unit_project.log`: project `.venv`: 43 passed, 1 skipped with a reason (table_env needs the sim env), exit 0.
- `runs/real_b1_review_r5_py310.log`: `import g1sim.real_safety, g1sim.real_timing` under the robot env (Python 3.10.21): OK.
- `runs/real_b1_mutation.log` (the author's, 18:36): control green, 61 mutations on fresh temp copies, each against its named test, all
  caught. I read entries 47-61: each round-4 survivor (R4-M1/M2/M3/M4/M5/M9b/M10/M11/M15/M19/M20/M21) fails its new test. Accepted.
- `runs/real_b1_review_r5_mutation.log` (mine; script `runs/real_b1_review_r5_mutate_script.py`): 16 new mutations, each on a fresh temp
  copy of g1sim/tests/scripts/docs (repo never edited), run against all three B1 test files **and** B2's `test_unit_real_run.py` +
  `test_unit_motion_gate.py` (fakes only, no DDS participant). The stats/table_env pin test is deselected in the copies (it needs the
  real tree; no mutation touches it). Controls: B1 43 passed, B2 25 passed. **Result: 8 caught by B1, 1 caught by B2 only, 7 survive both.**
  Every survivor is analysed below; none is reachable with harm on Monday.
- Lint: **not run** (`runs/real_b1_review_r5_lint.log`: no ruff, flake8 or pyflakes in either env). Cosmetic; does not move the gate.
- No MuJoCo, DDS or GPU process was started (B2's tests import `unitree_sdk2py` but create no participant).

### Round-4 findings re-checked (read in the code and shown failing)
- R4-F1 fixed: `test_waist_max_clips_waist_yaw_target` drives a second filter to -0.5 in `policy` (inside the envelope bottom -0.81) and
  asserts `min >= -0.15 - EPS` and last `== -0.15`. Author's mutation 47 (`np.minimum`) fails it.
- R4-F2 fixed: `test_sdk_targets_maps_action_slots_to_motors` pins the literal `[26, 101, 102, 0..13]`; mutations 48-50 fail it.
- R4-F3 fixed: `test_factory_pose_gap_is_the_largest_joint_difference` (+0.3 / -0.1 -> 0.3); min (0.0) and sum (0.4) fail it (51, 52).
- R4-F4 fixed: `test_stale_reason_thresholds` now has `hand_age 0.51` in `run` and `ran_out_s 1.01` in `replay` (53, 54).
- R4-F5 fixed: `test_ramp_out_from_weight_0_4_starts_at_0_4` (55).
- R4-F6 fixed: `test_scheduler_requests_after_25_actions_once_until_the_answer` (24.9 False, 25.0 True, pending False, answer clears)
  and `test_scheduler_refuses_chunk_of_the_wrong_shape` (56-61).
- Carried, outside B1's files: the runbook still has no `MAX LEAD` / `M3_LEAD_MAX` line (`grep` on `docs/ROBOT_DAY_RUNBOOK.md` finds
  nothing; E2's), and the PLAN.md B1 task block still does not name the round-3/round-4 tests (planner's).

### Verdict 1: did the code do what the plan says?
Yes. Every 3.3 name has the contract's signature and meaning, the filter order matches 3.3, and every behaviour that 86 reaches on
Monday is now shown failing on a broken version (author's 61 + my 8 B1 catches). The survivors below are defensive guards that the
current callers cannot reach, or changes that are equivalent / harmless given how B2 calls B1.

### Verdict 2: is the plan right for real hardware?
Nothing new. The two carried items (runbook `MAX LEAD` line, B1 task-block test list) are E2's and the planner's.

### Findings

**R5-F1. MINOR: three fail-closed guards documented in the code are not shown failing; none is reachable from today's callers.**
- R5-M1 (`step()` unknown-phase `ValueError` removed, `real_safety.py:231`): survives B1 and B2. Reachability: every phase B2 passes is
  a literal from `PHASES` (`real_run_modes.py:49/69/78/84`, `real_run.py:284-310`, `real_run_policy.py:69`). If a later edit typed
  "Policy", the envelope would silently be skipped; the guard prevents that but no test proves it.
- R5-M2 (`smoothstep_weights` [0, 1] check removed, `:89-91`): survives. B2's `weight_step_problem` would still refuse a jump.
- R5-M6 (`stale_reason`: LowState age `None` counted as fresh, `real_timing.py:103`): survives. Unreachable: `real_io` `latest()` returns
  `(None, inf)` before the first message (`real_io.py:211-212`), and `measured_now` faults on `msg is None` before anything is sent.
- Fix (optional, about 3 min, test only): `pytest.raises(ValueError)` for `flt.step(t, t, "Policy", 1.0)` and for
  `smoothstep_weights(0.0, 1.2)`; `stale_reason(None, ...) == "lowstate_stale"` in `test_stale_reason_thresholds`.

**R5-F2. MINOR: the lead monitor's joint scope is not pinned. Restricting it to the arms (waist yaw excluded) passes every test.**
- R5-M7 (`gap = ...[ARMS]`, `real_safety.py:276`): survives B1 and B2. Every lead test puts the lead on `ARM`.
- Reachable in principle: a torso blocked while turning (waist yaw lead) would then not raise `contact_fault`. Impact is small: the
  waist is limited to +-0.15 rad at 0.3 rad/s by default, kp 250, and the contract does not name the joints. B2's `MAX LEAD` line already
  covers arm and waist.
- Fix (optional, 1 min): in `test_lead_fault_fires_after_13_ticks_only_at_full_weight` add a third filter with the 0.26 lead on
  `WAIST_YAW` (waist_max wide enough) and assert `contact_fault` on tick 13.

**R5-F3. MINOR: `soft_exit_plan`'s "13/14 stay q0 in every target" (3.3) is not asserted.**
- R5-M3 (current pose not passed through `_hold_waist_roll_pitch`, `:146`): survives. Harmless today on two counts: B2 passes
  `self.command`, which the filter already set to q0 at 13/14, and every exit target goes through `SafetyFilter.step`, which forces 13/14
  := q0 again (`test_waist_roll_pitch_always_equal_q0`).
- Fix (optional, 1 min): `test_soft_exit_from_policy_passes_through_ready_pose` already uses a `current` whose 13/14 are 0.3 off q0;
  add `assert all(np.array_equal(q[1:3], q0[1:3]) for _, q in plan)`.

**Survivors that need no action (recorded):**
- R5-M4 (`reset_last` a no-op): harmless. Its only caller, `real_run_policy.py:82`, passes `r.command`, which always equals
  `filter.last` (every sent message goes through the same filter, and `step` sets `last` to what it returns). The method is redundant
  but correct.
- R5-M8 (no "first request at once" when no chunk exists): caught by B2's `test_replay_requests_advance_tick_with_scheduler` and
  `test_replay_without_hand_state_fills_last_command`. Covered at the integration level.
- R5-M9 (waist rate limit 3x faster downwards only): contrived (needs a per-joint asymmetric bound). The realistic one-sided bug
  (`np.minimum` without the lower bound) is caught through the arms in the `policy` phase, where the envelope pulls them downwards.
- R5-M14 (`target()` plays action 49 for one more action instead of None): equivalent on Monday, because `_target` holds the last
  played action (`hold17`) whenever `target()` is None, and the run-out timer counts from action 49 either way.

**Clean code lens.** A second-year student can explain every function: each is short, named for what it does, and the two subtle lines
(`_envelope17` with +-inf, the NaN-as-infinite lead and gap) say why in a comment. Every one of the 44 tests has a Proves/Catches
docstring (checked with `ast` on all three files) and reads Arrange-Act-Assert. Cosmetic only: `test_unit_real_safety.py` is 314 lines
(the ~300 rule is about modules), `test_unit_real_safety_filter.py:109` is 132 characters, `test_stale_reason_thresholds` names no
expected outcome (its name comes from the plan), and `ready17` is still duplicated in two test files (accepted in earlier rounds).

### Overall: GO
All round-4 majors and minors are fixed and shown failing on broken copies. My own 16 mutations found no reachable, harmful gap: 8 were
caught by B1, 1 by B2, and the 7 survivors are unreachable guards, a masked contract line or equivalent changes (R5-F1 to R5-F3 are
optional test additions of about 5 minutes together; they do not block). The safety logic was not changed in this round. Not run: lint
(no tool installed), cosmetic only. Carried outside B1: runbook `MAX LEAD` line (E2), B1 task-block test list (planner).
