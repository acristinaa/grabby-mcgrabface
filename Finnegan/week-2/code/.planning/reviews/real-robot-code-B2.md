# Code review: Track B2 (adapter 86, real_send, real_run*, real_preflight, motion_gate)

## Track B2 - round 1

Reviewer: independent code reviewer (opus). I did not write this code and have not seen the author's reasoning.
Date: 2026-10-18, 18:12 to 18:22. Plan: `.planning/real-robot/PLAN.md` rev 6.3 (sections 0, 3.3, 3.4, 4.2, 4.3, task B2).
Files reviewed: `scripts/86_real_run.py`, `g1sim/real_send.py`, `g1sim/motion_gate.py`, `g1sim/real_run.py`,
`g1sim/real_run_modes.py`, `g1sim/real_run_policy.py`, `g1sim/real_run_log.py`, `g1sim/real_preflight.py`,
`tests/test_unit_real_run.py`, `tests/test_unit_motion_gate.py`, `tests/helpers/fake_real_run.py`. I also read the interfaces
they use: `real_io`, `real_factory_pose`, `real_camera`, `real_safety`, `policy_wire`, 81's hello.

### What I ran (full output in runs/)
- Unit tests, sim env: `runs/real_b2_review_r1_unit.log`: 29 passed in 1.8 s, exit 0.
- `86 gate`: `runs/real_b2_review_r1_gate.log`: "MOTION: LOCKED / docs/ROBOT_DAY_MOTION.json not found - only Finnegan can unlock", exit 2.
- Robot env (Python 3.10.21, `~/dev/g1-ext/robot-day-test/robot-env`): 86 imports and its loaded-file list
  (`runs/real_b2_review_r1_loaded_py310.log`): 17 g1sim files, OK; `MotorCmds_()` + `fill_hand_cmd` + serialize OK under 3.10 and 3.12
  (`runs/real_b2_review_r1_handcmd.log`).
- My own 20 mutations (deliberately broken copies) on a scratch copy, never in the repo (`runs/real_b2_review_r1_mutation.log`,
  script in the session scratchpad): **7 caught, 13 survived**.
- Probes with the author's fakes (`runs/real_b2_review_r1_probes.log`): Ctrl+C in the ramp-in -> exit 3, weight steps <= 0.015, ends at w 0;
  stale LowState in the ramp-in -> exit 4; camera stale and policy stall in `run` -> exit 3 through `to_ready` then `to_start`;
  NaN in a chunk -> exit 4 `invalid_target`; release from a 0.8 rad gap -> first message w 1 at the measured angles, peak 0.299 rad/s,
  ends at w 0. All of these match rules 3, 10, 11, 12 and 13.
- Dead terminal probe (`runs/real_b2_review_r1_sighup.log`) and caffeinate probe (`runs/real_b2_review_r1_caffeinate.log`).

### Verdict 1 - did the code do what the plan says? PARTLY
The core is right and I could not break it in any probe: q0 frozen for the whole ramp-in, waist roll/pitch held in every message, every
writer created only after preflight and after the early sidecar, release at 0.3 rad/s with its single allowed weight jump, soft exits from
`policy` through `BALL_READY_POSE`, fault ramp-out on the measured angles, gains never above SIM_GAINS, `rt/lowcmd` and the banned SDK
names absent. But three rev-6.3 items in the task text are not built (R3-3 as specified, R3-5, R3-10 (c)), three plan-named tests are
missing, and a large share of the safety refusals have no test that fails when they are broken.

### Verdict 2 - is the plan right for real hardware? MOSTLY, two additions
(1) Rule 12 lists SIGHUP (terminal closed) as a soft-exit trigger, but after the terminal closes every write to stdout/stderr fails, so
the plan must require that 86's exits never depend on the terminal (finding 2). (2) Rule 5 says "hard clip + rate limit in every phase",
while rule 12's fault ramp-out must jump to the measured angles; the plan should state that the fault ramp-out is exempt from the clip
and the rate limit, and whether 13/14 stay at q0 there (minor 2). (3) `files_modified` lacks B2's five split files (minor 4).

### Findings

**1. MAJOR - R3-5 not built: `--out` outside `runs/real` is not refused off loopback; test missing.**
Plan 3.4 and task step 2a: "off loopback a DIR outside `real_io.RUN_LOG_DIR` is refused (exit 2)", test
`test_out_dir_outside_run_log_dir_refused_off_loopback`. `grep RUN_LOG_DIR` finds only the defaults (`real_run.py:55`,
`86_real_run.py:87`); no check exists. Probe: `ready` with `iface="en5"` and `out=/tmp/...` ran to exit 0 (probes log, last-but-two line);
`86 ready --iface en5 --out /tmp/x` passed the argument checks and stopped only at the locked gate. Impact: a robot run that logs
elsewhere and is then killed leaves its `exit_code: null` sidecar where 84's `arms_may_be_held_by_us()` (default `RUN_LOG_DIR`) and the
next 86 run (default `--out`) never look, so 84 allows `--overwrite-factory-pose` on a pose our code still holds. Fix: in
`real_preflight.argument_problem` (or 86 before the gate), resolve `--out` against the repo and refuse off loopback when it is not
inside `real_io.RUN_LOG_DIR`, with the plan's message; add the planned test and a mutation entry.

**2. MAJOR - closing the terminal (SIGHUP) leaves the arms held at weight 1: no soft exit, no fault ramp-out.**
Rule 12 makes SIGHUP a soft exit and promises "a `finally` block ramps out on any exception". Measured on this Mac: after the terminal
closes, `print()` raises `OSError: [Errno 5] Input/output error` (pty probe). In 86 the first thing after the signal is
`check_signals()` -> `print("stop requested ...")` (`real_run.py:208`), so `tick()` raises OSError; `_drive`'s `except Exception` then
calls `traceback.print_exc()` (`real_run.py:134`), which raises again inside the handler, so neither `_soft_exit` nor `_fault_exit` runs
(their own `print`s at lines 272 and 296 would fail as well). Reproduced with the author's fakes and a print that fails after the
signal: `run_mode raised OSError`, last message `to_ready` at weight 1.0, 0 `ramp_out` messages, sidecar exit 4 / last_weight 1.0
(`runs/real_b2_review_r1_sighup.log`). The sidecar makes 84 demand `86 release`, so it is recoverable like a kill -9, but it is not the
designed behaviour and it hits the same arm-over-the-table case rule 16 (e) worries about. The same happens with a BrokenPipe if anyone
ever pipes 86 through `tee` (Ctrl+C also stops `tee`). Fix: one small `say(text)` helper in the runner that writes to the terminal and
swallows `OSError` (logging the text in the sidecar events instead), used for every operator message and for the traceback; or, in the
SIGHUP handler, point stdout/stderr at `/dev/null`. Add a unit test "print fails after the signal -> exit 3, last weight 0" and see it fail
on today's code. Plan: add one sentence to rule 12.

**3. MAJOR - R3-3 gate (a) not built as specified; its two plan tests are missing (one mutation survives).**
Plan 4.3 (a): `git cat-file -e HEAD:docs/ROBOT_DAY_MOTION.json` and parse the JSON from `git show HEAD:...`, "never from the working
tree"; tests `test_gate_refuses_untracked_motion_json` [mutation] and `test_gate_reads_motion_json_from_head_not_worktree`. The code reads
the working-tree file (`motion_gate.py:64`) and uses `git ls-files --error-unmatch` (the index, not HEAD) plus `git diff --quiet HEAD`.
I found no functional bypass today (a staged-only or edited JSON is caught by the diff), but the safety of the worktree read now rests on
the single `diff --quiet HEAD` line, and dropping that line survives every test (mutation "gate drops the unlock-file diff check":
SURVIVED). The untracked case is covered by `test_gate_refuses_uncommitted_motion_json` (my mutation dropping `ls-files` was caught), but
under another name and not in `runs/real_b2_mutation.log`. Fix: build (a) with the plan's two git primitives, add both named tests,
record the mutations.

**4. MAJOR - R3-10 (c) not built: foreign hand-command samples are not warned about.**
Plan rule 2 / step 2a: samples on `rt/brainco/<side>/cmd` warn "another program sends hand commands - ask the owner to stop their hand
tool, or run with --no-hands" and log `foreign_hand_cmd`. `real_io.active_writers` already listens on those topics, but
`real_preflight.run_preflight` only looks at `rt/lowcmd` and `rt/arm_sdk` (lines 29-36); `grep foreign_hand` finds nothing. Impact: an
owner's hand tool and our 50 Hz hand commands fight silently during M3-M6. Fix: a few lines in `run_preflight` plus a unit test with a
fake sample count.

**5. MAJOR - many safety checks have no test that fails when they are broken (13 of 20 reviewer mutations survive).**
Survived (`runs/real_b2_review_r1_mutation.log`):
- m6-8 runner weight-step check: removing the call in `tick()` survives, and applying the release exemption to every release message
  survives. The planned test `test_weight_step_above_0_02_refused_except_release_first_message` calls only the pure helper, so it cannot
  see the runner skip it (it also tests three things in one test).
- preflight refusals (rule 2): stale LowState at start, a joint outside `JOINT_LIMITS`, a replay server accepted by `run`, no fresh camera
  frame for shadow/run, the speed gate (e) at its call site.
- sim bypass: removing the loopback check in `sim_bypass_problem` survives. In 86 that means `--yes-sim --iface en5` would skip the unlock,
  both names and HANGING on the robot (the robot-network check still stands behind it, but only that one).
- policy phase: stale camera/policy/hand state, the 2 s policy timeout, and the `envelope_streak` reaction (`real_run.py:216`);
  hands still sent during a fault (rule 12 "hands hold"); the exit-path overrun.
Several of these are scoped to Track C's e2e (F1, F2, F7), which is pending; the rest (weight step in the runner, preflight refusals,
sim-bypass loopback, envelope streak, hands hold in a fault) are unit-sized and belong here. Fix: about six short tests with the existing
fakes (one behaviour each, Proves/Catches), seen failing on the broken copy, logged in `runs/real_b2_mutation.log`.

**6. MAJOR (clean code / test correctness) - the guard in `test_policy_exit_goes_through_ready_pose_before_q0` can never fail.**
`tests/test_unit_real_run.py:210-214`: `phases.index(after_policy[0]) - 1` finds the FIRST `to_ready` (the rise before the policy), so
`last_policy` is the last `ramp_in` message (index 100), not the last policy message, and "policy never left the ready pose: the test
proves nothing" always passes. Shown with a policy that never leaves the ready pose: the guard value is 1.70 (passes) while the real
distance is 0.0 (`runs/real_b2_review_r1_guard.log`). The main assertion still works (my mutation "straight to q0" was caught). The
index arithmetic on lines 210-213 (`phases[len(phases) - phases[::-1].index("policy"):]`, `len(phases) - after_policy[::-1].index(...) - 1`)
is exactly what a second-year student cannot explain, which is how the bug got through. Fix: compute `last_policy_i` with one plainly
named helper (for example "index of the last message whose phase is X") and assert on it.

**Minor 1 - `hands-open --no-hands` sends nothing and reports `86 HANDS-OPEN: DONE`.** Probe: exit 0, reason "done". Refuse it
("hands-open needs the hands; drop --no-hands").

**Minor 2 - the fault ramp-out bypasses the filter (by design) and sends 13/14 at the measured angles, not q0.** `real_run.py:301`
(`filtered=False`) with `fault_targets` = measured. Rule 4 says 13/14 at q0 in every message while w > 0; rule 5 says clip + rate limit
in every phase. Probe "contact in policy": one 0.32 rad step at the start of the ramp-out (towards the measured pose, so it reduces
force). I think the behaviour is right; set 13/14 := q0 in the fault target or write the exemption into the plan.

**Minor 3 - clean-code smalls.** `GAIN_TOLERANCE` (`real_send.py:21`) is also used as the hand-cap slack (rename to `FLOAT_SLACK`);
86 `report()` repeats the exit codes as the literals 0/2/3/4 instead of `real_safety.EXIT_CODES`; `record['recorded_at'][11:16]`
(`real_preflight.py:101`) needs a name or comment ("HH:MM"); `stack = lambda ...` in `real_run_log.py:96` should be a small function;
`tests/test_unit_real_run.py` is 334 lines (over ~300) and `motion_code_problems` (the docstring-id set comprehension) is dense for
the reader; two tests build a whole `make_io(...)` only to read `.factory_record` while the fake module has `factory_record()`;
`RunArgs.out` is typed `str` but defaults to a Path; `RunResult` has no docstring.

**Minor 4 - plan file list.** B2 added `g1sim/real_run_modes.py`, `real_run_policy.py`, `real_run_log.py`, `real_preflight.py` and
`tests/helpers/fake_real_run.py` (all loaded by 86 or its tests). They must be in `files_modified` and in the final gate's list for
Finnegan's `git add`; otherwise gate (b) locks with "not in the unlocked commit" (it fails closed, but it would cost the robot day).

**Minor 5 - `RunLog.finish` writes the npz before the final sidecar.** A failing npz write (disk full) ends 86 with a traceback instead
of the two last lines; the arms are already handed back and the sidecar stays `exit_code: null` (safe side). Write the sidecar first,
or catch the npz error and note it.

### Checks I could not run (they keep parts of the gate open)
- Unit tests under Python 3.10 (the robot env has no pytest; I ran 86's imports, the gate and the hand/arm message fill under 3.10.21).
- e2e against 85 (Track C, pending integration).
- Ctrl+C behaviour of `systemd-inhibit ... python` on Ubuntu (macOS `caffeinate -i` checked: it waits for the python child, so a
  second Ctrl+C still reaches 86).

### Gate line
Track B2 round 1: **NO-GO** - 6 majors (R3-5 `--out` rule missing; dead terminal skips every exit; R3-3 gate (a) and its tests not as
planned; R3-10 (c) hand-command warning missing; 13 of 20 safety mutations survive; a test guard that can never fail), 5 minors.
Core motion logic correct in every probe.

## Track B2 - response (fix round 1)

Implementer (opus), 2026-10-18 18:34. Every major and every minor fixed; none refuted. Plan read at rev 7.

**1. R3-5 `--out` (fixed).** `real_preflight.argument_problem(mode, args)` now also calls `out_dir_problem`: `--out` is resolved
against the repo root (`resolve_out_dir`); off loopback it must be exactly `real_io.RUN_LOG_DIR` (not a subfolder either, because
`arms_may_be_held_by_us` reads only the sidecars directly in that folder), else exit 2 with the plan's text. 86 calls the same function
before the gate, so `86 ready --iface en5 --out <elsewhere>` stops at once (`runs/real_b2_fix1_out_refusal.log`, exit 2). Test
`test_out_dir_outside_run_log_dir_refused_off_loopback` [mutation logged].

**2. Closed terminal (fixed).** New `Runner.say(text)`: prints, and on OSError keeps the text in the sidecar's
`unprinted_messages`. Every operator message in real_run, real_run_modes, real_run_policy and the preflight warnings go through it; the
two tracebacks go through `say(traceback.format_exc())`. Test `test_closed_terminal_after_stop_signal_still_hands_arms_back` (print
raises OSError EIO after the signal -> exit 3 operator_stop, last message weight 0, message in the sidecar); seen failing on the copy
where `say` does not catch OSError (`runs/real_b2_mutation.log`: the OSError escapes `run_mode`, as in your probe). One sentence added
to plan rule 12.

**3. R3-3 gate (a) (fixed).** `_unlock_problems` now asks `git cat-file -e HEAD:docs/ROBOT_DAY_MOTION.json` (missing -> "not found",
or "is not committed" when a working-tree file exists), parses the JSON from `git show HEAD:...`, and keeps
`git diff --quiet HEAD -- <file>`. The `ls-files` check and the working-tree read are gone. Tests `test_gate_refuses_untracked_motion_json`
(renamed from `..._uncommitted_...`; untracked JSON with commit = HEAD and today's date -> exactly "is not committed") and
`test_gate_reads_motion_json_from_head_not_worktree` (committed false, working copy true -> "does not allow motion" and "was changed
after its commit"). Three mutations logged: drop cat-file, read the working tree, drop the diff check; all caught. Note: with cat-file
dropped, `git show HEAD:` still fails, so the gate stays locked; the test catches the lost "not committed" message.

**4. R3-10 (c) (fixed).** `real_preflight.warn_foreign_commands` warns (plan text) and logs `foreign_hand_cmd` for samples on
`real_io.HAND_CMD_TOPICS`; the run goes on (warn, not refuse). Test `test_foreign_hand_commands_warn_and_are_logged` [mutation logged].

**5. Surviving mutations (fixed, unit-sized ones).** New tests, each seen failing on its broken copy:
`test_weight_step_above_0_02_refused_except_release_first_message` (rewritten: the runner drives a release plan with a 1 -> 0.5 jump;
catches both "call removed" and "exemption for every message"), `test_preflight_refuses_before_any_writer` with 5 cases (stale LowState,
joint outside limits, replay server for `run`, stale camera for `shadow`, speed 0.8 off loopback at the call site),
`test_sim_bypass_refused_off_loopback`, `test_envelope_streak_ends_policy_with_soft_exit`, `test_fault_exit_sends_no_hand_command`.
Still without a unit test (left to Track C's e2e as you scoped them): policy-phase stale camera/policy/hand, the 2 s policy timeout, the
exit-path overrun.

**6. Guard that could not fail (fixed).** `last_index(phases, name)` helper in `tests/helpers/fake_real_run.py`; the guard now reads
the last policy message. Probe with a policy that never leaves the ready pose: the guard fails as it should
(`runs/real_b2_fix1_guard.log`). A mutation "policy exit straight to q0" is logged as caught.

**Minors.** (1) `hands-open --no-hands` refused ("hands-open needs the hands; drop --no-hands"; `runs/real_b2_fix1_handsopen_nohands.log`).
(2) Fault ramp-out target now has 13/14 := q0; plan rule 5 names the fault ramp-out as the one unfiltered phase. (3) `FLOAT_SLACK`
replaces `GAIN_TOLERANCE`; 86 `report()` uses `CODE_WORDS` built from `EXIT_CODES`; `recorded_hhmm` named; the lambda is now
`RunLog._stack_frames`; `RunArgs.out: pathlib.Path | str`; `RunResult` docstring; `motion_code_problems` split into
`docstring_nodes` + `names_used`; tests use `factory_record()`. The test file is split into `tests/test_unit_real_run.py` (257 lines),
`tests/test_unit_real_preflight.py` (new) and `tests/test_unit_real_send.py` (new). (4) Plan rev 7 already lists the five split files;
the two new test files must join B2's push set (named in STATUS). (5) `RunLog.finish` catches an OSError from the npz write and puts it
in the sidecar's `npz_error`, then writes the final sidecar.

**Verify.** Unit (4 files): 41 passed (`runs/real_b2_unit.log`). Mutations: 27 of 27 caught (the 10 old ones re-run + 17 new;
`runs/real_b2_mutation.log`, script `runs/real_b2_fix1_mutate.py`). `86 gate`: MOTION: LOCKED, exit 2, in the sim env
(`runs/real_b2_gate.log`) and in Python 3.10.21 (`runs/real_b2_gate_py310.log`). All unit tests in the repo: 351 passed, 2 skipped
(`runs/real_b2_fix1_all_unit.log`).

**Not done, open.** Plan rev 7 step 1b (`MAX LEAD` line + `test_max_lead_line_reports_the_largest_policy_phase_lead`) was not in this
round's findings and is not built; the runbook's stand-in (the M2 hold lead) still applies. `g1sim/real_run.py` is 311 lines (was 302).

## Track B2 - round 2

Reviewer: independent code reviewer (opus). I did not write this code and have not seen the author's reasoning.
Date: 2026-10-18, 18:34 to 18:43. Plan: `.planning/real-robot/PLAN.md` rev 7 (sections 0, 3.4, 4.2, 4.3, task B2 incl. steps 1b/2a/2b).
Files reviewed: `scripts/86_real_run.py`, `g1sim/real_run.py`, `real_run_modes.py`, `real_run_policy.py`, `real_run_log.py`,
`real_preflight.py`, `real_send.py`, `motion_gate.py`, `tests/helpers/fake_real_run.py`, `tests/test_unit_real_run.py`,
`tests/test_unit_real_preflight.py`, `tests/test_unit_real_send.py`, `tests/test_unit_motion_gate.py`; interfaces read in
`real_safety.py` (fault_targets, ramp_out, soft_exit_plan) and `real_io.sdk_q`.

### What I ran (full output in runs/)
- Unit tests, sim env (4 B2 files, verbose): `runs/real_b2_review_r2_unit.log`: **41 passed** in 2.7 s, exit 0.
- `86 gate`: `runs/real_b2_review_r2_gate.log`: "MOTION: LOCKED / docs/ROBOT_DAY_MOTION.json not found - only Finnegan can unlock", exit 2.
- Robot env Python 3.10.21: `runs/real_b2_review_r2_py310.log`: `86 gate` LOCKED exit 2; `86 ready --iface en5 --out /tmp/x`
  -> "REFUSED --out /tmp/x: robot runs log only to runs/real ..." exit 2 (R3-5 fix confirmed end to end). pytest is not installed there.
- My own 30 mutations on a scratch COPY (never in the repo), each run against the 9 real-robot test files (107 tests):
  `runs/real_b2_review_r2_mutation.log` (script in the session scratchpad, `b2r2/mutate.py`): **4 caught, 26 survived**.
  Baseline of the unmutated copy: 107 passed, 2 skipped.
- Probes with the author's fakes: `runs/real_b2_review_r2_probes.log` (Ctrl+C in release; shadow with sending forced on) and
  `runs/real_b2_review_r2_probe_one_signal.log` (one Ctrl+C during a normal end).

### Round-1 fixes: all present and working
#1 `--out` refusal (code + test, and live under 3.10); #2 `Runner.say()` swallows OSError and keeps the text in
`unprinted_messages`, used in every exit path; #3 gate (a) uses `cat-file -e HEAD:` + `git show HEAD:` + `diff --quiet HEAD`, both
planned tests exist; #4 `foreign_hand_cmd` warning + test; #5 the listed tests exist and their mutations are logged; #6 the guard now
uses `last_index` and can fail. Minors 1-5 done (13/14 := q0 in the fault target, `FLOAT_SLACK`, `CODE_WORDS`, sidecar after a failed
npz write). Core motion logic correct in every probe (frozen q0, release jump only on the first message, fault ramp-out on measured
angles, NaN angle falls back to the last command via `fault_targets`).

### Verdict 1 - did the code do what the plan says? PARTLY
Everything from round 1 holds. Two plan items are not met: rule 12's "first Ctrl+C = soft exit" is broken while a normal end is
already walking home (finding 1), and rev-7 step 1b (`MAX LEAD` line + its test) is not built (finding 4). The motion-gate call site
in 86 and several safety behaviours still have no test that fails when they are broken (findings 2, 3).

### Verdict 2 - is the plan right for real hardware? MOSTLY, three additions
(a) Rule 12 must say what ONE signal does while a soft exit that no signal started (normal end of ready/replay/run) is already running:
the runbook (line 333) promises "one Ctrl+C = soft exit", so it must not become a fault; and it must say what a signal during
`release` does (today: fault ramp-out mid-path, which I think is right, but it is written nowhere). (b) The B2 test list tests the gate
helpers only; add one test that drives 86's own `check_gate`/`main` (finding 2) - otherwise the operators' only real gate path is
covered by nothing (Track C's e2e uses `--yes-sim`, which skips (a)-(d)). (c) `files_modified` and task B2 `<files>` lack
`tests/test_unit_real_preflight.py` and `tests/test_unit_real_send.py`. Also for E2/integration: rule 16 (c) says that without the
`MAX LEAD` line the runbook uses the M2 hold lead "and says so" - the runbook has no such text today (`grep -i lead` finds only M2).

### Findings

**1. MAJOR - one Ctrl+C while a normal end walks home is a FAULT exit ("second_signal"), against rule 12 and the runbook.**
Trigger: `ready`, `replay` or `run` reaches its normal end (hold over / `--max-seconds` reached), the runner starts the soft exit with
`in_exit = True` (`real_run.py:273`); the operator presses Ctrl+C once. `check_signals` (`real_run.py:207`) faults on ANY signal while
`in_exit`, so the arms ramp out at once wherever they are. Reproduced (`runs/real_b2_review_r2_probe_one_signal.log`): `run`,
max 3 s, one signal 0.5 s into the 8.9 s way home -> exit 4 `second_signal`, phases `to_ready ... ramp_out`, `to_start` never reached.
Impact: in M4/M6 at the table this is exactly the R2-1 case (fault drop from a policy pose across the table edge) that rule 16/R15 try
to avoid, triggered by the action the runbook teaches as the safe one ("Soft exit (normal end, one Ctrl+C)", runbook line 333; 86
docstring line 19); the log reason "second_signal" is also false. Fix: escalate to a fault only on a signal after an earlier signal
(e.g. count the signals of the whole run: `total >= 2`), so a first signal during a normal-end exit is noted and the slow path
continues; keep release's "signal = fault" explicitly (with its own reason) or let it follow the same rule, and write the choice into
rule 12. Test: `test_one_ctrl_c_during_normal_end_keeps_the_slow_way_home` (seen failing on today's code).

**2. MAJOR - the operators' motion-gate path in 86 (`check_gate`) has no test; 4 of 4 mutations there survive.**
`scripts/86_real_run.py:141-154` is the only place where gate (a)/(b), the two names and HANGING are enforced for `ready/replay/
shadow/run/release/hands-open`. Survived (`runs/real_b2_review_r2_mutation.log`): "check_gate never asks HANGING", "ignores
gate_problems", "ignores the operator confirmation", "--yes-sim call site ignores sim_bypass_problem" (that last one would let
`--yes-sim --iface en5` skip everything). The helper tests (`test_gate_*`, `test_sim_bypass_*`) cannot see a call site that drops
them, the same pattern as round-1 #5 (m6-8). The task's safety list names the motion gate as a must-fail-on-mutation check. Fix: two
or three short tests that call `main([...])` (or `check_gate`) of 86 with `motion_gate.gate_problems`, `input` and `build_io` faked:
(i) LOCKED -> exit 2 and no `build_io`; (ii) unlocked, names OK, answer "yes" instead of HANGING -> exit 2, no `build_io`;
(iii) `--yes-sim --iface en5` -> exit 2. Load 86 with `importlib` as other tests load scripts; log the mutations.

**3. MAJOR - more safety behaviours at B2's call sites survive deliberate breakage (shadow moves the arms, Ctrl+C in release goes back
to the crash pose, limits ignored).** Survived, with the two most dangerous shown in probes (`runs/real_b2_review_r2_probes.log`):
- **shadow sends the policy** (`real_run_modes.py:34` `send_policy=True`): probe - shadow messages spread 0.350 rad (real code 0.000);
  no test runs `shadow` past preflight. M5 is the step where everybody believes nothing moves.
- **release without `r.in_exit = True`** (`real_run_modes.py:42`): probe - Ctrl+C mid-release -> soft exit that drives the left elbow
  from 0.600 back to 0.300 = the crash pose (towards the table), exit 3. No test sends a signal during release.
- runner ignores `--waist-hold`, ignores `--waist-max` (always 0.6), plays at speed 1.0 regardless of `--speed` (B1 r3 flagged the
  speed one; the call site is B2's `real_run_policy.py:70`).
- NaN chunk -> soft instead of fault (rule 11, `real_run_policy.py:161`); `fill_arm_sdk` weight range; `fill_hand_cmd` HAND_CAP and
  hand speed (the last line of defence of rules 8 and 9).
Fix: about six one-behaviour tests with the existing fakes (shadow sends only the hold pose; signal in release -> exit 4 and the elbow
never moves back towards the measured start; `--waist-hold` keeps motor 12 at q0 in every policy message; `--speed 0.5` vs `0.25`
halves the scheduler's position after N ticks; NaN in a chunk -> exit 4 `invalid_target` with no policy message after it; three
`pytest.raises` lines for weight 1.2, hand target 0.7, hand speed 0.6). Record each in `runs/real_b2_mutation.log`.

**4. MAJOR - rev-7 step 1b not built: no `MAX LEAD <rad> (<joint>)` line, no `test_max_lead_line_reports_the_largest_policy_phase_lead`.**
`grep -rn "MAX LEAD" g1sim scripts tests` finds nothing. Rule 16 (c) makes M4 at the table depend on the M3 lead being below
`M3_LEAD_MAX` 0.15 rad; without the line the operators have no number to read, and the stated stand-in is not in the runbook either
(Verdict 2). The author's response lists it as "not done". Fix: a small helper `max_policy_lead(sent_q, measured_q, phase, weight)`
over motors 12..28 where phase == "policy" and weight == 1, printed via `say()` before the final line in replay/run, plus the planned
test with a sagging fake arm (Catches: a lead taken over the ramp, where w < 1).

**Minor 1 - gate content details untested (5 survive):** `unlocked_by`, the ancestor check, the 40-hex commit pattern, `START`, and
names outside `OPERATORS` (`test_gate_needs_two_distinct_operators` only tries the same valid name twice). Add one parametrized refusal
test for the JSON fields and one for a wrong name / missing START.

**Minor 2 - still untested, scoped to Track C's e2e by the author:** preflight hello format (`chunk_size`/`action_dim`/`image_shape`)
and missing hand state for shadow/run, the 2 s policy timeout, the exit-path overrun, a signal during the 2 s preflight listen. Fine
for C, but the hello and hand-state refusals are one-line unit cases next to the existing parametrized preflight test.

**Minor 3 - a refused `--out` still writes there.** `86 ready --iface en5 --out /tmp/x` created `/tmp/x` and wrote
`20261018_183632_ready.npz/.json` into the folder it just refused (`record_refusal` uses `args.out`). Harmless (exit 2,
`arm_sdk_writer: false`) but surprising; log refusals of `--out` to `real_io.RUN_LOG_DIR`.

**Minor 4 - clean code smalls.** `g1sim/real_run.py` is 311 lines (over ~300); `ExitRequest` lives in `real_run_policy.py` though the
runner and the modes raise it (a reader looks for it in `real_run.py`; a tiny `real_run_exits.py` or a comment says why);
`WEIGHT_STEP_TOLERANCE` (real_run) and `FLOAT_SLACK` (real_send) are the same idea under two names; 86 `run_args` uses unexplained
fallbacks `speed 1.0` / `max_seconds 0.0` that differ from `RunArgs`' defaults (comment: "unused by this mode").

**Minor 5 - push set:** the two new test files are not in `files_modified` (Verdict 2 (c)); the author named them in STATUS.

### Checks I could not run
- Unit tests under Python 3.10 (no pytest in the robot env; imports, `gate` and the `--out` refusal ran under 3.10.21). Plan: integration.
- e2e against 85 (Track C, pending integration) - findings 2 and 3 are not covered there either (`--yes-sim`).

### Gate line
Track B2 round 2: **NO-GO** - 4 majors (one Ctrl+C during a normal end becomes a fault drop, against rule 12 and the runbook; 86's
motion-gate call site untested; shadow-sends / release-signal / waist / speed / NaN-chunk / send-range checks survive breakage;
MAX LEAD line of step 1b missing), 5 minors. All round-1 findings fixed; 41 unit tests pass; 4 of my 30 mutations caught.
Estimated fix: ~45-60 min (one small code change in `check_signals`, the MAX LEAD helper, ~10 short tests).

## Track B2 - response (fix round 2)

Implementer (opus), 2026-10-18 18:43-18:56. All 4 majors fixed, minors 1-4 fixed; minor 5 (PLAN.md) is the planner's file and is
handed over below. None refuted.

**1. One Ctrl+C during a normal end (fixed).** `Runner.check_signals` now counts the signals of the whole run
(`signals_at_start`): the first signal starts the soft exit, or - when a soft exit is already walking home (normal end, stale input,
envelope streak) - is printed ("already on the slow way home ...") and changes nothing; any later signal (`total >= 2`) is the
fault `second_signal`. `release` keeps "signal = fault" on purpose with its own flag `in_release` and reason `signal_in_release`
(a soft exit from release would walk back to the measured start = the crash pose). Test
`test_one_ctrl_c_during_normal_end_keeps_the_slow_way_home` (your probe: 3 s `run`, one signal 0.5 s into the way home -> now exit 0
`done`, `to_start` reached at q0, weight 0); seen failing on the copy that restores today's `or self.in_exit`. Note: a normal end
with one Ctrl+C on the way home reports `86 RUN: DONE` (exit 0); runbook line 374 says "normal end, one Ctrl+C, exit code 3" -
a normal end alone is exit 0 too, so that runbook line is already slightly off (E2's file, not changed here).

**2. 86's gate call site (fixed).** Three tests in `tests/test_unit_motion_gate.py` load 86 with importlib, fake `gate_problems`,
`input` and `build_io` (a call to build_io = the run reached the robot network): `test_86_locked_gate_refuses_before_any_robot_connection`,
`test_86_unlocked_gate_needs_two_operators_and_hanging` (same name twice then HANGING / "yes" instead of HANGING / all confirmed as the
positive control that does reach build_io), `test_86_yes_sim_on_robot_interface_refused`. 86 now passes `input` to
`confirm_hanging(input)` explicitly: the default argument `ask=input` was bound at import, so no test could type the answer. All 4 of
your mutations are caught.

**3. More call-site behaviours (fixed).** New tests, each seen failing on its broken copy: `test_shadow_sends_only_the_ready_pose`
(two mutations: `send_policy=True`, and shadow sending after `_shadow_evaluate`), `test_signal_during_release_ramps_out_without_going_back`
(elbow 0.6 rad past the factory pose, signal at 1 s: exit 4, elbow never moves back up; the mutation without the flag walks back),
`test_waist_hold_keeps_waist_yaw_at_q0`, `test_waist_max_clamps_policy_waist_yaw`, `test_speed_option_sets_the_playback_rate`
(30 vs 60 frames in 4 s), `test_nan_in_a_chunk_gives_fault_exit_before_it_is_sent`, `test_send_refuses_values_outside_the_safe_range`
(weight 1.2, hand target 0.7, hand speed 0.6).

**4. MAX LEAD (built).** `RunLog.max_policy_lead()` = largest |sent_q - measured_q| over the messages with phase "policy" and weight 1,
with its joint; `real_run.max_lead_line()` formats `MAX LEAD 0.030 rad (LeftShoulderPitch)` (or `MAX LEAD none (...)` when no
policy message went out); the runner prints it through `say()` in `_finish` for `replay`/`run` after any sent message, i.e. before
86's final line. Test `test_max_lead_line_reports_the_largest_policy_phase_lead` (arm sags 0.206 rad in the ramp, trails 0.03 rad at
weight 1: the line says 0.030; mutations "over every message" and "never printed" caught; numbers in
`runs/real_b2_fix2_probe_numbers.log`). The runbook stand-in (M2 hold lead) is no longer needed once this ships; the runbook's M3 step
should tell the operators to write the line down (E2).

**Minors.** (1) `test_gate_refuses_wrong_unlock_content` (unlocked_by, 12-char commit id, commit not in the checkout) and
`test_gate_refuses_unknown_operator_or_missing_start`; 5 mutations caught. (2) Preflight cases `wrong_chunk_size_in_hello` and
`no_hand_state_for_run`; the timing cases stay with Track C. (3) A run refused for its `--out` logs to `real_io.RUN_LOG_DIR`
(`_log_dir`, used by `record_refusal` and the Runner): live under Python 3.10.21, `86 ready --iface en5 --out /tmp/b2fix2_refused`
-> exit 2, log in runs/real, the refused folder never created (`runs/real_b2_fix2_out_refusal_py310.log`; I deleted that one
refusal log from runs/real afterwards). The preflight tests got an autouse fixture that points RUN_LOG_DIR into tmp_path - my
first run had leaked six refusal logs into runs/real, which I deleted. (4) The two exit paths and `ExitRequest` (plus `OPEN_HANDS`,
`HANDS_OPEN_S`) moved into the new `g1sim/real_run_exits.py` (69 lines); `real_run.py` is 294 lines. `WEIGHT_STEP_TOLERANCE` is gone,
the runner uses `real_send.FLOAT_SLACK`. 86 `run_args` docstring says the fallbacks are never read by modes without the option.

**For the planner (minor 5, not my file).** Push set / `files_modified` / task B2 `<files>` need: `g1sim/real_run_exits.py` (new),
`tests/test_unit_real_preflight.py`, `tests/test_unit_real_send.py`, `tests/test_unit_real_run_exits.py` (new),
`tests/test_unit_real_run_options.py` (new); the B2 verify line should run all six B2 test files. Proposed rule-12 sentence: "The run's
first signal starts the soft exit; if a soft exit is already running (normal end, stale input, envelope streak) it is noted and the
slow path continues; any further signal is a fault (`second_signal`). In `release` every signal is a fault (`signal_in_release`):
its plan is itself an exit path, and a soft exit would walk back to the crash pose."

**Verify.** Unit, 6 B2 files: 64 passed (`runs/real_b2_unit.log`). Mutations: 53 of 53 caught (27 from round 1 re-run, 26 new;
`runs/real_b2_mutation.log`, script `runs/real_b2_fix2_mutate.py`). `86 gate`: MOTION: LOCKED, exit 2 (`runs/real_b2_gate.log`; and
Python 3.10.21: `runs/real_b2_gate_py310.log`). All unit tests in the repo: 379 passed, 2 skipped (`runs/real_b2_fix2_all_unit.log`).
Not run: e2e against 85 (Track C).

## Track B2 - round 3

Reviewer: independent code reviewer (opus). I did not write this code and have not seen the author's reasoning.
Date: 2026-10-18, 18:57 to 19:10. Plan: `.planning/real-robot/PLAN.md` rev 7.1 (sections 0, 3.3, 3.4, 4.2, 4.3, task B2 incl. 1b/2a/2b).
Files reviewed: `scripts/86_real_run.py`, `g1sim/real_run.py`, `real_run_exits.py` (new), `real_run_modes.py`, `real_run_policy.py`,
`real_run_log.py`, `real_preflight.py`, `real_send.py`, `motion_gate.py`, `tests/helpers/fake_real_run.py` and the six B2 test files;
interfaces read in `real_safety.py` / `real_timing.py` (fault_targets, ramp_out, release_plan, soft_exit_plan, add_chunk) and `real_io.py`
(Track AF working tree, `git hash-object g1sim/real_io.py` = 846ba8ad25d842f33f8ddf3268b0f92bf924cbe9).

### What I ran (full output in runs/)
- Unit tests, sim env, 6 B2 files, verbose: `runs/real_b2_review_r3_unit.log`: **64 passed** in 5.2 s, exit 0. `runs/real` stayed empty.
- `86 gate`: LOCKED, "docs/ROBOT_DAY_MOTION.json not found - only Finnegan can unlock", exit 2, in the sim env
  (`runs/real_b2_review_r3_gate.log`) and in the operators' Python 3.10.21 (`runs/real_b2_review_r3_gate_py310.log`).
- Gate (b) input under Python 3.10.21 (`runs/real_b2_review_r3_loaded_py310.log`): `loaded_repo_files` lists 19 files, incl.
  `real_run_exits.py` and `scripts/86_real_run.py` - correct today.
- My own 34 mutations on a scratch COPY (never in the repo), each against all six B2 test files (`runs/real_b2_review_r3_mutation.log`,
  script in the session scratchpad `b2r3/mutate.py`): **11 caught, 23 survived**; baseline after restore 64 passed.
  Every mutation of the round-2 fixes was caught (both signal rules, release signal, MAX LEAD weight filter, hands opened, to_start).
- Probes with the author's fakes (`runs/real_b2_review_r3_probes.log`): signal in the ramp-in -> exit 3, steps <= 0.015, ends w 0;
  one hand stale in `run` -> exit 3 `hand_state_stale`; policy hangs -> exit 3 `policy_stale` through `to_ready`; camera stale -> exit 3;
  release from a pose whose waist roll differs from the record -> 13/14 held at the measured angles in every message; a wrong-shape
  chunk mid-run -> exit 4 `internal_error` (minor 4).
- macOS `caffeinate -i python` (`runs/real_b2_review_r3_caffeinate.log`): SIGINT to the whole foreground group (what Ctrl+C does)
  reaches 86 exactly ONCE (`signals 1`) and caffeinate waits for the child - so the new "count the signals" rule is not fooled by a
  forwarded duplicate on the Mac.
- Read: close_dds is registered at `atexit` by `init_dds`, so it runs after `main()` returned, i.e. after the `finally` ramp-out
  and the final sidecar. The policy worker thread is a daemon and is never joined, but it touches only the socket (no DDS), so this
  ordering is safe (rev 7.1 task-B2 note).

### Round-2 fixes: all present and working
#1 one Ctrl+C during a normal end keeps the slow way home (counted from `signals_at_start`; test + both directions of the mutation
caught); release keeps "signal = fault" (`signal_in_release`, test caught); #2 86's own gate call site tested (locked, names, HANGING,
`--yes-sim --iface en5`); #3 shadow sends only the ready pose, waist hold / waist max / speed honoured, NaN chunk = fault, send range
checks; #4 `MAX LEAD <rad> (<joint>)` over policy messages at weight 1, printed through `say()` before 86's last line. Minors 1-4 done;
`real_run.py` 294 lines, `real_run_exits.py` 69.

### Verdict 1 - did the code do what the plan says? YES for behaviour, PARTLY for the test rule
I could not make the code misbehave in any probe: frozen q0, 13/14 held in every message (also in release and the fault ramp-out),
weight steps <= 0.015 except release's first message, every exit ends at weight 0, writers only after preflight and the early sidecar,
gate (a) from `git show HEAD:`, (b) over the right 19 files, (c)/(d) at 86's call site, `--out` rule, foreign hand warning, MAX LEAD.
What is not met is section 0 / the task's test rule ("safety-critical checks must FAIL on a deliberately broken version"): gate (b)'s
input and four rule-2 / gate (e) refusals at their call sites survive deletion (majors 1, 2).

### Verdict 2 - is the plan right for real hardware? YES, with three paperwork items (minor 7)
(a) Rule 12 still lacks the sentence the fix round proposed (first signal during an already running soft exit is noted; any signal in
`release` is a fault). (b) `files_modified`, task B2 `<files>` and the B2 `<verify>` line still lack `g1sim/real_run_exits.py`,
`tests/test_unit_real_run_exits.py`, `tests/test_unit_real_run_options.py` (gate (b) fails closed if the first is not committed - it would
cost the robot day, not hurt anyone). (c) Runbook line 374 says "normal end, one Ctrl+C, exit code 3"; the code gives 0 for a normal
end (also with one Ctrl+C on the way home) and 3 for a Ctrl+C during the policy phase; the runbook still has no MAX LEAD step (E2).

### Findings

**1. MAJOR - gate (b) can be emptied without any test failing (`loaded_repo_files` has no test).**
`g1sim/motion_gate.py:28-40` produces the list gate (b) checks against the pinned commit. Mutations "return []" and "only files under
g1sim/ (drops scripts/86_real_run.py)" both SURVIVE all 64 tests (mutation log). Every gate test passes an explicit `[CODE]`, and the 86
call-site tests fake `gate_problems`. Track C cannot catch it either: its e2e compares `loaded_files_at_exit == gate.loaded_files`,
two outputs of the same function (`[] == []` passes), and the clean-clone `86 gate` of 6.4 would print UNLOCKED. Impact: the one guard
against an edited or stale file moving the robot (4.3 (b), a motion-gate check of the task's must-fail list) could silently check
nothing. It works today (19 files under 3.10, see above). Fix: one unit test with a fake `modules` dict (an in-repo `g1sim` module, a
`__main__` whose `__file__` is `scripts/86_real_run.py`, a site-packages module, a module outside the repo, a module without
`__file__`) asserting exactly the two repo files come back; Catches "gate (b) checks nothing". Log both mutations.

**2. MAJOR - four rule-2 / gate (e) refusals still survive deletion at their call sites.**
All four are one-line cases for the existing `test_preflight_refuses_before_any_writer` table (plus one release test), each a check the
plan lists, and none is scoped to Track C:
- `--waist-max` above 0.6 off loopback (gate (e), rule 6b): dropping `waist_max_refusal` from `argument_problem`
  (`real_preflight.py:73`) SURVIVES; then `86 run --iface en5 --waist-max 0.9` turns the torso past every column of the 87 lookup.
  The speed half of gate (e) has its case; the waist half does not.
- `replay` against a checkpoint server (3.4, rule 2): dropping the replay branch of `hello_problem` (`real_preflight.py:151`)
  SURVIVES (only "run with a replay server" is tested). Impact: M4 "replay at the table" would run the VLA, at the table, possibly on a
  blank `--no-camera` image.
- `ready`/`replay`/`run` without a factory record (rule 2 "record the factory pose first (runbook M0)", `real_preflight.py:130`):
  returning None SURVIVES (only the `release` variant is tested). Impact: a run starts with no record, so after a crash `86 release` is
  refused and only the owner's remote is left.
- release through the SafetyFilter (3.3, B1 review P2: "a caller that skips the filter must not exist"): `filtered=False` in
  `real_run_modes.release` SURVIVES; the release tests start with 13/14 equal to the record, so they cannot see 13/14 move.
  (My probe shows the real code holds them at the measured angles.) A release test from a pose whose waist roll differs from the
  record, asserting 13/14 == measured in every message, catches it.
Fix: three parametrized cases (`waist_max_above_ceiling_on_robot`, `checkpoint_server_for_replay`, `no_factory_record_for_ready`) and
one release test; log the four mutations in `runs/real_b2_mutation.log`.

**Minor 1 - the r1-minor-2 fix (13/14 := q0 in the fault ramp-out) has no test.** Deleting `real_run_exits.py:67` SURVIVES: in
`test_contact_fault_ramps_out_on_measured_angles` the measured 13/14 equal q0, so the line changes nothing. Make the fake waist roll
measure 0.02 rad off q0 at the contact and assert 13/14 == q0 in every ramp_out message.

**Minor 2 - rule 12 "hands opened actively there (at the ready pose)" is untested.** `pause = 0` (`real_run_exits.py:36`) SURVIVES:
the hands would open while the arms already descend (a held ball drops on the way down instead of at chest height). Assert in
`test_policy_exit_goes_through_ready_pose_before_q0` that the first open-hand command comes before the first `to_start` message.

**Minor 3 - three last-line-of-defence checks in `real_send` untested.** NaN in `q` (`real_send.py:42`, the only NaN guard of the
UNFILTERED fault ramp-out besides `fault_targets`), kd above SIM_GAINS (line 44) and negative hand targets (line 72) all SURVIVE. Three
more rows for `test_send_refuses_values_outside_the_safe_range`.

**Minor 4 - a malformed chunk mid-run is a fault, the plan says soft.** Rule 10: "policy error ... -> soft exit". A reply of the wrong
shape raises `ValueError` in `ChunkScheduler.add_chunk` (`real_timing.py:60`), which `_collect_chunk` (`real_run_policy.py:151-157`)
does not catch, so `_drive` turns it into `internal_error` and the arms ramp out at once from the policy pose (probe, the R2-1 case at
the table). Unlikely after the hello check. Fix: `except ValueError` -> `ExitRequest("soft", "policy_error")` after the InvalidTarget
branch (InvalidTarget is a ValueError subclass, so the order matters - say so in a comment).

**Minor 5 - `factory_problem` gets the raw `--out`.** `real_preflight.py:41` passes `args.out` to `arms_may_be_held_by_us`, which reads
a relative path against the cwd, while `RunLog` and `out_dir_problem` resolve it against the repo root (R3-5). Off loopback
`--out runs/real` from another folder passes the `--out` check and then reads the wrong folder ("arms free" -> the M0 hint). The
runbook never passes `--out`; pass `resolve_out_dir(args.out)`.

**Minor 6 - policy-phase input checks without a unit test.** Hand-state staleness uses the OLDER hand (`max`, `real_run_policy.py:138`);
`min` SURVIVES, and Track C's F1/F2 cover camera and server death, not one stale hand. Policy error reply (`status == "error"`) and the
2 s timeout also survive here (F2 covers server death in C). One unit case "right hand state 0.9 s old -> exit 3 hand_state_stale"
is enough; the rest can stay with C.

**Minor 7 - plan and runbook paperwork:** see Verdict 2 (a)-(c). Planner and E2, not B2's files.

**Minor 8 - clean code.** The runner travels as a one-letter, untyped `r` through every public function of `real_run_exits.py`,
`real_run_modes.py` and `real_run_policy.reset_policy` (`soft_exit(r, reason)`, `ready(r)`, ...); a student reading
`exits.soft_exit(self, reason)` has to find out what `r` is. Name it `runner` with a `"Runner"` type hint (TYPE_CHECKING import), as
`PolicyPhase.__init__(self, runner, ...)` already does. Everything else reads well: one job per function, named constants with sources,
every test has Proves/Catches and one behaviour.

### Mutations that survive but are covered elsewhere (no finding)
86 `signals.install()` removed / `SignalCounter` never counting (Track C F6 sends real signals to 86); camera stale, policy timeout,
exit-path overrun (C F1, F2, the 85 report gate); soft exit "jump to weight 1" during the ramp-in (the runner's weight-step check turns
it into a fault exit - defence in depth, probe); a signal during the 2 s preflight listen (the first tick then soft-exits before
anything is sent); hands-open checks no signals (2 s, hands only).

### Checks I could not run (they keep parts of the gate open)
- Unit tests under Python 3.10 (pytest is still not installed in the robot env; `86 gate` and the loaded-file list ran under 3.10.21).
- e2e against 85 (Track C started 18:57; F1/F2/F6 cover the C-scoped survivors above).
- Ubuntu `systemd-inhibit --what=... python 86 ...`: whether one Ctrl+C reaches 86 exactly once and systemd-inhibit waits for the child
  (no Linux here; on the Mac, caffeinate: once, waits). If systemd-inhibit died on SIGINT the shell would take the terminal back and a
  second Ctrl+C would no longer reach 86 - worth one check on the Ubuntu laptop in Part A.

### Gate line
Track B2 round 3: **NO-GO** - 2 majors, both tests only (gate (b)'s file list has no test; waist-max ceiling, replay-vs-checkpoint
server, no-factory-record and release-through-the-filter refusals survive deletion), 8 minors. The code itself behaved correctly in
every probe and all round-2 fixes hold; 64 unit tests pass; 11 of my 34 mutations caught (23 survive, 9 of them scoped to Track C or
harmless). Estimated fix: ~20-30 min, no production-code change needed for the majors (minors 4 and 5 are two-line code changes).

## Track B2 - response (round 3)

Implementer (opus), 2026-10-18 19:09-19:17. Both majors fixed (tests only, as the review said), minors 1-6 and 8 fixed; minor 7 is
planner/E2 paperwork (not my files, handed over below). None refuted.

**1. Gate (b)'s file list (fixed).** `test_loaded_repo_files_lists_exactly_the_repo_code_and_the_86_script`
(`tests/test_unit_motion_gate.py`): a fake `modules` dict with an in-repo `g1sim` module, a `__main__` at `scripts/86_real_run.py`, a
site-packages module inside a venv in the repo, a module outside the repo and one without `__file__`; exactly the two repo files come
back. Both of your mutations ("return []", "only g1sim/") are caught.

**2. Four refusals at their call sites (fixed).** Three rows in `test_preflight_refuses_before_any_writer`:
`waist_max_above_ceiling_on_robot` (iface en5, `--out` = RUN_LOG_DIR as in the runbook, so only the waist check can refuse: the
mutation now runs to exit 0 instead of a different refusal), `checkpoint_server_for_replay`, `no_factory_record_for_ready`. One release
test `test_release_holds_waist_roll_pitch_at_the_measured_angles` (`tests/test_unit_real_run.py`): start pose with waist roll 0.02 rad
off the record, 13/14 == measured in every message; with `filtered=False` the waist roll walks 0.05 -> 0.045 and the test fails.

**Minors.** (1) `FakeArm.stuck_drift` (added to the measured angles once stuck); `test_contact_fault_ramps_out_on_measured_angles`
pushes waist roll 0.02 rad at the contact and asserts 13/14 == q0 in every ramp_out message; deleting `real_run_exits.py` line
`target[WAIST_HOLD_POS] = q0[...]` is caught. (2) `test_policy_exit_goes_through_ready_pose_before_q0` now asserts the first open-hand
command goes out with a `to_ready` message at least HANDS_OPEN_S * SEND_HZ messages before the first `to_start`; `pause = 0` caught.
(3) Rows `nan_in_q`, `kd_above_sim_gains`, `hand_target_negative` in `test_send_refuses_values_outside_the_safe_range`; all three
mutations caught. (4) `_collect_chunk` catches `ValueError` after `InvalidTarget` and `ChunkTooLate` (comment says why the order
matters) -> `ExitRequest("soft", "policy_error")`; test `test_malformed_chunk_mid_run_gives_soft_exit_through_the_ready_pose`
(good chunk, then one a column short: exit 3 policy_error, next message after the policy phase is `to_ready`); the old behaviour
(exit 4 internal_error) is caught. (5) `run_preflight` passes `resolve_out_dir(args.out)` to `factory_problem` (no new test: the
finding asked none, and off loopback `--out` must already equal RUN_LOG_DIR). (6) `FakeHand` got an optional clock + `stale_after`;
`test_one_stale_hand_ends_run_with_soft_exit` (right hand 0.9 s old from 0.5 s on, left fresh: exit 3 hand_state_stale); `min` instead
of `max` is caught. (8) `r` -> `runner: "Runner"` (TYPE_CHECKING import) in every function of `real_run_exits.py`,
`real_run_modes.py` and in `real_run_policy.reset_policy` / `PolicyPhase.__init__`; no behaviour change.

**For the planner and E2 (minor 7, not my files).** Unchanged from the round-2 hand-over: rule-12 sentence; `files_modified`, task B2
`<files>` and `<verify>` need `g1sim/real_run_exits.py`, `tests/test_unit_real_run_exits.py`, `tests/test_unit_real_run_options.py`
(the B2 verify line should run all six B2 test files); runbook line 374 exit codes (normal end = 0, also with one Ctrl+C on the way
home; Ctrl+C in the policy phase = 3) and a MAX LEAD step for M3/M4. New: rule 10 now holds for a malformed chunk too (soft
`policy_error`), worth one word in the plan's rule-10 line.

**Verify.** Unit, 6 B2 files: 74 passed (`runs/real_b2_unit.log`; the task's 4-file command plus `test_unit_real_run_exits.py`
and `test_unit_real_run_options.py`). Mutations: 66 of 66 caught (53 from rounds 1-2 re-run with anchors updated for the rename, 13
new), run on a scratch COPY of g1sim/scripts/tests, the repo was never written (Track C runs from it); log `runs/real_b2_mutation.log`,
script `runs/real_b2_fix3_mutate.py`. `86 gate`: MOTION: LOCKED, exit 2 (`runs/real_b2_gate.log`; Python 3.10.21 robot env:
`runs/real_b2_gate_py310.log`, imports `runs/real_b2_fix3_import_py310.log`). `runs/real` stayed empty. Not run: the whole-repo unit
suite and e2e against 85 (Track C running sims; no MuJoCo process from this track).

## Track B2 - round 4

Reviewer: independent code reviewer (opus). I did not write this code and have not seen the author's reasoning.
Date: 2026-10-18, 19:18 to 19:30. Plan: `.planning/real-robot/PLAN.md` rev 7.1 (sections 0, 3.3, 3.4, 4.2, 4.3, task B2).
Files reviewed: `scripts/86_real_run.py`, `g1sim/real_run.py`, `real_run_exits.py`, `real_run_modes.py`, `real_run_policy.py`,
`real_run_log.py`, `real_preflight.py`, `real_send.py`, `motion_gate.py`, `tests/helpers/fake_real_run.py` and the six B2 test files.
`git hash-object g1sim/real_io.py` = 846ba8ad25d842f33f8ddf3268b0f92bf924cbe9 (Track AF working tree, unchanged since round 3).

### What I ran (full output in runs/)
- Unit tests, sim env, 6 B2 files, verbose: `runs/real_b2_review_r4_unit.log`: **74 passed**, exit 0. `runs/real` stayed empty.
- Unit tests under the operators' **Python 3.10.21** (robot env + a pytest overlay installed with `uv pip --target` into my session
  scratchpad; the robot env itself was not changed): `runs/real_b2_review_r4_unit_py310.log`: **74 passed**, exit 0. This closes the
  "unit tests under 3.10" item that rounds 1-3 could not run.
- `86 gate`: LOCKED, "docs/ROBOT_DAY_MOTION.json not found - only Finnegan can unlock", exit 2, sim env
  (`runs/real_b2_review_r4_gate.log`) and Python 3.10.21 (`runs/real_b2_review_r4_gate_py310.log`).
- My own 36 mutations on a scratch COPY (never the repo; script in the session scratchpad `b2r4/mutate.py`), each against all six
  B2 test files: `runs/real_b2_review_r4_mutation.log`: **28 caught, 8 survived**; baseline after restore 74 passed.
  Focus as asked: exits, frozen ramp-in, release speed, motion gate; plus re-checks of every round-3 fix.
- Probes with the author's fakes, real code and three mutated copies (`runs/real_b2_review_r4_probes.log`, script `b2r4/probe.py`).
  Real code: third Ctrl+C during the fault ramp-out -> exit 4, ends at w 0; stale LowState in the ramp-in -> exit 4, ends at w 0;
  two signals in the ramp-in -> exit 4, ends at w 0; release from waist yaw 0.35 -> first message at the measured 0.350, largest
  waist step 0.0059 rad/tick, ends at w 0. Weight steps <= 0.015 after release's first message in every probe.

### Round-3 findings: all fixed
- #1 gate (b)'s file list: `test_loaded_repo_files_lists_exactly_the_repo_code_and_the_86_script`; "return []" caught (my sweep).
- #2 the four refusals: waist-max ceiling, checkpoint server for replay, no factory record for ready (three preflight rows) and
  release through the filter (`test_release_holds_waist_roll_pitch_at_the_measured_angles`); "release: filter skipped" caught.
- Minors 1-6 fixed: 13/14 := q0 in the fault ramp-out caught; pause = 0 caught; three `real_send` rows present; a malformed chunk is
  now soft `policy_error` (ValueError after InvalidTarget, comment says why; both directions caught); `factory_problem` gets
  `resolve_out_dir(args.out)` (`real_preflight.py:41-42`); one stale hand -> exit 3 `hand_state_stale`, `min` caught.
- Minor 8 partly: `r` -> `runner: "Runner"` in every function signature, but see clean-code minor 3 below.
- Minor 7 (planner / E2) is still open, see minor 4.

### Verdict 1 - did the code do what the plan says? YES
Every probe behaved as rules 3, 4, 10-13 and 4.3 say: q0 frozen over the ramp (re-read mutation caught), 13/14 held at q0 in every
message incl. release and the fault ramp-out, every exit ends at weight 0 also from weight < 1, a third signal does not skip the
fault ramp-out, release's first message at the measured angles and 0.3 rad/s afterwards (1.0 and 0.5 rad/s mutations caught),
gate (a) from `git show HEAD:`, (b) over the loaded files, (c)/(d)/sim bypass at 86's call site (caught), MAX LEAD, two-step sidecar.
The task's named [mutation] tests all fail on their broken versions.

### Verdict 2 - is the plan right for real hardware? YES, with the carried-over paperwork (minor 4).

### Findings

**Minor 1 - "a third signal does not skip the ramp" (rule 12) has no test anywhere.** Mutation `if new <= 0 or self.in_fault:` ->
`if new <= 0:` (`g1sim/real_run.py:230`) SURVIVES all 74 tests, and Track C's `signal_86` sends at most two signals. Probe: with
the mutation a third Ctrl+C 0.5 s into the fault ramp-out raises ExitRequest out of `fault_exit` (no try around it in `_drive`),
`run_mode` raises, the last message sent has **w 0.855** and 86 prints no final line - the arms stay held at the measured pose.
Real code: exit 4, ends at w 0. Hammering Ctrl+C is the likeliest operator reaction, and `check_signals` has been rewritten in each
of the last three rounds, so this line needs its guard. Fix (no new test): add a third time to `signals_at(clock, 9.0, 10.0)` in
`test_second_ctrl_c_turns_soft_exit_into_fault_exit` (10.5 s = inside the 2 s fault ramp-out); the existing asserts (exit 4,
last weight 0) then catch it. Add "a third one does not stop the ramp-out" to its Proves line; log the mutation.

**Minor 2 - no unit test has a fault exit from weight < 1.** Mutation `runner.weight == 0.0` -> `runner.weight < 1.0`
(`g1sim/real_run_exits.py:69`) SURVIVES: every fault test faults at weight 1 (contact in to_ready, stale in the hold, second signal
on the way down, weight step in release). Probe with the mutation: stale LowState 1 s into the ramp-in ends with **w 0.500** sent
last, two signals in the ramp-in end with **w 0.198**; real code ends both at w 0. Track C's F7 pauses LowState only while the arms
are held at weight 1. Fix (no new test): parametrize `stale_after` in `test_stale_lowstate_gives_fault_exit_code_4` over the
ramp-in (1.0 s) and the hold (6.0 s); its last-weight-0 assert then catches the mutation.

**Minor 3 - clean code.** (a) Round-3 minor 8 is half done: `PolicyPhase` keeps the runner as `self.r` and re-aliases `r = self.r`
in seven methods (14 uses in `g1sim/real_run_policy.py`), so a reader still meets a one-letter name for the object that sends to
the robot; `self.runner` reads as the docstring already says. (b) `Runner._react(self, events, phase)` never uses `phase`
(`g1sim/real_run.py:250`). (c) `tests/test_unit_real_run.py` is 314 lines (section 0: ~300); moving the two policy-phase input tests
next to `test_unit_real_run_options.py` would do. None changes behaviour; fix only if the round is open anyway.

**Minor 4 - carried over, not B2's files (planner and E2):** (a) rule 12 still lacks the sentence on the first signal during an
already running soft exit and on `signal_in_release`; rule 10 could say "malformed chunk" too. (b) `files_modified`, task B2
`<files>` and `<verify>` still lack `g1sim/real_run_exits.py`, `tests/test_unit_real_run_exits.py`,
`tests/test_unit_real_run_options.py` (fails closed: the unlock recipe's `git status --porcelain -- g1sim scripts tests` and gate (b)
would refuse; it costs the day, it hurts nobody). (c) `docs/ROBOT_DAY_RUNBOOK.md:374` still says "Soft exit (normal end, one Ctrl+C,
exit code 3)" - a normal end is exit 0 (also with one Ctrl+C on the way home); and the runbook has no step that tells the operators
to write down 86's `MAX LEAD` line in M3, which rule 16 (c) needs before M4 at the table (grep finds no "MAX LEAD" in the runbook).

### Survivors without a finding
- soft exit "jump to weight 1": the runner's weight-step check turns it into a fault exit (defence in depth, as in round 3).
- ramp-in q0 = the factory record instead of the measured angles: preflight bounds that gap to 0.10 rad, so the ramp moves at most
  0.10 rad over 2 s; the plan's mutation for this test (re-read per tick) is caught.
- release filter with the run's `--waist-max` instead of the 0.6 ceiling: the filter's 0.3 rad/s waist rate limit walks the waist
  (probe: first message 0.344 from 0.350, 0.006 rad/tick) - no snap.
- 86 passing `[]` to `check_gate`: caught by Track C's e2e (`loaded_files_at_exit == gate.loaded_files`,
  `tests/test_e2e_real_adapter_sim.py:160`), pending its run.
- `_git_ok` True without git: `_git_output` then returns None, the JSON parse fails, the gate stays LOCKED.
- `last_weight_sent` always 0: only differs when an exception escapes the runner (see minor 1); A's reader is A's contract.

### Checks I could not run
- e2e against 85 (Track C, integration 6.3; B2's `<verify>` marks it "pending integration").
- Ubuntu `systemd-inhibit ... python 86`: whether one Ctrl+C reaches 86 exactly once (no Linux here; Part A on site).
Both are outside this gate's defined checks (task B2 `<verify>`/`<done>`), so they do not hold this gate open; they stay open for
the integration gate and Part A.

### Gate line
Track B2 round 4: **GO** - verdict 1 YES, verdict 2 YES. All round-3 findings fixed; 74 unit tests pass in the sim env and, for the
first time, under the operators' Python 3.10.21; 28 of my 36 mutations caught, every safety-relevant survivor either covered
elsewhere or harmless except two exit-invariant test gaps (minors 1 and 2: two one-line edits of existing tests, recommended before
the motion commit). Minor 3 clean code, minor 4 planner/E2 paperwork.
