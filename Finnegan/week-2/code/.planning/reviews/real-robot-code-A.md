# Code review - Track A (read-only bring-up)

## Track A - round 1

Reviewer: independent code reviewer (did not write Track A, no author reasoning seen). 2026-10-18 17:52-17:57.
Plan: `.planning/real-robot/PLAN.md` rev 6.3 (Track A was built on rev 6.2; rev 6.3 section 5 says A's R3-x items arrive through this
review and the fix round, so they are listed here as required work).
Files: g1sim/real_io.py, g1sim/real_factory_pose.py, g1sim/real_camera.py, g1sim/real_bringup.py, scripts/84_real_bringup.py,
tests/helpers/fake_camera.py, tests/test_unit_real_io.py, tests/test_unit_real_camera.py, tests/test_unit_no_publisher.py.

### What I ran (all output in full under runs/)
- Unit tests, sim env (Py 3.12): `runs/real_a_review1_unit.log` -> 23 passed, exit 0 (MJCF limit test ran, not skipped).
- `84 --dry-run`: `runs/real_a_review1_dryrun.log` -> exit 0, lists the 7 steps, "no DDS started".
- Project `.venv` collection (m6-3): `runs/real_a_review1_projvenv.log` -> 3 files skipped with a reason, no collection error.
- Operators' Py 3.10 robot env (`~/dev/g1-ext/robot-day-test/robot-env`): pytest is NOT installed there, so the unit tests were not run
  in 3.10 (not run). Instead a smoke script imported 84 + all Track A modules and called the core functions:
  `runs/real_a_review1_py310_smoke.log`, exit 0; `runs/real_a_review1_py310_dryrun.log` exit 0. The smoke also scanned every loaded
  module after `install_publisher_guard()`: no module still holds the original `cyclonedds.pub.DataWriter` (guard coverage complete
  for what 84 loads), and `Channel.SetWriter` resolves the patched name.
- Reviewer mutation checks on a scratch copy (repo untouched): `runs/real_a_review1_mutation.log`.
  M1 drop `exit_code is None` in arms_may_be_held_by_us -> caught (1 failed).
  M2 sidecar sort `reverse=True -> False` (oldest decides) -> SURVIVES (14 passed).
  M3 do not skip sidecars without `arm_sdk_writer` -> SURVIVES.
  M4 check_probe staleness 0.1 s -> 10 s -> SURVIVES (23 passed).
  M5 check_probe foreign `rt/arm_sdk` FAIL disabled -> SURVIVES.
  M6 joint_rows `inside` always True -> SURVIVES.
  M7 record_observation stale-LowState stop removed -> SURVIVES.
  The author's 8 mutants (`runs/real_a_mutation.log`) are genuine; I did not re-run them.

### Verdict 1 - implementation fidelity (did the code do what the plan says)
For rev 6.2: yes, closely. All 21 listed tests exist with Proves/Catches docstrings, the 8 [mutation] checks were seen failing, the
publisher guard patches all three bindings and runs first in main(), the factory-pose record, overwrite rule and conflict way-out
messages match 3.1 / R2-2 word for word, sidecar reading fails closed on unreadable files, Py 3.10 / 3.8 syntax is pinned by tests.
For rev 6.3 (required now): not yet - see major 1. Two safety-relevant decisions are correct in code but not proven by any test
(majors 2 and 3).

### Verdict 2 - plan correctness for real hardware
Mostly right. Two gaps in the plan itself: the Track A test list never asks for a test of the 84 preflight verdict (check_probe:
staleness, foreign arm_sdk samples), although 4.1 rule 2 is the read-only safety rule the operators rely on before 86 (major 3);
and the "newest arm-writer sidecar" selection, which is the heart of R2-2, is only tested with a single sidecar (major 2). Minor
plan note: joint limits have no measurement tolerance (minor 6).

### Findings

MAJOR 1 - rev 6.3 contract items missing (expected for the fix round, but required for GO).
 - R3-5: `RUN_LOG_DIR = "runs/real"` (real_factory_pose.py:18) is a relative string; the contract says `REPO / "runs" / "real"`.
   84's own reader is safe (`_resolve` anchors to REPO), but B2 imports this constant as 86's `--out` default and for the
   "outside RUN_LOG_DIR" refusal; a relative value lets 86 started from another cwd write sidecars 84 never reads (fail open).
   `BRINGUP_REPORT_DIR` missing; 84 writes reports to its own `REPO / "runs"` literal. Test `test_sidecar_found_when_86_ran_from_another_cwd`
   [mutation] missing.
 - R3-10 (c): `ARM_CMD_TOPICS` / `HAND_CMD_TOPICS` missing; `active_writers` (real_io.py:238) listens only on the two arm topics and
   `SampleCounter` hard-codes `LowCmd_` (real_io.py:224) - subscribing a hand topic with that type would silently match nothing, so the
   fix must pick `MotorCmds_` per topic and report `max_weight None` there; check_probe must warn on hand-command samples.
 - R3-9: `earliest_factory_arms_today()` missing; `--record-factory-pose` does not refuse while `rt/arm_sdk` samples arrive within 2 s,
   prints no `PART B ARMS: gap ...` line or the >0.10 rad warning; joints step does not store q17 explicitly in the report. Tests
   `test_factory_pose_record_refused_while_a_foreign_arm_sdk_writer_sends` [mutation] and
   `test_factory_pose_record_warns_when_it_differs_from_the_part_b_reading` missing.
 - R3-2: `factory_arm_summary` lacks the waist yaw (the waist column of the M4/M6 lookup needs it); test to rename/extend to
   `test_factory_arm_summary_lists_roll_yaw_wrist_and_waist`.
 - R3-6 net step: PC2 192.168.123.164 single-ping warning not implemented (check_net only checks the laptop IP).
 - R3-11: `test_publisher_guard_blocks_channel_publisher` calls the real `ChannelPublisher` / `DataWriter`; the contract wants
   `ChannelFactoryInitialize` and the writer classes replaced by fakes first, so a guard-less mutant fails on a fake, never on real DDS.
 Fix: implement the six items exactly as PLAN.md 3.1 and Track A action items 1-3 (rev 6.3) say, with their tests and mutation entries.

MAJOR 2 - the "newest arm-writer sidecar" rule (R2-2) is not proven; two fail-open mutants survive.
 Trigger (real Monday flow): 10:00 `86 ready` ends at weight 0; 11:00 `86 replay` is killed (exit_code null, arms latched at our pose);
 operator then runs `86 replay` again, which is refused for the factory-pose gap (exit 2, `arm_sdk_writer: false`); then
 `84 --record-factory-pose --overwrite-factory-pose`. The code today returns True (correct). But M2 (oldest sidecar decides) and M3
 (refused run counted) both make it return False and let OUR held pose be saved as the factory pose - and all tests still pass,
 because every test writes exactly one sidecar (tests/test_unit_real_io.py `write_sidecar`). A later refactor could break this
 unnoticed; `86 release` would then "release" to our own pose.
 Fix: one test (or a parametrized case) with three sidecars today - clean at weight 0, then unfinished, then a refused run with
 `arm_sdk_writer: false` -> `arms_may_be_held_by_us` True. Name e.g. `test_newest_arm_writer_run_decides_and_refused_runs_are_skipped`,
 Proves/Catches docstring; record M2 and M3 in `runs/real_a_mutation.log` as seen failing.

MAJOR 3 - the 84 preflight verdict (4.1 rule 2) has no test; staleness and foreign-writer mutants survive.
 `check_probe` (real_bringup.py:75-95): LowState older than 0.1 s -> FAIL (line 78), `rt/arm_sdk` samples -> FAIL "never start 86 while
 this happens" (line 85); `record_observation` stops on stale LowState (line 188). Mutants M4, M5, M7 all pass the suite. The
 foreign-arm_sdk FAIL is the operators' only read-only warning that someone else steers the arms before Part C; the task brief lists
 stale checks among the must-be-mutation-proven checks.
 Fix: two small unit tests on check_probe with plain dict inputs (no DDS): `test_probe_fails_on_stale_lowstate` and
 `test_probe_fails_on_foreign_arm_sdk_samples_unless_owner_motion` (each Proves/Catches), plus one on record_observation stopping
 when the fake LowState age exceeds 0.1 s; record the mutants as seen failing. (Plan fix: add these to Track A's test list.)

MINOR 4 - a first factory-pose record while our last run may hold the arms is accepted.
 `record_factory_pose` (real_bringup.py:108-125) consults `arms_may_be_held_by_us` only for `--overwrite` and for the conflict message.
 With no record yet today (or a new pose within 0.10 rad of the old one) and held == True it saves and PASSes - reproduced in
 `runs/real_a_review1_py310_smoke.log` ("first record while held -> PASS"). Mostly unreachable because 86 refuses robot runs without a
 record, but the R2-2 reason ("while our last run may still hold the arms, the measured pose is OUR pose") applies to every record.
 Fix: refuse every `--record-factory-pose` while held, same message; or state in 3.1 why the first record is exempt.

MINOR 5 - an unexpected exception in a step kills 84 without report or own-writer count, and loses the recording.
 `run_step` catches StepFailed, CameraError, TimeoutExpired, OSError (84:196). `PolicyServerError` (RuntimeError) and `ValueError` from
 `policy_wire` are not caught; in `step_record` the predictions run before the npz is written (84:165-171), so a bad `--policy` loses the
 owner-motion recording too. Fix: save the npz first, then ask the policy; catch `policy_wire.PolicyServerError, ValueError` as FAIL.

MINOR 6 - joint-limit check: untested and no measurement tolerance.
 M6 (`inside` always True) survives. On the robot a joint resting on its stop can read a hair past the MJCF range; `joints` then FAILs
 "stop, show the owner" and M0 cannot record, blocking Part C, with no documented way forward. Fix: one test that an out-of-range
 motor FAILs check_joints; plan: either a small named encoder tolerance or a runbook line for this case.

MINOR 7 - hand-state step checks freshness and values but not rate or NaN (Track A action 2: "rate and 6 values each, NaN-free").
 Fix: add rate from `HandReader.rate_hz()` and a NaN check in `check_hands`.

MINOR 8 - FrameStream never drains the grabber's stderr (real_camera.py:122, `stderr=PIPE` read only at close). If OpenCV/V4L2 or ssh
 write more than the pipe buffer (64 KB) during a long `record`, the grabber blocks and frames stop (record then FAILs "camera
 stopped"). Fix: drain stderr in a small thread into a bounded deque, or send it to a log file.

MINOR 9 - clean code nits.
 Unnamed numbers: `timeout_s=10, connect_wait_s=10` (84:165), `[-300:]` (84:90), `[-500:]` and `timeout=5` (real_camera.py), `ConnectTimeout=5`
 (explained in docstring - fine). `_GRABBER_CONSTANTS` builds source from `globals()` + `inspect.getsource` - explained by comments and
 tested for 3.8 syntax, acceptable, but it is the one spot a second-year student will need help with; one more sentence in the comment
 ("the robot PC has no copy of this repo, so the functions travel as text") would close it. `test_factory_pose_record_stores_mode_note`
 checks two behaviours (stores the note; refuses an empty note) - split into two tests. `struct.error` on a short header at EOF
 (real_camera.py `_read_one`) is not caught by `_read_loop` (prints a thread traceback; harmless, but noisy).

Readability overall: good. Functions are short with one job, constants carry source comments, docstrings say why (safety reason /
robot fact), every test has a Proves/Catches docstring and Arrange-Act-Assert layout. Files are under 300 lines.

Not run (keeps nothing open for round 1 beyond the majors): unit tests inside the Py 3.10 robot env (pytest not installed there; smoke
run instead); 84 against the loopback LowState publisher (pending integration, push-1 check 6.2, by plan); real robot (none connected).

GO/NO-GO: NO-GO - 3 majors (rev 6.3 items; two safety decisions that no test can catch breaking), 6 minors. Fix round needed.

## Track A - response (fix round 1, implementer, 2026-10-18 18:00-18:07)

All 3 majors and all 6 minors fixed; nothing refuted. Evidence (full output under runs/):
- Unit tests (sim env, Py 3.12): `runs/real_a_fix1_unit.log` -> 39 passed (was 23), exit 0.
- `84 --dry-run`: `runs/real_a_fix1_dryrun.log` exit 0. Project `.venv`: `runs/real_a_fix1_projvenv.log` -> 5 files skipped with a reason, no collection error.
- Operators' Py 3.10 robot env: `runs/real_a_fix1_py310_smoke.log` (dry run exit 0; RUN_LOG_DIR absolute; topic types LowCmd_/MotorCmds_; summary, Part B lines, net warning, guard). pytest still not installed there (not run).
- Mutation checks on a scratch copy (repo untouched), appended to `runs/real_a_mutation.log`: M2, M3, M4, M5, M6, M7 (the reviewer's survivors) and M9-M16 (new: relative RUN_LOG_DIR, record ignores foreign arm_sdk, first record allowed while held, guard installs nothing, guard patches only ChannelPublisher, hand topics read as LowCmd_, hand NaN check off, Part B warning dropped) - all 14 SEEN FAILING.
- Camera smoke: `runs/real_a_fix1_camera_smoke.log` (70 KB of grabber stderr no longer blocks; a short header ends with an error, no thread traceback).

Major 1 (rev 6.3 items):
- R3-5: `RUN_LOG_DIR = REPO/"runs"/"real"`, `BRINGUP_REPORT_DIR = REPO/"runs"` (real_factory_pose.py, re-exported by real_io with `REPO`); 84 writes its report to BRINGUP_REPORT_DIR. Test `test_sidecar_found_when_86_ran_from_another_cwd` [M9]. Note: in the repo run this test writes one sidecar `<today>_235959_test-cwd.json` into the real `runs/real/` for a few ms and deletes it in `finally` (needed to prove the real default path).
- R3-10 (c): `ARM_CMD_TOPICS`, `HAND_CMD_TOPICS`, `command_msg_type(topic)` (LowCmd_ / MotorCmds_, unknown topic -> ValueError); `SampleCounter` keeps `max_weight None` on hand topics; `active_writers` default = both tuples; `check_probe` warns on hand-command samples. Tests `test_active_writers_listens_on_hand_command_topics_with_their_own_type` [M14], `test_probe_warns_on_foreign_hand_commands`.
- R3-9: `earliest_factory_arms_today(report_dir, robot)`; joints step stores `q17` in the report; `record_factory_pose(..., writers, earliest)` refuses while rt/arm_sdk samples arrive (84 listens 2 s on rt/arm_sdk before recording), prints `PART B ARMS: gap ...` and the plan's WARNING word for word. Tests `test_factory_pose_record_refused_while_a_foreign_arm_sdk_writer_sends` [M10], `test_factory_pose_record_warns_when_it_differs_from_the_part_b_reading` [M16], `test_earliest_reading_ignores_sim_reports_on_the_robot`.
- R3-2: waist yaw in `factory_arm_summary`; test renamed `test_factory_arm_summary_lists_roll_yaw_wrist_and_waist`.
- R3-6: net step pings PC2 192.168.123.164 once (`ping -c 1`, 3 s timeout); no answer = PASS with a warning. Test `test_net_passes_with_a_warning_when_pc2_is_silent`.
- R3-11: guard test replaces `ChannelFactoryInitialize` (channel + real_io) and the three writer bindings with fakes, lists the bindings itself (so a shortened guard list fails, M13), and asserts no fake writer was created [M12, M13].
Major 2: `test_newest_arm_writer_run_decides_and_refused_runs_are_skipped` (10:00 clean, 11:00 killed, 12:00 refused) [M2, M3].
Major 3: `test_probe_fails_on_stale_lowstate` [M4], `test_probe_fails_on_foreign_arm_sdk_samples_unless_owner_motion` [M5], `test_record_stops_when_lowstate_goes_stale` [M7].
Minor 4: every `--record-factory-pose` is refused while `arms_may_be_held_by_us()` (code, not an exemption); the conflict message now offers only the not-held way out (the held case can no longer reach it). Test `test_factory_pose_first_record_refused_while_our_last_run_may_hold_the_arms` [M11].
Minor 5: the npz is saved before the policy is asked; a policy failure gives FAIL "recording is saved without predictions: <file>"; `run_step` also catches PolicyServerError and ValueError.
Minor 6: `test_joints_fail_when_a_motor_is_outside_its_limit` [M6]. The tolerance / runbook part is NOT done - see "For the planner".
Minor 7: `check_hands` takes `(msg, age, rate_hz)`: FAIL on non-6 or non-finite values, warning below `HAND_STATE_MIN_HZ = 50` (half of the service's 100 Hz, PLAN 2.2). Test `test_hand_state_fails_on_nan_values` [M15].
Minor 8: stderr drained by a thread into a bounded deque (50 lines).
Minor 9: named constants (POLICY_TIMEOUT_S, POLICY_CONNECT_WAIT_S, ERROR_TAIL_CHARS, PING_TIMEOUT_S, STDERR_TAIL_CHARS, CLOSE_WAIT_S, SSH_CONNECT_TIMEOUT_S), the "functions travel as text" sentence, mode-note test split in two, `struct.error` caught in `_read_loop`.
Test split (300-line rule): factory-pose tests moved to the new `tests/test_unit_real_factory_pose.py`, step verdicts to the new `tests/test_unit_real_bringup.py`, shared fakes to the new `tests/helpers/real_fakes.py` (added to the static no-publisher scan).

For the planner (not edited by me - PLAN.md is the planner's file and may be mid-edit):
1. Add the three new files above to the frontmatter / push-1 file list and to Track A's `<files>`.
2. Add to Track A's test list: the probe/record staleness and foreign-writer tests (major 3), the newest-run test (major 2), the first-record refusal (minor 4), joints-limit, hand NaN, net PC2 warning, hand-topic type, earliest-ignores-sim.
3. OPEN decision (minor 6): joint resting a hair past its MJCF limit. 84 and 86 both use `JOINT_LIMITS`, so an 84-only tolerance would just move the dead end to 86. Recommendation: a runbook line ("joints FAIL on one joint within ~0.02 rad past its limit while resting on its stop: do not record M0, show Finnegan") or one shared named tolerance in real_io used by 84 and 86 - the planner/user decides.
4. Track B2: `real_run.py:58` types `out: str = real_io.RUN_LOG_DIR`; it is now a `pathlib.Path` (as the contract says).

## Track A - round 2

Reviewer: independent code reviewer (did not write Track A, saw no author reasoning beyond the response section above). 2026-10-18 18:07-18:13.
Plan: `.planning/real-robot/PLAN.md` rev 6.3. Files: the 8 in Track A's `<files>` plus the fix round's new `tests/helpers/real_fakes.py`,
`tests/test_unit_real_factory_pose.py`, `tests/test_unit_real_bringup.py`.

### What I ran (all output in full under runs/)
- Unit tests, sim env (Py 3.12), the five Track A test files: `runs/real_a_review2_unit.log` -> 39 passed, exit 0 (MJCF test ran).
  Six more runs in a scratch copy, all 39 passed (no flaky test).
- `84 --dry-run`: `runs/real_a_review2_dryrun.log` exit 0.
- Operators' Py 3.10 robot env (`~/dev/g1-ext/robot-day-test/robot-env`): dry run + label check `runs/real_a_review2_py310.log` exit 0.
  NEW this round: the unit tests themselves under Py 3.10 (pytest put into a side folder `~/dev/g1-ext/review-pytest310` via
  `uv pip install --target`, the robot env itself is untouched): `runs/real_a_review2_py310_unit.log` -> 38 passed, 1 failed (minor 4).
- Reviewer mutation checks, scratch copy, repo untouched, run with `-B` / `PYTHONDONTWRITEBYTECODE=1`: `runs/real_a_review2_mutation.log`.
  Round-1 survivors now caught: M2 oldest sidecar decides, M3 refused runs counted, M4 staleness 0.1 -> 10 s, M5 probe foreign
  arm_sdk FAIL off, M6 joint `inside` always True, M7 record stale stop removed, M11 first record allowed while held - all SEEN FAILING.
  New probes: R3 weight `> 0.0 -> > 0.5` caught; R8 earliest reading ignores the robot flag caught.
  Survivors: R1 staleness 0.1 -> 0.15 s; R2 unreadable sidecar counted as "free"; R4 stale hand state accepted; R5 record ignores a
  stopped camera; R6 84 joints step skips the rt/arm_sdk listen before the factory-pose record; R7 lower joint limit not checked.
  Method note: my first pass gave false "CAUGHT" lines because a same-size mutant (`> 0.0` -> `> 0.5`) restored within the same
  second left its .pyc in __pycache__, so later mutants ran against the old bytecode. Mutation runs must use `-B` and delete
  __pycache__ (process note for every track; I re-ran the important ones cleanly, results above).

### Verdict 1 - implementation fidelity: YES
All round-1 majors are closed and I re-derived each one: R3-5 (anchored RUN_LOG_DIR / BRINGUP_REPORT_DIR, 84 report dir), R3-10 (c)
(hand-command topics with MotorCmds_, `max_weight` None, probe warning), R3-9 (2 s rt/arm_sdk listen + refusal, PART B line and the
plan's WARNING word for word, q17 in the report), R3-2 (waist yaw), R3-6 (PC2 ping as a warning), R3-11 (guard test on fakes, own
binding list). 86's sidecar name (`real_run_log.py:44`, `<YYYYmmdd_HHMMSS>_<mode>.json`, temp file `*.json.<pid>.part`) matches 84's
reader regex, so 84 sees 86's sidecars. All 6 round-1 minors fixed as described. One small contract deviation remains (minor 3).

### Verdict 2 - plan correctness for real hardware: right, with one push-blocking gap
The plan's push list is still missing the fix round's three new files (major 1, planner's file). The rest are minors.

### Findings

MAJOR 1 (plan, owner: planner) - push list misses three files four Track A tests depend on.
 PLAN.md frontmatter (lines 11-19, "Push 1 files: Track A and E1 files (frontmatter)", 6.2 line 1000) and Track A `<files>` still list
 only the original 8 files. `tests/test_unit_real_io.py`, `test_unit_no_publisher.py`, `test_unit_real_factory_pose.py`,
 `test_unit_real_bringup.py` all `from helpers.real_fakes import ...` (grep). A push that follows the frontmatter ships the first two
 without `tests/helpers/real_fakes.py` -> ModuleNotFoundError at collection (importorskip does not cover it), and drops the two new
 test files (the probe/stale/newest-sidecar/foreign-writer tests) entirely. Fix: add the three files to the frontmatter, Track A
 `<files>` and the test list (implementer's "For the planner" items 1-2). Track A's code needs no change for this.

MINOR 2 - the 84-level wiring of the R3-9 refusal is not proven (mutant R6 survives).
 `scripts/84_real_bringup.py:127` listens on rt/arm_sdk and passes `writers` into `record_factory_pose`. Replacing that line with
 `writers = {}` passes all 39 tests: the function-level refusal is proven, the step that feeds it is not, and no integration test
 covers it (Track C's F8 records BEFORE starting its foreign writer, PLAN R3-9 row). Fix: one small test of `step_joints` with
 `real_io.active_writers` monkeypatched to report 40 rt/arm_sdk samples and a fake LowState reader -> FAIL "another program steers
 the arms" and no file written (Proves/Catches).

MINOR 3 - `owner_motion` label: exact match where the plan says "starts with" (fails closed).
 `real_bringup.py:97` and `84:192` use `label != OWNER_MOTION_LABEL`. PLAN 4.1 rule 1 / Track A action 2 say "unless the label starts
 with owner_motion" with the example `"owner_motion arm7 example, R2+A"`, and section 7 (line 1064) tells operators to pass
 `--label "owner_motion <tool>, <mode>"`. Reproduced in Py 3.10 (`runs/real_a_review2_py310.log`): that label FAILs the probe ("never
 start 86") and marks the owner-motion recording FAIL. Safe direction (the runbook today says plain `--label owner_motion`), but a
 false abort in Part B step 4. Fix: `label.startswith(OWNER_MOTION_LABEL)` in both places, plus one parametrized case with the
 plan's example label.

MINOR 4 - `test_joint_limits_match_the_mjcf` fails in the robot env when the MJCF exists but mujoco is not installed.
 It skips only on a missing file (`test_unit_real_io.py:47`) and then does `import mujoco`. Under the Py 3.10 robot env on this Mac:
 `ModuleNotFoundError: No module named 'mujoco'` (`runs/real_a_review2_py310_unit.log`). Fix: `pytest.importorskip("mujoco")` at
 the top of the test.

MINOR 5 - fail-closed branches the docstrings promise but no test pins (mutants R2, R4, R5, R7 survive).
 R2: `_newest_arm_writer_sidecar` returning `{"unreadable": True}` (real_factory_pose.py:126) is the documented safety reason
 ("guessing free could let a held pose be recorded") - turning it into `continue` passes. R4: stale hand state accepted
 (real_bringup.py:179). R5: recording keeps a frozen camera frame (real_bringup.py:233). R7: lower joint limit unchecked
 (real_io.py:145). Fix: R2 deserves one test (a newest sidecar with broken JSON -> `arms_may_be_held_by_us` True); R4, R5, R7 can be
 one extra parametrized case each in the existing tests, or accepted as documented gaps - none of them is a motion check.

MINOR 6 (plan) - `arms_may_be_held_by_us` mixes sim and robot sidecars.
 86 writes `"source": "sim85"` on loopback, `"real"` on the robot (`real_run.py:87-88`) into the same RUN_LOG_DIR; the reader ignores
 it. A newer clean sim run masks an older killed robot run (fail open), and a killed sim run blocks the robot M0 with "run 86 release
 first", while 86 release on the robot needs a robot record that does not exist yet (dead end). Reachable only if 86 is run on
 loopback on the robot laptop on the same day (no runbook step does that today). Fix: filter by `source` the way
 `earliest_factory_arms_today` filters by iface.

MINOR 7 - `test_sidecar_found_when_86_ran_from_another_cwd` writes into the real `runs/real/`.
 If the test is killed hard between write and `finally`, an `exit_code: null` sidecar named `<today>_235959_test-cwd.json` stays and,
 sorting newest all day, blocks every M0 record with a misleading "run 86 release first". Fix: prove the same thing without touching
 the real folder: `assert real_io.RUN_LOG_DIR.is_absolute() and real_io.RUN_LOG_DIR == real_io.REPO / "runs" / "real"` (mutant M9,
 relative path, still fails), keeping the cwd test only if it uses `tmp_path`.

MINOR 8 (plan, still OPEN for planner/user) - joint resting a hair past its MJCF limit: no tolerance and no runbook line yet
 (round-1 minor 6, implementer's item 3). Not blocking Track A; the decision belongs in PLAN 3.1 / runbook before Monday.

MINOR 9 - clean-code nits.
 `old['recorded_at'][11:16]` (real_factory_pose.py:55) is an unexplained slice (HH:MM of an ISO time - name it or use
 `datetime.fromisoformat(...).strftime("%H:%M")`); `silent_writers()` in test_unit_real_bringup.py has no docstring; R1 survives because
 the stale test uses 0.2 s - an age of 0.11 s would pin the 0.1 s limit itself.

Readability: a second-year student can read and explain every function and test. Functions are short with one job, every constant
carries its source, all 37 test functions have Proves/Catches docstrings and Arrange-Act-Assert layout, all files are under 300 lines.
`_GRABBER_CONSTANTS` now explains why the functions travel as text.

Not run: 84 against the loopback LowState publisher and `own_writers == 0` live (pending by plan, push-1 check 6.2); real robot (none
connected). Neither is a condition of this code gate.

GO/NO-GO: GO for Track A's code (fidelity yes, 0 code majors, 8 minors). Condition before push 1: the planner fixes MAJOR 1 (push
list / frontmatter); minors 2-4 are cheap and recommended before push 1, the rest can follow.

## Track A - follow-up (push-1 check)

Implementer, 2026-10-18 18:20-18:40, from the main session's push-1 check (PLAN 6.2). Loopback (lo0) only; no MuJoCo, GPU, policy
server or robot. All logs in full under runs/ (gitignored, local to this laptop).

### 1. Intermittent segfault of 84 (exit 139) - root cause in cyclonedds 0.10.2 + the SDK, triggered by how our code drops readers

Where it crashes: `PYTHONUNBUFFERED=1 python -X faulthandler 84 ...` against the loopback publisher, old code: 6 of 11 runs exit 139
(runs/real_a_fu_diag_repeat.log, runs/real_a_fu_diag_run*.log). Five crash during the record step (main thread asleep in
`record_observation`'s 30 fps sleep, camera threads blocked in pipe reads, i.e. no Python thread running Python code), one crashes
at interpreter teardown right after `REPORT ...` (run8: exit 139 with a PASS report on disk). The faulting thread has no Python frame:
a DDS receive thread.

Why: every unitree_sdk2py reader channel sits in a reference cycle (DataReader -> Listener -> bound `__OnDataAvailable` -> SDK
`__Reader` -> DataReader; our `LatestReader` -> channel -> handler -> `LatestReader` adds another). 84 drops readers after use (the two
`HandReader`s of `step_hands`, the `SampleCounter`s of `active_writers`, the record step's readers), so only Python's cyclic garbage
collector frees them, at a random later moment. The collector clears weak references before it runs `__del__`; cyclonedds'
`Entity.__del__` looks its handle up in a `WeakValueDictionary`, finds it gone and returns WITHOUT `dds_delete`. The C reader stays
subscribed, the Python callback object is freed, and the next sample makes the receive thread jump into freed memory.
Deterministic proof (scratch probe: create a HandReader, `del`, `gc.collect()`, keep listening): old code 3 of 3 exit 139
(runs/real_a_fu_drop_reader_old1-3.log; "Current thread ... <no Python frame>"); with a logging wrapper the DataReader's `__del__`
prints "ref in Entity._entities = False" just before the crash (runs/real_a_fu_drop_reader_why.log). Teardown is the same mechanism:
interpreter shutdown frees the remaining readers through the collector while the publisher keeps sending.

Fix (g1sim/real_io.py):
- `subscribe_topic` keeps every reader channel in `_open_channels` until the process ends, so no reader is ever half-collected.
- `close_dds()` deletes the SDK's DomainParticipant (and with it every reader) via `Entity.__del__` = dds_delete; `init_dds`
  registers it with `atexit`, which runs before interpreter teardown while Python can still serve a callback. cyclonedds loads its
  library with `ctypes.CDLL`, which releases the GIL, so dds_delete can wait for a running callback without a deadlock.
  This also protects 86 and 89, which use the same readers: 86's preflight `active_writers` counters on rt/arm_sdk would otherwise
  be collected later while 86 itself sends on rt/arm_sdk - a segfault in the middle of a motion run.
Operator robustness (scripts/84_real_bringup.py `__main__`): `faulthandler.enable()` and `sys.stdout.reconfigure(line_buffering=True)`,
so a crash leaves every STEP line printed so far plus a Python trace even with stdout in a file (in `__main__`, not in `main()`,
because pytest's captured streams have no file descriptor). The report is written before teardown and close_dds runs after it, so a
teardown problem can no longer hide a written verdict.

Evidence after the fix: scratch probe 3 of 3 exit 0, "gc.collect() freed 0 objects ... NO CRASH" (runs/real_a_fu_drop_reader_new1-3.log);
the exact push-1 command (plain `python`, stdout to a file) 12 of 12 exit 0 (runs/real_a_followup_84_repeat.log, logs
runs/real_a_followup_84_run1-12.log) and 6 of 6 more on the final file (runs/real_a_followup_84b_repeat.log); crash trail: `kill -SEGV`
of an 84 run after 20 s leaves the probe/joints/hand-state/camera STEP lines and the faulthandler trace, exit 139
(runs/real_a_followup_crashtrail.log).

### 2. Hand-state rate - 84's number was wrong (and the publisher is a bit slow too)

84 printed "slow hand state (left 4 Hz, right 4 Hz)". `step_hands` creates fresh HandReaders and read `rate_hz()` right after the first
sample arrived; `rate_hz` divided the samples of a 1 s window by the full 1 s although the reader had listened ~50 ms. Independent
count with the same reader after 3 s of listening: 78 Hz both hands, LowState 200 Hz (runs/real_a_fu_hand_rate_probe.log: "rate_hz
then (old 84 path): 4.0 Hz", "156 samples in last 2.00 s = 77.7 Hz"). On the robot this would have shown a healthy 100 Hz hand
service as "slow".
Fix: `LatestReader.rate_hz` = gaps between the samples in the window / time from the first of them until now (fewer than 2 samples
give 0, a stopped stream reads lower and lower); `step_hands` waits for both hands, listens `HAND_RATE_LISTEN_S` = 1 s, then reads.
84 now reports 80.6 Hz for the loopback hands (runs/real_bringup_20261018_182707.json) and no "slow" warning in 18 runs.
Test: `test_rate_hz_of_a_young_reader_is_the_stream_rate` (0.2 s and 2.2 s of a 100 Hz stream, fake clock) - fails on the old code
with 20 Hz instead of 100 (runs/real_a_fu_new_tests_on_old_code.log). `test_a_dropped_reader_stays_subscribed` pins fix 1 and also
fails on the old code (same log).
Publisher (Track E1's file, not edited): tests/helpers/loopback_lowstate_pub.py `run_loop` schedules the hands with
`next_hands = now + 1.0 / HAND_HZ`; `now` is the start of a 5 ms LowState tick, so a slightly early tick misses its slot and the hands
slip to the next tick: 78 Hz, not the 100 Hz it prints. One-line fix: `next_hands += 1.0 / HAND_HZ` (deadline scheduling, as
`next_tick` already does). Harmless for 89 and 84 (warning bound 50 Hz), but the printed "100 Hz" is not true.

### 3. Minor (round 2): mujoco skip

`test_joint_limits_match_the_mjcf` now uses `pytest.importorskip("mujoco", reason="mujoco is not installed (the operators' robot env
has no MuJoCo)")`. With mujoco blocked: "1 skipped" with that reason (runs/real_a_followup_no_mujoco.log).

### Verify

- `~/dev/g1-ext/.venv/bin/python -m pytest tests/test_unit_real_io.py tests/test_unit_real_camera.py tests/test_unit_no_publisher.py
  tests/test_unit_real_factory_pose.py tests/test_unit_real_bringup.py -q` -> 41 passed (runs/real_a_followup_unit.log).
- Neighbouring Track B/C tests that import real_io (real_run, real_safety, real_safety_filter, real_timing, motion_gate) -> 68 passed
  (runs/real_a_followup_other_real_tests.log).
- 84 push-1 command x12 -> all exit 0, then x6 -> all exit 0 (see above).

Files changed: g1sim/real_io.py (307 lines, was 286), scripts/84_real_bringup.py, tests/test_unit_real_io.py. Not changed: runbook,
PLAN.md, the loopback publisher.

## Track A - follow-up review

Reviewer (independent; did not write the follow-up, saw only the diff, the follow-up notes above and the code), 2026-10-18
18:38-18:52. Loopback (lo0) only; no robot, MuJoCo, GPU or policy server. Scope: `git diff HEAD` of g1sim/real_io.py,
scripts/84_real_bringup.py, tests/test_unit_real_io.py (real_bringup.py, real_camera.py, real_factory_pose.py: unchanged).

### What I ran (all output in full under runs/, gitignored)
- Track A unit tests, .venv Py 3.12: 41 passed (`runs/real_a_followup_review_unit.log`). Operators' Py 3.10 robot env (side
  pytest `~/dev/g1-ext/review-pytest310`): 40 passed, 1 skipped "mujoco is not installed ..." (`runs/real_a_followup_review_unit_py310.log`)
  - round-2 minor 4 is fixed in the env it was found in. Mujoco blocked under 3.12: 1 skipped (`runs/real_a_followup_review_nomujoco.log`).
- 84 push-1 command x10 against `loopback_lowstate_pub.py --hands` (publisher given 900 s instead of 400 so one publisher covers all
  10 runs): **10/10 exit 0**, every run all five STEPs PASS and `OWN WRITERS: 0`, no faulthandler trace, no stray SDK line
  (`runs/real_a_followup_review_84.log`, per-run `..._84_run1-10.log`, `..._84_summary.log`). Publisher killed afterwards (ps: none left).
- Root cause, my own probe (create a `real_io.HandReader`, wait for a sample, `del`, `gc.collect()`, listen 4 s):
  code as is 3/3 exit 0 "NO CRASH"; same code with `_open_channels.append` disabled 3/3 **exit 139**, faulthandler shows only the
  main thread asleep, i.e. the fault is in a non-Python (DDS) thread (`runs/real_a_followup_review_live_probes.log`,
  `..._drop_keep1-3.log`, `..._drop_nokeep1-3.log`). Pure-Python check of the mechanism in Py 3.10: an object in a
  `WeakValueDictionary` freed by refcount sees itself in the dict in `__del__`; freed by the cyclic GC it does not. cyclonedds 0.10.2
  `core.py:189-194` `Entity.__del__` returns without `dds_delete` exactly in that case, and its own `__init__` docstring
  (core.py:170-173) names the resulting "C calls into freed memory -> segfault". The SDK's `Channel.__Reader.Init` builds
  `DataReader(..., Listener(on_data_available=self.__OnDataAvailable))` = a reference cycle; `CloseReader` only `del`s a name and
  does not break it. Root cause **confirmed**, and the fix removes the trigger (no reader is ever collected while DDS runs) instead of
  hiding the symptom.
- `close_dds`: cyclonedds 0.10.2 has no `close()`; `__del__` is the only delete. The library is loaded with `ct.CDLL`
  (internal.py:34), so `dds_delete` releases the GIL and a running callback can finish. atexit handlers run before the interpreter is
  marked finalizing, so DDS threads can still take the GIL then. No other atexit user in g1sim/scripts 7x-8x; 86 does not rely on
  atexit for its ramp-out.
- Hand rate: A/B by accident. My 10-run loop used a publisher started 18:39 from the OLD `loopback_lowstate_pub.py`; at 18:43 someone
  outside this follow-up changed it to `next_hands += 1.0 / HAND_HZ` (working tree, `git diff HEAD -- tests/helpers/`). Old publisher:
  84 reported 77.1-82.0 Hz both hands (`runs/real_a_followup_review_hand_rates.log`), matching the implementer's independent 78 Hz.
  New publisher: an independent plain-SDK counter in its own process 100-101 samples per second for 34 s WHILE an 84 run was going,
  and that 84 run reported 99.5 Hz; `HandReader` alone with 84's timing 99.4 Hz after 1 s, 100.0 after 6 s
  (`runs/real_a_followup_review_rate_probes.log`, `runs/real_bringup_20261018_184926.json`). So 84's number now equals the stream rate.
- Mutations on a scratch copy, Track A unit tests (`runs/real_a_followup_review_mutations.log`): keep-alive removed CAUGHT; old
  count/window rate CAUGHT; rate divided by last-first sample SURVIVES; `atexit.register(close_dds)` removed SURVIVES; `close_dds` body
  removed SURVIVES; 84 `time.sleep(HAND_RATE_LISTEN_S)` removed SURVIVES. Live: 84 with `close_dds` never registered 6/6 exit 0
  (`runs/real_a_followup_review_noclose_run1-6.log`).
- Adjacent: 71_real_probe.py (plain `ChannelSubscriber`s, collected at teardown) x15: 15/15 exit 0 (`runs/real_a_followup_review_71x15.log`).

### Verdict 1 - implementation fidelity: YES
All three claimed fixes are in the diff and behave as described: readers are kept in `_open_channels` and the participant is deleted at
exit; `rate_hz` = (n-1)/(now - first sample in window) and 84 listens 1 s before reading it; the mujoco test uses `importorskip`.
`faulthandler.enable()` + line-buffered stdout are in `__main__` only. The publisher guard is untouched and still first in `main()`;
84 creates zero writers (OWN WRITERS 0 in 17/17 of my runs). The two new tests have accurate Proves/Catches docstrings, AAA layout,
and both fail on the old code (keep-alive and count/window mutants caught).

### Verdict 2 - plan / approach correctness: right
Keeping every reader for the life of the process is the correct fix for this cyclonedds 0.10.2 + SDK combination (the only alternative,
explicitly breaking the SDK's private cycle per reader, is more fragile). The cost is bounded: 84 opens about 11 readers; in 86 the
four preflight `SampleCounter`s keep receiving during the run, including 86's own rt/arm_sdk at 50 Hz - LowCmd_ deserialisation
measured 107 us = 0.5 % of one core (`runs/real_a_followup_review_lowcmd_deser.log`): acceptable, no finding. The rate formula is
right for a steady stream and never over-reports beyond the mean inter-sample rate of the window. On the real robot it will report
what arrives at the laptop; the 100 Hz brainco_hand_service rate itself (PLAN 2.2) is still unverified on hardware and stays a
warn-only check (50 Hz bound), so it cannot block or wrongly pass anything that moves.

### Findings (0 major, 4 minor, nits)

MINOR 1 - `close_dds` / its atexit registration is not pinned by any check.
 Mutants "atexit.register removed" and "close_dds body removed" pass all 41 tests, and 84 without close_dds still exited 0 in 6/6
 live runs (the teardown crash it targets was 1 of 11 in the old-code diagnosis). So its benefit is plausible but unproven, and a later
 edit can drop it silently. It is low-risk to keep (17/17 runs with it exit 0). Fix: one unit test with a fake participant: `init_dds`
 registers `close_dds` exactly once (monkeypatch `real_io.atexit`) and `close_dds` deletes the participant - or record it as a
 documented gap.

MINOR 2 - `rate_hz`: "a stream that stopped reads lower and lower" is not pinned, and few close samples over-report.
 Mutant "divide by (last - first)" survives; with it a stream that stopped 0.9 s ago keeps its full rate. Separately, with the
 shipped formula a window holding only a few closely spaced samples reads high (2 samples 1 ms apart read at once = about 1000 Hz).
 No caller reads that early today (84 listens 1 s, 86 preflight 2 s, 89 2 s), but 86's preflight "refuse below 200 Hz" relies on
 this function, so it should be pinned. Fix: one more case in `test_rate_hz_of_a_young_reader_is_the_stream_rate` style - 1 s of
 100 Hz, then 0.5 s silence -> about 50 Hz; optionally return 0 when the span is below a fraction of `window_s`.

MINOR 3 - 84's half of the hand-rate fix has no test.
 Removing `time.sleep(HAND_RATE_LISTEN_S)` from `step_hands` passes every test; only a loopback run would show it, and only as
 warning text inside a PASS line. Fix: a `step_hands` test with fake readers and a fake clock (pattern of the R6 `step_joints` test
 suggested in round 2) asserting the rate is read after the listen window.

MINOR 4 - a stray SDK line can appear after REPORT.
 Implementer run `runs/real_a_followup_84b_run5.log` (1 of 18; 0 of 17 in mine) ends with
 `[Reader] catch DDSException error. msg: Occurred while taking data in <Entity, type=cyclonedds.sub.DataReader ...>`:
 a callback that was running while `close_dds` deleted the participant. Harmless (the SDK catches it, exit 0, report already
 written), but it looks like an error to an operator on robot day. Fix: one runbook sentence ("a `[Reader] catch DDSException` line
 after REPORT is harmless") or mention it in close_dds's docstring.

NITS - code comments/docstrings cite `runs/real_a_fu_drop_reader_why.log`, a gitignored log that exists only on this laptop (cite the
 review file section instead); `rate_hz`'s docstring narrates history ("84 said 4 Hz for a 78 Hz hand stream") that belongs in the
 test's Catches line, where it already is; real_io.py grew from 286 to 307 lines against PLAN's "~300 lines (split, do not grow)".
 Out of this follow-up's scope: `tests/helpers/loopback_lowstate_pub.py` was changed at 18:43 by the runbook RV-1..RV-8 fix
 (STATUS 18:47; the implementer's suggested one-liner); it now really publishes the hands at 100 Hz. Its review is that gate's job.

Not run: real robot / real brainco_hand_service rate (forbidden here; pending robot day, warn-only check). Nothing else open.

GO/NO-GO: **GO** for the Track A follow-up (fidelity yes, approach right, 0 majors). Minors 1-3 are cheap test additions,
recommended before push 1; minor 4 and the nits can follow.
