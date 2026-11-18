# Review: robot-day model switch run1 40k -> run3 150k (uncommitted diff, 2026-11-16)

Reviewer: fresh-context reviewer (opus). Did not author the change and saw no author reasoning.
Scope: `git diff` in the canonical repo (6 files: real_safety.py, 89, runbook, solo page, two tests) plus the day-path
consumers. Read-only. No DDS, sim or policy server started.

## Verdict 1: did the code do what the spec said? YES

| Spec item | Result | Evidence |
|---|---|---|
| 1. ENVELOPE_LO/HI = elementwise min/max over v1+v2 action stats -/+ 0.10, outward to 1e-4, hands unchanged | done | Re-derived from `~/dev/g1-ext/datasets/g1_volleyball_walkup_v{1,2}/meta/stats.json` (27-D each) with floor/ceil at 1e-4: all 15 non-hand entries in LO and HI match the code exactly. Hands (indices 14-25) are bit-identical to HEAD. The new envelope contains the old one on every index. Widest growth: HI +0.2455 (right wrist pitch, idx 12), +0.2309 (left wrist pitch, idx 5), LO -0.082. |
| 2. Envelope contains every action of `tests/data/replay_demo.npz` | done | All 726x27 actions inside, smallest slack exactly 0.100 rad (the demo sits at the v1 extremes). The states are inside too. |
| 3. 89 loads `smolvla-runs/run3/checkpoints/150000/pretrained_model` | done | `scripts/89_robot_day_selftest.py:61`. The 89 run at 16:20 today (`runs/robot_day_selftest_20261116_162008.json`, `runs/robot_day_selftest_policy_mps.log`) passed every check. The log shows run3's config (`Device 'cuda' is not available. Switching to 'mps'`), `chunk_ms 508` and `SELF-TEST PASS`. |
| 4. Runbook and solo page: 81 lines point at run3, solo page re-dated, unlock date = run day, MOTION.json untouched | done | Runbook :624-625 and solo :166 use the run3 path. The solo page uses 2026-11-18 in the title, unlock JSON, printf recipe, commit message, gate message and backup folder. `g1sim/motion_gate.py:92-94` compares `date` to `datetime.date.today().isoformat()` and the hint text `only Finnegan can unlock` (:27). The documented line `the unlock is for 2026-11-18; only Finnegan can unlock today` matches that code. 2026-11-18 is a Wednesday. `docs/ROBOT_DAY_MOTION.json` does not exist and is not in the diff. No `2026-11-11` is left in the runbook. |
| 5. Envelope test uses the v1+v2 union; e2e CKPT can be overridden with G1_E2E_CKPT (default unchanged); FAKE_CAMERA path shell-quoted | done | `tests/test_unit_real_safety.py:306-319` ran and passed (not skipped, `-rs`). `tests/test_e2e_real_adapter_sim.py:37-38` keeps the run1 default. `:40` uses `shlex.quote`, and `g1sim/real_camera.py:119` splits with `shlex.split`. |

Unit suite: `469 passed, 2 skipped, 148 deselected in 63.83s`, exit 0. The model file's sha256 is
`ef86fffee140e065e0e19625e125af3dc1c1a9d4d0ae1384983c90d5221db931`, which matches the `ef86fffe...db931` quoted on the solo page.
Comparing the run1 and run3 checkpoints: `policy_postprocessor.json` is identical. The configs differ only in device (cuda),
`pretrained_path` (a Hive PC path) and decay steps. 81 loads with `from_pretrained(path)` and overrides the device
(`scripts/81_smolvla_server.py:93-98`), and the 89 run proves that this load works offline on MPS.

## Verdict 2: was the spec right? MOSTLY. Some day-path scope is missing (M1, M2) and one safety-coverage question is not addressed (M3)

## Findings

**M1 (Medium, docs): the solo page says run3 is not used.**
- Where: `docs/ROBOT_DAY_2026-11-11_SOLO.md:23-24`, which reads "**Will not:** ... train anything, use run3."
- Why it matters: the same page now says "Model: **run3 150k**" at :13 and serves run3 at :166. This is the one page the solo
  operator works from, and it contradicts itself on which model runs.
- Fix: delete ", use run3" (or change it to "use run1").

**M2 (Low, docs): stale run1 success figure on the solo page.**
- Where: `docs/ROBOT_DAY_2026-11-11_SOLO.md:27-28`, which reads "the policy has only ever seen the simulator (72 % in the sim, 36 of 50 starts)".
- Why it matters: that is run1's number. :13 now gives run3's 68/100, and the operator is told to say this line to the owner.

**M3 (Medium, safety coverage, needs your decision): S1 and 87 never checked the wider policy poses.**
- 87 does not read the envelope. Its "way out" and lift_home rows from policy poses
  (`scripts/87_arm_path_clearance.py` `policy_exits`/`--waypoint-paths`, runbook S1 at :421 with `--demo tests/data/replay_demo.npz`)
  use the frames of the single v1 replay demo as stand-ins for "where the policy can be".
  `.planning/real-robot/SAFE-PATHS.md:16,120` records the result: 5265/5265 pass at waist 0.6, worst margin +0.06 cm above the 5 cm rule.
- The fence guards only the policy phase (`g1sim/real_safety.py:32`, `FENCE_PHASES = ("policy",)`). The scripted exits from
  a policy pose (soft exit and recover, the way out) are covered only by those 87 rows.
- With run3 the envelope now lets the policy reach up to 0.23-0.25 rad more wrist pitch (idx 5, 12), plus more wrist roll
  (idx 4: 0.18 -> 0.27) and more shoulder roll and waist yaw. The worst margin was 0.06 cm, so a start pose outside the
  demo's frames could drop below the 5 cm rule on the way out.
- This gap already existed with run1, because one demo never stood in for the whole v1 range. The wider envelope makes it larger.
- Nothing in the diff is wrong. The point is that the claim "S1: clear" covers less of what run3 can do.
- Options:
  - (a) Accept it as the same class of gap as before.
  - (b) Before M6, run 87 `--waypoint-paths` with `--demo` set to an npz of run3 sim rollout states (any npz with a `state` key works).
  - (c) Let M6 at the table rely on the fence plus the owner's e-stop.
- Not a blocker for this diff.

**L1 (Low): 89 messages still point at the run1 release.**
- Where: `scripts/89_robot_day_selftest.py:176` (docstring "The released 40k checkpoint") and `:179-180`.
- Why it matters: if the run3 directory is missing on the Mac, the FAIL message says to run
  `setup_robot_laptop.sh --release robot-day-2026-10-18`. That command installs run1 into `smolvla-runs/run1/...`
  (`scripts/setup_robot_laptop.sh:21`), so it would not fix that FAIL.
- On the Mac the run3 directory exists today, so this only matters if it gets lost. Fix the message to name the run3
  copy (scp from the Hive PC).

**L2 (Low, info): the camera colour warning still compares against v1 image stats.**
- Where: `g1sim/real_camera.py:36`, read by `g1sim/real_bringup.py:213`.
- The reference is still v1's image mean (0.378, 0.479, 0.551). v2's mean is (0.495, 0.552, 0.581), a gap of up to 0.117.
- It only produces a warning (tolerance 0.15, still PASS). It may now warn, or stay silent, for the wrong reason. No action needed for the day.

**L3 (Low, info): the runbook's Ubuntu 81 line points at a model that only the Mac has.**
- Where: `docs/ROBOT_DAY_RUNBOOK.md:625`.
- The Ubuntu/cuda line now uses the run3 path, but teammate laptops only get run1 from the setup script.
- That is accepted per the spec (teammates do not serve the model), so this is mentioned only for completeness.

**L4 (Low, test gap, no fix round): no test pins spec item 2.**
- No unit test checks that `replay_demo.npz` stays inside the envelope. I checked it by hand (slack 0.100).
- Cheap to add, but under the "test-only gaps" rule it does not justify a fix round.

**L5 (Low, process): LEARNING_PATH and PROGRESS are not updated.**
- `docs/LEARNING_PATH.md` and `PROGRESS.md` are not in the diff. CLAUDE.md rule 8 puts LEARNING_PATH in every commit proposal.

**Other consumers checked, no finding:**
- `SafetyFilter._apply_envelope` (`g1sim/real_safety.py:335-347`): clips less with the wider envelope.
- The `envelope_streak` soft exit (`g1sim/real_run.py:284-285`): fires less.
- The M5 shadow abort rule ">5 % envelope-clipped" (`g1sim/real_run_policy.py:190-208`, runbook :555-557): easier to pass.
  This is intended, because run3 was trained on the v2 range.
- Joint limits, the fence (policy phase), the rate limit and the lead monitor still run after the envelope, in the same order.
- `g1sim/fence.py` and 87 do not import the envelope.
- `scripts/setup_robot_laptop.sh` still installing run1 does not touch anything on the Mac's day path. 89, 81 and 86 all read the run3 path.

## Checks I could not run
- e2e `test_e2e_real_adapter_sim.py` with `G1_E2E_CKPT` set to run3 (it needs the sim and 81, which this review was told not to start).
  I checked the quoted FAKE_CAMERA and the override only by reading the code.
- 87 S1 with run3-like start poses (see M3).

## Overall: GO
The diff does what the spec says, the numbers re-derive exactly, the tests pass, and 89 passed with run3 on the Mac.
Fix M1 (one-line doc contradiction), and preferably M2 and L1, before the commit. They are text-only and need no re-review.
M3 is your decision before M6 at the table.

## Applied before commit (main session, 2026-11-16)
- M1, M2, L1 fixed (solo page: "will not use run1", run3 numbers for the owner; 89 docstring + missing-model hint name run3).
- M3 open, ruling for Finnegan: run 87 exits from run3 rollout poses before M6 (needs a state dump from 82; none stored today).
- E2E note: 86 run with run3 against 85 over DDS was run by hand (60 s live, exit 0, DONE, no envelope clip); the pytest e2e fails only on its own 120 s sim budget.
