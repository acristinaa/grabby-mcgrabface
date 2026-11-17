# Track E2 - round 1

Reviewer: independent code reviewer (did not author E2, saw no author reasoning). Finished 2026-10-18 18:22.
Scope: docs/ROBOT_DAY_RUNBOOK.md Part C, scripts/89_robot_day_selftest.py (motion step), docs/robot_day/volleyball_natural_phases.png,
tests/test_unit_runbook.py (two new tests + commands_in/runbook_section helpers). Plan: section 5 task E2, section 7 Part C, 4.2 rule 16, 3.5.

## Evidence (all re-run by the reviewer)
- `.venv/bin/python -m pytest tests/test_unit_runbook.py -v -rs` -> runs/real_e2_review_r1_unit.log: exit 0, 11 passed, 8 skipped
  (clearance test: report absent, as planned; 7 helper tests: no unitree_sdk2py in .venv). The flag test now covers 86_real_run.py too.
- `git check-ignore -v docs/robot_day/volleyball_natural_phases.png` exit 1 (not ignored, wanted); byte-identical to docs/volleyball_natural_phases.png.
- Reviewer mutations against the clearance oracle with a synthetic 87 report (scratchpad e2_review_mut.py) -> runs/real_e2_review_r1_mutation.log:
  correct table PASSED; MUTANT "M6 yes beyond setback_max_policy_m" FAILED (caught). Author's 5 mutations (runs/real_e2_mutation.log) consistent.
- 89 motion step in a real robot env (~/dev/g1-ext/robot-day-test2/robot-env): `86 gate` exit 2, prints `MOTION: LOCKED` +
  `docs/ROBOT_DAY_MOTION.json not found - only Finnegan can unlock` (runs/real_e2_review_r1_gate.log); check_motion_gate() returns
  `('INFO', 'MOTION: LOCKED')` (runs/real_e2_review_r1_motioncheck.log). With a planted 86 that hangs past GATE_TIMEOUT_S: TimeoutExpired
  raised (runs/real_e2_review_r1_timeout.log).

## Verdict 1 - implementation fidelity: did the code do what the plan said? YES
Part C follows section 7 in order: M0 record first (with WARNING / arm_sdk-writer refusal / overwrite-once / "86 release first" branches),
the bounds line, elbow rounding up, the four waist columns and the waist rule, the bold NOT cleared placeholder, M1-M6 with command,
"you should see" and abort rule, soft/fault exit explanation, the foreign_hand_cmd box and the rule 16 (e) crash box. Every command's
flags exist (flag test passes for 84 and 86). 89 runs `86 gate` in a separate process and reports INFO. Image copied and trackable.
Both tests exist with Proves/Catches docstrings and AAA layout; the mutation check was done and logged.

## Verdict 2 - plan correctness for real hardware: right, with two small contract gaps (findings 1 and 2)

## Findings
1. minor - The clearance oracle omits one clause of rule 16 (c): M4 needs `setback_m <= setback_max_keep_layout_m`. tests/test_unit_runbook.py
   expected_cells() follows the task-E2 formula (which also omits it). Reproduced: report with an elbow row `table_steps: "setback",
   setback_m 0.08` and `setback_max_keep_layout_m 0.04`, runbook row M4 "yes" -> test PASSED (runs/real_e2_review_r1_mutation.log,
   "RULE16c"). Impact: if 87 ever emits a setback row beyond the layout limit, a runbook "yes" for a layout that cannot be built on the
   real table would pass. Fix: add `and setback <= summary["setback_max_keep_layout_m"]` to the M4 cell (or pin in PLAN 3.5 that 87 writes
   table_steps "skip" in that case), and align the task-E2 formula with rule 16 (c).
2. minor - The oracle assumes the report shape `summary["elbows"]` = rows {elbow, table_steps, setback_m, fault_exits_clear}; PLAN 3.5 says only
   "per elbow" and does not name the container. Track C's 87 does not exist yet. Impact: at integration a different key raises KeyError
   (loud, not silent, but costs time in the tight 02:45-05:15 window). Fix: pin the key in 3.5 / tell Track C, or check it at integration first.
3. minor - 89 check_motion_gate() can FAIL the self-test although the plan says "information, not a FAIL": a hang past GATE_TIMEOUT_S raises
   TimeoutExpired, which main() turns into `CHECK motion: FAIL` (reproduced, runs/real_e2_review_r1_timeout.log). Fix: catch
   subprocess.TimeoutExpired inside check_motion_gate and return ("INFO", "86 gate hung for 60 s - run `python scripts/86_real_run.py gate` by hand").
4. minor - 89 prints only `MOTION: LOCKED` and drops 86's reason lines (e.g. "docs/ROBOT_DAY_MOTION.json not found", or a pinned-commit
   mismatch that would surprise the operators). Also the no-MOTION-line case says what happened but not what to do. Fix: append the first
   indented problem line to the INFO detail; add "run 86 gate by hand and read its lines" to the no-MOTION-line message.
5. minor - The bounds line in Part C is hand-written prose ("more than 0.15 from 0", "-0.1 .. 0.3", "0.0 .. 1.6") and (a) carries no unit (rad)
   in a safety lookup, (b) is not checked against 87's GRID_BOUNDS / FACTORY_LOOKUP_TOL by any test, although task E2 says it is "copied from"
   the report. Fix: write "rad"; extend test_runbook_clearance_table_matches_report to compare the bounds numbers with summary GRID_BOUNDS.
6. minor (readability) - expected_cells() packs four nested by_number lookups into one chained `and` inside a list comprehension; by_number()
   uses bare next(), so a missing setback/limit key surfaces as an unexplained StopIteration. A second-year student can follow it with the
   docstring, but it is the densest function in the file. Fix: split out `m4_allowed(summary, elbow, limit) -> bool` and give next() a
   default plus an assert message naming the missing key.

Not findings (checked): Part C order matches section 7; first Part C command is the 84 record (86 gate before Part B lies outside Part C and
moves nothing); M6 uses run's own 60 s default; replay default 120 s covers the 97 s demo at 0.25; M3 --no-camera is valid for replay only;
the crash box keeps "86 release" behind the table pull (rule 13/16 (e)); nothing in E2 publishes DDS or touches the T7.

Not run: test_runbook_clearance_table_matches_report against a real 87 report (Track C pending; skip is planned, integration re-runs it).

GO (round 1) - implementation matches the plan; all six findings are minor and can be folded in the fix round or at integration.
