# Review: solo robot day 2026-11-11 (uncommitted change of 2026-11-10)

## Review round 1 (2026-11-10, independent reviewer, did not write the change)

Scope: `git diff` of g1sim/motion_gate.py, scripts/86_real_run.py, tests/test_unit_motion_gate.py, tests/test_unit_runbook.py,
docs/ROBOT_DAY_RUNBOOK.md, PLAN.md, PROGRESS.md, docs/LEARNING_PATH.md, plus the new docs/ROBOT_DAY_2026-11-11_SOLO.md.
Logs: `/tmp/solo/review/`.

**Verdict 1 - did the change do what the brief asked: yes.** **Verdict 2 - was the brief/design right: yes, with one gap (m1).**
**GO** (m1 is a doc line; docs are not loaded by 86, so fixing it after the unlock does not lock motion).

### What I checked myself

1. **Solo gate** (`probe_gate.log`): finneganadam + OWNER + START passes; acristinaa + OWNER, saadsheikh-tech + OWNER,
   finneganadam twice, OWNER first, OWNER twice, lowercase `owner`, finneganadam + saad, empty names: all refused before START
   is asked; wrong START (`start`, `yes`, empty) refused; the team path is unchanged. Diff hunks in motion_gate.py touch only the
   docstring, the new `OWNER_WORD` constant and `confirm_operators`: checks (a), (b), (d) and the sim bypass are unchanged.
2. **run_tests.sh once** (`tests.log`): 321 passed / 600 passed / 10 passed, exit 0 in all three envs.
3. **S1 command once**, exactly as in the runbook (sim Python, robot env active as on site, no DYLD path), on my own pose
   `/tmp/solo/review/factory_review.json` (elbow 1.3, shoulder pitch 0.2, small roll/yaw/wrist, waist pitch 0.02): 42.6 s,
   `elbow 1.3: way in clear (min 8.9 cm) ...; recover from the way in clear ...; way out per waist: 0.60 clear`,
   `lift home waist 0.60: 5265/5265 pass ...`, `REPORT ...`. Matches the "what you see" lines. Refusal case (waist pitch
   -0.08): `ValueError: M0 q0 waist roll 0.000 / pitch -0.080 rad outside the checked lean ...: skip table steps`, no REPORT,
   no file. Matches the runbook.
4. **Every command line** of the solo page and of runbook Part B, Part C and Reference (42 lines with 81/84/86/87/89) parsed
   with the scripts' real argparse (`parse_cmds.log`): 0 errors. 81 without `--host` listens on 127.0.0.1:8765 (so skipping
   the policy-env firewall block is right). The model path `~/g1-robot-day/smolvla-runs/run1/checkpoints/040000/pretrained_model`
   exists. 84 writes `runs/factory_pose_robot.json` for a robot iface, the path S1 reads.
5. **Unlock recipe vs motion_gate.py**, run literally in a throw-away clone (`gate_*.log`): with `"date": "2026-11-11"` tonight
   86 gate prints `MOTION: LOCKED` and exactly `  the unlock is for 2026-11-11; only Finnegan can unlock today` (exit 2); the
   same pin with today's date prints `MOTION: UNLOCKED`; a later docs-only commit stays UNLOCKED. Field names, value types
   (`true`, 40-hex commit, string date), committed-file rule, ancestor rule all match.
6. **Step order:** M0 and S1 move nothing; M1 release goes measured -> factory (no motion at the factory pose); M2/M3 with the
   table away; waist 0.6 at the table only after a clean M3 line 4; M4 stand flags (+6.7 / -1.7) match the R-F8 placement;
   M5/M6 flags (stand 0/0) match "stand back on its B3 mark" after the robot and head check; table-away fallback uses
   `--fence-setback-cm 100`. The crash box and the "no 86 recover after a policy-phase crash" rule are consistent with S6/O2-m3.
   Demo waist yaw is -0.498 .. +0.456 rad, so the "numbers hold for waist 0.6" claim is right.
7. **Nothing loosened:** the runbook only got stricter (no "back N cm" table position, only waist 0.6 at the table, M5/M6 away
   at the last clean M3 setting). No constants, fault actions, fence, envelope or lead values changed. Section 5's sentences
   match the code (STALE_S 0.1 s, STALE_RETURN_WAIT_S 3 s, LEAD_LIMIT 0.25 / 13 ticks, RECOVER_HOLD_S 0.5, half speed 0.15,
   FAULT_ACTIONS, MARGIN_M 1.3/2.0/0.9, SELF 2.0, FENCE_STREAK_EXTRA_S 0.5, BLEND_ACTIONS 5, PELVIS_TILT_FROM_IMU True).

### Problems (NO-GO)

None.

### Minors

- **m1 (should-fix, doc only): S1 does not cover every table layout the runbook allows.** 87 checks table heights -4 / 0 / +1 cm
  only (`arm_paths_clearance.TABLE_DZS`) and the robot square on its marks. Runbook M4 says "if the sheet says otherwise, change
  `--fence-table-dz-cm`" with no bound, so a table set 2-3 cm too high (or a turned robot, `--fence-robot-yaw-deg`) would run the
  table steps on a way in that S1 never checked. Add one sentence to the S1 decision and the solo page: table steps only with
  table top between 4 cm below and 1 cm above the B3 target and no robot yaw; else re-set the table or skip the table steps.
- **m2: the "How an 86 run ends" text (runbook, said out loud before M1) is stale.** It says a fault exit lowers the arms within
  about 2 s. Since F1-F5 a fault holds (weight 1, arms stay up) or recovers along the waypoint path. The owner should hear the
  new behaviour ("a fault may leave the arms held still: then your remote or e-stop"). Section 5 of the solo page has it right.
- **m3: the solo page does not print the M0-M3 lines and the M3 replay-server line in full;** it sends Finnegan to the runbook,
  whose second-terminal block starts with `cd ~/UnitreeG1-Robot`. That folder does not exist on this Mac, so the cd fails and 81
  then says "No such file" (harmless, but a stumble). The solo page's "every terminal starts with" lines cover it if he follows
  them; copying the 81 `--replay` line into the solo page would remove the trap.
- **m4: the solo path is not tied to a date.** `finneganadam` + `OWNER` works on any future day. Harmless, since (a) still needs
  Finnegan's own committed unlock for that date; worth one line in the docstring that the solo path is meant for days the
  author is present.
- **m5 (wording):** docstrings in g1sim/arm_clearance_report.py, scripts/87, g1sim/real_factory_pose.py and g1sim/real_bringup.py
  still say "M0 lookup" / "table lookup". Do not change them tonight (it would need a new pin); after the day.

## After round 1 (main session, 2026-11-10)

Minors m1 (table dz -4..+1 cm and robot straight, runbook Fence box + solo page), m2 (fault exit text: hold or recover, no drop) and m3 (81 --replay line copied into the solo page) applied, doc only; tests/test_unit_runbook.py + test_unit_motion_gate.py 52 passed.
