# Review: Level 1 finger-clearance plan (`.planning/level1-harden/FINGER-PLAN.md`, revision 1)

Stage: plan review (pre-build). Reviewer: independent; I did not write the plan and have not seen the planner's reasoning.
Date: 2026-10-27. Scope: FINGER-PLAN.md checked against `g1sim/ball_grasp.py`, `scripts/67_volleyball_pick_place.py`,
`scripts/69_volleyball_demo_factory.py`, `configs/randomization.json`, `g1sim/table_env.py`, `scripts/run_tests.sh`, the two run1 eval
JSONs and the planner's probe outputs in `/tmp/l1day1/`. Reviewer runs are in `/tmp/l1day1/rev/`. No repo file was changed
except this review and `.planning/STATUS.md`.

## Verdict: GO

Implementation fidelity: pending (nothing built yet). Plan correctness: right for the goal. The fixes below are minor and can go into
the build without another plan round. Item 1 is a one-line change to a gate command, and the builder must make it.

## What I re-derived (evidence)

1. **Student failure table (§1.1): correct.** Recounted from `runs/vla_eval_run1_04{0,5}k_seed0.json` → `sections.main.rows`.
   40k: 36 pass, 6 contacts (L pinky-stand 2, L pinky-basket 3, L pinky-table 1), 7 `ball_lifted`, 1 placement. 45k: 35 pass, 12 contacts
   (L pinky-stand 6, R pinky-stand 1, table 2, R pinky-basket 1, R palm (`rh_right_base_link`)-basket 1, L pinky-basket 1), 3 `ball_lifted`.
   The table contacts come at 4.40-4.43 s and the teacher's pinky-table minimum at 4.43-4.53 s. The timing matches.
2. **The constants are read only inside `ball_grasp.py`:** grep of g1sim/scripts/tests/openpi_g1 finds `CRADLE_DEG`, `REACH_ABOVE` and `SQUEEZE` only
   in `ball_grasp.py`. The teacher is imported by 67, 69, 82, `vla_eval.py`, `labview.py` and two test files. None of the real-robot
   files (`arm_clearance*`, 85-87) import it. `tests/data/replay_demo.npz` comes from a recorded demo, not from the live teacher.
3. **The meter definition in Task 1 reproduces the probe exactly.** I wrote my own meter as Task 1 specifies it: the subtree of `*_elbow_link`
   via `body_parentid`, collision geoms only, and every geom of the `FORBIDDEN` bodies that exist (`/tmp/l1day1/rev/indep_meter.py`).
   It finds 47 arm geoms and 1 table, 16 stand and 5 basket geoms, all with contype 1. Walk seed 10, first 10 trials: the per-trial
   minimum equals `fc_s10.json` (old constants) and `fch_cr10ra6sq30_s10.json` (new constants) to 0.0 cm in 10/10 and 10/10 trials
   (`/tmp/l1day1/rev/old_s10.json`, `new_s10.json`). So the ±0.01 cm cross-check in Task 1 can be met.
4. **67 walk starts are the probe starts.** `f67_cr10ra6sq30_walk_s10.json` and `fch_cr10ra6sq30_s10.json` match in arrival for 50 of 50
   trials, and the lift heights are bit-identical. The probe numbers carry over to 67, so the G5/G6/G7 numbers reproduce.
5. **Probe pass counts for the chosen variant:** walk 599/600 (the only failure is s104_t0072, which also fails with the old constants).
   Fixed scene: nominal 10/10 ACCEPTANCE PASSED, ±2 cm 50/50, rand0 50/50, rand1 50/50, distractors 30/30. Lift min over G5+G6 is
   8.19 cm, so G9 (≥ 8.0) passes with 0.19 cm to spare. The test-2 nominal numbers are right: old 2.56 cm, new 4.06 cm, against a 3.5 cm threshold.
6. **Randomization:** the config randomizes stance, layout, arrival, physics (mass and friction 0.8-1.2, gains 0.9-1.1, latency 0-30 ms)
   and the image. It does not randomize the ball radius, stand height or table height. So the pinky-stand margin really is fixed by
   geometry, as §1.2 says.
7. **69's own gates do not move.** `record()` checks `episode.validate`, 600-800 frames and waist tracking. The phase durations are
   unchanged, so the frame count is too. `test_teacher_on_the_recorded_handover` records with 69's `record()` and stands in for a 69 smoke run.

## Root cause: supported

Every table contact falls at the end of the reach, at the same tick as the teacher's 0.7-0.9 cm (p5) dip. The two table-contact
starts stand far back (back_m 0.043, 0.056), which matches the correlation of -0.75 with `back_m`. Six of the nine stand contacts fall
during touch, squeeze or lift, while the teacher sits at a constant 2.3-2.5 cm. The other three come at 3.87 s, 15.4 s and 24.7 s, in
phases where the teacher's margin is larger. The half-normal sigma model is crude. Only the stand events fit it: at the table, a sigma
of 1.2-1.6 cm with a 0.7-2.3 cm margin would predict far more than 1-2 contacts in 50, because the table dip lasts only a few ticks
while the stand margin lasts about 4 s. The 3.5 cm number still comes from the stand, which is the conservative location.

## Is it the smallest change: yes

It changes three constants and their comments. `REACH_ABOVE` alone lowers the stand margin, the finger tilt drops the wrist onto the
basket rim, and `APPROACH_OUT` pushes the wrong way. The chosen variant is the only measured one that reaches 3.5 cm without losing
a start. The fallback (CRADLE 15 + REACH_ABOVE 0.06, 2.9 cm, squeeze unchanged) is offered honestly as ruling R1.

## Does it keep the teacher's gates: yes, except one item

G1-G4 are the merge gates from PROGRESS 10-09 (10/10, 50/50, 50/50 + 50/50, 30/30). G5 is the walk sweep, G8 the 82 teacher run and
G10 the test suite. **Missing:** the reach map `67 --scene volleyball_walk --map` (52/65, "every failure explained", PROGRESS 10-10).
`configs/randomization.json` takes its `walk.arrival_*` ranges from that map. See minor 3.

## Is the clearance gate number justified: yes, as a working number

p5 ≥ 3.5 cm, min ≥ 3.0 cm and pinky p5 ≥ 3.5 cm sit 0.15-0.2 cm under the values the chosen variant reaches. As a regression guard
on a deterministic sim this is sound. As a promise it is not, and the plan says so itself (R3, "re-derive after run2"). If I count only
the stand contacts that happen during touch, squeeze or lift (40k 1/50, 45k 5/50), sigma comes out at about 1.0-1.5 cm instead of 1.2-1.6.
That puts the per-location contact chance at 3.5 cm at about 0.1-1.6 %, so the number holds a little better than the plan claims.

## Does anything force re-recording all v1 demos? Stated plainly

- **In the code: no.** The observation/action format, `episode.validate`, 79 and 82 do not change. The v1 files on the T7 stay valid and
  convertible. The only test that reads the T7 (`test_no_walk_seed_101_reproduces_the_recorded_handover`) compares the handover state,
  which is fixed before the teacher runs.
- **For the purpose of the change: yes, all 300 v1 demos (seeds 101-104) and the partial v2add seeds 105-108 are superseded.** A student only
  gets the margin if it learns from the new path, and mixing old and new demos teaches two grasps whose average is the near-miss path
  again (plan §9.3, R2). So run2 needs a full re-recording with the new teacher: the same seeds 101-104 give the same starts, about 300
  demos, and it needs the disk (the T7 is not mounted). This plan does not record anything, which is correct.

## Findings

### Behaviour bugs (plan errors that would make the build wrong)
None.

### Minors (fold into the build; no new plan round needed)

1. **The G10 test check reports a false alarm by design.** The baseline test run happens "before editing" in Task 2, after Task 1 has
   added test 2. Test 2 is meant to be red at that point, so the "before" list has a FAILED line for
   `test_teacher_keeps_the_margin_on_the_nominal_walk_start` and the "after" list does not. A plain `diff` then exits 1, and the plan reads
   that as "new failure". The same happens if the runbook test gets fixed in between (E2 is running in parallel).
   Fix: strip ` - <message>` and count only the lines that appear in the "after" list, e.g.
   `sed 's/ - .*//' … | sort -u` on both files, then `comm -13 before after > new_failures.txt` (empty = pass).
2. **The hold tolerates less squeeze error after the change (not measured in the plan).** Reviewer runs, fixed scene `--randomize --seed 0`,
   50 starts each (`/tmp/l1day1/rev/h_*.log`):
   - old cradle (20 deg): squeeze 2.0 cm → 50/50, 1.5 cm → 49/50.
   - CRADLE 10 + REACH_ABOVE 0.06: squeeze 2.0 cm → 48/50 (trials 23 and 37, `ball_carried_to_basket`), 2.5 cm → 49/50 (plan), 3.0 cm → 50/50, 4.0 cm → 50/50.
   - Walk seed 10 at CRADLE 10, squeeze 2.0 cm → 50/50.

   So in the fixed scene the old grasp holds with the squeeze 0.5 cm short of nominal, while the new grasp first loses a start at 0.5 cm
   short. A student that squeezes too little sits closer to that edge. This does not block the plan: the walk world, where v2 and the
   eval live, held at 2.0 cm. But §9.2 and R1 should say it, and "lift failures unchanged" (§9, Other) is an estimate, not a measurement.
3. **Add the reach map as an info row:** `67 --scene volleyball_walk --map --clearance --no-labview --out runs/l1h_finger_map.json`
   (65 trials). Compare it with 52/65 and with the failure set of `runs/l1w_s3_map.json`. That shows whether the edge of the arrival range moved.
4. **Stale docstring lines.** The plan changes only docstring line 3. Line 9 then still says "squeeze (palms 2.5 cm in", which becomes
   3 cm, and line 8 "palms 4.5 cm out and 2 cm up" was already stale (`REACH_ABOVE` is 0.04 and becomes 0.06). Update lines 8-9 together with
   line 3. The `FINGERS_CURLED` comment (":79-85", "moved <= 1.5 mm", measured at the 20 deg cradle) is also stale: the slip p95 is now
   7.5 mm. Give it a one-line "measured at CRADLE 20" note.
5. **Evidence text errors in §1.1:** the stand contacts at 8.9-12.3 s are **6**, not 7. Basket "16.5-33 s" leaves out 45k
   `s0_t0039` (R palm vs basket at **10.57 s**, during the grasp, before any lift). That one is a reach/grasp clearance contact, which
   the change covers through R wrist-basket p5 1.1 → 5.0 cm. It is not "a consequence of a failed grasp". The plan slightly
   under-claims what the change fixes.
6. **`docs/volleyball_teacher_squeeze.png` is gitignored** (`.gitignore:7 docs/*.png`). Every fixed-scene 67 run writes it, so G2-G4
   overwrite G1's picture. Drop the "files_modified" entry, or run G1 last if the local picture matters.
7. **Test 1's "every labelled body is in an elbow subtree" checks itself:** it uses the same walk that builds the set. To really catch
   "legs/torso let in", assert that `pelvis`, `torso_link`, `left_knee_link` and `left_shoulder_pitch_link` are absent and that
   `lh_left_pinky_distal_link` and `right_wrist_yaw_link` are present.
8. Wording: §1.2 calls seeds 10-13 "the 10-14 sweep". It is 4 seeds and 200 starts.

## Checks I could not run
- I did not run the full gate G1-G10 on changed code. Nothing is built yet, and that is the build's job. I reproduced the probe at the
  trial level (point 3 above) instead.
- I did not run a student (run2) eval. It needs v2 demos (disk) and training (GPU). The plan says so too.
