# Table clearance: the "arms up first, then table in" variant (2026-10-27)

**Question.** `runs/real_c_clearance.log` says: for every factory elbow 0.0-1.6 and waist limit 0 / 0.15 / 0.35 / 0.6, the table
steps M4-M6 are skipped and the replay and fault exits are NOT clear. Does it help to raise the arms to the ready pose away
from the table first, and push the table in (or place the robot at it) only then?

**Answer: it fixes the way in, not the ways out.** Raising the arms away from the table and pushing the table in is clear.
But every exit that ends in the factory pose (soft exit leg 2, fault exit, `86 release`) still sweeps the hand down through the
table top and the basket when the table stands at the reach distance. A VLA attempt at the table is therefore still not
"clear" by 87's 5 cm rule in any layout 87 can express.

## What 87 checks now

New flag `87 --arms-up-first` (scripts/87_arm_path_clearance.py, summary in g1sim/arm_clearance_report.py):
- block 2 checks **every** factory elbow at the nominal table (before: only at the elbow's own table setback, so a "skip" elbow
  got no fault exit numbers at all - the old log's "NOT clear" for replay/fault exits was partly "never computed");
- adds the `86 release` line (0.3 rad/s straight joint line from a policy pose to the factory pose, `real_safety.release_plan`);
- prints one verdict line per exit with the smallest distance.
- Test: `test_arms_up_first_checks_fault_and_release_exits_of_a_skipped_elbow_at_the_nominal_table` (Proves/Catches in its
  docstring; a mutant without the flag's rule fails it). 11 of 11 clearance tests pass.

## Verdict per exit (nominal table, torso lean 0.126 rad as in 87 since review B1)

Negative distance = the hand/arm geometry is inside the object in the FK. "Clear" = never closer than 5 cm, or than the arm
already was at the start of the exit.

| Exit | Verdict | Smallest distance | What it hits |
|---|---|---|---|
| Arms up (factory pose -> ready) with the table pulled back >= 30 cm | **clear** for every elbow 0.0-1.6 | >= 9.9 cm | - (at 25 cm back: elbows 0.0-1.1 only, 4.9 cm) |
| Push the table in (arms held at the ready pose, table 0.5 m back -> nominal) | **clear** | >= 10 cm at setbacks 0-0.30 m (5 cm steps), 0.40 and 0.50 m | - |
| Ready -> grasp -> basket (the demo path itself, 2 cm-spaced poses) | **NOT clear** | table -0.5 to -0.7 cm (1 pose, frame ~128); stand -2.4 cm, basket -2.6 cm | the grasp at the stand and the drop over the basket rim |
| Replay/soft exit, leg 1 (policy pose -> ready) | **NOT clear** (207 of 356 start poses pass) | -1.4 to -2.7 cm | stand / basket only; the table passes in 356 of 356 |
| Replay/soft exit, leg 2 (ready -> factory pose, at the table) | **NOT clear**, every elbow | -5.2 (elbow 0.0) to -11.9 cm | table edge (elbow >= 0.1), basket (elbow 0.0) |
| Fault exit (ramp out, factory controller pulls to its pose; worst of 3 orderings) | **NOT clear**, 0 of 18156 rows | -14.9 to -16.3 cm | table top and basket |
| Release after a crash (`86 release` from a policy pose) | **NOT clear**, 0 of 18156 rows | -7.3 to -15.9 cm | table top and basket |

Same picture for every waist limit; details per elbow x waist in `runs/arm_path_clearance_arms_up_first.log`.

**Lean sensitivity** (probe, elbows 0.0:1.6:0.4, lean = the held q0 pitch 0.041 instead of the max sag 0.126): the demo path
keeps 3.3-3.5 cm from the table (no pose inside it), stand/basket still down to -1.9 cm; soft exit leg 1 still NOT clear
(-0.5 to -2.2 cm); fault and release exits still NOT clear (down to -15.2 and -14.8 cm). The real robot's lean is not measured yet.

## Why no layout at the reach distance clears the exits

- The table top is at 0.74 m, the same height as the pelvis (0.75 m in the walk crouch). The factory pose holds the hands at
  the sides, below the table top. Any path from a pose above the table down to the factory pose crosses the table front
  edge or the basket, whatever the order of the joints.
- The ball stays in the policy's reach only up to a 0.05-0.07 m setback (training reach 0.52 m). The hanging <-> ready path
  needs >= 0.25 m (elbows <= 1.1) or >= 0.30 m (all elbows). These two ranges do not overlap.

## Layout and order that make a VLA attempt safe(r) - decisions for Finnegan

1. **Proven by 87:** raise the arms with the table >= 30 cm farther back than nominal (front edge >= 24.5 + 30 = 54.5 cm from the
   robot, rule RV-2 measure), then push the table in to its tape marks with the arms at the ready pose, and pull it back
   >= 30 cm before the arms go down. Moving the table while the robot holds its arms is safe in the FK (>= 10 cm all the way).
2. **Needs an 86 change (not built):** the soft exit would have to stop at the ready pose (leg 1) and wait until the table
   is pulled out before leg 2. Today it goes on to the factory pose after a 2 s pause. Leg 1 itself is clear of the table but
   grazes stand/basket from poses that already touch them.
3. **Not solvable by layout at the reach distance:** fault exit and release after a crash. The options are: (a) accept the
   residual risk with a light, unfixed, padded table that slides away, slow speed and the e-stop (a safety call for Finnegan
   and the owner, not a 87 "clear"); (b) a lower table with the ball and basket raised on stands to the trained height.
   87 cannot model table height today, and the camera image would differ from training. Unverified; it would need a scene
   option in 87; (c) shadow mode at the table (the plan's fallback: policy runs on real frames, arms hold the ready pose).

## Files

- Report: `runs/arm_path_clearance_arms_up_first.json` + `.log` (merged from 4 runs split by waist limit, 87 `--merge`;
  sampling <= 2 cm checked).
- Probes: `runs/arm_path_clearance_far_setbacks.{json,log}` (block 1, setbacks 0-0.5 m),
  `runs/arm_path_clearance_arms_up_first_lean0041.{json,log}`, `runs/arm_path_clearance_arms_up_first_objects.txt` (per-object
  pass counts), `runs/arm_path_clearance_static_poses.txt` (ready and hanging poses vs setback).
- The default report `runs/arm_path_clearance.json` (which the runbook test reads) is unchanged.
