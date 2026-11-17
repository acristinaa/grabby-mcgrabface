# Code review round 2: level1-walkup Stage 2 (perception: ball sphere fit + basket rim)

Reviewer: independent gate. I did not write this code and did not use the author's reasoning; the fix report was treated as a claim
and checked again. Date 2026-10-08. Scope: `g1sim/perception.py` and `tests/test_unit_perception.py`. Stage 1 is being edited in
the same tree, so its files were not judged.

## Verdict
- (1) Implementation fidelity: **YES.** The Stage 2 `<action>` is still met with the planned signatures. The fix adds one helper
  function (`in_densest_square`, perception.py:135) and a free-radius check (perception.py:121-123). No new classes. `<verify>` and
  `<done>` were re-run and are met.
- (2) Plan correctness: **YES for Stage 2's scope.** One major issue is carried forward to the distractor stage (Stage D, section I)
  and Stage 4. It is N1 below and comes from the same family of problems as F1.
- Overall: **GO** for Stage 2. One condition carries forward: N1 must be closed, or the Stage D placement must make it unreachable,
  before distractors are switched on. The section-I gate must test the basket as well as the ball.

## Re-run evidence (full output in files; summaries read from those files)
| Check | Log | Result |
|---|---|---|
| `.venv/bin/python -m pytest tests/test_unit_perception.py` | `runs/l1w_s2_r2review.log` | 44 passed, 10.7 s |
| Same file in the g1-ext env | `runs/l1w_s2_r2review_g1ext.log` | 44 passed |
| `$PY scripts/54_grasp_scripted.py --trials 5` (g1-ext) | `runs/l1w_s2_r2review_54.log` | 5/5, perception 2.8 mm, ABNAHME BESTANDEN. `runs/stufe1_4_greifen.json` was rewritten, and its content matches the earlier file byte for byte |
| Locator tests (`test_integration_grasp_ik.py`, `test_integration_randomization.py`) | `runs/l1w_s2_r2review_locator_tests.log` | 11 passed |
| Round-1 adversarial probe, re-run unchanged | `runs/l1w_s2_r2review_adv.log` | see F1 and F2 below |
| Round-1 real-render probe, 200x320, 2 mm | `runs/l1w_s2_r2review_render.log` | tilt 0.5: ball 25/28 (max 3.76 mm), basket 21/28 (max 3.26 mm), ≤ 51 ms. tilt 0.85: ball 10/28 (max 7.33 mm), basket 17/28 (was 11), max 4.14 mm. Ball counts are the same as round 1, so the free-radius check rejected no real ball |
| New probes (scratchpad `rev_s2r2/adv2.py`, `adv3.py`, `adv4.py`) | `runs/l1w_s2_r2review_adv2.log`, `_adv3.log`, `_adv4.log` | see N1, plus the noise check below |

## Round-1 findings
1. **F1 (major): FIXED for the stated trigger.** A 0.2 m cylinder (r 0.04) at 0.03, 0.06 or 0.10 m in front of or beside the
   basket, with `outer_size` given: centre error 0.45-1.18 mm (stop) and 1.6-2.98 mm (1 m back). Before the fix these cases returned
   None (`runs/l1w_s2_r2review_adv.log`). The new test `test_basket_next_to_a_tall_object` covers it. Gaps below the stated 0.03 m
   limit: 0.01 m gives 6.3 mm, and 0.02-0.025 m give ≤ 2.8 mm (`_adv2.log`). Without `outer_size` the behaviour has not changed
   (41-91 mm error). The docstring says so, and Stage 4 has to pass the size.
2. **Minor 2 (spheres of another size pass): FIXED.** Spheres of radius 0.07, 0.09, 0.12 and 0.14 return None from both cameras
   (`_adv.log`, and the new test `test_sphere_of_another_size_no_ball`). The free-radius fit runs on the trimmed points and did not
   cost recall: the true ball was found 10/10 at depth noise 2, 4 and 6 mm from both cameras (`_adv2.log`), and the real-render ball
   counts did not change.
3. **Minor 3 (thresholds tied to 2 mm noise): NOT FIXED, deferral accepted.** Fixing it needs real D455 noise data. It belongs on the
   real-adapter list, and the author has put it there.
4. **Minor 4 (`BASKET_SIZE_TOL` 0.02): NOT FIXED, deferral accepted**, as recommended in round 1.

## New findings
**N1 (MAJOR, carried to Stage D / section I and Stage 4): an object in front of the basket, or one next to a partly visible
basket, can pass the size check. `find_basket` then returns a wrong centre 0.06-0.25 m off instead of None.**
- Mechanism (perception.py:167-173): `in_densest_square` keeps the 0.31 m square that holds the most rim-band points. Points on an
  object close to the camera are dense. When that object hides the near wall, or when the basket is cut by the image edge, the
  densest square is the object's face plus part of the basket walls. The bounding box of those points is then about 0.29 m wide, so
  the size check that should return None passes.
- Evidence (synthetic clouds, test helpers, 2 mm noise, `outer_size = 0.29`):
  - Object between robot and basket (`_adv3.log`, `_adv4.log`): a box of half-width 0.06 m (0.2 tall) at gap 0.03 or 0.06 gives
    145-175 mm error from 0.5, 0.75 and 1.0 m back. A box of half-width 0.1 seen from 0.5 m back and 0.25 m to the side gives
    224-254 mm. A cylinder of r 0.08 seen from 0.5 m back gives 184-213 mm. A cylinder of r 0.06 at gap 0.03 seen from 1 m back gives
    145 mm. That is the size of the "red cylinder" in the unit test. Thinner objects (r ≤ 0.05) are fine, and wider ones mostly give
    None, which is safe.
  - Basket cut by the image edge on one side, with a cylinder (r 0.04-0.06) at gap 0.03-0.05 on the far side (`_adv2.log`): 55-135 mm
    error in 22 of 24 cases. The same cut basket with no other object returns None. Cut baskets do happen in the look views: the
    real-render probe returns None for basket in 7 of 28 views at tilt 0.5, partly because of the size check.
  - Objects behind the basket or between basket and ball are fine (≤ 3.9 mm, `_adv3.log`).
- Reachability: there are no distractors in v1, and `g1sim/distractors.py` does not exist yet. Under section I (0-3 objects, a red
  cylinder, a grey cylinder and a box, ≥ 0.03 m from the basket), the front case needs the basket set back from the table edge, which
  the layout rule allows ("flush with the edge or further back"), and an object placed there. Whether Stage D's keep-out ("outside the
  hands' sweep and the waist-turn path") excludes the front of the basket is not decided yet. The cut-basket case needs only an object
  beside the basket and a look view where the basket is near the image edge. That fits the walk-up start-pose ranges.
- Impact: a wrong estimate skips the "not found, turn and look again" path. In sim it fails the episode as `detection_error`
  (> 0.06 m). On the real robot nothing checks it against ground truth, so the robot walks to a wrong stop pose.
- Test gap: `test_basket_next_to_a_tall_object` uses only an object 0.08 m wide and a fully visible basket, so it does not catch this.
- Fix direction (the author chooses): before accepting the square, require that the chosen points cover each of the four edges along
  most of their length (a 0.12 m object face does not cover a 0.29 m edge), and/or return None when a rim cluster crosses the square's
  border. Add unit tests for "box half 0.06 in front, 0.5-1.0 m back" and "cut basket plus an object on the other side". Extend the
  section-I gate to basket error.

**N2 (MINOR, already listed by the author): grid size in `in_densest_square`** (perception.py:138-143). The grid covers the full xy
range of the rim points at 5 mm cells. That is cheap in the sim (≤ 51 ms per look in total), but on real data with far surfaces at
rim height it grows with the square of the range. The author's mitigation is to crop to the table area first. I agree it is not
needed for sim v1.

## Not run / not judged
- `scripts/run_tests.sh` in all three envs was not run, because Stage 1 is being edited in the same tree. The Stage 2 test file passes
  in the project and g1-ext envs.
- I did not run the 400x640 real-render probe again. The author's `runs/l1w_s2_fix_render_640.log` was not re-derived. The 200x320 run
  matches round 1 plus the expected basket gain at tilt 0.85.
- N1 was shown on synthetic clouds only, not on a MuJoCo render with a distractor, because the distractor scene hook does not exist yet.
