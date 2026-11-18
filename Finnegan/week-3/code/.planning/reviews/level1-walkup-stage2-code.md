# Code review: level1-walkup Stage 2 (perception: ball sphere fit + basket rim)

Reviewer: independent gate, did not author the code, no author reasoning seen. Date 2026-10-08.
Scope: `g1sim/perception.py` (diff), `tests/test_unit_perception.py` (new). Stage 1 is being built in the same tree in parallel;
its files were not judged.

## Verdict
- (1) Implementation fidelity: **YES.** Every item of the Stage 2 `<action>` is implemented with the planned signatures; `<verify>` and `<done>` re-run and met.
- (2) Plan correctness: **YES for Stage 2's scope, with one major gap carried forward** (F1: `find_basket` is a bounding box over every
  rim-band point, so any tall object in view, e.g. the section-I distractors, breaks the basket estimate; the section-I gate only checks the ball).
- Overall: **GO** for Stage 2. F1 must be closed before or inside the distractor stage (section I), not later.

## Re-run evidence (all output in files, summaries read from the files)
| Check | Command / file | Result |
|---|---|---|
| Unit tests, project env | `.venv/bin/python -m pytest tests/test_unit_perception.py` → `runs/l1w_s2_review.log` | 30 passed, 8.5 s |
| Unit tests, g1-ext env | same with `~/dev/g1-ext/.venv` → `runs/l1w_s2_review_g1ext.log` | 30 passed |
| Locator regression | `$PY scripts/54_grasp_scripted.py --trials 5` → `runs/l1w_s2_review_54.log` | 5/5, perception 2.8 mm, identical to `runs/d2_nachher.log` before the change (2.8 mm). Side effect: the script rewrote `runs/stufe1_4_greifen.json`. |
| Locator-dependent tests | `tests/test_integration_grasp_ik.py tests/test_integration_randomization.py` (g1-ext) → `runs/l1w_s2_review_locator_tests.log` | 11 passed |
| Own real-render probe (not author's) | scratchpad `rev_s2/real_render.py`: `volleyball_walk` model, pelvis at stop pose and 0.5/0.75/1.0 m back, lateral ±0.25, heading ±30 deg, head tilt 0.5, MuJoCo depth + 2 mm noise → `runs/l1w_s2_review_render.log` | 200x320: ball found 25/28, max error 3.76 mm (others ≤ 1.07 mm); basket 21/28, max 3.26 mm; ≤ 55 ms per look. Every miss is the object outside the FOV (e.g. lat -0.25 heading -30: ball 59 deg off-axis) or a cut basket returning None by the size check. 400x640: ball 25/28 max 3.83 mm, basket 23/28 max 10.7 mm, ≤ 455 ms. |
| Noise margin | same probe, depth noise 4 mm / 8 mm → `runs/l1w_s2_review_render_0.004.log`, `_0.008.log` | 4 mm: ball 24, basket 16 found; 8 mm: ball 11, basket 0 |
| Adversarial synthetic | scratchpad `rev_s2/adversarial.py` (uses the test helpers) → `runs/l1w_s2_review_adv.log` | see F1, F2 |

`<done>` check: ball centre ≤ 2 mm and basket ≤ 5 mm on synthetic data (test assertions use exactly these bounds and pass), None when
missing (`test_no_sphere_no_ball`, `test_too_few_points_no_ball`, `test_basket_missing_or_cut_off`), 54 still 5/5. Met.

## Fidelity against the task block
- `points_from_depth(depth, cam_pos, cam_xmat, fovy, pixels)` (perception.py:38) extracted; `Locator.locate` calls it (perception.py:174). Formula
  unchanged; focal now from `fovy` per call; `Locator.focal` removed, no other user of it in g1sim/scripts/tests/openpi_g1 (grep).
- `find_ball(points, radius, table_top, max_residual)` (perception.py:92): no colour input at all, z > table_top + 0.01, voxel connected
  components, known-radius Gauss-Newton fit with trimmed refits, lowest residual below threshold wins, None below MIN_PIXELS. Extra
  plane/upright-cylinder curvature test and on-sphere share go beyond the plan in the right direction (reject cylinders/boxes).
- `find_basket(points, ball_centre, rim_z, ...)` (perception.py:129): band ±0.015, ball/stand keep-out, bounding-box centre, None below 100.
  Optional `outer_size` size check is an addition, not a deviation.
- No new classes. Tests cover every case the plan names: sphere one side, grey sphere next to red/grey cylinder and box, cylinder alone
  (plus thin cylinder, puck, box, basket) → None, ring with each wall hidden, floor reflection, missing basket.
- `cluster_labels` checked by hand: codes are collision-free (shifted keys, span = max + 2) and sorted with the lexicographic `np.unique`
  order; min-label propagation with pointer jumping converges to one label per connected component. Correct.

## Findings

1. **MAJOR (plan gap, carried to section I / distractor stage): `find_basket` is corrupted by any other object crossing the rim band.**
   Trigger: a 0.2 m tall cylinder (section I distractor type) 0.03-0.10 m beside or in front of the basket. Evidence `runs/l1w_s2_review_adv.log`:
   without `outer_size` the centre is off by 41-91 mm (above the 0.06 m `detection_error` gate in B); with `outer_size` it returns None,
   i.e. `not_found`. Only the 1 m-back view with the distractor on the far side survives. Cause: perception.py:143-146 keeps every rim-band
   cluster ≥ MIN_PIXELS and takes one bounding box over all of them. The ball side is robust (box 5-50 mm from the ball: error ≤ 0.82 mm).
   Section I's gate ("Stage 2 still picks the ball, not a distractor") checks only the ball, so this would surface first in Stage 4/recording.
   Fix: choose the basket per rim cluster (or cluster set) whose box matches `outer_size` within tolerance, instead of one box over all
   rim points; add a unit test "basket + tall cylinder/box 0.03 m away" and extend the section-I gate to the basket.

2. **MINOR: spheres of other radii pass as the ball.** `runs/l1w_s2_review_adv.log`: a lone sphere of radius 0.09 or 0.12 m is accepted as the
   0.105 ball (0.07 and 0.14 are rejected). No spherical distractor is in v1 (section I lists cylinders and a box), so not reachable now;
   it is reachable in Level 2 ("second ball + prompt"). Fix when needed: also fit with free radius and reject |r_free - radius| > ~1 cm.

3. **MINOR: thresholds are tied to the sim's 2 mm depth noise.** perception.py:19 `MAX_SPHERE_RESIDUAL = 0.005` and the ±0.015 rim band.
   At 8 mm noise the basket is never found and the ball only in 11/28 views (`runs/l1w_s2_review_render_0.008.log`). Not a sim-v1 problem
   (config noise is a fixed 2 mm), but the real D455 is noisier at 0.6-1.0 m (inferred from the sensor class, not measured here), and the
   user's motivation is the real grey ball. Note it on the real-adapter list; consider scaling the residual threshold with the noise std.

4. **MINOR: `BASKET_SIZE_TOL = 0.02` (perception.py:28) lets a partially cut basket through with up to ~1 cm centre error**
   (400x640, back 0.75 heading +30: 10.7 mm, `runs/l1w_s2_review_render_640.log`). Inside the Stage 4 gate (p95 ≤ 0.04 m), so no action
   needed now; tighten to ~0.01 if Stage 4 shows basket error in the tail.

No blocker. No regression found in the bottle/Locator path. The fixed volleyball scene, openpi and teleop paths do not import the
changed code (perception is used only by `g1sim/grasp.py`, `scripts/54`, `g1sim/randomization.py` docstring, `tests/test_integration_randomization.py` via the unchanged `red_mask`).

## Not run / not judged
- `scripts/run_tests.sh` in all three envs (Stage 1 is mid-build in the same tree; its failures would not be attributable). The Stage 2 file
  passes in the project and g1-ext envs; the openpi env does not run it.
- Real-render accuracy over a real walk-up (moving robot, randomized layouts) stays a Stage 4 gate as the plan says.
