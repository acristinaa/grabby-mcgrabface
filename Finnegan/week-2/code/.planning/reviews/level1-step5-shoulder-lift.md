# Review: Level 1 Step 5 shoulder lift (`g1sim/ball_grasp.py`, uncommitted diff), 2026-10-14

Reviewer: independent gate (did not write the change, no author reasoning seen). Stage: finished review.
Scope: `git diff g1sim/ball_grasp.py`: `_shoulder_lift` (hand targets turned about the shoulder-pitch line, bisection
on the angle so the ball centre rises LIFT, minus the arc's horizontal drift) + IK posture target = squeeze configuration
during the lift. Intent: straighter pick-up, arms less cramped, same reliability (PLAN section L).

## Verdicts
1. **Implementation fidelity (did the code do what was intended): YES.**
2. **Intent right for the purpose: YES.**
3. **Overall: GO.** No blocker, no major. Four minor/nit items below; none needs to change before the demos keep recording.

## What I re-derived (scratch: `.../scratchpad/review2/`)
- **Rotation sign/axis, bracket, straight-up, rigid pair, waist turned**: `bracket_probe.py` (kinematics only, waist at the
  teacher's grasp yaw, ball at rest, real `palm_pose` squeeze targets fed through the real `_shoulder_lift`), 400 random stances
  per scene (`bracket400_volleyball_walk.log`, `bracket400_volleyball.log`):
  - walk world: grasp waist yaw 16.7..43.0 deg; angle found 9.7..11.5 deg; rise available at the 0.6 rad bracket end
    0.270..0.318 m (needed 0.09); ball rigidly carried with the left hand ends exactly +0.0900 m up, 0.0000 m horizontal.
  - fixed scene: yaw -1.4..58.1 deg; angle 9.9..14.9 deg; rise at 0.6 rad >= 0.215 m. Same exact +0.09 / 0 result.
  - axis vs torso lateral axis: 0.000 deg in both scenes (axis follows the turned torso); axis tilt from horizontal <= 0.56 deg.
  - palm gap (squeeze) along the straight-line `_move` interpolation: change 0.0000 m at 21 points (the inter-hand vector is
    parallel to the rotation axis, so linear interpolation of a rigid rotation keeps the squeeze).
  - positive angle about (right - left) raises a point in front: confirmed (rise > 0 for all 800 stances).
- **Posture probe (allowed run, 5 s)**, walk world nominal, new vs HEAD code (HEAD run from a temporary scratch tree, since
  deleted): `posture_probe.log`, `p.json`, `p_old.json`.

  | after lift (R / L)       | old (HEAD)      | new             | squeeze (both) |
  |--------------------------|-----------------|-----------------|----------------|
  | forearm pitch deg        | 39.3 / 39.8     | 31.0 / 31.6     | 10.0 / 10.4    |
  | forearm yaw deg          | -11.3 / +9.5    | -1.5 / -0.2     | 0.3 / -1.3     |
  | upper arm down deg       | 34.0 / 33.7     | 39.6 / 39.3     | 45.6 / 45.4    |
  | elbow below shoulder m   | 0.106 / 0.105   | 0.123 / 0.122   | 0.137 / 0.136  |
  | wrist from shoulder m    | 0.238 / 0.238   | 0.244 / 0.244   | 0.261 / 0.260  |
  | ball rise / horiz drift  | 0.0838 / 0.0005 | 0.0899 / 0.0021 |                |

  Matches the PLAN L numbers; left/right symmetric (old lift splayed the forearms +-10 deg, new one does not).
- **Posture-target leak**: `ArmIK.reset` (`g1sim/grasp_ik.py:84-88`) sets the posture target. `_reset_ik` runs at the start of
  `run()` (`ball_grasp.py:192`), after face_ball (203) and after the turn (235). The turn (230) and turn hold (231) issue no IK;
  retract (245) is joint space. So the squeeze-posture target lives only through lift + `_correct_sag`; centre/release get the
  post-turn commanded configuration exactly as before; every episode starts clean. No leak.
- **`_correct_sag`** (345-354): uses only translations of the (rotated) lift goals in the torso frame; the sideways mean keeps
  the squeeze. The hand-frame `self.sag` rotates with the 11 deg pitched hand (small tilt of the compensation), and the 3
  correction rounds absorb it: ball rise 0.0899 in the probe; walk sweep lift 0.0847..0.1093 m (med 0.0917, old sweep med 0.0874).
- **Release direction** (242-243): palm normal now pitched ~11 deg about the lateral axis; its out-direction gains ~sin20*sin11
  = 6.5 % fore/aft component (~4.6 mm over 7 cm). Negligible; release passes 200/200.
- **Metrics/tests**: `lift_height_m`, `ball_lifted`, `ball_carried_to_basket` measure the ball, not the hands, so they keep
  their meaning; `posture_at_squeeze` is taken before the lift. `tests/test_integration_volleyball_grasp.py` asserts only
  pass / lift >= MIN_LIFT / end pose; `runs/l1w_s5_tests_v2.log` (17:49, after the 17:38 edit) has the file 7/7 and the suite
  233 passed. No test or report assumed hand-straight-up motion (grep over g1sim/scripts/tests).
- **Gates re-read from JSON** (not re-run): walk sweep seeds 10-13 50/50 each (lift min 0.0847, wall clearance min 0.0499 m,
  >= 2 hand parts on the ball after the lift on both sides); fixed nominal 10/10, jitter 50/50, rand0 50/50, rand1 50/50,
  distr 30/30. Baseline `l1w_s3_sweep.json` 199/200 used seed 0, the new sweep seeds 10-13: not a paired comparison.
- **Recording runs on this code**: `ball_grasp.py` mtime 17:38, the four `69` processes started 17:56/17:59. Read-only check on
  the T7 at 18:13: 110 demos saved (29/29/29/23), success_rate 1.0 per seed, 726 frames, ~30 s wall and ~117 MB per demo,
  latest sidecar lift_height 0.0918. Lab View at :8030 answers 200, live cards updated 18:15 and show head + outside frames.

## Findings

### F1 (minor) Demos recorded from an uncommitted teacher
`scripts/69_volleyball_demo_factory.py:64` (`GIT = labview.git_state()`) writes `{"commit": "6403b25", "dirty": true}` into every
sidecar and `factory_s10x.json`. Scenario: if the lift is edited again before it is committed, nothing in the dataset identifies
which teacher produced `level1_walkup_v1`. Evidence: `factory_s101.json` and `episode_s104_t0022.json` on the T7 both carry that dict.
Fix: commit this diff unchanged, and note in PLAN L / PROGRESS that `level1_walkup_v1` (17:56 on) = that commit.

### F2 (minor, docs) Comment overstates the contrast with the old lift
`g1sim/ball_grasp.py:68-69` lists "hands 2 cm closer to the shoulders, elbows raised" as defects of the fixed-orientation lift.
The new lift also brings the wrists 1.7 cm closer (0.261 -> 0.244 m vs old 0.238) and raises the elbows 1.4 cm (vs 3.1 cm).
This is geometric: the ball starts 0.13 m below the shoulder line and rises straight up, so it gets closer to the shoulder. The real
gains are forearm yaw (+-10 -> ~0 deg), forearm pitch 39 -> 31 deg and upper arm 34 -> 40 deg down. Suggest wording it as "less".

### F3 (nit) Bisection saturates silently if the bracket ever misses
`g1sim/ball_grasp.py:376-380`: if the rise at 0.6 rad were below LIFT, the loop converges to 0.6 rad and lifts less, with no
signal (only MIN_LIFT would catch it). Not reachable in practice: worst sampled case needs 14.9 deg; 0.6 rad gives >= 0.215 m.
A miss needs the ball within ~0.12 m in front of the shoulder line, which the arms cannot grasp. Optional: assert the rise at `high`.

### F4 (nit) No direct test of `_shoulder_lift` geometry
Covered only end-to-end (pass/fail). A cheap kinematic unit test (ball +LIFT, ~0 horizontal, palm gap kept, axis = torso lateral)
would pin the property the comment claims. `bracket_probe.py` is a template.

### Observation (no action) Lowest-margin fixed-scene trial got 4 mm lower
`runs/l1w_s5_g_rand0.json` trial 40: lift 0.0694 (HEAD) -> 0.0653 (new), MIN_LIFT 0.06; reproduced deterministically with both
versions (`trial40.py`). The ball is held by palm + pinky only in both versions, so the slip is pre-existing. Mean over the paired
50 starts: +4.5 mm. It is the fixed scene, not the walk distribution being recorded.

## Not run / limits
- Full test suite and 67 sweeps not re-run (hard rule); I relied on `runs/l1w_s5_tests_v2.log` and the sweep/gate JSONs.
- The user's video viewer itself was not opened. Checked instead: Lab View server, live frames, T7 files.
- Side effects of the allowed probe runs: MuJoCo appended its attach warnings to the gitignored repo `MUJOCO_LOG.TXT`;
  `table_scene.write` atomically rewrote the volleyball_walk scene XML in `~/dev/g1-ext/g1_brainco_sim` (same content). No repo
  file in git was touched, nothing was written to the T7.
