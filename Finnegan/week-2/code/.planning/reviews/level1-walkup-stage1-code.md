# Code review: level1-walkup Stage 1 (27-D action space, volleyball_walk world, grey ball)

Reviewer: independent gate (did not author the code, no author reasoning seen). Date 2026-10-08.
Scope: g1sim/action_space.py, episode.py, table_env.py, table_scene.py, ball_grasp.py (waist via action[26]),
scripts/58_teleop_check.py, scripts/66_volleyball_scene_check.py, tests listed in the Stage 1 task block,
tests/test_integration_volleyball_walk.py (new). Not reviewed: perception.py / test_unit_perception.py (Stage 2).

## Verification re-run (own logs, author logs left untouched)

| Check | Command / log | Result |
|---|---|---|
| All three envs | `scripts/run_tests.sh > runs/l1w_s1_rev_tests.log` | project 89 passed / 56 skipped; g1-ext 146 passed / 2 skipped; openpi 10 passed; all exit 0 (no D2 failures at all) |
| Walk-world + touched tests, verbose | `runs/l1w_s1_rev_walk_tests.log` | 23 passed, all 6 new walk tests PASSED (not skipped) |
| Fixed-scene 67 nominal | `runs/l1w_s1_rev_67.log` | 10/10, ACCEPTANCE PASSED, lift 8.8 cm, end 3.6 cm from centre |
| Fixed-scene 67 jitter | `runs/l1w_s1_rev_jitter.log/.json` | 50/50 (done: >= 48/50) |
| Bottle demo 27-D | `64_demo_factory.py --demos 1 --name s1_27d_rev`, `runs/l1w_s1_rev_64_validate.log` | state/actions (615, 27), `validate == []`, success True; state[:,26] max 0.0048 rad |
| Convert v2.1 | `openpi_g1.convert ... --repo-id local/s1_27d_rev`, `runs/l1w_s1_rev_convert.log` | 1 episode, 615 frames, skipped {}; info.json state/actions shape [27], last name kWaistYaw |
| Hand-slice grep | `grep -rn "14:\]" g1sim scripts tests openpi_g1` (`runs/l1w_s1_rev_grep.log`) | no match |
| Extra probes | `runs/l1w_s1_rev_probe{,2,3}.log` (scripts in session scratchpad `rev/`) | see findings |

Not run: `scripts/66_volleyball_scene_check.py` itself (it rewrites docs/*.png). Its check 5 logic was replicated in
`rev/probe.py` on the fixed scene: 2500 ball pixels at 320x200 (> 2000), not on the border; author log
`runs/l1w_s1_66.log` shows SCENE CHECK PASSED with 2430 pixels.

## Verdict 1: did the code do what the plan said? YES

Every Stage 1 item (a)-(f) is present and behaves as specified:
- (a) DIM 27, `kWaistYaw`, `SLICES["waist"]`, READY_POSE/DELTA_MASK 27th entry, `join`/`from_sim` take waist.
- (b) `episode.HANDS` replaces `14:` in episode.py:94, 58_teleop_check.py:132, test_unit_openpi_policy.py:61; waist checked as delta.
- (c) `state()` appends waist qpos, `apply()` reads actions[26] into `waist_yaw_target`, `set_waist_yaw` deleted (no caller left),
  `render_depth` through one renderer with noise, `reset()` re-seeds `noise_rng = default_rng([seed, k, 1])`.
- (d) `volleyball_walk` with free joint kept, mocap `pelvis_hold` + weld (relpose identity, solref 0.01/1, inactive in file),
  `GROOT_STANCE` with source comment, `STAND_Z`, `R_STOP = 0.45`, basket offset after `model_variation.apply`.
  Section I: ball rgba 0.55 grey in both volleyball scenes (shared VOLLEYBALL_OBJECTS), 66 check 5 colour-free (segmentation).
- (e) `_set_up` poses base, legs, waist roll/pitch, mocap, weld, head tilt before the settle; `hold_pelvis`, `stance_clearance`,
  `snapshot_handover`/`restore_handover`; `_start_is_clear` includes static bodies (bottle has `static=()`, so no bottle change).
- (f) tests updated; new walk tests cover every listed criterion.
- ball_grasp: `_turn_waist` now drives `self.action[WAIST]`; `_reset_ik`/`_correct_sag` still read `env.waist_yaw_target`, which
  `apply()` sets every tick, so the semantics are identical (67 10/10 and 50/50 confirm).

Measured against <done>: all met.

## Verdict 2: was the plan right? PARTLY (one major carry-forward for Stages 3/4)

## Findings

1. **MAJOR (plan, Stage 3/4 thresholds) - nominal stop clearance is 0.031 m, below the plan's own sampler threshold 0.04 m.**
   Evidence: `runs/l1w_s1_rev_probe.log` / `probe3.log`: `stance_clearance(WALK_STANCE)` = 0.0314 m, closest pair
   right_knee_link-table_top; at pelvis-edge 0.21 = +0.0054, 0.20 = -0.0046, 0.17 = -0.0346. Physics agrees: reset at
   pelvis-edge 0.20 produces a real `('right_knee_link', 'table')` contact, 0.21 does not. So the FK in the code is right and the
   plan's basis is off: PLAN Assumptions/B1 cite `fk3.out` "+0.019 at 0.20, -0.011 at 0.17", but `fk3.py` used a symmetric
   crouch hip -0.35 / knee 0.70 with zero pelvis pitch, while the plan's own GROOT_STANCE (right hip -0.402) flexes the hip more
   and moves the knee ~2.4 cm closer. Impact: PLAN A "keep the draw only if stance_clearance >= 0.04" rejects the nominal
   layout and every layout whose stop puts the pelvis < ~0.245 m from the edge; the walk envelope ">= 0.02" leaves only 1.1 cm
   of forward arrival error (fwd +0.01 -> 0.0214, +0.02 -> 0.0114), versus Stage 4's accepted p95 arrival error of 4 cm.
   Also the Stage 1 test `>= 0.03` (test_integration_volleyball_walk.py:96) passes by 1.4 mm and will likely flip when Stage 4
   overwrites GROOT_STANCE. Fix (plan, before Stage 3 builds): re-derive R_STOP and both thresholds from `stance_clearance`
   with GROOT_STANCE (e.g. R_STOP ~0.46-0.47, or sampler >= 0.03 / envelope >= 0.0 plus a contact check), re-check reach
   (teachback) at the new distance, and make the Stage 1 test assert against the threshold the sampler actually uses.

2. **MINOR (info for Stage 4) - the weld carries ~71 % of the robot's weight in the provisional stance.**
   Evidence: `probe.log`: robot mass 38.5 kg (378 N), floor normal force 109.8 N after 2 s, weld constraint force z 268 N.
   The "feet within 3 mm" test passes but does not show the legs carry the robot. Impact: none for the welded grasp; at the
   band release in Stage 4 the robot will settle/drop by the provisional STAND_Z error. Fix: Stage 4 re-measure (already
   planned); optionally add a floor-load assertion (e.g. >= 80 % of weight on the feet) once GROOT_STANCE is measured.

3. **MINOR (test gap) - `restore_handover` is only proven in the same TableEnv; in a fresh TableEnv images are not bit-exact.**
   Evidence: `probe2.log`: fresh-env restore gives identical qpos/xpos/cam_xpos/mocap, but 1 pixel differs by 1 grey level
   (noise on and off); same-env restore is bit-exact. The docstring (table_env.py:226) claims "also in a fresh TableEnv".
   Cause is the second GL context's rasterisation, not the state. Impact: Stage 5's `--from-handover` replay runs in a new
   process; any bit-exact image assertion there will fail. Fix: test fresh-env restore for qpos bit-exact and images within
   1 grey level, and say so in the docstring.

4. **MINOR (test gap, out-of-list file) - `tests/test_integration_openpi_env.py:47` `np.full(26, np.nan)` no longer tests NaN
   rejection** (it is now rejected for the wrong length by openpi_env.py:40). Fix: `np.full(action_space.DIM, np.nan)`.

5. **NIT - stale "26-D" docstrings** in openpi_g1/policy.py:6,8,12,69, openpi_g1/stand_in.py:6, g1sim/openpi_env.py:5,
   g1sim/teleop.py:7,100,127, scripts/64_demo_factory.py:7, scripts/60_openpi_closed_loop.py:7. Behaviour is correct (all
   read `DIM`/`NAMES`). Fix when those files are next touched.

6. **NIT (Stage 4/6 note) - the snapshot is not npz-native**: `noise_rng` is a PCG64 state dict with 128-bit ints and
   `variation`/`hand_target` are nested dicts. JSON with an array default works (probe). Stage 6 needs that wrapper.

No regressions found in the bottle, fixed-volleyball, openpi or teleop paths.

## Overall: GO for Stage 1
Stage 1 code is correct and meets its done criteria. Finding 1 is a plan defect that must be fixed in PLAN A/B (R_STOP and
the clearance thresholds) before Stage 3's sampler and Stage 4's envelope are built; it does not block Stage 1.
