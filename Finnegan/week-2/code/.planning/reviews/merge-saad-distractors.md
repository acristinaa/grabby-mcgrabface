# Review gate: merge origin/saad/table-distractors into main (branch integrate/saad-distractors, uncommitted)

Reviewer: fresh-context reviewer (opus), 2026-10-09. Read-only; no sims or tests run (author reports all green, see task).
Scope: g1sim/ball_grasp.py, g1sim/table_env.py, scripts/68_walk_to_table.py, g1sim/table_scene.py (SCENES, build_spec tail),
compared with `git show main:` and `git show origin/saad/table-distractors:` (merge base 9642cbf). Plan: .planning/level1-walkup/PLAN.md section J.

## Verdict 1: implementation fidelity (does the merge do what section J says): YES
- ball_grasp.py: ours kept: start `env.ready_pose` (178), straight `reach` with REACH_ABOVE 0.04 + open fingers (192-195),
  cosine `smooth` in `_move`/`_turn_waist`/`_move_joints`, waist only via `self.action[WAIST]` = action[26] (284),
  `_move_joints` over indices 0..25 (304), retract to `env.ready_pose` + face_front to `ready_pose[WAIST]` (228-229),
  settle with `_hold_action` (no IK while waist moved). Saad's kept: `base_xy/base_yaw/waist_axis` in `__init__` (168-172),
  `_in_base`/`_bearing_change` (348-358) identical to Saad's, grasp/place yaw = nominal + bearing change of stand/basket
  (185-187), hand orientation with `torso_yaw = base_yaw + grasp_yaw` (188, 192, 199, 204), `_correct_sag` torso =
  `base_yaw + env.waist_yaw_target` (330), FORBIDDEN includes `VOLLEYBALL_DISTRACTORS` (87). No `set_waist_yaw` call left
  anywhere (grep g1sim/scripts/tests); env.waist_yaw_target is set by `apply()` from action[26] (table_env.py:276), so
  `_reset_ik` (272) and `_correct_sag` (330) read the commanded waist correctly.
- table_env.py: `ready_pose` + distractors both in `__init__` (53, 55, 90-92); `reset()` guard is `randomize and free_base`
  (114) so `volleyball` randomizes, `volleyball_walk` raises; noise re-seed per reset (117-118) kept; `sample(..., scene_name)`
  (124); `_set_up` places distractors after `mj_resetData` (154-163), basket offset (144-145), head eq reset (146), arms/hands
  from `ready_pose` (174-185), free-base pose/weld (177-182). `_start_is_clear` has reach_m + static-body clearance (201-211).
  BALL_READY_POSE waist = 0 (action_space.py:61), so `waist_yaw_target = 0.0` in `_set_up` (173) agrees with `ready_pose`.
- 68_walk_to_table.py: ARMS_DOWN now 7+7+13 = 27 entries (63); ARMS_UP = BALL_READY_POSE (64); `arms["now"]` inits are
  `zeros(action_space.DIM)` = 27 (319, 353); all `np.allclose`/ramp arithmetic (106, 110, 155, 197-203, 297-307) is 27 vs 27.
  Consumers: `WalkSim.send` -> `lerobot_arm_action` uses `[:14]` only (action_space.py:104-106); `ArmSwing.apply` uses
  indices 0/7/3/10 (arm_swing.py:58). No 26/27 mix found.
- table_scene.py: `volleyball` has ready/reach_m/distractors; `volleyball_walk` has free_base/stance/ready, no distractors
  (matches J); build_spec adds distractors only from `setup.get("distractors")`, stand + basket for both volleyball scenes.

## Verdict 2: plan correctness (are the merge decisions in J right): YES
- Bearing-change geometry generalizes our fixed +/-35 deg: nominal stand/basket coordinates are pelvis-frame at the
  nominal fixed pelvis (0, 0, yaw 0), so the change is exactly 0 on the nominal fixed start. Teacher is constructed per
  trial after reset (67:152), so base pose is current.
- Guarding randomization on free base is correct: ModelVariation moves `body_pos[pelvis]` (randomization.py:166), which a
  free joint overrides.

## Findings (blocker/major)
None.

## Notes (minor, not gating)
- tests/test_integration_distractors.py:91 drives `action_space.READY_POSE` in the volleyball scene, whose start is
  BALL_READY_POSE, so the arms travel during the "distractors stand still" check. Fix: `env.apply(env.ready_pose)`.
- On `volleyball_walk` (WALK_STANCE x = -0.036) the bearing change is no longer 0: grasp/place yaw differ slightly from
  +/-35 deg compared with main. This is the intended behavior (geometrically correct), but the walk-scene 67 numbers on main
  were taken with the fixed 35 deg; the author's green runs cover 67 fixed scene and 68, not explicitly 67 --scene volleyball_walk.
  Not run by me.

## Checks not run
No sim or test execution (by instruction). Gate relies on the author's reported runs for runtime behavior.

## Overall: GO
