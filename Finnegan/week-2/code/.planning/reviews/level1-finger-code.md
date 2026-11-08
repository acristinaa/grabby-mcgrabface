# Review: Level 1 little-finger clearance, finished code (against `.planning/level1-harden/FINGER-PLAN.md` rev 1)

Stage: finished review. Reviewer: independent; I did not write the code or the plan and have not seen the builder's reasoning.
Date: 2026-10-27. Scope: `git diff` of `g1sim/ball_grasp.py` and `scripts/67_volleyball_pick_place.py`, the new
`g1sim/hand_clearance.py` and `tests/test_integration_hand_clearance.py`. Track C files (85, 87, `arm_clearance*`, `sim_host`
and their tests) are excluded. Reviewer runs are in `/tmp/l1day1/rev/rev_lfc_*` and `runs/rev_lfc_*` (gitignored). The only repo
files I changed are this review and `.planning/STATUS.md`.

## Verdicts

- **Implementation fidelity: GO.** The code does what Task 1 and Task 2 say. It also folds in plan-review minors 4 (stale
  docstring lines 8-9 and the `FINGERS_CURLED` note) and 7 (test 1 asserts which bodies are present and which are absent).
- **Plan correctness: GO.** Every gate reproduces to the bit on my own re-runs. The clearance gain is real (pooled p5 0.65 -> 3.65 cm,
  min 0.05 -> 3.18 cm), and nothing I re-ran lost a start.
- **Overall: GO.** No behaviour bugs. Four minors, none blocking.

## What I re-ran (all on the changed tree, g1-ext env, mujoco 3.13)

| Gate | My run | Result | Builder log | Match |
|---|---|---|---|---|
| G1 fixed nominal | `/tmp/l1day1/rev/rev_lfc_fixed_nominal.{json,log}` | 10/10, exit 0 | `runs/l1h_finger_fixed_nominal.json` | 10/10 trial records identical |
| G3 fixed randomized seed 0 | `rev_lfc_fixed_rand0` | 50/50 | `l1h_finger_fixed_rand0` | 50/50 identical |
| G3 fixed randomized seed 1 | `rev_lfc_fixed_rand1` | 50/50 | `l1h_finger_fixed_rand1` | 50/50 identical |
| G5 walk seeds 10-13 `--clearance` | `runs/rev_lfc_walk_s1{0..3}.json` | 4 x 50/50 | `l1h_finger_walk_s1{0..3}` | every trial record and `clearance_summary` identical |
| G6 walk seeds 0, 1 `--clearance` | `runs/rev_lfc_walk_s{0,1}.json` | 2 x 50/50 | `l1h_finger_walk_s{0,1}` | identical |
| G7 pooled clearance (G5 + G6) | `/tmp/l1day1/rev/cmp_lfc.txt` | n 300, p5 3.65 cm, min 3.18 cm, pinky p5 3.70 cm, pinky min 3.41 cm | `runs/l1h_finger_g7.log` | identical; gate p5 >= 3.5, min >= 3.0, pinky p5 >= 3.5 passes |
| G9 lift min | builder `runs/l1h_finger_g9.log` (300/300, 0.0819 m) | | | same rows as mine (identical records) |
| New tests + `test_integration_volleyball_grasp.py` | `/tmp/l1day1/rev/rev_lfc_tests.log` | 9 passed | `l1h_finger_t2_tests_new.log` | yes |
| Old constants, walk seed 10 (Task 1 cross-check, via override runner `/tmp/l1day1/rev/runner/run67_consts.py 20 0.04 0.025`) | `runs/rev_lfc_old_walk_s10.json` | p5 0.65, min 0.05, pinky p5 0.82 cm | `l1h_finger_old_walk_s10.json` | `clearance_summary` identical, so my override runner reproduces the old tree |
| Old constants, walk seed 104 x 75 | `runs/rev_lfc_old_walk_s104.json` | 74/75, trial 72 `no_forbidden_contacts`, ball moved 17.4 cm before touch | new: `l1h_finger_walk_s104` 74/75, same trial, 15.8 cm | the one failure is pre-existing, as the plan says |
| Reach map old vs new (plan-review minor 3) | `rev_lfc_map_{old,new}{,_noclr}.log` | all 4 runs abort at trial 63 | `l1h_finger_map.log` aborts the same way | see minor 1 |

The builder's other logs, which I read but did not re-run: G2 jitter 50/50, G4 distractors 30/30, G8 82 `--teacher` 50/50 with
agree 50/50 (`runs/vla_eval_l1h_finger_teacher_seed0.json`), info seeds 101-104 299/300 with p5 3.63 cm, test 2 red on the old
constants (`l1h_finger_t1_tests_old.log`: 0.0256 < 0.035), no-flag JSON without any clearance key (`l1h_finger_noflag.json`), and
G10. The G10 before-run has exactly one FAILED line, the intended red test 2. The after-run has 300 + 511 + 10 passed and 0 failed.

## Fidelity, item by item

- `ball_grasp.py`: exactly the three constants (`CRADLE_DEG` 10, `REACH_ABOVE` 0.06, `SQUEEZE` 0.03), their comments, and docstring
  lines 3 and 8-9, plus a one-line note on `FINGERS_CURLED`. No logic changes. The constants are read at call time (`:127`, `:208`, `:219`),
  and no default argument binds them early.
- `hand_clearance.py` matches the spec. It keeps the elbow subtree via `body_parentid` and the collision geoms (contype or conaffinity),
  labels parts with `finger_of(...) or "arm"`, uses the `FORBIDDEN` bodies the scene has and `DISTMAX_M = 0.15` with its source, and gives
  `summarize` over passed trials with the numpy default percentile. The functions are small and their docstrings say why.
- 67 is purely additive. `--clearance` adds one meter per trial, called after the video grab in `on_tick`, plus `result["clearance"]`,
  `report["clearance_summary"]`, one printed line and one docstring command line. Without the flag the report has no new keys
  (verified in `l1h_finger_noflag.json`).
- The tests have "Proves / Catches" docstrings and named constants with their source, and the planted overlap uses real geometry.
  Test 2 was seen red on the old constants and is green now.

## Plan correctness, re-checked

- The sim is deterministic: 9 of 9 re-runs were bit-identical to the builder's, so the gate numbers are reproducible facts, not noise.
- What becomes reachable: every consumer of the teacher (67, 69 `record()`, 82 `--teacher`, lab view, the vla_eval tests) now runs
  the new grasp. All are covered by G1-G10, and the full suite is green. The two margins that are now tight are lift min 8.19 cm against
  the 8.0 gate and the squeeze tolerance (plan-review minor 2: fixed scene 48/50 at a 2 cm squeeze). Both were already disclosed in
  the plan review. Neither is a regression on the gate, but the PROGRESS entry should carry them.

## Findings

### Behaviour bugs
None.

### Minors
1. **The reach map cannot finish, and the status line "reach map unchanged 52/65" overstates what ran.** `67 --scene volleyball_walk --map`
   aborts at trial 63 with `AssertionError: shoulder lift: no angle in the search range raises the ball by LIFT`
   (`g1sim/ball_grasp.py:385`). No `runs/l1h_finger_map.json` is written; only the log exists. The crash is **pre-existing**: it is the same
   with the old constants (my override runs `rev_lfc_map_old*.log`), so it dates from the shoulder lift (commit 450fd4b), after the 10-09
   map. Over the 62 trials that do run, old and new constants give the same pass/fail and first-failed check per trial (52 passes). The
   10-09 map's trials 63-65 were failures, so "52/65" is consistent, but it was not measured. Not caused by this change and out of scope.
   Record it as "first 62 of 65 identical; the map aborts at 63 on a pre-existing assert" and fix the assert later (for example, make an
   unreachable lift a failed trial rather than a crash).
2. The `CRADLE_DEG` comment says the grasp "held ... in 600 walk + 190 fixed-scene starts". Measured, it is 599/600 walk: s104_t0072 fails,
   identically with the old constants. The wording came from the plan; "599/600 (the 1 failure pre-existing)" would be exact.
3. `ClearanceMeter.result()` raises a TypeError (`dict(None, ...)`) if `update()` was never called. This is unreachable from 67, because the
   teacher always ticks. Noted only; no change needed.
4. Side effect, already known (plan-review minor 6): fixed-scene runs rewrite `docs/volleyball_teacher_squeeze.png` (gitignored). My
   re-runs of rand0 and rand1 rewrote it with the same deterministic image the builder's runs produced.

## Checks I could not run / did not run
- I did not re-run G2, G4, G8, the info seeds 101-103 or the full `scripts/run_tests.sh`. I read the builder's logs for those. Every
  gate I did re-run matched bit for bit, and the sim is deterministic, so I accept them.
- I did not run a student (run2) eval. It needs v2 demos (the T7 is not mounted) and training. This remains a teacher-level result, as
  the plan says.
