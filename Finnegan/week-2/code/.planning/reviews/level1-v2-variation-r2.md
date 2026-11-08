# Review round 2: Level 1 v2 variation (plan humming-squishing-treasure A1-A7, B)

Reviewer: independent (did not write the code, has not seen the author's reasoning). Date 2026-11-01.
Scope: all uncommitted changes in the canonical repo (11 modified files, 5 new incl. the round-1 review), after the fix round
(scripts/69, 79, 82 and requirements-record.txt touched 12:38-12:39; gate-relevant code table_env/randomization/table_scene/
ball_grasp unchanged since 12:12:54, so the gate logs written 12:14-12:23 are still valid for the code under review).

## Verdicts

1. **Did the code do what the spec said? Yes.** A1-A7 implemented as specified; B ran on the NEW config; the two failing bars were
   reported with numbers and nothing was loosened or shrunk. The fix round closed round-1 minors m1, m3, m5 and most of m2 correctly.
   No code bug found in this round either.
2. **Was the spec right? No, not as it stands.** Two decided numbers fail the plan's own acceptance bars (unchanged from round 1):
   - G7 with latency up to 100 ms: pooled p5 **3.47 cm** vs bar 3.50 cm (min 3.18 >= 3.0 and pinky p5 3.51 >= 3.5 pass).
   - Camera extremes at +-0.025 m / +-5 deg: worst **-12.5 px** (left palm) vs bar +10 px. New in this round: the ball/basket
     points alone also leave the image, worst **-11.2 px**, so it is not only the palms.
3. **Overall: NO-GO for section C (smoke + v2 recording) with this config.** This is a decision for Finnegan, not another fix round:
   the code needs no change. Once he accepts the two numbers or picks other ranges (then re-run G5-G7 and 88 --camera-extremes),
   the code side is GO.

## What I re-ran myself

| Check | Result | Log |
|---|---|---|
| 3 new tests (5 cases) + existing bit-exactness tests (walk snapshot/restore, restore in a fresh env, distractor variation replay) + all 5 vla_eval tests (69 sequence, seed-101 v1 handover vs T7, replay through the wire, teacher on handover, stale observation) | 13/13 pass | runs/l1v2_r2_tests.log |
| FIFO vs my own model (which command is the target at every physics step, 7 ticks), L = 0, 1, 5, 9, 10, 11, 15, 19, 20, 21, 25, 29, 30 substeps, one env reused across L | 0 mismatches for every L; nothing leaks from one episode into the next (L=11 left 1 pending command, the next reset cleared it) | runs/l1v2_r2_checks2.log |
| 2 real v1 demos from the T7 not used in round 1 (s102_t0011, s102_t0017, latency 9 substeps = the v1 maximum), replayed in a fresh env with the NEW apply and with a copy of the OLD apply | full qpos new == old on all 726 ticks; recorded float32 state matched on 726/726 ticks | runs/l1v2_r2_checks.log |
| Texture round trip, walk scene, v2 config: random start -> JSON -> fresh env (other seed) reset(variation) | matid equal; only floor + table_top changed; ball and basket geoms unchanged; noise-free head render max diff 0; nominal reset clears | runs/l1v2_r2_checks2.log |
| 88's projection on starts 0, 6, 7 | reproduces the builder's numbers exactly (24.9 / -12.5, 36.6 / -1.2, 43.6 / 8.0 px). Cross-check against a segmentation render: the projected ball pixel lands on the ball or on the left hand in front of it (ball disc 2532 of 3566 px visible at handover = hand occlusion, explains the 9-15 px centroid offset); at the worst corner centroid and projection agree within 1-5 px | runs/l1v2_r2_camera.log, runs/l1v2_r2_camera2.log |
| Fix-round demo (69 seed 101, v2) sidecar | task "place the volleyball into the basket" in npz and sidecar; sha256 equals a recompute of randomization.load(); versions python 3.12.14, mujoco 3.13.0, mink 1.3.0, numpy 2.2.6, opencv 4.13.0 (from cv2) | inline |
| 82 --from-handover report of the fix round | header prompt "per demo (npz task)", header randomization path, per-row prompt = the demo's sentence | runs/l1v2_fix_82_report.json |
| requirements-record.txt closure over installed metadata | all 29 pins match the env; 3 transitive deps still unpinned (m1 below) | runs/l1v2_r2_pins_closure.log |

## Gate logs read (builder's), on the NEW config

Commands in runs/l1v2_gates_driver.log are FINGER-PLAN section 6's, all exit 0. Walk JSONs: latency max 0.0958-0.0999 s per seed,
textured 49-50/50 (and 74-75/75), head offset up to 0.0250 m, tilt up to 5.00 deg; fixed randomized runs latency up to 0.098/0.099 s
and 50/50 textured. Gate numbers recomputed from the JSONs where possible:

| Gate | Value | Bar | Status |
|---|---|---|---|
| G1 | 10/10, ACCEPTANCE PASSED | 10/10 | pass |
| G2 | 50/50 | 50/50 | pass |
| G3 | 50/50, 50/50 | 50/50 each | pass |
| G4 | 30/30 | 30/30 | pass |
| G5 | 200/200 (recomputed) | >= 199/200 | pass |
| G6 | 100/100 (recomputed) | 100/100 | pass |
| G7 | p5 3.47 cm, min 3.18, pinky p5 3.51 (recomputed with hand_clearance.summarize, identical) | p5 >= 3.5, min >= 3.0, pinky p5 >= 3.5 | **FAIL (p5)** |
| G8 | 50/50, agree 50/50 | 50/50 | pass |
| G9 | lift min 8.36 cm (recomputed) | >= 8.0 | pass |
| G10 | before 300/511/10, after 300/516/10, after fix 300/516/10, 0 FAILED/ERROR lines | no new failure by name | pass |
| info s101-104 | 300/300 | report | - |
| Camera extremes | -12.5 px | >= 10 px | **FAIL** |

Mutation logs: each new test fails once against its mutation (clamp to one tick -> test a; one step late -> test b x3; matid not
restored -> test c); the tree now holds the unmutated code (my run is green, the diff is the intended code).

## Blocking findings (plan level, Finnegan decides; no code change)

B1. **G7 p5 3.47 cm < 3.50 cm with latency 0-100 ms.** Trigger: any v2 run; the starts with latency 67-100 ms have p5 3.38 cm
    (0-33 ms: 3.71, 33-67 ms: 3.54; my recompute from the walk JSONs). Every pick-and-place still succeeds. Decision data (subsets
    of the same 300 trials, not fresh draws): latency < 83 ms -> p5 3.49 cm; < 67 ms -> 3.67 cm. Options: accept 3.47 cm as the
    v2 bar, or cap the latency (about 0.067 s would pass, to be confirmed with a fresh G5-G7 run).
B2. **Camera extremes fail at +-0.025 m / +-5 deg, for the ball/basket too.** Trigger: worst corners (offset [+,-,-], tilt [-,-,+])
    at the lift/centre moments. Worst margin palms -12.5 px, ball/basket -11.2 px (runs/l1v2_r2_camera_ranges.log). Decision data
    over the same 10 starts x 64 corners: offset 0.025 / tilt 4 deg -> -6.6 px; 0.025 / 3 deg -> -2.1 px; 0.015 / 5 deg -> -3.9 px;
    v1 range 0.005 / 1.5 deg -> +16.7 px. Note the bar is a worst-corner bar (all six axes at their limits at once); round 1
    measured 0-12 % of random draws per start below 10 px. Options: accept with that caveat, or pick a smaller range (re-run 88).

## Minor findings

m1. requirements-record.txt still leaves 3 transitive deps unpinned: markdown-it-py and mdurl (needed by rich), zipp (needed by
    etils[epath], which mujoco declares). pip installs them anyway; none touches the sim numbers. One line each would close it.
m2. g1sim/table_scene.py:139 `TEXTURE_SURFACES` is defined but never read; the surfaces come from config image.texture_surfaces.
    A dead constant next to the live list can drift (readability).
m3. scripts/82_vla_eval.py:167 is 154 characters (the new prompt expression); hard to read in one line.
m4. (carried from round 1 m4, outside the spec) scripts/83_open_loop_check.py sends policy_wire.TASK for frames of demos that carry
    another phrasing; tests/helpers/make_robot_day_data.py:99 exits if dataset episode 0's task != TASK. With seeds 109-112, v2
    episode 0 is seed 109 trial 0 -> TASKS[(109 + 0) % 5] = TASKS[4], so that helper would refuse a v2 dataset.
m5. (carried, info) With textures on, the texture draw shifts every later random draw, so seed-0 held-out starts under v2 are other
    layouts; the run1 comparison must use --randomization configs/randomization_v1.json (round 1 verified it reproduces run1's start).
m6. (info) Each surface keeps its own look with probability 1/13 only (None + 12 materials, equal weights); the original wooden
    table appears in about 8 % of v2 demos. Fine for variation, worth knowing when reading the contact sheet.
m7. (test-only, not a NO-GO reason) test (a) asserts `env.latency == 0` after the shifted run and covers whole ticks only; my FIFO
    check covered L = 0..30 including 11, 15, 19, 21, 25, 29.

## Round-1 minors: status

m1 (82 report prompt) fixed: header "per demo (npz task)" in --from-handover mode, per-row prompt, docstring updated.
m2 (pins) mostly fixed: 7 pins added incl. PyOpenGL; 3 remain (m1 above). m3 (opencv version) fixed: cv2.__version__.
m5 (79 docstring) fixed. m4, m6, m7, m8 unchanged (outside spec / info / test-only).

## Lens notes

- Latency FIFO: no off-by-one (per-substep model, 13 latency values), pending reset in _set_up on every reset and before
  restore_handover refills it; deterministic. apply() copies body_target, which under L > 10 is an older command, but every slot
  that can differ (14 arm joints, waist yaw) is overwritten from the action; the other slots are constant from _set_up.
  `waist_yaw_target` is still set immediately (the commanded value, as before), which is what ball_grasp reads.
- <= 1 tick bit-identity: test (b) plus 2 more real v1 demos at the v1 maximum latency, full qpos new == old for 726 ticks.
- Textures: in the variation dict, restored by ModelVariation (geom_matid in FIELDS), ball/basket untouched; old variations
  without "textures" load via .get; materials keep r <= g, b.
- Wrist cameras: not recorded (episode.IMAGE_KEYS = ("head_image",)), not used by 69/79/82/policy_wire; the scalar range change
  only matters for the head camera. Fine as specified.
- Config/flag wiring: 82 (both load() calls) and 69 take --randomization; v1 = old values + textures off.
- Prompt reaches the dataset: npz, sidecar, 79 per-frame task (79:85 ep["task"]), 80 checks npz == dataset and membership in TASKS;
  82 --from-handover sends the demo's own sentence, which the replay server compares against the npz (policy_wire:199).
- Provenance: sha256 of the effective (loaded) config + versions, correct.
- Readability: small functions, named constants with source, why-comments; no readability major (m2, m3 are small).
