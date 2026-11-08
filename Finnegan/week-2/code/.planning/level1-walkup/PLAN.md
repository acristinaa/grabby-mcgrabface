---
phase: level1-walkup
plan: 01
revision: 2 (after reviews level1-walkup-sim-feasibility.md and level1-walkup-data-learning.md, 2026-10-08)
depends_on: []
files_modified:
  - g1sim/action_space.py
  - g1sim/episode.py
  - g1sim/table_env.py
  - g1sim/table_scene.py
  - g1sim/ball_grasp.py
  - g1sim/randomization.py
  - g1sim/navigation.py
  - g1sim/perception.py
  - g1sim/labview.py
  - g1sim/walkup.py                      # new
  - configs/randomization.json
  - scripts/58_teleop_check.py
  - scripts/67_volleyball_pick_place.py
  - scripts/69_volleyball_demo_factory.py  # new (68 is Saad's walk to the table; renumbered 2026-10-14)
  - scripts/79_convert_smolvla.py          # new (was 69)
  - scripts/80_validate_dataset.py         # new
  - scripts/81_smolvla_server.py           # new
  - scripts/82_vla_eval.py                 # new
  - scripts/83_open_loop_check.py          # new
  - scripts/train_smolvla.sh               # new
  - tests/test_unit_action_space.py, tests/test_integration_openpi.py, tests/test_unit_openpi_policy.py,
    tests/test_integration_volleyball_scene.py, tests/test_integration_volleyball_grasp.py, tests/test_unit_navigation.py
  - tests/test_unit_perception.py, tests/test_integration_volleyball_walk.py   # new
---

# Level 1 walk-up: walk to the table, find ball and basket, put the ball in the basket (sim, then VLA)

Repo root (all paths below are relative to it):
`/Users/finneganadam/Documents/Projekt Tim and Lukas/Cursor/Software Engineering/SAP Robot Project/First Ideas/UnitreeG1-Robot`.
Sim env `~/dev/g1-ext/.venv` (`PY=~/dev/g1-ext/.venv/bin/python`), SmolVLA env `~/dev/g1-ext/smolvla-test/.venv` (`SV=.../bin/python`).
User decisions 1-3 (script walks, VLA = arms + hands + waist yaw, camera check for the walk-up, everything random) are fixed.
Probe scripts cited as "probe" live in the session scratchpad
`/private/tmp/claude-501/-Users-finneganadam-Documents-Projekt-Tim-and-Lukas-Cursor-Software-Engineering-SAP-Robot-Project-First-Ideas/a243d53c-3b76-4e71-b454-57420330ec27/scratchpad/`
(`fk.py`, `fk2.py`, `fk3.py`, `slab.py`, `stand_*.log`, `teachback.py/.out`) and `/private/tmp/claude-501/walkprobe/` (P1-P3). They are
not repo code; every number taken from them is re-measured by a repo test or gate below.

## Assumptions
- Assuming: the grasp phase runs with the pelvis held by a weld (a "virtual harness") instead of GR00T balancing. On the real robot,
  Unitree's own controller balances; in sim, GR00T owns the waist (`gr00t_locomotion.py:199-206`), which conflicts with "VLA owns waist yaw".
  Ruling U1.
- Assuming: GR00T can be called synchronously, without DDS. P3 did that (`walkprobe/p3.log`: arrived, 9.3 cm, 31x real time, READY arms).
  GR00T *stands* at dt 1/300 with 6 substeps per 50 Hz tick (`stand_0.0033333333333_ready.log`); *walking* at 1/300 is not probed yet (Stage 4 probe).
- Assuming: GR00T's real standing posture is a crouch, not its default angles: pelvis z 0.7505-0.7517, hip pitch -0.30..-0.43,
  knee 0.68-0.72, ankle -0.24..-0.37, pelvis pitch about -1 deg (`stand_*.log`, both timesteps, both arm poses). The default angles
  (hip -0.1, knee 0.3, ankle -0.2) would put the pelvis at 0.784 (`fk2.out`). All stance constants come from the measured crouch.
- Assuming: the G1 hips are at table height (pelvis 0.751, table top 0.74). The solid sim table touches the knee/thigh when the pelvis is
  closer than about 0.20 m to the front edge (`fk3.out`: +0.019 at 0.20, -0.011 at 0.17). A thin open-front table does not help: a 4 cm slab
  still hits `right_hip_yaw_link` at 0.17 m (`slab_0.04.out`). So the robot must stop further back than the fixed scene's 0.20 m.
- Assuming: the current teacher still passes with the pelvis 0.02-0.05 m further back than nominal (r 0.43-0.46, crouch at 0.751, solid
  table): 5/5 single runs passed, lift 0.085 m, ball 1.3-4.6 cm off the basket centre with the fixed 35 deg waist (`teachback.out`).
  This is the basis for a stop radius of 0.45 m. Stage 3 re-measures it over the whole range.
- Assuming: the ball stays on its stand (z 0.912) in v1, so the lift height stays 9 cm. The basket stays aligned with the table axes (no yaw),
  so `ball_grasp.placement_checks` keeps working unchanged.
- Assuming: the SSD is the mounted `/Volumes/T7` (exFAT, 891 GiB free, checked 2026-10-08). The name is ruling U2.
- Assuming: 27-D is applied globally: `action_space.DIM` becomes 27 and waist yaw is appended at index 26. The bottle, openpi and teleop paths
  then carry waist = 0. Size checks read `DIM`/`NAMES`. The hard-coded slices are `episode.py:92`, `58_teleop_check.py:132` and the tests in Stage 1.
- Not verified (no logs on disk): the earlier grasp-reach probe claims "45 deg separation failed / 70 deg passed 60/60", "mirrored layouts passed",
  "align needed, up to 4.8 cm". They are treated as hypotheses; the Stage 3 map decides the ranges.
- Not found: `.planning/REVIEW-LENSES.md` is not in this repo. The second-order checks in section G were done by hand.

## A. Scene: one free-floating world for the walk-up and the grasp
- **New scene key `volleyball_walk`** (in `table_scene.SCENES`, own file `assets/scene_29dof_brainco_volleyball_walk.xml`):
  - Same build as `volleyball` (elliptic cone, impratio 10, solid table with front edge x = 0.20, stand, basket). Table unchanged
    (see Assumptions: an open front would not buy clearance).
  - It keeps the pelvis free joint. `build_spec` skips the deletion at `table_scene.py:108-111` when `setup["free_base"]`.
  - It adds a mocap body `pelvis_hold` (no geoms) and a weld equality `pelvis_hold` (pelvis to mocap, relpose identity, solref (0.01, 1)).
    Switching it on: put the mocap body at the current pelvis pose, then set `data.eq_active = 1`. The weld is the starting "elastic band"
    and holds the pelvis after the handover.
  - The fixed scene `volleyball` stays untouched as the regression reference for 67.
- **Standing posture = measured GR00T crouch** (constants `table_scene.GROOT_STANCE` with a source comment):
  - Provisional values from `stand_0.0033333333333_ready.log`: pelvis z 0.7505, pitch -0.96 deg; left hip/knee/ankle pitch -0.377/0.709/-0.317,
    right -0.402/0.697/-0.281; other leg joints and waist roll/pitch 0.
  - Stage 4 re-measures all 12 leg angles, waist roll/pitch and pelvis z/roll/pitch after a 4 s Balance settle in `volleyball_walk` and
    overwrites the constants (one edit, Stage 1 tests re-run).
  - `STAND_Z` is that measured pelvis z, not an FK guess. A Stage 1 test checks that the feet then touch the floor within 3 mm.
- **Arm pose: READY from the very start.** The robot spawns, stands and walks with READY arms (P3 walked like this) and the episode starts
  at READY, exactly as the validated teacher does today. No hanging start pose and no `raise` phase: both hit the table in FK (`fk.out`,
  `fk2.out`, every path tried, -0.09 to -0.15 m). `READY_POSE` stays the single start pose (also for openpi `reset_pose`).
- **Clearance by FK, not by hand:** new `TableEnv.stance_clearance(stance)` puts the pelvis at the stance with `GROOT_STANCE` legs and READY arms
  in a scratch `MjData` and returns the minimum `mj_geomDistance` between robot geoms and table, stand, ball and basket. Used by the
  sampler (reject a draw) and by the walk-up envelope.
- **Stop-pose rule** (`navigation.table_stop_pose`, pure numbers, unit-tested): heading perpendicular to the ball-basket line, facing the
  objects; position = midpoint - D·heading with D = sqrt(R_STOP² - (|basket - ball| / 2)²), `R_STOP = 0.45`. For the nominal layout this gives
  (-0.036, 0, 0 deg): pelvis 0.236 m from the edge, r 0.45, bearings ±32.4 deg.
- **Layout ranges** (world coordinates; the table is fixed, "random" = sampled per episode). Sample ball and basket on the table, compute the
  stop pose, keep the draw only if (provisional, replaced by the Stage 3 map):
  - stop-pose clearance `stance_clearance ≥ 0.04 m` (2 cm arrival error + 2 cm margin; the knee is the closest geom);
  - centre separation ball-basket 0.44-0.56 m (nominal 0.482; this replaces the old 70-100 deg angle rule, which is infeasible at
    r 0.45 with the pelvis ≥ 0.23 m from the edge: max about 77 deg);
  - stand centre ≥ 0.08 m behind the front edge, basket flush with the edge or further back, both ≥ 0.02 m from the side edges;
  - stand and basket ≥ 0.03 m apart; needed waist yaw |·| ≤ 45 deg for both objects.
  - Ball left or right of the basket at random (ruling U3; first item dropped if Stage 3 is below target).
- **Robot start pose (walk mode):** relative to the stop pose: 0.5-1.0 m further back, lateral ±0.25 m, heading ±30 deg (user: 0.5-1.0 m).
  The head camera's horizontal FOV is about 91 deg, so both objects are usually in view; larger headings exercise the search turn.
- **No goal perturbation.** The stance variation comes from the layout (each layout has its own stop pose) plus the natural arrival error.
  The config keeps a `goal_perturbation` field at 0 so it can be switched on later.
- **Stance in no-walk mode** (fallback and fast mode): stance = stop pose + an arrival offset drawn from the *measured* Stage 4 arrival
  distribution (bootstrap from the ~100 recorded offsets in the goal frame, stored in `configs/randomization.json`), plus the measured handover
  pelvis roll/pitch and waist roll/pitch. Until Stage 4 has measured it, a provisional distribution is used (forward -0.03..0, lateral ±0.02,
  yaw ±3 deg, roll/pitch 0) and every sidecar records `stance_source: provisional | measured`. Data for training is recorded only with `measured`,
  except under the late-fallback rule in F.
- **Domain randomization** (rule 3): physics and image groups of `configs/randomization.json` stay on.
  - `randomization.sample()` loops over `table_scene.OBJECTS` (lines 86, 101). It loops over the scene's objects instead; the target object gets
    the colour jitter.
  - `sample_volleyball` turns the `stand` and `objects` groups off explicitly: `ModelVariation.apply` writes the pelvis `body_pos`/`body_quat`
    (`randomization.py:137-139`), which a free joint ignores, so a logged-but-unapplied offset would appear in the sidecars. Stance lives only in
    `stance` and `walk_start`.
  - Camera noise stays at 3.0 rgb.

## B. Walk-up script (stand-in for BotBrain): `g1sim/walkup.py`
- **Synchronous GR00T**, like P3:
  - Each tick, a lowstate is built from qpos/qvel: motors in G1 order via `env.body_qadr`; IMU = pelvis `qpos[3:7]` and `qvel[3:6]`.
  - `GrootLocomotionController.run_step` writes targets 0-14 into `env.body_target[:15]`. Arms hold READY, hands open.
  - **One timestep throughout: 1/300**, GR00T every 6 substeps (50 Hz). Stage 4 starts with a walking probe at 1/300. Only if that fails:
    0.004 during the walk with 5 substeps, and `reset()` and `handover()` both set `timestep = 1/300` unconditionally; the recorder asserts
    `model.opt.timestep == 1/300` at the first recorded tick and writes it into the sidecar.
  - `GotoController(hz=50)` stepped every GR00T tick, as in P3.
- **Sequence:** band (weld on, 1.5 s) → release → stand 2 s → **look** → walk to the stop pose → **fine positioning** → stop → **handover**.
- **Look ("stop, look, go"):**
  - Head tilt `LOOK_TILT = 0.5` (the camera probe saw both objects at tilt 0.45-0.6 from 0.5-1.0 m back).
  - Render RGB and depth once with the env's single renderer; pixels → 3D with the formula from `Locator.locate`.
  - **Ball: by shape, never by colour alone** (the real ball is grey, section I): points with z > table top + 0.01 m (removes the floor
    reflection), split into clusters, least-squares sphere fit with the known radius per cluster; the ball is the cluster whose fit residual is
    lowest and below a threshold (a cylinder, box or the basket fits badly). Colour may only be a soft hint, never a filter. No `red_mask`.
  - **Basket:** by geometry (its colour matches the floor): points in the rim band z = 0.845 ± 0.015, drop those within (ball radius + 0.05 m)
    of the ball, centre of their bounding box.
  - Not found: turn in place by ±30 deg and look again, at most 3 looks, then fail with `not_found`.
- **Stop goal** = `table_stop_pose(estimates)`. The robot pose comes from sim qpos (as in 61); ball and basket only from the camera estimate.
- **Fine positioning (fixes the 8-9 cm "stops short" bias, `navigation.py:64-65, 90-91`):** after the default `arrived()` (0.10 m / 10 deg), switch to
  a second `GotoParams` instance `FINE` (pos_tol 0.025 m, yaw_tol 3 deg, max_vx 0.15, max_vy 0.10, max_omega 0.3, face_dist large so it
  sidesteps while holding the goal heading). The defaults stay unchanged for 61/62. Cap 8 s. `min_cmd` 0.08 lifts small commands over GR00T's
  deadband, so overshoot is possible; the Stage 4 probe measures it before anything depends on it.
- **Envelope after the stop** (no re-approach any more): accept the stance if `stance_clearance ≥ 0.02 m`, r of both objects 0.40-0.50 and needed
  waist |·| ≤ 45 deg (numbers in `configs/randomization.json`, updated from the Stage 3 map). Otherwise fail with `stance_outside_envelope`
  (logged, not recorded).
- **Failure handling:** fall (pelvis z < 0.5, `fell`); 30 s sim walk-up timeout (`walkup_timeout`); robot touches table, stand, ball or basket while
  walking (`walkup_contact`); detection error vs ground truth logged per episode, ball > 0.05 m or basket > 0.06 m fails with `detection_error`.
- **Handover:**
  - zero command, wait 1 s on Balance;
  - weld on at the current pelvis pose; leg targets = current leg qpos; waist roll/pitch targets = current qpos;
  - head tilt to 0.85, hold 1 s;
  - **snapshot**: `mujoco.mj_getState(m, d, mjSTATE_INTEGRATION)` (qpos, qvel, act, ctrl, mocap, eq_active, ...) plus the model-side head
    tilt `eq_data`, the timestep, the variation, and TableEnv's controller state (`body_target`, `hand_target`, `waist_yaw_target`).
    Saved in the npz/sidecar. `TableEnv.restore_handover(snapshot)` restores all of it. Logged: pelvis z/roll/pitch, waist roll/pitch, arrival error.
  - `data.time = 0`. The episode clock starts here.
- **Reproducibility:** `TableEnv.reset()` re-seeds `noise_rng = default_rng([seed, reset_index, 1])`, so trial k's camera noise no longer
  depends on how many images trials 0..k-1 rendered. `_set_up` also writes the default head tilt (`payload.set_head`), because
  `ModelVariation` does not restore `eq_data` (`randomization.py:119-120`).

## C. Grasp/place teacher from any stop pose (`g1sim/ball_grasp.py`)
- **Start = READY** (no raise phase). The teacher's existing phases start from READY as today.
- **Geometry from the sim state:** axis a = `xanchor` of the waist yaw joint; ψ = pelvis yaw; grasp waist = atan2(ball - a) - ψ;
  place waist = atan2(basket - a) - ψ. `hand_rotation(side, world_yaw)` gets ψ + waist; `_correct_sag` uses `Rz(ψ + waist_target)`.
  The nominal fixed scene gives 35.02 deg, so its behaviour is unchanged.
- **`align` phase after the turn:** after `_reset_ik`, shift both hand targets horizontally by (basket centre - carried ball centre) in 1.0 s,
  keeping the height (`teachback.out`: 1.3-4.6 cm offset without it when the stance moves 2-5 cm).
- **Waist through the 27-D action:** `_turn_waist` writes `action[26]`; `TableEnv.apply` reads it into `waist_yaw_target`. `set_waist_yaw` is removed.
- **Balance:** weld after the handover; GR00T is not running during the grasp (risk R3).
- **Checks:** the 7 checks from 67 unchanged. New walk-mode checks before them: `walkup_arrived`, `no_fall`, `objects_found`, `detection_ok`,
  `no_walkup_contact`, `stance_in_envelope`. Logged with every result: geometry (r, bearings, ψ, waist yaws), arrival error, detection error, stance.

## D. Recorder and dataset
- **Episodes start at the handover.** Walk-up frames go only into the sidecar, the video and the Lab View: the VLA never commands the legs,
  at eval it also starts after the scripted stop, and the recorded distribution equals the eval handover distribution.
- **Frame at tick t:** observation before the action (head image 200x320 from the TableEnv default `IMAGE_SIZES`, 27-D measured state) plus the 27-D
  action actually commanded (64's pending pattern).
- **27-D layout:** 0-13 arms, 14-25 hands (0 = open), 26 waist yaw (rad, absolute).
- **Prompt:** "put the volleyball in the basket". fps 30.
- **Trimming:** recording stops 30 ticks after `back_off`; the 3 s placement check runs but is not recorded. About 700 frames
  (25.0 s per data-learning review M1, minus the 1.5 s raise that no longer exists).
- **Only successful episodes are kept**; failures logged by reason.
- **Sidecar JSON:** seed, trial, mode (walk / no-walk), `stance_source`, full variation, walk-up start, goal, arrival pose and error, detection
  estimates vs truth, handover roll/pitch, timestep, geometry, teacher checks, frames, wall time. The handover snapshot goes into the npz.
- **Output:** `/Volumes/T7/Robot Project SAP/DATA Collection/g1-demos/level1_walkup_v1/`. Seeds ≥ 100 for demos; 0 and 1 reserved for eval.
- **Conversion:**
  - v3.0 for SmolVLA, new `79_convert_smolvla.py`: `LeRobotDataset.create(..., use_videos=False)` (lossless `image` dtype: no codec artefacts on the
    noisy frames, no video decode cost in training; about 30-40 GB on T7), `add_frame`, `save_episode`, `finalize`.
    Keys `observation.images.camera1` (matches `smolvla_base`'s first camera, so no rename map), `observation.state`, `action`, names from `action_space.NAMES`.
  - v2.1 for openpi (only on a GPU day): `openpi_g1/convert.py` works with DIM 27 unchanged, but needs a new repo id (`config.py:28`) and
    `HF_LEROBOT_HOME` on T7 (`convert.py:35` writes to the internal disk). `reset_pose = READY_POSE` (`config.py:80`) stays right because episodes start at READY.
- **Validation:** `80_validate_dataset.py`, Stage 7.

## E. Training and eval
- **SmolVLA run** (`scripts/train_smolvla.sh`): `lerobot-train --policy.path=lerobot/smolvla_base`, local root, `--batch_size=8 --policy.device=mps`,
  `--save_freq=5000 --wandb.enable=false --policy.push_to_hub=false`, loss logged to a file. Actions absolute; 27-D padded to 32.
- **Budget, stated honestly:** this Mac does about 9.5-10 samples/s. 20k steps × 8 = 160k samples ≈ 0.76 passes over 300 × 700 frames
  (LeRobot's SmolVLA recipe runs about 100 passes on its 50-episode example). **Night 1 is a pipeline proof, not the Level-1 result.**
  Plan: night 1 to 20k, resume night 2 to about 45k (≈ 1.7 passes). A rented GPU (pi0.5 path) changes this; then follow PLAN (i).
- **Undertraining vs generalization (decision rule):** every eval also runs 10 *training* variations from their saved handover snapshots.
  - training-variation success low as well → undertrained: continue training (resume), do **not** add demos;
  - training high, seed-0 low → generalization gap: more demos / narrower ranges.
- **Open-loop check (`83_open_loop_check.py`, per checkpoint 5k/10k/15k/20k/...):** 20 recorded frames from training episodes through the server;
  MAE of the returned chunk vs the recorded next 50 actions, per group (arms rad, hands 0..1, waist rad), next to the spread (std) of those actions.
  Gate: MAE falls with steps and ends well below the spread.
- **Policy server (`81_smolvla_server.py`):** loads the checkpoint *with its processors* (`make_pre_post_processors(pretrained_path=<ckpt>)`,
  `lerobot/policies/factory.py:150`) and calls `predict_action_chunk` (`modeling_smolvla.py:232`), so normalization, key mapping and uint8 HWC →
  float CHW happen exactly as in training. It rejects any `head_image` that is not (200, 320, 3) uint8.
- **Eval (`82_vla_eval.py`, sim env):**
  - Same `volleyball_walk` world, seeds 0 and 1. Observation exactly as 68: TableEnv default `IMAGE_SIZES` and
    `episode.policy_observation(env.observation(), prompt)` (not 67's 400x640 Lab View renderer).
  - Per trial: scripted walk-up → handover snapshot saved → VLA loop (websocket → server → chunk of 50, `--exec-steps` (default 25) run at 30 Hz).
    `--from-handover <npz|dir>` restores snapshots instead of walking (replay gate, training-variation eval, paired teacher comparison).
  - Paired comparison: the teacher runs from the *same* snapshots (upper bound), not from re-run walk-ups.
  - Timeout 45 s after the handover.
- **Event checks (no teacher phases):** lifted = max(ball z - rest) ≥ 0.06 m; carried = ball z - rest ≥ 0.06 m at some tick while the ball xy is inside the
  basket footprint; placed = the 4 placement checks from `ball_grasp.placement_checks`. **Success latches** at the first tick where all 4 placement
  checks have held continuously for 3 s with no forbidden contact before that tick; the trial then stops.
- **Metrics:** VLA success = passes / trials that reached the handover; end-to-end = passes / all trials; both with Wilson 95 % intervals
  (50 trials ≈ ±12-14 points). Failure table by first failed check, plus "furthest event reached" and "stalled" (|Δaction| < 0.005 for > 2 s before
  success; the demos contain about 6 s of near-static holds, a known BC stall cause). Walk-up failures are reported separately, never charged to the VLA.
  Also: time to placement, arrival and detection error, inference latency mean/max, chunks.
- **Lab View:** task `vla_eval` with event phases.
- **Pipeline proof before any trained model:** server `--replay <demo.npz>` sends a recorded demo's actions back; 82 restores that demo's handover
  snapshot (`--from-handover`). It must pass.

## F. Stages, order and parallel waves

| Stage | What | Depends on | Effort | Wave |
|---|---|---|---|---|
| 1 | 27-D action space + `volleyball_walk` scene + TableEnv free base, weld, snapshot/restore, noise re-seed, `stance_clearance` | – | 6-7 h | 1 |
| 2 | Perception: ball sphere fit + basket rim | – | 2 h | 1 (parallel to 1) |
| 3 | Reach map, then teacher from any stance + layout sampler + 67 flags + 200-trial sweep | 1 | 6-8 h | 2 |
| 4 | Walk-up: 1/300 walk probe, fine positioning, 100 arrivals measured, look, handover; `67 --walkup` | 1, 2; wiring into 67 after 3 | 5-7 h | 2 (parallel to 3) |
| 5 | Policy server (processors) + eval script (`--from-handover`, event checks, denominators) + replay check | 1 (contract: 27-D, image key, snapshot) | 4-5 h | 2 (parallel) |
| 6 | Demo factory 68 + Lab View labels | 3 (walk mode needs 4) | 2-3 h | 3 |
| 7 | Record 300 + convert v3.0 + validate | 6 | 2 h build + 1.5-3 h run | 4 |
| 8 | SmolVLA smoke + night 1 + open-loop check + resume night 2 | 7 | 1.5 h + two nights | 4 |
| 9 | VLA eval (50 seed-0 + 10 training variations) + failure analysis | 4, 5, 8 | 2 h per checkpoint | 5 |

Not parallel: 1→3 and 3→6 (same files: `table_env.py`, `ball_grasp.py`, 67). All Lab View label edits sit in Stage 6.

**Timeline against the Potsdam day plan (PROGRESS.md NEXT STEP).** Honest view: one extra night of training and the walk-up push everything by about a day.

| Day | Plan |
|---|---|
| Wed 10-08 (rest) | Stages 1 + 2 (wave 1). Answer U1-U5. |
| Thu 10-09 | Stages 3, 4, 5 in parallel. Stage 4 publishes the measured stance constants and arrival distribution first (needed by no-walk mode). |
| Fri 10-10 | Stage 6 smoke, Stage 7 record (about 1-1.5 h in 4 processes) + convert + validate, Stage 8 smoke + **night 1**. **Data never waits for the walk-up:** if Stage 4 is not green by Thu 20:00, record in no-walk mode with the measured distribution (or, if none was measured, the provisional one, flagged). |
| Sat 10-11 | Open-loop check of the 5k-20k checkpoints; pipeline-proof eval (10 training variations + 20 seed-0 trials); **night 2** (resume to ~45k). |
| Sun 10-12 | Stage 9 full eval + failure analysis. Level 2 (colour/lighting DR, second ball + prompt) only if Level 1 VLA ≥ 50 % in sim; otherwise data/training fix and night 3. |
| Mon 10-13 → Sat 10-18 | Unchanged (model gate, real adapter incl. READY arms before walking, head tilt at handover, waist via `rt/arm_sdk`, safety, tests, Potsdam). |

If late, drop in this order: v2.1/openpi conversion (only with a GPU) → mirrored layouts → walk-mode recording (no-walk fallback) → 500 demos (stay at 200-300)
→ Level 2 two-ball prompt. Never drop: the validation, the replay check, the open-loop check, eval seeds kept apart from demo seeds.

## Tasks

<task type="auto">
<name>Stage 1 - 27-D action space and the free-floating volleyball_walk world</name>
<files>g1sim/action_space.py, g1sim/episode.py, scripts/58_teleop_check.py, g1sim/table_env.py, g1sim/table_scene.py,
tests/test_unit_action_space.py, tests/test_integration_openpi.py, tests/test_unit_openpi_policy.py, tests/test_integration_volleyball_grasp.py,
tests/test_integration_volleyball_scene.py, tests/test_integration_volleyball_walk.py (new)</files>
<action>
(a) action_space: `DIM = 27`; `NAMES` gets `kWaistYaw`; `SLICES["waist"] = slice(26, 27)`; `READY_POSE` and `DELTA_MASK` get a 27th entry
(0.0; True, like the arms); `join()` and `from_sim()` take `waist`.

(b) Slices that read "hands = everything from 14 on" use `SLICES` for 14:26: `episode.py:92`, `scripts/58_teleop_check.py:132`,
`tests/test_unit_openpi_policy.py:61` (hands 14:26 absolute; index 26 checked as a delta like the arms).

(c) TableEnv: `state()` appends the measured waist qpos; `apply()` sets `body[WAIST_YAW] = actions[26]` and keeps `waist_yaw_target`; delete
`set_waist_yaw`; `render_depth(camera)` → (rgb, depth) through the same renderer with `camera_noise`; `reset()` re-seeds
`noise_rng = default_rng([seed, reset_index, 1])` (all scenes).

(d) table_scene, scene `volleyball_walk`: `free_base: True` (skip the free-joint deletion), mocap `pelvis_hold` + weld, `GROOT_STANCE` constants
(Section A, source comment), `R_STOP = 0.45`; a `basket` offset (move `body_pos[basket]` after `model_variation.apply`, like `holders`).

(e) TableEnv for a free base:
- `_set_up`: after `mj_resetData` (which resets `mocap_pos` and `eq_active`, `table_env.py:117`) write the stance into `qpos[0:7]` (z = `STAND_Z`,
  measured roll/pitch), legs/waist roll-pitch from `GROOT_STANCE` into qpos and targets, mocap at that pose, weld on, default head tilt via
  `payload.set_head`, all before the 0.3 s settle loop (`table_env.py:104-105`).
- `hold_pelvis(on)`; `stance_clearance(stance)` (Section A); `snapshot_handover()` / `restore_handover(s)` (Section B).
- `_start_is_clear` also checks the `static` bodies.

(f) Tests: the hard-coded 26 at `test_integration_openpi.py:83-84` and `test_unit_action_space.py:23,38,45`; the `set_waist_yaw` test; new walk-world tests:
- with `GROOT_STANCE` the feet touch the floor within 3 mm and the weld holds the pelvis within 2 mm / 0.2 deg for 3 s;
- a stance (-0.05, -0.05, 8 deg) is applied within 1 mm / 0.1 deg (no pull back to the origin);
- `stance_clearance` reproduces the probe: nominal stop (-0.036, 0, 0) ≥ 0.03 m; pelvis 0.17 m from the edge < 0 (knee);
- snapshot → 30 ticks → restore → 30 ticks gives identical qpos (bit-exact) and identical images with the re-seeded noise;
- two `reset()` calls with the same seed and index give identical noise regardless of renders in between.
</action>
<verify>
- `scripts/run_tests.sh > runs/l1w_s1_tests.log 2>&1`, then read the summary from the file.
- `$PY scripts/67_volleyball_pick_place.py --trials 10 --no-labview > runs/l1w_s1_67.log 2>&1`
- `$PY scripts/67_volleyball_pick_place.py --trials 50 --jitter 0.02 --seed 0 --no-labview --out runs/l1w_s1_jitter.json > runs/l1w_s1_jitter.log 2>&1`
- `$PY scripts/64_demo_factory.py --demos 1 --name s1_27d`, then `~/dev/g1-ext/openpi/.venv/bin/python -m openpi_g1.convert runs/demos.nosync/s1_27d/*.npz --repo-id local/s1_27d --overwrite`
- `grep -rn "14:\]" g1sim scripts tests openpi_g1` shows no hand slice left.
</verify>
<done>
- All three envs pass, except the 2 known D2 failures. Fixed-scene 67: nominal 10/10 and jitter ≥ 48/50.
- The bottle demo saves, validates (`validate == []`) and converts with 27-D state/action. The new walk-world tests pass.
</done>
</task>

<task type="auto">
<name>Stage 2 - Perception for the walk-up: ball sphere fit and basket rim (parallel to Stage 1)</name>
<files>g1sim/perception.py, tests/test_unit_perception.py (new)</files>
<action>
- Move the pixel→3D part of `Locator.locate` (lines 44-48) into `points_from_depth(depth, cam_pos, cam_xmat, fovy, pixels)`; Locator calls it.
- `find_ball(points, radius, table_top, max_residual)`: no colour filter; z > table_top + 0.01, cluster the points, least-squares sphere fit with
  the known radius per cluster, the lowest residual below `max_residual` wins; None below `MIN_PIXELS` or when no cluster fits a sphere.
- Extra unit tests: a grey sphere next to a red/grey cylinder and a box of similar size → the sphere is chosen; a cylinder alone → None.
- `find_basket(points, ball_centre, rim_z, ...)`: rim band rim_z ± 0.015, drop points near ball/stand, bounding-box centre; None below 100 points.
- No new classes. Unit tests on synthetic clouds: sphere seen from one side; square ring with one wall hidden; floor reflection points below the table.
</action>
<verify>
- `.venv/bin/python -m pytest tests/test_unit_perception.py > runs/l1w_s2.log 2>&1`
- `$PY scripts/54_grasp_scripted.py --trials 5 > runs/l1w_s2_54.log 2>&1` (Locator regression).
</verify>
<done>
- Ball centre ≤ 2 mm, basket centre ≤ 5 mm on synthetic data, None when missing. 54 still 5/5.
- Real-render accuracy from 0.5-1.0 m with heading ±30 deg is measured in Stage 4, not here.
</done>
</task>

<task type="auto">
<name>Stage 3 - Reach map, teacher from any stance, layout sampler (no-walk mode)</name>
<files>g1sim/ball_grasp.py, g1sim/randomization.py, g1sim/navigation.py, configs/randomization.json, g1sim/table_env.py,
scripts/67_volleyball_pick_place.py, tests/test_unit_navigation.py, tests/test_integration_volleyball_grasp.py,
tests/test_integration_volleyball_scene.py</files>
<action>
Code:
- `navigation.table_stop_pose(ball_xy, basket_xy, from_xy, radius=R_STOP)`, rule in A, unit tests (nominal → (-0.036, 0, 0); mirrored layout → mirrored pose).
- Config group `"volleyball"`: layout constraints, envelope, walk-start ranges, `goal_perturbation = 0`, `arrival` (provisional, replaced by Stage 4).
- `randomization.sample_volleyball(config, rng)`: rejection sampling over the A constraints (`stand`/`objects` groups off); returns `objects.volleyball`,
  `basket`, `stance` (stop + arrival draw), `walk_start`, physics and image groups. `TableEnv.reset(randomize=True)` for `volleyball_walk` rejects
  draws with `stance_clearance < 0.04` via the existing `max_draws` loop (fixed `volleyball` still refuses; keep that test).
- `ball_grasp` as in C: geometry-derived waist and hand yaw, `align`, waist through `action[26]`, geometry in the result.
- 67: `--scene {volleyball,volleyball_walk}`, `--randomize`, `--stance dx,dy,yaw` (for the map).

Then, in this order (no images, about 1.3 s per trial):
1. **Reach map** on the nominal layout: stance grid back 0.00-0.08 m (step 0.02, relative to the R_STOP stop), lateral ±0.04, yaw ±6 deg (45 trials),
   plus 20 layouts at separations 0.40/0.44/0.50/0.56/0.60 m. Write the passing region into the config (r, separation, waist bounds, envelope).
2. **Sweep:** 200 random trials over the config ranges.
</action>
<verify>
- `$PY scripts/67_volleyball_pick_place.py --scene volleyball_walk --map --no-labview --out runs/l1w_s3_map.json > runs/l1w_s3_map.log 2>&1`
- `$PY scripts/67_volleyball_pick_place.py --scene volleyball_walk --randomize --trials 200 --seed 0 --no-labview --out runs/l1w_s3_sweep.json > runs/l1w_s3_sweep.log 2>&1`
- Regression: fixed-scene 67 nominal `--trials 10`. `scripts/run_tests.sh > runs/l1w_s3_tests.log 2>&1`
</verify>
<done>
- Map written; config ranges taken from it (no range is kept that the map did not pass).
- Sweep ≥ 170/200 (85 %), `run_without_error` failures ≤ 4, no `no_forbidden_contacts` failure with a table/knee contact at the start.
- p95 of `carried_ball_over_basket_m` ≤ 0.03. Failures listed by check with their layout/stance. If < 85 %: narrow the ranges (first drop the mirror, U3).
- Fixed scene still 10/10. Tests green.
</done>
</task>

<task type="auto">
<name>Stage 4 - Scripted walk-up with synchronous GR00T, camera look, fine positioning and handover</name>
<files>g1sim/walkup.py (new), g1sim/navigation.py (FINE params), g1sim/table_scene.py (GROOT_STANCE update), configs/randomization.json (arrival),
scripts/67_volleyball_pick_place.py (--walkup, after Stage 3 merged), tests/test_integration_volleyball_walk.py</files>
<action>
- Probes first, each logged to `runs/`:
  1. stand 4 s at dt 1/300 (6 substeps) in `volleyball_walk` with READY arms → measured `GROOT_STANCE` (12 legs, waist roll/pitch, pelvis z/roll/pitch) into table_scene;
  2. walk 0.8 m at dt 1/300; if it fails, use the 0.004 fallback from B;
  3. fine positioning from 10 starts; report final error and overshoot.
- Build `walkup.WalkUp(env)` with `release()`, `look()`, `walk_to(goal)`, `fine_position(goal)`, `run(variation)`, `handover()` as in B.
  Import `GrootLocomotionController` lazily, so TableEnv stays importable without lerobot. Arms READY, hands open.
- `on_tick(phase)` with band, stand, look, walk_up, fine, handover (labels in Stage 6).
- `67 --walkup`: `reset(randomize=True)` puts the robot at `walk_start`, runs `WalkUp.run`, then the teacher; walk checks from C first.
- **Arrival distribution:** 100 walk-ups (seeds 1000-1099, no teacher) → arrival offsets in the goal frame, handover roll/pitch → `configs/randomization.json`
  `volleyball.arrival` with `source: measured`.
- Integration test: one seeded walk-up arrives and hands over (needs g1-ext).
</action>
<verify>
- `$PY scripts/67_volleyball_pick_place.py --scene volleyball_walk --randomize --walkup --arrivals-only --trials 100 --seed 1000 --no-labview --out runs/l1w_s4_arrivals.json > runs/l1w_s4_arrivals.log 2>&1`
- `$PY scripts/67_volleyball_pick_place.py --scene volleyball_walk --randomize --walkup --trials 50 --seed 0 --no-labview --out runs/l1w_s4_walk.json > runs/l1w_s4_walk.log 2>&1`
- `$PY scripts/67_volleyball_pick_place.py --scene volleyball_walk --randomize --walkup --trials 1 --video > runs/l1w_s4_video.log 2>&1` (watch the video).
</verify>
<done>
- 0 falls; both objects found ≥ 49/50; detection error p95 ball ≤ 0.02 m, basket ≤ 0.04 m.
- Fine positioning: arrival error p95 ≤ 0.04 m and ≤ 4 deg. Stance inside the envelope ≥ 47/50; no walk-up contacts.
- Full sequence (walk-up + pick-and-place) ≥ 40/50.
- Handover pelvis z within 0.01 m of `STAND_Z` (the measured one); handover roll/pitch logged.
- Measured arrival distribution stored; wall time per trial reported (expected < 5 s without images).
- If GR00T does not walk at either timestep inside TableEnv, or fine positioning cannot reach p95 ≤ 0.06 m: stop, report, record in no-walk mode.
</done>
</task>

<task type="auto">
<name>Stage 5 - SmolVLA policy server and eval script, proven by replay (parallel to 3 and 4)</name>
<files>scripts/81_smolvla_server.py (new), scripts/82_vla_eval.py (new), scripts/83_open_loop_check.py (new)</files>
<action>
- Server (SmolVLA env): websocket with the protocol of `openpi_client` (`60_openpi_closed_loop.py`); install `openpi-client` (or `websockets` + `msgpack`), ruling U4.
  Input `head_image` (200, 320, 3) uint8 (else reject), 27-D state, prompt; output (N, 27).
  Modes `--checkpoint <dir>` (processors + `predict_action_chunk`, Section E) and `--replay <npz>`.
- Eval (sim env), Section E: walk-up or `--from-handover`; event checks and latching; both denominators with Wilson intervals; paired teacher mode
  (`--teacher` from the same snapshots); `runs/vla_eval_<name>.json` + Lab View `vla_eval`.
- `83_open_loop_check.py`: Section E, prints MAE vs spread per group.
</action>
<verify>
- `$SV scripts/81_smolvla_server.py --replay <demo.npz> &` + `$PY scripts/82_vla_eval.py --from-handover <demo.npz> --trials 1 > runs/l1w_s5_replay.log 2>&1`
- Before Stage 6 exists: a 67 no-walk trial recorded with the factory pattern. After: 5 demos from the Stage 6 smoke set (walk mode).
- `$SV scripts/81_smolvla_server.py --checkpoint lerobot/smolvla_base` + one observation → chunk shape and latency.
</verify>
<done>
- Replay passes 5/5 from the saved handover snapshots (layout, 30 Hz timing, handover state, waist channel correct).
- The server answers a 200x320 + 27-D observation with an (N, 27) chunk in < 1 s on mps and rejects a 400x640 image.
- Event checks reproduce the teacher's verdict on those 5 demos (all latched success).
</done>
</task>

<task type="auto">
<name>Stage 6 - Volleyball demo factory and Lab View labels</name>
<files>scripts/69_volleyball_demo_factory.py (new; plan text said 68), g1sim/labview.py</files>
<action>
- 68 modelled on 64 `record()`: pending-observation pattern, `EpisodeRecorder(task="put the volleyball in the basket", fps=30)`, recording from the
  handover; trim rule from D; saves only successes, the handover snapshot, the sidecar (D) and `factory_s<seed>.json`.
  Flags `--walkup / --no-walkup`, `--seed` (≥ 100 enforced), `--demos`, `--out`, `--name`. Skips `._*` files. Refuses `--no-walkup` with a provisional
  arrival distribution unless `--allow-provisional`.
- labview: `WALK_PICK_PLACE_PHASE_LABELS` (band, stand, look, walk_up, fine, handover, the pick-place phases, align), `PHASE_ORDERS["walk_pick_place"]`,
  `vla_eval` phases; reasons fell, not_found, detection_error, stance_outside_envelope, walkup_timeout, walkup_contact; reporter in 67 `--walkup` and 68.
</action>
<verify>
- `$PY scripts/69_volleyball_demo_factory.py --walkup --demos 5 --seed 100 --name smoke --out "/Volumes/T7/<U2>/g1-demos" > runs/l1w_s6.log 2>&1`
- Same with 3 processes (seeds 101-103, `--demos 3`) for throughput.
</verify>
<done>
- 5/5 saved, `validate == []`, frames 600-800, 27-D; `state[:, 26]` follows `actions[:, 26]` within 0.05 rad after 10 ticks; timestep 1/300 in every sidecar.
- Sidecars complete; Lab View shows the walk-up phases; throughput per process and with 3 in parallel in the log.
</done>
</task>

<task type="auto">
<name>Stage 7 - Record 300 demos, convert to LeRobot v3.0, validate</name>
<files>scripts/79_convert_smolvla.py (new), scripts/80_validate_dataset.py (new)</files>
<action>
- Install `lerobot[dataset]==0.6.1` extras into the SmolVLA env (U4).
- 69: npz → v3.0 with `LeRobotDataset.create(repo_id="local/g1_volleyball_walkup_v1", root=<SSD>/lerobot/..., fps=30, robot_type="unitree_g1_brainco",
  features=..., use_videos=False)`, `add_frame`, `save_episode`, `finalize`; key `observation.images.camera1`; skips `._*`, unsuccessful and invalid episodes.
- 80 (SmolVLA env): npz level: all `episode.validate` empty, success True, sidecar present, seeds ≥ 100, no duplicate variation, state within joint
  limits, hands 0..1, |waist| < 2.6, timestep 1/300, `stance_source` reported. Dataset level: loads; episode and frame counts equal the npz; 20 random
  frames match the npz exactly (state, action and image, lossless); stats for state/action have 27 entries; task string present.
- Record with 4 processes, seeds 100-103, 75 demos each, `--walkup` (fallback `--no-walkup`).
</action>
<verify>
- `$PY scripts/69_volleyball_demo_factory.py --no-walkup --demos 75 --seed 10X --name level1_walkup_v1 --out ... > runs/l1w_s7_rec_10X.log 2>&1` ×4 in the background.
- `$SV scripts/79_convert_smolvla.py <dir> > runs/l1w_s7_convert.log 2>&1`, `$SV scripts/80_validate_dataset.py <dir> <root> > runs/l1w_s7_validate.log 2>&1`
</verify>
<done>
- ≥ 300 validated demos on the SSD; 80 reports 0 problems; v3.0 has 300 episodes and the same frame count as the npz.
- Recording success rate, failure reasons and wall time written down.
</done>
</task>

<task type="auto">
<name>Stage 8 - SmolVLA training (smoke, night 1, open-loop check, night 2)</name>
<files>scripts/train_smolvla.sh (new)</files>
<action>
- Script with the settings from E; `--steps` and `--resume` passed through.
- 200-step smoke: assert in the saved checkpoint that `config.json` input/output features are `observation.images.camera1` and 27-D
  `observation.state`/`action` (with `--policy.path`, `make_policy` keeps the base model's `[6]` features when they are set, `factory.py:305-306`;
  if so, clear them in the script so they are filled from the dataset, checked against the installed source), and that the normalizer stats in the
  processor files have 27 entries. Loss at 200 < loss at 10. Wall s/step with real image loading recorded.
- Night 1: 20k steps with nohup → `runs/smolvla_level1_walkup_v1/train.log`. Night 2: resume to about 45k.
</action>
<verify>
- `bash scripts/train_smolvla.sh --steps 200 > runs/l1w_s8_smoke.log 2>&1`
- After night 1: `$SV scripts/83_open_loop_check.py --checkpoints <5k,10k,15k,20k> > runs/l1w_s8_openloop.log 2>&1`
</verify>
<done>
- Smoke asserts pass; checkpoint written.
- Night 1 reaches 20k (or reports s/step and ETA). Open-loop MAE falls from 5k to 20k; if it does not, stop and debug before night 2.
</done>
</task>

<task type="auto">
<name>Stage 9 - VLA eval after the scripted walk-up</name>
<files>(no new code; runs 81 + 82)</files>
<action>
- Per checkpoint: 10 training variations from their handover snapshots + 50 trials seed 0 (walk-up → snapshot → VLA), `--exec-steps 25`.
- Teacher from the same seed-0 snapshots as the paired upper bound.
- Apply the decision rule from E; failure analysis by first failed check, furthest event, stalls, and layout/stance.
- If time allows: `--exec-steps 10/50`.
</action>
<verify>`$SV scripts/81_smolvla_server.py --checkpoint <ckpt> &` + `$PY scripts/82_vla_eval.py --trials 50 --seed 0 --train-variations 10 --name smolvla_v1 > runs/l1w_s9.log 2>&1`</verify>
<done>
- Report JSON + Lab View record exist with both denominators, Wilson intervals, failure table, latency, and the undertrained/generalization verdict.
- Level-1 target ≥ 80 % (PLAN (a)) is reported, not assumed; it is not expected from night 1.
</done>
</task>

Docs, after each stage (rule 6/8): a PROGRESS.md LOG entry plus `docs/LEARNING_PATH.md` §10. Files plus commit message go to the user, who commits and pushes himself.

## G. Risks and second-order checks
- **R1, synchronous GR00T inside TableEnv** (different model build, gain randomization, dt 1/300 walking) does not walk.
  Mitigation: Stage 4 probes first; 0.004 fallback; leg gain randomization off if needed; no-walk recording with the measured/provisional distribution.
- **R2, teacher < 85 % at random layouts.** Mitigation: the reach map sets the ranges before the sweep; narrow ranges; drop the mirror; keep only successes.
- **R3, the weld hides balance problems.** GR00T measured 2-5 cm / 5-13 deg drift while the waist moved. Real robot: Unitree controller balances.
  Stretch test later: grasp on GR00T Balance with the waist through `groot_orientation_cmd`.
- **R4, waist yaw via `rt/arm_sdk` on the real G1 is unknown** (U5).
- **R5, the 27-D change ripples.** Bottle, openpi, teleop and tests updated in Stage 1, gated by `run_tests.sh` in all three envs. The 2 old bottle probe demos (26-D) become invalid.
- **R6, exFAT.** `._*.npz` files are skipped; a torn npz is caught by `validate`/load.
- **R7, too little training on a Mac.** Night 1 ≈ 0.76 passes. The E decision rule separates undertraining from a data gap, so more demos are added only for a generalization gap.
- **R8, eval/recording mismatch.** Same world, handover snapshot, image size and observation function; the replay gate restores the snapshot.
- **R9, the head tilt changes at the handover (0.5 → 0.85)** and the arms are at READY before walking. The real adapter must do both. Added to the real-adapter list.
- **R10, the D455 minimum depth is about 0.4-0.52 m.** The look happens from ≥ 0.66 m. The VLA uses RGB only.
- **R11, test fixtures rewrite scene files** under `~/dev/g1-ext/g1_brainco_sim`. Never run `run_tests.sh` while the factory is recording.
- **R12, project rules.** `CLAUDE.md` says "never suggest a local full training run" and "always fine-tune with LoRA". PLAN (h)/(i), the user's newer
  decisions from 2026-10-07, allow fine-tuning SmolVLA overnight on this Mac (a fine-tune of `smolvla_base`, expert only). This plan follows (i).
- **R13, the real table and the stop distance.** The sim stop keeps the pelvis ≥ 0.21 m from the edge because the G1's hips are at table height.
  A real table with an apron or a cabinet front may need more; the real stop distance comes from the real table (U5).
- **R14, READY hands pass about 2.3 cm above the ball top while arriving** (READY hand geoms z ≥ 1.04 at pelvis 0.75, ball top 1.017, `fk.out`).
  `stance_clearance` includes ball and basket, and `walkup_contact` catches a touch. If Stage 4 sees ball contacts: a slightly higher walk-arm pose
  that returns to READY during the 1 s Balance wait, with its path proven clear by an FK test.

## Open rulings for the user (each with a recommendation)
- **U1:** Weld the pelvis after the handover (recommended for v1: deterministic, reuses the validated teacher, same world for recording and eval).
  Alternative: GR00T balancing during the grasp, 1-2 days more, high risk.
- **U2:** Demo location. T7 is mounted (891 GiB free). Recommended `/Volumes/T7/Robot Project SAP/g1-demos/`. Is T7 the "BrainBot" SSD?
- **U3:** Ball left or right at random in v1. Recommended yes if the Stage 3 sweep stays ≥ 85 %.
- **U4:** OK to install `lerobot[dataset]` extras and `openpi-client` into `~/dev/g1-ext/smolvla-test/.venv`? Recommended yes (reversible, separate env).
- **U5 (team / robot owner):** Is waist yaw allowed via `rt/arm_sdk`? Who drives the head tilt? Real basket colour and size? Does the real table have
  legs, an apron or a closed front (sets the real stop distance, R13)?
- **U6 (new):** Two training nights instead of one (night 1 = pipeline proof, night 2 = first real number), which moves the Level-1 sim result to
  Sun 10-12. Recommended yes; the alternative is renting a GPU for a few hours (pi0.5/LoRA path, PLAN (i)).

## I. User rulings after revision 2 (2026-10-08)
- **The real ball is grey.** Consequences:
  - Sim ball base colour grey (about rgb 0.55, 0.55, 0.55) in `volleyball` and `volleyball_walk` (Stage 1 (d)); the per-episode colour jitter stays
    on the target object, so the VLA sees grey shades, not one exact grey. Fixed-scene 67 must still pass 10/10 (colour does not change physics).
  - Detection is by shape (B, Stage 2). `66_volleyball_scene_check.py` check 5 ("> 2000 red pixels") is replaced by a colour-free visibility check
    (projected ball silhouette or segmentation render, same pixel threshold) in Stage 1.
  - Wider ball-colour variation (other colours, patterns) stays Level 2.
- **Distractors (owned by the teammate, own files, Stage D):** 0-3 objects (e.g. a red cylinder, a grey cylinder, a box) placed on the table in a
  keep-out-aware way (outside the hands' sweep and the waist-turn path, ≥ 0.03 m from stand and basket), in `g1sim/distractors.py` + a
  `distractors` group in `configs/randomization.json`. Finnegan's side adds one hook in the scene build / reset. Distractors count as forbidden
  contacts for the teacher. Gate: the Stage 3 sweep with distractors on stays ≥ 85 %, and Stage 2 still picks the ball, not a distractor.
  Starts after Stage 3 (it needs the layout sampler); the hook signature is agreed first so both sides can build in parallel.
- **U2 answered:** demos go to `/Volumes/T7/Robot Project SAP/DATA Collection/g1-demos/level1_walkup_v1/` (folder created by the user, exFAT).
- U1, U3, U4, U6: no answer yet; the recommendations are used until the user says otherwise.

**Carried in from the Stage 1/2 code reviews (2026-10-08, `.planning/reviews/level1-walkup-stage{1,2}-code*.md`):**
- **Stop distance (overrides A, B and Stage 3/4 numbers).** With the measured `GROOT_STANCE` the right knee touches the table when the pelvis is
  closer than about 0.205 m to the front edge; at the R_STOP 0.45 nominal stop (0.236 m) `stance_clearance` is only 0.031 m. The `fk3.out` numbers
  above (+0.019 at 0.20) used a symmetric crouch and are wrong. Stage 3 starts by fixing this before building the sampler: pick R_STOP (likely
  0.46-0.47) so the nominal clearance is ≥ 0.04 m, re-check reach there (teachback / reach map), and derive the sampler threshold and the walk-up
  envelope from the measured arrival error (envelope must allow the Stage 4 p95 of 4 cm). Point the Stage 1 test
  (`tests/test_integration_volleyball_walk.py:96`) at the sampler's threshold.
- **Weld carries ~71 % of the weight** in the provisional stance: Stage 4 re-measures `GROOT_STANCE`/`STAND_Z` and adds a floor-load check
  (≥ 80 % of the weight on the feet).
- **Basket detection with distractors (blocks Stage D, matters for Stage 4):** `find_basket` can return a wrong centre (6-25 cm off) instead of
  None when a tall object stands in front of the basket or the basket is cut off by the image edge with an object on the other side.
  Fix inside Stage D (or Stage 4 if it shows up there): reject estimates whose rim does not form a consistent square, and add those cases as tests.
- **Grey-ball colour jitter is not wired yet:** `randomization.sample` still loops over the bottle objects; Stage 3 (sampler) wires it.
- **Snapshot → npz:** the handover snapshot holds a PCG64 state with 128-bit ints and nested dicts; Stage 6 stores it as JSON.

**Natural arm motion (user request 2026-10-08, built before Stage 3; overrides "READY from the very start" in A and "Start = READY" in C):**
- Volleyball scenes start and end in `action_space.BALL_READY_POSE` (`SCENES[...]["ready"]`, `TableEnv.ready_pose`): elbows low and close to
  the body, forearms forward, palms facing each other, fingers relaxed (0.10-0.15). The bottle scene keeps `READY_POSE`.
- Teacher phases (revised the same day after user feedback on the images): face_ball → reach (one straight, smooth move to
  beside the ball, hands already turned to it and opening) → touch → squeeze → lift → turn → release → retract (start pose)
  → face_front → settle. All moves use a cosine speed profile. Start pose revised so the elbows are beside the body, not behind
  it (hands at chest height: lower hands touch the ball while arriving or turning). Result: 67 10/10, ±2 cm 50/50, tests green.
- `stance_clearance` now uses the scene's start pose. The walk-up arm pose (Stage 4) is `BALL_READY_POSE`; Stage 4 checks it against the ball
  while arriving (R14). Recording (Stage 6) ends after `face_front` + 30 ticks.

## J. Merge with Saad's branch `saad/table-distractors` (2026-10-09, branch `integrate/saad-distractors`)
Saad's branch (distractors, full randomization in the fixed volleyball scene, walk to the table with checkpoint, `scripts/68_walk_to_table.py`)
was based on the main before Stages 1+2 and the natural arm motion. Decisions while merging:
- **Teacher (`ball_grasp.py`):** our motion (start pose `BALL_READY_POSE`, straight reach, cosine profiles, waist through `action[26]`) +
  Saad's geometry (waist angles from the bearing change of stand and basket around the waist axis, hand orientation in world from
  pelvis yaw + waist, distractors as forbidden contacts). This covers the "teacher from any stance" part of Stage 3 for the fixed scene.
- **Found by the gates, fixed:** after the merge, random starts (seed 0) passed 46/50: the arm sags under gravity during the straight
  reach, and with the ball pushed towards the basket the right wrist landed on the basket rim. `REACH_ABOVE` 0.02 → 0.04 (the sag is
  measured and removed at the end of the reach). An arc in the reach and the basket as an IK obstacle were tried and did not help / were
  not needed; both reverted.
- **Found by reading, fixed:** `68_walk_to_table.py` held 26-D arm vectors (`ARMS_DOWN`) next to the 27-D `READY` pose → shape error;
  now 27-D. The walk-up raises the arms to `BALL_READY_POSE` (was the old `READY_POSE`), so the handover pose = the teacher's start pose.
- **Scene:** `volleyball` has distractors + `reach_m` + `ready`; `volleyball_walk` has no distractors and no randomization yet (Saad's
  keep-out is measured from the fixed robot's waist axis; his "stand" group moves the pelvis body, which a free joint ignores) → raises.
- **Docs:** both learning-path series kept; Saad's sections renumbered 10.31-10.40 (his README updated).
- **Quick review (2026-10-09, `.planning/reviews/merge-saad-distractors.md`): GO.** Minor 1 fixed (distractor test held READY_POSE in a scene
  that starts in BALL_READY_POSE). Minor 2 → Stage 3: in `volleyball_walk` (stance x = -0.036) the bearing change is no longer 0, so grasp and
  place angles differ slightly from ±35°; geometrically right, but not yet run there (67 has no `--scene volleyball_walk` yet).
- **Not merged in:** Saad's walking world (`approach_world.py`, movable table, DDS walk sim) and our `volleyball_walk` (TableEnv, weld,
  snapshot) are still two worlds. Joining them is the next step (Saad: walk-in controller into `volleyball_walk`; Finnegan: Stage 3
  remainder - stop radius, basket variation, 200-trial sweep from walk-like stances).

## K. Stage 3 in the walk world (2026-10-10, Step 1 of the 2026-10-09 list)
- `R_STOP` 0.45 → 0.458 (nominal stop clearance 0.041 m ≥ 0.04); `WALK_STANCE` from `navigation.table_stop_pose`.
- Walk sampler `randomization.sample_walk` + config group `"walk"`; `TableEnv.reset(randomize=True)` works for the free base (redraw on
  stop clearance < 0.04 or stance clearance < 0.02). Arrival ranges from the reach map: back 0-0.06, sideways ±0.04, ±6°. Separation
  0.40-0.56; new rule: ball and basket ≥ 50° apart from the stance (all 4 sweep-1 failures below it).
- Section C `align` built as teacher phase **`centre`** (`align` is a walk-to-table phase key). Mirror (U3): not in v1.
- Gates: map 52/65 (failures explained: knee / reach limit / an impossible start), sweep **199/200**, centred p95 0.003 m, fixed scene
  unchanged, tests green. Demos v1 in no-walk mode (F: "data never waits for the walk-up"), stance from the map ranges rather than a
  measured Stage 4 distribution; sidecars must say so (`stance_source`).

## L. Step 5 recorder + shoulder lift (2026-10-14)
- **Recorder `scripts/69_volleyball_demo_factory.py`** (Stage 6, no-walk mode only): episodes from the handover to face_front + 30 ticks
  (726 frames, 24.2 s), head image 200x320 + 27-D state + 27-D action, handover snapshot in the npz (`episode.handover_arrays`, JSON for the
  PCG64 state), sidecar per demo (`stance_source: provisional` + `stance_note`, git, variation, walk geometry, handover pelvis pose, teacher
  result), `factory_s<seed>.json` after every trial, lock file per seed and folder, verified writes (`episode.write_verified`). Demos that
  fail validate / 600-800 frames / waist tracking 0.05 rad are not saved. Lab View card per process (head frame | outside view, one renderer,
  a throw-away head render after each outside render keeps the recorded frames bit-exact).
- **Not built (deferred):** `--walkup` mode and `--allow-provisional` (walk mode needs Step 2), walk-up Lab View labels.
- **Review gate** (`.planning/reviews/level1-step5-recorder-{contract,learning}.md`): contract GO, learning OPEN → GO after 5/5 on the T7 and
  removing the reviewer's `guardcheck` folder (done). Fixed: reject instead of flag, verified sidecar/report writes, report per trial, lock,
  `stance_source` token, dataset seeds 101-104 (100 = smoke). The 0-byte episode on 2026-10-14 was most likely a second recorder writing the
  same file (reviewer run, same seed and folder), not the exFAT driver.
- **M1 from the learning review (open, before Stage 5/9):** the eval has no held-out **no-walk** path. `82_vla_eval.py` gets `--no-walkup`:
  seeds 0/1, `reset(randomize=True)` → snapshot → first observation, exactly like 69. The decision rule in E compares training variations vs
  held-out no-walk first; walk-mode eval is reported separately as a distribution-shift number.
- **Converter and validator (Stage 7):** skip `._*` AppleDouble files (exFAT) and `*.part`; `80` checks duplicate variations across folders.
- **Shoulder lift (user, 2026-10-14: "straighter pick-up, arms not so cramped"):** posture probe showed the 10-07 and today's teacher give
  the same arm posture in the same scene; the cramped look is the walk-world stance (4.3 cm lower, 3.8 cm further back) plus a lift that
  curls the elbows. `ball_grasp`: lift targets turned about the shoulder line (rigid pair, ball straight up), IK posture target = squeeze
  arm during the lift. Walk world after the lift: forearm yaw -11 → -1.5 deg, forearm pitch 39 → 31 deg, upper arm 34 → 40 deg down. A stiff
  IK elbow (cost 1.0) was tried and rejected (elbows flared 11 cm out). Gates: walk sweep 200/200 (seeds 10-13), fixed nominal 10/10,
  ±2 cm 50/50, randomized 50/50 + 50/50, distractors 30/30. User approved from the side-by-side video (`docs/videos/l1w_s5_lift_before_after.mp4`).

- **Step 6 (2026-10-14): 300 demos recorded** (301 trials, 1 rejected: forbidden contact; 726 frames each, 35.1 GB on the T7). Converter
  `scripts/79_convert_smolvla.py` + validator `scripts/80_validate_dataset.py` (SmolVLA env, `lerobot[dataset]==0.6.1` installed = U4).
  **Dataset location changed:** the LeRobot v3.0 dataset lives on the internal disk `~/dev/g1-ext/datasets/g1_volleyball_walkup_v1`, not on
  the exFAT T7: macOS writes a `._` twin of every file there, lerobot globs `*/*.parquet` and crashes on `._file-000.parquet`, and its temp-image
  cleanup crashes when macOS deletes the twins itself (reproduced 2026-10-14). The raw npz demos stay on the T7 (source of truth); a single-file
  archive of the dataset goes to the T7 as a backup. Loading also builds an Arrow cache in `~/.cache/huggingface/datasets` (~ dataset size).
  Reviews: shoulder lift GO (`level1-step5-shoulder-lift.md`), converter NO-GO → fixed (`level1-step6-converter.md`: F1 location, F3 the
  validator now compares every state/action value of every episode + stats + one image per episode + 100 random images, F4/F5).
  Still open from F4: arm joint-limit check (needs the model, not in the SmolVLA env).

## H. Definition of done (whole scenario)
1. `67 --scene volleyball_walk --randomize --walkup --trials 50 --seed 0` gives ≥ 40/50 full-sequence success, 0 falls, with Lab View and a video of trial 0.
2. ≥ 300 successful demos (episodes from the handover, 27-D, head image, prompt, handover snapshot) on the SSD, converted to v3.0 (lossless images),
   and `80_validate_dataset.py` reports 0 problems.
3. The replay check passes 5/5 from saved snapshots, which proves the eval pipeline.
4. SmolVLA trained (night 1 20k, night 2 ~45k); open-loop error falls with steps; checkpoints saved.
5. `82_vla_eval.py --trials 50 --seed 0 --train-variations 10` has run after the scripted walk-up. Both success rates with Wilson intervals,
   failure table and the undertrained/generalization verdict are in `runs/` and the Lab View.
6. All tests green in the three envs; fixed-scene 67 still 10/10; PROGRESS and LEARNING_PATH updated.

## Review response (revision 2)
Sim feasibility review (`.planning/reviews/level1-walkup-sim-feasibility.md`):
- **B1 knees hit the solid table — fixed.** Re-derived: `fk3.py` re-run gives knee-table +0.019 / -0.011 / -0.051 m at pelvis-edge 0.20 / 0.17 / 0.13.
  Option 1 (open-front table) refuted by own probe `slab.py`: a 4 cm slab hits `right_hip_yaw_link` at 0.17 m (-0.012) because the hips are at table
  height; a 10 cm slab is the same. Option 2 taken: stop radius R_STOP 0.45 (pelvis 0.236 m from the edge for the nominal layout), FK-based
  `stance_clearance` for sampling (≥ 0.04) and the envelope (≥ 0.02). Reach at that distance checked with `teachback.py` (5/5 pass at 0.02-0.05 m back).
  The 70-100 deg angle rule became a separation rule, because it is infeasible at the new distance. Stage 3 starts with a reach map.
- **B2 arms hit the table in the start pose and in the raise — fixed.** Re-derived with `fk.py`/`fk2.py` (start pose +0.024 / -0.006 / -0.046 m;
  every raise path -0.09 to -0.15 m). Hanging pose and `raise` phase deleted; READY from spawn (P3 walked with READY). Hand-to-ball clearance added (R14).
- **M3 STAND_Z contradiction — fixed.** `fk2.out` 0.784 vs measured 0.7505 confirmed. Constants from the measured crouch, re-measured in Stage 4; feet test in Stage 1.
- **M4 arrival precision / useless re-approach — fixed.** Confirmed `navigation.py:64-65, 90-91`. Goal perturbation removed, `FINE` positioning phase added
  (probed first in Stage 4), re-approach removed, no-walk stances drawn from the measured arrival distribution; envelope criterion 47/50.
- **m5 bearing bounds undefined — fixed.** Waist |·| ≤ 45 deg in config, replaced by the Stage 3 map values.
- **m6 timestep switch — fixed.** One dt 1/300 with 6 substeps (standing confirmed in `stand_0.0033333333333_*.log`); walking probed first; fallback sets dt unconditionally and is asserted.
- **m7 replay through a re-run walk-up — fixed** by the handover snapshot (+ head `eq_data`, controller state) and `--from-handover`; head tilt reset in `_set_up`.
- **m8 reset ordering and the stand group — fixed** in Stage 1 (e) and A (groups off).
- **m9 effort/timeline — fixed.** Stage 1 6-7 h, Stage 4 5-7 h; timeline rebuilt (recording Fri, two nights, result Sun); U6 added.
- "Not verifiable" items: grasp-reach claims now marked unverified in Assumptions and decided by the Stage 3 map; walking at 1/300 and real-render
  perception accuracy are Stage 4 gates.

Data/learning review (`.planning/reviews/level1-walkup-data-learning.md`):
- **M1 training budget — fixed.** Budget stated (0.76 passes), night 1 = pipeline proof, night 2 resume, training-variation eval and the
  undertrained-vs-generalization rule, open-loop check per checkpoint. Kept 300 demos (recording is cheap, about 1-1.5 h); the rule prevents the wrong
  "more demos" fix.
- **M2 non-reproducible handover — fixed.** Confirmed `table_env.py:50, 197-203`. Snapshot via `mj_getState(mjSTATE_INTEGRATION)` + controller state,
  `--from-handover`, per-reset noise seed, paired teacher comparison from snapshots.
- **M3 checkpoint path unchecked — fixed.** Confirmed `make_pre_post_processors` (`factory.py:150`) and `predict_action_chunk` (`modeling_smolvla.py:232`)
  in the installed 0.6.1. Server uses them; `83_open_loop_check.py` per checkpoint.
- **M4 eval denominators and latching — fixed** in E (two denominators, event definitions, latching, furthest event, stalled).
- **m1 `test_unit_openpi_policy.py` — fixed.** Confirmed line 61; added to Stage 1, grep extended to `tests` and `openpi_g1`.
- **m2 video encoding — fixed.** `use_videos=False` (confirmed parameter at `lerobot_dataset.py:691`); exact image equality in 80.
- **m3 eval observation path — fixed.** 82 uses TableEnv `IMAGE_SIZES` + `episode.policy_observation`; server rejects other shapes.
- **m4 feature mapping — fixed.** Key `observation.images.camera1`; smoke asserts 27-D features and 27-entry stats (`factory.py:305-306` confirmed).
- **m5 no-walk vs walk handover pose — fixed.** Roll/pitch measured and logged in Stage 4; no-walk samples them.
- **m6 randomization groups — fixed** (same as sim m8).
- **m7 openpi config — fixed as a GPU-day note in D.** `reset_pose` is now correct because episodes start at READY.
- **m8 eval resolution — fixed.** Wilson intervals in E/H.
