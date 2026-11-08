# Review: Level 1 Step 4 plan (`.planning/level1-walkup/STEP4-PLAN.md`, revision 1), plan review before the build

Reviewer: an independent gate. I did not write the plan and saw none of the author's reasoning beyond the plan text and its probe files.
Date: 2026-10-15. Stage: plan review (pre-build). Spec read: PLAN.md SCOPE UPDATE, PROGRESS.md NEXT STEP (row 4 "replay 5/5"),
`.planning/level1-walkup/PLAN.md` D, E, F (Stage 5, Stage 9), I, K, L (M1), H, and `.planning/reviews/level1-step5-recorder-learning.md` (M1, m5, m6).
Hard rules kept: run1 (PID 32059, lerobot-train on MPS) was left alone. No model was loaded (MPS or CPU), no training was started, and
`smolvla-runs/run1` was not touched. Nothing was written to the T7. The only runs were two read-only numpy/MuJoCo probes of about 10 s each
(scratchpad `step4/review_*.py|log`).

## Verdict 1: is the plan internally right? Mostly yes. One gate number is wrong (F1).
I checked these against the installed sources and the repo code:
- **lerobot 0.6.1** (dist-info in the SmolVLA env): `predict_action_chunk` is `@torch.no_grad`, calls `eval()` and takes `noise`
  (`modeling_smolvla.py:231-241`). It unpads to `action_feature` (`:216-218`). `select_action` keeps its own queue (`:243-269`). `sample_actions`
  builds noise as `(B, chunk_size, max_action_dim)` (`:726-742`). `prepare_state` pads (`:409-413`). Missing cameras are skipped when
  `empty_cameras == 0` (`:340, :373-375`). `reset()` leaves only the ACTION queue (`:170-174`), and `populate_queues` skips keys that are not in
  the queues (`utils.py:30-47`), so the server holds no stale observation between calls. `prepare_observation_for_inference` converts in
  place via `torch.from_numpy` (`utils.py:97-138`), so the plan's "pass copies" is right. `make_pre_post_processors(pretrained_path=...)` loads
  both pipelines (`factory.py:150, 200-219`). Overrides are matched by registry name and checked to be used (`processor/pipeline.py:741-747`),
  so `{"device_processor": ...}` is valid; the saved preprocessor has `device_processor` with `"device": "mps"` and needs that override on CPU.
  `from_pretrained` loads local `model.safetensors`, then calls `.to(device)` and `.eval()` (`pretrained.py:169-227`).
- **Smoke checkpoint config:** `n_obs_steps 1`, `chunk_size 50`, `n_action_steps 50`, `resize_imgs_with_padding [512,512]`,
  `empty_cameras 0`, `adapt_to_pi_aloha false`, image transforms disabled, VISUAL norm IDENTITY. The quirk in the input features
  (state `[6]`, 3 cameras at 256x256) is there as the plan describes. Asserting against the stats file instead of `input_features` is the right call.
- **Transport evidence:** `async_inference/policy_server.py` imports `grpc` and uses `pickle.loads`/`dumps` (`:28, :38, :125, :183, :236`). The
  SmolVLA env has no websockets, msgpack or pytest (import checked). `openpi-client` pins `numpy<2.0.0` (`pyproject.toml:8`). The SmolVLA env
  imports `g1sim.episode` and `g1sim.action_space` with `-I`, with no mujoco/torch/mink loaded (checked). The project `.venv` has pytest + numpy 2.5.3.
- **Handover contract against 69:** the sequence `69:203-214, 127, 130` is `write` → `TableEnv(seed)` → `reset(randomize=True)` (which
  itself renders once, `table_env.py:134`) → `snapshot_handover` → `observation`. `env.rng` is consumed only in `reset`
  (`table_env.py:124`, grep over g1sim + 67 + 69), and `noise_rng` is re-seeded per reset index (`:116`). So trial k of seed S really does
  not depend on what ran between resets. `ModelVariation.apply` starts from the saved originals (`randomization.py:229-232`), so a
  from-handover restore before the no-walk trials (the `--train-variations` run) does not shift later resets. Image 200x320 uint8,
  state (27,) float32, and frame t = the observation before action t: all match `episode.py` and `69:130-143`.
- **Event checks against `ball_grasp.py`:** `MIN_LIFT` (`:94`), `FORBIDDEN` (`:100`), `placement_checks` (`:136-150`), check order
  (`:259-264`), and the outer basket footprint 0.14 + 0.005 = 0.145 (`table_scene.py:61-63`). Contacts are sampled once per control tick, as
  in the teacher (`:362-365`). (Small nit: the teacher's `rest` is at `:194`, not `:192`.)
- **Labview:** `Reporter(..., frame_getter=...)`, `trial_end(passed, reason, {"reachable": ...})`, `PHASE_ORDERS`, `phase_label`
  (`labview.py:72-106, 153-232`), `write_atomic`, `git_state`, the uniqueness test (`tests/test_unit_labview.py:86-94`) and
  `conftest.probe_result` (`:37`) all exist as the plan uses them.
- **Wilson test values:** I recomputed all four and they are correct.
- **Bit-exact replay:** confirmed on the exact 10 demos the gate selects (below). `replay_state_error == 0.0` held for all 10. Requests
  equal `ceil(latch/25)` for all 10, and none of them hits a 25-multiple edge, so the 0-based vs 1-based tick ambiguity does not bite here.
  The convention should still be stated (1-based = number of applied actions).

## Verdict 2: is it the right plan for the purpose? Yes in structure. One purpose gap (F2) and one gate that needs calibration (m1).
The stdlib TCP transport, the server in the SmolVLA env, the synchronous loop with latency reported, the held-out no-walk mode that copies
69 (this closes learning-review M1), teacher mode as a paired upper bound, the denominators and the Lab View task all fit Step 4 and the
Stage 9 decision rule. The gaps are about what the gates actually prove.

## Findings

### F1 (major, gate criterion wrong): "latch 24-26 s" fails 3 of the 10 demos the Step 4 gate itself selects
- Trigger: plan §3 pass criterion "latch 24-26 s" (STEP4-PLAN.md:401-402), from a probe of 3 demos (decision (4), :158-160: "latch 727-749").
- Evidence: `scratchpad/step4/review_gate_preview.log`. I restored each demo's snapshot in a fresh TableEnv, applied the recorded float32
  actions in 25-tick chunks, then held the last action. Picks: the 5 smoke demos, plus `np.linspace(0, 299, 5)` over the sorted training folder
  (s101_t0000, s102_t0000, s103_t0000, s103_t0074, s104_t0075). The latch came at ticks 703-754 = **23.43-25.13 s**:
  s100_t0003 23.70 s, s103_t0074 23.43 s, s104_t0075 23.93 s. All had `state_err 0.0`.
- Impact: a correct pipeline fails its own done gate (smoke 4/5, train 3/5 within the band). The implementer then either "fixes" correct
  code or edits the number after the fact. The latch can also come before the end of the recording (703 < 726). The text "latch comes
  1-23 ticks later" (:172-173) is wrong too (-23 to +28). The stall argument still holds, since 28 < 60.
- Fix: drop the absolute band. Gate the latch on equality instead: the latch tick through 81/82 must equal the latch tick of a direct
  in-process replay of the same npz (deterministic, as shown). Or use a band measured on all 305 demos.

### F2 (major, purpose gap): no gate checks the observation half of the contract
- Trigger: `ReplayPredictor.infer(obs, tick)` returns `replay_chunk(actions, tick)` and ignores `obs` (decision (1), :86-89). The CPU
  checkpoint run (§3, :404-407) only shows that a 200-step model runs and fails. 83 reads frames straight from the npz and calls
  `SmolVLAPredictor` without 82 or the wire (decision (6)). `--self-test` also uses an npz frame.
- Impact: none of the gates checks that what 82 sends at tick t (image + state + prompt) is the training-data observation. All of these
  would pass replay 5/5 and the open-loop gate, then show up in Stage 9 as a low success rate that looks like "undertrained" or a
  "generalization gap": an observation taken once and reused, the image of the previous request tick, an observation taken after the
  action instead of before, an image from the wrong renderer or camera, or a typo in the prompt. That is exactly the wrong-verdict
  outcome the plan E decision rule is meant to prevent. The item asked for is "replay 5/5 proves the eval pipeline" (H.3), and today it
  proves only the action path.
- Evidence that the check is cheap and well defined: state after restore is bit-exact at every recorded tick (both probes), so the state
  sent at tick t must equal `state[t]` exactly. The image at tick 0 is within 0-2 grey levels (`probe_replay_events.log`). Later images
  differ only by an independent noise draw: 69 renders every tick, 82 only at request ticks, so `noise_rng` is at a different position.
  With `rgb_noise_std` up to 3, the mean absolute difference stays at noise level, about 3-4 grey levels, while a stale or wrong frame
  differs far more in the arm/ball region.
- Fix: (a) `ReplayPredictor.infer` compares the received observation with the recorded frame: `obs_state_err` (must be 0.0),
  `obs_image_maxdiff` at tick 0 (≤ 2), `obs_image_mad` at later ticks (below a bound measured once), and the prompt equal to the npz
  `task`. It returns these in the `actions` reply; 82 puts them in the row, and the Step 4 gate requires them. (b) One wire-equivalence
  check on the CPU smoke checkpoint: the chunk for `82 --from-handover <smoke npz>` at tick 0 with reset seed s, compared with the
  `--self-test` chunk on recorded frame 0 with the same seed. The max difference should be far below the 0.79 noise spread. (c) A mutation
  in Task 2 verify: send the tick-0 observation for every request, and the gate must fail on `obs_*`.

### m1 (minor, the 83 gate needs calibration): the hold baseline is the binding bar and is close to impossible for hands; "spread" is loose
- Evidence: `scratchpad/step4/review_openloop_baselines.log`, using frames picked exactly as decision (6) picks them (20 training demos,
  ticks 1..675):

  | Group | Hold-baseline MAE(1-25) | Frames where hold MAE < 0.005 | 0.5 x spread |
  |---|---|---|---|
  | Arms | 0.026 | 7/20 | 0.108 |
  | Hands | 0.012 | 16/20 | 0.042 |
  | Waist | 0.036 | 12/20 | 0.198 |

  On the Task 3 verify set (smoke_v1, 4 frames) the hands' hold MAE is **0.0000**, so "< hold baseline" cannot be met there.
- Impact: "spread" is the std across phases, so 0.5 x spread barely constrains anything. A well-trained model can still fail on hands
  (and possibly waist) because "hold still" is nearly perfect on 16 of 20 frames. Since 83 is diagnostic, this is not a blocker, but the
  "gate" line will read FAIL on a good model. Also, the probe reference "arm std 0.104" (:227-228) is a within-window std, not the
  plan's spread, so it cannot be compared with the gate.
- Fix: report a skill score `1 - MAE/hold_MAE` only on frames where `hold_MAE(1-25)` exceeds a floor (motion frames), per group.
  Drop the hands hold condition or make it apply to motion frames only, and define "spread" the same way in the probe and the gate.

### m2 (minor, hard-rule guard): 81 `--checkpoint` defaults to `--device mps` with no check for a running lerobot-train
- Trigger: Task 1 action (:281, :289). 83 gets a `pgrep -f lerobot-train` refusal (:377-378) but 81 does not. A plain
  `81 --checkpoint <ckpt>` during run1 loads the model on MPS next to the training, the one thing §0 forbids.
- Fix: the same guard plus `--force-mps` in `SmolVLAPredictor` (which covers 81 and 83 at once), or default 81 to cpu while run1 exists.

### m3 (minor, inferred, not measured): the stall threshold may miss real BC stalls
- A 60-tick window spans 2-3 chunk boundaries at exec 25. With fresh flow-matching noise per chunk, a policy hovering at one pose still
  jumps between chunks. For the untrained smoke model two noises gave chunks up to 0.79 apart (`probe_cpu_chunk.log`); for a trained
  model I do not know the size. A range threshold of 0.01 over all 27 dims then reports "not stalled" for a hovering policy. The
  teacher-side margin (0.038 minimum) is measured, but the policy side is not.
- Fix: add a progress-based diagnostic: no new event, and ball displacement under X cm over the last N s before timeout. Re-calibrate
  `STALL_RANGE` on the first run1 eval. Keep the unit tests on the window function.

### m4 (minor, wire robustness): `recv` bounds only the header length
- Decision (1)/Task 1 (:276) refuse `length > MAX_BYTES` for the JSON header, but array sizes come from header shapes and are not bounded or
  validated (negative, non-int or huge shapes). Fix: before reading the body, compute the sum of `prod(shape) * itemsize`, refuse it above
  `MAX_BYTES`, and require shapes to be lists of non-negative ints. Add one unit test.

### m5 (minor): the prompt is only checked for "non-empty"
- `check_observation` (:85) accepts any non-empty prompt, and 82 types the task string itself. Fix: one shared constant (69's `TASK`
  moved to `episode` or `vla_eval`), and in from-handover mode assert the prompt equals the npz `task` (also covered by F2 (a)).

### m6 (minor): integration test (c) re-implements "a 69-style recorder"
- A copy of 69's recording loop can drift from 69: the tail, the pending observation, the snapshot moment. Fix: import 69 with
  `importlib` (the 69:48-50 pattern) and call `record(env, 100, 0, tmp_path)`. It writes the npz + sidecar into `tmp_path`, never to the T7.

### Nits
- Step from the folder name: `checkpoints/last/pretrained_model` gives step "last". Resolve the symlink first.
- 83: `--frames 1` divides by zero (`F - 1`). `np.load(...)["head_image"]` decompresses the whole member, so "no full-episode copy" is
  inaccurate (harmless).
- Verdict boundary in floating point: `0.8 - 0.2 = 0.6000000000000001`, so held-out 0.6 counts as a "gap". Round or use `<=` explicitly.
- Reset seeds: no-walk seed 0 (`trial`) and from-handover (`demo index`) reuse the same noise seeds 0..N-1. Harmless, but `seed*100000 + trial`
  suggests uniqueness that is not there.
- The §3 gate commands leave the background servers running. Add the kill step.
- Reading run1 checkpoints while training writes them (R4): skip a checkpoint dir younger than ~60 s or one without a complete `model.safetensors`.

## Checks not run (gate stays OPEN on these)
- I loaded no model, not even on CPU. The API behaviour (load 4.6 s, chunk (1,50,27), same noise gives identical output, processor steps) is
  taken from the author's probe logs plus my own reading of the installed sources. I did not reproduce it.
- No MPS latency (forbidden while run1 trains; the plan reports it as open).
- Not run: teacher-vs-event agreement on seed 0 (no-walk), integration (a) "first image identical" across two TableEnvs in one process
  (risk of 1-2 grey levels across GL contexts; reset path, so probably exact), and the seed 101 trial 0 handover equality through
  `no_walk_handovers` (the author's `teacher_repro.log` shows the 69 sequence is bit-exact).

## Overall: NO-GO for building as written. Two small plan edits make it GO.
Required before the build: **F1** (replace the 24-26 s band with latch equality against a direct replay) and **F2** (observation checks
in the replay predictor plus one wire-equivalence chunk check on the CPU smoke checkpoint, with a stale-observation mutation).
Recommended in the same revision: m1 (83 gate calibration), m2 (MPS guard in `SmolVLAPredictor`), m4, m5. m3 and m6 can go in the build.
The rest of the plan is sound: the API claims match lerobot 0.6.1, the no-walk handover copies 69 exactly, the event checks follow
`ball_grasp`, the bit-exact replay holds on the gate's own 10 demos, and the run1 constraints are respected. A quick re-check of the F1/F2
text is enough; a full re-review is not needed.

## Re-check of revision 2

Reviewer: an independent gate (different from the author). I did not write the plan or the first review and saw no author reasoning
beyond the plan text. Date: 2026-10-15. Scope: only F1, F2, m1, m2, m4, m5, m6, file ownership and the wire contract.
Hard rules kept: no model loaded (CPU or MPS), no lerobot-train, run1 not read (only `pgrep -fl lerobot-train` for its command
line), nothing written to the T7. The only runs were import checks in the project `.venv` (logs `scratchpad/step4/recheck_import_*.log`).

### F1: addressed
- The absolute band is gone (grep finds "24-26" only in the revision table). The gate is now equality with
  `vla_eval.reference_replay`: a fresh TableEnv, `actions[min(t, T-1)]` per tick, the same `EventTracker`, no wire and no
  `replay_chunk` (decision (4) :201-204, `replay_gate_problems` :212-215, §3 :519-522).
- The action sequences are the same by construction: `replay_chunk` pads with the last row, and the reference holds `actions[T-1]`.
  Rendering only at request ticks does not touch the physics: `plan2_obs_probe.py` asserts state equal to the recorded state at every
  request tick (renders every 25) while 69 rendered every tick, and it passed on all 10 demos. The first review's preview showed the
  latch to be deterministic. The noise stream is separate from `env.rng` (`table_env.py:64-66, 116`), and `restore_handover` restores it (`:267`).
- The tick convention is stated (:107-109). The probe text is corrected (703-754, -23..+28).

### F2: addressed
- (a) `checks` are fixed in the contract (:97-101), aggregated per trial (:229-232) and gated (:212-215, §3). The thresholds hold
  against the code: `configs/randomization.json:49` fixes `rgb_noise_std` at 3.0, not a range, and noise is added and then truncated
  to uint8 (`table_env.py:330`). With two independent draws, the expected MAD is about sqrt(2*9 + 1/6) * sqrt(2/pi) = 3.40, which is
  the ceiling and matches the measured max of 3.39 (`plan2_obs_probe.log`). 4.0 holds with margin as long as the config stays at 3.0.
  Tick 0 diff 0 on 10/10, bound 2 (`restore_handover` docstring: up to 1 grey level across GL contexts). `obs_checked` is tied to the
  request count, so a request that skips the checks cannot pass.
- (b) The wire check is concrete. Both sides use the same noise seed, `noise_seed(stem)` (self-test :374, 82 decision (5) :252), the
  same CPU thread count (4) and the first infer after a reset. The tick-0 image diff is 0 on this demo, so the expected diff is 0.0
  with tolerance 0.01 (§3 :527-537). `episode.policy_observation` exists (`episode.py:34`) and builds exactly the
  `POLICY_KEYS` + `prompt` payload.
- (c) Integration test (d) and mutation (iii) are permanent and explicit. By the probe, a stale observation differs in state by
  >= 0.007 at every later request tick, so the bit-equal state check catches it.

### Minor items
- m1: addressed (decision (6) :272-283). The motion/still counts match `review_openloop_baselines.log`: the default set has still
  frames 7/16/12 out of 20, i.e. 13/4/8 motion frames. The smoke 4-frame set has still frames 3/4, 4/4 and 1/4, i.e. 1/0/3 motion
  frames. So Task 3's expected n/a for arms and hands is right.
- m2: addressed. `device_problem` comes first in `SmolVLAPredictor`, before the checkpoint is resolved or torch is imported (:143-146),
  and the 81 check uses a non-existent path. The run1 line really carries `--output_dir=/Users/finneganadam/dev/g1-ext/smolvla-runs/run1`
  (`pgrep -fl lerobot-train`, today, one line). Small residual: the 83 MPS check (:499) uses the real `$SMOKE` and relies on the same
  guard having passed in Task 1. That is stated and acceptable.
- m4: addressed (:84-88, tests :293-295, including the "header only, must raise instead of blocking" case).
- m5: addressed (`policy_wire.TASK`, an assert in 82 :261/:436, and the pin against 69's `TASK` at `69:53` in integration (c)).
- m6: addressed. `record(env, seed, trial, out_dir, reporter=None)` (`69:123`) writes only to `out_dir`. `OUT` (`69:54`) is not used
  on import, and `__main__` is at `69:254`.

### File ownership and contract
- Disjoint: Task 1 {policy_wire, 81, test_unit_policy_wire}, Task 2 {vla_eval, 82, labview, test_unit_vla_eval,
  test_integration_vla_eval, test_unit_labview}, Task 3 {83}. No file appears twice. The `replay_chunk` mutation (a Task 1 file)
  moved to the integration step. Docs are owned by the integration step.
- Wire contract fixed in the text: framing, bounds, dtypes, message types and fields, `checks` keys, the exact `policy_wire` API,
  the predictor interface, and the `SmolVLAPredictor` signature and init order.

### New finding m7 (minor, non-blocking; fold into Task 2 at build time)
- Trigger: Task 2 verify "before Task 1 lands: `.venv/bin/python -m pytest tests/test_unit_vla_eval.py ...`" (:452), and
  "numpy only, no MuJoCo" (:309). `vla_eval` needs `ball_grasp.MIN_LIFT`, `FORBIDDEN` and `placement_checks`, plus `table_env` and
  `randomization` for `reference_replay` and `no_walk_handovers`. In the project `.venv`, `from g1sim import ball_grasp` fails with
  `No module named 'mink'`, and `table_env` and `randomization` fail with `No module named 'huggingface_hub'`. Only `episode` and
  `labview` import (`recheck_import_*.log`).
- Impact: a module-level `from g1sim import ball_grasp` (the natural way to write it) breaks collection of `test_unit_vla_eval.py` in
  the project venv and in `scripts/run_tests.sh`'s project-venv pass. The plan's "MuJoCo only inside the functions that take an
  env" does not cover `reference_replay` (it takes `ep`, not an env) or the constants the pure tracker needs.
- Fix (one sentence in Task 2): at module level `vla_eval` imports only numpy, `g1sim.episode` and `g1sim.action_space`;
  `ball_grasp`, `table_env`, `table_scene` and `randomization` are imported inside the env-side functions. The tracker gets `min_lift` and
  the basket half-width from local constants that the integration test pins equal to `ball_grasp.MIN_LIFT` and
  `table_scene.BASKET_INNER_HALF + BASKET_WALL`.

### Nits
- 83 `--run <run_dir>`: `checkpoint_problem` on the run dir itself must treat "equal to output_dir" as inside, not only strict
  descendants. State this in the docstring or test it.
- `training_cmdlines` matches any process whose command line contains `lerobot-train`, for example a shell wrapper whose command text
  mentions it. Such a line has no `--output_dir`, so every checkpoint is refused. That is conservative and safe, but a surprising refusal.

### Verdicts
- F1 and F2: correctly and concretely addressed. m1, m2, m4, m5 and m6 are addressed. Ownership is disjoint, and the wire contract is fixed.
- Not run (still open from the first review, covered by the plan's own verify steps at build time): model load and same-noise
  determinism on CPU, and MPS latency (forbidden while run1 trains).
- **Overall: GO for the build**, with m7 folded into Task 2 (one sentence; non-blocking because the Task 2 verify would surface it at once).
