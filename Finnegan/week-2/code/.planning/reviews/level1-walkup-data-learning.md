# Review: level1-walkup PLAN.md, lens DATA + LEARNING SOUNDNESS (plan review, pre-build)

Reviewer: independent gate (did not author the plan, no author reasoning seen). Date 2026-10-08.
Inputs read: `.planning/level1-walkup/PLAN.md`, `PROGRESS.md` (NEXT STEP + log 2026-10-07/08), `PLAN.md` scope update (h)/(i),
`g1sim/{action_space,episode,table_env,ball_grasp,randomization,payload,labview,navigation}.py`, `scripts/{64_demo_factory,67_volleyball_pick_place}.py`,
`openpi_g1/{convert,config,policy}.py`, `tests/test_unit_openpi_policy.py`, and the installed SmolVLA env
(`~/dev/g1-ext/smolvla-test/.venv`, lerobot 0.6.1 source + cached `lerobot/smolvla_base` config).
Fixed user decisions (scripted walk-up then VLA, camera check for walk-up, vary start/ball/basket) were not challenged.

## Verdict 1: is the plan internally sound and buildable as written?
Mostly yes. The 27-D contract, episode boundary, recorder pattern and the LeRobot 0.6.1 API it names exist as described.
Three gaps make specific gates unreliable as written: the replay gate and the teacher-vs-VLA pairing assume a reproducible
walk-mode handover that the code does not give (M2); the SmolVLA server's checkpoint path (normalization processors) is
not checked before Stage 9 (M3); eval denominators and success latching are not defined (M4). One test file that the
change breaks is missing from Stage 1 (m1).

## Verdict 2: is it the right plan for the goal?
The data design is right: episodes start at the handover, actions are commanded targets, there is one head camera like
the real G1, only successful episodes are kept, demo seeds are separate from eval seeds, a sidecar is written per episode,
and there is a validator. The training budget does not fit the dataset. 20k steps at batch 8 is 0.7 passes over 300
episodes of about 750 frames. That is about 50x fewer samples per episode than LeRobot's own SmolVLA recipe. The plan's
decision rule ("< 30 % -> more demos") would then push in the wrong direction (M1). The timeline is optimistic: a
Thursday-night training start is realistic only with the no-walk fallback.

## Overall: NO-GO (revise once). No blockers. Folding M1-M4 into the plan text is enough for GO.

---

## Findings

### M1 (major): the training budget is below one pass over the data, and the "< 30 % -> more demos" rule can misdiagnose
- Evidence:
  - Frames per episode, summed from the `ball_grasp.py:61-64` phase durations plus the new raise (1.5 s) and align (1.0 s)
    and the 1 s tail: raise 1.5, face 2.0, approach 2.0 + settle 1.0, descend 1.0 + settle 1.5, touch 1.5 + settle 1.0,
    squeeze 1.0 + 0.5, lift 2.5 + sag fix 0.9, turn 4.0 + 0.5, align 1.0, open 0.3, release 0.8, back_off 1.0, tail 1.0.
    Total 25.0 s, so 750 frames (the plan says 650-750 and 600-800). 300 demos give about 225k frames.
  - Training: 20k steps x batch 8 = 160k samples, which is 0.71 passes and about 530 samples per episode.
  - Reference: LeRobot's SmolVLA docs (fetched just now, `docs/source/smolvla.mdx` lines 60-61) use `--batch_size=64 --steps=20000`
    on `lerobot/svla_so101_pickplace`, whose `meta/info.json` (fetched) has 50 episodes and 11,939 frames. That is about 107 passes and
    about 25,600 samples per episode.
  - Throughput on this Mac is fixed at about 9.5-10 samples/s (PROGRESS log 2026-10-07: 0.84 s/step at batch 8, 1.6 s/step at batch 16,
    measured on a synthetic batch, so without video decode). A larger batch does not help.
  - smolvla_base trains only the action expert (`train_expert_only: true`, `freeze_vision_encoder: true` in the cached `config.json`).
    Image grounding of ball and basket positions therefore rests on a small trained part.
- Impact: Stage 9 will likely report low success, caused by undertraining rather than too little data. R7's rule "< 30 % -> 500 demos
  and retrain" lowers the passes per episode further (0.43) and spends Friday on the wrong fix.
- Fix:
  - (a) Before reading the eval, separate undertraining from a generalization gap. Run the closed-loop eval on 10 training
    variations (demo seeds, from their handover snapshots, see M2) as well as on seed 0. If training-variation success is also low, the
    model is undertrained: continue training (resume; plan for 2-3 nights, or the GPU path) and do not add demos.
  - (b) Log the training loss curve and per-checkpoint open-loop action error (M3) at 5k/10k/15k/20k steps.
  - (c) Consider 150-200 demos for v1 (more passes per episode at the same compute), or drop mirrored layouts in v1 (halves the modes).
    The plan's drop order already lists both.
  - (d) State the expectation in the plan: the first overnight run is a pipeline proof. The ≥ 80 % Level-1 target is not expected
    from it.

### M2 (major): the walk-mode handover is not reproducible, so the replay gate (Stage 5/H.3) and the paired teacher comparison (E) are unreliable
- Evidence:
  - Camera noise comes from one stream per env: `table_env.py:50` `self.noise_rng = np.random.default_rng([seed, 1])`, consumed on every
    render (`table_env.py:197-203`).
  - In 68, one env records many trials (pattern `64_demo_factory.py:101-107`). The noise that trial k's `look()` sees therefore depends on how
    many images trials 0..k-1 rendered.
  - The camera estimate drives the stop goal (plan B: goal = rule(estimates) + perturbation), and the goal drives a GR00T walk with
    contacts. A fresh 82 process re-running the walk-up from `<demo.json>` (Stage 5 verify `--replay-variation`) therefore reaches a
    different arrival pose. The open-loop recorded actions are then replayed against a ball and basket that are shifted relative to the
    robot.
  - The Stage 5 done criterion uses "5 demos from the Stage 6 smoke set", which is recorded `--walkup`.
  - Likewise, E's "teacher on the same 50 seeds" compares the teacher and the VLA on different handovers: the VLA process renders every tick,
    the teacher process does not, so their noise streams diverge after trial 0.
  - No-walk replays are fine: `reset(variation=...)` is deterministic (`table_env.py:83-107`) and the teacher does not use camera
    noise.
- Impact: the replay gate either fails for reasons unrelated to layout, timing or the waist channel, or passes only by luck. The "upper
  bound" comparison is not paired.
- Fix:
  - Save the full physics state at the handover: `mujoco.mj_getState(..., mjSTATE_INTEGRATION)`. That includes qpos, qvel, mocap and
    eq_active. Also save the head tilt and the active timestep in the npz/sidecar.
  - Give 82 a `--from-handover <file>` mode that restores this state instead of walking. Use it for the replay gate, for the paired
    teacher/VLA comparison (run the walk-up once per eval seed, snapshot, then run both from the snapshot), and for M1(a).
  - Seed the noise stream per trial (`default_rng([seed, trial, 1])`) so that walk-ups are reproducible per trial.

### M3 (major): the SmolVLA server's checkpoint path has no check before Stage 9
- Evidence:
  - In lerobot 0.6.1, normalization lives in separate processors, not in the policy. `lerobot_train.py:345-395` builds
    `policy_preprocessor`/`policy_postprocessor` with dataset stats and the rename_map; the base model ships them as separate files
    (`policy_preprocessor.json`, `policy_postprocessor.json`, MEAN_STD for state and action).
  - Stage 5 specifies only "SmolVLA `select_action`/chunk on mps". The replay mode bypasses the model entirely. The "< 1 s with
    smolvla_base" check exercises the base model, not a fine-tuned checkpoint with our stats and key mapping.
  - A server that skips the checkpoint's pre/postprocessors (normalization, rename, uint8 HWC -> float CHW in [0,1]) produces
    actions in normalized units. In Stage 9 that would look exactly like "the model did not learn".
- Fix:
  - Stage 5/8: the server must load the processors from the checkpoint dir (`make_pre_post_processors(pretrained_path=<ckpt>)`) and use
    `predict_action_chunk`.
  - Add an open-loop check on the first checkpoint: feed 20 recorded frames of training episodes through the server and compare the
    returned chunks with the recorded next-50 actions (MAE per joint group, in rad and hand units). Gate: the error falls with training
    steps and is well below the spread of the actions. Run it at 5k steps, before the overnight run finishes.

### M4 (major): eval denominators, walk-up failures and success latching are undefined
- Evidence:
  - E/Stage 9 define VLA success as "the same 7 checks, computed every tick from events". The teacher computes `ball_carried_to_basket`
    at the end of its own turn phase (`ball_grasp.py:186, 212`), and the placement checks over a fixed 3 s window after its own back_off
    (`ball_grasp.py:197-203`). The VLA has no phases.
  - The plan does not say how an eval trial whose scripted walk-up fails (`stance_outside_envelope`, `detection_error`, `fell`) counts.
- Impact: the headline VLA success rate is ambiguous. Walk-up failures would be charged to the VLA, or silently dropped.
- Fix:
  - Define the denominators: VLA success = passes / trials that reached the handover. End-to-end = passes / all trials. Report both.
  - Define the event checks: lifted = max(ball z - rest) ≥ 0.06; carried = ball z - rest ≥ 0.06 at some tick while its xy is inside the
    basket footprint.
  - Success latches at the first tick where all 4 placement checks have held continuously for 3 s, with no forbidden contact before that
    tick. Then stop the trial.
  - Add "furthest event reached" and "stalled" (|Δaction| < ε for > 2 s before success) to the failure table. The demos contain about 6 s of
    near-static holds (settle/sag-fix/turn_hold, `ball_grasp.py:161-163, 169, 185, 288-293`), which are a known cause of BC stalls.

### m1 (minor): Stage 1 misses `tests/test_unit_openpi_policy.py`
- Evidence: line 61 asserts `data["actions"][:, 14:] == actions[:, 14:]` after `DeltaActions(DELTA_MASK)`. With the planned
  `DELTA_MASK[26] = True` (plan Stage 1a), index 26 becomes a delta, so this assertion fails in the openpi env. The Stage 1 file list
  does not include this test. The verify grep `grep -rn "14:\]" g1sim scripts` does not search `tests/`.
- Fix: add the file to Stage 1 (hands = `SLICES` 14:26; waist checked as a delta), and grep `tests/` too.

### m2 (minor): video encoding changes the images the VLA trains on
- Evidence: plan D/Stage 7 store the head image as video. The recorded frames carry iid noise with std 3.0 (`configs/randomization.json`
  `rgb_noise_std`). A lossy codec removes most of that noise and adds block artefacts, while eval feeds raw noisy renders. Validator PSNR
  against the noisy npz is capped near 20·log10(255/3) ≈ 38.6 dB by the noise alone, so the ≥ 30 dB gate is not very tight.
- Fix: for v1, prefer `use_videos=False` (the `image` dtype; lossless; also removes the decode cost the plan already worries about; about
  30-40 GB on T7 is fine). Otherwise accept the mismatch knowingly. In 0.6.1 the codec is set through `rgb_encoder: RGBEncoderConfig`
  (`lerobot_dataset.py:697`), not a `vcodec` argument.

### m3 (minor): the eval observation path must match the recorder exactly
- Evidence:
  - 67 builds its env with a single 400x640 head renderer (`67_volleyball_pick_place.py:44, 89`), shared with the Lab View
    (`labview.py:263-276`). Recording follows 64 and uses the env default of 200x320 (`table_env.py:31`).
  - If 82 is modelled on 67, the policy sees 400x640 renders (or downsampled ones). SmolVLA's resize-with-pad to 512 hides the
    difference without an error.
- Fix: 82 must use the `TableEnv` default `IMAGE_SIZES` and `episode.policy_observation(env.observation(), prompt)`, exactly as 68 does. The
  server rejects any `head_image` that is not (200, 320, 3) uint8.

### m4 (minor): SmolVLA feature mapping
- Evidence:
  - With `--policy.path`, `make_policy` keeps the base model's `input_features` (`factory.py:305-306` fills them only when they are empty).
    So the policy expects `observation.images.camera1..3` and a state of shape [6]. The normalizer is then given those features with 27-D
    dataset stats (`lerobot_train.py:357-372`).
  - Visual validation passes when the dataset's cameras are a subset of the policy's (`policies/utils.py:241-249`). Missing cameras are
    simply dropped when `empty_cameras = 0` (`modeling_smolvla.py:373-379`).
- Fix: name the dataset key `observation.images.camera1`, so no rename_map is needed. In the 200-step smoke run, assert that the saved
  checkpoint's normalizer stats for `observation.state` and `action` have 27 entries. "loss at 200 < loss at 10" alone does not prove the
  mapping.

### m5 (minor): no-walk fallback data vs walk-mode eval
- Evidence: in walk mode the handover keeps GR00T's residual torso roll/pitch and the waist roll/pitch (plan B: "waist target = current
  waist qpos"). No-walk places a level pelvis at `STAND_Z` with waist 0. Those few degrees change the camera view and the arm kinematics.
  Stage 4 checks only pelvis z.
- Fix: log pelvis roll/pitch and waist roll/pitch at the handover in Stage 4. If data is recorded in no-walk mode, sample those from the
  Stage 4 distribution, or evaluate in no-walk mode too.

### m6 (minor): randomization groups and logging
- Evidence: `randomization.sample()` also draws the `stand` (±10 cm, ±10°) and `objects` groups (`randomization.py:77-82`). For a free
  base, `ModelVariation.apply` shifts the pelvis `body_pos` (`randomization.py:137-139`), which the free joint overrides. The sidecar
  would then log a stance offset that was never applied.
- Fix: `sample_volleyball` disables `stand` and `objects` explicitly. Stance lives only in `stance` and `walk_start`.

### m7 (minor): openpi path "needs no code change" is true only for convert
- Evidence: `openpi_g1/config.py:28` `REPO_ID = "g1_brainco/red_bottle_sim"` and `:80` `reset_pose = READY_POSE`, but volleyball
  episodes start at `VOLLEYBALL_START_POSE`. `convert.py:35` writes PNGs under `HF_LEROBOT_HOME` on the internal disk (136 GiB free).
- Fix: note these for the GPU day: a new repo id/config, the reset pose, and `HF_LEROBOT_HOME` on T7.

### m8 (minor): statistical resolution of the eval
- 50 trials give a 95 % interval of about ±12-14 percentage points around 30-80 %. Read the 30 % / 50 % / 80 % thresholds (R7, Sat-Sun row,
  Stage 9) with that margin. Report Wilson intervals.

## Timeline
- Thursday holds Stages 3 (6-8 h), 4 (4-6 h), 5 (4 h) and 6 (2-3 h), then recording, conversion and validation. The Stage 3 teacher
  generalization (raise, align, any stance, ≥ 85 % over 200) is the main risk.
- Starting training on Thursday night is realistic only if the "no-walk by 18:00" rule fires.
- With M1, Friday's eval is a pipeline proof. A meaningful Level-1 VLA number is realistic on Saturday at the earliest (second night of
  training). The plan already shrinks the Level 2 weekend; keep that.

## Checks not run
- No plan code exists yet, so nothing was executed against new code.
- Reproduction of M2 is by code reading (`table_env.py:50, 197-203` + `64_demo_factory.py:101-107`), not by a run.
- SmolVLA throughput with real video decode was not measured; the 0.84 s/step figure is on a synthetic batch (PROGRESS log).
