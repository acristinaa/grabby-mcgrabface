# Review: Level 1 Step 6 / plan Stage 7 - converter 79, validator 80, g1sim/episode.py

Reviewer: independent (did not author; no author reasoning seen). Stage: finished review. Date 2026-10-14.
Scope: `scripts/79_convert_smolvla.py`, `scripts/80_validate_dataset.py`, `g1sim/episode.py`, against PLAN.md D, E, L and the Stage 7 task,
checked against the installed lerobot 0.6.1 (`~/dev/g1-ext/smolvla-test/.venv/lib/python3.12/site-packages/lerobot/`).
Scratch evidence: `/private/tmp/claude-501/-Users-finneganadam-Documents-Projekt-Tim-and-Lukas-Cursor-Software-Engineering-SAP-Robot-Project-First-Ideas/f0ff7f6a-b54c-4e4d-a5d8-61eb8f19683e/scratchpad/review2/`
(`convert.log`, `full_check.log`, `appledouble_load.log`, `train_smoke.log`, `sampling_miss.log`, `validate_review2.log`).

## Verdicts
1. Implementation fidelity (did the code do what the plan said): **mostly yes.** 79 does what D and Stage 7 say. 80 has small gaps (F4, F5).
2. Plan correctness (is the intent right for the purpose): **no, in one place.** The plan puts the v3.0 dataset on the exFAT T7. lerobot cannot load
   a dataset there once macOS has written its `._*` twins (F1). The rest of the intent is right for SmolVLA training: a real `lerobot-train` run worked.
3. Overall: **NO-GO** for running 79 with the documented `--root` on `/Volumes/T7`. The fix is small (F1). Once F1 is fixed: GO.

## What I re-derived (works)
- Reconverted the 5 smoke demos into `review2/ds`: exit 0, 5 episodes, 3630 frames, 10 s.
- Every frame, not a sample (`full_check.log`): state and action bit-equal to the npz in all 3630 frames. Images bit-equal in 3630/3630
  through the public `ds[i]` path. Timestamps match the npz to 8.9e-7 (float32). `index` is contiguous from 0. Task = "put the volleyball in the basket".
- Stats: state/action mean, std, min and max from `meta/stats.json` equal numpy over all npz frames (max diff 3.9e-6; min/max exact), 27 entries.
  Image stats exist with shape (3,1,1). The smallest std is 0.053 (shoulder roll), so no dimension is near zero and MEAN_STD stays safe.
- The uint8 -> `to_tensor` -> `*255` round trip is exact for all 256 levels, so 80's image comparison really is bit-exact.
- API use matches the installed source: `create(..., use_videos=False)`, `add_frame` (validate_frame accepts the (200,320,3) HWC uint8 image
  and float32 vectors; timestamp/frame_index are computed internally as frame_index/fps, `dataset_writer.py:226-227`), `save_episode`, `finalize`.
  `dataset_to_policy_features` turns names `[height,width,channel]` into (3,200,320) (`utils/feature_utils.py`).
- **SmolVLA training finds its keys:** `lerobot-train --policy.path=lerobot/smolvla_base --dataset.root=review2/ds --steps=2 --batch_size=2
  --policy.device=cpu` ran to the end (`train_smoke.log`: loss 1.908 / 2.603, 100M trainable params). Here is how the keys work out:
  `factory.py:304` overwrites output_features from the dataset (action 27). `validate_visual_features_consistency` passes because {camera1} is a subset of
  {camera1,2,3}. `prepare_images` (`modeling_smolvla.py:340-374`) uses the cameras that are present and pads none (empty_cameras=0). State 27 is padded to 32.
  The normalizer gets the dataset stats (`lerobot_train.py:351,376`). The base config's `observation.state` shape [6] stays in the config, but
  modeling only reads `action_feature.shape` (lines 217, 302), so it does not change training. Stage 8's planned config assertion still applies.
- AppleDouble/temp files in the **input** folder: `glob("episode_*.npz")` does not match `._episode_*` or `*.part`. Smoke run: 5/5, nothing skipped.
- libavdevice warning: it appears when PyAV (`av/.dylibs/libavdevice.61`) and torchcodec (which loads Homebrew ffmpeg 8 `libavdevice.62`) are both
  imported. `import av; import cv2` gives the same warning against cv2's bundled copy. The duplicated classes are `AVFFrameReceiver`/`AVFAudioReceiver`
  of the avfoundation *capture device*. This dataset has no video keys, so neither conversion nor training decodes video or opens a device.
  It does not matter here. The 2-step training ran with the warning present.
- The recording is progressing (read-only): `level1_walkup_v1` has 154 npz files (s101-s103: 40 each, s104: 34). The factory reports were updated at 18:19. No 0-byte or `.part` files.

## Findings

### F1 - blocker - the dataset root on exFAT T7 makes the dataset unloadable (80 and lerobot-train crash)
- Where: `scripts/79_convert_smolvla.py:12` (documented `--root` on `/Volumes/T7/...`), PLAN.md D "Conversion" ("about 30-40 GB on T7").
  The lerobot code involved is `datasets/io_utils.py:75` `paths = sorted(pq_dir.glob("*/*.parquet"))`, used by `load_episodes` (`dataset_metadata.py:218`) and by the data load.
- Trigger: every file these processes write on the T7 has the `com.apple.provenance` xattr (`xattr -l` on the smoke npz/json). exFAT cannot store
  xattrs, so macOS writes a `._<name>` twin: `level1_walkup_v1/` has one for every npz, json and even `.lock_s10X`. 79 on the T7 will therefore leave
  `meta/episodes/chunk-000/._file-000.parquet` and `data/chunk-000/._file-NNN.parquet`. Python 3.12 `pathlib.glob("*/*.parquet")` matches
  dot-files (checked: it returned `._file-000.parquet` first).
- Impact: `LeRobotDataset(...)` raises `pyarrow.lib.ArrowInvalid: Parquet magic bytes not found in footer` while loading metadata. 80 crashes
  (no report), and lerobot-train for night 1 crashes the same way. Conversion itself probably completes, because the writer never globs. So the failure shows up only at
  validation or training, after about 10+ min of conversion.
- Evidence: `review2/ds_ad` = clone of the good dataset plus a real T7 AppleDouble file (copied from `smoke_v1/._episode_s100_t0000.npz`) saved as
  `._file-000.parquet` in both globbed dirs. Load fails with the error above (`appledouble_load.log`). Not reproduced by writing to the T7 itself (forbidden here).
  The twins in `level1_walkup_v1` make this near-certain.
- Fix options (pick one): (a) convert to the internal APFS disk (5 demos = 437 MB, so 300 ≈ 26 GB; 133 GiB free); (b) keep the T7 and have 79
  delete `root.rglob("._*")` after `finalize()`, and have 80 report any `._*` under root as a problem. Nothing reading the dataset writes xattrs afterwards.
  (a) also avoids about 2 x 218k small PNG + AppleDouble creates and deletes on exFAT during conversion (the temp images, `dataset_writer.py:257-263`).

### F2 - minor - every load copies the whole dataset into the HF datasets cache on the internal disk
- Where: lerobot `io_utils.py:82` `Dataset.from_parquet(...)` builds an Arrow cache under `HF_DATASETS_CACHE` (default `~/.cache/huggingface/datasets`).
- Evidence: each smoke-dataset load today left a 447,372 KB `parquet/default-*` folder (two from the author's runs at 18:06/18:07, and see the
  side effect note below). `review2/hfds` reached 874 MB after loading two 437 MB datasets.
- Impact: the 300-demo dataset costs about 26 GB on the internal disk in addition to the T7 copy. Every reconversion (`--overwrite`, new fingerprint) adds another
  26 GB, and old caches are not removed. 133 GiB free today, so two or three reconversions plus checkpoints can fill the disk during a night run. The plan's disk budget
  does not include this. Fix: set `HF_DATASETS_CACHE` on purpose in `train_smolvla.sh` and in the 80 call, and clean old `parquet/default-*` after a reconversion.

### F3 - minor - 80 samples frames, so it can miss a corrupted or misaligned episode at full scale
- Where: `scripts/80_validate_dataset.py:78` (`--frames` default 20).
- Evidence: `sampling_miss.log`. State of episode 3 shifted by 0.01 rad (left elbow) in every frame of a scratch copy. 80's `check_dataset` missed it
  in 4/6 seeds with 1 frame and 3/6 with 2 frames. At 300 episodes x 726 frames with 20 frames, P(miss one bad episode) = (1-1/300)^20 = 0.935.
  The swap counter-test only caught it because there were 5 episodes and 50 frames.
- Also self-referential: the per-episode length check compares the dataset with `frames` from `convert.json` (written by 79). 80 loads every npz in
  `check_demos` but never compares `len(ep["timestamp"])` with the map. Stats are checked for length only, not values.
- Impact: "the dataset is exactly the recorded data" holds for 20 frames, not for the dataset. The real risk is low (79 and the map come from one loop),
  but the full check is cheap. Fix: in 80, compare the full `observation.state`/`action` columns per episode against the npz (`ds.hf_dataset.select_columns`,
  no image decode; 80 already loads every npz). Compare `len(ep["timestamp"])` with the map. Recompute mean/std/min/max from the npz and compare them with `meta.stats`.
  Keep image sampling, or sample at least one frame per episode.

### F4 - minor - 80 does not cover two Stage 7 items and the documented call
- `80_validate_dataset.py:34-61`: the plan's "state within joint limits" check is missing. Hands are covered by `episode.validate`, waist by < 2.6 rad, but the 14 arm joints have no check.
  `stance_source` is only checked against the allowed set, and the counts per value are not reported.
- PLAN Stage 7 verify calls `80 <dir> <root>`. 80 only takes `--root` and validates the demos that the converter mapped. So demos in the folder that were never
  converted (conversion started while the 4 recorders were still running, or a folder left off the command line) pass silently. Fix: optional
  `folders` argument; report npz files present but not in the map.

### F5 - minor - 80 crashes instead of reporting on a bad sidecar
- `80_validate_dataset.py:50` `info.get("seed", -1) < 100` raises TypeError if `seed` is null. `:56` `info["variation"]` raises KeyError if it is missing.
  80 then prints no problem list. Low likelihood with 69's sidecars.

## Not checked / limits
- The `._*` creation for parquet files on the T7 was not observed directly (writing to the T7 is forbidden). The conclusion rests on the twins of every file
  in `level1_walkup_v1` and on the injection reproduction.
- Training on MPS and with `num_workers>0` was not run. The run used CPU, 2 steps, `num_workers=0`. The checkpoint save/processor-stats assertion is Stage 8's.
- Full-scale (300 demos) conversion time and size were extrapolated from the smoke run (437 MB / 5 episodes, 10 s), not measured.
- Side effect of this review: `full_check.py` ran without `HF_DATASETS_CACHE` and left one 437 MB cache folder
  `~/.cache/huggingface/datasets/parquet/default-2e81c481018ed79e` (18:12). It is safe to delete. All other writes went to `review2/`.
