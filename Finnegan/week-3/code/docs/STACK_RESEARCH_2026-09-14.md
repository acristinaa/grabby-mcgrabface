> **Raw report of an automated research run (2026-09-14), taken over without checking.** Basis for Stage 0.4 — check it together
> before the decision is written into PLAN.md.

# VLA training stack decision: π₀ / π₀.₅ for Unitree G1 (29 DoF) + BrainCo Revo2

Research date: **2026-09-14**. All repos were shallow-cloned and read locally; HF metadata was read via `huggingface.co/api/models/...`; GitHub issues/PRs via `gh`.

## Versions checked

| Source | Version / commit | Date |
|---|---|---|
| `Physical-Intelligence/openpi` | `215abfb` (main) | 2026-08-24 |
| `huggingface/lerobot` | `89236ea` (main, pyproject `0.6.2`) | 2026-09-14 |
| LeRobot PyPI | `0.6.1` | 2026-08-03 |
| `unitreerobotics/unitree_lerobot` | `41c2805` (pyproject `0.3.0`) | 2026-05-25 |
| `NVIDIA/Isaac-GR00T` | `51d4c89` (N1.7) | 2026-08-20 |
| HF `lerobot/pi05_base` | sha `b211f3d` | lastModified 2026-07-29 |
| HF `google/paligemma-3b-pt-224` | sha `35e4f46` | — |
| HF `nvidia/GR00T-N1.7-3B` | sha `2fc962b` | — |
| HF `lerobot/unitree-g1-mujoco` | sha `a38dc86` | — |

**Legend:** **VERIFIED** means read in a primary source at the version above. **UNVERIFIED** means an estimate, a secondary source, or something I could not confirm.

---

## 1. openpi: current state

### 1.1 PyTorch support: yes, with gaps (VERIFIED)
- README "Updates": "[Sept 2025] We released PyTorch support in openpi." PyTorch implementations exist for π₀ and π₀.₅. There is none for π₀-FAST. https://github.com/Physical-Intelligence/openpi#pytorch-support
- **Not supported in PyTorch:** π₀-FAST, mixed-precision training, FSDP, **LoRA**, EMA. Training runs in full bf16 (default) or full fp32 (README, same section).
- PyTorch setup has two awkward requirements:
  - It overwrites files inside the installed `transformers==4.53.2` (`cp -r ./src/openpi/models_pytorch/transformers_replace/* .venv/.../transformers/`). The README warns this can leak into the uv cache.
  - JAX checkpoints must be converted first with `examples/convert_jax_model_to_pytorch.py`.
- Training entrypoints:
  - PyTorch: `scripts/train_pytorch.py` (single GPU or torchrun DDP; multi-node).
  - JAX: `scripts/train.py`.
- PyTorch LoRA exists only as a community PR, **#854 (open, 2026-01-21)**. Issue #842 claims about 16 GB peak on an RTX 4090 (community claim, UNVERIFIED). https://github.com/Physical-Intelligence/openpi/pull/854 , https://github.com/Physical-Intelligence/openpi/issues/842
- Dependencies (pyproject.toml): `jax[cuda12]==0.5.3`, `flax==0.10.2`, `torch==2.7.1`, `transformers==4.53.2`, `requires-python>=3.11`. https://github.com/Physical-Intelligence/openpi/blob/main/pyproject.toml

### 1.2 Pinned LeRobot means dataset format v2.1 only (VERIFIED)
- `pyproject.toml` pins `lerobot = { git = "https://github.com/huggingface/lerobot", rev = "0cf864870cf29f4738d3ade893e6fd13fbd7cdb5" }`. `uv.lock` resolves it as `lerobot 0.1.0`.
- That commit is dated **2025-05-28** ("[Fix] Unpin torch beyond 2.6.0 & torchcodec beyond 0.2.1 (#1127)"). At that commit, `lerobot/common/datasets/lerobot_dataset.py` has `CODEBASE_VERSION = "v2.1"`. https://github.com/huggingface/lerobot/commit/0cf864870cf29f4738d3ade893e6fd13fbd7cdb5
- openpi's `src/openpi/training/data_loader.py` imports the old module path `lerobot.common.datasets.lerobot_dataset`.
- **v3.0 datasets cannot be read.** Open PR #972 (2026-06-12) says: "datasets produced by recent lerobot releases cannot be read by the pinned lerobot at all" (refs #948, #916). The PR only proposes a clearer error message. https://github.com/Physical-Intelligence/openpi/pull/972

### 1.3 OS support (VERIFIED)
- README: "The repo has been tested with Ubuntu 22.04, we do not currently support other operating systems." It also says an NVIDIA GPU is required. macOS is not supported, so on the M4 use Docker or the rental GPU only.

### 1.4 GPU memory, from the README table (VERIFIED; the table does not say JAX or PyTorch, but LoRA is JAX-only)

| Mode | Memory | Example |
|---|---|---|
| Inference | > 8 GB | RTX 4090 |
| Fine-tuning (LoRA) | > 22.5 GB | RTX 4090 |
| Fine-tuning (full) | > 70 GB | A100 80GB / H100 |

`fsdp_devices` can shard across GPUs (JAX). The README's DROID config needs about 2 days on 8×H100 for 100k iterations at batch 256. https://github.com/Physical-Intelligence/openpi/blob/main/examples/droid/README_train.md

### 1.5 How configs and new robots work (VERIFIED)
- **`TrainConfig` is a Python dataclass** in `src/openpi/training/config.py` (line 466). Its fields:
  - `model`, `weight_loader`, `pytorch_weight_path`, `pytorch_training_precision`
  - `lr_schedule`, `optimizer`, `ema_decay`, `freeze_filter`, `data` (a `DataConfigFactory`)
  - `batch_size=32`, `num_train_steps=30_000`, `fsdp_devices`, and others
- Configs are registered by name and selected on the CLI (tyro), e.g. `scripts/train.py pi05_libero --exp-name=...`.
- Registered configs at `215abfb`:
  - ALOHA: `pi0_aloha`, `pi05_aloha`, `pi0_aloha_towel`, `pi0_aloha_tupperware`, `pi0_aloha_pen_uncap`, `pi05_aloha_pen_uncap`, `pi0_aloha_sim`
  - DROID: `pi0_droid`, `pi0_fast_droid`, `pi05_droid`, `pi0_fast_full_droid_finetune`, `pi05_full_droid_finetune`, `pi05_droid_finetune`
  - LIBERO: `pi0_libero`, `pi0_libero_low_mem_finetune`, `pi0_fast_libero`, `pi0_fast_libero_low_mem_finetune`, `pi05_libero`
  - Debug: `debug`, `debug_restore`, `debug_pi05`
- **Adding a robot** follows the README "Fine-Tuning Base Models on Your Own Data" and the LIBERO example:
  1. Convert the data to a LeRobot (v2.1) dataset, modelled on `examples/libero/convert_libero_data_to_lerobot.py`.
  2. Write `XInputs` / `XOutputs` (`DataTransformFn`) in `src/openpi/policies/x_policy.py`. Existing ones: `aloha_policy.py`, `droid_policy.py`, `libero_policy.py`.
  3. Write a `DataConfigFactory` subclass modelled on `LeRobotLiberoDataConfig`. It holds a `RepackTransform` (dataset keys to inference keys), the data transforms, optional `DeltaActions` for absolute joint targets (grippers stay absolute), and model transforms.
  4. Add a `TrainConfig` entry.
  5. Run `scripts/compute_norm_stats.py --config-name ...`.
  6. Train.
  7. Serve with `scripts/serve_policy.py` (websocket; client package `openpi-client`).
- **JAX LoRA recipe:** `paligemma_variant="gemma_2b_lora"`, `action_expert_variant="gemma_300m_lora"`, `freeze_filter=...get_freeze_filter()`, `ema_decay=None`. See the `pi0_libero_low_mem_finetune` config.
- **Model limits:**
  - `Pi0Config.action_dim=32`, `action_horizon=50` (`src/openpi/models/pi0_config.py`). **The 26-D action space fits** via padding.
  - `IMAGE_KEYS = ("base_0_rgb", "left_wrist_0_rgb", "right_wrist_0_rgb")` in `src/openpi/models/model.py` (JAX) and `preprocessing_pytorch.py`, so **the code exposes 3 camera slots**. In issue #620 a PI researcher said π₀.₅ was pretrained with up to 4 cameras, but users found only 3 keys in code. That thread is unresolved. https://github.com/Physical-Intelligence/openpi/issues/620
- Pretraining normalization-stat assets exist for `trossen`, `trossen_mobile`, `droid`, `franka`, `ur5e`. **There are none for a humanoid.** https://github.com/Physical-Intelligence/openpi/blob/main/docs/norm_stats.md
- **No Real-Time Chunking implementation** in `src/` (grep for RTC / `inference_delay` at `215abfb` found nothing).

### 1.6 Checkpoints, all at `gs://openpi-assets/checkpoints/...` (VERIFIED, README)
- **Base:** `pi0_base`, `pi0_fast_base`, `pi05_base`
- **Fine-tuned:** `pi0_fast_droid`, `pi0_droid`, `pi0_aloha_towel`, `pi0_aloha_tupperware`, `pi0_aloha_pen_uncap`, `pi05_libero`, `pi05_droid`
- They are cached in `~/.cache/openpi`. π₀.₅ supports only the flow-matching head in this repo.

### 1.7 Humanoid / Unitree G1 (VERIFIED absence as of 2026-09-14)
- No humanoid, G1 or dexterous-hand example in `examples/` (aloha_real, aloha_sim, droid, libero, simple_client, ur5).
- Issue #591, "Fine-Tuning Pi0-FAST on a Humanoid Robot" (Unitree G1), is open with no maintainer answer. https://github.com/Physical-Intelligence/openpi/issues/591
- `gh search repos` for "openpi g1", "openpi humanoid", "pi05 unitree", "pi05 brainco", "openpi dexterous hand" and "openpi unitree" returned **0 results**. Private or unindexed forks: UNVERIFIED.

---

## 2. LeRobot π₀.₅ (`policy.type=pi05`)

### 2.1 Same model, ported from openpi checkpoints (VERIFIED)
- LeRobot PR #1910, "Add OpenPi, Pi0 and Pi0.5", merged 2025-10-02: "The policies were ported from the Physical-Intelligence/openpi repo ... ensuring the forward pass and preprocessing pipeline between the LeRobot and OpenPI implementation match." It also says: "All base models were ported using the original conversion script (with config adjustments)", covering `lerobot/pi0_base`, `lerobot/pi05_base`, `pi0_libero` and `pi05_libero`. https://github.com/huggingface/lerobot/pull/1910
- LIBERO parity in the docs: LeRobot average 97.5% vs OpenPI reference 96.85%. https://huggingface.co/docs/lerobot/pi05
- `lerobot/pi05_base` has one file, `model.safetensors` at **14.47 GB** (F32). Its `config.json` stores `dtype: float32`, `device: mps`, `chunk_size: 50`, `max_action_dim: 32`, `num_inference_steps: 10`. I counted **4.14 B parameters** when instantiating the model locally. https://huggingface.co/lerobot/pi05_base

### 2.2 Training CLI and flags (VERIFIED: docs plus `src/lerobot/policies/pi05/configuration_pi05.py` at `89236ea`)
- Command: `lerobot-train --policy.type=pi05 --policy.pretrained_path=lerobot/pi05_base ...`
- `--policy.train_expert_only` (default `false`): "Freeze entire VLM, train only action expert and projections".
- `--policy.freeze_vision_encoder` (default `false`).
- `--policy.gradient_checkpointing`, `--policy.dtype=bfloat16` (default `float32`), `--policy.compile_model`.
- `--policy.use_relative_actions` + `--policy.relative_exclude_joints` (openpi-style delta actions), `--policy.normalization_mapping`, `--policy.empty_cameras`, `--policy.n_action_steps`.
- Defaults: `chunk_size=50`, `max_state_dim=32`, `max_action_dim=32`, `tokenizer_max_length=200`, lr `2.5e-5`, warmup 1000, cosine decay 30k.
- `--policy.pretrained_path` loads **weights only**, so settings such as `n_action_steps` reset to defaults. `--policy.path` also loads the config.
- **Camera count:** the model iterates over `config.image_features` and pads missing ones with empty cameras (`modeling_pi05.py` ~L1181-1239). **4 cameras (BrainCo layout) are accepted by the code.** Whether a 4th camera helps or hurts quality is UNVERIFIED.
- **PEFT / LoRA: supported** (VERIFIED).
  - Flags: `--peft.method_type=LORA --peft.r=... --peft.lora_alpha=...`, extra `lerobot[peft]` (`peft>=0.18.0,<1.0.0`). https://huggingface.co/docs/lerobot/peft_training
  - `PI05Policy._get_default_peft_targets()` targets **only** `gemma_expert.*.self_attn.(q|v)_proj` plus `state_proj|action_in_proj|action_out_proj|time_mlp_in|time_mlp_out`. The VLM gets no adapters by default; override with `--peft.target_modules`.
  - This method also exists at tag `v0.6.1`.
  - A community comment (openpi #711, 2026-03-30) reports poor LIBERO performance with default-target PEFT LoRA on pi05 (UNVERIFIED).

### 2.3 Quantile normalization (VERIFIED)
- The default `normalization_mapping` is `STATE: QUANTILES`, `ACTION: QUANTILES`, `VISUAL: IDENTITY`. `meta/stats.json` must contain `q01`/`q99`, otherwise training fails with `ValueError: QUANTILES normalization mode requires q01 and q99 stats`.
- Fix option 1: `lerobot-edit-dataset --repo_id X --new_repo_id X --operation.type recompute_stats --operation.overwrite true`.
- Fix option 2: pass `MEAN_STD`, which the LIBERO example does.
- A standalone script `src/lerobot/scripts/augment_dataset_quantile_stats.py` also exists. https://huggingface.co/docs/lerobot/pi05

### 2.4 Memory
- Docs (VERIFIED): the full fine-tune example with batch 64, gradient checkpointing and bf16 is "Sized for a single 80 GB GPU". The expert-only variant uses "less memory, at some cost in success rate".
- No official GB figure exists for expert-only or LoRA. My estimate: expert-only or LoRA with bs 32, bf16 and gradient checkpointing should fit in 48 GB, and 24-32 GB with smaller batches (**UNVERIFIED**). For reference, openpi JAX LoRA needs >22.5 GB.

### 2.5 Dataset format (VERIFIED)
- LeRobot main has `CODEBASE_VERSION = "v3.0"` (`src/lerobot/datasets/dataset_metadata.py`). v3.0 has been included since `lerobot >= 0.4.0`. https://huggingface.co/docs/lerobot/lerobot-dataset-v3

### 2.6 Apple Silicon (MPS / CPU)
- LeRobot officially supports `--policy.device=mps` (VERIFIED). https://huggingface.co/docs/lerobot/torch_accelerators
- **Measured on this machine (Apple M4 Pro, 24 GB unified memory), 2026-09-14:**
  - Setup: LeRobot main `89236ea`, torch 2.7.1, transformers 5.5.4, `PI05Pytorch` with **random weights** but the real architecture (4.14 B params) and LeRobot's mixed bf16/fp32 layout. 3 cameras at 224×224, 200 prompt tokens, 10 flow-matching steps, chunk 50, batch 1.
  - Result: **median 1.21 s per 50-action chunk** (min 1.20 s; first call 3.2 s), MPS driver memory **9.9 GB**. Script: `scratchpad/bench_pi05_mps.py`.
  - The compute cost matches real weights. Output quality was not tested, and pre/post-processing and tokenization are excluded.
  - Rough meaning: 50 actions cover 1.67 s at 30 Hz, so the Mac can produce chunks faster than real time. Closed-loop use needs async inference or RTC. It is OK for sim debugging, not ideal for a fast real-robot loop.
- **Pitfalls found:**
  - Building every parameter in bf16 crashed MPSGraph (`'mps.add' op requires the same element type`). It works only when the fp32/bf16 layout matches LeRobot's own loader.
  - fp32, the checkpoint's stored default, needs about 16.5 GB for weights alone, which pushes a 24 GB Mac into swap. Use `--policy.dtype=bfloat16`.
- CPU inference speed: not measured (UNVERIFIED; expected far slower).
- Reference point, third party (UNVERIFIED): SmolVLA via PyTorch-MPS took 204.58 ms per 50-action chunk on an M5 Pro. https://pypi.org/project/mlx-smolvla/0.1.1/

### 2.7 Real-Time Chunking: yes (VERIFIED)
- `rtc.mdx`: RTC works with Pi0, **Pi0.5** and SmolVLA.
  - `guided` mode works with ordinary flow-matching checkpoints.
  - `trained` mode needs a pi05 checkpoint trained with `policy.rtc_training_max_delay > 0`. That field exists on main but **not in tag v0.6.1**, where only `rtc_config` exists.
  - Deploy with `lerobot-rollout --inference.type=rtc`. https://huggingface.co/docs/lerobot/rtc
- The Unitree G1 docs page recommends inference-time RTC for pi05 deployment. https://huggingface.co/docs/lerobot/unitree_g1
- Async inference (policy server on the GPU box, robot client elsewhere) is supported for all policies. https://huggingface.co/docs/lerobot/async

### 2.8 Known risk: LeRobot-trained pi05 may act worse than openpi-trained (community, VERIFIED that it is reported, not that it is true)
- Issue #2559 (open since 2025-12-01): "Why does lerobot-pi05 have strong inference jitter, while openpi-pi05 has smooth inference?"
  - A commenter attributes it to openpi using delta actions while LeRobot used absolute actions.
  - A user on 2026-05-07 reports openpi still converges faster and less noisily than LeRobot **with** the relative-actions flag.
  - No fix as of 2026-06-29. https://github.com/huggingface/lerobot/issues/2559

### 2.9 LeRobot environment facts (VERIFIED, pyproject at `89236ea`)
- `requires-python >=3.12`, `transformers>=5.4.0,<5.6.0`, extra `lerobot[pi]`, license Apache-2.0.

---

## 3. Dataset format conversion (VERIFIED)
- **v2.1 → v3.0 (official):** `python -m lerobot.scripts.convert_dataset_v21_to_v30 --repo-id=<id>`, with `--root` and `--push-to-hub=false` for local data. Source: `src/lerobot/scripts/convert_dataset_v21_to_v30.py`, https://huggingface.co/docs/lerobot/lerobot-dataset-v3 , https://github.com/huggingface/lerobot/blob/main/docs/source/porting_datasets_v3.mdx
- **v3.0 → v2.1: no official converter** (grep of LeRobot main `89236ea` found none).
  - A community tool exists: `Tavish9/any4lerobot`, `ds_version_convert/v30_to_v21/convert_dataset_v30_to_v21.py --repo-id=... --root=...` (needs a v3.0 LeRobot install). Also offers v1.6→v2.0, v2.0→v2.1, v2.1→v2.0 and v2.1→v3.0. https://github.com/Tavish9/any4lerobot/tree/main/ds_version_convert
  - The unitree_lerobot README links this tool too. Reliability for 4-camera video datasets: UNVERIFIED.
- **Training with both stacks:** record in **v3.0** (native for LeRobot 0.6.x and unitree_lerobot).
  - LeRobot trains on it directly.
  - openpi needs a v2.1 copy, made with the community v3.0→v2.1 script or by writing the dataset directly with openpi's pinned LeRobot, as its `examples/*/convert_*_to_lerobot.py` scripts do.
  - Keep sim and real data as separate datasets so either path can re-export them.
  - LeRobot has `datasets/aggregate.py` and `multi_dataset.py` for merging or co-training; exact CLI usage for co-training: UNVERIFIED.

---

## 4. Licenses

| Artifact | License (primary source) | Notes |
|---|---|---|
| openpi **code** | Apache-2.0 (`LICENSE`, pyproject) — VERIFIED | https://github.com/Physical-Intelligence/openpi/blob/main/LICENSE |
| openpi **weights** (pi0/pi05 checkpoints) | **Conflicting signals, see below** | |
| LeRobot code | Apache-2.0 (pyproject) — VERIFIED | |
| HF `lerobot/pi05_base` (also `pi0_base`, `pi05_libero_base`) | Card metadata **`license: gemma`**, `gated: False` — VERIFIED | https://huggingface.co/lerobot/pi05_base |
| LeRobot docs page pi05 | Says "This model follows the **Apache 2.0 License**, consistent with the original OpenPI repository" — VERIFIED (contradicts model card) | https://huggingface.co/docs/lerobot/pi05 |
| `google/paligemma-3b-pt-224` | `license: gemma`, **`gated: manual`** — VERIFIED | https://huggingface.co/google/paligemma-3b-pt-224 |

**Timeline of the openpi weights license (VERIFIED):**
- 2025-02-05, issue #263 "Model License": PI's uzhilinsky answered "We are using the Apache 2.0 license for everything." https://github.com/Physical-Intelligence/openpi/issues/263
- 2025-09-30, commit `2a34154` "Added the Gemma license": `LICENSE_GEMMA.txt` (Gemma Terms of Use, "Last modified: February 21, 2024") was added to the repo. The README has no statement about the weights license. https://github.com/Physical-Intelligence/openpi/blob/main/LICENSE_GEMMA.txt

**Does the Gemma ToU apply?**
- Current text: https://ai.google.dev/gemma/terms, "Last modified: April 1, 2026".
- "Model Derivatives" means "modifications to Gemma, works based on Gemma, or any other machine learning model which is created by transfer of patterns of the weights, parameters, operations, or Output of Gemma".
- π₀/π₀.₅ are initialized from PaliGemma (Gemma-based), and both PI (via `LICENSE_GEMMA.txt`) and HF (via `license: gemma`) now label them this way. **The conservative reading is that the checkpoints and your fine-tunes are Gemma Model Derivatives governed by the Gemma ToU.** This is an interpretation, not legal advice.

**Commercial use (SAP demo):**
- The Gemma ToU sections read (2.2 Use, 3.1 Distribution, 3.2 Use restrictions, 3.3 Outputs, 4.5 Termination) contain **no non-commercial restriction** (VERIFIED for those sections).
- Use is limited only by the Gemma Prohibited Use Policy and applicable law. The policy covers harmful or illegal content, deception, unlicensed professional advice, and similar; it does not mention commercial use. https://ai.google.dev/gemma/prohibited_use_policy
- Secondary source stating PaliGemma's Gemma license "permits commercial use subject to the Prohibited Use Policy": https://playground.roboflow.com/models/google/paligemma (UNVERIFIED as authoritative).
- **Obligations if you share weights:** Gemma ToU 1.1(b) defines "Distribution" to include making the model "available as a hosted service via API, web access". If you hand fine-tuned weights to SAP or expose the policy as a service, 3.1 applies:
  - pass on the use restrictions
  - include a copy of the agreement
  - mark modified files
  - include the notice "Gemma is provided under and subject to the Gemma Terms of Use found at ai.google.dev/gemma/terms"
- **Recommendation:** treat the demo as allowed, but have SAP legal confirm before any external distribution.

**Is PaliGemma license acceptance on HF required?**
- **LeRobot path: yes.**
  - The pi05 weights repo itself is not gated.
  - The pi05 processor loads its tokenizer with `AutoTokenizer.from_pretrained("google/paligemma-3b-pt-224")` (`PI05Config.text_tokenizer_name`; `processor/tokenizer_processor.py`), and that repo is gated (manual).
  - The docs say "accept its license on the Hub and log in with `hf auth login` before training".
  - No HF token file exists on this Mac yet (checked `~/.cache/huggingface/token`).
- **openpi path: no HF acceptance.** The tokenizer downloads anonymously from `gs://big_vision/paligemma_tokenizer.model` (`src/openpi/models/tokenizer.py`), and checkpoints come from public `gs://openpi-assets`. The Gemma ToU still applies by its own terms.

**Other licenses:**
- unitree_lerobot: Apache-2.0 (pyproject).
- GR00T and SmolVLA: see section 6.

---

## 5. unitree_lerobot (VERIFIED at `41c2805`, 2026-05-25)

- **LeRobot base:** git submodule `unitree_lerobot/lerobot` → `a5b29d4` (2025-11-10, LeRobot pyproject `0.4.1`, `CODEBASE_VERSION = "v3.0"`).
  - The README table still says "commit version number is `0878c68`" (2025-08-08), which is stale; the submodule pointer is authoritative.
  - Release note v0.3: "Update lerobot dataset v3.0" and "More policy support (pi05, groot)".
  - Package `requires-python >=3.10,<3.11`. The team's LeRobot 0.6.x is a newer version but reads the same v3.0 format. Checkpoint compatibility between LeRobot 0.4.1 and 0.6.x: UNVERIFIED.
  - https://github.com/unitreerobotics/unitree_lerobot
- **Policies documented:** ACT, Diffusion, Pi0, **Pi05** (`--policy.pretrained_path=lerobot/pi05_base`), GR00T (`--policy.type=groot`). In LeRobot 0.4.1 this is GR00T N1.5, since the LeRobot docs say N1.5 support was removed later and to pin `lerobot==0.5.1` to keep it (inference).
- **BrainCo layout: supported.**
  - `robot_type Unitree_G1_Brainco` → `G1_BRAINCO_CONFIG` in `unitree_lerobot/utils/constants.py`.
  - 26 motors: left arm 7 (`kLeftShoulderPitch` … `kLeftWristYaw`), right arm 7, left hand 6 (`kLeftHandThumb`, `kLeftHandThumbAux`, `Index`, `Middle`, `Ring`, `Pinky`), right hand 6.
  - **4 cameras:** `cam_left_high`, `cam_right_high`, `cam_left_wrist`, `cam_right_wrist`.
  - Real-robot eval: `eval_robot/eval_g1.py --arm=G1_29 --ee=brainco` (`robot_control/robot_hand_brainco.py`).
- **Sim eval path: Isaac Lab only.**
  - `eval_robot/eval_g1_sim.py` uses DDS sim-state/reward subscribers and `IsaacSimCamera` shared memory from `unitree_sim_isaaclab`.
  - **No MuJoCo path.** A maintainer in issue #40 (2025-11-07) said it would "theoretically" work if the sim exchanges the same DDS messages. https://github.com/unitreerobotics/unitree_lerobot/issues/40
  - Issue #69 (pi05 eval on a G1 with this code) is open with action/camera mapping trouble. https://github.com/unitreerobotics/unitree_lerobot/issues/69
- **Related: LeRobot's own G1 integration.**
  - `src/lerobot/robots/unitree_g1` plus HF `lerobot/unitree-g1-mujoco` ship `g1_29dof_with_hand.xml` and `g1_body29_hand14.urdf`. The hand link names (`thumb_0..2`, `index_0..1`, `middle_0..1`) are Dex3-style 7-DoF hands.
  - There is **no BrainCo string** in LeRobot's G1 robot code. The docs example trains `pi05` from `lerobot/pi05_base` for 3000 steps at bs 32 and deploys with RTC. https://huggingface.co/docs/lerobot/unitree_g1

---

## 6. Alternatives (open-weight), one line each

| Model | G1 / dex-hand relevance | License | Notes |
|---|---|---|---|
| **NVIDIA Isaac GR00T N1.7** (`nvidia/GR00T-N1.7-3B`) | Pretrain tag `REAL_G1` ("Real-world Unitree G1, relative EEF + relative joints"). Per `statistics.json`: state left/right_hand **7-D each** (Dex3-like, inferred), arms 7, waist 3, EEF 9-D, single `ego_view` camera. Posttrain tag `UNITREE_G1_SONIC` (whole-body latents). **No BrainCo**; custom robots use `NEW_EMBODIMENT`. | Repo code Apache-2.0; README says "fully commercially licensable under Apache 2.0"; **HF model card: NVIDIA Open Model License**, "ready for commercial/non-commercial use". Backbone `nvidia/Cosmos-Reason2-2B` is gated (auto), NVIDIA Open Model License. | Inference 16 GB+, fine-tune 40 GB+ recommended; also available in LeRobot as `policy.type=groot` (N1.7 only). https://github.com/NVIDIA/Isaac-GR00T , https://huggingface.co/nvidia/GR00T-N1.7-3B |
| **SmolVLA** (`lerobot/smolvla_base`) | Generic, no G1 data | HF card has **no license field**; backbone `SmolVLM2-500M-Video-Instruct` is Apache-2.0; LeRobot code Apache-2.0 | ~0.5B-class, runs well on a Mac (third-party MPS figure ~205 ms per chunk on M5 Pro); weaker than π₀.₅ for dexterous bimanual work (UNVERIFIED claim). https://huggingface.co/lerobot/smolvla_base |
| **Unitree UnifoLM-VLA-0** (`unitreerobotics/unifolm-vla`, released 2026-01-29) | Trained on Unitree open G1 datasets (Dex1 gripper collection); installs `lerobot@0878c68` (v2.1 era), CUDA 12.4 | GitHub license field **null**; HF `UnifoLM-VLA-Base` has no license metadata (secondary sources claim Apache-2.0: UNVERIFIED) | https://github.com/unitreerobotics/unifolm-vla |
| **Unitree UnifoLM-WLA-1.0** (`unitreerobotics/unifolm-wla`) | 6B, ~2,500 h real-robot data, "supports two-finger grippers and multiple five-finger dexterous hands" on G1 | Released parts `UnifoLM-ER-1` / `UnifoLM-ER-Flow` are Apache-2.0 (HF, 2026-09-11) | **WLA-Base weights and post-train code not released yet** (open-source plan unchecked as of 2026-09-13); ER models are embodied-reasoning VLMs, not a drop-in policy. https://github.com/unitreerobotics/unifolm-wla |

---

## 7. Cost and time for a π₀.₅ fine-tune on ~2000 episodes

**Workload:** 2000 × 10 s × 30 Hz = **600,000 frames**, 2-3 cameras at 224×224. At bs 32, one epoch is about 18.8k steps; a typical 30k-step run is about 1.6 epochs.

**Throughput anchors (VERIFIED sources):**
- π₀ in LeRobot, **1×H100, bs 32, 30k steps ≈ 11 h** (≈1.3 s/step, ≈24 samples/s). arXiv 2510.17369. https://arxiv.org/html/2510.17369
- openpi JAX DROID: 8×H100, bs 256, 100k iterations ≈ 2 days (≈18-19 samples/s per GPU, full fine-tune with EMA). https://github.com/Physical-Intelligence/openpi/blob/main/examples/droid/README_train.md

**Estimates (UNVERIFIED, derived from the anchors):**
- **Full fine-tune, 1×H100 80GB, bs 32, 30k steps:** about 10-13 h.
- **Expert-only (`train_expert_only=true`, `freeze_vision_encoder=true`) or PEFT-LoRA:** there is no backward pass through the 2B VLM, but the forward pass still runs, so roughly 1.5-2× faster. That gives about **5-8 h on an H100**, or about 10-16 h on an L40S / RTX 6000 Ada (48 GB).
- Unitree G1 doc example: only 3000 steps at bs 32, well under 2 h on an H100.

**Prices, on-demand per GPU-hour, read 2026-09-14:**

| GPU (VRAM) | RunPod Community / Secure | Lambda (1×) | vast.ai live API (verified hosts, 1 GPU: min / median, n) |
|---|---|---|---|
| RTX 4090 (24 GB) | $0.34 / $0.74 | — | $0.30 / $0.57 (22) |
| RTX 5090 (32 GB) | $0.69 / $0.99 | — | $0.44 / $0.87 (41) |
| L40S (48 GB) | $0.79 / $1.09 | — | $0.80 (1) |
| RTX 6000 Ada (48 GB) | $0.74 / $0.84 | A6000 48 GB: $1.09 | $0.57 / $0.77 (4) |
| RTX PRO 6000 (96 GB) | $1.69 / $2.09 | — | $1.20 / $1.60 (16) |
| A100 80 GB | $1.19 (PCIe) / $1.59 | A100 40 GB only: $1.99 | A100 PCIe 80 GB: $0.94 / $0.97 (2); SXM4 offers are 40 GB only |
| H100 80 GB | $1.99 (PCIe) / $2.89 | $3.29 (PCIe), $4.29 (SXM) | $1.87 / $4.17 (6) |
| H200 (141 GB) | — / $4.59 | — | $3.98 / $4.49 (2) |

Sources: https://www.runpod.io/pricing , https://lambda.ai/pricing , vast.ai `console.vast.ai/api/v0/bundles` queried 2026-09-14T16:54Z (the vast.ai pricing page itself shows no static numbers).

**Recommendation:**
- **1×H100 80GB on RunPod Community** ($1.99/h). Expert-only or LoRA run: about 5-8 h, **≈ $10-16**. Full fine-tune: about 10-13 h, **≈ $20-26**.
- **RTX PRO 6000 96 GB** ($1.69/h) gives full-fine-tune headroom at lower cost, but is slower than an H100 (UNVERIFIED ratio).
- **Cheapest viable expert-only option:** L40S / RTX 6000 Ada (48 GB) at ~$0.74-0.79/h, about 10-16 h, **≈ $8-13**.
- **Total budget:** plan 3-5 runs (sim-only baseline, sim + real co-train, ablations such as relative actions or cameras), plus norm stats and eval. That is about **$60-150** and 1-3 GPU-days wall time.
- **Disk:** 14.5 GB base weights plus the video dataset (size UNVERIFIED) plus checkpoints of about 8-15 GB each. Rent ≥200 GB of volume.

---

## Decision

**Recommendation: (a) LeRobot π₀.₅ (PyTorch, dataset v3.0), keeping openpi as a validated fallback.**

**Why (a):**
1. **One toolchain.** The sim (`lerobot/unitree-g1-mujoco`), LeRobot 0.6.x, dataset v3.0, and Unitree's own `unitree_lerobot` (pi05 + BrainCo layout + v3.0) all already use LeRobot. openpi pins LeRobot at a 2025-05-28 commit that **cannot read v3.0 datasets**, is **Ubuntu-22.04-only**, and needs a patched `transformers` for PyTorch.
2. **Same weights.** LeRobot ported openpi's checkpoints with the original conversion script and shows LIBERO parity.
3. **Features the project needs are already in LeRobot:** PEFT/LoRA and `train_expert_only` in PyTorch, quantile stats tooling, **RTC** plus async inference for deployment, and **4-camera** input.
   - openpi PyTorch has no LoRA, EMA or mixed precision, and its code exposes 3 camera slots.
4. **Works on the M4 for development.** Measured 1.2 s per chunk in bf16 on MPS (9.9 GB), so sim-loop debugging is possible locally; training goes to the rented GPU.

**Main risk and mitigation:**
- LeRobot issue #2559 (open) reports jitter and slower convergence for pi05 trained in LeRobot compared with openpi on the same data, suspected to come from absolute vs. delta actions.
- Mitigations:
  - Train with `--policy.use_relative_actions=true` and hand joints in `relative_exclude_joints` (analogous to openpi keeping grippers absolute).
  - Use RTC at deployment.
  - Run an early A/B test on a few hundred sim episodes. If LeRobot is clearly worse, export a v2.1 copy (community `any4lerobot` converter) and train the same data with openpi JAX: a `TrainConfig` plus G1 `Inputs`/`Outputs` transforms, LoRA on a 24-48 GB GPU, run on Ubuntu/Docker.

**Before training, in any case:**
- Accept the PaliGemma license on HF and log in.
- Recompute q01/q99 stats.
- Treat the weights as Gemma-ToU derivatives for the SAP business case: commercial demo use is permitted, but distribution or hosting brings notice obligations. Have legal confirm.
