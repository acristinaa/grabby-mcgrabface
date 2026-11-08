#!/usr/bin/env bash
# Level 1 Step 7 (plan Stage 8) - fine-tune SmolVLA (lerobot/smolvla_base) on the volleyball dataset, on this Mac (MPS).
#   scripts/train_smolvla.sh smoke          200 steps, checkpoint at 200: proves dataset -> training -> checkpoint
#   scripts/train_smolvla.sh run1           the real run: one schedule for 45k steps (warm-up 1k, decay over 45k), checkpoint every
#                                           5k; stop it any time (Ctrl+C / kill) and continue with:
#   scripts/train_smolvla.sh resume run1    continues from the last checkpoint with the same schedule
#   scripts/train_smolvla.sh run2           v2 dataset (800 demos, ~580k frames) from smolvla_base: 90k steps ≈ 1.2 epochs (run1 plateaued
#                                           at ~1.3 epochs), checkpoint every 10k, ~21 h on MPS. Dataset and the datasets Arrow cache live
#                                           in the APFS image on the T7 (/Volumes/G1DATA): the internal disk is too small for 800 demos.
# One 45k schedule instead of "20k tonight + 25k later": a 20k run decays the learning rate to its minimum by 20k (lerobot scales the
# schedule to --steps), so a second run would barely learn. ~0.84 s/step at batch 8 (measured 2026-10-07) -> 45k ≈ 10.5 h in two sittings.
# Dataset: ~/dev/g1-ext/datasets/g1_volleyball_walkup_v1 (internal disk; never on the exFAT T7). Runs: ~/dev/g1-ext/smolvla-runs/<name>.
# Log: pipe stdout into runs/l1w_s7_<name>.log.
# Other machines (the Ubuntu PC, 2026-11-03): DEVICE=cuda V2_ROOT=<dataset> HF_HUB_CACHE=<hub cache> HF_DATASETS_CACHE=<cache> override the Mac
# defaults below (GTX 1060 6 GB measured: 1.62 s/step at batch 8, 3.6 GB GPU memory).
set -euo pipefail

EXT="$HOME/dev/g1-ext"
BIN="$EXT/smolvla-test/.venv/bin"
DATASET="$EXT/datasets/g1_volleyball_walkup_v1"
REPO_ID=local/g1_volleyball_walkup_v1
OUT="$EXT/smolvla-runs"
export HF_HUB_CACHE="${HF_HUB_CACHE:-$EXT/smolvla-test/hf_cache/hub}"   # smolvla_base + SmolVLM2 are cached here
export HF_HUB_OFFLINE=1                                  # local dataset, cached model: never ask the Hub
mkdir -p "$OUT"

mode="${1:?usage: train_smolvla.sh smoke | run1 | resume <name>}"
case "$mode" in
  smoke) name=smoke; steps=200; save=200; log=10; decay=200 ;;
  run1) name=run1; steps=45000; save=5000; log=50; decay=45000 ;;
  run2) name=run2; steps=90000; save=10000; log=50; decay=90000
        DATASET="${V2_ROOT:-/Volumes/G1DATA/datasets/g1_volleyball_walkup_v2}"; REPO_ID=local/g1_volleyball_walkup_v2
        export HF_DATASETS_CACHE="${HF_DATASETS_CACHE:-/Volumes/G1DATA/hf-cache}" ;;
  resume)
    name="${2:?resume needs the run name}"
    exec "$BIN/lerobot-train" --config_path="$OUT/$name/checkpoints/last/pretrained_model/train_config.json" --resume=true ;;
  *) echo "unknown mode $mode" >&2; exit 2 ;;
esac

exec "$BIN/lerobot-train" \
  --policy.path=lerobot/smolvla_base \
  --policy.device="${DEVICE:-mps}" \
  --policy.push_to_hub=false \
  --policy.scheduler_decay_steps="$decay" \
  --dataset.repo_id="$REPO_ID" \
  --dataset.root="$DATASET" \
  --batch_size=8 \
  --steps="$steps" \
  --save_freq="$save" \
  --log_freq="$log" \
  --seed=1000 \
  --wandb.enable=false \
  --job_name="g1_volleyball_$name" \
  --output_dir="$OUT/$name"
