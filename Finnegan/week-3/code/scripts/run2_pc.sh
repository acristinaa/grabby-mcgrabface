#!/usr/bin/env bash
# Level 1 v2 on the Ubuntu PC (ubucode, 2026-11-03): record 800 demos -> convert -> validate -> SmolVLA run2 on the GPU.
#   nohup scripts/run2_pc.sh > runs/l1v2_pc_chain.log 2>&1 &
# Same code and seeds as the Mac's v2 (T7 copy stays as backup); demos are not bit-identical across machines, only equivalent.
# 1,600 demos (user decision 2026-11-03). Short recorder processes: a 69 process grows ~1.8 -> 3-4 GB RAM over ~40 demos (leak, not
# yet found; on the 15 GB PC 6 froze the machine at 17:13 and 3-4 long ones kept tripping the RAM watchdog), so every process records
# BATCH demos on a fresh seed and exits, PARALLEL at a time, until TOTAL demos exist. Demos already in the folder (also partial seeds
# from earlier starts, 109-116) count; running recorders from an earlier start are waited for. Seeds start at FIRST_SEED (0/1 = eval).
# Automatic suspend on mains power is off on this PC (gsettings sleep-inactive-ac-type 'nothing', checked 2026-11-03); keep it off.
# Watch: Lab View on the PC (scripts/65_labview.py, 127.0.0.1:8030), from the Mac through an SSH tunnel.
set -uo pipefail
cd "$(dirname "$0")/.."
export MUJOCO_GL=egl
PY="$HOME/dev/g1-ext/.venv/bin/python"
SV="$HOME/dev/g1-ext/smolvla-test/.venv/bin/python"
DEMOS="$HOME/g1-demos"
SRC="$DEMOS/level1_walkup_v2"
ROOT="$HOME/datasets/g1_volleyball_walkup_v2"
TOTAL=1600
BATCH=40
PARALLEL=3
FIRST_SEED=117   # 109-116 hold the partial seeds of the earlier starts
say() { echo "$(date '+%m-%d %H:%M') $*"; }
count() { ls "$SRC" 2>/dev/null | grep -c '^episode.*npz$'; }
seed_used() { ls "$SRC" 2>/dev/null | grep -q "^episode_s$1_"; }
wait_recorders() { while pgrep -f "[6]9_volleyball_demo_factory" >/dev/null; do sleep 30; done; }

wait_recorders
seed=$FIRST_SEED
idle=0
while [ "$(count)" -lt "$TOTAL" ]; do
    before=$(count)
    say "have $before / $TOTAL demos"
    for ((k = 0; k < PARALLEL; k++)); do
        while seed_used "$seed"; do seed=$((seed + 1)); done
        say "record seed $seed x $BATCH"
        "$PY" scripts/69_volleyball_demo_factory.py --demos "$BATCH" --seed "$seed" --name level1_walkup_v2 \
            --out "$DEMOS" > "runs/l1v2_pc_rec_${seed}.log" 2>&1 &
        seed=$((seed + 1))
    done
    wait
    if [ "$(count)" -le "$before" ]; then idle=$((idle + 1)); else idle=0; fi
    [ "$idle" -ge 2 ] && { say "TWO ROUNDS WITHOUT NEW DEMOS - stop (see runs/l1v2_pc_rec_*.log)"; exit 1; }
done
n=$(ls "$SRC" | grep -c '^episode.*npz$')
say "recorded $n demos"

say "convert -> runs/l1v2_pc_convert.log"
"$SV" scripts/79_convert_smolvla.py "$SRC" --root "$ROOT" --overwrite > runs/l1v2_pc_convert.log 2>&1 || { say "CONVERT FAILED"; exit 1; }
say "validate -> runs/l1v2_pc_validate.log"
"$SV" scripts/80_validate_dataset.py "$SRC" --root "$ROOT" > runs/l1v2_pc_validate.log 2>&1 || true
grep -q "^0 problems" runs/l1v2_pc_validate.log || { say "VALIDATE FAILED - no training"; exit 1; }
say "validation: 0 problems - training run2 (~40 h) -> runs/l1v2_train_run2.log"

DEVICE=cuda V2_ROOT="$ROOT" HF_HUB_CACHE="$HOME/.cache/huggingface/hub" HF_DATASETS_CACHE="$HOME/.cache/huggingface/datasets" \
    scripts/train_smolvla.sh run2 > runs/l1v2_train_run2.log 2>&1
say "training exit $?"
