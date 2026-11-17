#!/usr/bin/env bash
# One command for the v2 training (Level 1): convert the 800 v2 demos -> validate -> start SmolVLA run2 (detached).
#   scripts/run2_start.sh            T7 plugged in, Mac on power, lid open; returns after validation, training keeps running
# Steps: mount the APFS image on the T7 (/Volumes/G1DATA, LeRobot crashes on exFAT "._" files), 79 --overwrite (~20 min),
# 80 must end with "0 problems" (else nothing is trained), then scripts/train_smolvla.sh run2 under nohup + caffeinate (~21 h).
# Watch: Lab View http://127.0.0.1:8030/#training (scripts/65_labview.py) or tail runs/l1v2_train_run2.log.
set -euo pipefail
cd "$(dirname "$0")/.."

SV="$HOME/dev/g1-ext/smolvla-test/.venv/bin/python"
T7="/Volumes/T7/Robot Project SAP"
SRC="$T7/DATA Collection/g1-demos/level1_walkup_v2"
IMAGE="$T7/g1data-apfs.sparsebundle"
ROOT=/Volumes/G1DATA/datasets/g1_volleyball_walkup_v2

[ -d "$SRC" ] || { echo "T7 not found at $T7 - plug it in (directly, not via a hub)"; exit 1; }
[ -d /Volumes/G1DATA ] || hdiutil attach "$IMAGE" >/dev/null
mkdir -p /Volumes/G1DATA/datasets /Volumes/G1DATA/hf-cache
echo "$(date +%H:%M) demos: $(ls "$SRC" | grep -c '^episode.*npz$') (expected 800)"

echo "$(date +%H:%M) convert (~20 min) -> runs/l1v2_convert.log"
caffeinate -i "$SV" scripts/79_convert_smolvla.py "$SRC" --root "$ROOT" --overwrite > runs/l1v2_convert.log 2>&1

echo "$(date +%H:%M) validate -> runs/l1v2_validate.log"
caffeinate -i "$SV" scripts/80_validate_dataset.py "$SRC" --root "$ROOT" > runs/l1v2_validate.log 2>&1 || true
if ! grep -q "^0 problems" runs/l1v2_validate.log; then
    echo "VALIDATION FAILED - not training. See runs/l1v2_validate.log"; exit 1
fi
echo "$(date +%H:%M) validation: 0 problems"

nohup caffeinate -i scripts/train_smolvla.sh run2 > runs/l1v2_train_run2.log 2>&1 &
disown
echo "$(date +%H:%M) training run2 started (~21 h). Keep the Mac on power, lid open, T7 plugged in."
echo "Watch: http://127.0.0.1:8030/#training  or  tail -f runs/l1v2_train_run2.log"
