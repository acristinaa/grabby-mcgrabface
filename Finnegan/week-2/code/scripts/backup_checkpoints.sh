#!/usr/bin/env bash
# Level 1 Step 7 - back up SmolVLA training checkpoints to the T7, one verified .tar per checkpoint.
#   scripts/backup_checkpoints.sh run1            back up every finished checkpoint of run1 that is not on the T7 yet, then exit
#   scripts/backup_checkpoints.sh run1 --watch    the same every 10 min until lerobot-train for this run has ended (and once after)
# Training and loading stay on the internal disk: the exFAT T7 gets macOS "._" twins of every file, which crash lerobot, and an
# unplugged T7 would crash a run that writes to it. One .tar per checkpoint is safe there. A checkpoint is taken only when it is at
# least 60 s old (lerobot has finished writing it). Each archive is written as .part, its sha256 is computed while writing and again
# from the T7, and only matching archives are renamed to checkpoint_<step>.tar (the .sha256 file sits next to it).
set -euo pipefail

run="${1:?usage: backup_checkpoints.sh <run name> [--watch]}"
SRC="$HOME/dev/g1-ext/smolvla-runs/$run/checkpoints"
DST="/Volumes/T7/Robot Project SAP/DATA Collection/smolvla-runs/$run"
MIN_AGE_S=60

backup_once() {
  if [ ! -d /Volumes/T7 ]; then echo "$(date +%H:%M) T7 not mounted - skipped"; return 0; fi
  mkdir -p "$DST"
  for dir in "$SRC"/[0-9]*; do
    [ -d "$dir" ] || continue
    step=$(basename "$dir")
    out="$DST/checkpoint_$step.tar"
    [ -f "$out" ] && continue
    age=$(( $(date +%s) - $(stat -f %m "$dir") ))
    if [ "$age" -lt "$MIN_AGE_S" ]; then echo "$(date +%H:%M) $step is ${age}s old - next round"; continue; fi
    written=$(cd "$SRC" && COPYFILE_DISABLE=1 tar -cf - "$step" | tee "$out.part" | shasum -a 256 | cut -d' ' -f1)
    readback=$(shasum -a 256 "$out.part" | cut -d' ' -f1)
    if [ "$written" = "$readback" ]; then
      mv "$out.part" "$out"
      echo "$written  checkpoint_$step.tar" > "$out.sha256"
      echo "$(date +%H:%M) $step -> $out ($(du -h "$out" | cut -f1), sha256 ok)"
    else
      rm -f "$out.part"
      echo "$(date +%H:%M) $step: sha256 mismatch ($written vs $readback) - removed, retry next round" >&2
    fi
  done
}

backup_once
if [ "${2:-}" = "--watch" ]; then
  while pgrep -f "lerobot-train.*smolvla-runs/$run" > /dev/null; do
    sleep 600
    backup_once
  done
  backup_once
  echo "$(date +%H:%M) training for $run has ended - watch done"
fi
