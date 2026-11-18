#!/bin/bash
# Downloads the SmolVLA run3 150k model from this repo's release and installs it where the robot code expects it.
# Run:  bash Finnegan/sap-day-model/get_model.sh        Safe to re-run. Needs internet (907 MB) and about 2 GB free disk.
set -u
URL="https://github.com/acristinaa/grabby-mcgrabface/releases/download/sap-day-run3-model"
TAR="smolvla_run3_150000.tar"
DEST="${G1_PREFIX:-$HOME/g1-robot-day}/smolvla-runs/run3/checkpoints/150000"
MODEL_SHA="ef86fffee140e065e0e19625e125af3dc1c1a9d4d0ae1384983c90d5221db931"
TMP="${TMPDIR:-/tmp}/g1-model-download"
stop() { echo "STOP: $1"; exit 1; }

if [ -f "$DEST/pretrained_model/model.safetensors" ] && \
   [ "$(shasum -a 256 "$DEST/pretrained_model/model.safetensors" | cut -d' ' -f1)" = "$MODEL_SHA" ]; then
  echo "MODEL: OK already installed in $DEST/pretrained_model"; exit 0
fi
mkdir -p "$TMP" && cd "$TMP" || stop "cannot create $TMP"
echo "downloading $TAR (907 MB) ..."
curl -fL --retry 5 -C - -o "$TAR" "$URL/$TAR" || stop "download failed (internet?) - run this script again, it resumes"
curl -fsSL --retry 5 -o "$TAR.sha256" "$URL/$TAR.sha256" || stop "checksum download failed"
shasum -a 256 -c "$TAR.sha256" || { rm -f "$TAR"; stop "download damaged - run this script again"; }
rm -rf run3_150000 && tar -xf "$TAR" || stop "unpacking failed (disk full?)"
mkdir -p "$DEST" && rm -rf "$DEST/pretrained_model" && mv run3_150000/pretrained_model "$DEST/" || stop "cannot write $DEST"
(cd "$DEST/pretrained_model" && shasum -a 256 -c model.safetensors.sha256) || stop "model checksum wrong"
rm -rf "$TMP"
echo "MODEL: OK installed in $DEST/pretrained_model"
