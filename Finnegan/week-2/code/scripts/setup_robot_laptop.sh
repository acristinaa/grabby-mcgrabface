#!/usr/bin/env bash
# Robot-day laptop setup (macOS or Ubuntu x86_64): the environments 84/86/89 and the policy server 81 need, plus the model.
# Why: the operators set up their own laptops without Finnegan; every block prints one line "SETUP <block>: OK" or
# "SETUP <block>: FAILED <what to do>", can be re-run safely, and nothing here talks to the robot.
#   bash scripts/setup_robot_laptop.sh --robot-env --policy-env --release robot-day-2026-10-18     # everything
#   bash scripts/setup_robot_laptop.sh --release-file ~/Downloads/smolvla_run1_040000.tar          # model from a file
#   bash scripts/setup_robot_laptop.sh --show-ip                                                   # network adapters
# Blocks:
#   --robot-env    Python 3.10 venv (cyclonedds 0.10.2 has wheels only up to 3.10, PLAN.md 2.7) -> <prefix>/robot-env.sh
#   --policy-env   Python 3.12 venv with lerobot 0.6.1 + torch 2.11.0 and the 2.7 GB SmolVLA backbone -> <prefix>/policy-env.sh
#                  (download about 3 GB on a Mac, about 6 GB on Ubuntu with the CUDA torch; 20-60 min on home Wi-Fi)
#   --release TAG  download the model tar from the GitHub release (needs `gh auth login`), check its sha256, unpack
#   --release-file PATH   the same check + unpack for a tar already on disk (PATH.sha256 next to it)
#   --prefix DIR   where everything goes (default ~/g1-robot-day)
set -uo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
PREFIX="$HOME/g1-robot-day"
GH_REPO="finneganadam-maker/UnitreeG1-Robot"
TAR_NAME="smolvla_run1_040000.tar"                                     # packed by Finnegan, ~/dev/g1-ext/release
CKPT_DIR_REL="smolvla-runs/run1/checkpoints/040000"                    # 89 and the runbook read the model from here
CKPT_FILES="config.json model.safetensors policy_preprocessor.json policy_postprocessor.json"   # policy_wire.CHECKPOINT_FILES
SDK_URL="https://github.com/unitreerobotics/unitree_sdk2_python/archive/65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5.tar.gz"  # PLAN.md 2.7 pin
ROBOT_PINS="cyclonedds==0.10.2 numpy==2.2.6 opencv-python-headless==4.13.0.92"                   # PLAN.md 2.7
POLICY_PINS="lerobot[smolvla]==0.6.1 torch==2.11.0 torchvision==0.26.0 numpy==2.2.6"            # the SmolVLA env that trained run1
BACKBONES="lerobot/smolvla_base@5e8d12a6e2975b0e5e5fce7c8caf47c371d257b6 HuggingFaceTB/SmolVLM2-500M-Video-Instruct@7b375e1b73b11138ff12fe22c8f2822d8fe03467"  # revisions in Finnegan's cache
# Files 81 never reads: SmolVLM2's onnx/ folder alone is 5.5 GB of the 7.4 GB repo (review E1-1); Finnegan's 2.7 GB cache has none of them
BACKBONE_SKIP='["onnx/*", "*.gif", "*.ipynb"]'

DO_ROBOT=0; DO_POLICY=0; DO_SHOW_IP=0; RELEASE_TAG=""; RELEASE_FILE=""
FAILED=0

usage() { sed -n '2,14p' "$0"; }
ok() { echo "SETUP $1: OK${2:+ $2}"; }   # ${2:+ $2}: " <detail>" only when a detail was given
failed() { echo "SETUP $1: FAILED $2"; FAILED=1; }

while [ $# -gt 0 ]; do
  case "$1" in
    --robot-env) DO_ROBOT=1 ;;
    --policy-env) DO_POLICY=1 ;;
    --release) RELEASE_TAG="${2:?--release needs a tag, e.g. robot-day-2026-10-18}"; shift ;;
    --release-file) RELEASE_FILE="${2:?--release-file needs the path of $TAR_NAME}"; shift ;;
    --prefix) PREFIX="${2:?--prefix needs a directory}"; shift ;;
    --show-ip) DO_SHOW_IP=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option $1"; usage; exit 2 ;;
  esac
  shift
done
if [ $((DO_ROBOT + DO_POLICY + DO_SHOW_IP)) -eq 0 ] && [ -z "$RELEASE_TAG$RELEASE_FILE" ]; then usage; exit 2; fi

# --- platform: macOS (arm64 or x86_64) or Linux x86_64 (no cyclonedds 0.10.2 wheel for Linux on ARM, PLAN.md 1) --------------
OS="$(uname -s)"; ARCH="$(uname -m)"
if [ "$OS" = "Linux" ] && [ "$ARCH" != "x86_64" ]; then
  failed platform "Linux on $ARCH is not supported (no cyclonedds wheel) - use a macOS or Ubuntu x86_64 laptop"; exit 1
fi
if [ "$OS" != "Linux" ] && [ "$OS" != "Darwin" ]; then
  failed platform "$OS is not supported - use macOS or Ubuntu x86_64"; exit 1
fi

need_uv() {
  command -v uv >/dev/null 2>&1 && return 0
  failed "$1" "uv not found - install it (runbook Part A step 1: curl -LsSf https://astral.sh/uv/install.sh | sh), open a new terminal, re-run"
  return 1
}

# Create (or reuse) a venv at $2 with Python $1; uv downloads its own Python, so no system Python is needed (review r5-5).
make_venv() {
  uv python install "$1" && uv venv --allow-existing --python "$1" "$2"
}

robot_env() {
  local venv="$PREFIX/robot-env"
  need_uv robot-env || return
  make_venv 3.10 "$venv" || { failed robot-env "could not create the Python 3.10 venv in $venv (internet? disk space?)"; return; }
  uv pip install --python "$venv/bin/python" --only-binary cyclonedds $ROBOT_PINS \
    || { failed robot-env "pip install of $ROBOT_PINS failed (internet?)"; return; }
  # --no-deps: the SDK would pull opencv-python next to the pinned headless build
  uv pip install --python "$venv/bin/python" --no-deps "unitree_sdk2py @ $SDK_URL" \
    || { failed robot-env "unitree_sdk2py download failed (github.com reachable?)"; return; }
  "$venv/bin/python" -c "import cyclonedds, cv2, numpy, unitree_sdk2py.core.channel" \
    || { failed robot-env "the robot env does not import cleanly - send runs/ and this output to Finnegan"; return; }
  cat > "$PREFIX/robot-env.sh" <<EOF
# source this before 84, 86, 89 and the loopback helper (written by scripts/setup_robot_laptop.sh)
export ROBOT_DAY_PREFIX="$PREFIX"
source "$venv/bin/activate"
EOF
  ok robot-env "$("$venv/bin/python" --version), source $PREFIX/robot-env.sh"
}

policy_env() {
  local venv="$PREFIX/policy-env" backend=()
  need_uv policy-env || return
  [ "$OS" = "Linux" ] && backend=(--torch-backend auto)   # picks the CUDA build that matches the NVIDIA driver, else CPU
  make_venv 3.12 "$venv" || { failed policy-env "could not create the Python 3.12 venv in $venv"; return; }
  # ${backend[@]+...}: pass the option only when set (an empty array is an "unbound variable" error under set -u on bash 3.2)
  uv pip install --python "$venv/bin/python" ${backend[@]+"${backend[@]}"} $POLICY_PINS \
    || { failed policy-env "pip install of lerobot/torch failed (internet? about 2 GB)"; return; }
  local spec repo rev
  for spec in $BACKBONES; do
    repo="${spec%@*}"; rev="${spec#*@}"
    HF_HUB_CACHE="$PREFIX/hf_cache/hub" "$venv/bin/python" -c \
      "import json, sys; from huggingface_hub import snapshot_download; snapshot_download(sys.argv[1], revision=sys.argv[2], ignore_patterns=json.loads(sys.argv[3]))" \
      "$repo" "$rev" "$BACKBONE_SKIP" \
      || { failed policy-env "download of $repo failed (internet? 2.7 GB in total) - re-run, it resumes"; return; }
    # offline (HF_HUB_OFFLINE=1) lerobot asks for "main"; a download by commit does not write that pointer, so point it at the pin
    # ${repo/\//--}: "lerobot/smolvla_base" -> "lerobot--smolvla_base", the Hugging Face cache folder name
    mkdir -p "$PREFIX/hf_cache/hub/models--${repo/\//--}/refs"
    printf '%s' "$rev" > "$PREFIX/hf_cache/hub/models--${repo/\//--}/refs/main"
  done
  cat > "$PREFIX/policy-env.sh" <<EOF
# source this before the policy server 81 (written by scripts/setup_robot_laptop.sh); the model never asks the internet
export ROBOT_DAY_PREFIX="$PREFIX"
export HF_HUB_CACHE="$PREFIX/hf_cache/hub"
export HF_HUB_OFFLINE=1
source "$venv/bin/activate"
EOF
  ok policy-env "source $PREFIX/policy-env.sh"
}

sha256_check() {  # in the tar's folder, against the .sha256 file next to it
  local dir; dir="$(dirname "$1")"
  if command -v shasum >/dev/null 2>&1; then
    (cd "$dir" && shasum -a 256 -c "$(basename "$1").sha256")
  else
    (cd "$dir" && sha256sum -c "$(basename "$1").sha256")
  fi
}

# Shared by --release and --release-file: checksum, unpack into a temp folder, check the files, then swap it in.
check_and_unpack() {
  local tar="$1" target="$PREFIX/$CKPT_DIR_REL" tmp name
  [ -f "$tar" ] && [ -f "$tar.sha256" ] || { failed release "$tar or $tar.sha256 not found"; return; }
  sha256_check "$tar" || { failed release "checksum of $tar does not match - delete it and download again"; return; }
  tmp="$PREFIX/$CKPT_DIR_REL.unpack.$$"
  rm -rf "$tmp"; mkdir -p "$tmp"
  tar -xf "$tar" -C "$tmp" || { rm -rf "$tmp"; failed release "unpacking $tar failed (disk full?)"; return; }
  for name in $CKPT_FILES; do
    [ -f "$tmp/pretrained_model/$name" ] || { rm -rf "$tmp"; failed release "pretrained_model/$name missing in $tar"; return; }
  done
  rm -rf "$target"; mv "$tmp" "$target"
  ok release "model in $target/pretrained_model"
}

release_download() {
  local dir="$PREFIX/release"
  command -v gh >/dev/null 2>&1 || { failed release "gh not found - install the GitHub CLI and run gh auth login (runbook Part A)"; return; }
  mkdir -p "$dir"
  # a re-run skips the 865 MB download when the tar from last time still matches its checksum (review E1-5)
  if [ -f "$dir/$TAR_NAME" ] && [ -f "$dir/$TAR_NAME.sha256" ] && sha256_check "$dir/$TAR_NAME" >/dev/null 2>&1; then
    echo "release: $dir/$TAR_NAME already downloaded and checksum OK - not downloading again"
    check_and_unpack "$dir/$TAR_NAME"; return
  fi
  gh release download "$RELEASE_TAG" --repo "$GH_REPO" --dir "$dir" --pattern "$TAR_NAME" --pattern "$TAR_NAME.sha256" --clobber \
    || { failed release "download of $RELEASE_TAG failed - gh auth status ok? internet? (865 MB)"; return; }
  check_and_unpack "$dir/$TAR_NAME"
}

show_ip() {
  echo "IPv4 addresses (the robot cable adapter must show 192.168.123.222, runbook Part B step 0):"
  if [ "$OS" = "Darwin" ]; then
    ifconfig | awk '/^[a-z]/ {iface=$1} /inet / {print "  " iface, $2}'
  else
    ip -4 -o addr show | awk '{print "  " $2 ":", $4}'
  fi
  ok show-ip
}

if [ $((DO_ROBOT + DO_POLICY)) -gt 0 ] || [ -n "$RELEASE_TAG$RELEASE_FILE" ]; then mkdir -p "$PREFIX"; fi
[ "$DO_ROBOT" -eq 1 ] && robot_env
if [ -n "$RELEASE_FILE" ]; then check_and_unpack "$RELEASE_FILE"; elif [ -n "$RELEASE_TAG" ]; then release_download; fi
[ "$DO_POLICY" -eq 1 ] && policy_env
[ "$DO_SHOW_IP" -eq 1 ] && show_ip
echo "repo: $REPO  prefix: $PREFIX"
exit $FAILED
