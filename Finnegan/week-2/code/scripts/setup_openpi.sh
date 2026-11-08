#!/usr/bin/env bash
# openpi-Umgebung einrichten - Trainingsstack entschieden 2026-09-14 (PLAN.md, SCOPE UPDATE).
#
#   Ubuntu + NVIDIA-GPU (Training, Policy-Server): wie openpi-README ueber `uv sync`, mit CUDA-JAX
#   macOS (Entwicklung): CPU-JAX statt CUDA - fuer Konfiguration, Datenumwandlung, Normierungs-Statistik; kein Training
#
# Zusaetzlich auf dem Mac: openpi-client in die Sim-Umgebung ~/dev/g1-ext/.venv. Dort laeuft numpy 2 (LeRobot),
# openpi-client pinnt numpy < 2, laeuft aber mit 2 -> Installation ohne Abhaengigkeiten.
#
# Aufruf: scripts/setup_openpi.sh        (OPENPI_DIR ueberschreibbar, Standard ~/dev/g1-ext/openpi)
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OPENPI_DIR="${OPENPI_DIR:-$HOME/dev/g1-ext/openpi}"
OPENPI_COMMIT=215abfb   # Stand der Recherche 2026-09-14
SIM_PY="$HOME/dev/g1-ext/.venv/bin/python"

command -v uv >/dev/null || { echo "uv fehlt: brew install uv (macOS) bzw. curl -LsSf https://astral.sh/uv/install.sh | sh"; exit 1; }

if [ ! -d "$OPENPI_DIR/.git" ]; then
  GIT_LFS_SKIP_SMUDGE=1 git clone --recurse-submodules https://github.com/Physical-Intelligence/openpi.git "$OPENPI_DIR"
fi
cd "$OPENPI_DIR"
GIT_LFS_SKIP_SMUDGE=1 git checkout -q "$OPENPI_COMMIT"
git submodule update --init --recursive -q
echo "openpi $(git log -1 --format='%h %cd' --date=short) in $OPENPI_DIR"

if [ "$(uname)" = "Linux" ]; then
  GIT_LFS_SKIP_SMUDGE=1 uv sync
  GIT_LFS_SKIP_SMUDGE=1 uv pip install -e .
else
  uv venv --python 3.11 .venv
  OVERRIDES="$(mktemp)"
  # jax[cuda12] gibt es nicht fuer macOS; ml-dtypes/tensorstore spiegeln [tool.uv] override-dependencies.
  # Ohne uv.lock loest uv sonst neuere Pakete auf: datasets 5.x bricht lerobot 0.1.0 beim Laden eines Datensatzes
  # (torch.stack auf Column, gefunden 2026-09-15). Daher Datensatz-Stapel wie in openpis uv.lock; gcsfs passt nur zu gleichem fsspec.
  printf 'jax==0.5.3\nml-dtypes==0.4.1\ntensorstore==0.1.74\ndatasets==3.6.0\npyarrow==20.0.0\nhuggingface-hub==0.32.3\nfsspec==2025.3.0\ngcsfs==2025.3.0\n' > "$OVERRIDES"
  GIT_LFS_SKIP_SMUDGE=1 uv pip install --python .venv/bin/python --override "$OVERRIDES" -e . -e packages/openpi-client
  rm -f "$OVERRIDES"
fi

# Unser Repo (openpi_g1, g1sim.action_space) in der openpi-Umgebung importierbar machen
SITE="$(.venv/bin/python -c 'import site; print(site.getsitepackages()[0])')"
echo "$REPO" > "$SITE/g1_brainco.pth"
.venv/bin/python -c "import openpi, openpi_client, openpi_g1; print('openpi-Umgebung ok:', openpi.__file__)"
uv pip install --python .venv/bin/python -q pytest   # Tests der openpi-Anbindung (scripts/run_tests.sh)

if [ -x "$SIM_PY" ]; then
  "$SIM_PY" -m pip install -q --no-deps -e "$OPENPI_DIR/packages/openpi-client"
  "$SIM_PY" -m pip install -q websockets dm-tree
  "$SIM_PY" -c "import numpy, openpi_client.websocket_client_policy; print('openpi-client in der Sim-Umgebung ok, numpy', numpy.__version__)"
fi
