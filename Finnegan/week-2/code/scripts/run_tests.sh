#!/usr/bin/env bash
# Alle Tests in allen drei Umgebungen.
#   scripts/run_tests.sh        Unit + Integration (ca. 2-3 min)
#   scripts/run_tests.sh all    zusaetzlich End-to-End: verbinden, Haende per DDS, 10 m laufen (ca. 8-10 min)
# Weitere pytest-Argumente werden durchgereicht, z. B.: scripts/run_tests.sh all -k laeuft
set -uo pipefail
cd "$(dirname "$0")/.."

ARGS=()
if [ "${1:-}" = "all" ]; then
  shift
else
  ARGS=(-m "not e2e")
fi

echo "== Projekt-Umgebung (.venv): Tischszene, Montage, Normierung"
.venv/bin/python -m pytest "${ARGS[@]}" "$@"
PROJECT=$?

echo "== LeRobot-Umgebung (~/dev/g1-ext/.venv): zusaetzlich Laufmodell, DDS, Lauf-Controller, Greifszene, openpi-Client"
DYLD_LIBRARY_PATH="$HOME/dev/g1-ext/cyclonedds/install/lib" "$HOME/dev/g1-ext/.venv/bin/python" -m pytest "${ARGS[@]}" "$@"
G1EXT=$?

# Nur die openpi-Tests: dort liegt ein aelteres mujoco, die Sim-Tests gehoeren in die anderen beiden Umgebungen
echo "== openpi-Umgebung (~/dev/g1-ext/openpi/.venv): Konfiguration, Umrechnung, Ersatz-Policy, Datensatz"
OPENPI_PY="$HOME/dev/g1-ext/openpi/.venv/bin/python"
if [ -x "$OPENPI_PY" ]; then
  "$OPENPI_PY" -m pytest "${ARGS[@]}" tests/test_unit_openpi_policy.py tests/test_integration_openpi.py "$@"
  OPENPI=$?
else
  echo "openpi-Umgebung fehlt - scripts/setup_openpi.sh"
  OPENPI=1
fi

echo "== Ergebnis: Projekt-Umgebung Exit $PROJECT, LeRobot-Umgebung Exit $G1EXT, openpi-Umgebung Exit $OPENPI"
[ "$PROJECT" -eq 0 ] && [ "$G1EXT" -eq 0 ] && [ "$OPENPI" -eq 0 ]
