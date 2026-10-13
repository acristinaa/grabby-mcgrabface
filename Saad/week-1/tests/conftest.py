"""Gemeinsame Test-Hilfen.

Zwei Python-Umgebungen, beide werden von scripts/run_tests.sh durchlaufen:
  .venv (Projekt)       Tischszene, Handmontage, Normierung (mujoco 3.12)
  ~/dev/g1-ext/.venv    zusaetzlich LeRobot-G1-Sim, DDS, Lauf-Controller (mujoco 3.13)
Tests, die LeRobot oder DDS brauchen, werden in der Projekt-venv uebersprungen.

Alles mit DDS laeuft in einem eigenen Prozess: DDS-Threads halten sonst den Testprozess am Leben, und ein
abgebrochener Lauf haette den Kamera-Prozess der Sim im Testprozess.
"""
import importlib.util
import json
import os
import pathlib
import subprocess
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

HAS_G1EXT = all(importlib.util.find_spec(m) is not None for m in ("lerobot", "unitree_sdk2py"))
needs_g1ext = pytest.mark.skipif(not HAS_G1EXT, reason="braucht ~/dev/g1-ext/.venv (LeRobot + unitree_sdk2py)")

CYCLONEDDS_LIB = pathlib.Path.home() / "dev" / "g1-ext" / "cyclonedds" / "install" / "lib"


def run_python(args, timeout):
    """Python-Skript im eigenen Prozess mit derselben Umgebung starten. Rueckgabe: CompletedProcess."""
    env = dict(os.environ, PYTHONUNBUFFERED="1")
    env.setdefault("DYLD_LIBRARY_PATH", str(CYCLONEDDS_LIB))
    return subprocess.run([sys.executable, *map(str, args)], cwd=REPO, env=env,
                          capture_output=True, text=True, timeout=timeout)


def probe_result(proc):
    """Die Zeile "RESULT {json}" einer Sonde auslesen; bei Fehlen mit der Ausgabe als Fehlermeldung abbrechen."""
    for line in reversed(proc.stdout.splitlines()):
        if line.startswith("RESULT "):
            return json.loads(line[len("RESULT "):])
    pytest.fail(f"Sonde ohne Ergebnis (Exit {proc.returncode}).\n--- stdout ---\n{proc.stdout[-3000:]}"
                f"\n--- stderr ---\n{proc.stderr[-3000:]}")
