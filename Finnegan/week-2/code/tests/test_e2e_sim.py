"""End-to-End: LeRobot-Roboter + BrainCo-Sim + GR00T-Lauf-Controller + DDS, wie spaeter Navigation und Greifen.

Dauert einige Minuten. Die Abnahme-Skripte laufen unveraendert, ihr Ergebnis-JSON wird geprueft.
"""
import json

import pytest

from conftest import REPO, needs_g1ext, probe_result, run_python

pytestmark = [pytest.mark.e2e, needs_g1ext]


@pytest.mark.parametrize("attempt", [1, 2, 3])
def test_verbinden_sim_laeuft_weiter(attempt):
    """Mehrfach, weil der Wettlauf in connect() zufallsabhaengig war."""
    result = probe_result(run_python([REPO / "tests" / "helpers" / "connect_probe.py"], timeout=120))
    assert result["sim_advanced_s"] >= 1.0
    assert result["pelvis_z"] > 0.5


def test_haende_per_dds(tmp_path):
    out = tmp_path / "hands.json"
    proc = run_python([REPO / "scripts" / "52_brainco_hand_dds.py", "--out", out], timeout=400)
    result = json.loads(out.read_text())
    assert proc.returncode == 0 and result["passed"], result.get("checks")


def test_laeuft_10_m_mit_brainco_haenden(tmp_path):
    out = tmp_path / "walk.json"
    proc = run_python([REPO / "scripts" / "50_spike_lerobot_walk.py", "--headless", "--brainco", "--out", out],
                      timeout=400)
    result = json.loads(out.read_text())
    assert proc.returncode == 0 and result["passed"], result
    # Beide Arme halten ihre Sollstellung beim Laufen (Dex3-Vergleich 2026-09-14: 0,08 rad)
    assert max(result["arm_tracking_max_rad"].values()) < 0.2
