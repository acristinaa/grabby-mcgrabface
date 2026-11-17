"""Integration: BrainCo-Sim-Umgebung mit DDS, ohne Lauf-Controller (Sonde tests/helpers/sim_env_probe.py)."""
import pytest

from conftest import REPO, needs_g1ext, probe_result, run_python

pytestmark = [pytest.mark.integration, needs_g1ext]

PROBE = REPO / "tests" / "helpers" / "sim_env_probe.py"


@pytest.fixture(scope="module")
def probe():
    return probe_result(run_python([PROBE], timeout=180))


def test_befehl_fuer_rechte_schulter_zieht_nur_rechte_schulter(probe):
    # Mit der Original-Zuordnung (Gelenk-Index - 1) wuerde ein Handmotor ziehen
    assert probe["pulling_actuators"] == ["right_shoulder_pitch"]


def test_rechte_hand_folgt_brainco_befehl(probe):
    for q, target in zip(probe["hand_sim_norm"], probe["hand_target"]):
        assert abs(q - target) < 0.1


def test_handzustand_ueber_dds_gleich_sim(probe):
    assert len(probe["hand_dds_state"]) == 6
    for dds, sim in zip(probe["hand_dds_state"], probe["hand_sim_norm"]):
        assert abs(dds - sim) < 0.02


def test_reset_waehrend_schritten_bricht_nichts(probe):
    assert probe["race_errors"] == []
