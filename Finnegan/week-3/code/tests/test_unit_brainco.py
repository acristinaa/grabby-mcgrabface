"""Unit: BrainCo-Handmotoren - Reihenfolge und Normierung wie an den echten Haenden.

Ein Fehler hier waere still: Jede aufgenommene Episode haette falsch skalierte oder vertauschte Handwerte.
"""
import numpy as np
import pytest

from g1sim import brainco

pytestmark = pytest.mark.unit


def test_reihenfolge_wie_echter_dds_dienst():
    # xr_teleoperate: Daumen, Daumen-Aux, Zeige, Mittel, Ring, Klein
    assert brainco.MOTORS == ("thumb_metacarpal", "thumb_proximal", "index_proximal",
                              "middle_proximal", "ring_proximal", "pinky_proximal")
    assert brainco.joint_names("right")[0] == "rh_right_thumb_metacarpal_joint"
    assert brainco.joint_names("left")[-1] == "lh_left_pinky_proximal_joint"


def test_prefix_gleich_wie_szene():
    from dashboard import scene

    assert brainco.PREFIX == scene.PREFIX


def test_null_ist_offen_eins_ist_zu():
    assert np.allclose(brainco.to_rad(np.zeros(6)), 0.0)
    assert np.allclose(brainco.to_rad(np.ones(6)), brainco.SIM_CLOSED_RAD)


def test_normierung_hin_und_zurueck():
    q = np.array([0.0, 0.25, 0.5, 0.75, 1.0, 0.1])
    assert np.allclose(brainco.to_norm(brainco.to_rad(q)), q)


def test_befehle_ausserhalb_0_bis_1_werden_begrenzt():
    rad = brainco.to_rad([-0.5, 1.5, 0.0, 0.0, 0.0, 0.0])
    assert rad[0] == 0.0
    assert rad[1] == pytest.approx(brainco.SIM_CLOSED_RAD[1])


def test_sim_zu_nicht_weiter_als_echte_hand():
    # Reale Wege laut xr_teleoperate: 1,52 / 1,05 / 1,47 rad
    real = np.array([1.52, 1.05, 1.47, 1.47, 1.47, 1.47])
    assert np.all(brainco.SIM_CLOSED_RAD <= real)
