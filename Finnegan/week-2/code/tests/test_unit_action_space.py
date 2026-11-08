"""Unit: 27-D action space - indices 0-25 as unitree_lerobot, 26 waist yaw, Rundlauf ohne Verlust."""
import numpy as np
import pytest

from g1sim import action_space as aspace
from g1sim import brainco

pytestmark = pytest.mark.unit

# Abgeschrieben aus unitree_lerobot/utils/constants.py (Commit 41c2805), G1_BRAINCO_CONFIG.motors
UNITREE_LEROBOT_G1_BRAINCO = [
    "kLeftShoulderPitch", "kLeftShoulderRoll", "kLeftShoulderYaw", "kLeftElbow",
    "kLeftWristRoll", "kLeftWristPitch", "kLeftWristYaw",
    "kRightShoulderPitch", "kRightShoulderRoll", "kRightShoulderYaw", "kRightElbow",
    "kRightWristRoll", "kRightWristPitch", "kRightWristYaw",
    "kLeftHandThumb", "kLeftHandThumbAux", "kLeftHandIndex", "kLeftHandMiddle", "kLeftHandRing", "kLeftHandPinky",
    "kRightHandThumb", "kRightHandThumbAux", "kRightHandIndex", "kRightHandMiddle", "kRightHandRing", "kRightHandPinky",
]


def test_layout_wie_unitree_lerobot():
    assert list(aspace.NAMES[:26]) == UNITREE_LEROBOT_G1_BRAINCO
    assert aspace.NAMES[26] == "kWaistYaw" and aspace.SLICES["waist"] == slice(26, 27)
    assert aspace.DIM == 27 == len(aspace.NAMES) == len(aspace.READY_POSE) == len(aspace.DELTA_MASK)


def test_handmotoren_passen_zu_brainco_reihenfolge():
    # Daumen = Daumendrehung (thumb_metacarpal), Daumen-Aux = Daumenbeugung (thumb_proximal)
    assert len(aspace.HAND_MOTORS) == len(brainco.MOTORS)
    assert brainco.MOTORS[0] == "thumb_metacarpal" and aspace.HAND_MOTORS[0] == "Thumb"
    assert brainco.MOTORS[1] == "thumb_proximal" and aspace.HAND_MOTORS[1] == "ThumbAux"


def test_rundlauf_sim_vektor_sollwerte():
    rng = np.random.default_rng(0)
    arm = {s: rng.uniform(-1, 1, 7) for s in aspace.SIDES}
    hand_norm = {s: rng.uniform(0, 1, 6) for s in aspace.SIDES}
    vec = aspace.from_sim(arm, {s: brainco.to_rad(hand_norm[s]) for s in aspace.SIDES}, 0.4)
    assert vec.shape == (aspace.DIM,) and vec.dtype == np.float32
    assert aspace.split(vec)["waist"][0] == pytest.approx(0.4)
    for side in aspace.SIDES:
        assert np.allclose(aspace.arm_targets(vec)[side], arm[side], atol=1e-6)
        assert np.allclose(aspace.hand_targets(vec)[side], hand_norm[side], atol=1e-6)


def test_lerobot_arm_aktion_hat_die_14_armschluessel():
    action = aspace.lerobot_arm_action(np.arange(aspace.DIM, dtype=np.float32))
    assert list(action) == [f"{n}.q" for n in UNITREE_LEROBOT_G1_BRAINCO[:14]]
    assert action["kRightShoulderPitch.q"] == 7.0


def test_delta_nur_fuer_arme():
    assert aspace.DELTA_MASK == (True,) * 14 + (False,) * 12 + (True,)  # arms and waist yaw as deltas


def test_falsche_laenge_wird_abgelehnt():
    with pytest.raises(ValueError):
        aspace.split(np.zeros(29))
