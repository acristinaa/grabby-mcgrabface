"""Unit: Handmontage - stimmt die Standardmontage mit Unitrees offiziellem G1+BrainCo-URDF ueberein?

Bis 2026-09-14 waren beide Haende 90 Grad abgeknickt und die linke verdreht. Diese Tests pruefen die Mathematik
ohne Szene; tests/test_integration_table_scene.py prueft das gebaute Modell.
"""
import mujoco
import numpy as np
import pytest

from dashboard import scene

pytestmark = pytest.mark.unit


def quat(euler, seq):
    q = np.zeros(4)
    mujoco.mju_euler2Quat(q, np.asarray(euler, dtype=float), seq)
    return q


def same_rotation(a, b):
    return abs(abs(float(np.dot(a, b))) - 1.0) < 1e-9


def hand_base_quat(side):
    """Drehung, die das Hersteller-MJCF seiner Handbasis selbst gibt."""
    model = mujoco.MjSpec.from_file(str(scene.HAND_XML[side])).compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return data.xquat[model.body(f"{side}_base_link").id].copy()


@pytest.mark.parametrize("side", ["left", "right"])
def test_standardmontage_ergibt_urdf_drehung(side):
    total = np.zeros(4)
    mujoco.mju_mulQuat(total, quat(scene.DEFAULT_MOUNT[side]["euler"], "xyz"), hand_base_quat(side))
    assert same_rotation(total, quat(scene.URDF_HAND_BASE[side]["rpy"], "XYZ"))


@pytest.mark.parametrize("side", ["left", "right"])
def test_standardmontage_ergibt_urdf_position(side):
    assert np.allclose(scene.DEFAULT_MOUNT[side]["pos"], scene.URDF_HAND_BASE[side]["xyz"])


def test_rechte_hand_hat_lose_spitzengelenke_linke_nicht():
    right = mujoco.MjSpec.from_file(str(scene.HAND_XML["right"]))
    left = mujoco.MjSpec.from_file(str(scene.HAND_XML["left"]))
    assert len(scene.remove_passive_tip_joints(right)) == 5
    assert scene.remove_passive_tip_joints(left) == []
    assert len(right.joints) == len(left.joints) == 11
