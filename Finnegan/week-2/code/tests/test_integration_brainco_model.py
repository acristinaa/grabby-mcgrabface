"""Integration: Laufmodell (LeRobot-G1 + Adapter + Revo2) bauen und pruefen - ohne DDS.

Baut ~/dev/g1-ext/g1_brainco_sim neu, genau wie scripts/51_build_brainco_sim.py.
"""
import mujoco
import numpy as np
import pytest
import yaml

from conftest import needs_g1ext
from dashboard import scene

pytestmark = [pytest.mark.integration, needs_g1ext]

BODY_RULE = ("hip", "knee", "ankle", "waist", "shoulder", "elbow", "wrist")  # Namensregel in sim/base_sim.py


@pytest.fixture(scope="module")
def built():
    from g1sim import build

    snap = build.snapshot()
    spec = build.build_spec(snap)
    spec.compile()
    model, _ = build.write_sim(spec, snap)
    return model


def test_29_koerpermotoren_12_handmotoren(built):
    hand = [i for i in range(built.nu) if built.actuator(i).name.startswith(("lh_", "rh_"))]
    assert (built.nu, len(hand)) == (41, 12)


def test_alle_aktuatoren_sind_drehmoment_motoren(built):
    assert np.all(built.actuator_biastype == mujoco.mjtBias.mjBIAS_NONE)
    assert np.all(built.actuator_gainprm[:, 0] == 1.0)
    assert np.all(built.actuator_ctrllimited)


def test_sim_namensregel_findet_genau_29_koerpergelenke(built):
    n = sum(1 for j in range(built.njnt) if any(p in built.joint(j).name for p in BODY_RULE))
    assert n == 29


@pytest.mark.parametrize("joint", ["right_shoulder_pitch_joint", "right_elbow_joint", "right_wrist_yaw_joint"])
def test_rechter_arm_motor_zuordnung(built, joint):
    """Regression: Die Original-Sim nimmt Motor = Gelenk-Index - 1. Mit Revo2 ist das fuer den rechten Arm falsch."""
    j = built.joint(joint).id
    act = int(np.flatnonzero(built.actuator_trnid[:, 0] == j)[0])
    assert built.actuator(act).name == joint.removesuffix("_joint")
    assert act != j - 1, "Gegenprobe: genau hier waere die Annahme der Original-Sim falsch"


def test_montage_wie_unitree_urdf(built):
    data = mujoco.MjData(built)
    mujoco.mj_forward(built, data)
    for pos_mm, angle_deg in scene.mount_error(built, data).values():
        assert pos_mm < 1.0
        assert angle_deg < 0.5


def test_config_ohne_dex3():
    from g1sim import SCENE_FILE, SIM_DIR

    cfg = yaml.safe_load((SIM_DIR / "config.yaml").read_text())
    assert cfg["ROBOT_SCENE"] == SCENE_FILE
    assert cfg["NUM_HAND_MOTORS"] == 0
    assert len(cfg["motor_effort_limit_list"]) == 29
