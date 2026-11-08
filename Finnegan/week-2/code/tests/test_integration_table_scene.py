"""Integration: Tischszene (Menagerie-G1 + Adapter + Revo2) bauen und pruefen.

Nutzt dieselbe Anbau-Funktion wie das Laufmodell (dashboard/scene.py::attach_revo2).
"""
import mujoco
import numpy as np
import pytest

from dashboard import scene
from g1sim import brainco

pytestmark = pytest.mark.integration

OLD_MOUNT = {  # Stand bis 2026-09-14: Gummihand-Position, keine Drehung
    "left": {"pos": [0.0415, 0.003, 0.0], "euler": [0.0, 0.0, 0.0]},
    "right": {"pos": [0.0415, -0.003, 0.0], "euler": [0.0, 0.0, 0.0]},
}


def compile_table(mount=None):
    if not scene.SCENE_IN.is_file():
        pytest.skip("Tischszene fehlt - zuerst scripts/10_build_scene.py")
    model = scene.build_with_hands(mount=mount, offscreen=None).compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def hand_contacts(model, data):
    """Kontakte zwischen Hand/Adapter und dem restlichen Roboter."""
    def is_hand(body):
        return (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body) or "").startswith(("lh_", "rh_"))
    pairs = [(model.geom_bodyid[data.contact[i].geom1], model.geom_bodyid[data.contact[i].geom2])
             for i in range(data.ncon)]
    return [p for p in pairs if is_hand(p[0]) != is_hand(p[1])]


@pytest.fixture(scope="module")
def table():
    return compile_table()


def test_12_handmotoren(table):
    model, _ = table
    assert model.nu == 41
    assert len(scene.hand_actuator_ids(model)) == 12


def test_montage_wie_unitree_urdf(table):
    for pos_mm, angle_deg in scene.mount_error(*table).values():
        assert pos_mm < 1.0
        assert angle_deg < 0.5


def test_keine_handkontakte_in_ruhe(table):
    assert hand_contacts(*table) == []


def test_adapter_mit_masse_aus_urdf(table):
    model, _ = table
    for side in ("left", "right"):
        assert model.body_mass[model.body(f"{scene.PREFIX[side]}{side}_base2_link").id] == pytest.approx(0.0228, abs=1e-4)


def test_alte_montage_wird_erkannt():
    """Gegenprobe: Mit der alten Montage muessen Montage-Check und Kontakt-Check anschlagen."""
    model, data = compile_table(OLD_MOUNT)
    assert all(angle_deg > 45.0 for _, angle_deg in scene.mount_error(model, data).values())
    assert hand_contacts(model, data), "linker Daumen steckte im Handgelenk - Kontakt-Check muss das finden"


def test_haende_schliessen(table):
    model, _ = table
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    open_tips = {s: brainco.tips_in_hand_frame(model, data, s) for s in brainco.SIDES}
    for i in scene.hand_actuator_ids(model):
        data.ctrl[i] = model.actuator_ctrlrange[i][1]
    for _ in range(1500):
        mujoco.mj_step(model, data)
    for side in brainco.SIDES:
        travel = np.linalg.norm(brainco.tips_in_hand_frame(model, data, side) - open_tips[side], axis=1)
        assert travel.min() > 0.01, f"{side}: Fingerspitze bewegt sich nur {travel.min() * 100:.2f} cm"
