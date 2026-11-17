"""Unit: Anbauteile (g1sim/payload.py) an einem Ersatz-Oberkoerper - Anbaurahmen und Servo-Mathematik.

Kopfstellung = Body-Drehung + joint.ref + equality. Rechnet eine davon falsch, sieht die Kopfkamera woanders hin, ohne
dass ein Fehler auftritt. scripts/51_build_brainco_sim.py prueft das gebaute Laufmodell gegen das Unitree-URDF.
"""
import mujoco
import numpy as np
import pytest

from g1sim import payload

pytestmark = pytest.mark.unit

TABLE_TILT = 0.85
XL330_PITCH = 0.039968  # Unitree-URDF: der Servo sitzt schon leicht nach vorn geneigt


def build(pan=0.0, tilt=0.0):
    spec = mujoco.MjSpec()
    torso = spec.worldbody.add_body()
    torso.name = payload.PARENT
    payload.attach(spec, pan=pan, tilt=tilt)
    model = spec.compile()
    return model, mujoco.MjData(model)


def pose_at(model, data, pan, tilt):
    """Weltpose des D455-Bodys und der Kopfkamera bei gesetzten Servowinkeln (ohne Physikschritt)."""
    data.qpos[model.joint("xl330_joint").qposadr[0]] = pan
    data.qpos[model.joint("d455_joint").qposadr[0]] = tilt
    mujoco.mj_forward(model, data)
    body, cam = model.body("d455_link").id, model.camera(payload.CAMERA).id
    return np.concatenate([data.xpos[body], data.xmat[body], data.cam_xpos[cam], data.cam_xmat[cam]])


def test_anbaurahmen_1cm_unter_lerobots_torso():
    model, data = build()
    mujoco.mj_forward(model, data)
    # Unitree-URDF: head_servo_joint (0.0039635, 0, -0.047) im URDF-torso_link, der 1 cm unter LeRobots liegt
    assert data.xpos[model.body("head_servo_link").id] == pytest.approx([0.0039635, 0.0, -0.057], abs=1e-9)


def test_startstellung_ist_derselbe_winkel_wie_gesetztes_qpos():
    """Start bei (0.3, 0.85) muss dieselbe Kinematik ergeben wie Start bei 0 und qpos = (0.3, 0.85)."""
    started, d1 = build(pan=0.3, tilt=TABLE_TILT)
    zero, d2 = build()
    assert started.qpos0[started.joint("d455_joint").qposadr[0]] == pytest.approx(TABLE_TILT)
    assert pose_at(started, d1, 0.3, TABLE_TILT) == pytest.approx(pose_at(zero, d2, 0.3, TABLE_TILT), abs=1e-9)
    assert pose_at(started, d1, 0.0, 0.0) == pytest.approx(pose_at(zero, d2, 0.0, 0.0), abs=1e-9)


def test_kopf_neigt_nach_unten():
    model, data = build(tilt=TABLE_TILT)
    mujoco.mj_forward(model, data)
    forward = -data.cam_xmat[model.camera(payload.CAMERA).id].reshape(3, 3)[:, 2]
    down = TABLE_TILT + XL330_PITCH
    assert forward == pytest.approx([np.cos(down), 0.0, -np.sin(down)], abs=1e-6)


@pytest.mark.parametrize("start", [0.0, TABLE_TILT])
def test_servo_haelt_und_folgt_unter_schwerkraft(start):
    model, data = build(tilt=start)
    mujoco.mj_step(model, data, nstep=500)
    assert payload.head_angles(model, data)["tilt"] == pytest.approx(start, abs=0.01)
    payload.set_head(model, pan=-0.5, tilt=0.3)
    mujoco.mj_step(model, data, nstep=500)
    assert payload.head_angles(model, data) == pytest.approx({"pan": -0.5, "tilt": 0.3}, abs=0.01)


def test_winkel_ausserhalb_des_bereichs():
    model, _ = build()
    with pytest.raises(ValueError):
        payload.set_head(model, tilt=1.7)
    with pytest.raises(ValueError):
        build(pan=-1.0)
