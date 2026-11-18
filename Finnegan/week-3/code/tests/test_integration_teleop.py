"""Integration: Teleop-Kern (g1sim/teleop.py) - Greifreihenfolge, Grenzen, Totmann, iPhone-Umrechnung.

Den ganzen Ablauf mit simuliertem Bediener, Browser-Server und Umwandlung prueft scripts/58_teleop_check.py.
Braucht das gebaute Laufmodell (scripts/51_build_brainco_sim.py); die Tischszene schreibt der Test selbst.
"""
import types

import numpy as np
import pytest

from conftest import needs_g1ext

pytestmark = [pytest.mark.integration, needs_g1ext]


@pytest.fixture(scope="module")
def env():
    pytest.importorskip("mink")
    from g1sim import MANIFEST, SIM_DIR, table_env, table_scene

    if not (SIM_DIR / MANIFEST).is_file():
        pytest.skip("Laufmodell fehlt - zuerst scripts/51_build_brainco_sim.py")
    table_scene.write()
    e = table_env.TableEnv()
    yield e
    e.close()


@pytest.fixture
def arm(env):
    from g1sim import teleop

    env.reset()
    a = teleop.TeleopArm(env)
    a.reset()
    return a


def test_greifwert_erst_daumen_dann_finger():
    from g1sim import teleop

    assert teleop.grip_to_hand(0.0) == pytest.approx(np.zeros(6))
    assert teleop.grip_to_hand(teleop.THUMB_SHARE) == pytest.approx([1, 0, 0, 0, 0, 0])
    # Fully closed = the grasp's CLOSED: ring and pinky stay open since D2 (2026-10-02)
    assert teleop.grip_to_hand(1.0) == pytest.approx(teleop.CLOSED)
    values = np.array([teleop.grip_to_hand(g) for g in np.linspace(0, 1, 50)])
    assert np.all(np.diff(values, axis=0) >= 0)


def test_finger_warten_auf_den_daumen(arm):
    """Befund Stufe 0.3/Teleop: Schliessen die Finger vor dem Daumen, blockieren sie ihn und die Flasche rutscht durch."""
    from g1sim import teleop

    for _ in range(10):  # 1/3 s: der Daumen ist noch lange nicht gedreht (~1,8 s)
        arm.step(arm.target, 1.0)
    assert arm.grip <= teleop.THUMB_SHARE + 1e-9


def test_daumensperre_oeffnet_nie(arm):
    """Befund Teleop: Drueckt die Flasche beim Tragen den Daumen zurueck, darf die Hand nicht aufgehen."""
    arm.grip = 0.8  # Hand fast zu, gemessener Daumen steht nach reset() auf 0 - wie weggedrueckt
    for _ in range(10):
        arm.step(arm.target, 1.0)
    assert arm.grip >= 0.8


def test_ziel_folgt_hoechstens_mit_max_tempo(arm):
    from g1sim import teleop

    far = teleop._pose(arm.target.rotation(), arm.target.translation() + [0.0, 0.0, 1.0])
    before = arm.target.translation()
    for _ in range(5):
        arm.step(far, 0.0)
        after = arm.target.translation()
        assert np.linalg.norm(after - before) <= teleop.MAX_SPEED * teleop.DT + 1e-9
        before = after


def test_ziel_bleibt_im_arbeitsraum(arm):
    from g1sim import teleop

    far = teleop._pose(arm.target.rotation(), arm.body_pos + arm.to_world([2.0, -2.0, -2.0]))
    for _ in range(150):
        arm.step(far, 0.0)
    inside = arm.in_body(arm.target.translation())
    assert np.all(inside >= teleop.WORKSPACE[:, 0] - 1e-9) and np.all(inside <= teleop.WORKSPACE[:, 1] + 1e-9)


def test_ohne_eingabe_bleibt_alles_stehen(env, tmp_path):
    """Totmann: keine Taste (Browser weg) -> Ziel bewegt sich nicht, keine Aufnahme."""
    from g1sim import teleop

    session = teleop.TeleopSession(env, tmp_path)
    start = session.arm.target.translation()
    for _ in range(30):
        session.tick(teleop.Inputs())
    assert np.allclose(session.arm.target.translation(), start, atol=1e-9)
    assert session.state == "bereit" and session.recorder is None


def test_aufnahme_ab_erster_bewegung_verwerfen_setzt_zurueck(env, tmp_path):
    from g1sim import teleop

    session = teleop.TeleopSession(env, tmp_path)
    start = session.arm.target.translation()
    for _ in range(3):
        session.tick(teleop.Inputs(held=frozenset({"KeyW"})))
    assert session.state == "aufnahme" and len(session.recorder) == 3
    assert session.arm.in_body(session.arm.target.translation())[0] > session.arm.in_body(start)[0]
    session.tick(teleop.Inputs(events=("discard",)))
    assert session.state == "bereit" and session.recorder is None
    assert np.allclose(session.arm.target.translation(), start, atol=1e-6)
    assert list(tmp_path.glob("*.npz")) == []


def test_iphone_kupplung_und_richtung():
    """10 cm Handy nach vorn (ARKit -z) bei gehaltenem Knopf = 10 cm Hand nach vorn; Loslassen haelt die Hand an."""
    from g1sim import teleop

    arm = types.SimpleNamespace(body_rot=np.eye(3), target=teleop._pose(teleop.mink.SO3.identity(), [0.3, -0.2, 0.9]))
    pilot = teleop.PhonePilot()
    pressed = {"position": np.zeros(3), "rotation": np.eye(3), "button": True, "toggle": True}
    pilot.update(arm, pressed)
    goal, grip, acted = pilot.update(arm, {**pressed, "position": np.array([0.0, 0.0, -0.10])})
    assert goal.translation() - arm.target.translation() == pytest.approx([0.10, 0.0, 0.0])
    assert grip == 1.0 and acted
    held, _, acted = pilot.update(arm, {**pressed, "button": False})
    assert not acted and held.translation() == pytest.approx(arm.target.translation())
