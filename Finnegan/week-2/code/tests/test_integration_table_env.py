"""Integration: Greif-Umgebung am Tisch (g1sim/table_env.py) - Startpose frei, Halten ohne Absacken, ohne DDS.

Braucht das gebaute Laufmodell (scripts/51_build_brainco_sim.py); die Tischszene schreibt der Test selbst.
Befund 2026-09-15: In Nullstellung steckten die rechten Finger 2,1 cm in der Flasche (sie kippte beim Absetzen um),
und Halten mit "Sollwert = aktuelle Stellung" liess die Arme auf den Tisch sacken.
"""
import mujoco
import numpy as np
import pytest

from conftest import needs_g1ext

pytestmark = [pytest.mark.integration, needs_g1ext]

HOLD_S = 2.0


@pytest.fixture(scope="module")
def env():
    from g1sim import MANIFEST, SIM_DIR, table_env, table_scene

    if not (SIM_DIR / MANIFEST).is_file():
        pytest.skip("Laufmodell fehlt - zuerst scripts/51_build_brainco_sim.py")
    # Szene selbst schreiben: test_integration_brainco_model.py baut SIM_DIR neu und loescht sie dabei
    table_scene.write()
    e = table_env.TableEnv()
    yield e
    e.close()


def hold(env, target_fn):
    from g1sim import table_env

    touched = set()
    for _ in range(int(HOLD_S * table_env.CONTROL_HZ)):
        env.apply(target_fn())
        touched.update(env.robot_contacts())
    return touched


def test_startpose_ohne_kontakt(env):
    env.reset()
    assert env.robot_contacts() == []


def test_halten_ohne_absacken_und_ohne_kontakt(env):
    from g1sim import action_space, table_scene

    env.reset()
    start = {o: env.object_position(o) for o in table_scene.OBJECTS}
    assert hold(env, lambda: action_space.READY_POSE) == set()
    assert np.abs(env.state()[:14] - action_space.READY_POSE[:14]).max() < 0.1
    for o in table_scene.OBJECTS:
        assert np.linalg.norm(env.object_position(o) - start[o]) < 0.005, o


def test_zufallsstarts_ohne_kontakt(env):
    for _ in range(10):
        env.reset(randomize=True)
        assert env.robot_contacts() == []


def test_gegenprobe_nullstellung_steckt_in_flasche(env):
    """Gegenprobe: In der alten Startpose (Nullstellung) muss der Kontakt-Check die Finger in der Flasche finden."""
    env.reset()
    env.data.qpos[env.body_qadr] = 0.0
    mujoco.mj_forward(env.model, env.data)
    assert any(robot.startswith("rh_") and other == "red_bottle" for robot, other in env.robot_contacts())


def test_gegenprobe_sollwert_aktuelle_stellung_sackt_ab(env):
    """Gegenprobe zum alten Pruefskript: Wer state() als Sollwert schickt, laesst die Arme absacken."""
    from g1sim import action_space

    env.reset()
    hold(env, env.state)
    assert np.abs(env.state()[:14] - action_space.READY_POSE[:14]).max() > 0.1
