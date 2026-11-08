"""Integration: clearance meter of the sim teacher (g1sim/hand_clearance.py) in the walk world.

(1) the meter sees a planted pinky-stand overlap and labels only arm parts from the elbow down, (2) the teacher's nominal
walk-world run keeps NOMINAL_MARGIN_M from table, stand and basket with every such part.
Needs the built walking model (scripts/51_build_brainco_sim.py); writes the scene XML (table_scene.write).
"""
import mujoco
import pytest

from conftest import needs_g1ext

pytestmark = [pytest.mark.integration, needs_g1ext]

SCENE = "volleyball_walk"
# m: gate G7 of .planning/level1-harden/FINGER-PLAN.md (p5 >= 3.5 cm). Probe on the nominal start: old constants 2.56 cm
# (left pinky - stand at the touch), new constants 4.06 cm.
NOMINAL_MARGIN_M = 0.035
PINKY_BODY = "lh_left_pinky_distal_link"


@pytest.fixture(scope="module")
def scene():
    from g1sim import MANIFEST, SIM_DIR, table_scene

    if not (SIM_DIR / MANIFEST).is_file():
        pytest.skip("walking model missing - run scripts/51_build_brainco_sim.py first")
    table_scene.write(scene=SCENE)


@pytest.fixture
def env(scene):
    from g1sim import table_env

    e = table_env.TableEnv(scene=SCENE)
    e.reset()
    yield e
    e.close()


def collision_geom_of(model, body_name):
    """First collision geom (contype or conaffinity) of the body."""
    body = model.body(body_name).id
    return next(g for g in range(model.ngeom)
                if model.geom_bodyid[g] == body and (model.geom_contype[g] or model.geom_conaffinity[g]))


def test_meter_sees_a_pinky_touching_the_stand(env):
    """Proves: the meter measures hand geoms against forbidden bodies with real distances, and labels exactly the arm from the
    elbow down. Catches: a geom filter that drops the hand, a clamp that hides contact, or one that lets legs/torso in."""
    from g1sim import hand_clearance

    m, d = env.model, env.data
    stand = m.body("volleyball_stand").id
    m.body_pos[stand] += d.geom_xpos[collision_geom_of(m, PINKY_BODY)] - d.geom_xpos[m.geom("volleyball_stand_0").id]
    mujoco.mj_forward(m, d)
    meter = hand_clearance.ClearanceMeter(env)
    meter.update("probe")

    assert meter.result()["min_m"]["left|pinky|volleyball_stand"] <= 0.001
    bodies = {m.body(m.geom_bodyid[g]).name for g, _, _ in meter.parts}
    assert {PINKY_BODY, "right_wrist_yaw_link"} <= bodies
    assert not bodies & {"pelvis", "torso_link", "left_knee_link", "left_shoulder_pitch_link"}


def test_teacher_keeps_the_margin_on_the_nominal_walk_start(env):
    """Proves: the teacher's nominal path keeps 3.5 cm from table, stand and basket with every arm part from the elbow down.
    Catches: a return to the old cradle or reach height (2.6 cm at the stand)."""
    from g1sim import ball_grasp, hand_clearance

    meter = hand_clearance.ClearanceMeter(env)
    result = ball_grasp.TwoHandBallGrasp(env, on_tick=meter.update).run()

    assert result["passed"], (result["first_failed"], result["forbidden_contacts"])
    closest = meter.result()["closest"]
    assert closest["d_m"] >= NOMINAL_MARGIN_M, closest
