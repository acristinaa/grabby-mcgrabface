"""Integration: volleyball scene (Level 1, g1sim/table_scene.py SCENES["volleyball"]) - ball, stand, basket, resting ball.

Also guards the bottle scene: table_scene.write() without a scene must still write exactly the bottle scene.
Needs the built walking model (scripts/51_build_brainco_sim.py); the test writes the scenes itself.
"""
import mujoco
import numpy as np
import pytest

from conftest import needs_g1ext

pytestmark = [pytest.mark.integration, needs_g1ext]


@pytest.fixture(scope="module")
def env():
    from g1sim import MANIFEST, SIM_DIR, table_env, table_scene

    if not (SIM_DIR / MANIFEST).is_file():
        pytest.skip("walking model missing - run scripts/51_build_brainco_sim.py first")
    table_scene.write(scene="volleyball")
    e = table_env.TableEnv(scene="volleyball")
    yield e
    e.close()


def test_ball_is_a_hollow_sphere(env):
    m = env.model
    geom, body = m.geom("volleyball_geom"), m.body("volleyball")
    assert geom.type[0] == mujoco.mjtGeom.mjGEOM_SPHERE
    assert geom.size[0] == pytest.approx(0.105)
    assert geom.condim[0] == 6
    assert np.allclose(geom.rgba, [0.55, 0.55, 0.55, 1.0])  # grey like the real ball
    assert body.mass[0] == pytest.approx(0.27)
    assert np.allclose(body.inertia, 2.0 / 3.0 * 0.27 * 0.105 ** 2)
    assert m.jnt_type[m.joint("volleyball_joint").id] == mujoco.mjtJoint.mjJNT_FREE


def test_stand_and_basket_are_static(env):
    m = env.model
    for name in ("volleyball_stand", "basket"):
        body = m.body(name).id
        assert m.body_jntnum[body] == 0, name
        assert m.body_weldid[body] == 0, name
        assert body in env.scene_bodies, f"{name}: contacts with it must be reported"


def test_ball_rests_on_stand(env):
    from g1sim import action_space, table_env, table_scene

    env.reset()
    start = env.object_position("volleyball")
    for _ in range(table_env.CONTROL_HZ):  # 1 s
        env.apply(action_space.READY_POSE)
    ball = env.object_position("volleyball")
    assert np.linalg.norm(ball - start) < 0.003
    assert ball[2] == pytest.approx(table_scene.object_rest_height("volleyball"), abs=0.003)
    assert env.lift_height() == pytest.approx(0.0, abs=0.003)  # default target object is the ball


def test_randomization_varies_every_group(env):
    """Volleyball scene: randomize=True varies ball, stand, physics, image and the distractors - not only the distractors."""
    from g1sim import randomization

    env.reset(randomize=True)
    nominal = randomization.nominal(env.objects)
    for group in randomization.GROUPS:
        assert env.variation[group] != nominal[group], group
    assert env.variation["image"]["object_rgb"]["volleyball"] is not None  # the target's colour is jittered, stays red
    env.reset()


def test_volleyball_ranges_come_from_the_scenes_block(env):
    """Ball offsets stay within the volleyball override (narrower than the bottle range), stand within the stand group."""
    from g1sim import randomization

    config = randomization.load()
    ball_offset = randomization.for_scene(config, "volleyball")["objects"]["offset_m"]
    assert ball_offset < config["objects"]["offset_m"]
    rng = np.random.default_rng(0)
    for _ in range(200):
        v = randomization.sample(config, rng, "volleyball")
        o = v["objects"]["volleyball"]
        assert max(abs(o["dx_m"]), abs(o["dy_m"])) <= ball_offset
        assert abs(v["stand"]["dx_m"]) <= config["stand"]["forward_m"]


def test_random_starts_keep_the_ball_in_reach(env):
    """reset(randomize=True) redraws starts where stand and ball offset add up to a ball out of the teacher's reach."""
    from g1sim import table_scene

    low, high = table_scene.SCENES["volleyball"]["reach_m"]
    pelvis = env.model.body("pelvis").id
    for _ in range(30):
        env.reset(randomize=True)
        distance = np.linalg.norm(env.object_position("volleyball")[:2] - env.data.xpos[pelvis][:2])
        assert low <= distance <= high
    env.reset()


def test_unknown_scene_override_is_refused(tmp_path):
    import json

    from g1sim import randomization

    for scenes, word in (({"tennis": {}}, "tennis"), ({"volleyball": {"objects": {"enabled": False}}}, "enabled"),
                         ({"volleyball": {"objects": {"radius": 1}}}, "radius")):
        config = json.loads(randomization.CONFIG.read_text())
        config["scenes"] = scenes
        path = tmp_path / "randomization.json"
        path.write_text(json.dumps(config))
        with pytest.raises(ValueError, match=word):
            randomization.load(path)


def test_default_write_is_the_bottle_scene():
    """write() without a scene writes the bottle scene, unchanged: same XML as build_spec of the bottle scene."""
    from g1sim import SIM_DIR, build, table_scene

    table_scene.write()
    spec = table_scene.build_spec(build.snapshot(), "bottle")
    spec.compile()
    assert (SIM_DIR / table_scene.SCENE_FILE).read_text() == spec.to_xml()
    model = mujoco.MjModel.from_xml_path(str(SIM_DIR / table_scene.SCENE_FILE))
    names = {model.body(b).name for b in range(model.nbody)}
    assert set(table_scene.OBJECTS) <= names
    assert not names & {"volleyball", "volleyball_stand", "basket"}
    assert model.body("table").pos[0] == pytest.approx(table_scene.TABLE_CENTER_X)
    assert np.allclose(model.geom("table_top").size[:2], table_scene.TABLE_HALF)
