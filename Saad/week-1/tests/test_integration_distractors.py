"""Integration: distractors in the volleyball scene (table_scene.VOLLEYBALL_DISTRACTORS, randomization.sample_distractors).

Guarded: the sampler only puts distractors where the ball teacher cannot hit them (on the table, clear of ball, basket and the
carry arc, apart from each other); TableEnv puts exactly the drawn ones on the table and parks the others out of sight; a
saved variation replays bit-exact; the nominal start has none on the table, so runs without distractors are unchanged.
Needs the built walking model (scripts/51_build_brainco_sim.py); the test writes the scene itself.
"""
import copy

import numpy as np
import pytest

from conftest import needs_g1ext

pytestmark = [pytest.mark.integration, needs_g1ext]

SCENE = "volleyball"
SAMPLES = 300


@pytest.fixture(scope="module")
def env():
    from g1sim import MANIFEST, SIM_DIR, table_env, table_scene

    if not (SIM_DIR / MANIFEST).is_file():
        pytest.skip("walking model missing - run scripts/51_build_brainco_sim.py first")
    table_scene.write(scene=SCENE)
    e = table_env.TableEnv(scene=SCENE, seed=3)
    yield e
    e.close()


@pytest.fixture(scope="module")
def config():
    from g1sim import randomization

    return randomization.load()["distractors"]


def test_draws_1_to_3_valid_placements(config):
    from g1sim import randomization, table_scene

    centre_x, (half_x, half_y) = table_scene.SCENES[SCENE]["table"]
    rng = np.random.default_rng(0)
    counts = set()
    for _ in range(SAMPLES):
        placed = randomization.sample_distractors(config, rng, SCENE)
        counts.add(len(placed))
        assert set(placed) <= set(table_scene.VOLLEYBALL_DISTRACTORS)
        xy = {name: np.array([p["x_m"], p["y_m"]]) for name, p in placed.items()}
        for name, p in xy.items():
            r = randomization.footprint(name)
            assert abs(p[0] - centre_x) <= half_x - r - config["table_margin_m"] + 1e-9
            assert abs(p[1]) <= half_y - r - config["table_margin_m"] + 1e-9
            assert randomization._clear_of_volleyball_setup(p, r + config["keepout_m"])
            assert np.linalg.norm(p) >= table_scene.DISTRACTOR_MIN_REACH + r  # outside the carry arc
        names = list(xy)
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                gap = randomization.footprint(a) + randomization.footprint(b) + config["min_gap_m"]
                assert np.linalg.norm(xy[a] - xy[b]) >= gap - 1e-9
    assert counts == {1, 2, 3}


def test_same_seed_same_distractors_and_switch_off(config):
    from g1sim import randomization

    draw = lambda seed: randomization.sample_distractors(config, np.random.default_rng(seed), SCENE)  # noqa: E731
    assert draw(7) == draw(7)
    assert randomization.sample_distractors(dict(config, enabled=False), np.random.default_rng(0), SCENE) == {}
    assert randomization.sample_distractors(config, np.random.default_rng(0), "bottle") == {}  # no pool there


def test_nominal_start_has_no_distractor_on_the_table(env):
    from g1sim import table_scene

    env.reset()
    assert env.variation["distractors"] == {}
    for name in table_scene.VOLLEYBALL_DISTRACTORS:
        assert env.object_position(name)[0] < -5.0  # parked far behind the robot


def test_drawn_distractors_stand_still_on_the_table(env):
    from g1sim import table_env, table_scene

    for _ in range(5):
        env.reset(randomize=True)
        on_table = env.variation["distractors"]
        start = {name: env.object_position(name) for name in table_scene.VOLLEYBALL_DISTRACTORS}
        for _ in range(table_env.CONTROL_HZ):  # 1 s
            env.apply(env.ready_pose)
        for name in table_scene.VOLLEYBALL_DISTRACTORS:
            pos = env.object_position(name)
            assert np.linalg.norm(pos - start[name]) < 0.002
            if name in on_table:
                assert pos[:2] == pytest.approx([on_table[name]["x_m"], on_table[name]["y_m"]], abs=0.002)
                assert pos[2] == pytest.approx(table_scene.object_rest_height(name), abs=0.003)
                assert env.object_tilt(name) < 1.0
            else:
                assert pos[0] < -5.0
    env.reset()


def test_distractors_are_scene_parts_not_robot(env):
    from g1sim import table_scene

    m = env.model
    for name in table_scene.VOLLEYBALL_DISTRACTORS:
        body = m.body(name).id
        assert env.scene_bodies[body] == name
        assert all(m.geom_bodyid[g] != body for g in env.robot_geoms)


def test_saved_variation_replays_bit_exact(env):
    env.reset(randomize=True)
    v, qpos = copy.deepcopy(env.variation), env.data.qpos.copy()
    env.reset(randomize=True)
    env.reset(variation=v)
    assert env.variation == v
    assert np.array_equal(env.data.qpos, qpos)
    env.reset()
