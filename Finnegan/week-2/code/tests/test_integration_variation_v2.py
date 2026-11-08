"""Integration: the v2 variation (plan humming-squishing-treasure A1, A3, 2026-11-01) - latency up to 3 control ticks and
textures per episode. What can break silently: a latency FIFO that applies commands a step early or late (old demos would no
longer replay bit-exactly, new demos would carry a different delay than the config says), and a texture that is not part of
the variation (a replay or a handover restore would show another table).
Needs the built walking model (scripts/51_build_brainco_sim.py); the table scene is written by the test itself.
"""
import copy
import json

import numpy as np
import pytest

from conftest import needs_g1ext

pytestmark = [pytest.mark.integration, needs_g1ext]

TICKS = 8  # long enough for every command of a 2-tick latency to take effect at least 5 times


@pytest.fixture(scope="module")
def env():
    from g1sim import MANIFEST, SIM_DIR, table_env, table_scene

    if not (SIM_DIR / MANIFEST).is_file():
        pytest.skip("walking model missing - run scripts/51_build_brainco_sim.py first")
    table_scene.write()
    e = table_env.TableEnv()
    yield e
    e.close()


def with_latency(env, ticks):
    """The nominal variation with a latency of `ticks` control ticks."""
    from g1sim import randomization, table_env

    v = randomization.nominal(env.objects)
    v["physics"]["latency_s"] = ticks / table_env.CONTROL_HZ
    return v


def commands(env):
    """TICKS different 27-D actions: arms and hands move a little further every tick, so a late or early command shows."""
    return [env.ready_pose + 0.02 * (t + 1) for t in range(TICKS)]


def run(env, variation, actions, step):
    """Reset to variation, then step(env, action) per action. Returns qpos after every tick."""
    env.reset(variation=variation)
    out = []
    for action in actions:
        step(env, action)
        out.append(env.data.qpos.copy())
    return np.array(out)


def apply_before_fifo(env, actions):
    """TableEnv.apply as it was before 2026-11-01 (latency at most one tick): the old targets for the first env.latency physics
    steps, then the new ones. The reference for old demos."""
    from g1sim import action_space, table_env

    actions = np.asarray(actions, dtype=float)
    body = env.body_target.copy()
    for side, q in action_space.arm_targets(actions).items():
        body[env.arm_slot[side]] = q
    body[table_env.WAIST_YAW] = env.waist_yaw_target = float(actions[action_space.SLICES["waist"]][0])
    hand = action_space.hand_targets(actions)
    env._substeps(env.latency)
    env.body_target, env.hand_target = body, hand
    env._substeps(table_env.SUBSTEPS - env.latency)


def test_two_tick_latency_applies_each_command_two_ticks_later(env):
    """Proves: with latency_s = 2 ticks the robot moves exactly as a run without latency that gets the same commands two ticks
    later (holding the start pose for the first two), bit for bit.
    Catches: a FIFO that clamps the latency to one tick (the old limit), drops or reorders commands, or applies them a physics
    step early or late."""
    actions = commands(env)
    late = run(env, with_latency(env, 2), actions, lambda e, a: e.apply(a))
    shifted = run(env, with_latency(env, 0), [env.ready_pose] * 2 + actions[:-2], lambda e, a: e.apply(a))
    assert env.latency == 0 and np.array_equal(late, shifted)
    assert not np.array_equal(late, run(env, with_latency(env, 0), actions, lambda e, a: e.apply(a)))  # the delay is real


@pytest.mark.parametrize("substeps", [0, 5, 10])
def test_latency_up_to_one_tick_is_bit_identical_to_the_old_apply(env, substeps):
    """Proves: for 0, 5 and 10 physics steps (0, half and one tick, the v1 range) the FIFO gives the same qpos bit for bit as
    the apply of before 2026-11-01, so demos recorded with v1 still replay exactly.
    Catches: an off-by-one in the FIFO (a command one physics step early or late), or a latency rounded differently."""
    from g1sim import table_env

    v = with_latency(env, substeps / table_env.SUBSTEPS)
    actions = commands(env)
    new = run(env, v, actions, lambda e, a: e.apply(a))
    assert env.latency == substeps
    assert np.array_equal(new, run(env, v, actions, apply_before_fifo))


def test_texture_choice_survives_a_saved_variation(env):
    """Proves: the textures of a random start are part of env.variation; after a JSON round trip (as in the sidecar and the npz
    handover) reset(variation=...) gives the same materials, and a nominal reset takes them off again.
    Catches: textures drawn outside the variation (a replay would show another table), or geom_matid not restored by
    ModelVariation (a texture would leak into the next episode)."""
    nominal = env.model.geom_matid.copy()
    for _ in range(20):
        env.reset(randomize=True)
        if any(env.variation["image"]["textures"].values()):
            break
    textured = env.model.geom_matid.copy()
    saved = json.loads(json.dumps(copy.deepcopy(env.variation)))
    assert not np.array_equal(textured, nominal)
    env.reset()
    assert np.array_equal(env.model.geom_matid, nominal)
    env.reset(variation=saved)
    assert np.array_equal(env.model.geom_matid, textured)
    for surface, material in saved["image"]["textures"].items():
        expected = env.model.mat(material).id if material else nominal[env.model.geom(surface).id]
        assert env.model.geom_matid[env.model.geom(surface).id] == expected
