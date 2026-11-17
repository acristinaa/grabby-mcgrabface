"""Integration: openpi-Client-Seite der Greifszene (g1sim/openpi_env.py) - ohne Server, mit lokaler Ersatz-Policy.

Prueft Beobachtungsformat, Ablehnung kaputter Aktionen und eine kurze Episode ueber openpis Runtime + ActionChunkBroker.
Die Server-Seite pruefen tests/test_*openpi*.py in der openpi-Umgebung, die Leitung dazwischen scripts/60_openpi_closed_loop.py.
"""
import numpy as np
import pytest

from conftest import needs_g1ext

pytest.importorskip("openpi_client")

pytestmark = [pytest.mark.integration, needs_g1ext]


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


def test_beobachtung_im_eingangsformat_des_servers(env):
    from g1sim import action_space, episode, openpi_env

    environment = openpi_env.TableEnvironment(env, max_steps=5)
    environment.reset()
    obs = environment.get_observation()
    assert set(obs) == {*episode.POLICY_KEYS.values(), "prompt"}
    from g1sim import table_env

    for key in episode.IMAGE_KEYS:
        image = obs[episode.POLICY_KEYS[key]]
        assert image.shape == (*table_env.IMAGE_SIZES[key.replace("_image", "_camera")], 3) and image.dtype == np.uint8
    assert obs[episode.POLICY_KEYS["head_image"]].shape == (200, 320, 3)  # Kopfkamera 16:10 wie die D455
    state = obs[episode.POLICY_KEYS["state"]]
    assert state.shape == (action_space.DIM,) and state.dtype == np.float32
    assert np.abs(state[:14] - action_space.READY_POSE[:14]).max() < 0.1


@pytest.mark.parametrize("bad", ["wrong_length", "nan"])
def test_kaputte_aktion_wird_abgelehnt(env, bad):
    from g1sim import action_space, openpi_env

    # NaN with the right length, so the NaN check is what rejects it (not the length check)
    actions = np.zeros(action_space.DIM + 2) if bad == "wrong_length" else np.full(action_space.DIM, np.nan)
    environment = openpi_env.TableEnvironment(env, max_steps=5)
    environment.reset()
    with pytest.raises(ValueError):
        environment.apply_action({"actions": actions})


def test_kurze_episode_ueber_openpi_runtime(env, tmp_path):
    from openpi_client import action_chunk_broker, base_policy
    from openpi_client.runtime import runtime as _runtime
    from openpi_client.runtime.agents import policy_agent

    from g1sim import action_space, episode, openpi_env

    class HoldReadyPose(base_policy.BasePolicy):
        calls = 0

        def infer(self, obs):
            HoldReadyPose.calls += 1
            return {"actions": np.tile(action_space.READY_POSE, (10, 1))}

    environment = openpi_env.TableEnvironment(env, max_steps=25)
    saver = openpi_env.EpisodeSaver(tmp_path)
    agent = policy_agent.PolicyAgent(action_chunk_broker.ActionChunkBroker(HoldReadyPose(), action_horizon=10))
    _runtime.Runtime(environment, agent, [saver]).run()

    ep = episode.load(saver.saved[0])
    assert len(ep["timestamp"]) == 25 and episode.validate(ep) == [] and ep["task"] == openpi_env.DEFAULT_TASK
    assert HoldReadyPose.calls == 3  # 25 Takte, Chunks zu 10
    np.testing.assert_allclose(ep["actions"], np.tile(action_space.READY_POSE, (25, 1)))
