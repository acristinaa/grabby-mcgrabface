"""Integration: openpi-Anbindung - Konfiguration, Ersatz-Policy durch openpis Policy-Klasse, Datensatz v2.1 + Datenlader.

Laeuft nur in der openpi-Umgebung (~/dev/g1-ext/openpi/.venv), sonst uebersprungen.
Beim ersten Lauf laedt openpi den PaliGemma-Tokenizer (~4 MB, gs://big_vision) nach ~/.cache/openpi.
Datensaetze entstehen in einem temporaeren Ordner, nicht unter ~/.cache/huggingface/lerobot.
"""
import dataclasses
import json
import pathlib

import numpy as np
import pytest

pytest.importorskip("openpi")

from openpi.shared import normalize  # noqa: E402
from openpi.training import config as _config  # noqa: E402
from openpi.training import data_loader  # noqa: E402

from g1sim import action_space, episode  # noqa: E402
from openpi_g1 import config, convert, policy, run, stand_in  # noqa: E402

pytestmark = pytest.mark.integration

TASK = "pick up the red bottle"


@pytest.fixture(scope="module")
def train_config():
    return config.register()


@pytest.fixture
def lerobot_home(tmp_path, monkeypatch):
    import lerobot.common.datasets.lerobot_dataset as lerobot_dataset

    home = tmp_path / "lerobot"
    monkeypatch.setattr(lerobot_dataset, "HF_LEROBOT_HOME", home)
    return home


def write_episodes(directory, n=2, frames=40, size=32, success=True, seed=0):
    """Kleine Zufallsepisoden im Rohformat (ohne Sim). Rueckgabe: Pfade, geladene Episoden."""
    rng = np.random.default_rng(seed)
    paths = []
    for i in range(n):
        recorder = episode.EpisodeRecorder(TASK, fps=30)
        for _ in range(frames):
            frame = {key: rng.integers(256, size=(size, size, 3), dtype=np.uint8) for key in episode.IMAGE_KEYS}
            # arms rad, hands 0..1, waist yaw rad
            frame["state"] = np.concatenate([rng.uniform(-1, 1, 14), rng.uniform(0, 1, 12), rng.uniform(-1, 1, 1)])
            actions = np.concatenate([rng.uniform(-1, 1, 14), rng.uniform(0, 1, 12), rng.uniform(-1, 1, 1)]).astype(np.float32)
            recorder.add(frame, actions)
        paths.append(recorder.save(pathlib.Path(directory) / f"episode_{i:03d}.npz", success=success))
    return paths, [episode.load(p) for p in paths]


def test_konfiguration_pi05_g1_brainco_lora(train_config):
    assert _config.get_config(config.CONFIG_NAME) is train_config
    config.register()
    assert sum(c.name == config.CONFIG_NAME for c in _config._CONFIGS) == 1
    model = train_config.model
    assert model.pi05 and model.action_horizon == 30 and model.action_dim >= action_space.DIM
    assert (model.paligemma_variant, model.action_expert_variant) == ("gemma_2b_lora", "gemma_300m_lora")
    assert train_config.ema_decay is None
    assert train_config.weight_loader.params_path.endswith("pi05_base/params")
    data = train_config.data.create(train_config.assets_dirs, train_config.model)
    assert (data.repo_id, data.prompt_from_task, data.use_quantile_norm) == (config.REPO_ID, True, True)


def test_ersatz_policy_liefert_startpose_durch_openpis_umrechnung(train_config):
    p = stand_in.create_policy(train_config)
    actions = p.infer(policy.make_example(seed=1))["actions"]
    assert actions.shape == (30, action_space.DIM)
    np.testing.assert_allclose(actions, np.tile(action_space.READY_POSE, (30, 1)), atol=1e-4)
    assert p.metadata["action_names"] == list(action_space.NAMES) and p.metadata["stand_in"]


def test_ersatz_policy_auch_mit_echter_normierung(train_config):
    """Mit nicht-neutraler Statistik muss die Umkehrung im Ersatz-Modell zu openpis Normalize/Unnormalize passen."""
    rng = np.random.default_rng(2)

    def stats():
        dim = action_space.DIM
        q01, q99 = -rng.uniform(0.5, 2.0, dim), rng.uniform(0.5, 2.0, dim)
        return normalize.NormStats(mean=np.zeros(dim), std=np.ones(dim), q01=q01.astype(np.float32), q99=q99.astype(np.float32))

    p = stand_in.create_policy(train_config, norm_stats={"state": stats(), "actions": stats()})
    actions = p.infer(policy.make_example(seed=3))["actions"]
    np.testing.assert_allclose(actions, np.tile(action_space.READY_POSE, (30, 1)), atol=1e-4)


def test_datensatz_v21_und_openpis_datenlader(train_config, tmp_path, lerobot_home):
    paths, eps = write_episodes(tmp_path)
    summary = convert.convert(paths, repo_id="test/g1_brainco")
    assert (summary["episodes"], summary["frames"], summary["skipped"]) == (2, 80, {})
    info = json.loads((lerobot_home / "test/g1_brainco/meta/info.json").read_text())
    assert info["codebase_version"] == "v2.1"

    data_config = dataclasses.replace(train_config.data, repo_id="test/g1_brainco").create(
        train_config.assets_dirs, train_config.model)
    dataset = data_loader.create_torch_dataset(data_config, train_config.model.action_horizon, train_config.model)
    item = data_loader.TransformedDataset(
        dataset, [*data_config.repack_transforms.inputs, *data_config.data_transforms.inputs])[0]
    ep = eps[0]
    for key, slot in policy.CAMERA_SLOTS.items():
        np.testing.assert_array_equal(item["image"][slot], ep[key][0])
    np.testing.assert_allclose(item["state"], ep["state"][0], atol=1e-6)
    assert item["actions"].shape == (30, action_space.DIM)
    np.testing.assert_allclose(item["actions"][:, :14], ep["actions"][:30, :14] - ep["state"][0, :14], atol=1e-5)
    np.testing.assert_allclose(item["actions"][:, episode.HANDS], ep["actions"][:30, episode.HANDS], atol=1e-6)
    np.testing.assert_allclose(item["actions"][:, 26], ep["actions"][:30, 26] - ep["state"][0, 26], atol=1e-5)  # waist delta
    assert item["prompt"] == TASK


def test_fehlgeschlagene_episoden_bleiben_draussen(tmp_path, lerobot_home):
    failed, _ = write_episodes(tmp_path / "fail", n=1, success=False)
    good, _ = write_episodes(tmp_path / "ok", n=1)
    summary = convert.convert([*failed, *good], repo_id="test/nur_erfolg")
    assert summary["episodes"] == 1 and list(summary["skipped"]) == [str(failed[0])]


def test_norm_stats_ueber_run(train_config, tmp_path, lerobot_home, monkeypatch):
    paths, _ = write_episodes(tmp_path)
    convert.convert(paths, repo_id="test/norm")
    small = dataclasses.replace(train_config, data=dataclasses.replace(train_config.data, repo_id="test/norm"),
                                assets_base_dir=str(tmp_path / "assets"), num_workers=0, batch_size=8)
    monkeypatch.setitem(_config._CONFIGS_DICT, config.CONFIG_NAME, small)
    run.norm_stats()
    stats = normalize.load(small.assets_dirs / "test/norm")
    assert stats["state"].q01.shape == (action_space.DIM,) and stats["actions"].q99.shape == (action_space.DIM,)
