"""Unit: wire protocol, replay predictor with observation checks, run1 guards (g1sim/policy_wire.py, Level 1 Step 4).

numpy only: no MuJoCo, no torch. Server and client run in threads on localhost.
"""
import json
import os
import pathlib
import socket
import struct
import threading
import time

import numpy as np
import pytest

from g1sim import episode, policy_wire as w

pytestmark = pytest.mark.unit

RUN1_LINE = ("32059 /opt/homebrew/Cellar/python@3.12/3.12.14/Frameworks/Python.framework/Versions/3.12/Resources/Python.app/"
             "Contents/MacOS/Python /Users/finneganadam/dev/g1-ext/smolvla-test/.venv/bin/lerobot-train "
             "--policy.path=lerobot/smolvla_base --policy.device=mps --policy.push_to_hub=false "
             "--policy.scheduler_decay_steps=45000 --dataset.repo_id=local/g1_volleyball_walkup_v1 "
             "--dataset.root=/Users/finneganadam/dev/g1-ext/datasets/g1_volleyball_walkup_v1 --batch_size=8 --steps=45000 "
             "--save_freq=5000 --log_freq=50 --seed=1000 --wandb.enable=false --job_name=g1_volleyball_run1 "
             "--output_dir=/Users/finneganadam/dev/g1-ext/smolvla-runs/run1")
RUN1 = pathlib.Path("/Users/finneganadam/dev/g1-ext/smolvla-runs/run1")


def valid_obs(rng=None, prompt=w.TASK):
    rng = rng or np.random.default_rng(0)
    return {w.IMAGE_KEY: rng.integers(0, 256, w.IMAGE_SHAPE, dtype=np.uint8),
            w.STATE_KEY: rng.normal(size=w.ACTION_DIM).astype(np.float32), "prompt": prompt}


@pytest.fixture
def pair():
    a, b = socket.socketpair()
    b.settimeout(2)  # a recv that blocks instead of refusing fails the test with a timeout
    yield a, b
    a.close()
    b.close()


def send_header_only(sock, header):
    head = json.dumps(header).encode()
    sock.sendall(struct.pack(">I", len(head)) + head)


# --- framing ---------------------------------------------------------------------------------------------------------------

def test_round_trip_bit_exact(pair):
    a, b = pair
    rng = np.random.default_rng(1)
    image = rng.integers(0, 256, (200, 320, 3), dtype=np.uint8)
    state = rng.normal(size=(50, 27)).astype(np.float32)
    sender = threading.Thread(target=w.send, args=(a, {"type": "x", "tick": 3}, {"image": image, "state": state}))
    sender.start()  # 197 kB do not fit the socket buffer: send and recv must run at the same time
    header, arrays = w.recv(b)
    sender.join(5)
    assert header == {"type": "x", "tick": 3}
    assert arrays["image"].dtype == np.uint8 and arrays["image"].shape == image.shape
    assert arrays["state"].dtype == np.float32 and arrays["state"].shape == state.shape
    assert arrays["image"].tobytes() == image.tobytes() and arrays["state"].tobytes() == state.tobytes()


def test_header_only_message(pair):
    a, b = pair
    w.send(a, {"type": "ok"})
    assert w.recv(b) == ({"type": "ok"}, {})


@pytest.mark.parametrize("value", [np.zeros(3, np.float64), np.array([1, "a"], dtype=object)])
def test_send_refuses_other_dtypes(pair, value):
    with pytest.raises(ValueError):
        w.send(pair[0], {"type": "x"}, {"a": value})


@pytest.mark.parametrize("dtype", ["float64", "object"])
def test_recv_refuses_other_dtypes(pair, dtype):
    a, b = pair
    send_header_only(a, {"type": "x", "arrays": [{"name": "a", "dtype": dtype, "shape": [3]}]})
    with pytest.raises(ValueError):
        w.recv(b)


def test_recv_refuses_long_header_without_reading_it(pair):
    a, b = pair
    a.sendall(struct.pack(">I", w.MAX_HEADER + 1))  # no header bytes follow: reading them would block
    with pytest.raises(ValueError):
        w.recv(b)


@pytest.mark.parametrize("specs", [
    [{"name": "a", "dtype": "uint8", "shape": [-1]}],
    [{"name": "a", "dtype": "uint8", "shape": ["3"]}],
    [{"name": "a", "dtype": "uint8", "shape": [2.0]}],
    [{"name": "a", "dtype": "uint8", "shape": [True]}],
    [{"name": "a", "dtype": "uint8", "shape": [1, 1, 1, 1, 1]}],
    [{"name": f"a{i}", "dtype": "uint8", "shape": [1]} for i in range(9)],
    [{"name": "a", "dtype": "uint8", "shape": [100000, 100000, 3]}],
    [{"name": "a", "dtype": "uint8", "shape": [2 ** 70]}],
    [{"name": "a", "dtype": "uint8", "shape": [2 ** 32, 2 ** 32]}],
], ids=["negative", "string", "float", "bool", "rank5", "9arrays", "too_big", "beyond_int64", "int64_wrap"])
def test_recv_refuses_bad_specs_before_the_body(pair, specs):
    a, b = pair
    send_header_only(a, {"type": "x", "arrays": specs})  # header only: a recv that reads on would block
    with pytest.raises(ValueError):
        w.recv(b)


def test_eof_mid_message(pair):
    a, b = pair
    head = json.dumps({"type": "x", "arrays": [{"name": "a", "dtype": "uint8", "shape": [10]}]}).encode()
    a.sendall(struct.pack(">I", len(head)) + head + b"12345")
    a.shutdown(socket.SHUT_WR)
    with pytest.raises(ConnectionError):
        w.recv(b)


# --- observation, seeds, replay --------------------------------------------------------------------------------------------

def test_check_observation():
    assert w.check_observation(valid_obs()) == []
    bad = [
        {w.IMAGE_KEY: np.zeros((400, 640, 3), np.uint8)},
        {w.IMAGE_KEY: np.zeros((200, 320, 3), np.float32)},
        {w.STATE_KEY: np.zeros(26, np.float32)},
        {w.STATE_KEY: np.full(27, np.nan, np.float32)},
        {"prompt": ""},
    ]
    for change in bad:
        assert len(w.check_observation({**valid_obs(), **change})) == 1, change


def test_noise_seed():
    seed = w.noise_seed("episode_s100_t0000")
    assert seed == w.noise_seed("episode_s100_t0000") and isinstance(seed, int)
    assert seed != w.noise_seed("episode_s100_t0001")


def test_replay_chunk():
    actions = np.arange(60 * 27, dtype=np.float32).reshape(60, 27)
    np.testing.assert_array_equal(w.replay_chunk(actions, 0), actions[:50])
    tail = w.replay_chunk(actions, 50)
    np.testing.assert_array_equal(tail[:10], actions[50:])
    np.testing.assert_array_equal(tail[10:], np.repeat(actions[-1:], 40, axis=0))
    assert w.replay_chunk(actions, 100).shape == (50, 27)
    np.testing.assert_array_equal(w.replay_chunk(actions, 100), np.repeat(actions[-1:], 50, axis=0))
    with pytest.raises(ValueError):
        w.replay_chunk(actions, -1)


def write_episode(path, frames=60, seed=0, task=w.TASK):
    rng = np.random.default_rng(seed)
    rec = episode.EpisodeRecorder(task, 30)
    for _ in range(frames):
        rec.add({"head_image": rng.integers(0, 256, (200, 320, 3), dtype=np.uint8),
                 "state": rng.normal(size=27).astype(np.float32)}, rng.normal(size=27).astype(np.float32))
    return rec.save(path, success=True)


@pytest.fixture
def episodes(tmp_path):
    folder = tmp_path / "demos"
    write_episode(folder / "episode_s100_t0000.npz", seed=0)
    write_episode(folder / "episode_s100_t0001.npz", frames=5, seed=1)
    (folder / "._episode_x.npz").write_bytes(b"\x00\x05\x16\x07")  # macOS twin on exFAT
    (folder / "episode_s100_t0002.npz.123.part").write_bytes(b"half")
    return folder


def test_replay_predictor_lists_only_real_episodes(episodes):
    p = w.ReplayPredictor([episodes])
    assert sorted(p.files) == ["episode_s100_t0000", "episode_s100_t0001"]
    assert p.metadata["mode"] == "replay" and p.metadata["chunk_size"] == 50
    for stem in ("episode_x", None):
        with pytest.raises(ValueError):
            p.reset(stem, 0)
    with pytest.raises(ValueError):
        w.ReplayPredictor([episodes, episodes / "episode_s100_t0000.npz"])  # same stem twice


def test_replay_predictor_checks(episodes):
    ep = episode.load(episodes / "episode_s100_t0000.npz")
    p = w.ReplayPredictor([episodes])
    p.reset("episode_s100_t0000", w.noise_seed("episode_s100_t0000"))
    frame0 = episode.policy_observation({"head_image": ep["head_image"][0], "state": ep["state"][0]}, w.TASK)

    chunk, checks = p.infer(frame0, 0)
    np.testing.assert_array_equal(chunk, ep["actions"][:50])
    assert checks == {"recorded_tick": 0, "obs_state_err": 0.0, "obs_image_maxdiff": 0, "obs_image_mad": 0.0,
                      "obs_prompt_ok": True}

    chunk, checks = p.infer(frame0, 25)  # stale observation: frame 0 sent at tick 25
    np.testing.assert_array_equal(chunk[:35], ep["actions"][25:])
    assert checks["recorded_tick"] == 25 and checks["obs_state_err"] > 0 and checks["obs_image_mad"] > 0

    assert p.infer({**frame0, "prompt": "pick up the ball"}, 0)[1]["obs_prompt_ok"] is False
    chunk, checks = p.infer(frame0, 60)
    assert checks == {"recorded_tick": None, "obs_prompt_ok": True}
    np.testing.assert_array_equal(chunk, np.repeat(ep["actions"][-1:], 50, axis=0))
    json.dumps(checks)  # goes over the wire as JSON


# --- run1 guards -----------------------------------------------------------------------------------------------------------

def test_device_problem():
    assert "lerobot-train is running" in w.device_problem("mps", [RUN1_LINE], False)
    assert w.device_problem("mps", [RUN1_LINE], True) is None
    assert w.device_problem("cpu", [RUN1_LINE], False) is None
    assert w.device_problem("mps", [], False) is None


def test_training_output_dirs():
    assert w.training_output_dirs([RUN1_LINE]) == [RUN1]
    assert w.training_output_dirs([RUN1_LINE.split(" --output_dir")[0]]) is None
    assert w.training_output_dirs([]) == []


def test_checkpoint_problem_decides_from_the_path(monkeypatch):
    def no_stat(*args, **kwargs):
        raise AssertionError("checkpoint_problem touched the filesystem")

    monkeypatch.setattr(os, "stat", no_stat)
    refusal = w.checkpoint_problem(RUN1 / "checkpoints" / "999999" / "pretrained_model", [RUN1_LINE], False, time.time())
    assert "inside a running training's output dir" in refusal
    assert w.checkpoint_problem(RUN1, [RUN1_LINE], False, time.time()) is not None
    assert w.checkpoint_problem(RUN1.parent / "smoke" / "x", [RUN1_LINE], False, time.time()) is None
    assert w.checkpoint_problem(RUN1.parent / "run10" / "x", [RUN1_LINE], False, time.time()) is None  # prefix, not inside
    assert w.checkpoint_problem(RUN1.parent / "smoke", ["123 lerobot-train --steps=5"], False, time.time()) is not None
    assert w.checkpoint_problem(RUN1 / "x", [], False, time.time()) is None  # no training


def test_checkpoint_ready_with_allow(tmp_path):
    out = tmp_path / "run"
    ckpt = out / "checkpoints" / "005000" / "pretrained_model"
    ckpt.mkdir(parents=True)
    line = f"1 lerobot-train --output_dir={out}"
    names = [n for n in w.CHECKPOINT_FILES if n != "model.safetensors"]
    for name in names:
        (ckpt / name).write_text("{}")
    now = time.time()
    for name in names:
        os.utime(ckpt / name, (now - 120, now - 120))
    assert "model.safetensors" in w.checkpoint_problem(ckpt, [line], True, now)
    (ckpt / "model.safetensors").write_bytes(b"x")
    os.utime(ckpt / "model.safetensors", (now - 10, now - 10))
    assert "not ready" in w.checkpoint_problem(ckpt, [line], True, now)
    os.utime(ckpt / "model.safetensors", (now - 120, now - 120))
    assert w.checkpoint_problem(ckpt, [line], True, now) is None
    assert w.checkpoint_ready(ckpt.parent, now) is None  # the step folder works as well
    assert "inside" in w.checkpoint_problem(ckpt, [line], False, now)


# --- server and client -----------------------------------------------------------------------------------------------------

def start_server(predictor, port=0):
    ready = {}
    event = threading.Event()

    def on_listen(line):
        ready["port"] = int(line.split()[1])
        event.set()

    threading.Thread(target=w.serve, args=(predictor, "127.0.0.1", port, on_listen), daemon=True).start()
    assert event.wait(5)
    return ready["port"]


def test_serve_and_client(episodes):
    ep = episode.load(episodes / "episode_s100_t0000.npz")
    port = start_server(w.ReplayPredictor([episodes]))
    client = w.PolicyClient("127.0.0.1", port, timeout_s=10, connect_wait_s=5)
    try:
        assert client.metadata["type"] == "hello" and client.metadata["mode"] == "replay"
        assert client.metadata["image_shape"] == [200, 320, 3] and client.metadata["action_dim"] == 27
        with pytest.raises(w.PolicyServerError):
            client.infer(valid_obs(), 0)  # no reset yet
        client.reset("episode_s100_t0000", w.noise_seed("episode_s100_t0000"))
        obs = episode.policy_observation({"head_image": ep["head_image"][25], "state": ep["state"][25]}, w.TASK)
        reply = client.infer(obs, 25)
        np.testing.assert_array_equal(reply["actions"][:35], ep["actions"][25:])
        assert reply["actions"].dtype == np.float32 and reply["actions"].shape == (50, 27)
        assert reply["checks"]["recorded_tick"] == 25 and reply["checks"]["obs_state_err"] == 0.0
        assert reply["round_trip_ms"] >= reply["server_ms"] >= 0
        with pytest.raises(w.PolicyServerError, match="expected"):
            client.infer({**obs, w.IMAGE_KEY: np.zeros((400, 640, 3), np.uint8)}, 0)
        with pytest.raises(w.PolicyServerError):
            client.reset("episode_unknown", 0)
        assert client.infer(obs, 25)["checks"]["obs_image_maxdiff"] == 0  # the connection stays usable
    finally:
        client.close()


def test_client_waits_for_a_late_server(episodes):
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    result = {}

    def connect():
        client = w.PolicyClient("127.0.0.1", port, timeout_s=10, connect_wait_s=10)
        result["mode"] = client.metadata["mode"]
        client.close()

    thread = threading.Thread(target=connect)
    thread.start()
    time.sleep(0.5)
    start_server(w.ReplayPredictor([episodes]), port)
    thread.join(10)
    assert result == {"mode": "replay"}


class FailingPredictor(w.ReplayPredictor):
    """Raises like a torch model error at tick 99 (review level1-step4-code F2)."""

    def infer(self, obs, tick):
        if tick == 99:
            raise RuntimeError("model blew up")
        return super().infer(obs, tick)


def test_model_runtime_error_gets_an_error_reply_and_the_server_keeps_serving(episodes):
    ep = episode.load(episodes / "episode_s100_t0000.npz")
    port = start_server(FailingPredictor([episodes]))
    client = w.PolicyClient("127.0.0.1", port, timeout_s=10, connect_wait_s=5)
    try:
        client.reset("episode_s100_t0000", w.noise_seed("episode_s100_t0000"))
        obs = episode.policy_observation({"head_image": ep["head_image"][25], "state": ep["state"][25]}, w.TASK)
        with pytest.raises(w.PolicyServerError, match="RuntimeError: model blew up"):
            client.infer(obs, 99)
        assert client.infer(obs, 25)["checks"]["obs_state_err"] == 0.0  # same connection still works
    finally:
        client.close()
