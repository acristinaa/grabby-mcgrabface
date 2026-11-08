"""Integration: closed-loop eval (g1sim/vla_eval.py) in the walk world, against the replay server (scripts/81_smolvla_server.py).

(a) the no-walk handover is 69's sequence (reset, snapshot, first observation), (b) bit-exact against a recorded T7 demo (skipped
without the T7; read-only), (c) a demo recorded by 69 into tmp_path, served by 81 --replay in a subprocess (sim env, no torch),
run through the wire passes the replay gate against vla_eval.reference_replay; the teacher on the same handover passes both its
own and the event verdict, (d) a client that sends the stale tick-0 observation at every request fails the gate.
Never writes to the T7. Writes the scene XML (table_scene.write): do not run it while 67/69 record.
"""
import importlib.util
import json
import os
import pathlib
import queue
import re
import subprocess
import sys
import threading
import time

import numpy as np
import pytest

from conftest import REPO, needs_g1ext

pytestmark = [pytest.mark.integration, needs_g1ext]

SCENE = "volleyball_walk"
TIMEOUT_TICKS = 45 * 30
T7_DEMO = pathlib.Path("/Volumes/T7/Robot Project SAP/DATA Collection/g1-demos/level1_walkup_v1/episode_s101_t0000.npz")


@pytest.fixture(scope="module")
def scene():
    from g1sim import MANIFEST, SIM_DIR, table_scene

    if not (SIM_DIR / MANIFEST).is_file():
        pytest.skip("walking model missing - run scripts/51_build_brainco_sim.py first")
    table_scene.write(scene=SCENE)


def make_env(seed=0, config=None):
    """config: randomization config path (None = configs/randomization.json)."""
    from g1sim import randomization, table_env

    return table_env.TableEnv(scene=SCENE, seed=seed, variation_config=randomization.load(config or randomization.CONFIG))


def plain(value):
    return json.dumps(value, sort_keys=True, default=lambda v: v.tolist() if hasattr(v, "tolist") else str(v))


def test_no_walk_handovers_match_69s_sequence(scene):
    from g1sim import ball_grasp, table_scene, vla_eval

    by_hand, env = make_env(100), make_env(100)
    try:
        expected = []
        for _ in range(2):  # 69:214, :127, :130
            by_hand.reset(randomize=True)
            snapshot = by_hand.snapshot_handover()
            expected.append((snapshot, by_hand.observation()))
        got = list(vla_eval.no_walk_handovers(env, 2))
        assert [h["trial"] for h in got] == [0, 1] and all(h["error"] is None for h in got)
        for (snapshot, obs), h in zip(expected, got):
            assert np.array_equal(snapshot["state"], h["snapshot"]["state"])
            assert plain(snapshot["variation"]) == plain(h["snapshot"]["variation"])
            assert np.array_equal(obs["head_image"], h["obs"]["head_image"])
            assert np.array_equal(obs["state"], h["obs"]["state"])
        # the module's copies of sim constants (it imports without MuJoCo)
        assert vla_eval.MIN_LIFT == ball_grasp.MIN_LIFT
        assert vla_eval.FOOTPRINT_HALF == table_scene.BASKET_INNER_HALF + table_scene.BASKET_WALL
        assert tuple(ball_grasp.placement_checks(env)) == vla_eval.PLACEMENT_CHECKS
    finally:
        by_hand.close()
        env.close()


@pytest.mark.skipif(not T7_DEMO.is_file(), reason="T7 not mounted")
def test_no_walk_seed_101_reproduces_the_recorded_handover(scene):
    from g1sim import vla_eval

    env = make_env(101, REPO / "configs" / "randomization_v1.json")  # level1_walkup_v1 was recorded with the v1 settings
    try:
        h = next(vla_eval.no_walk_handovers(env, 1))
    finally:
        env.close()
    with np.load(T7_DEMO, allow_pickle=False) as f:
        recorded = f["handover_state"]
    assert np.array_equal(h["snapshot"]["state"], recorded)


@pytest.fixture(scope="module")
def recorded(scene, tmp_path_factory):
    """One demo recorded by 69's own record() into tmp_path (never the T7). Returns (69 module, npz path)."""
    spec = importlib.util.spec_from_file_location("sixty_nine", REPO / "scripts" / "69_volleyball_demo_factory.py")
    sixty_nine = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sixty_nine)
    out = tmp_path_factory.mktemp("demos")
    env = make_env(100)
    try:
        for trial in range(3):
            env.reset(randomize=True)
            entry = sixty_nine.record(env, 100, trial, out)
            if entry["result"] == "saved":
                return sixty_nine, pathlib.Path(entry["path"])
    finally:
        env.close()
    pytest.fail(f"69 saved no demo in 3 trials: {entry}")


@pytest.fixture(scope="module")
def reference(recorded):
    from g1sim import episode, vla_eval

    ep = episode.load(recorded[1])
    return ep, vla_eval.reference_replay(ep, TIMEOUT_TICKS)


@pytest.fixture(scope="module")
def replay_server(recorded):
    """81 --replay <tmp npz> --port 0 --verbose with this (sim env) python; output lines in a queue. Terminated at the end."""
    proc = subprocess.Popen([sys.executable, str(REPO / "scripts" / "81_smolvla_server.py"), "--replay", str(recorded[1]),
                             "--port", "0", "--verbose"], cwd=REPO, env=dict(os.environ, PYTHONUNBUFFERED="1"),
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    lines = queue.Queue()

    def read():
        for line in proc.stdout:
            lines.put(line)
        lines.put(None)

    threading.Thread(target=read, daemon=True).start()
    try:
        log, port, deadline = [], None, time.monotonic() + 60
        while port is None:
            line = lines.get(timeout=max(0.1, deadline - time.monotonic()))
            if line is None:
                pytest.fail("81 exited before LISTENING:\n" + "".join(log))
            log.append(line)
            match = re.search(r"LISTENING (\d+)", line)
            port = int(match.group(1)) if match else None
        yield {"port": port, "lines": lines, "log": log}
    finally:
        proc.terminate()
        try:
            proc.wait(10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()


def wire_trial(ep, stem, port, wrap=None):
    """One trial from ep's handover through the wire (PolicyClient, optionally wrapped). Returns the row."""
    from g1sim import ball_grasp, policy_wire, vla_eval

    client = policy_wire.PolicyClient("127.0.0.1", port, timeout_s=60, connect_wait_s=10)
    env = make_env()
    try:
        assert client.metadata["mode"] == "replay"
        obs = vla_eval.restore(env, ep)
        tracker = vla_eval.EventTracker(env.object_position(ball_grasp.BALL), timeout_ticks=TIMEOUT_TICKS)
        return vla_eval.run_policy_trial(env, wrap(client) if wrap else client, policy_wire.TASK, tracker, 25, stem,
                                         policy_wire.noise_seed(stem), recorded_states=ep["state"], first_obs=obs)
    finally:
        env.close()
        client.close()


def server_line(server, text, timeout=15):
    """Wait for the next server output line containing text."""
    deadline = time.monotonic() + timeout
    while True:
        line = server["lines"].get(timeout=max(0.1, deadline - time.monotonic()))
        if line is None:
            pytest.fail("81 exited:\n" + "".join(server["log"]))
        server["log"].append(line)
        if text in line:
            return line


def test_replay_through_the_wire_passes_the_gate(recorded, reference, replay_server):
    from g1sim import policy_wire, vla_eval

    sixty_nine, npz = recorded
    ep, ref = reference
    assert sixty_nine.TASKS[0] == policy_wire.TASK and ep["task"] == policy_wire.TASK
    assert ref["passed"] and ref["latch_tick"] is not None
    row = wire_trial(ep, npz.stem, replay_server["port"])
    assert vla_eval.replay_gate_problems(row, ref) == [], row
    assert row["replay_state_error"] == 0.0 and row["hand_clips"] == 0
    assert "torch loaded: False" in server_line(replay_server, "torch loaded")


def test_teacher_on_the_recorded_handover(reference):
    from g1sim import ball_grasp, vla_eval

    ep, ref = reference
    env = make_env()
    try:
        vla_eval.restore(env, ep)
        tracker = vla_eval.EventTracker(env.object_position(ball_grasp.BALL), timeout_ticks=TIMEOUT_TICKS)
        row = vla_eval.run_teacher_trial(env, tracker)
    finally:
        env.close()
    assert row["teacher"]["passed"] and row["passed"] and row["agree"]
    assert row["latch_tick"] == ref["latch_tick"]  # the teacher re-plays the recorded actions up to the latch


class StaleClient:
    """Test-only client that sends the tick-0 observation at every request (a broken client, F2c)."""

    def __init__(self, client):
        self.client, self.first = client, None

    def reset(self, episode=None, seed=0):
        self.first = None
        self.client.reset(episode=episode, seed=seed)

    def infer(self, obs, tick):
        self.first = self.first or obs
        return self.client.infer(self.first, tick)


def test_stale_observation_fails_the_gate(recorded, reference, replay_server):
    from g1sim import vla_eval

    ep, ref = reference
    row = wire_trial(ep, recorded[1].stem, replay_server["port"], wrap=StaleClient)
    problems = vla_eval.replay_gate_problems(row, ref)
    fields = {p.split(":")[0] for p in problems}
    assert "obs_state_err_max" in fields and "obs_image_mad_max" in fields, problems
    assert row["passed"] and row["latch_tick"] == ref["latch_tick"]  # the replay server ignores the observation
