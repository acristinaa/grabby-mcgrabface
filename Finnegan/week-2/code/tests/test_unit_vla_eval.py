"""Unit: closed-loop eval logic (g1sim/vla_eval.py) - Wilson intervals, event tracker, stall and no_progress diagnostics, chunk
schedule, hand clipping, demo selection, plan E verdict and the replay gate.

numpy only: no MuJoCo and no g1sim/policy_wire.py (vla_eval never imports it).
"""
import numpy as np
import pytest

from g1sim import vla_eval

pytestmark = pytest.mark.unit

REST = np.array([0.3, 0.2, 0.9])
BASKET = np.array([0.3, -0.25, 0.8])
OVER_BASKET = BASKET + [0.0, 0.0, 0.25]
ALL_TRUE = dict.fromkeys(vla_eval.PLACEMENT_CHECKS, True)
ALL_FALSE = dict.fromkeys(vla_eval.PLACEMENT_CHECKS, False)


def feed(tracker, steps):
    """steps: list of (ball_xyz, placement, forbidden_pair) per tick, from tick 1 on; stops when the tracker is done."""
    for i, (ball, placement, forbidden) in enumerate(steps, start=1):
        if tracker.update(i, ball, BASKET, placement, forbidden):
            return i
    return None


def to_placing(lift_ticks=10, carry_ticks=10):
    """Ball lifted beside the basket, then carried over it (placement false), as tick steps."""
    lifted = REST + [0.0, 0.0, 0.07]
    return [(lifted, ALL_FALSE, None)] * lift_ticks + [(OVER_BASKET, ALL_FALSE, None)] * carry_ticks


@pytest.mark.parametrize("k, n, low, high", [(8, 10, 0.4902, 0.9433), (0, 5, 0.0, 0.4345), (5, 5, 0.5655, 1.0),
                                             (40, 50, 0.6696, 0.8876)])
def test_wilson(k, n, low, high):
    assert vla_eval.wilson(k, n) == pytest.approx((low, high), abs=1e-4)


def test_wilson_empty():
    assert vla_eval.wilson(0, 0) == (None, None)


def test_tracker_lifted_and_carried_ticks():
    t = vla_eval.EventTracker(REST, timeout_ticks=1000)
    steps = [(REST + [0.0, 0.0, 0.03], ALL_FALSE, None)] * 4 + to_placing(lift_ticks=6, carry_ticks=3)
    assert feed(t, steps) is None
    assert t.ticks["lifted"] == 5 and t.ticks["carried"] == 11 and t.ticks["placing"] is None
    assert t.phase == "vla_carried" and t.furthest_event == "carried"


def test_lift_just_below_min_lift_is_not_lifted():
    t = vla_eval.EventTracker(REST, timeout_ticks=3)
    feed(t, [(REST + [0.0, 0.0, 0.0599], ALL_FALSE, None)] * 3)
    assert t.done and t.ticks["lifted"] is None and t.reason == "ball_lifted"


def test_lifted_outside_the_footprint_is_not_carried():
    t = vla_eval.EventTracker(REST, timeout_ticks=5)
    beside = OVER_BASKET + [vla_eval.FOOTPRINT_HALF + 0.001, 0.0, 0.0]
    feed(t, [(beside, ALL_FALSE, None)] * 5)
    assert t.ticks["lifted"] == 1 and t.ticks["carried"] is None and t.reason == "ball_carried_to_basket"


def test_latch_exactly_at_the_90th_consecutive_placing_tick():
    t = vla_eval.EventTracker(REST, timeout_ticks=1000)
    pre = to_placing()
    done = feed(t, pre + [(BASKET, ALL_TRUE, None)] * 200)
    assert done == len(pre) + 90 and t.ticks["latched"] == done
    assert t.passed and t.reason is None and t.ticks["placing"] == len(pre) + 1 and t.furthest_event == "latched"
    assert t.result()["time_to_latch_s"] == pytest.approx(done / 30, abs=1e-3)


def test_one_false_tick_at_89_resets_the_streak():
    t = vla_eval.EventTracker(REST, timeout_ticks=1000)
    pre = to_placing()
    one_false = dict(ALL_TRUE, ball_at_rest=False)
    steps = pre + [(BASKET, ALL_TRUE, None)] * 88 + [(BASKET, one_false, None)] + [(BASKET, ALL_TRUE, None)] * 200
    done = feed(t, steps)
    assert done == len(pre) + 89 + 90 and t.passed
    assert t.ticks["placing"] == len(pre) + 1  # the first placing tick, not the start of the latched streak


def test_forbidden_contact_before_the_latch_ends_the_trial():
    t = vla_eval.EventTracker(REST, timeout_ticks=1000)
    pre = to_placing()
    steps = pre + [(BASKET, ALL_TRUE, None)] * 50 + [(BASKET, ALL_TRUE, ("right_wrist_yaw_link", "basket"))]
    steps += [(BASKET, ALL_TRUE, None)] * 100
    done = feed(t, steps)
    assert done == len(pre) + 51 and t.done and not t.passed
    assert t.reason == "no_forbidden_contacts" and t.forbidden == ("right_wrist_yaw_link", "basket")
    assert t.result()["forbidden_pair"] == ["right_wrist_yaw_link", "basket"]
    assert not t.no_progress


def test_timeout_with_all_placement_checks_true_but_streak_60():
    pre = to_placing()
    t = vla_eval.EventTracker(REST, timeout_ticks=len(pre) + 60)
    done = feed(t, pre + [(BASKET, ALL_TRUE, None)] * 200)
    assert done == len(pre) + 60 and t.streak == 60
    assert not t.passed and t.reason == "placement_not_held_3s" and t.furthest_event == "placing"


def test_timeout_after_the_lift_only():
    t = vla_eval.EventTracker(REST, timeout_ticks=30)
    feed(t, [(REST + [0.0, 0.0, 0.08], ALL_FALSE, None)] * 40)
    assert t.done and t.tick == 30 and t.reason == "ball_carried_to_basket" and t.furthest_event == "lifted"


def test_timeout_reason_is_the_first_false_placement_check_of_the_last_tick():
    t = vla_eval.EventTracker(REST, timeout_ticks=25)
    last = dict(ALL_TRUE, ball_on_basket_floor=False, hands_off_ball=False)
    feed(t, to_placing() + [(BASKET, ALL_TRUE, None)] * 4 + [(BASKET, last, None)])
    assert t.reason == "ball_on_basket_floor"


def test_furthest_event_order_and_phases():
    t = vla_eval.EventTracker(REST, timeout_ticks=1000)
    assert t.furthest_event == "handover" and t.phase == "vla_start"
    t.update(1, REST + [0.0, 0.0, 0.07], BASKET, ALL_FALSE)
    assert t.furthest_event == "lifted" and t.phase == "vla_lifted"
    t.update(2, OVER_BASKET, BASKET, ALL_FALSE)
    assert t.furthest_event == "carried" and t.phase == "vla_carried"
    t.update(3, BASKET, BASKET, ALL_TRUE)
    assert t.furthest_event == "placing" and t.phase == "vla_placing"
    t.update(4, BASKET, BASKET, dict(ALL_TRUE, ball_at_rest=False))
    assert t.furthest_event == "placing" and t.phase == "vla_carried"  # the streak broke; placing was reached
    assert list(vla_eval.EVENTS) == ["handover", "lifted", "carried", "placing", "latched"]


def test_update_after_done_raises():
    t = vla_eval.EventTracker(REST, timeout_ticks=1)
    t.update(1, REST, BASKET, ALL_FALSE)
    with pytest.raises(RuntimeError):
        t.update(2, REST, BASKET, ALL_FALSE)


def actions_with_window(range_in_window, n=300, start=100):
    """Executed actions that move 0.05 per tick everywhere except one 60-tick window with the given range."""
    ramp = np.arange(n, dtype=float)[:, None] * 0.05 * np.ones((1, 27))
    flat = np.linspace(0.0, range_in_window, vla_eval.STALL_WINDOW)[:, None] * np.ones((1, 27))
    ramp[start:start + vla_eval.STALL_WINDOW] = ramp[start] + flat
    return ramp


def test_stall_range_below_threshold_in_a_failed_trial():
    assert vla_eval.stalled(actions_with_window(0.009), passed=False) == (True, 100)


def test_stall_range_above_threshold_is_not_a_stall():
    assert vla_eval.stalled(actions_with_window(0.011), passed=False) == (False, None)


def test_success_with_a_static_hold_is_never_stalled():
    actions = np.zeros((200, 27))
    assert vla_eval.stalled(actions, passed=True) == (False, None)
    assert vla_eval.stalled(actions, passed=False) == (True, 0)


def test_no_progress_ball_still_for_150_ticks_before_the_timeout():
    t = vla_eval.EventTracker(REST, timeout_ticks=300)
    lifted = REST + [0.0, 0.0, 0.07]
    feed(t, [(lifted, ALL_FALSE, None)] * 300)  # lifted at tick 1, then still
    assert t.done and t.reason == "ball_carried_to_basket" and t.no_progress


def test_no_progress_false_when_the_ball_moves_3_cm():
    t = vla_eval.EventTracker(REST, timeout_ticks=300)
    lifted = REST + [0.0, 0.0, 0.07]
    moving = [(lifted + [0.03 * max(0, i - 150) / 149, 0.0, 0.0], ALL_FALSE, None) for i in range(300)]  # last 150 ticks
    feed(t, moving)
    assert t.done and not t.no_progress


def test_no_progress_false_with_a_recent_event():
    t = vla_eval.EventTracker(REST, timeout_ticks=300)
    feed(t, [(REST, ALL_FALSE, None)] * 200 + [(REST + [0.0, 0.0, 0.07], ALL_FALSE, None)] * 100)
    assert t.ticks["lifted"] == 201 and not t.no_progress  # lifted 99 ticks before the timeout


def test_request_ticks_and_exec_steps_bounds():
    assert vla_eval.request_ticks(100, 25) == [0, 25, 50, 75]
    assert vla_eval.request_ticks(101, 25) == [0, 25, 50, 75, 100]
    for bad in (0, 51, -1, 2.0, True):
        with pytest.raises(ValueError):
            vla_eval.request_ticks(100, bad)


def test_clip_hands_counts_and_keeps_in_range_values_bit_identical():
    rng = np.random.default_rng(0)
    chunk = rng.uniform(-2.0, 2.0, (50, 27)).astype(np.float32)
    chunk[:, vla_eval.HANDS] = rng.uniform(0.0, 1.0, (50, 12)).astype(np.float32)
    chunk[3, 14], chunk[7, 25], chunk[9, 20] = -0.2, 1.3, 1.0
    clipped, count = vla_eval.clip_hands(chunk)
    assert count == 2 and clipped.dtype == np.float32
    assert clipped[3, 14] == 0.0 and clipped[7, 25] == 1.0 and clipped[9, 20] == 1.0
    keep = np.ones(chunk.shape, bool)
    keep[3, 14] = keep[7, 25] = False
    assert np.array_equal(clipped[keep], chunk[keep])  # arms, waist and in-range hands untouched
    assert chunk[3, 14] == np.float32(-0.2)  # input not modified
    assert vla_eval.clip_hands(clipped)[1] == 0


def test_pick_evenly():
    paths = [f"episode_{i:03d}.npz" for i in range(300)]
    assert vla_eval.pick_evenly(paths, 5) == [paths[i] for i in (0, 75, 150, 224, 299)]
    assert vla_eval.pick_evenly(paths[:5], 5) == paths[:5]
    assert vla_eval.pick_evenly(paths[:3], 10) == paths[:3]
    assert vla_eval.pick_evenly(paths, 1) == [paths[0]]
    assert vla_eval.pick_evenly(paths[:10], 4) == [paths[i] for i in (0, 3, 6, 9)]


def test_list_demos_skips_decoys(tmp_path):
    for name in ("episode_s1_t0001.npz", "episode_s1_t0000.npz", "._episode_s1_t0000.npz", "episode_s1_t0002.npz.123.part",
                 "factory_s1.json"):
        (tmp_path / name).write_bytes(b"x")
    assert [p.name for p in vla_eval.list_demos(tmp_path)] == ["episode_s1_t0000.npz", "episode_s1_t0001.npz"]
    assert vla_eval.list_demos(tmp_path / "episode_s1_t0001.npz") == [tmp_path / "episode_s1_t0001.npz"]
    with pytest.raises(FileNotFoundError):
        vla_eval.list_demos(tmp_path / "missing")


def section(rate):
    k = int(round(rate * 100))
    return {"vla_success": {"rate": k / 100, "ci95": list(vla_eval.wilson(k, 100))}}


@pytest.mark.parametrize("train, heldout, expected", [(0.5, 0.9, "undertrained"), (0.9, 0.5, "generalization gap"),
                                                      (0.8, 0.7, "consistent"), (0.8, 0.6, "consistent"),
                                                      (0.8, 0.59, "generalization gap"), (0.6, 0.6, "consistent")])
def test_verdict(train, heldout, expected):
    out = vla_eval.verdict(section(train), section(heldout))
    assert out["verdict"].startswith(expected)
    assert out["train"] == train and len(out["train_ci95"]) == 2 and len(out["heldout_ci95"]) == 2


def test_verdict_without_data():
    empty = {"vla_success": {"rate": None, "ci95": [None, None]}}
    assert vla_eval.verdict(empty, section(0.5))["verdict"].startswith("no verdict")


REFERENCE = {"events": {"handover": 0, "lifted": 355, "carried": 476, "placing": 638, "latched": 727}, "passed": True,
             "reason": None, "latch_tick": 727, "frames": 726}


def passing_row():
    return {"id": "episode_s100_t0000", "passed": True, "reason": None, "events": dict(REFERENCE["events"]), "latch_tick": 727,
            "replay_state_error": 0.0, "obs_state_err_max": 0.0, "obs_image_maxdiff_tick0": 0, "obs_image_mad_max": 3.3,
            "obs_prompt_ok_all": True, "obs_checked": 30, "requests": 30, "exec_steps": 25, "stalled": False,
            "stall_start": None}


def test_replay_gate_passing_row():
    assert vla_eval.replay_gate_problems(passing_row(), REFERENCE) == []


def test_replay_gate_counts_only_requests_with_a_recorded_frame():
    row = dict(passing_row(), latch_tick=754, requests=31, obs_checked=30,
               events=dict(REFERENCE["events"], latched=754))
    reference = dict(REFERENCE, latch_tick=754, events=dict(REFERENCE["events"], latched=754))
    assert vla_eval.replay_gate_problems(row, reference) == []  # request at tick 750 >= T = 726: no recorded frame
    assert vla_eval.replay_gate_problems(dict(row, obs_checked=31), reference)[0].startswith("obs_checked")


@pytest.mark.parametrize("field, value", [("latch_tick", 728), ("obs_state_err_max", 1e-7), ("obs_image_maxdiff_tick0", 3),
                                          ("obs_image_mad_max", 4.1), ("obs_prompt_ok_all", False), ("requests", 31),
                                          ("replay_state_error", 1e-7), ("stalled", True), ("obs_checked", 29),
                                          ("obs_state_err_max", None)])
def test_replay_gate_names_exactly_the_broken_field(field, value):
    problems = vla_eval.replay_gate_problems(dict(passing_row(), **{field: value}), REFERENCE)
    assert len(problems) == 1 and problems[0].startswith(f"{field}:"), problems


def test_replay_gate_event_tick_mismatch():
    row = passing_row()
    row["events"]["carried"] = 477
    problems = vla_eval.replay_gate_problems(row, REFERENCE)
    assert problems == ["events.carried: 477, reference 476"]


def test_summarize_rates_failures_and_gate():
    rows = [dict(passing_row(), reached_handover=True, furthest_event="latched", time_to_latch_s=24.2, hand_clips=1,
                 latency_ms_all={"server": [10.0, 20.0], "round_trip": [12.0, 22.0]}, gate_problems=[]),
            dict(passing_row(), id="b", passed=False, reason="ball_lifted", reached_handover=True, furthest_event="handover",
                 time_to_latch_s=None, hand_clips=2, no_progress=True, gate_problems=["passed: not latched (ball_lifted)"]),
            {"id": "c", "passed": False, "reason": "handover_failed", "reached_handover": False}]
    s = vla_eval.summarize(rows)
    assert (s["trials"], s["reached_handover"], s["passed"]) == (3, 2, 1)
    assert s["vla_success"]["rate"] == 0.5 and s["end_to_end"]["rate"] == pytest.approx(1 / 3)
    assert s["vla_success"]["ci95"] == pytest.approx(list(vla_eval.wilson(1, 2)))
    assert s["failures"] == {"ball_lifted": 1, "handover_failed": 1}
    assert s["furthest_event"]["latched"] == 1 and s["furthest_event"]["handover"] == 1
    assert s["no_progress"] == 1 and s["hand_clips"] == 3 and s["time_to_latch_s_mean"] == 24.2
    assert s["latency_ms"]["server"]["max"] == 20.0 and s["latency_ms"]["round_trip"]["mean"] == 17.0
    assert s["replay_gate"] == {"passed": 1, "trials": 2, "problems": ["b: passed: not latched (ball_lifted)"]}


class ScriptedClient:
    """Test double of policy_wire.PolicyClient: replies with fixed chunks and records the request ticks."""

    def __init__(self, chunks):
        self.chunks, self.ticks, self.resets = list(chunks), [], []

    def reset(self, episode=None, seed=0):
        self.resets.append((episode, seed))

    def infer(self, obs, tick):
        self.ticks.append(tick)
        return {"actions": self.chunks.pop(0), "server_ms": 1.0, "round_trip_ms": 2.0, "checks": {}}


class FakeEnv:
    """Just enough of TableEnv for run_policy_trial with measure() patched out."""

    def __init__(self):
        self.applied, self.renders = [], 0

    def observation(self):
        self.renders += 1
        return {"head_image": np.zeros((200, 320, 3), np.uint8), "state": np.zeros(27, np.float32)}

    def state(self):
        return np.zeros(27, np.float32)

    def apply(self, action):
        self.applied.append(np.array(action))


def test_run_policy_trial_schedule_and_invalid_output(monkeypatch):
    monkeypatch.setattr(vla_eval, "measure", lambda env: {"ball_xyz": REST, "basket_xyz": BASKET, "placement": ALL_FALSE,
                                                          "forbidden_pair": None})
    good = np.full((50, 27), 0.5, np.float32)
    env, client = FakeEnv(), ScriptedClient([good] * 4)
    row = vla_eval.run_policy_trial(env, client, "p", vla_eval.EventTracker(REST, timeout_ticks=100), 25, None, 7,
                                    first_obs=env.observation())
    assert client.ticks == [0, 25, 50, 75] and client.resets == [(None, 7)] and len(env.applied) == 100
    assert env.renders == 4  # first_obs + 3 fresh observations, none at tick 0
    assert row["requests"] == 4 and row["reason"] == "ball_lifted" and row["obs_checked"] is None

    bad = good.copy()
    bad[30, 3] = np.nan
    env, client = FakeEnv(), ScriptedClient([good, bad])
    row = vla_eval.run_policy_trial(env, client, "p", vla_eval.EventTracker(REST, timeout_ticks=100), 25, None, 0)
    assert row["reason"] == "policy_output_invalid" and len(env.applied) == 25 and not row["passed"]

    with pytest.raises(vla_eval.ContractError):
        vla_eval.run_policy_trial(FakeEnv(), ScriptedClient([good[:, :26]]), "p", vla_eval.EventTracker(REST), 25, None, 0)


def test_placing_without_a_lift_never_latches():
    # Review level1-step4-code F4: a ball pushed or rolled into the basket without the 6 cm lift is not a success.
    t = vla_eval.EventTracker(REST, timeout_ticks=200)
    done = feed(t, [(BASKET, ALL_TRUE, None)] * 300)
    assert done == 200 and not t.passed and t.ticks["lifted"] is None and t.ticks["latched"] is None
    assert t.reason == "ball_lifted"
