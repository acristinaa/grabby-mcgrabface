"""Unit: g1sim/real_timing.py - chunk timing and staleness rules of the 86 runner (PLAN.md 3.3, rules 10 and 14).

No DDS, no robot, no clock: the scheduler is moved on by hand with advance(). One test chunk holds the value j in action j,
so a target value tells which action (or which point between two actions) is playing.
Tests marked [mutation] in PLAN.md were seen failing on a broken copy (runs/real_b1_mutation.log).
"""
import numpy as np
import pytest

from g1sim import real_timing as rt

pytestmark = pytest.mark.unit

COUNTING_CHUNK = np.tile(np.arange(rt.CHUNK_SIZE, dtype=float)[:, None], (1, 27))   # action j holds the value j
# The plan's numbers, written out on purpose (not rt.*): a changed constant must fail these tests
HALF_SPEED_ACTIONS_PER_S = 30 * 0.5                    # rule 14: 30 Hz x --speed 0.5 = 15 actions per wall-clock second


def test_scheduler_starts_chunk_at_the_action_matching_its_latency():
    """Proves: a chunk that arrives 5.5 actions after its request starts playing at action 5.5, not at action 0 (rule 14).
    Catches: a scheduler that replays a late chunk from action 0, so the arm lags behind what the policy saw."""
    sched = rt.ChunkScheduler(speed=1.0)
    requested_at = sched.mark_requested()
    sched.advance(5.5 / rt.ACTION_HZ)                  # 5.5 actions pass while the server thinks

    sched.add_chunk(COUNTING_CHUNK, requested_at)

    assert np.allclose(sched.target(), 5.5)


def test_scheduler_refuses_chunk_that_lies_fully_in_the_past():
    """Proves: once a chunk has run out the target is None, and a chunk 50 actions late raises ChunkTooLate (soft exit).
    Catches: a scheduler that plays a chunk whose every action is already in the past."""
    sched = rt.ChunkScheduler(speed=1.0)
    sched.add_chunk(COUNTING_CHUNK, sched.mark_requested())
    late_request = sched.mark_requested()

    sched.advance(rt.CHUNK_SIZE / rt.ACTION_HZ)

    assert sched.target() is None
    with pytest.raises(rt.ChunkTooLate):
        sched.add_chunk(COUNTING_CHUNK, late_request)


def test_scheduler_interpolates_between_actions():
    """Proves: between two actions the target is their linear interpolation, so 30 Hz actions play smoothly at 50 Hz.
    Catches: a scheduler that steps from action to action, which jerks the arm 30 times a second."""
    sched = rt.ChunkScheduler(speed=1.0)
    sched.add_chunk(COUNTING_CHUNK, sched.mark_requested())

    sched.advance(5.5 / rt.ACTION_HZ)

    assert np.allclose(sched.target(), 5.5)


def test_scheduler_plays_at_speed_times_30_actions_per_second():
    """Proves: at --speed 0.5, one second of 50 Hz ticks plays up to action 15 (30 Hz x 0.5), not action 30 (gate (e)).
    Catches: a scheduler that ignores --speed, so "slow, in the air" (M3) runs the policy at full demo speed."""
    sched = rt.ChunkScheduler(speed=0.5)
    sched.add_chunk(COUNTING_CHUNK, sched.mark_requested())

    for _ in range(50):                                # one second of wall time at the 50 Hz send rate
        sched.advance(1 / 50)

    assert np.allclose(sched.target(), HALF_SPEED_ACTIONS_PER_S)


def test_run_out_timer_counts_wall_seconds_at_speed_0_5():
    """Proves: at --speed 0.5, one wall-clock second after the last action played, seconds_since_ran_out() says 1.0 s.
    Catches: a run-out timer that counts in speed-1.0 seconds, so policy_stale fires at twice the time it should."""
    sched = rt.ChunkScheduler(speed=0.5)
    sched.add_chunk(COUNTING_CHUNK, sched.mark_requested())
    sched.advance((rt.CHUNK_SIZE - 1) / HALF_SPEED_ACTIONS_PER_S)   # the last action (49) plays now

    sched.advance(1.0)

    assert sched.seconds_since_ran_out() == pytest.approx(1.0)


def test_scheduler_requests_after_25_actions_once_until_the_answer():
    """Proves: the next chunk is requested once 25 actions of the current one have played (not at 24.9), and no second request
    goes out while one is pending; the answer clears the pending flag (rule 14).
    Catches: a request on every tick while the server thinks (a flood of infer calls), or a request point moved off 25
    (review R4-F6)."""
    sched = rt.ChunkScheduler(speed=1.0)
    sched.add_chunk(COUNTING_CHUNK, sched.mark_requested())

    sched.advance(24.9 / 30)
    before_25 = sched.needs_request()
    sched.advance(0.1 / 30)                            # exactly 25.0 actions in
    at_25 = sched.needs_request()
    requested_at = sched.mark_requested()
    sched.advance(10 / 30)
    while_pending = sched.needs_request()
    sched.add_chunk(COUNTING_CHUNK, requested_at)
    sched.advance(15 / 30)                             # the new chunk is now 25 actions in (10 + 15)
    after_answer = sched.needs_request()

    assert (before_25, at_25, while_pending, after_answer) == (False, True, False, True)


def test_scheduler_refuses_chunk_of_the_wrong_shape():
    """Proves: a (50, 26) chunk is refused with ValueError when it arrives (rule 14: 50 actions x 27-D).
    Catches: a chunk from another server (e.g. without the waist slot) whose columns would then drive the wrong motors."""
    sched = rt.ChunkScheduler(speed=1.0)

    with pytest.raises(ValueError, match="shape"):
        sched.add_chunk(COUNTING_CHUNK[:, :26], sched.mark_requested())


def test_stale_reason_thresholds():
    """Proves: LowState over 0.1 s, camera and hand state over 1.0 s / 0.5 s (run and shadow), no chunk 1.0 s after run-out
    (run and replay) stop the run.
    Catches: a threshold loosened even slightly (each is probed 0.01 s above its value), so the arm steers on old data."""
    ok = dict(lowstate_age=0.05, camera_age=0.1, hand_age=0.1, ran_out_s=None, mode="run")

    assert rt.stale_reason(**ok) is None
    assert rt.stale_reason(**{**ok, "lowstate_age": 0.11}) == "lowstate_stale"
    assert rt.stale_reason(**{**ok, "camera_age": 1.01}) == "camera_stale"
    assert rt.stale_reason(**{**ok, "camera_age": 1.01, "mode": "shadow"}) == "camera_stale"
    assert rt.stale_reason(**{**ok, "camera_age": 1.01, "mode": "replay"}) is None
    assert rt.stale_reason(**{**ok, "hand_age": 0.51, "mode": "shadow"}) == "hand_state_stale"
    assert rt.stale_reason(**{**ok, "hand_age": 0.51}) == "hand_state_stale"
    assert rt.stale_reason(**{**ok, "ran_out_s": 1.01}) == "policy_stale"
    assert rt.stale_reason(**{**ok, "ran_out_s": 1.01, "mode": "replay"}) == "policy_stale"


# Chunk-timing numbers of PLAN.md 3.3 / rule 14, copied as literals: the tests above check the logic, this one the numbers
TIMING_PLAN_NUMBERS = {
    "CHUNK_SIZE": 50,                          # actions per chunk (policy_wire.CHUNK; rule 14 "d >= 50 -> soft exit")
    "EXEC_STEPS": 25,                          # rule 14: request after 25 actions
    "ACTION_HZ": 30,                           # rule 14: 30 Hz x speed (dataset fps 30)
}


def test_timing_numbers_equal_the_plan():
    """Proves: the chunk size, the request point and the action rate in real_timing equal rule 14 of PLAN.md.
    Catches: a quiet change such as requesting only after the whole chunk (EXEC_STEPS 50), which leaves no time for the
    next answer and ends every run in policy_stale or chunk_too_late."""
    actual = {name: getattr(rt, name) for name in TIMING_PLAN_NUMBERS}

    assert actual == TIMING_PLAN_NUMBERS
