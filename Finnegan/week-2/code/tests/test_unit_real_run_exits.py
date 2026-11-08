"""Unit: how a run of 86 ends when a stop signal or a broken chunk arrives (g1sim/real_run.py check_signals,
g1sim/real_run_exits.py, PLAN.md 4.2 rules 10-13).

Every test drives the real runner with the fakes of tests/helpers/fake_real_run.py. No DDS, no robot, seconds.
"""
import numpy as np
import pytest

pytest.importorskip("unitree_sdk2py")

from g1sim import policy_wire  # noqa: E402
from helpers.fake_real_run import (FACTORY, FakeArm, FakeClock, FakeFrames, FakePolicy, demo_actions,  # noqa: E402
                                   factory_record, last_index, make_io, run, sent, signals_at)

pytestmark = pytest.mark.unit

ELBOW_POS = 6                                  # left elbow (motor 18) in a 17-vector
CRASH_GAP = 0.6                                # rad the elbow is bent past the factory pose, as after a crash in M4
WRONG_SHAPE_CHUNK = np.zeros((policy_wire.CHUNK, policy_wire.ACTION_DIM - 1), np.float32)   # a column short: other server


def policy_io(clock, arm, **overrides):
    """make_io for a 3 s `run` whose policy moves both shoulders away from the ready pose."""
    policy = FakePolicy(demo_actions(shoulder_pitch=-0.6), mode="smolvla")
    return make_io(clock, arm, frames=FakeFrames(), policy=policy, **overrides)


def soft_exit_start_s(tmp_path) -> float:
    """Seconds after the start at which an undisturbed 3 s `run` sends its first message after the policy phase."""
    clock = FakeClock()
    result = run("run", policy_io(clock, FakeArm(clock)), tmp_path / "undisturbed", speed=0.5, max_seconds=3.0)
    phases = list(sent(result, "sent_phase"))
    return float(sent(result, "sent_t")[last_index(phases, "policy") + 1])


def test_one_ctrl_c_during_normal_end_keeps_the_slow_way_home(tmp_path):
    """Proves: ONE stop signal while a normal end already walks home changes nothing: the arms still reach q0 and ramp
    out there (exit 0), as the runbook promises ("one Ctrl+C = soft exit").
    Catches: a first Ctrl+C that is treated as a second one and drops the arms from a policy pose over the table edge
    (B2 review r2 #1, the R2-1 case) [mutation]."""
    signal_at = soft_exit_start_s(tmp_path) + 0.5
    clock = FakeClock()
    io = policy_io(clock, FakeArm(clock), signal_count=signals_at(clock, signal_at))

    result = run("run", io, tmp_path / "one_signal", speed=0.5, max_seconds=3.0)

    phases = list(sent(result, "sent_phase"))
    assert (result.exit_code, result.reason) == (0, "done")
    assert "to_start" in phases and phases[-1] == "ramp_out"
    assert np.allclose(sent(result, "sent_q")[last_index(phases, "to_start")], FACTORY, atol=1e-9)
    assert sent(result, "sent_weight")[-1] == 0.0


def test_signal_during_release_holds_without_going_back(tmp_path):
    """Proves: a stop signal halfway through `release` holds the arms where they are (exit 4, ruling F5), the next one
    ramps out there, and the elbow never moves back towards the crash pose it started from.
    Catches: a release that treats the signal as a soft exit, whose way "home" is the measured start = the crash pose
    towards the table (B2 review r2 #3) [mutation]."""
    clock = FakeClock()
    crash = FACTORY.copy()
    crash[ELBOW_POS] += CRASH_GAP
    arm = FakeArm(clock, crash)
    io = make_io(clock, arm, factory_record=factory_record(FACTORY), signal_count=signals_at(clock, 1.0, 2.0))

    result = run("release", io, tmp_path)

    elbow = sent(result, "sent_q")[:, ELBOW_POS]
    assert (result.exit_code, result.reason) == (4, "signal_in_release")
    assert "fault_hold" in sent(result, "sent_phase"), "signal_in_release must hold (ruling F5) until the next signal"
    assert np.diff(elbow).max() <= 1e-9, "the elbow moved back towards the crash pose"
    assert elbow[-1] > FACTORY[ELBOW_POS] + 0.1, "release finished before the signal: the test proves nothing"
    assert sent(result, "sent_weight")[-1] == 0.0


def test_nan_in_a_chunk_gives_fault_exit_before_it_is_sent(tmp_path):
    """Proves: a chunk with NaN ends the run with a fault exit (code 4, invalid_target) and no policy-phase message ever
    left the ready pose (rule 11).
    Catches: a NaN chunk handled as a soft exit (slow walk home with a broken policy) or played to the motors [mutation]."""
    clock = FakeClock()
    actions = demo_actions()
    actions[:, 3] = np.nan                     # left elbow of every action
    io = make_io(clock, FakeArm(clock), frames=FakeFrames(), policy=FakePolicy(actions, mode="smolvla"))

    result = run("run", io, tmp_path, speed=0.5, max_seconds=3.0)

    phases, q = sent(result, "sent_phase"), sent(result, "sent_q")
    assert (result.exit_code, result.reason) == (4, "invalid_target")
    assert np.ptp(q[phases == "policy"], axis=0).max() == 0.0
    assert sent(result, "sent_weight")[-1] == 0.0


def test_malformed_chunk_mid_run_gives_soft_exit_through_the_ready_pose(tmp_path):
    """Proves: a chunk of the wrong shape after a good one ends the run softly (code 3, policy_error) and the arms go
    back up to the ready pose before they go home (rule 10).
    Catches: a malformed reply that becomes internal_error and ramps the arms out at once from the policy pose, over
    the table edge (B2 review r3 minor 4) [mutation]."""
    clock = FakeClock()
    policy = FakePolicy(demo_actions(shoulder_pitch=-0.6), mode="smolvla")
    good_infer = policy.infer
    policy.infer = lambda obs, tick: good_infer(obs, tick) if tick == 0 else {"actions": WRONG_SHAPE_CHUNK}
    io = make_io(clock, FakeArm(clock), frames=FakeFrames(), policy=policy)

    result = run("run", io, tmp_path, speed=0.5, max_seconds=10.0)

    phases = list(sent(result, "sent_phase"))
    assert (result.exit_code, result.reason) == (3, "policy_error")
    assert phases[last_index(phases, "policy") + 1] == "to_ready"
    assert sent(result, "sent_weight")[-1] == 0.0
