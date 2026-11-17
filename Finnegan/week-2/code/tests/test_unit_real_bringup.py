"""Unit: the verdicts of 84's read-only steps (g1sim/real_bringup.py) - net, probe (PLAN.md 4.1 rule 2), joints, hands, record.

Why these tests matter: the operators decide on these PASS/FAIL lines whether Part C may start. A stale LowState or a
foreign rt/arm_sdk writer that reads as PASS would let 86 start on old joint data or against someone else's commands.
Fakes only (tests/helpers/real_fakes.py): plain dicts and SimpleNamespaces, no DDS, no robot.
"""
import io

import numpy as np
import pytest

pytest.importorskip("unitree_sdk2py")
pytest.importorskip("cv2")
from g1sim import episode, real_bringup as rb  # noqa: E402
from helpers.real_fakes import FakeClock, FakeReader, fake_hand, fake_lowstate  # noqa: E402

pytestmark = pytest.mark.unit

ROBOT_HZ = 500.0           # the robot's LowState rate (PLAN.md 2.5)
FRESH_S, STALE_S = 0.01, 0.2   # LowState ages either side of the 0.1 s limit
HAND_HZ = 100.0            # brainco_hand_service rate (PLAN.md 2.2)


def silent_writers() -> dict:
    return {"rt/arm_sdk": {"samples": 0, "max_weight": None}, "rt/lowcmd": {"samples": 0, "max_weight": None}}


def test_net_passes_with_a_warning_when_pc2_is_silent():
    """Proves: with the laptop IP set but no ping answer from PC2 the net step still PASSes, with a PC2 warning.
    Catches: a ping failure turned into a FAIL (blocks Part B on a firewall), or dropped without a word (R3-6)."""
    result = rb.check_net("en5", {"en5": [rb.ROBOT_LAPTOP_IP]}, pc2_answers=False)

    assert result.result == rb.PASS and rb.PC2_IP in result.reason and "warning" in result.reason


def test_probe_fails_on_stale_lowstate():
    """Proves: a LowState older than 0.1 s FAILs the probe step even at a good rate.
    Catches: a raised or removed staleness limit - 86 would then start on joint angles that are no longer true."""
    result = rb.check_probe("en5", ROBOT_HZ, STALE_S, silent_writers(), label="")

    assert result.result == rb.FAIL and "no fresh rt/lowstate" in result.reason


@pytest.mark.parametrize("label, expected", [("", rb.FAIL), (rb.OWNER_MOTION_LABEL, rb.PASS)])
def test_probe_fails_on_foreign_arm_sdk_samples_unless_owner_motion(label, expected):
    """Proves: samples on rt/arm_sdk FAIL the probe ("never start 86"), except under --label owner_motion.
    Catches: a disabled foreign-writer check - the operators' only read-only warning that someone else steers the arms."""
    writers = silent_writers()
    writers["rt/arm_sdk"] = {"samples": 25, "max_weight": 1.0}

    result = rb.check_probe("en5", ROBOT_HZ, FRESH_S, writers, label)

    assert result.result == expected


def test_probe_warns_on_foreign_hand_commands():
    """Proves: samples on rt/brainco/left/cmd keep the probe PASS but name the topic and the owner's hand tool.
    Catches: an owner hand tool that would fight our hand commands in Part C going unmentioned (R3-10 (c))."""
    writers = {**silent_writers(), "rt/brainco/left/cmd": {"samples": 10, "max_weight": None}}

    result = rb.check_probe("en5", ROBOT_HZ, FRESH_S, writers, label="")

    assert result.result == rb.PASS and "rt/brainco/left/cmd" in result.reason


def test_joints_fail_when_a_motor_is_outside_its_limit():
    """Proves: a left elbow (motor 18) at 2.2 rad, above its 2.0944 limit, FAILs the joints step and names the joint.
    Catches: a limit check that always passes - a robot outside the trained range would reach M0 and Part C."""
    lowstate = fake_lowstate(lambda i: 2.2 if i == 18 else 0.0)

    result = rb.check_joints(lowstate)

    assert result.result == rb.FAIL and "LeftElbow" in result.reason


def test_hand_state_fails_on_nan_values():
    """Proves: a hand that reports NaN FAILs the hand-state step and names the side.
    Catches: a broken hand service passing the step, so Part C would start with NaN in the policy's hand state."""
    states = {"left": (fake_hand([float("nan")] * 6), FRESH_S, HAND_HZ), "right": (fake_hand([0.1] * 6), FRESH_S, HAND_HZ)}

    result = rb.check_hands(states)

    assert result.result == rb.FAIL and "left" in result.reason


def test_record_stops_when_lowstate_goes_stale():
    """Proves: record_observation raises StepFailed as soon as the LowState is older than 0.1 s.
    Catches: a recording that keeps writing the last joint angles after the cable dropped (frozen data in dataset v2)."""
    lowstate = FakeReader(fake_lowstate(lambda i: 0.1), age=STALE_S)
    frames = FakeReader(np.zeros((200, 320, 3), dtype=np.uint8))
    clock = FakeClock()

    with pytest.raises(rb.StepFailed, match="rt/lowstate stopped"):
        rb.record_observation(lowstate, {}, frames, seconds=0.2, clock=clock.now, sleep=clock.sleep)


def test_record_output_reads_back_with_episode_load(tmp_path):
    """Proves: an observe-only recording saves as an episode that episode.load reads and episode.validate accepts.
    Catches: a recording the dataset v2 converter cannot use (wrong layout, NaN hands, uneven timestamps)."""
    lowstate = FakeReader(fake_lowstate(lambda i: 0.1))
    hands = {"left": FakeReader(fake_hand([0.2] * 6)), "right": FakeReader(None, age=float("inf"))}
    frames = FakeReader(np.full((200, 320, 3), 128, dtype=np.uint8))
    clock = FakeClock()

    ep = rb.record_observation(lowstate, hands, frames, seconds=0.2, clock=clock.now, sleep=clock.sleep)
    buffer = io.BytesIO()
    np.savez_compressed(buffer, **ep)
    loaded = episode.load(episode.write_verified(tmp_path / "observe.npz", buffer.getbuffer()))

    assert episode.validate(loaded) == []
    assert len(loaded["timestamp"]) == 6 and str(loaded["source"]) == "real_observe" and not loaded["commanded_by_us"]
    assert not loaded["hands_present"].any()
