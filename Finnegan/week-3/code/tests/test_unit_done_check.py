"""Unit: the done check of the policy phase (g1sim/done_check.py) on recorded run3 hand sequences and on edge cases."""
import numpy as np
import pytest

from conftest import REPO
from g1sim import done_check as dc

pytestmark = pytest.mark.unit

HANDS = np.load(REPO / "tests" / "data" / "done_check_hands.npz")
READY = np.array([0.1, 0.1, 0.15, 0.15, 0.15, 0.15] * 2)   # BrainCo ready values the teacher starts and ends with
OPEN, GRIP = np.zeros(12), np.array([0.3, 0.3, 0.12, 0.12, 0.12, 0.12] * 2)


def first_done(seq) -> int | None:
    check = dc.DoneCheck()
    return next((i for i, hands in enumerate(seq) if check.update(hands, i)), None)


def frames(*parts) -> np.ndarray:
    """(hands, seconds) parts -> one action per policy tick (dc.FPS)."""
    return np.concatenate([np.tile(h, (round(s * dc.FPS), 1)) for h, s in parts])


def test_recorded_place_is_done_before_the_latch():
    """Proves: on a recorded run3 grasp-and-place (82 PASS, latch at 22.5 s) the check says done after the release, at 20.0 s.
    Catches: thresholds that never fire on the policy's real hand schedule, or fire before the ball is let go."""
    done = first_done(HANDS["placed"])
    assert done is not None and 19.5 <= done / dc.FPS <= 21.0


def test_recorded_put_down_elsewhere_is_done_too():
    """Proves: a release on the table followed by relaxed hands also ends the attempt (the ball is out of the hands)."""
    assert first_done(HANDS["put_on_table"]) is not None


@pytest.mark.parametrize("name", ["never_released", "released_not_relaxed"])
def test_recorded_unfinished_attempts_are_not_done(name):
    """Proves: a policy that still holds the ball (45 s timeout) or never relaxes after opening is not cut off.
    Catches: a check that fires on the grip alone or on an open hand alone."""
    assert first_done(HANDS[name]) is None


def test_needs_grip_then_release_then_relax_in_order():
    """Proves: relaxed hands without a grip and release before them (the ready pose at the start) never end the phase,
    and each stage needs its full duration [mutation: drop a stage or a duration]."""
    assert first_done(frames((READY, 5.0))) is None
    assert first_done(frames((READY, 1), (OPEN, 1), (GRIP, dc.GRIP_S - 0.1), (OPEN, 1), (READY, 3))) is None
    assert first_done(frames((READY, 1), (OPEN, 1), (GRIP, 2), (OPEN, dc.OPEN_S - 0.1), (READY, 3))) is None
    assert first_done(frames((READY, 1), (OPEN, 1), (GRIP, 2), (OPEN, 1), (READY, dc.RELAX_S - 0.1))) is None
    done = first_done(frames((READY, 1), (OPEN, 1), (GRIP, 2), (OPEN, 1), (READY, 3)))
    assert done == round((1 + 1 + 2 + 1 + dc.RELAX_S) * dc.FPS) - 1


def test_a_broken_relax_starts_its_second_over():
    """Proves: the relaxed hands must be held RELAX_S without a break (a reopen for a new grasp resets the count)."""
    seq = frames((READY, 1), (OPEN, 1), (GRIP, 2), (OPEN, 1), (READY, dc.RELAX_S - 0.2), (OPEN, 0.1), (READY, dc.RELAX_S - 0.2))
    assert first_done(seq) is None
