"""Unit: natural arm swing (g1sim/arm_swing.py) - each arm opposite to its leg, still when standing, within its limits."""
import math

import numpy as np
import pytest

from g1sim import arm_swing

pytestmark = pytest.mark.unit
DT = 0.05


def gait(amplitude=0.09, period=0.75, offset=0.04, seconds=6.0):
    """Foot gap (right - left) of a steady walk: a sine around a constant offset (the feet never stand exactly level)."""
    t = np.arange(0.0, seconds, DT)
    return t, offset + amplitude * np.sin(2 * math.pi * t / period)


def test_arms_swing_opposite_to_the_legs():
    swing = arm_swing.ArmSwing()
    t, gap = gait()
    out = np.array([[o[0][0], o[1][0]] for o in (swing.step(g, DT) for g in gap)])[len(t) // 2:]  # left, right pitch
    g = gap[len(t) // 2:] - gap.mean()
    assert np.corrcoef(g, -out[:, 0])[0, 1] > 0.6  # right foot forward -> left arm forward (pitch negative; smoothed,
    assert np.corrcoef(g, out[:, 1])[0, 1] > 0.6   # so a little behind the feet) ... and right arm back
    assert np.allclose(out[:, 0], -out[:, 1])
    assert np.abs(out).max() <= arm_swing.SwingParams().max_pitch + 1e-9
    assert 0.04 < np.abs(out).max() <= 0.12  # a slight nudge: a few degrees, at most ~7


def test_no_swing_while_standing_even_with_feet_apart():
    swing = arm_swing.ArmSwing()
    for _ in range(200):  # standing, right foot 4 cm ahead (as measured after a walk)
        (lp, le), (rp, re) = swing.step(0.04, DT, walking=False)
    assert max(abs(lp), abs(rp), le, re) < 1e-6
    swing = arm_swing.ArmSwing()
    for _ in range(200):  # told to walk, but the feet stay put: the offset is taken out
        (lp, _), (rp, _) = swing.step(0.04, DT, walking=True)
    assert max(abs(lp), abs(rp)) < 1e-3


def test_no_elbow_bend_and_apply_adds_offsets():
    swing = arm_swing.ArmSwing()
    for _ in range(100):
        swing.step(0.0, DT)
    for _ in range(10):
        (lp, le), (rp, re) = swing.step(0.10, DT)  # right foot forward: left arm forward
    assert lp < 0 < rp and le == 0.0 and re == 0.0  # no elbow bend
    arms = np.zeros(27, dtype=np.float32)
    out = swing.apply(arms, ((lp, le), (rp, re)))
    assert out[0] == pytest.approx(lp) and out[3] == pytest.approx(le) and out[7] == pytest.approx(rp)
    assert np.count_nonzero(out) == 2 and np.count_nonzero(arms) == 0


def test_swing_is_slow_and_subtle():
    """The arm angle changes at most max_rate (0.5 rad/s, ~29 deg/s) even at the fastest walk, not snapping with every step."""
    swing = arm_swing.ArmSwing()
    _, gap = gait(amplitude=0.125)  # the widest steps measured (0.35 m/s)
    pitch = np.array([swing.step(g, DT)[0][0] for g in gap])
    assert np.abs(np.diff(pitch)).max() / DT <= arm_swing.SwingParams().max_rate + 1e-9
