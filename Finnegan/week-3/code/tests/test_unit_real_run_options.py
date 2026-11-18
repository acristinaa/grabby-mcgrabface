"""Unit: the policy phase of 86 honours the operators' options and reports the arm lag (g1sim/real_run_policy.py,
g1sim/real_run.py, PLAN.md 3.4, 4.2 rules 6, 6b, 16 (c), section 7 M5).

Every test drives the real runner with the fakes of tests/helpers/fake_real_run.py. No DDS, no robot, seconds.
"""
import numpy as np
import pytest

pytest.importorskip("unitree_sdk2py")

from g1sim import action_space, real_safety as rs  # noqa: E402
from helpers.fake_real_run import (FACTORY, FakeArm, FakeClock, FakeFrames, FakePolicy, demo_actions,  # noqa: E402
                                   make_io, run, sent)

pytestmark = pytest.mark.unit

READY17 = rs.sdk_targets(action_space.BALL_READY_POSE, FACTORY)
WAIST_YAW = 26                                 # waist yaw in the 27-D action (action_space.SLICES)
START_WAIST = 0.05                             # rad, a factory waist yaw off 0 so "held at q0" differs from "sent 0"
POLICY_LAG = 0.03                              # rad, how far the fake arm trails our command at weight 1
RAMP_SAG_PER_READ = 0.002                      # rad per LowState read while the weight is below 1 (~0.2 rad over the ramp)


class SagThenTrackArm(FakeArm):
    """Sags while the weight is below 1; at weight 1 our gains pull it to within POLICY_LAG of the command."""

    def send(self, weight, q17, kp17, kd17) -> None:
        super().send(weight, q17, kp17, kd17)
        if weight == 1.0:
            self.sag = POLICY_LAG


def policy_waist(result) -> np.ndarray:
    """The waist yaw of every message sent in the policy phase."""
    return sent(result, "sent_q")[sent(result, "sent_phase") == "policy", 0]


def test_shadow_sends_only_the_ready_pose(tmp_path):
    """Proves: in `shadow` the policy is asked, but every message sent after the ready path holds the ready pose (M5).
    Catches: shadow sending the policy's actions, so the arms move in the step everybody believes moves nothing
    (B2 review r2 #3: 0.35 rad) [mutation]."""
    clock = FakeClock()
    policy = FakePolicy(demo_actions(shoulder_pitch=-0.6), mode="smolvla")
    io = make_io(clock, FakeArm(clock), frames=FakeFrames(), policy=policy)

    result = run("shadow", io, tmp_path, max_seconds=3.0)

    phases, q = list(sent(result, "sent_phase")), sent(result, "sent_q")
    shadow = q[np.array(phases) == "shadow"]
    assert policy.ticks, "the policy was never asked: the test proves nothing"
    assert "policy" not in phases and len(shadow) > 0
    assert np.allclose(shadow, READY17, atol=1e-9)


def test_waist_hold_keeps_waist_yaw_at_q0(tmp_path):
    """Proves: with --waist-hold every policy message keeps motor 12 at its measured start angle, although the policy
    asks for 0.1 rad.
    Catches: a runner that ignores --waist-hold, so the waist turns when the operators chose to hold it [mutation]."""
    clock = FakeClock()
    start = FACTORY.copy()
    start[0] = START_WAIST
    actions = demo_actions()
    actions[:, WAIST_YAW] = 0.1
    io = make_io(clock, FakeArm(clock, start), frames=FakeFrames(), policy=FakePolicy(actions, mode="smolvla"))

    result = run("run", io, tmp_path, speed=0.5, max_seconds=3.0, waist_hold=True)

    waist = policy_waist(result)
    assert len(waist) > 0 and np.all(waist == START_WAIST)


def test_waist_max_clamps_policy_waist_yaw(tmp_path):
    """Proves: with --waist-max 0.15 a policy that asks for 0.4 rad waist yaw is held at 0.15 rad (rule 6b).
    Catches: a runner that always uses the 0.6 rad ceiling, so the waist turns 4x further than the checked limit
    whose table-clearance column the operators read (rule 16 (b)) [mutation]."""
    clock = FakeClock()
    actions = demo_actions()
    actions[:, WAIST_YAW] = 0.4
    io = make_io(clock, FakeArm(clock), frames=FakeFrames(), policy=FakePolicy(actions, mode="smolvla"))

    result = run("run", io, tmp_path, speed=0.5, max_seconds=3.0, waist_max=0.15)

    waist = policy_waist(result)
    assert np.abs(waist).max() == pytest.approx(0.15), "the waist never reached the limit: the test proves nothing"


def test_speed_option_sets_the_playback_rate(tmp_path):
    """Proves: --speed 0.5 plays twice as many policy actions as --speed 0.25 in the same time (frames = actions played).
    Catches: a runner that plays at speed 1.0 whatever --speed says (4x faster than the robot default) [mutation]."""
    frames = {}
    for speed in (0.25, 0.5):
        clock = FakeClock()
        io = make_io(clock, FakeArm(clock), policy=FakePolicy(demo_actions()))
        result = run("replay", io, tmp_path / str(speed), speed=speed, max_seconds=4.0, episode="replay_demo")
        frames[speed] = len(sent(result, "state"))

    assert frames[0.25] > 10
    assert abs(frames[0.5] - 2 * frames[0.25]) <= 2


def test_max_lead_line_reports_the_largest_policy_phase_lead(tmp_path, capsys):
    """Proves: replay prints `MAX LEAD <rad> rad (<joint>)` with the lag of the policy phase at weight 1 (here ~0.03 rad on
    the left shoulder pitch), not the 0.2 rad sag of the ramp-in (PLAN.md task B2 step 1b, rule 16 (c)).
    Catches: a lead taken over the ramp, where weight < 1, so M3 would wrongly forbid M4 - or over nothing at all."""
    clock = FakeClock()
    arm = SagThenTrackArm(clock)
    arm.sag_per_read = RAMP_SAG_PER_READ
    io = make_io(clock, arm, policy=FakePolicy(demo_actions()))

    result = run("replay", io, tmp_path, speed=0.5, max_seconds=3.0, episode="replay_demo")

    lines = [line for line in capsys.readouterr().out.splitlines() if line.startswith("MAX LEAD")]
    phases = sent(result, "sent_phase")
    ramp_gap = np.abs(sent(result, "sent_q") - sent(result, "measured_q"))[phases == "ramp_in"].max()
    assert len(lines) == 1 and lines[0].endswith("rad (LeftShoulderPitch)"), lines
    assert ramp_gap > 0.15, "the arm never sagged in the ramp: the test proves nothing"
    assert POLICY_LAG - 0.01 <= float(lines[0].split()[2]) <= POLICY_LAG + 0.01
