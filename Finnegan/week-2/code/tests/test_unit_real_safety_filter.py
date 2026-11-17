"""Unit: the per-message SafetyFilter of g1sim/real_safety.py - what 86 actually sends on rt/arm_sdk (PLAN.md 3.3, rules 4-11).

No DDS, no robot: every test feeds plain numpy vectors. A "17-vector" holds motors 12..28 (position i = motor 12 + i).
Tests marked [mutation] in PLAN.md were seen failing on a broken copy (runs/real_b1_mutation.log).
"""
import math

import numpy as np
import pytest

from g1sim import action_space
from g1sim import real_safety as rs

pytestmark = pytest.mark.unit

WIDE_LO, WIDE_HI = np.full(17, -3.0), np.full(17, 3.0)   # fake joint limits wider than any target used here
ARM = 3                                                # position of motor 15 (left shoulder pitch) in a 17-vector
ARM_ROLL = 4                                           # position of motor 16 (left shoulder roll) in a 17-vector
WAIST_YAW = 0                                          # position of motor 12 (waist yaw) in a 17-vector
EPS = 1e-9                                             # float tolerance for step-size comparisons
# The plan's numbers, written out on purpose (not rs.*): a changed constant must fail these tests
ARM_STEP_MAX = 1.0 / 50                                # rad per 50 Hz message, arms (rule 6: 1.0 rad/s)
WAIST_STEP_MAX = 0.3 / 50                              # rad per 50 Hz message, waist yaw (rule 6: 0.3 rad/s)
SAFE_LO, SAFE_HI = -0.45, 0.45                         # fake limits -/+0.5 minus the 0.05 rad margin (rule 5)
LEAD_JUST_OVER = 0.26                                  # rad, 0.01 above the 0.25 rad lead limit (rule 7)


def make_filter(q0, lo=WIDE_LO, hi=WIDE_HI, waist_max=rs.ROBOT_WAIST_MAX_CEILING, waist_hold=False):
    """A SafetyFilter starting at q0, with wide fake joint limits unless a test passes its own."""
    return rs.SafetyFilter(lo, hi, np.asarray(q0, dtype=float), waist_max, waist_hold)


def ready17(q0=np.zeros(17)):
    """BALL_READY_POSE as a 17-vector (waist roll/pitch from q0)."""
    return rs.sdk_targets(action_space.BALL_READY_POSE, q0)


def test_envelope_clips_only_in_policy_phase():
    """Proves: the demo envelope limits policy targets only; scripted phases pass the same target unchanged.
    Catches: an envelope applied to scripted moves, which blocks the hanging factory pose outside the demos (review B1)."""
    target = ready17()
    target[ARM] = 0.2                                  # left shoulder pitch above the envelope top (-0.15)
    target[ARM_ROLL] = -0.5                            # left shoulder roll below the envelope bottom (-0.2105)
    scripted = [p for p in rs.PHASES if p not in rs.ENVELOPE_PHASES]

    cmd_policy, events_policy = make_filter(target).step(target, target, "policy", 1.0)
    scripted_results = [make_filter(target).step(target, target, p, 1.0) for p in scripted]

    assert "envelope_clip" in events_policy and cmd_policy[ARM] < 0.2 and cmd_policy[ARM_ROLL] > -0.5
    assert all("envelope_clip" not in ev and np.array_equal(cmd, target) for cmd, ev in scripted_results)


def test_envelope_streak_fires_on_the_30th_clipped_policy_tick():
    """Proves: 30 clipped policy ticks in a row raise envelope_streak (soft exit, rule 5) on tick 30 and not before, and one
    unclipped tick in between starts the count again (14 clipped, 1 unclipped, then the streak fires on the 30th after it).
    Catches: a policy that keeps pushing against the envelope forever, an exit after the first clip, or a count that
    adds up clips that were not in a row."""
    inside = ready17()
    outside = ready17()
    outside[ARM] = 0.2                                 # left shoulder pitch above the envelope top (-0.15)
    in_a_row, interrupted = make_filter(inside), make_filter(inside)
    with_a_gap = [outside] * 14 + [inside] + [outside] * 30   # tick 15 unclipped; 44 clipped, but at most 30 in a row

    events = [in_a_row.step(outside, outside, "policy", 1.0)[1] for _ in range(30)]
    gap_events = [interrupted.step(t, t, "policy", 1.0)[1] for t in with_a_gap]

    assert all("envelope_streak" not in ev for ev in events[:-1]) and "envelope_streak" in events[-1]
    assert all("envelope_streak" not in ev for ev in gap_events[:-1]) and "envelope_streak" in gap_events[-1]


def test_filter_with_nan_waist_max_raises_instead_of_sending_nan():
    """Proves: a NaN limit inside the filter ends in InvalidTarget (fault exit), not in a NaN command (rule 11).
    Catches: np.clip with a NaN bound, which turns a good waist target into NaN on the motor."""
    q0 = np.zeros(17)
    flt = make_filter(q0, waist_max=math.nan)

    with pytest.raises(rs.InvalidTarget):
        flt.step(q0, q0, "hold", 1.0)


def test_ball_ready_pose_lies_inside_envelope():
    """Proves: the pose every policy phase starts from passes the envelope without a clip.
    Catches: an envelope (or ready pose) change that clips the very first policy target and starts an envelope streak."""
    target = ready17()

    _, events = make_filter(target).step(target, target, "policy", 1.0)

    assert "envelope_clip" not in events


def test_hard_limits_clip_in_every_phase():
    """Proves: every phase keeps every joint 0.05 rad inside both joint limits, the lower one as well as the upper one.
    Catches: a joint driven into its end stop: a clip that only works upwards, a margin applied outwards, or a margin of 0."""
    lo, hi = np.full(17, -0.5), np.full(17, 0.5)
    near_top, near_bottom = np.full(17, 0.44), np.full(17, -0.44)

    upward = [make_filter(near_top, lo, hi).step(np.full(17, 2.0), near_top, p, 1.0) for p in rs.PHASES]
    downward = [make_filter(near_bottom, lo, hi).step(np.full(17, -2.0), near_bottom, p, 1.0) for p in rs.PHASES]

    assert all(np.all(cmd <= SAFE_HI + EPS) and "limit_clip" in ev for cmd, ev in upward)
    assert all(np.all(cmd >= SAFE_LO - EPS) and "limit_clip" in ev for cmd, ev in downward)


def test_rate_limit_caps_arm_and_waist_steps_in_every_phase():
    """Proves: in every phase, on every one of 5 messages in a row, an arm joint moves exactly 1.0 rad/s and the waist yaw
    0.3 rad/s towards a far target, never more (rule 6).
    Catches: a policy jump passed to the motors - in one phase, from the second message on, or with a raised speed limit."""
    q0 = np.zeros(17)
    target = np.full(17, 0.5)                          # in "policy" the envelope moves some joints to -0.15 or 0.1, still 0.1+ away

    for phase in rs.PHASES:
        flt = make_filter(q0)
        commands = [q0] + [flt.step(target, q0, phase, 1.0)[0] for _ in range(5)]
        steps = np.abs(np.diff(commands, axis=0))

        assert np.allclose(steps[:, rs.ARMS], ARM_STEP_MAX), phase
        assert np.allclose(steps[:, WAIST_YAW], WAIST_STEP_MAX), phase


def test_waist_roll_pitch_always_equal_q0():
    """Proves: motors 13/14 get exactly q0 in every phase and every tick, whatever the target says (rule 4).
    Catches: a waist roll/pitch command left at the target (or at kp 0), which tips the torso (F-72a)."""
    q0 = np.zeros(17)
    q0[1], q0[2] = 0.1, -0.05
    targets = np.random.default_rng(0).uniform(-0.4, 0.4, (len(rs.PHASES) * 5, 17))
    flt = make_filter(q0)

    held = [flt.step(t, q0, rs.PHASES[i % len(rs.PHASES)], 1.0)[0][1:3] for i, t in enumerate(targets)]

    assert all(np.array_equal(h, q0[1:3]) for h in held)


def test_waist_hold_keeps_motor_12_at_q0_in_every_phase():
    """Proves: with --waist-hold the waist yaw (motor 12) gets exactly its q0 on every message of every phase (rule 6b).
    Catches: a waist that still turns under --waist-hold, which breaks 87's table-clearance lookup ("waist limit 0.0")."""
    q0 = np.zeros(17)
    q0[WAIST_YAW] = 0.02
    target = q0.copy()
    target[WAIST_YAW] = 0.5

    for phase in rs.PHASES:
        flt = make_filter(q0, waist_max=0.15, waist_hold=True)
        yaws = [flt.step(target, q0, phase, 1.0)[0][WAIST_YAW] for _ in range(5)]

        assert yaws == [0.02] * 5, phase


def test_waist_max_clips_waist_yaw_target():
    """Proves: --waist-max 0.15 stops the waist yaw at +0.15 rad and at -0.15 rad however far the target lies, in the policy
    phase as well (rule 6b, ruling R10).
    Catches: a waist limit applied wrongly (e.g. doubled), or only upwards, so the torso turns to the envelope bottom
    (-0.81 rad) while the clearance lookup assumed 0.15 (review R4-F1)."""
    q0 = np.zeros(17)
    left, right = q0.copy(), q0.copy()
    left[WAIST_YAW], right[WAIST_YAW] = 0.5, -0.5     # -0.5 lies inside the envelope (-0.81), so only --waist-max can stop it
    up, down = make_filter(q0, waist_max=0.15), make_filter(q0, waist_max=0.15)

    up_yaws = [up.step(left, q0, "hold", 1.0)[0][WAIST_YAW] for _ in range(40)]   # 0.15 rad at 0.006 rad/tick needs 25
    down_yaws = [down.step(right, q0, "policy", 1.0)[0][WAIST_YAW] for _ in range(40)]

    assert max(up_yaws) <= 0.15 + EPS and up_yaws[-1] == pytest.approx(0.15)
    assert min(down_yaws) >= -0.15 - EPS and down_yaws[-1] == pytest.approx(-0.15)


def test_lead_fault_fires_after_13_ticks_only_at_full_weight():
    """Proves: a lead of 0.26 rad (just over 0.25) fires contact_fault on the 13th tick at weight 1, never while the weight ramps.
    Catches: a lead count during a ramp, where the blend makes command and measured differ by design (review N2)."""
    target = ready17()
    measured = target.copy()
    measured[ARM] -= LEAD_JUST_OVER
    ramping, full = make_filter(target), make_filter(target)

    ramp_events = [ev for _ in range(3 * rs.LEAD_TICKS) for ev in ramping.step(target, measured, "hold", 0.99)[1]]
    full_events = [full.step(target, measured, "hold", 1.0)[1] for _ in range(rs.LEAD_TICKS)]

    assert "contact_fault" not in ramp_events
    assert all("contact_fault" not in ev for ev in full_events[:-1]) and "contact_fault" in full_events[-1]


def test_nan_measured_angle_counts_as_a_lead_over_the_limit():
    """Proves: a NaN measured angle at full weight fires contact_fault on the 13th tick, like a real lead over 0.25 rad.
    Catches: a lead monitor that a NaN switches off silently (NaN > 0.25 is False, so the count resets every tick)."""
    target = ready17()
    measured = target.copy()
    measured[ARM] = math.nan
    flt = make_filter(target)

    events = [flt.step(target, measured, "hold", 1.0)[1] for _ in range(13)]

    assert all("contact_fault" not in ev for ev in events[:-1]) and "contact_fault" in events[-1]


def nan_action27():
    """BALL_READY_POSE with one NaN in the left arm: a broken policy output."""
    action = action_space.BALL_READY_POSE.astype(float)
    action[5] = math.nan
    return action


def test_nan_target_raises_invalid_target_in_safety_filter():
    """Proves: a NaN arm target is refused with InvalidTarget before anything is sent (rule 11).
    Catches: a NaN passed to the motors, where the controller's behaviour is undefined."""
    flt = make_filter(np.zeros(17))
    target = rs.sdk_targets(nan_action27(), np.zeros(17))

    with pytest.raises(rs.InvalidTarget):
        flt.step(target, np.zeros(17), "policy", 1.0)


def test_nan_target_raises_invalid_target_in_chunk_scheduler():
    """Proves: a policy chunk with a NaN is refused with InvalidTarget when it arrives, before it is played (rule 11).
    Catches: a broken chunk that is only noticed (or never noticed) when its NaN action comes up."""
    scheduler = rs.ChunkScheduler(0.5)
    chunk = np.tile(nan_action27(), (50, 1))

    with pytest.raises(rs.InvalidTarget):
        scheduler.add_chunk(chunk, 0.0)


def test_nan_target_raises_invalid_target_in_hand_targets():
    """Proves: a NaN hand target is refused with InvalidTarget, not clipped into a number (rule 11).
    Catches: np.clip passing NaN through to the BrainCo hand."""
    action = np.zeros(27)
    action[15] = math.nan                              # one finger of the left hand

    with pytest.raises(rs.InvalidTarget):
        rs.hand_targets(action)
