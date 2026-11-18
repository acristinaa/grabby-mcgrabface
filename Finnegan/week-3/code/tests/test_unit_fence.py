"""Unit: the virtual fence stage of the SafetyFilter, the never-closer rule, the midpoint check and the chunk crossfade
(FENCE-SMOOTH.md 1.5, 2.3; tests U1-U5, U7, U8, U9/N1, U10, U12, S1).

No MuJoCo distances, no DDS: WallFence is the real g1sim.fence.Fence logic (values, pred, midpoint, held values, check) with
one stubbed geometry - a wall in front of the left shoulder pitch, distance = WALL - angle (rad read as metres). Numbers
below are written out on purpose (not fence.*), so a changed rule or constant fails here. Runner-level fence tests (hands,
soft exit, shadow, replay without blend, refusals): tests/test_unit_real_run_fence.py. FK geometry: test_integration_fence.py.
"""
import pathlib
import types

import numpy as np
import pytest

from g1sim import action_space, fence
from g1sim import real_safety as rs

pytestmark = pytest.mark.unit

DATA = pathlib.Path(__file__).resolve().parent / "data"
ARM = 3                          # position of motor 15 (left shoulder pitch) in a 17-vector
RIGHT_ARM = 10                   # position of motor 22 (right shoulder pitch)
WALL = -0.20                     # the stub wall: distance = WALL - q[ARM]; the envelope allows q[ARM] up to -0.15
TABLE_MARGIN = 0.013             # FENCE-SMOOTH.md 1.2 rule for the v1 demos (R-F1), written out
WIDE_LO, WIDE_HI = np.full(17, -3.0), np.full(17, 3.0)
HANDS = np.zeros(12)
READY17 = rs.sdk_targets(action_space.BALL_READY_POSE, np.zeros(17))


class WallFence(fence.Fence):
    """The real Fence decision code on a one-wall geometry; records every pose it was asked about."""

    def __init__(self, wall=WALL, shape="line"):
        self.wall, self.shape, self.pairs, self.poses = wall, shape, {"table": None}, []

    def pose_values(self, q17, hands12, pelvis_rp=None):
        q = np.array(q17, dtype=float)
        self.poses.append((q, np.array(hands12, dtype=float), pelvis_rp))
        if self.shape == "vee":      # closest exactly at the wall, 2 mm away (a convex corner the arm swings past)
            return {"table": abs(q[ARM] - self.wall) + 0.002}
        return {"table": self.wall - q[ARM]}


def fenced_filter(last_arm, streak_ticks=None, wall_fence=None):
    """A SafetyFilter whose last command has the left shoulder pitch at last_arm, with a WallFence."""
    q0 = READY17.copy()
    q0[ARM] = last_arm
    return rs.SafetyFilter(WIDE_LO, WIDE_HI, q0, rs.ROBOT_WAIST_MAX_CEILING, False, fence=wall_fence or WallFence(),
                           fence_streak_ticks=streak_ticks)


def push(filt, target_arm, measured=None, phase="policy", pelvis_rp=None, hands=HANDS):
    """One message wanting the left shoulder pitch at target_arm (rest as last sent); measured defaults to the last command."""
    target = filt.last.copy()
    target[ARM] = target_arm
    measured = filt.last.copy() if measured is None else measured
    return filt.step(target, measured, phase, 1.0, hands, pelvis_rp)


def test_fence_holds_a_command_that_pushes_inside_the_margin():
    """Proves (U1): a policy command that would bring the arm below the table margin and closer than holding still is not
    sent: the last command goes out again, `last` is unchanged, event fence_hold.
    Catches: a removed fence stage, or a hold that sends the new command anyway [mutants 1, 2]."""
    filt = fenced_filter(-0.215)                         # 1.5 cm from the wall, above the 1.3 cm margin

    cmd, events = push(filt, -0.10)                      # rate limit -> -0.195: 0.5 cm through the wall

    assert "fence_hold" in events
    assert cmd[ARM] == pytest.approx(-0.215) and filt.last[ARM] == pytest.approx(-0.215)


def test_fence_streak_fires_on_the_nth_held_tick_and_a_sent_tick_resets_it():
    """Proves (U2): N - 1 held ticks raise no fence_streak, the N-th does; one sent tick in between starts the count again.
    Catches: an off-by-one streak or a streak that never resets [mutants 6, 7]."""
    filt = fenced_filter(-0.215, streak_ticks=5)

    first = [push(filt, -0.10)[1] for _ in range(4)]
    fifth = push(filt, -0.10)[1]
    push(filt, -0.30)                                    # moves away: sent, streak back to 0 (last -0.235)
    push(filt, -0.10)                                    # -0.215: 1.5 cm, above the margin: sent
    after_reset = push(filt, -0.10)[1]                   # -0.195: held again, the first of a new streak

    assert all("fence_hold" in e and "fence_streak" not in e for e in first)
    assert "fence_streak" in fifth
    assert "fence_hold" in after_reset and "fence_streak" not in after_reset and filt.fence_streak == 1


def test_fence_judges_the_command_after_envelope_and_rate_limit():
    """Proves (U3): the fence sees the command that would be sent - envelope (-0.15) and rate limit (0.02 rad) applied -
    not the raw policy target (-0.10).
    Catches: a fence placed before the rate limit, which would refuse targets the arm never reaches this tick [mutant 15]."""
    wall_fence = WallFence(wall=-0.10)                   # far away: nothing is held
    filt = fenced_filter(-0.25, wall_fence=wall_fence)

    push(filt, -0.10)

    cmd_pose = wall_fence.poses[0][0]
    assert cmd_pose[ARM] == pytest.approx(-0.23)


def test_fence_is_off_outside_the_policy_phase():
    """Proves (U4): in every scripted phase the fence is not asked and holds nothing (87 checked those paths with 5 cm).
    Catches: a fence that refuses the way in or the soft exit, freezing the arm on a checked path [mutant 12]."""
    for phase in (p for p in rs.PHASES if p != "policy"):
        wall_fence = WallFence()
        filt = fenced_filter(-0.215, wall_fence=wall_fence)

        cmd, events = push(filt, -0.10, phase=phase)

        assert wall_fence.poses == [] and "fence_hold" not in events, phase


def test_lead_monitor_measures_against_the_held_command():
    """Proves (U5): on a held tick the lead monitor compares the measured angles with what was really sent (the held
    command): a right arm 0.26 rad off its held command faults after 13 ticks, although the refused command (0.02 closer)
    would have been inside the 0.25 rad limit.
    Catches: a lead monitor fed with the refused command, which would hide a real contact during a hold."""
    filt = fenced_filter(-0.215)
    measured = filt.last.copy()
    measured[RIGHT_ARM] += 0.26

    events = []
    for _ in range(13):
        target = filt.last.copy()
        target[ARM], target[RIGHT_ARM] = -0.10, filt.last[RIGHT_ARM] + 0.02
        events = filt.step(target, measured, "policy", 1.0, HANDS, None)[1]

    assert "fence_hold" in events and "contact_fault" in events


@pytest.mark.parametrize("bad", ["measured", "imu"])
def test_fence_holds_on_a_nan_angle_or_imu_tilt(bad):
    """Proves (U7): a NaN measured arm/waist angle or a NaN IMU roll/pitch holds the command (fail closed) without asking
    the geometry.
    Catches: NaN slipping through as "far away" (NaN < margin is False) and the arm moving on blind."""
    wall_fence = WallFence(wall=-0.10)                   # would send: only the NaN can hold
    filt = fenced_filter(-0.25, wall_fence=wall_fence)
    measured = filt.last.copy()
    if bad == "measured":
        measured[ARM] = np.nan
    imu = np.array([np.nan, 0.0]) if bad == "imu" else np.zeros(2)

    cmd, events = push(filt, -0.20, measured=measured, pelvis_rp=imu)

    assert "fence_hold" in events and cmd[ARM] == pytest.approx(-0.25) and wall_fence.poses == []


def test_never_closer_rule():
    """Proves (U10, review M2): below the margin and closer than holding still -> refused; below the margin but farther
    than holding (the arm leaves) or equal (along the surface) -> sent; above the margin -> sent; and through the filter an
    arm that overshot inside the margin can move away again.
    Catches: the old rule "below the margin -> hold", which keeps an overshot arm stuck until the streak exit [mutant 18]."""
    held = {"table": 0.008}

    assert fence.refused_groups({"table": 0.006}, held) == ["table"]
    assert fence.refused_groups({"table": 0.010}, held) == []
    assert fence.refused_groups({"table": 0.008}, held) == []
    assert fence.refused_groups({"table": 0.020}, held) == []
    assert fence.low_groups({"table": TABLE_MARGIN - 1e-6}) == ["table"] and fence.low_groups({"table": TABLE_MARGIN}) == []
    filt = fenced_filter(-0.205)                         # 0.5 cm from the wall: inside the margin already
    cmd, events = push(filt, -0.30)                      # moves away
    assert "fence_hold" not in events and cmd[ARM] == pytest.approx(-0.225)


def test_fence_summary_keeps_sent_and_refused_minima_apart():
    """Proves (code review m1): the summary's smallest_cm is the closest a SENT command came; the closest refused (held)
    command is reported on its own as refused_smallest_cm.
    Catches: a "smallest" that counts commands the fence never let out, which reads as "the arm came this close"."""
    filt = fenced_filter(-0.235)

    push(filt, -0.10)                                    # -> -0.215: 1.5 cm from the wall, sent
    _, events = push(filt, -0.10)                        # -> -0.195: 0.5 cm through the wall, refused
    summary = filt.fence_log.summary()

    assert "fence_hold" in events
    assert summary["smallest_cm"]["table"] == pytest.approx(1.5)
    assert summary["refused_smallest_cm"]["table"] == pytest.approx(-0.5)


def test_first_hold_of_the_policy_phase_uses_the_hands_handed_over():
    """Proves (code review m5): after reset_last(command, hands) at the hand-over to the policy phase, the fence's "hold
    still" value of the very first policy tick puts the hands where they really are, not at the policy's first target.
    Catches: F(hold) computed with the candidate's fingers on the first tick (last_hands still None)."""
    wall_fence = WallFence()
    filt = fenced_filter(-0.235, wall_fence=wall_fence)
    ready_hands, policy_hands = np.full(12, 0.1), np.full(12, 0.5)
    last = filt.last.copy()
    last[ARM] = -0.205                                   # 0.5 cm from the wall: the candidate is low, so F(hold) is asked
    filt.reset_last(last, ready_hands)

    push(filt, -0.10, hands=policy_hands)

    held = [hands for q, hands, _ in wall_fence.poses if q[ARM] == pytest.approx(-0.205)]
    assert held and all(np.array_equal(h, ready_hands) for h in held)


def test_midpoint_between_two_clear_ends_is_checked_near_an_obstacle():
    """Proves (review N1, replaces U9): when the arm passes a corner, both ends of the 20 ms step can be clear (2.2 cm) while
    the pose halfway is 0.2 cm away; the fence checks the halfway pose too and holds.
    Catches: a fence that only checks the tick ends - the rate limit allows ~2.3 cm per tick, more than twice the smallest
    margin, so the arm could cut through a corner between two checked poses."""
    vee = WallFence(shape="vee")
    filt = fenced_filter(-0.22, wall_fence=vee)          # 2.2 cm before the corner

    cmd, events = push(filt, -0.18)                      # 2.2 cm past it; halfway = at the corner

    assert vee.values(np.r_[READY17[:ARM], -0.18, READY17[ARM + 1:]], HANDS,
                      np.r_[READY17[:ARM], -0.22, READY17[ARM + 1:]],
                      np.r_[READY17[:ARM], -0.22, READY17[ARM + 1:]])["table"] == pytest.approx(0.002)
    assert "fence_hold" in events and cmd[ARM] == pytest.approx(-0.22)


def test_pred_shifts_the_arm_by_its_tracking_error_and_keeps_the_commanded_hands():
    """Proves (FENCE-SMOOTH.md 1.1, review M3): `pred` = command + (measured - last sent) on the arms, waist roll/pitch at
    the measured angles, and both poses carry the commanded hand target, not the measured fingers.
    Catches: a cmd-only fence (blind to the sagging arm), a flipped sign, or measured fingers in the check [mutants 3, 4, 22]."""
    wall_fence = WallFence(wall=-0.10)
    cand, last, measured = READY17.copy(), READY17.copy(), READY17.copy()
    cand[ARM], last[ARM], measured[ARM] = -0.25, -0.26, -0.20     # the arm sits 0.06 rad off its last command
    measured[1:3] = [0.01, 0.05]
    hands = np.full(12, 0.3)

    wall_fence.values(cand, hands, measured, last)

    (cmd_pose, cmd_hands, _), (pred_pose, pred_hands, _) = wall_fence.poses[:2]
    assert cmd_pose[ARM] == pytest.approx(-0.25) and pred_pose[ARM] == pytest.approx(-0.25 + 0.06)
    assert np.allclose(cmd_pose[1:3], [0.01, 0.05]) and np.allclose(pred_pose[1:3], [0.01, 0.05])
    assert np.array_equal(cmd_hands, hands) and np.array_equal(pred_hands, hands)


def test_fence_streak_length_is_one_chunk_plus_half_a_second():
    """Proves (R-F2): the streak that ends a run is ceil((25 / (30 x speed) + 0.5) x 50) held messages: 109 at speed 0.5,
    192 at 0.25.
    Catches: a streak shorter than one chunk (the policy never gets its next chunk to steer away) or a changed formula."""
    assert rs.fence_streak_ticks(0.5) == 109 and rs.fence_streak_ticks(0.25) == 192


def test_layout_from_sheet_maps_the_runbook_numbers():
    """Proves (I3, the arithmetic part): setback moves the robot back (table further away), stand/basket offsets and table
    height land in the sim variation keys in metres, robot yaw adds to the stance, no basket removes it.
    Catches: a cm/m mix-up or an ignored sheet number [mutant 8]; the FK effect is pinned in test_integration_fence.py."""
    layout = fence.layout_from_sheet(10.0, table_dz_cm=2.0, stand_dx_cm=6.7, stand_dy_cm=-1.7, basket_dx_cm=1.0,
                                     basket_dy_cm=-3.0, robot_yaw_deg=2.0)
    no_basket = fence.layout_from_sheet(0.0, basket_dx_cm=None)

    assert layout["stance"]["x_m"] == pytest.approx(fence.WALK_STANCE["x_m"] - 0.10)
    assert layout["stance"]["yaw_deg"] == pytest.approx(2.0) and layout["table"]["dz_m"] == pytest.approx(0.02)
    assert layout["objects"]["volleyball"] == pytest.approx({"dx_m": 0.067, "dy_m": -0.017})
    assert layout["basket"] == pytest.approx({"dx_m": 0.01, "dy_m": -0.03}) and "basket" not in no_basket


def test_fence_refuses_to_build_without_the_pinned_mujoco_the_model_file_or_a_layout(monkeypatch, tmp_path):
    """Proves (U12): no mujoco, another mujoco version, a missing model file or no layout -> FenceUnavailable with a line
    the operator can act on (86 turns it into REFUSED).
    Catches: a silent no-fence run, or distances from a MuJoCo version the margins were not measured with."""
    def problem():
        with pytest.raises(fence.FenceUnavailable) as error:
            fence.Fence.nominal(fence.layout_from_sheet(0.0), model_file=tmp_path / "missing.xml.gz")
        return str(error.value)

    monkeypatch.setattr(fence, "mujoco", None)
    no_mujoco = problem()
    monkeypatch.setattr(fence, "mujoco", types.SimpleNamespace(__version__="3.12.0"))
    old_mujoco = problem()
    monkeypatch.setattr(fence, "mujoco", types.SimpleNamespace(__version__=fence.MUJOCO_VERSION))
    missing = problem()
    with pytest.raises(fence.FenceUnavailable, match="no layout"):
        fence.Fence.nominal(None)

    assert "not installed" in no_mujoco and "3.12.0" in old_mujoco and "missing" in missing


# --- crossfade at the chunk switch (U8, S1) ---------------------------------------------------------------------------

def test_blend_spreads_a_chunk_step_over_five_actions():
    """Proves (U8): a 0.05 rad jump between the old and the new chunk is faded in linearly over 5 actions (half way after
    2.5), without blend it lands at once; a fade over 0 actions is the old behaviour.
    Catches: an inverted fade (starting at the new chunk), a fade that never ends, or a blend that is always on [mutant 14]."""
    old, new = np.zeros((50, 27)), np.full((50, 27), 0.05)
    scheduler = rs.ChunkScheduler(1.0, blend=5)
    scheduler.add_chunk(old, scheduler.mark_requested())
    scheduler.advance(10 / 30)                            # 10 actions in
    scheduler.mark_requested()
    scheduler.add_chunk(new, 10.0)
    at = {}
    for actions in (0.0, 2.5, 5.0, 6.0):
        probe = rs.ChunkScheduler(1.0, blend=5)
        probe.__dict__.update({k: v for k, v in scheduler.__dict__.items()})
        probe.advance(actions / 30)
        at[actions] = probe.target()[0]
    unblended = rs.ChunkScheduler(1.0)
    unblended.add_chunk(old, unblended.mark_requested())
    unblended.advance(10 / 30)
    unblended.mark_requested()
    unblended.add_chunk(new, 10.0)

    assert at[0.0] == pytest.approx(0.0) and at[2.5] == pytest.approx(0.025) and at[5.0] == pytest.approx(0.05)
    assert at[6.0] == pytest.approx(0.05) and unblended.target()[0] == pytest.approx(0.05)


def test_blend_changes_nothing_when_the_new_chunk_continues_the_old_one_or_the_old_one_ran_out():
    """Proves (U8): a new chunk that continues the old one exactly (the same demo rows at the same moments) blends to the
    unblended targets; a new chunk after the old one ran out is played as is.
    Catches: a fade that mixes in something other than the old chunk at the same moment, or a fade toward a chunk that is no
    longer there. (A real replay request starts at a fractional position, so its chunks are shifted by up to one action:
    that is why `replay` gets no blend at all, review B3, test_unit_real_run_fence.py.)"""
    demo = np.cumsum(np.full((200, 27), 0.003), axis=0)
    results = {}
    for blend in (0, 5):
        scheduler = rs.ChunkScheduler(0.5, blend=blend)
        scheduler.add_chunk(demo[0:50], scheduler.mark_requested())
        targets = []
        for tick in range(150):
            if tick == 90:                               # position 27: request "at" the whole action 25
                scheduler.mark_requested()
                scheduler.add_chunk(demo[25:75], 25.0)
            targets.append(scheduler.target())
            scheduler.advance(1.0 / rs.SEND_HZ)
        results[blend] = np.array(targets)
    ran_out = rs.ChunkScheduler(1.0, blend=5)
    ran_out.add_chunk(np.zeros((50, 27)), ran_out.mark_requested())
    ran_out.advance(60 / 30)                             # 60 actions: the old chunk (50) has run out
    ran_out.mark_requested()
    ran_out.add_chunk(np.full((50, 27), 0.05), 60.0)

    assert np.abs(results[5] - results[0]).max() < 1e-12
    assert ran_out.target()[0] == pytest.approx(0.05)


def smoothness_86(chunks, speed, blend, latency_actions):
    """The 86 timing harness of FENCE-SMOOTH.md 2.2: stored chunks through ChunkScheduler + SafetyFilter at 50 Hz; per-trial
    p99 |acceleration| over moving ticks (worst joint |v| > 0.05 rad/s), worst joint (2.1)."""
    q0 = np.zeros(17)
    filt = rs.SafetyFilter(WIDE_LO, WIDE_HI, q0, rs.ROBOT_WAIST_MAX_CEILING, False)
    filt.reset_last(rs.sdk_targets(chunks[0][0], q0))
    scheduler = rs.ChunkScheduler(speed, blend=blend)
    scheduler.add_chunk(chunks[0], scheduler.mark_requested())
    sent, pending, k = [], None, 1
    while scheduler.position < 25 * len(chunks):
        if pending is not None and scheduler.position >= pending[1] + latency_actions:
            scheduler.add_chunk(*pending)
            pending = None
        if pending is None and scheduler.needs_request():
            at = scheduler.mark_requested()
            pending, k = (chunks[min(k, len(chunks) - 1)], at), k + 1
        action = scheduler.target()
        if action is None:
            break
        sent.append(filt.step(rs.sdk_targets(action, q0), filt.last, "policy", 1.0)[0])
        scheduler.advance(1.0 / rs.SEND_HZ)
    q = np.array(sent)[:, np.r_[3:17, 0]]
    v = np.diff(q, axis=0) * rs.SEND_HZ
    a = np.abs(np.diff(v, axis=0)) * rs.SEND_HZ
    moving = np.abs(v[1:]).max(axis=1) > 0.05
    return float(np.percentile(a[moving], 99, axis=0).max())


@pytest.mark.parametrize("latency_actions", [0, 3])
def test_blend_brings_run1_chunks_below_the_teacher_bar(latency_actions):
    """Proves (S1): the run1 45k policy's stored chunks (tests/data/run1_45k_chunks.npz, one trial) through 86's scheduler
    at speed 0.5 are above the smoothness bar without the fade and below it with BLEND_ACTIONS = 5 (bar 17.1 rad/s^2 = the
    teacher's 95th-percentile trial, FENCE-SMOOTH.md 2.1; measured 22.9 / 30.2 -> 12.9 / 13.0).
    Catches: a fade that does not reach the 86 path, or a regression that brings the chunk-boundary jerk back."""
    chunks = np.load(DATA / "run1_45k_chunks.npz")["chunks"].astype(float)

    unblended = smoothness_86(chunks, 0.5, 0, latency_actions)
    blended = smoothness_86(chunks, 0.5, rs.BLEND_ACTIONS, latency_actions)

    assert rs.BLEND_ACTIONS == 5
    assert unblended > 17.1 and blended < 17.1, (unblended, blended)
