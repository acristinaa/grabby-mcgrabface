"""Unit: the 86 runner - ramp-in, exits and the policy phase (g1sim/real_run.py, real_run_modes.py, real_run_policy.py).

Every test drives the real runner with fakes (tests/helpers/fake_real_run.py): a simulated arm behind LowState and rt/arm_sdk,
fake hands, camera, policy client and a clock that only moves when the runner sleeps. No DDS, no robot, seconds.
Preflight refusals: tests/test_unit_real_preflight.py. The sending module and the motion-code ban: tests/test_unit_real_send.py.
"""
import errno
import json

import numpy as np
import pytest

pytest.importorskip("unitree_sdk2py")

from g1sim import action_space, arm_paths, policy_wire, real_safety as rs  # noqa: E402
from g1sim.real_run_exits import HANDS_OPEN_S  # noqa: E402
from g1sim import real_preflight as pre  # noqa: E402
from helpers.fake_real_run import (FACTORY, FakeArm, FakeClock, FakeFrames, FakeHand, FakePolicy, demo_actions,  # noqa: E402
                                   factory_record, last_index, make_io, read_sidecar, run, sent, signals_at)

pytestmark = pytest.mark.unit

READY17 = rs.sdk_targets(action_space.BALL_READY_POSE, FACTORY)
WAIST_ROLL_PITCH = list(rs.WAIST_HOLD_POS)      # motors 13/14 in a 17-vector
WAIST_ROLL_DRIFT = 0.02                        # rad the waist roll reads off its earlier angle (visible, inside the limits)
WAY_IN = arm_paths.way_in(FACTORY, READY17)    # the plan 86 runs from the fake factory pose
CONTACT_AT_S = 3.0                             # s after the start: 1 s into way-in leg 1 (after preflight and the 2 s ramp)
CONTACT_CLEARS_S = 1.5                         # s later the obstacle is gone: the fault (~1.1 s) and its 0.5 s hold overlap it
HANG_GUARD_S = 60.0                            # s after a fault: a stop signal that only a hold still waiting would see
REAL_PRINT = print


def print_fails_after_signal(signal_count):
    """print() as after the terminal closed: once a stop signal arrived, every print raises OSError EIO (measured on
    this Mac by the B2 reviewer, runs/real_b2_review_r1_sighup.log)."""
    def dead_terminal_print(*args, **kwargs):
        if signal_count() > 0:
            raise OSError(errno.EIO, "Input/output error")
        REAL_PRINT(*args, **kwargs)
    return dead_terminal_print


# --- ramp-in, waist hold, exits -------------------------------------------------------------------------------------------

def test_ready_sends_frozen_q0_during_ramp_in_while_arm_sags(tmp_path):
    """Proves: every ramp_in message commands the angles measured once at the first ramp tick, although the arm sags.
    Catches: a ramp that re-reads the angles each tick and so chases the sagging arm down (review N1) [mutation]."""
    clock = FakeClock()
    arm = FakeArm(clock)
    arm.sag_per_read = 0.002

    result = run("ready", make_io(clock, arm), tmp_path, hold=0.5)

    ramp = sent(result, "sent_q")[sent(result, "sent_phase") == "ramp_in"]
    assert result.exit_code == 0
    assert len(ramp) == 101
    assert np.ptp(ramp, axis=0).max() == 0.0, "q0 changed during the ramp"


def test_every_arm_sdk_message_holds_waist_roll_pitch(tmp_path):
    """Proves: every rt/arm_sdk sample with weight > 0 carries motors 13/14 at q0 with the sim gains (kp 250).
    Catches: the 72 `arm` bug (F-72a) - waist roll/pitch left at kp 0 or a wrong angle while we hold the waist [mutation]."""
    clock = FakeClock()
    arm = FakeArm(clock)

    run("ready", make_io(clock, arm), tmp_path, hold=0.5)

    held = [m for m in arm.messages if m["weight"] > 0]
    assert held
    for message in held:
        assert np.allclose(message["q"][1:3], FACTORY[1:3], atol=1e-9)
        assert message["kp13_14"] == tuple(rs.SIM_GAINS_KP[1:3])


def test_release_holds_waist_roll_pitch_at_the_measured_angles(tmp_path):
    """Proves: `release` from a pose whose waist roll differs from the factory record keeps motors 13/14 at the measured
    angles in every message, i.e. release goes through the SafetyFilter (B1 review P2).
    Catches: a release that skips the filter and turns the waist towards the recorded angles while it walks the arms
    home (B2 review r3 #2) [mutation]."""
    clock = FakeClock()
    start = FACTORY.copy()
    start[WAIST_ROLL_PITCH[0]] += WAIST_ROLL_DRIFT
    arm = FakeArm(clock, start)

    result = run("release", make_io(clock, arm, factory_record=factory_record(FACTORY)), tmp_path)

    assert result.exit_code == 0 and arm.messages
    for message in arm.messages:
        assert np.allclose(message["q"][WAIST_ROLL_PITCH], start[WAIST_ROLL_PITCH], atol=1e-9)


def test_contact_fault_holds_measured_then_goes_back_along_the_way_in_at_half_speed(tmp_path):
    """Proves: a contact on way-in leg 1 ends in exit 4 after: hold at the measured (stuck) angles with waist roll/pitch
    at q0, then back along the way in to q0 at 0.15 rad/s (ruling F2), and only there the ramp-out to weight 0.
    Catches: the old ramp-out at the contact pose - Unitree's controller then sweeps the low hands forward through the
    table (SAFE-PATHS.md) - a hold that keeps pushing towards the blocked target (review M2), waist roll/pitch on the
    pushed angles (rule 4), or a recover at full speed [mutation]."""
    clock = FakeClock()
    arm = FakeArm(clock)
    arm.stuck_after = clock() + CONTACT_AT_S
    arm.stuck_until = arm.stuck_after + CONTACT_CLEARS_S
    arm.stuck_drift[WAIST_ROLL_PITCH[0]] = WAIST_ROLL_DRIFT

    result = run("ready", make_io(clock, arm), tmp_path, hold=0.5)

    phases, q = sent(result, "sent_phase"), sent(result, "sent_q")
    held, home = q[phases == "fault_hold"], q[phases == "to_factory"]
    stuck = sent(result, "measured_q")[phases == "fault_hold"][0]
    assert (result.exit_code, result.reason) == (4, "contact_fault")
    assert np.ptp(held, axis=0).max() == 0.0 and np.allclose(np.delete(held[0], WAIST_ROLL_PITCH),
                                                              np.delete(stuck, WAIST_ROLL_PITCH), atol=1e-9)
    assert np.allclose(held[:, WAIST_ROLL_PITCH], FACTORY[WAIST_ROLL_PITCH], atol=1e-9), "waist roll/pitch left q0"
    half_speed = rs.SCRIPTED_SPEED / 2                                  # ruling F2, pinned here, not read from 86
    expected = arm_paths.path_ticks(arm_paths.recover(held[0], arm_paths.WAY_IN, WAY_IN, 1), half_speed)
    assert np.allclose(home, expected, atol=1e-9), "the recover is not the way in backwards at half speed"
    assert np.allclose(q[phases == "ramp_out"], FACTORY, atol=1e-9), "handed back away from the factory pose"
    assert sent(result, "sent_weight")[-1] == 0.0


def test_fault_exit_sends_no_hand_command(tmp_path):
    """Proves: hands get a command with every to_ready/hold message and none during a fault hold, recover or ramp-out
    (rule 12).
    Catches: a fault exit that keeps driving the fingers (closing on whatever the hand touched) [mutation]."""
    clock = FakeClock()
    arm, hands = FakeArm(clock), {"left": FakeHand(), "right": FakeHand()}
    arm.stuck_after = clock() + CONTACT_AT_S   # contact fault on the way in; the fault then recovers home
    arm.stuck_until = arm.stuck_after + CONTACT_CLEARS_S

    result = run("ready", make_io(clock, arm, hands), tmp_path, hold=0.5)

    phases = sent(result, "sent_phase")
    assert result.reason == "contact_fault"
    for hand in hands.values():
        assert len(hand.sent) == np.isin(phases, ["to_ready", "hold"]).sum()


def test_soft_exit_returns_to_start_and_opens_hands(tmp_path):
    """Proves: the normal end of `ready` walks back to q0, ramps out to weight 0 and leaves both hands opened at 0.3.
    Catches: an exit that drops the weight away from q0 (the arm swings) or leaves the fingers half closed."""
    clock = FakeClock()
    arm, hands = FakeArm(clock), {"left": FakeHand(), "right": FakeHand()}

    result = run("ready", make_io(clock, arm, hands), tmp_path, hold=0.5)

    phases, q = sent(result, "sent_phase"), sent(result, "sent_q")
    assert result.exit_code == 0
    assert np.allclose(q[phases == "to_start"][-1], FACTORY, atol=1e-9)
    assert sent(result, "sent_weight")[-1] == 0.0
    for hand in hands.values():
        targets, speed = hand.sent[-1]
        assert np.all(targets == 0.0) and speed == rs.HANDS_OPEN_SPEED


def hold_start_s(tmp_path) -> float:
    """Seconds after the start at which an undisturbed `ready` sends its first hold message (the way in is done)."""
    clock = FakeClock()
    result = run("ready", make_io(clock, FakeArm(clock)), tmp_path / "undisturbed", hold=5.0)
    return float(sent(result, "sent_t")[list(sent(result, "sent_phase")).index("hold")])


def test_stale_lowstate_holds_still_then_recovers_when_it_is_back(tmp_path):
    """Proves: LowState 0.5 s old during the hold gives exit 4 lowstate_stale, and while it is stale the arms are held on
    our last command at weight 1; once it is back (1 s later) the arms go home on the way out (from a hold pose,
    SAFE-PATHS.md) and ramp out at q0 (ruling F1).
    Catches: a stale limit loosened from 0.1 s (the arm steered on old angles), the old ramp-out where the arms are
    (Unitree's path then crosses the table), or a recover that starts before LowState is back [mutation]."""
    stale_at = hold_start_s(tmp_path) + 1.0
    clock = FakeClock()
    arm = FakeArm(clock)
    arm.stale_after, arm.stale_until = clock() + stale_at, clock() + stale_at + 1.0
    hang_guard = signals_at(clock, stale_at + HANG_GUARD_S)            # ends the hold a broken recover would never leave

    result = run("ready", make_io(clock, arm, signal_count=hang_guard), tmp_path, hold=5.0)

    phases, q, t = sent(result, "sent_phase"), sent(result, "sent_q"), sent(result, "sent_t")
    held, home = q[phases == "fault_hold"], q[phases == "to_factory"]
    assert (result.exit_code, result.reason) == (4, "lowstate_stale")
    assert np.allclose(held, READY17, atol=1e-9), "the hold is not our last command"
    assert t[phases == "to_factory"][0] >= stale_at + 1.0, "the recover started on stale angles"
    expected = arm_paths.path_ticks(arm_paths.way_out(READY17, FACTORY, READY17, pre.LIMITS_LO, pre.LIMITS_HI))
    assert np.allclose(home, expected, atol=1e-9), "the recover is not the way out"
    assert np.allclose(q[phases == "ramp_out"], FACTORY, atol=1e-9) and sent(result, "sent_weight")[-1] == 0.0


def test_stale_lowstate_that_never_returns_holds_until_the_operator_signals(tmp_path):
    """Proves: LowState that stays stale keeps the arms held at weight 1 past STALE_RETURN_WAIT_S - no recover, no
    ramp-out - until a stop signal, which hands them back there (rulings F1, F4).
    Catches: a recover on blind angles, or a ramp-out nobody asked for, which drops the arms on Unitree's path [mutation]."""
    stale_at = hold_start_s(tmp_path) + 1.0
    signal_at = stale_at + rs.STALE_RETURN_WAIT_S + 5.0
    clock = FakeClock()
    arm = FakeArm(clock)
    arm.stale_after = clock() + stale_at

    result = run("ready", make_io(clock, arm, signal_count=signals_at(clock, signal_at)), tmp_path, hold=5.0)

    phases, t, weight = sent(result, "sent_phase"), sent(result, "sent_t"), sent(result, "sent_weight")
    assert (result.exit_code, result.reason) == (4, "lowstate_stale")
    assert "to_factory" not in phases, "recovered on stale angles"
    assert t[(weight < 1.0) & (t > stale_at)].min() >= signal_at, "the weight dropped before the operator's signal"
    assert np.allclose(sent(result, "sent_q")[phases == "ramp_out"], READY17, atol=1e-9) and weight[-1] == 0.0


def test_second_ctrl_c_freezes_the_arms_and_a_third_hands_them_back(tmp_path):
    """Proves: one stop signal starts the slow way home; a second one freezes the arms where they are at weight 1 (exit 4
    second_signal); only a third ramps out, on the frozen target (ruling F4).
    Catches: a second Ctrl+C that is ignored (the operator cannot stop an arm heading for a person), that kills 86, or that
    ramps out at once and drops the arms on Unitree's own path [mutation]."""
    start = hold_start_s(tmp_path)
    clock = FakeClock()
    arm = FakeArm(clock)
    io = make_io(clock, arm, signal_count=signals_at(clock, start + 1.0, start + 3.0, start + 6.0))

    result = run("ready", io, tmp_path, hold=5.0)

    phases, q, t = list(sent(result, "sent_phase")), sent(result, "sent_q"), sent(result, "sent_t")
    frozen = q[np.array(phases) == "fault_hold"]
    assert (result.exit_code, result.reason) == (4, "second_signal")
    assert phases.index("to_start") < phases.index("fault_hold") < phases.index("ramp_out")
    assert np.ptp(frozen, axis=0).max() == 0.0 and len(frozen) >= 2.9 * rs.SEND_HZ, "the arms moved while frozen"
    assert t[phases.index("ramp_out")] >= start + 6.0, "ramped out before the third signal"
    assert np.allclose(q[np.array(phases) == "ramp_out"], frozen[0], atol=1e-9)
    assert sent(result, "sent_weight")[-1] == 0.0


def test_closed_terminal_after_stop_signal_still_hands_arms_back(tmp_path, monkeypatch):
    """Proves: when every print() fails after the stop signal (terminal closed = SIGHUP, or a broken pipe), the soft exit
    still ends at weight 0 (exit 3) and the message that could not be printed is kept in the sidecar.
    Catches: an exit path that dies on its first failed print and leaves the arms held at weight 1 (B2 review r1 #2)
    [mutation]."""
    clock = FakeClock()
    arm = FakeArm(clock)
    io = make_io(clock, arm, signal_count=signals_at(clock, 9.0))        # during the hold
    monkeypatch.setattr("builtins.print", print_fails_after_signal(io.signal_count))

    result = run("ready", io, tmp_path, hold=5.0)

    assert (result.exit_code, result.reason) == (3, "operator_stop")
    assert arm.messages[-1]["weight"] == 0.0
    assert any(text.startswith("stop requested") for text in read_sidecar(result)["unprinted_messages"])


def test_weight_step_above_0_02_refused_except_release_first_message(tmp_path, monkeypatch):
    """Proves: the runner sends release's first message (weight 0 -> 1) but refuses a later weight jump (1 -> 0.5) with a
    fault exit, so no message ever changes the weight by more than 0.02 except that first one (m6-8).
    Catches: a runner that skips the weight-step check, or grants the release exemption to every message [mutation]."""
    clock = FakeClock()
    arm = FakeArm(clock)
    jumpy_plan = [(1.0, FACTORY)] * 5 + [(0.5, FACTORY)] * 5            # a planner bug: weight 1 -> 0.5 in one message
    monkeypatch.setattr(rs, "release_plan", lambda measured, factory: jumpy_plan)
    operator_hands_back = signals_at(clock, 1.0)                         # weight_step holds (F5) until a signal

    result = run("release", make_io(clock, arm, signal_count=operator_hands_back), tmp_path)

    weights = np.array([m["weight"] for m in arm.messages])
    assert result.exit_code == 4 and result.reason.startswith("weight_step")
    assert weights[0] == 1.0 and weights[-1] == 0.0
    assert np.abs(np.diff(weights)).max() <= rs.MAX_WEIGHT_STEP + 1e-9


# --- policy phase ---------------------------------------------------------------------------------------------------------

def first_open_hand_message(phases: list, hand: FakeHand) -> int:
    """Index of the arm message with which the first open-hand command (0 at HANDS_OPEN_SPEED) went out. The runner sends
    the hand command once with every arm message from the first to_ready on (none during the ramp-in)."""
    first_hand_message = phases.index("to_ready")
    opened = next(k for k, (targets, speed) in enumerate(hand.sent)
                  if np.all(targets == 0.0) and speed == rs.HANDS_OPEN_SPEED)
    return first_hand_message + opened


def test_policy_exit_goes_through_ready_pose_before_q0(tmp_path):
    """Proves: the end of a policy phase goes back up to BALL_READY_POSE first, opens the hands there and waits
    HANDS_OPEN_S, then goes down to q0 and ramps out.
    Catches: an exit straight from a pose at the ball down to the hanging pose, through the table edge (RV6-1), or one
    that opens the hands only while the arms already descend, so a held ball drops on the way down (rule 12, B2 review
    r3 minor 2) [mutation, two]."""
    clock = FakeClock()
    arm, hands = FakeArm(clock), {"left": FakeHand(), "right": FakeHand()}
    policy = FakePolicy(demo_actions(shoulder_pitch=-0.6), mode="smolvla")
    io = make_io(clock, arm, hands, frames=FakeFrames(), policy=policy)

    result = run("run", io, tmp_path, speed=0.5, max_seconds=3.0)

    phases, q = list(sent(result, "sent_phase")), sent(result, "sent_q")
    last_policy = last_index(phases, "policy")
    after_policy = phases[last_policy + 1:]
    assert result.exit_code == 0
    assert after_policy[0] == "to_ready" and "to_start" in after_policy and after_policy[-1] == "ramp_out"
    assert np.abs(q[last_policy] - READY17).max() > 0.3, "policy never left the ready pose: the test proves nothing"
    assert np.allclose(q[last_index(phases, "to_ready")][3:], READY17[3:], atol=1e-9)
    opened_at = first_open_hand_message(phases, hands["left"])
    assert phases[opened_at] == "to_ready"
    assert phases.index("to_start") - opened_at >= HANDS_OPEN_S * rs.SEND_HZ, "hands not open at the ready pose for 2 s"


def test_envelope_streak_ends_policy_with_soft_exit(tmp_path):
    """Proves: a policy that keeps asking for a pose outside the demo envelope ends with a soft exit (code 3,
    envelope_streak) through the ready pose (rule 5).
    Catches: a runner that ignores the streak event and lets a policy press against the envelope for the whole run
    [mutation]."""
    clock = FakeClock()
    actions = demo_actions()
    actions[:, 1] = 1.0                        # left shoulder roll far outside the demo envelope (0.36 + margin)
    io = make_io(clock, FakeArm(clock), frames=FakeFrames(), policy=FakePolicy(actions, mode="smolvla"))

    result = run("run", io, tmp_path, speed=0.5, max_seconds=10.0)

    phases = list(sent(result, "sent_phase"))
    assert (result.exit_code, result.reason) == (3, "envelope_streak")
    assert phases[last_index(phases, "policy") + 1] == "to_ready"


def test_one_stale_hand_ends_run_with_soft_exit(tmp_path):
    """Proves: in `run`, the right hand's state turning 0.9 s old while the left one stays fresh ends the run softly
    (code 3, hand_state_stale): staleness is judged on the OLDER of the two hands.
    Catches: a check on the fresher hand, so the policy keeps steering on a hand state that stopped updating (rule 10,
    B2 review r3 minor 6) [mutation]."""
    clock = FakeClock()
    hands = {"left": FakeHand(clock=clock), "right": FakeHand(clock=clock)}
    hands["right"].stale_after = clock() + 0.5             # after the preflight, before the policy phase starts
    policy = FakePolicy(demo_actions(), mode="smolvla")
    io = make_io(clock, FakeArm(clock), hands, frames=FakeFrames(), policy=policy)

    result = run("run", io, tmp_path, speed=0.5, max_seconds=3.0)

    assert (result.exit_code, result.reason) == (3, "hand_state_stale")
    assert sent(result, "sent_weight")[-1] == 0.0


def test_replay_requests_advance_tick_with_scheduler(tmp_path):
    """Proves: each replay request carries tick = int(scheduler position), so the ticks rise through the demo.
    Catches: a replay stuck at tick 0 that plays the first 50 actions again and again (m6-5)."""
    clock = FakeClock()
    policy = FakePolicy(demo_actions())
    io = make_io(clock, FakeArm(clock), policy=policy)

    run("replay", io, tmp_path, speed=0.5, max_seconds=6.0, episode="replay_demo")

    assert policy.ticks[0] == 0 and len(policy.ticks) >= 3
    assert all(later > earlier for earlier, later in zip(policy.ticks, policy.ticks[1:]))
    assert policy.resets == [("replay_demo", policy_wire.noise_seed("replay_demo"))]


def test_replay_without_hand_state_fills_last_command(tmp_path):
    """Proves: a replay without hand state sends the last hand command in the hand slots and logs hand_state_synthetic.
    Catches: NaN hand slots that check_observation refuses, so a replay could never start without the hand service (m6-6)."""
    clock = FakeClock()
    policy = FakePolicy(demo_actions())
    hands = {"left": FakeHand(present=False), "right": FakeHand(present=False)}

    result = run("replay", make_io(clock, FakeArm(clock), hands, policy=policy), tmp_path, max_seconds=2.0,
                 episode="replay_demo")

    ready_hands = action_space.BALL_READY_POSE[14:26]
    assert np.allclose(policy.states[0][14:26], ready_hands)
    assert "hand_state_synthetic" in read_sidecar(result)["events"]


# --- sidecar --------------------------------------------------------------------------------------------------------------

def test_sidecar_marks_arm_writer_before_first_publish(tmp_path):
    """Proves: when 86 creates its rt/arm_sdk writer, the sidecar already says exit_code null and arm_sdk_writer true.
    Catches: a sidecar written only at exit - a run killed mid-motion would leave no trace for 84's overwrite rule (R2-2)
    [mutation]."""
    clock = FakeClock()
    arm = FakeArm(clock)
    seen = []
    arm.on_create = lambda: seen.extend(json.loads(p.read_text()) for p in tmp_path.glob("*.json"))

    run("ready", make_io(clock, arm), tmp_path, hold=0.2)

    assert len(seen) == 1
    assert seen[0]["exit_code"] is None and seen[0]["arm_sdk_writer"] is True
