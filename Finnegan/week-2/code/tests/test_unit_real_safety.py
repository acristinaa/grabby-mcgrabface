"""Unit: g1sim/real_safety.py - the pure safety logic 86 applies to every rt/arm_sdk message (PLAN.md 3.3, rules 4.2).

No DDS, no robot: every test feeds plain numpy vectors. A "17-vector" holds motors 12..28 (position i = motor 12 + i).
Tests marked [mutation] in PLAN.md were seen failing on a broken copy (runs/real_b1_mutation.log).
The per-message SafetyFilter is tested in tests/test_unit_real_safety_filter.py, chunk timing and staleness
(g1sim/real_timing.py) in tests/test_unit_real_timing.py.
"""
import json
import math
import pathlib

import numpy as np
import pytest

from g1sim import action_space
from g1sim import real_safety as rs

pytestmark = pytest.mark.unit

STATS = pathlib.Path.home() / "dev" / "g1-ext" / "datasets" / "g1_volleyball_walkup_v1" / "meta" / "stats.json"
ARM = 3                                                # position of motor 15 (left shoulder pitch) in a 17-vector
EPS = 1e-9                                             # float tolerance for step-size comparisons


def ready17(q0=np.zeros(17)):
    """BALL_READY_POSE as a 17-vector (waist roll/pitch from q0)."""
    return rs.sdk_targets(action_space.BALL_READY_POSE, q0)


def largest_joint_step(targets):
    """Largest single-joint change between two consecutive targets of a path."""
    return float(np.max(np.abs(np.diff(np.asarray(targets), axis=0))))


# --- 27-D action -> rt/arm_sdk motors ---------------------------------------------------------------------------------

def test_sdk_targets_maps_action_slots_to_motors():
    """Proves: motor 12 gets action 26 (waist yaw), motors 13/14 get q0, motors 15-28 get actions 0-13 in order (PLAN.md 3.3).
    Catches: swapped arms or the waist read from a hand slot - every other test builds target and measured with sdk_targets
    itself, and the envelope is mapped through it too, so a wrong mapping cancels out there (review R4-F2)."""
    action27, q0 = np.arange(27.0), np.arange(100.0, 117.0)

    target = rs.sdk_targets(action27, q0)

    assert target.tolist() == [26.0, 101.0, 102.0, *[float(j) for j in range(14)]]


# --- ramps and release ---------------------------------------------------------------------------------------------

def test_ramp_in_holds_q0_and_raises_weight_in_steps_below_0_02():
    """Proves: ramp_in commands the frozen q0 while the weight rises 0 -> 1 in small steps.
    Catches: a ramp that jumps the weight (arm snaps to our target) or changes the target during the ramp."""
    q0 = np.linspace(-0.5, 0.5, 17)

    plan = rs.ramp_in(q0)

    weights = np.array([w for w, _ in plan])
    assert weights[0] == 0.0 and weights[-1] == 1.0
    assert np.all(np.diff(weights) >= 0) and np.max(np.diff(weights)) <= rs.MAX_WEIGHT_STEP
    assert all(np.array_equal(q, q0) for _, q in plan)


def test_ramp_in_target_stays_frozen_when_the_measured_buffer_changes():
    """Proves: ramp_in keeps the q0 it was given, even when the caller's array is overwritten later (review N1).
    Catches: a ramp that aliases a reused LowState buffer and so chases the sagging arm instead of holding q0."""
    q0 = np.linspace(-0.5, 0.5, 17)
    original = q0.copy()

    plan = rs.ramp_in(q0)
    q0[:] = 0.9                                        # the caller reuses its buffer for the next LowState reading

    assert all(np.array_equal(q, original) for _, q in plan)


def test_ramp_out_holds_target_and_lowers_weight_in_steps_below_0_02():
    """Proves: ramp_out commands the frozen hold pose while the weight falls 1 -> 0 in steps of at most 0.02 (rule 3).
    Catches: a fast ramp-out that drops the arm into Unitree's controller with a jerk, or a target that moves meanwhile."""
    q_hold = np.linspace(-0.5, 0.5, 17)

    plan = rs.ramp_out(q_hold, 1.0)

    weights = np.array([w for w, _ in plan])
    assert weights[0] == 1.0 and weights[-1] == 0.0
    assert np.all(np.diff(weights) <= 0) and np.max(np.abs(np.diff(weights))) <= 0.02
    assert all(np.array_equal(q, q_hold) for _, q in plan)


def test_ramp_out_from_weight_0_4_starts_at_0_4():
    """Proves: a ramp-out that begins mid ramp-in (weight 0.4) starts at 0.4 and falls to 0 in steps of at most 0.02.
    Catches: a ramp-out that ignores weight_now and starts at 1.0: B2's weight-step guard then refuses the jump, the exit
    ends in weight_step faults instead of a ramp-out, and the arms stay held (review R4-F5)."""
    q_hold = np.linspace(-0.5, 0.5, 17)

    plan = rs.ramp_out(q_hold, 0.4)

    weights = np.array([w for w, _ in plan])
    assert weights[0] == 0.4 and weights[-1] == 0.0
    assert np.max(np.abs(np.diff(weights))) <= 0.02


def test_release_first_message_is_full_weight_at_measured_angles():
    """Proves: release starts with weight 1 at the measured angles (Unitree arm7 example line 135).
    Catches: a release that starts at weight 0 or at the factory pose, which jumps the arm if Unitree latched our last message."""
    measured, factory = np.full(17, 0.4), np.zeros(17)

    weight, q = rs.release_plan(measured, factory)[0]

    assert weight == 1.0 and np.array_equal(q, measured)


def test_release_moves_to_factory_pose_at_0_3_rad_s_before_weight_drops():
    """Proves: release walks to the factory pose at <= 0.3 rad/s with full weight, and only then ramps out there.
    Catches: a ramp-out straight from the measured angles, which lets the controller swing the arm home fast (review N2)."""
    measured, factory = np.full(17, 0.4), np.zeros(17)

    plan = rs.release_plan(measured, factory)

    first_drop = next(i for i, (w, _) in enumerate(plan) if w < 1.0)
    assert np.allclose(plan[first_drop - 1][1], factory)
    assert all(np.allclose(q, factory) for _, q in plan[first_drop:])
    assert largest_joint_step([q for _, q in plan]) <= rs.SCRIPTED_SPEED / rs.SEND_HZ + EPS
    assert plan[-1][0] == 0.0


# --- soft exit -----------------------------------------------------------------------------------------------------

def test_soft_exit_from_policy_passes_through_ready_pose():
    """Proves: a soft exit tags every target up to the ready waypoint "to_ready" (the hands open there) and the rest
    "to_start" down to q0, at <= 0.3 rad/s.
    Catches: an exit straight from a pose at the ball down to q0, which can drag the arm through the table edge (RV6-1)."""
    q0 = np.zeros(17)
    current = ready17(q0) + 0.3

    plan = rs.soft_exit_plan([current, ready17(q0), q0], q0, ready_index=1)

    phases = [p for p, _, _ in plan]
    assert phases[0] == "to_ready"
    last_ready = max(i for i, p in enumerate(phases) if p == "to_ready")
    assert set(phases[last_ready + 1:]) == {"to_start"}
    assert np.allclose(plan[last_ready][2], ready17(q0))
    assert np.allclose(plan[-1][2], q0)
    assert largest_joint_step([q for _, _, q in plan]) <= rs.SCRIPTED_SPEED / rs.SEND_HZ + EPS


def test_soft_exit_path_is_never_truncated():
    """Proves: a 2.0 rad gap still ends exactly at q0, however long the path takes (m6-9).
    Catches: a time cap that stops the path early, after which the ramp-out swings the rest of the gap."""
    q0 = np.zeros(17)
    current = q0.copy()
    current[ARM] = 2.0

    plan = rs.soft_exit_plan([current, q0], q0, ready_index=0)

    assert np.array_equal(plan[-1][2], q0)
    assert rs.exit_path_seconds(plan) >= 2.0 / rs.SCRIPTED_SPEED


# --- hands ---------------------------------------------------------------------------------------------------------

def test_hands_never_envelope_clipped():
    """Proves: hand targets are only capped at HAND_CAP, never clipped to the demo envelope (m6-7).
    Catches: a grasp command the envelope (demos closed to 0.15-0.3 only) silently weakens."""
    action = np.zeros(27)
    action[14:26] = 0.5                                # above the envelope top of every hand slot (0.25-0.4)

    left, right = rs.hand_targets(action)

    assert np.allclose(left, 0.5) and np.allclose(right, 0.5)


def test_hand_targets_capped_at_0_6():
    """Proves: hand targets are clipped to [0, HAND_CAP] (ruling R6).
    Catches: a fully closed fist on the real BrainCo hand, which the demos never commanded."""
    action = np.zeros(27)
    action[14:20], action[20:26] = 0.9, -0.2

    left, right = rs.hand_targets(action)

    assert np.allclose(left, rs.HAND_CAP) and np.allclose(right, 0.0)


# --- faults, staleness, refusals -------------------------------------------------------------------------------------

def test_fault_targets_measured_on_contact_last_command_on_stale_lowstate():
    """Proves: a fault ramps out at the measured angles, except stale LowState, which uses our last command (review M2).
    Catches: a contact fault that keeps pushing towards the blocked target, or a stale-LowState exit on old angles."""
    measured, last = np.full(17, 0.1), np.full(17, 0.3)

    on_contact = rs.fault_targets("contact_fault", measured, last)
    on_stale = rs.fault_targets("lowstate_stale", measured, last)

    assert np.array_equal(on_contact, measured) and np.array_equal(on_stale, last)


def test_fault_targets_fall_back_to_last_command_when_measured_is_nan():
    """Proves: a contact fault with a NaN measured angle holds our last command instead (rule 11).
    Catches: a fault ramp-out that sends the NaN LowState angle straight back to the motor."""
    measured, last = np.full(17, 0.1), np.full(17, 0.3)
    measured[ARM] = math.nan

    held = rs.fault_targets("contact_fault", measured, last)

    assert np.array_equal(held, last)


def test_exit_kind_faults_on_lowstate_contact_nan_and_second_signal():
    """Proves: the four fault reasons give a fault exit, every other reason a soft exit (rule 12).
    Catches: a stale LowState treated as soft, so the arm walks a long scripted path on blind angles."""
    faults = ("lowstate_stale", "contact_fault", "invalid_target", "second_signal")
    softs = ("done", "signal", "camera_stale", "policy_stale", "hand_state_stale", "envelope_streak", "chunk_too_late")

    assert all(rs.exit_kind(r) == "fault" for r in faults) and all(rs.exit_kind(r) == "soft" for r in softs)


def test_speed_above_0_5_and_waist_above_0_6_refused_off_loopback():
    """Proves: off loopback a speed factor above 0.5 and a waist limit above 0.6 rad are refused; the sim allows them.
    Catches: a fast VLA run or an unchecked waist swing on the real robot (gate (e), ruling R10)."""
    assert rs.speed_refusal(0.6, on_loopback=False) is not None
    assert rs.speed_refusal(0.5, on_loopback=False) is None
    assert rs.speed_refusal(1.0, on_loopback=True) is None
    assert rs.waist_max_refusal(0.61, on_loopback=False) is not None
    assert rs.waist_max_refusal(0.6, on_loopback=False) is None
    assert rs.waist_max_refusal(1.0, on_loopback=True) is None


def test_zero_or_negative_speed_and_negative_waist_max_refused():
    """Proves: --speed 0 or below and a negative --waist-max are refused, on the robot and on loopback; --waist-max 0 is allowed.
    Catches: a negative waist limit (np.clip with lo > hi returns hi, so the waist turns to -0.15 rad from a target of 0),
    or a speed of 0 that never advances the policy."""
    assert rs.speed_refusal(0.0, on_loopback=True) is not None
    assert rs.speed_refusal(-0.5, on_loopback=False) is not None
    assert rs.waist_max_refusal(-0.15, on_loopback=False) is not None
    assert rs.waist_max_refusal(-0.15, on_loopback=True) is not None
    assert rs.waist_max_refusal(0.0, on_loopback=False) is None


def test_nan_speed_and_waist_max_refused():
    """Proves: --speed nan and --waist-max nan are refused, on the robot and on loopback (argparse float accepts 'nan').
    Catches: a NaN that passes every '>' check (NaN > 0.5 is False) and then reaches the filter."""
    assert rs.speed_refusal(math.nan, on_loopback=False) is not None
    assert rs.speed_refusal(math.nan, on_loopback=True) is not None
    assert rs.waist_max_refusal(math.nan, on_loopback=False) is not None
    assert rs.waist_max_refusal(math.nan, on_loopback=True) is not None


def test_factory_pose_gap_treats_nan_as_infinite():
    """Proves: a NaN in the measured angles gives an infinite factory-pose gap.
    Catches: max() of a NaN difference comparing False against the 0.10 rad tolerance, so the gap check silently passes."""
    measured = np.zeros(17)
    measured[ARM] = math.nan

    assert rs.factory_pose_gap(measured, np.zeros(17)) == math.inf


def test_factory_pose_gap_is_the_largest_joint_difference():
    """Proves: the factory-pose gap is the largest single-joint difference: joints 0.3 and 0.1 rad off, the rest equal -> 0.3.
    Catches: the smallest (0.0), a sum (0.4) or a mean, which lets 86 start with one arm still held away from the factory
    pose, so the controller swings it back fast on ramp-in (rule 2, R2-2, review R4-F3)."""
    factory = np.linspace(-0.2, 0.2, 17)
    measured = factory.copy()
    measured[ARM] += 0.3
    measured[ARM + 7] -= 0.1                            # the same joint of the right arm, 0.1 rad off the other way

    gap = rs.factory_pose_gap(measured, factory)

    assert gap == pytest.approx(0.3)


# --- pinned constants ----------------------------------------------------------------------------------------------

# Contract table of PLAN.md 3.3, copied as literals: the tests above check the logic, this one checks the numbers
PLAN_NUMBERS = {
    "SEND_HZ": 50,                             # Unitree arm7 example, line 69
    "RAMP_SECONDS": 2.0,                       # 72_real_check.py:76-89
    "MAX_WEIGHT_STEP": 0.02,                   # m6-8
    "SCRIPTED_SPEED": 0.3,                     # rad/s, release and soft exits (review N2)
    "ARM_SPEED_MAX": 1.0,                      # rad/s (rule 6)
    "WAIST_SPEED_MAX": 0.3,                    # rad/s (rule 6)
    "LIMIT_MARGIN": 0.05,                      # rad inside the joint limits (rule 5)
    "ENVELOPE_STREAK_TICKS": 30,               # rule 5
    "LEAD_LIMIT": 0.25,                        # rad (rule 7)
    "LEAD_TICKS": 13,                          # 0.25 s at 50 Hz, rounded up (review M2/n2)
    "HAND_CAP": 0.6,                           # ruling R6
    "HAND_SPEED": 0.5,                         # BrainCo speed and cap for policy hand targets (ruling R6)
    "HANDS_OPEN_SPEED": 0.3,                   # BrainCo speed for hands-open and the soft exit (ruling R6)
    "ENVELOPE_MARGIN": 0.10,                   # rad around the demo action min/max (PLAN.md 2.8)
    "EXIT_PATH_MARGIN_S": 1.0,                 # s past the planned exit path before the stuck-exit fault (m6-9)
    "ROBOT_MAX_SPEED_FACTOR": 0.5,             # motion gate (e)
    "ROBOT_WAIST_MAX_DEFAULT": 0.15,           # rad (ruling R10)
    "ROBOT_WAIST_MAX_CEILING": 0.6,            # rad (ruling R10)
    "POLICY_TIMEOUT_S": 2.0,                   # rule 10
    "STALE_S": {"lowstate": 0.1, "camera": 1.0, "hand_state": 0.5, "chunk_ran_out": 1.0},   # rule 10
}


def test_safety_numbers_equal_the_plan():
    """Proves: every safety number in real_safety equals the PLAN.md 3.3 contract table (the chunk-timing numbers are pinned
    in tests/test_unit_real_timing.py, the arrays SIM_GAINS and ENVELOPE in the next test).
    Catches: a quiet "tuning" of a constant (e.g. release 10x faster, contact fault after 1 s) that every logic test follows."""
    actual = {name: getattr(rs, name) for name in PLAN_NUMBERS}

    assert actual == PLAN_NUMBERS


def test_envelope_and_gains_pinned_to_stats_and_table_env():
    """Proves: SIM_GAINS equal table_env.KP/KD of motors 12-28 and the envelope equals stats.json min/max -/+ 0.10.
    Catches: gains above what the policy was trained with, or an envelope copied from the wrong dataset."""
    table_env = pytest.importorskip("g1sim.table_env", reason="g1sim.table_env needs the sim env ~/dev/g1-ext/.venv")
    assert np.array_equal(rs.SIM_GAINS_KP, table_env.KP[12:29]) and np.array_equal(rs.SIM_GAINS_KD, table_env.KD[12:29])
    if not STATS.exists():
        pytest.skip(f"{STATS} not found (dataset on the internal disk); gains checked, envelope not")
    stats = json.loads(STATS.read_text())["action"]

    assert np.allclose(rs.ENVELOPE_LO, np.array(stats["min"]) - rs.ENVELOPE_MARGIN, atol=1e-4)
    assert np.allclose(rs.ENVELOPE_HI, np.array(stats["max"]) + rs.ENVELOPE_MARGIN, atol=1e-4)
