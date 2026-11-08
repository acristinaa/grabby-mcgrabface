"""Real G1: the safety logic for every rt/arm_sdk message 86 sends - ramps, release, exits, limits, faults (PLAN.md 3.3, 4.2).

Why it exists: on the robot day non-authors run a sim-trained policy on a borrowed G1. Every command passes through the
plans and the SafetyFilter here, so a wrong policy output can never move a joint into its end stop, faster than the rate
limit, or the waist roll/pitch away from where we found them; and every way out of a run (soft exit, fault, release) is
a slow, known path. Pure numpy logic, no DDS and no I/O, so every rule is unit-tested without a robot.
Chunk timing and staleness live in g1sim/real_timing.py and are re-exported here (one contract, PLAN.md 3.3).
A "17-vector" holds values for motors 12..28 in that order (position i = motor 12 + i).
"""
import math

import numpy as np

from g1sim.real_timing import (  # noqa: F401 - part of the real_safety contract (PLAN.md 3.3)
    POLICY_TIMEOUT_S, SEND_HZ, STALE_S, ChunkScheduler, ChunkTooLate, InvalidTarget, require_finite, stale_reason)

SDK_MOTORS = tuple(range(12, 29))       # Unitree arm7 example lines 40-64 (same literal as real_io; a B2 test pins them equal)
WAIST_YAW_POS, WAIST_HOLD_POS = 0, (1, 2)   # positions of motors 12 and 13/14 in a 17-vector
ARMS = slice(3, 17)                     # positions of motors 15-28 in a 17-vector
RAMP_SECONDS = 2.0                      # weight 0<->1 smoothstep (72_real_check.py:76-89)
MAX_WEIGHT_STEP = 0.02                  # per message in ramp_in/ramp_out; smoothstep over 100 ticks peaks at 0.015 (m6-8: B2's
                                        # runner checks it on every message; release's first message is the one exception)
SCRIPTED_SPEED = 0.3                    # rad/s for every scripted move (review N2)
ARM_SPEED_MAX, WAIST_SPEED_MAX = 1.0, 0.3   # rad/s command rate limit in every phase (rule 6)
LIMIT_MARGIN = 0.05                     # rad inside JOINT_LIMITS (rule 5)
ENVELOPE_MARGIN = 0.10                  # rad around the demo action min/max (PLAN.md 2.8)
ENVELOPE_STREAK_TICKS = 30              # clipped policy ticks in a row -> soft exit (rule 5)
LEAD_LIMIT = 0.25                       # rad between command and measured angle (rule 7)
LEAD_TICKS = 13                         # 0.25 s at 50 Hz, rounded up (review M2/n2)
HAND_CAP = 0.6                          # BrainCo target cap, demos never exceed 0.3 (ruling R6)
HAND_SPEED = 0.5                        # BrainCo speed for policy hand targets (ruling R6)
HANDS_OPEN_SPEED = 0.3                  # BrainCo speed for `hands-open` and the soft exit (ruling R6)
ROBOT_MAX_SPEED_FACTOR = 0.5            # largest --speed off loopback (motion gate (e))
ROBOT_WAIST_MAX_DEFAULT, ROBOT_WAIST_MAX_CEILING = 0.15, 0.6   # rad, --waist-max default and ceiling (ruling R10)
# (m6-9) no time cap on the soft-exit path: a cap could cut the path short and the ramp-out would then swing the rest of the gap
EXIT_PATH_MARGIN_S = 1.0                # the runner faults if executing the exit path takes > planned duration + this
FLOAT_TOLERANCE = 1e-9                  # rad; a gap that is an exact multiple of the step must not get an extra tick

# table_env.KP/KD[12:29]: waist 12-14, left arm 15-21, right arm 22-28 (LeRobot G1 gains the policy was trained with)
SIM_GAINS_KP = np.array([250, 250, 250] + [50, 50, 80, 80, 40, 40, 40] * 2, dtype=float)
SIM_GAINS_KD = np.array([5, 5, 5] + [3, 3, 3, 3, 1.5, 1.5, 1.5] * 2, dtype=float)
# 27-D action order (action_space.py): $DATASET/meta/stats.json action min - 0.10 / max + 0.10, rounded outward to 1e-4 rad
ENVELOPE_LO = np.array([-1.7209, -0.2105, -0.1, -0.9001, -0.4924, -0.2123, -0.2616,          # left arm
                        -1.7065, -0.3443, -0.777, -0.9001, -0.1916, -0.2166, -0.3815,        # right arm
                        *[-0.1] * 12,                                                        # hands (never applied, m6-7)
                        -0.8137])                                                            # waist yaw
ENVELOPE_HI = np.array([-0.15, 0.36, 0.752, 1.5248, 0.1811, 0.8826, 0.4046,
                        -0.15, 0.2444, 0.1, 1.5226, 0.4867, 0.8552, 0.2472,
                        *[0.4, 0.4, 0.25, 0.25, 0.25, 0.25] * 2,
                        0.7798])

PHASES = ("release", "ramp_in", "to_ready", "hold", "shadow", "policy", "to_start", "to_factory", "fault_hold",
          "ramp_out")
ENVELOPE_PHASES = ("policy",)           # replay and VLA both run in "policy" (review B1)
EXIT_CODES = {"done": 0, "refused": 2, "soft": 3, "fault": 4}
FAULT_REASONS = ("lowstate_stale", "contact_fault", "invalid_target", "second_signal")   # rule 12: these end in a fault exit

# What the arms do after a fault, per reason (.planning/real-robot/SAFE-PATHS.md "What the arms do per fault reason").
# Every value is a RULING for Finnegan (F1-F5); the values here are the proposed defaults. RAMP_OUT is the old rule 12:
# Unitree's controller then takes the arms home on its own path, 15.7-16.2 cm through the table from a policy pose (87).
# HOLD keeps weight 1 on a still target until the operator acts (owner's remote / e-stop; one more stop signal ramps out
# there). RECOVER holds briefly, then drives our waypoint path to the factory pose and ramps out there.
RAMP_OUT, HOLD, RECOVER = "ramp_out", "hold", "recover"
FAULT_ACTIONS = {"lowstate_stale": RECOVER,     # F1: hold until LowState is back, then recover; else keep holding
                 "contact_fault": RECOVER,      # F2: stop pushing, short hold, recover at half speed
                 "invalid_target": RECOVER,     # F3: LowState is fine, the arm sits at the last valid command
                 "second_signal": HOLD,         # F4: "stop" means hold still; the next signal ramps out
                 "internal_error": HOLD,        # F5: these four say nothing about where the arm is wrong
                 "exit_path_overrun": HOLD,
                 "weight_step": HOLD,
                 "signal_in_release": HOLD}
FAULT_ACTION_UNKNOWN = HOLD             # a reason missing above: holding still is the one action that moves nothing
# Reasons whose hold target is our last command, not the measured angles: stale LowState (old angles, F1) and the F5
# reasons ("hold at weight 1 on the last command"). Contact, NaN and a second signal hold the measured angles (review M2).
HOLD_LAST_COMMAND = ("lowstate_stale", "internal_error", "exit_path_overrun", "weight_step", "signal_in_release")
RECOVER_HOLD_S = 0.5                    # s at the hold target before a recover starts (F2 "short hold"; ruling F2)
STALE_RETURN_WAIT_S = 3.0               # s F1 waits for fresh LowState before it only holds (ruling F1 "a few seconds")
CONTACT_RECOVER_SPEED = SCRIPTED_SPEED / 2   # rad/s, F2: a recover after contact moves at half speed (0.15)


# --- 27-D <-> 17-vector ----------------------------------------------------------------------------------------------

def sdk_targets(action27, q0) -> np.ndarray:
    """27-D action -> 17-vector for rt/arm_sdk: motor 12 = waist yaw, 13/14 = q0 (held, rule 4), 15-28 = the two arms."""
    action27, q0 = np.asarray(action27, dtype=float), np.asarray(q0, dtype=float)
    target = np.empty(17)
    target[WAIST_YAW_POS] = action27[26]
    target[list(WAIST_HOLD_POS)] = q0[list(WAIST_HOLD_POS)]
    target[ARMS] = action27[0:14]
    return target


def hand_targets(action27) -> tuple[np.ndarray, np.ndarray]:
    """27-D action -> (left6, right6) BrainCo targets in [0, HAND_CAP]. Never envelope-clipped (m6-7): a grasp needs them."""
    action27 = require_finite(action27, "hand targets")
    capped = np.clip(action27[14:26], 0.0, HAND_CAP)
    return capped[:6], capped[6:]


def _envelope17(lo27: np.ndarray, hi27: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The 27-D envelope in 17-vector positions; waist roll/pitch are unbounded here because they are held at q0 anyway."""
    return sdk_targets(lo27, np.full(17, -np.inf)), sdk_targets(hi27, np.full(17, np.inf))


# --- scripted plans --------------------------------------------------------------------------------------------------

def smoothstep_weights(start: float, end: float, seconds: float = RAMP_SECONDS) -> np.ndarray:
    """Weights from `start` to `end`, one per 50 Hz tick, monotone, smooth at both ends, last value exactly `end`.

    Smooth (no jerk) because the weight blends our target against Unitree's controller; a step would snap the arm.
    """
    for w in (start, end):
        if not 0.0 <= w <= 1.0:
            raise ValueError(f"arm_sdk weight {w} is outside [0, 1] - this is a bug in the caller, do not run motion")
    ticks = round(seconds * SEND_HZ)
    x = np.arange(ticks + 1) / ticks
    weights = start + (end - start) * (3 * x**2 - 2 * x**3)
    weights[-1] = end
    return weights


def ramp_in(q0) -> list[tuple[float, np.ndarray]]:
    """(weight 0 -> 1, q0) pairs. q0 is read once and frozen for the whole ramp (review N1): a re-read would chase the
    sagging arm and the blend would drop it further."""
    frozen = np.array(q0, dtype=float)
    frozen.setflags(write=False)
    return [(float(w), frozen) for w in smoothstep_weights(0.0, 1.0)]


def ramp_out(q_hold, weight_now: float) -> list[tuple[float, np.ndarray]]:
    """(weight_now -> 0, q_hold) pairs: hands the arms back to Unitree's controller while we hold still."""
    frozen = np.array(q_hold, dtype=float)
    frozen.setflags(write=False)
    return [(float(w), frozen) for w in smoothstep_weights(weight_now, 0.0)]


def move(start, goal, speed: float = SCRIPTED_SPEED) -> list[np.ndarray]:
    """Straight joint-space line start -> goal; no joint moves more than speed / SEND_HZ per tick; the last entry is goal.

    Straight in joint space so Track C's 87 can check exactly this line against the table (RV6-1).
    """
    start = require_finite(start, "move start")
    goal = require_finite(goal, "move goal")
    max_step = speed / SEND_HZ
    ticks = max(1, math.ceil(np.max(np.abs(goal - start)) / max_step - FLOAT_TOLERANCE))
    path = [start + (goal - start) * k / ticks for k in range(1, ticks)]
    return path + [goal.copy()]


def move_through(waypoints, speed: float = SCRIPTED_SPEED) -> list[tuple[int, np.ndarray]]:
    """(leg, target) pairs along waypoint legs: leg k (from 1) is move(waypoints[k - 1], waypoints[k], speed).

    The one place where waypoints become 50 Hz targets, for 86 and for 87 (g1sim/arm_paths.path_ticks): every leg stays a
    straight joint line, so 87 checks exactly what 86 sends. The leg number lets 86 log where on a plan the arm is.
    """
    return [(leg, q) for leg, (start, goal) in enumerate(zip(waypoints, waypoints[1:]), start=1)
            for q in move(start, goal, speed)]


def release_plan(measured, factory) -> list[tuple[float, np.ndarray]]:
    """`86 release`: weight 1 at the measured angles, walk to the recorded factory pose at 0.3 rad/s, then ramp out there.

    The first message is the one allowed weight jump (Unitree arm7 example line 135): at the measured angles it moves
    nothing, whether or not Unitree latched our last message. Ramping out only at the factory pose means the controller
    has no gap left to swing through (reviews M1, N2).
    """
    measured = np.array(measured, dtype=float)
    to_factory = [(1.0, q) for q in move(measured, factory)]
    return [(1.0, measured)] + to_factory + ramp_out(factory, 1.0)


def soft_exit_plan(waypoints, q0, ready_index: int) -> list[tuple[str, int, np.ndarray]]:
    """(phase, leg, target) of a soft exit along `waypoints` (g1sim/arm_paths.py): "to_ready" on the legs up to
    waypoints[ready_index], the ready pose where the hands open; "to_start" on the rest, down to q0. ready_index 0 = no
    "to_ready" part (an exit that starts on the way in goes back along it).

    Waypoints, not straight lines (SAFE-PATHS.md): a straight line from a pose at the ball up to the ready pose, or from
    the ready pose down to q0, sweeps the low hands through stand, basket or table edge (87). No time cap (m6-9). 13/14
    stay q0.
    """
    q0 = np.asarray(q0, dtype=float)
    held = [_hold_waist_roll_pitch(waypoint, q0) for waypoint in waypoints]
    return [("to_ready" if leg <= ready_index else "to_start", leg, q) for leg, q in move_through(held)]


def exit_path_seconds(plan) -> float:
    """Planned duration of a path at 50 Hz; the runner logs it and faults past it + EXIT_PATH_MARGIN_S (a stuck loop)."""
    return len(plan) / SEND_HZ


def _hold_waist_roll_pitch(q17, q0: np.ndarray) -> np.ndarray:
    held = np.array(q17, dtype=float)
    held[list(WAIST_HOLD_POS)] = q0[list(WAIST_HOLD_POS)]
    return held


# --- refusals, faults ------------------------------------------------------------------------------------------------

def factory_pose_gap(measured17, factory17) -> float:
    """Largest |measured - factory| in rad. A NaN counts as an infinite gap, so the 0.10 rad check refuses instead of
    passing silently (NaN > 0.10 is False)."""
    gap = np.abs(np.asarray(measured17, dtype=float) - np.asarray(factory17, dtype=float))
    return float(np.max(gap)) if np.all(np.isfinite(gap)) else math.inf


def speed_refusal(speed: float, on_loopback: bool) -> str | None:
    """Refusal message for --speed, or None. Off loopback at most 0.5 (gate (e)): slow is what makes Ctrl+C in time."""
    if not math.isfinite(speed):
        return f"--speed {speed} is not a number - use --speed 0.25 or 0.5 (runbook)"
    if speed <= 0:
        return f"--speed {speed} must be above 0 (the policy would never advance)"
    if not on_loopback and speed > ROBOT_MAX_SPEED_FACTOR:
        return f"--speed {speed} is above {ROBOT_MAX_SPEED_FACTOR} on the robot - use --speed 0.25 or 0.5 (runbook)"
    return None


def waist_max_refusal(waist_max: float, on_loopback: bool) -> str | None:
    """Refusal message for --waist-max, or None. Off loopback at most 0.6 rad (ruling R10): the waist swings the torso."""
    if not math.isfinite(waist_max):
        return f"--waist-max {waist_max} is not a number - use 0.15, 0.35 or 0.6"
    if waist_max < 0:
        return f"--waist-max {waist_max} must be 0 or more (use --waist-hold to keep the waist still)"
    if not on_loopback and waist_max > ROBOT_WAIST_MAX_CEILING:
        return f"--waist-max {waist_max} is above the robot ceiling {ROBOT_WAIST_MAX_CEILING} rad - use 0.15, 0.35 or 0.6"
    return None


def exit_kind(reason: str) -> str:
    """'fault' (fault_action decides what the arms do) for the four fault reasons of rule 12, else 'soft' (slow path home)."""
    return "fault" if reason in FAULT_REASONS else "soft"


def reason_key(reason: str) -> str:
    """The fault reason without its detail: "internal_error: KeyError: 3" -> "internal_error"."""
    return reason.split(":", 1)[0]


def fault_action(reason: str) -> str:
    """RAMP_OUT, HOLD or RECOVER for a fault reason (FAULT_ACTIONS, rulings F1-F5)."""
    return FAULT_ACTIONS.get(reason_key(reason), FAULT_ACTION_UNKNOWN)


def fault_targets(reason: str, measured17, last_command17) -> np.ndarray:
    """Target held in a fault: the measured angles, so a blocked arm stops pushing (review M2). For the reasons in
    HOLD_LAST_COMMAND (stale LowState: the measured angles are old) and when a measured angle is NaN (never sent, rule
    11): our last command."""
    measured_usable = (reason_key(reason) not in HOLD_LAST_COMMAND
                       and np.all(np.isfinite(np.asarray(measured17, dtype=float))))
    source = measured17 if measured_usable else last_command17
    return np.array(source, dtype=float)


# --- the per-message filter ------------------------------------------------------------------------------------------

class SafetyFilter:
    """Turns a wanted 17-vector into the command actually sent, and reports safety events (rule 5-8).

    Order per message: NaN check, waist hold or clip, demo envelope (policy phase only), joint limits, 13/14 := q0,
    rate limit against the last command, lead monitor (weight 1 only). Events: "limit_clip", "envelope_clip",
    "envelope_streak" (30 clipped policy ticks in a row), "contact_fault" (lead > 0.25 rad, or a NaN angle, for 13 ticks).
    """

    def __init__(self, limits_lo17, limits_hi17, q0, waist_max: float, waist_hold: bool):
        self.q0 = np.array(q0, dtype=float)
        self.lo = np.asarray(limits_lo17, dtype=float) + LIMIT_MARGIN
        self.hi = np.asarray(limits_hi17, dtype=float) - LIMIT_MARGIN
        self.env_lo, self.env_hi = _envelope17(ENVELOPE_LO, ENVELOPE_HI)
        self.waist_max, self.waist_hold = waist_max, waist_hold
        self.max_step = np.full(17, ARM_SPEED_MAX / SEND_HZ)
        self.max_step[:3] = WAIST_SPEED_MAX / SEND_HZ          # waist 12-14
        self.last = self.q0.copy()                             # every run starts from the frozen q0 (ramp_in)
        self.envelope_streak = 0
        self.lead_ticks = 0

    def step(self, target17, measured17, phase: str, weight: float) -> tuple[np.ndarray, list[str]]:
        """Filtered command for one 50 Hz message plus the events it raised. Raises InvalidTarget on NaN/Inf."""
        if phase not in PHASES:
            raise ValueError(f"unknown phase {phase!r} - expected one of {PHASES}")
        cmd = require_finite(target17, "arm target").copy()
        events = []
        self._limit_waist(cmd)
        self._apply_envelope(cmd, phase, events)
        if np.any(cmd < self.lo) or np.any(cmd > self.hi):
            events.append("limit_clip")
            cmd = np.clip(cmd, self.lo, self.hi)
        cmd[list(WAIST_HOLD_POS)] = self.q0[list(WAIST_HOLD_POS)]
        cmd = self.last + np.clip(cmd - self.last, -self.max_step, self.max_step)
        require_finite(cmd, "filtered arm command")   # a NaN limit (e.g. --waist-max nan) must never reach a motor (rule 11)
        self.last = cmd.copy()
        self._monitor_lead(cmd, measured17, weight, events)
        return cmd, events

    def reset_last(self, command17) -> None:
        """Set the command the rate limit measures from; used when a scripted move hands over to the policy phase."""
        self.last = np.array(command17, dtype=float)

    def _limit_waist(self, cmd: np.ndarray) -> None:
        if self.waist_hold:
            cmd[WAIST_YAW_POS] = self.q0[WAIST_YAW_POS]
        else:
            cmd[WAIST_YAW_POS] = np.clip(cmd[WAIST_YAW_POS], -self.waist_max, self.waist_max)

    def _apply_envelope(self, cmd: np.ndarray, phase: str, events: list[str]) -> None:
        if phase not in ENVELOPE_PHASES:
            self.envelope_streak = 0
            return
        clipped = np.clip(cmd, self.env_lo, self.env_hi)
        if np.array_equal(clipped, cmd):
            self.envelope_streak = 0
            return
        cmd[:] = clipped
        events.append("envelope_clip")
        self.envelope_streak += 1
        if self.envelope_streak >= ENVELOPE_STREAK_TICKS:
            events.append("envelope_streak")

    def _monitor_lead(self, cmd: np.ndarray, measured17, weight: float, events: list[str]) -> None:
        # Only at full weight: during a ramp the blend makes command and measured differ by design (review N2)
        if weight != 1.0:
            self.lead_ticks = 0
            return
        gap = np.abs(cmd - np.asarray(measured17, dtype=float))
        # A NaN angle counts as over the limit: NaN > 0.25 is False and would silently switch the monitor off (fail closed)
        lead = float(np.max(gap)) if np.all(np.isfinite(gap)) else math.inf
        self.lead_ticks = self.lead_ticks + 1 if lead > LEAD_LIMIT else 0
        if self.lead_ticks >= LEAD_TICKS:
            events.append("contact_fault")
