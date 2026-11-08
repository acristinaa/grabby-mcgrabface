"""Real G1: how an 86 run ends - the exit request, the slow way home (soft exit) and the fault exit (hold, recover or
ramp out, by reason).

Why it exists: every way out of a run must end with the arms handed back to Unitree's controller at weight 0 (PLAN.md 4.2
rule 12), or held still at weight 1 until the operator acts. Handing back near the table is only safe at the factory pose:
from anywhere else Unitree's controller takes the arms home on its own path, through the table (87, SAFE-PATHS.md). So both
exits walk our waypoint paths (g1sim/arm_paths.py) to the factory pose first. The phases (g1sim/real_run_modes.py,
g1sim/real_run_policy.py) only raise ExitRequest; the runner (g1sim/real_run.py) catches it and calls soft_exit or
fault_exit from here, so both exit paths sit together in one file.
"""
import math
import traceback
from typing import TYPE_CHECKING

import numpy as np

from g1sim import arm_paths, real_preflight as pre, real_safety as rs

if TYPE_CHECKING:                                   # for the type hints only: real_run imports this module
    from g1sim.real_run import Runner

OPEN_HANDS = np.zeros(6)                            # BrainCo 0 = open (PLAN.md 2.2)
HANDS_OPEN_S = 2.0                                  # `hands-open` sends 0 for 2 s (rule 9); the soft exit pauses as long
FAULT_SAYS = {rs.RECOVER: "holding still, then back to the factory pose on our safe path",
              rs.HOLD: "holding the arms still",
              rs.RAMP_OUT: "handing the arms back now (2 s ramp-out)"}   # operator line per real_safety fault action


class ExitRequest(Exception):
    """A run phase ends: kind "done" | "soft" | "fault" and the reason go to the runner's exit (g1sim/real_run.py)."""

    def __init__(self, kind: str, reason: str):
        super().__init__(reason)
        self.kind, self.reason = kind, reason


def soft_exit(runner: "Runner", reason: str) -> tuple:
    """Slow way home (rule 12) for `runner`: from the policy phase the way out (lift, waist home, ready pose - open the
    hands, pause - then the way in backwards to q0), ramp out; from anywhere else back along the way-in legs travelled so
    far. Returns (kind, reason); a second signal, stale LowState or contact makes it a fault."""
    if not runner.sent_arm:
        return "soft", reason                       # nothing was ever sent: nothing to hand back
    runner.in_exit = True
    plan_name, waypoints, ready_index = soft_exit_path(runner)
    plan = rs.soft_exit_plan(waypoints, runner.q0, ready_index)
    to_ready = [target for target in plan if target[0] == "to_ready"]
    to_start = [target for target in plan if target[0] == "to_start"]
    pause = round(HANDS_OPEN_S * rs.SEND_HZ) if to_ready and runner.hand_senders else 0
    weight = runner.weight                          # an exit during ramp_in keeps its weight: no jump to 1
    runner.start_exit_clock(len(plan) + pause + len(rs.ramp_out(runner.q0, weight)))
    planned_s = rs.exit_path_seconds(plan) + pause / rs.SEND_HZ
    runner.say(f"soft exit ({reason}): planned {planned_s:.1f} s path, then 2 s ramp-out")
    try:
        runner.drive_path(plan_name, waypoints, to_ready, weight)
        if runner.hand_target is not None:
            runner.command_hands(OPEN_HANDS, OPEN_HANDS, rs.HANDS_OPEN_SPEED)
        for _ in range(pause):
            runner.tick("to_ready", weight, to_ready[-1][2])
        runner.drive_path(plan_name, waypoints, to_start, weight)
        for w, q in rs.ramp_out(runner.command, weight):
            runner.tick("ramp_out", w, q)
    except ExitRequest as request:
        return "fault", request.reason
    except Exception as error:
        runner.say(traceback.format_exc())
        return "fault", f"internal_error: {type(error).__name__}: {error}"
    return "soft", reason


def soft_exit_path(runner: "Runner") -> tuple:
    """(plan name, waypoints, index of the ready pose) of a soft exit from where the arm is (SAFE-PATHS.md): from the
    policy phase the way out, whose ready pose is where the hands open; from ramp_in, to_ready, hold or shadow back along
    the way-in legs travelled so far (arm_paths.recover), no ready pose on the way."""
    state = runner.arm_path
    if state["phase"] == "policy":
        waypoints = arm_paths.way_out(runner.command, runner.q0, runner.ready17, pre.LIMITS_LO, pre.LIMITS_HI)
        return arm_paths.WAY_OUT, waypoints, arm_paths.WAY_OUT_READY
    waypoints = arm_paths.recover(runner.command, state["plan"], state["waypoints"], state["leg"])
    return arm_paths.RECOVER_PATH, waypoints, 0


def fault_exit(runner: "Runner", reason: str) -> None:
    """What the arms do after a fault (rule 12 as changed by SAFE-PATHS.md, rulings F1-F5 in real_safety.FAULT_ACTIONS):
    - RECOVER: hold the fault target (fault_targets: the measured angles, so a blocked arm stops pushing), for stale
      LowState until it is back (STALE_RETURN_WAIT_S), then our waypoint path home (arm_paths.home), ramp out at q0;
    - HOLD, or a recover that cannot start or is stopped (a second fault, a signal): weight 1 on a still target until the
      operator acts - owner's remote / e-stop, or one more stop signal, which ramps out there;
    - while the weight is below 1 (ramp_in, ramp-out) the arms are at q0 already: ramp out at once.
    Waist roll/pitch stay at q0 (rule 4); the hands keep their last command; a ramp-out is never stopped (review M2)."""
    runner.in_fault, runner.fault_stage, runner.exit_deadline = True, "hold", None
    if not runner.sent_arm or runner.weight == 0.0:
        runner.say(f"FAULT ({reason}): no arm command was ever held, nothing to hand back")
        return
    target = _fault_target(runner, reason)
    action = rs.fault_action(reason) if runner.weight == 1.0 else rs.RAMP_OUT
    runner.say(f"FAULT ({reason}): {FAULT_SAYS[action]}")
    if action == rs.RECOVER:
        stopped = _recover(runner, reason, target)
        if stopped is None:
            return                                  # handed back at q0
        if stopped == "signal":
            action = rs.RAMP_OUT                    # the operator pressed during the short hold: hand back here
        else:
            target, action = _fault_target(runner, stopped), rs.HOLD
    if action == rs.HOLD:
        _hold(runner, target)                       # ends only with a stop signal: the operator chose the hand-back here
    _ramp_out(runner, target)


def _fault_target(runner: "Runner", reason: str) -> np.ndarray:
    """The still target of a fault hold (real_safety.fault_targets) with waist roll/pitch at the filter's q0 (rule 4)."""
    target = rs.fault_targets(reason, runner.measured_now(), runner.command)
    target[list(rs.WAIST_HOLD_POS)] = runner.filter.q0[list(rs.WAIST_HOLD_POS)]
    return target


def _recover(runner: "Runner", reason: str, target: np.ndarray) -> str | None:
    """Hold `target` (RECOVER_HOLD_S, or for stale LowState until it is back), then drive arm_paths.home and ramp out.
    Returns None when the arms were handed back at q0, "signal" for a stop signal during the hold, else the reason the
    recover could not start or stopped (then fault_exit holds)."""
    stale = rs.reason_key(reason) == "lowstate_stale"
    waited = _hold(runner, target, rs.STALE_RETURN_WAIT_S if stale else rs.RECOVER_HOLD_S,
                   until=(lambda: _lowstate_back(runner)) if stale else None)
    if waited == "signal":
        return "signal"
    if waited == "timeout":
        runner.say(f"LowState not back within {rs.STALE_RETURN_WAIT_S} s: no recover")
        return "lowstate_stale"
    speed = rs.CONTACT_RECOVER_SPEED if rs.reason_key(reason) == "contact_fault" else rs.SCRIPTED_SPEED
    waypoints = arm_paths.home(target, runner.arm_path, pre.LIMITS_LO, pre.LIMITS_HI)
    targets = [("to_factory", leg, q) for leg, q in rs.move_through(waypoints, speed)]
    runner.note_event("fault_recover")
    runner.say(f"fault recover: {len(waypoints) - 1} legs at {speed} rad/s, {len(targets) / rs.SEND_HZ:.1f} s")
    runner.fault_stage = "recover"
    runner.filter.reset_last(target)                # the hold sent `target` unfiltered: the rate limit starts from it
    runner.start_exit_clock(len(targets))
    try:
        runner.drive_path(arm_paths.RECOVER_PATH, waypoints, targets)
    except ExitRequest as request:
        runner.say(f"fault recover stopped ({request.reason})")
        return request.reason
    except Exception as error:
        runner.say(traceback.format_exc())
        return f"internal_error: {type(error).__name__}: {error}"
    finally:
        runner.fault_stage, runner.exit_deadline = "hold", None
    _ramp_out(runner, runner.command)
    return None


def _hold(runner: "Runner", target: np.ndarray, seconds: float = math.inf, until=None) -> str:
    """Send `target` at weight 1, unfiltered (it is the measured pose or our last command: nothing moves). Ends "done"
    after `seconds` (or as soon as `until()` is true), "timeout" when `until` never came true in time, "signal" on a
    stop signal (Runner.check_signals). Without a time limit only a signal ends it: the operator decides."""
    runner.fault_stage, runner.exit_deadline = "hold", None
    if math.isinf(seconds):
        runner.note_event("fault_hold")
        runner.say("ARMS HELD STILL (weight 1). Owner: remote or e-stop. Or press Ctrl+C once more to hand them back HERE"
                   " - Unitree's controller then moves them home on its own path (pull the table back first).")
    ticks = 0
    try:
        while ticks < seconds * rs.SEND_HZ:
            runner.tick("fault_hold", 1.0, target, filtered=False)
            ticks += 1
            if until is not None and until():
                return "done"
    except ExitRequest as request:
        if request.reason != "signal_in_fault":
            raise
        return "signal"
    return "timeout" if until is not None else "done"


def _lowstate_back(runner: "Runner") -> bool:
    """F1: LowState is fresh again and the arm is within LEAD_LIMIT of our command, so the planned path starts where the
    arm really is."""
    msg, age = runner.io.lowstate.latest()
    if msg is None or age > rs.STALE_S["lowstate"]:
        return False
    gap = np.abs(runner.measured_now() - runner.command)
    return bool(np.all(np.isfinite(gap)) and gap.max() <= rs.LEAD_LIMIT)


def _ramp_out(runner: "Runner", target: np.ndarray) -> None:
    """Hand the arms back: weight -> 0 in 2 s on `target`, unfiltered; no signal and no staleness stops it (review M2)."""
    runner.fault_stage, runner.exit_deadline = "ramp_out", None
    for w, q in rs.ramp_out(target, runner.weight):
        runner.tick("ramp_out", w, q, filtered=False)
