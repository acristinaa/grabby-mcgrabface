"""Real G1: what each 86 mode does, phase by phase (PLAN.md 3.4, task B2 step 1). The runner (g1sim/real_run.py) owns how a
single message is filtered, sent and logged, and how a run ends; this file only says which messages come in which order.

Why it exists: the operators' runbook steps M1-M6 are exactly these bodies - release (M1), ready (M2), replay (M3/M4),
shadow (M5), run (M6), hands-open, and recover (after a crash near the table) - so each one must read like the step it
implements. Every body ends by raising ExitRequest: "soft"/"done" walks home through the runner's soft exit, "done" means
the arms are already handed back. Every move between the factory pose and the ready pose runs on the waypoints of
g1sim/arm_paths.py, the ones 87 checks against the table (SAFE-PATHS.md).
"""
from typing import TYPE_CHECKING

import numpy as np

from g1sim import action_space, arm_paths, real_preflight as pre, real_safety as rs
from g1sim.real_run_exits import HANDS_OPEN_S, OPEN_HANDS, ExitRequest
from g1sim.real_run_policy import PolicyPhase, reset_policy

if TYPE_CHECKING:                                   # for the type hints only: real_run imports this module
    from g1sim.real_run import Runner

READY_ACTION = action_space.BALL_READY_POSE         # arms at the ready pose, relaxed demo hands, waist 0 (PLAN.md 2.3)
SETTLE_BEFORE_POLICY_S = 1.0                        # rest at the ready pose before the first observation (demos start at rest)
ARM_ONLY_MODES = ("release", "recover")             # hand the arms back; no hand writers (the fingers stay as they are)


def ready(runner: "Runner") -> None:
    """M2: ramp in on the frozen measured angles, rise to the ready pose, hold (lead printed each second), then home."""
    take_over(runner)
    move_to_ready(runner)
    hold(runner, runner.args.hold, print_lead=True)
    raise ExitRequest("soft", "done")


def policy(runner: "Runner") -> None:
    """M3-M6: the ready path, a short rest, then the policy phase (replay/run send it, shadow only logs it)."""
    reset_policy(runner)
    take_over(runner)
    move_to_ready(runner)
    hold(runner, SETTLE_BEFORE_POLICY_S)
    runner.phase = "shadow" if runner.mode == "shadow" else "policy"
    runner.log_arm_path(runner.phase)
    PolicyPhase(runner, send_policy=runner.mode != "shadow").run()


def release(runner: "Runner") -> None:
    """M1 and the first step after any crash (rule 13): weight 1 at the measured angles (the one allowed weight jump), to
    the recorded factory pose at 0.3 rad/s, ramp out there. The whole plan is an exit path: a signal makes it a fault."""
    measured = runner.measured_now()
    runner.q0 = measured
    runner.filter = runner.new_filter(measured, waist_max=rs.ROBOT_WAIST_MAX_CEILING, waist_hold=False)
    runner.in_release = True                        # a signal here is a fault, never a soft exit back to the crash pose
    factory = np.asarray(runner.io.factory_record["q"], dtype=float)
    plan = rs.release_plan(measured, factory)
    ramp_start = len(plan) - len(rs.ramp_out(factory, 1.0))
    runner.log_arm_path("to_factory", arm_paths.RECOVER_PATH, [measured, factory], 1)   # a fault goes on along this line
    runner.start_exit_clock(len(plan))
    for i, (weight, q) in enumerate(plan):
        phase = "release" if i == 0 else ("to_factory" if i < ramp_start else "ramp_out")
        runner.tick(phase, weight, q, release_first=(i == 0))
    raise ExitRequest("done", "done")


def recover(runner: "Runner") -> None:
    """`86 recover`, the first step after a crash, kill or cable pull while our run may have held the arms near the table
    (SAFE-PATHS.md S4): weight 1 at the measured angles (the one allowed weight jump, as in release), then our own way home
    from where that run's sidecar says the arms were (arm_paths.home: back along the way in, on along an exit path, or the
    way out from a policy/hold/shadow pose) to that run's q0, ramp out there. Unlike release it never takes the straight
    line, which from a pose at the table goes through it. Preflight refused already if no arm path was logged, the log is
    a policy one, or the arms are not where it says (real_preflight.arm_path_problem)."""
    measured = runner.measured_now()
    last = runner.io.last_arm_path
    runner.q0 = np.asarray(last["q0"], dtype=float)                 # where that run took the arms over = its factory pose
    runner.ready17 = None if last["ready17"] is None else np.asarray(last["ready17"], dtype=float)
    runner.filter = runner.new_filter(measured, waist_max=rs.ROBOT_WAIST_MAX_CEILING, waist_hold=False)  # 13/14 as found
    runner.in_release = True                        # a signal here is a fault (F5: hold), never a soft exit
    waypoints = arm_paths.home(measured, last, pre.LIMITS_LO, pre.LIMITS_HI)
    targets = [("to_factory", leg, q) for leg, q in rs.move_through(waypoints)]
    runner.log_arm_path("to_factory", arm_paths.RECOVER_PATH, waypoints, 1)
    runner.say(f"recover from {last['phase']} ({last['plan']} leg {last['leg']}): {len(waypoints) - 1} legs, "
               f"{len(targets) / rs.SEND_HZ:.1f} s, then 2 s ramp-out")
    ramp = rs.ramp_out(waypoints[-1], 1.0)
    runner.start_exit_clock(1 + len(targets) + len(ramp))
    runner.tick("release", 1.0, measured, release_first=True)
    runner.drive_path(arm_paths.RECOVER_PATH, waypoints, targets)
    for weight, q in ramp:
        runner.tick("ramp_out", weight, q)
    raise ExitRequest("done", "done")


def hands_open(runner: "Runner") -> None:
    """Both hands to 0 (open) at speed 0.3 for 2 s, no rt/arm_sdk at all (rule 9)."""
    runner.command_hands(OPEN_HANDS, OPEN_HANDS, rs.HANDS_OPEN_SPEED)
    for _ in range(round(HANDS_OPEN_S * rs.SEND_HZ)):
        runner.check_signals()
        runner.send_hands()
        runner.wait()
    raise ExitRequest("done", "done")


def take_over(runner: "Runner") -> None:
    """Ramp in: q0 = the measured angles, read ONCE and commanded for the whole ramp (review N1, rule 3). Re-reading
    them each tick would chase an arm that sags while the weight is still low. Logged as the start of way-in leg 1, so an
    exit from here goes back to q0."""
    runner.q0 = runner.measured_now()
    runner.ready17 = rs.sdk_targets(READY_ACTION, runner.q0)
    runner.filter = runner.new_filter(runner.q0)
    runner.log_arm_path("ramp_in", arm_paths.WAY_IN, arm_paths.way_in(runner.q0, runner.ready17), 1)
    for weight, q in rs.ramp_in(runner.q0):
        runner.tick("ramp_in", weight, q)


def move_to_ready(runner: "Runner") -> None:
    """The way in at 0.3 rad/s (arm_paths.way_in: swing back, hands up beside the body, forward to BALL_READY_POSE; waist
    0, 13/14 held), demo hands relaxed. A straight line would sweep the low hands through the table edge (87)."""
    left, right = rs.hand_targets(READY_ACTION)
    runner.command_hands(left, right, rs.HAND_SPEED)
    waypoints = arm_paths.way_in(runner.q0, runner.ready17)
    targets = [("to_ready", leg, q) for leg, q in rs.move_through(waypoints)]
    runner.drive_path(arm_paths.WAY_IN, waypoints, targets)


def hold(runner: "Runner", seconds: float, print_lead: bool = False) -> None:
    """Hold the ready pose; with print_lead, show |command - measured| once per second (runbook M2 abort rule)."""
    runner.log_arm_path("hold")
    for k in range(round(seconds * rs.SEND_HZ)):
        runner.tick("hold", 1.0, runner.ready17)
        if print_lead and k % rs.SEND_HZ == 0:
            runner.say(f"hold: lead {runner.lead:.3f} rad (runbook M2 aborts above 0.10)")


MODE_BODIES = {"ready": ready, "replay": policy, "shadow": policy, "run": policy, "release": release,
               "recover": recover, "hands-open": hands_open}
