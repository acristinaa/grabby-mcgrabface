"""Waypoint arm paths at the table: the way in (factory pose -> ready pose), the way out (any pose -> factory pose), the
recover from wherever 86 logged the arm (home) and the leg timing. One definition for 86 (the robot) and 87 (the clearance
check), see .planning/real-robot/SAFE-PATHS.md.

Why it exists: 86's straight joint lines between the factory pose (hands hanging below the 0.74 m table top) and the ready pose
(hands raised in front) swing the hands forward while they are still low, through the table edge and the basket (87,
runs/arm_path_clearance*.log). These paths go round instead: hands back and up beside the body first, forward only above
the table; on the way out first up, then the waist home, then the same way back down. Every leg is still one straight joint
line at the scripted speed (real_safety.move), so 87 checks exactly what 86 would send. Pure numpy, no MuJoCo, no DDS.
A pose is a 17-vector (motors 12..28, real_safety.SDK_MOTORS). Waypoints are set for the right arm; the left arm is its mirror.
"""
import numpy as np

from g1sim import action_space, real_safety

LEFT_ARM, RIGHT_ARM = slice(3, 10), slice(10, 17)        # 17-vector positions of motors 15-21 and 22-28
SHOULDER_PITCH, SHOULDER_ROLL, SHOULDER_YAW, ELBOW, WRIST_PITCH = 0, 1, 2, 3, 5   # index in one arm (action_space.ARM_JOINTS)

# Way in, leg 1 "swing back": upper arm 0.5 rad back and turned 0.8 rad out, elbow and wrists stay as they are. Turning the
# upper arm out makes the next elbow bend lift the hand sideways instead of forward. Signs (FK probe, right arm): pitch + =
# back, roll - = out, yaw - = forearm out. Grid search: runs/safe_paths_probes/probe4+5.txt.
SWING_BACK_RIGHT = {SHOULDER_PITCH: 0.5, SHOULDER_ROLL: 0.0, SHOULDER_YAW: -0.8}
# Way in, leg 2 "side up": hands raised beside the body, elbow and wrist already as in the ready pose, so the last leg to the
# ready pose (shoulders only) swings the hands forward at chest height, above table, stand and basket.
SIDE_UP_RIGHT_ARM = (0.3, -0.6, -1.4, -0.8, 0.0, 0.6, 0.0)   # rad, order action_space.ARM_JOINTS
# Way out, leg 1 "lift": from wherever the hands are, shoulder 0.1 rad up, elbow 1.0 rad more bent (stops LIMIT_MARGIN inside
# its limit), wrist 0.5 rad up - the hands rise and fold in before anything turns. Then the waist turns home with the arms
# held, then the arms go to the ready pose. Grid search over deltas and leg orders: runs/safe_paths_probes/probe10/11/13.txt.
LIFT_DELTA_RIGHT = {SHOULDER_PITCH: -0.1, ELBOW: -1.0, WRIST_PITCH: -0.5}
# The waypoint plans 86 runs; it logs the plan name and leg number so a recover or `86 recover` knows which way is
# safe from a pose on that plan (review safe-paths B1). RECOVER_PATH = the waypoints of a recover or of a soft exit that
# goes back along the way in: a pose on it goes on along it, like on the way out (review safe-paths round 2 n3).
WAY_IN, WAY_OUT, RECOVER_PATH = "way_in", "way_out", "recover_path"
WAY_OUT_READY = 3                       # index of the ready pose in way_out(): current, lift, waist home, ready, ...
# 86 phases without a waypoint plan running (the arm sits at or moves around the ready pose): a recover from there is
# way_out from the measured pose (SAFE-PATHS.md "Recover" table)
FREE_PHASES = ("policy", "hold", "shadow")


def _both_arms(right_values: dict) -> dict:
    """{17-vector position: value} for the right-arm joints in right_values and their mirrored left-arm twins."""
    positions = {}
    for joint, value in right_values.items():
        positions[RIGHT_ARM.start + joint] = value
        positions[LEFT_ARM.start + joint] = value * action_space.MIRROR[joint]
    return positions


def swing_back(pose17) -> np.ndarray:
    """The pose with both shoulders at SWING_BACK_RIGHT (mirrored left); elbows, wrists and waist unchanged."""
    swung = np.array(pose17, dtype=float)
    for position, value in _both_arms(SWING_BACK_RIGHT).items():
        swung[position] = value
    return swung


def side_up(like17) -> np.ndarray:
    """SIDE_UP_RIGHT_ARM on both arms; waist yaw 0 and waist roll/pitch from like17 (86 holds 13/14 at q0)."""
    pose = np.array(like17, dtype=float)
    pose[real_safety.WAIST_YAW_POS] = 0.0
    pose[RIGHT_ARM] = SIDE_UP_RIGHT_ARM
    pose[LEFT_ARM] = np.asarray(SIDE_UP_RIGHT_ARM) * action_space.MIRROR
    return pose


def lift(pose17, limits_lo17, limits_hi17) -> np.ndarray:
    """The pose plus LIFT_DELTA_RIGHT (mirrored left), each lifted joint kept LIMIT_MARGIN inside the joint limits - the
    same margin 86's SafetyFilter clips to, so the line 87 checks is the line 86 sends."""
    lifted = np.array(pose17, dtype=float)
    lo = np.asarray(limits_lo17, dtype=float) + real_safety.LIMIT_MARGIN
    hi = np.asarray(limits_hi17, dtype=float) - real_safety.LIMIT_MARGIN
    for position, delta in _both_arms(LIFT_DELTA_RIGHT).items():
        lifted[position] = np.clip(lifted[position] + delta, lo[position], hi[position])
    return lifted


def way_in(factory17, ready17) -> list:
    """Waypoints factory -> ready: swing back (elbow as found), hands up beside the body, forward to the ready pose."""
    factory = np.array(factory17, dtype=float)
    return [factory, swing_back(factory), side_up(ready17), np.array(ready17, dtype=float)]


def way_out(current17, factory17, ready17, limits_lo17, limits_hi17) -> list:
    """Waypoints from any pose to the factory pose: lift, waist home with the arms held, the ready pose, then the way in
    backwards. Recover from the policy, hold or shadow phase = this path, then 86's ramp-out at the factory pose, so Unitree's
    controller has nothing left to move. From a pose on a waypoint plan use recover() instead."""
    current = np.array(current17, dtype=float)
    lifted = lift(current, limits_lo17, limits_hi17)
    waist_home = lifted.copy()
    waist_home[real_safety.WAIST_YAW_POS] = ready17[real_safety.WAIST_YAW_POS]
    return [current, lifted, waist_home] + way_in(factory17, ready17)[::-1]


def recover(current17, plan_name: str, plan: list, leg: int) -> list:
    """Waypoints from a pose on leg `leg` (1-based: between plan[leg - 1] and plan[leg]) of a running waypoint plan to the
    factory pose. On the way in: back along the legs already travelled (plan[leg - 1] ... plan[0]). On the way out: on
    along the rest of it (plan[leg] ... plan[-1]). Either way every line is (part of) a line 87 checked.

    Why not way_out from here: on way-in leg 1 or early leg 2 the hands are low beside the table edge, and lift -> ready is
    then the forward sweep the waypoints exist to avoid (review safe-paths B1: 6.9-11.5 cm into the table). From the
    policy, hold or shadow phase (no waypoint plan running) recover is way_out(current, ...) itself."""
    current = np.array(current17, dtype=float)
    if plan_name == WAY_IN:
        return [current] + plan[:leg][::-1]
    if plan_name in (WAY_OUT, RECOVER_PATH):
        return [current] + plan[leg:]
    raise ValueError(f"unknown plan {plan_name!r} - expected {WAY_IN!r}, {WAY_OUT!r} or {RECOVER_PATH!r}")


def home(current17, arm_path: dict, limits_lo17, limits_hi17) -> list:
    """Waypoints of a recover (after a fault, or `86 recover` after a crash) from current17 to the factory pose, chosen by
    where the arm is (SAFE-PATHS.md "Recover" table): from a policy, hold or shadow pose the way out; on a waypoint plan
    recover() along that plan. `arm_path` is what 86 logs at every leg start (real_run.Runner.log_arm_path): phase, plan,
    leg, waypoints, q0 (the factory pose of that run) and ready17."""
    if arm_path["phase"] in FREE_PHASES:
        return way_out(current17, arm_path["q0"], arm_path["ready17"], limits_lo17, limits_hi17)
    plan = [np.asarray(waypoint, dtype=float) for waypoint in arm_path["waypoints"]]
    return recover(current17, arm_path["plan"], plan, arm_path["leg"])


def path_ticks(waypoints, speed: float = real_safety.SCRIPTED_SPEED) -> list:
    """The 50 Hz targets 86 sends: real_safety.move_through along the legs (no joint faster than `speed`)."""
    return [q for _, q in real_safety.move_through(waypoints, speed)]


def path_seconds(waypoints, speed: float = real_safety.SCRIPTED_SPEED) -> float:
    """Duration of the waypoint path at `speed` (the 0.3 rad/s cap of every scripted move, rule 6)."""
    return len(path_ticks(waypoints, speed)) / real_safety.SEND_HZ
