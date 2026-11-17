"""Table clearance of the robot-day arm paths (87): path shapes, sampling by arm movement and the pass rule. Pure numpy.

Why it exists: on Monday the arms move between the real factory hold pose (unknown until M0) and the ready pose, and a fault
at the table hands the arm to Unitree's controller wherever the policy left it. 87 checks by forward kinematics that none of
these paths brings an arm closer than 5 cm to table, stand or basket (PLAN.md 3.5, rule 16). Nothing here touches MuJoCo:
the functions get `step_of` (largest arm movement between two poses, m) and `clearance_of` (distance per object, m) from the
caller (g1sim/arm_clearance_fk.py), so the sampling rules are unit-tested with a planted one-joint arm.
Every pose is a 17-vector (motors 12..28) built with real_safety.sdk_targets, so 87 checks exactly the joint lines 86 sends.
"""
from typing import Callable

import numpy as np

from g1sim import action_space, real_safety

CLEARANCE_MIN_M = 0.05      # 5 cm margin for real-vs-sim pose error and tracking lag (PLAN.md 3.5, rule 16)
MAX_SAMPLE_STEP_M = 0.02    # no arm point moves more than 2 cm between two checked poses: sampling error ~1 mm at 5 cm (R4-1)
MAX_HALVINGS = 10           # 2^10 = 1024 sub-steps per interval; a smooth arm never needs more, so more means a bug (R4-1)
FACTORY_LOOKUP_TOL = 0.15   # rad: largest shoulder roll/yaw, wrist or waist yaw of a factory pose the grid stands in for (R2-3)
EVEN_EVERY = 5              # the even samples of a 0.3 rad/s move line: every 5th 50 Hz tick (PLAN.md 3.5)
RAMP_OUT_SAMPLES = 20       # the ramp-out's 100 ticks, every 5th: even samples of the straight fault path (PLAN.md 3.5)
CORNER_LEG_SAMPLES = 10     # even samples per leg of a corner ordering, 10 + 10 (PLAN.md 3.5)
OBJECTS = ("table", "stand", "basket")
WAIST_LIMITS = (0.0, 0.15, 0.35, 0.6)   # --waist-hold (0.0) and the waist limits Monday's M3 escalation can reach (R3-2)
# Torso lean the FK checks (Track C review B1, 2026-10-27). 86 holds waist roll/pitch at the measured q0, and the torso then sags
# forward under the arms: in the e2e replay (G2) the sim held q0 pitch 0.041 and measured up to 0.126; roll stayed <= 0.001.
# Clearance falls as the pitch grows (FK probe -0.1..0.15 rad, runs/real_c_b1_waist_probe.txt), so the largest pitch counts.
WAIST_PITCH_CHECKED = 0.126              # rad: e2e G2 max measured waist pitch (sim)
WAIST_PITCH_SAG = 0.085                  # rad: that 0.126 minus the held q0 0.041 - the sag below the hold, from the same run
WAIST_ROLL_TOL = 0.02                    # rad: FK checks roll 0; +-0.02 costs <= ~0.25 cm (probe: +-0.05 costs 0.4-0.6 cm)
GRID_BOUNDS = {"elbow": (0.0, 1.6), "shoulder_pitch": (-0.1, 0.3), "FACTORY_LOOKUP_TOL": FACTORY_LOOKUP_TOL,
               "waist_pitch_q0_max": round(WAIST_PITCH_CHECKED - WAIST_PITCH_SAG, 3), "waist_roll_q0_tol": WAIST_ROLL_TOL}
BLEND_NOTE = "to_hanging = worst of straight and two corner orderings; a bracket of the factory blend, not a proof"
# 17-vector positions that move first in "proximal_first": waist yaw (motor 12) and both shoulders (motors 15-17, 22-24)
PROXIMAL = (0, 3, 4, 5, 10, 11, 12)
ELBOWS, SHOULDER_PITCHES = (6, 13), (3, 10)   # 17-vector positions of both elbows (motors 18, 25) and shoulder pitches (15, 22)
READY17 = real_safety.sdk_targets(action_space.BALL_READY_POSE, np.zeros(17))   # the ready pose, waist 0

StepOf = Callable[[np.ndarray, np.ndarray], float]
ClearanceOf = Callable[[np.ndarray], dict]


# --- poses -------------------------------------------------------------------------------------------------------------

def hanging_pose(elbow: float, shoulder_pitch: float) -> np.ndarray:
    """h(e, s): every arm joint 0 except both elbows = e and both shoulder pitches = s, waist 0 (PLAN.md 3.5)."""
    pose = np.zeros(17)
    pose[list(ELBOWS)] = elbow
    pose[list(SHOULDER_PITCHES)] = shoulder_pitch
    return pose


def demo_frames(state: np.ndarray, waist_limit: float) -> list:
    """Demo states (T, 27) -> 17-vectors with the waist yaw clipped to +-waist_limit, as SafetyFilter clips it (rule 6b)."""
    poses = []
    for row in np.asarray(state, dtype=float):
        clipped = row.copy()
        clipped[26] = np.clip(row[26], -waist_limit, waist_limit)
        poses.append(real_safety.sdk_targets(clipped, np.zeros(17)))
    return poses


# --- path shapes (even samples; densify() refines them) -----------------------------------------------------------------

def move_path(start: np.ndarray, goal: np.ndarray) -> list:
    """The 0.3 rad/s line 86 sends (real_safety.move), every EVEN_EVERY-th tick, with start and goal."""
    ticks = real_safety.move(start, goal)
    return [np.asarray(start, dtype=float)] + ticks[EVEN_EVERY - 1:-1:EVEN_EVERY] + [ticks[-1]]


def straight_path(start: np.ndarray, goal: np.ndarray, samples: int = RAMP_OUT_SAMPLES) -> list:
    """The joint-space line start -> goal in `samples` equal steps (start included)."""
    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    return [start + (goal - start) * k / samples for k in range(samples + 1)]


def corner_path(start: np.ndarray, goal: np.ndarray, first: tuple, leg_samples: int = CORNER_LEG_SAMPLES) -> list:
    """Two joint-space legs: the positions in `first` reach the goal while the rest stay, then the rest move."""
    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    corner = start.copy()
    corner[list(first)] = goal[list(first)]
    return straight_path(start, corner, leg_samples) + straight_path(corner, goal, leg_samples)[1:]


def fault_paths(start: np.ndarray, goal: np.ndarray) -> dict:
    """The three fault ramp-out paths whose worst counts (R3-7): straight, proximal joints first, distal joints first."""
    distal = tuple(i for i in range(17) if i not in PROXIMAL)
    return {"straight": straight_path(start, goal), "proximal_first": corner_path(start, goal, PROXIMAL),
            "distal_first": corner_path(start, goal, distal)}


# --- sampling by arm movement (R4-1, R5-1) ------------------------------------------------------------------------------

def interpolation_fractions(qa: np.ndarray, qb: np.ndarray, step_of: StepOf, max_step: float = MAX_SAMPLE_STEP_M) -> list:
    """Fractions in (0, 1) of the joint-space midpoints to insert between qa and qb until every step is <= max_step."""
    qa, qb = np.asarray(qa, dtype=float), np.asarray(qb, dtype=float)

    def split(fa: float, fb: float, pa: np.ndarray, pb: np.ndarray, depth: int) -> list:
        if step_of(pa, pb) <= max_step:
            return []
        if depth >= MAX_HALVINGS:
            raise RuntimeError("87: path could not be densified - a joint jumps; check the path inputs")
        fm = (fa + fb) / 2
        pm = qa + (qb - qa) * fm
        return split(fa, fm, pa, pm, depth + 1) + [fm] + split(fm, fb, pm, pb, depth + 1)

    return split(0.0, 1.0, qa, qb, 0)


def densify(path: list, step_of: StepOf, max_step: float = MAX_SAMPLE_STEP_M) -> list:
    """The path with joint-space midpoints inserted until no arm point moves more than max_step between two samples."""
    dense = [np.asarray(path[0], dtype=float)]
    for qa, qb in zip(path, path[1:]):
        qa, qb = np.asarray(qa, dtype=float), np.asarray(qb, dtype=float)
        dense += [qa + (qb - qa) * f for f in interpolation_fractions(qa, qb, step_of, max_step)] + [qb]
    return dense


def densify_measured(path: list, step_of: StepOf, max_step: float = MAX_SAMPLE_STEP_M) -> tuple:
    """(densified path, largest step actually left between two of its samples) - the report asserts that number (R4-1)."""
    dense = densify(path, step_of, max_step)
    return dense, max((step_of(a, b) for a, b in zip(dense, dense[1:])), default=0.0)


def select_start_poses(frames: list, step_of: StepOf, max_step: float = MAX_SAMPLE_STEP_M) -> tuple:
    """Demo frames -> ([{"frame", "q"}], max_start_step_m): start poses no more than max_step of arm movement apart (R5-1).

    Walks the frames in order and keeps frame 0; before a frame would be more than max_step from the last kept pose, the
    previous frame is kept; where two raw frames are farther apart, interpolated poses go in between (fractional "frame").
    The last frame is always kept. Every skipped frame lies within max_step of the pose kept before it."""
    kept = [{"frame": 0.0, "q": np.asarray(frames[0], dtype=float)}]
    largest = 0.0
    for i in range(1, len(frames)):
        frame = np.asarray(frames[i], dtype=float)
        step = step_of(kept[-1]["q"], frame)
        if step > max_step and kept[-1]["frame"] < i - 1:
            kept.append({"frame": float(i - 1), "q": np.asarray(frames[i - 1], dtype=float)})
            step = step_of(kept[-1]["q"], frame)
        if step > max_step:
            last = kept[-1]["q"]
            for f in interpolation_fractions(last, frame, step_of, max_step):
                kept.append({"frame": i - 1 + f, "q": last + (frame - last) * f})
            step = step_of(kept[-1]["q"], frame)
        largest = max(largest, step)
        if i == len(frames) - 1:
            kept.append({"frame": float(i), "q": frame})
    return kept, max(largest, _largest_kept_step(kept, step_of))


def _largest_kept_step(kept: list, step_of: StepOf) -> float:
    return max((step_of(a["q"], b["q"]) for a, b in zip(kept, kept[1:])), default=0.0)


# --- clearance along a path and the pass rule ---------------------------------------------------------------------------

def path_clearance(path: list, clearance_of: ClearanceOf) -> tuple:
    """({object: smallest clearance along the path, start pose excluded}, index of the worst sample) - excluded because the
    rule compares the way OUT with where the arm already was."""
    along = {name: np.inf for name in OBJECTS}
    worst, worst_index = np.inf, 0
    for index, pose in enumerate(path[1:], start=1):
        clearance = clearance_of(pose)
        for name in OBJECTS:
            along[name] = min(along[name], clearance[name])
        if min(clearance[name] for name in OBJECTS) < worst:
            worst, worst_index = min(clearance[name] for name in OBJECTS), index
    return along, worst_index


def exit_passes(start: dict, along: dict) -> bool:
    """Pass rule per object (R2-1): along the exit the clearance never drops below min(5 cm, clearance at the start pose) -
    "never closer than 5 cm, or than the arm already was" (a grasp pose may start 1-2 cm from the stand)."""
    return all(along[name] >= min(CLEARANCE_MIN_M, start[name]) for name in OBJECTS)


def exit_margin(start: dict, along: dict) -> float:
    """How far the exit stays above the pass line (negative = fails); used to pick the worst ordering."""
    return min(along[name] - min(CLEARANCE_MIN_M, start[name]) for name in OBJECTS)


def worst_fault_exit(start: np.ndarray, goal: np.ndarray, start_clearance: dict, step_of: StepOf,
                     clearance_of: ClearanceOf) -> dict:
    """The three fault orderings start -> goal, densified; returns the worst {ordering, along, passed, samples, max_step}.

    passed is True only if all three orderings pass (a torque-level blend may follow any path between them, R3-7)."""
    results, samples, largest = [], 0, 0.0
    for ordering, path in fault_paths(start, goal).items():
        dense, step = densify_measured(path, step_of)
        along, _ = path_clearance(dense, clearance_of)
        samples, largest = samples + len(dense), max(largest, step)
        results.append((exit_margin(start_clearance, along), ordering, along))
    margin, ordering, along = min(results, key=lambda r: r[0])
    return {"ordering": ordering, "along": along, "passed": bool(margin >= 0), "samples": samples, "max_step": largest}
