"""87 --waypoint-paths: table clearance of the waypoint paths in g1sim/arm_paths.py over every scene case Monday may meet.

Why it exists: the waypoint paths replace 86's straight lines only if they keep every arm and hand geom >= 5 cm from table,
stand and basket for every factory pose, every torso lean and every table height - not just in the one nominal FK scene the
old 87 blocks use. Two blocks, because the way out splits at the ready pose:
- way_in rows: factory pose h(e, s) -> ready (arm_paths.way_in). The way out's second half (ready -> factory) and the soft
  exit from to_ready/hold/shadow run the same legs backwards, so these rows cover them too. Pass: >= 5 cm at every sample,
  the factory pose itself included (Unitree's controller holds the arms there after the hand-back).
- lift_home rows: policy pose -> lift -> waist home -> ready (the first half of arm_paths.way_out) from 87's start poses.
  Pass: arm_clearance.exit_passes ("never closer than 5 cm, or than the arm already was").
- recover rows: a fault at RECOVER_FRACTIONS of each way-in leg, then arm_paths.recover back to the factory pose (review
  safe-paths B1). The way out's second half is the way in backwards, so a recover there runs the same lines; on its first
  half recover goes on along lines the lift_home rows checked. Pass: exit_passes.
Factory poses: the h(e, s) grid, or with 87 --factory-pose the one pose 84 recorded at M0 (review safe-paths B3: the grid
rows do not stand in for poses between rows or inside the rule 16 (a) tolerance).
Scene cases: torso lean pitch from the smallest allowed q0 to the largest sag, roll +-WAIST_ROLL_TOL (review C r1, review
safe-paths B2), table height nominal and the v2 range (configs/randomization.json objects.table_height_m). Uses the
ClearanceModel callables, FK only.
"""
import itertools

import numpy as np

from g1sim import arm_clearance as ac
from g1sim import arm_paths
from g1sim.arm_clearance_report import key

# rad: lowest M0 q0 waist pitch this check covers - the proposed lower bound for rule 16 (a), a ruling for Finnegan. At the
# factory pose the arms hang, the torso sits at about q0, and the hands are closest to the table at the SMALLEST pitch:
# h(1.1, 0.1) 4.00 cm at -0.05, 4.88 at 0.0, 5.65 at 0.041 (review safe-paths B2).
WAIST_PITCH_Q0_MIN = -0.05
WAIST_PITCH_Q0_MAX = ac.GRID_BOUNDS["waist_pitch_q0_max"]   # 0.041 rad, rule 16 (a)
# rad: the q0 range (upright end first, 0.0, the largest q0), its midpoint to the largest sim sag 0.126 (review C r1), the sag
LEAN_PITCHES = (WAIST_PITCH_Q0_MIN, 0.0, WAIST_PITCH_Q0_MAX, round((WAIST_PITCH_Q0_MAX + ac.WAIST_PITCH_CHECKED) / 2, 4),
                ac.WAIST_PITCH_CHECKED)
LEAN_ROLLS = (-ac.WAIST_ROLL_TOL, 0.0, ac.WAIST_ROLL_TOL)    # rad, rule 16 (a) roll tolerance
TABLE_DZS = (-0.04, 0.0, 0.01)                              # m, configs/randomization.json table_height_m [-0.04, 0.01] + nominal
ROUND = 4                                                   # decimals of clearances in the rows (0.1 mm), as 87
RECOVER_FRACTIONS = (0.1, 0.3, 0.5, 0.7, 0.9)               # where on a leg the fault strikes (review safe-paths B1 probe)
WAIST_ROLL_POS, WAIST_PITCH_POS = 1, 2                      # 17-vector positions of motors 13/14 (q0 roll/pitch 86 holds)


def scene_cases() -> list:
    """Every (lean roll, lean pitch, table dz) case, as dicts."""
    return [{"lean_roll": r, "lean_pitch": p, "table_dz": dz}
            for p, r, dz in itertools.product(LEAN_PITCHES, LEAN_ROLLS, TABLE_DZS)]


def set_case(cm, case: dict) -> None:
    cm.set_lean(case["lean_roll"], case["lean_pitch"])
    cm.set_table_height(case["table_dz"])


def reset_scene(cm) -> None:
    """Back to the scene every other 87 block uses (lean (0, WAIST_PITCH_CHECKED), nominal table, setback 0)."""
    cm.set_lean(0.0, ac.WAIST_PITCH_CHECKED)
    cm.set_table_height(0.0)
    cm.set_setback(0.0)


class Sampling:
    """Samples and the largest step left between two checked poses, for 87's 2 cm sampling assertion (R4-1)."""

    def __init__(self):
        self.samples, self.max_step = 0, 0.0

    def dense_leg(self, cm, start, goal) -> list:
        """One waypoint leg as 86 sends it (real_safety.move, even samples), densified to <= 2 cm of arm movement."""
        dense, step = ac.densify_measured(ac.move_path(start, goal), cm.step_of)
        self.samples, self.max_step = self.samples + len(dense), max(self.max_step, step)
        return dense


def leg_minima(cm, waypoints, sampling: Sampling) -> tuple:
    """([smallest clearance to any object per leg], worst pair on the worst leg); the start pose counts (its own leg 0)."""
    minima, worst = [], (np.inf, "")
    for start, goal in zip(waypoints, waypoints[1:]):
        dense = sampling.dense_leg(cm, start, goal)
        values = [min(cm.clearance_of(q).values()) for q in dense]
        i = int(np.argmin(values))
        minima.append(round(float(values[i]), ROUND))
        worst = min(worst, (values[i], cm.worst_pair(dense[i])))
    return minima, worst[1]


def grid_factories(elbows, pitches) -> list:
    """[(elbow, shoulder pitch, h(e, s))] for every grid row."""
    return [(e, s, ac.hanging_pose(e, s)) for e, s in itertools.product(elbows, pitches)]


def measured_factory(q17) -> list:
    """[(larger elbow, smaller shoulder pitch, pose)] for the pose 84 recorded at M0; the two numbers only label the rows.

    Refuses a q0 waist roll/pitch outside the lean cases LEAN_PITCHES / LEAN_ROLLS cover: 87 would then pass a torso
    lean it never checked (rule 16 (a), review safe-paths B2)."""
    pose = np.asarray(q17, dtype=float)
    roll, pitch = pose[WAIST_ROLL_POS], pose[WAIST_PITCH_POS]
    if not (WAIST_PITCH_Q0_MIN <= pitch <= WAIST_PITCH_Q0_MAX and abs(roll) <= ac.WAIST_ROLL_TOL):
        raise ValueError(f"M0 q0 waist roll {roll:.3f} / pitch {pitch:.3f} rad outside the checked lean (roll +-"
                         f"{ac.WAIST_ROLL_TOL}, pitch {WAIST_PITCH_Q0_MIN}..{WAIST_PITCH_Q0_MAX}): skip table steps")
    elbow = round(float(pose[list(ac.ELBOWS)].max()), ROUND)
    pitch_label = round(float(pose[list(ac.SHOULDER_PITCHES)].min()), ROUND)
    return [(elbow, pitch_label, pose)]


def way_in_rows(cm, factories: list, setbacks, sampling: Sampling) -> list:
    """Block way_in: one row per (setback, case, factory pose)."""
    rows = []
    for setback, case in itertools.product(setbacks, scene_cases()):
        cm.set_setback(setback)
        set_case(cm, case)
        for e, s, factory in factories:
            path = arm_paths.way_in(factory, ac.READY17)
            minima, pair = leg_minima(cm, path, sampling)
            rows.append({**case, "setback_m": setback, "elbow": e, "shoulder_pitch": s, "leg_min_m": minima,
                         "factory_min_m": round(float(min(cm.clearance_of(path[0]).values())), ROUND),
                         "min_clearance_m": min(minima), "passed": min(minima) >= ac.CLEARANCE_MIN_M, "worst_pair": pair,
                         "seconds": round(arm_paths.path_seconds(path), 2)})
    reset_scene(cm)
    return rows


def recover_rows(cm, factories: list, setbacks, sampling: Sampling) -> list:
    """Block recover: one row per (setback, case, factory pose, way-in leg, fraction): the fault pose on that leg, then
    arm_paths.recover(..., WAY_IN, ...) back to the factory pose."""
    rows = []
    for setback, case in itertools.product(setbacks, scene_cases()):
        cm.set_setback(setback)
        set_case(cm, case)
        for e, s, factory in factories:
            plan = arm_paths.way_in(factory, ac.READY17)
            for leg, fraction in itertools.product(range(1, len(plan)), RECOVER_FRACTIONS):
                start = plan[leg - 1] + (plan[leg] - plan[leg - 1]) * fraction
                path = arm_paths.recover(start, arm_paths.WAY_IN, plan, leg)
                start_clear = cm.clearance_of(start)
                dense = [q for a, b in zip(path, path[1:]) for q in sampling.dense_leg(cm, a, b)[1:]]
                along, worst = ac.path_clearance([start] + dense, cm.clearance_of)
                rows.append({**case, "setback_m": setback, "elbow": e, "shoulder_pitch": s, "leg": leg,
                             "fraction": fraction, "margin_m": round(ac.exit_margin(start_clear, along), ROUND),
                             "passed": ac.exit_passes(start_clear, along), "worst_pair": cm.worst_pair(dense[worst - 1])})
    reset_scene(cm)
    return rows


def lift_home_rows(cm, starts_by_limit: dict, limits_lo17, limits_hi17, sampling: Sampling) -> list:
    """Block lift_home at the nominal setback: one row per (case, waist limit, start pose); the waypoints are the first
    four of arm_paths.way_out (current, lifted, waist home, ready)."""
    rows = []
    for case in scene_cases():
        set_case(cm, case)
        for limit, starts in starts_by_limit.items():
            for start in starts:
                path = arm_paths.way_out(start["q"], ac.READY17, ac.READY17, limits_lo17, limits_hi17)[:4]
                start_clear = cm.clearance_of(path[0])
                dense = [q for a, b in zip(path, path[1:]) for q in sampling.dense_leg(cm, a, b)[1:]]
                along, worst = ac.path_clearance([path[0]] + dense, cm.clearance_of)
                rows.append({**case, "waist_limit": limit, "frame": round(start["frame"], 3),
                             "start_clearance_m": {k: round(float(v), ROUND) for k, v in start_clear.items()},
                             "min_clearance_m": {k: round(float(v), ROUND) for k, v in along.items()},
                             "margin_m": round(ac.exit_margin(start_clear, along), ROUND),
                             "passed": ac.exit_passes(start_clear, along), "worst_pair": cm.worst_pair(dense[worst - 1]),
                             "seconds": round(arm_paths.path_seconds(path), 2)})
    reset_scene(cm)
    return rows


def _worst(rows: list, value) -> dict:
    row = min(rows, key=value)
    return {"value_m": value(row), "worst_pair": row["worst_pair"],
            "case": {k: row[k] for k in ("lean_roll", "lean_pitch", "table_dz")}}


def summarize(in_rows: list, recover: list, home_rows: list, settings: dict, sampling: Sampling) -> dict:
    """Per elbow: way in and recover-from-the-way-in clear at the nominal table (every pitch and case), smallest setback where
    both clear; per waist limit: lift_home clear; per (elbow, waist limit): way out = lift_home and the way in backwards."""
    nominal = [r for r in in_rows if r["setback_m"] == 0.0]
    home = {key(limit): [r for r in home_rows if r["waist_limit"] == limit] for limit in settings["waist_limits"]}
    home_clear = {k: bool(rows) and all(r["passed"] for r in rows) for k, rows in home.items()}
    elbows = []
    for e in settings["elbows"]:
        mine = [r for r in nominal if r["elbow"] == e]
        mine_recover = [r for r in recover if r["elbow"] == e and r["setback_m"] == 0.0]
        clear_at = [sb for sb in sorted(settings["setbacks"])
                    if all(r["passed"] for r in in_rows + recover if r["elbow"] == e and r["setback_m"] == sb)]
        way_in = all(r["passed"] for r in mine)
        elbows.append({"elbow": e, "way_in_clear": way_in, "way_in": _worst(mine, lambda r: r["min_clearance_m"]),
                       "recover_clear": all(r["passed"] for r in mine_recover),
                       "recover": _worst(mine_recover, lambda r: r["margin_m"]),
                       "factory_pose_min_m": min(r["factory_min_m"] for r in mine),
                       "smallest_clear_setback_m": clear_at[0] if clear_at else None,
                       "way_out_clear": {k: way_in and home_clear[k] for k in home},
                       "seconds_in": max(r["seconds"] for r in mine)})
    lift = {k: {"clear": home_clear[k], "passed": sum(r["passed"] for r in rows), "rows": len(rows),
                "worst": _worst(rows, lambda r: r["margin_m"]),
                "seconds_max": max(r["seconds"] for r in rows)} for k, rows in home.items() if rows}
    return {"CLEARANCE_MIN_M": ac.CLEARANCE_MIN_M, "MAX_SAMPLE_STEP_M": ac.MAX_SAMPLE_STEP_M,
            "max_sample_step_m": round(sampling.max_step, 5), "max_start_step_m": settings["max_start_step_m"],
            "samples": sampling.samples, "elbows": elbows, "lift_home": lift}
