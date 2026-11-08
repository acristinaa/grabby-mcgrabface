"""Table clearance of the robot-day arm paths by forward kinematics (RV6-1, R2-1): the numbers behind runbook Part C's M0 lookup.

Why it exists: the real factory arm hold pose is only known on Monday (M0). 86's scripted paths between it and the ready pose,
and the exits from a policy pose at the table (soft exit to the ready pose, fault ramp-out to the factory pose), are straight
joint lines or brackets of them; whether they clear the table edge depends on the factory elbow. 87 checks them in the sim
kinematics against table, stand and basket (5 cm margin), for a grid of factory elbows and shoulder pitches, every waist
limit Monday may use, and table setbacks (PLAN.md 3.5, rule 16). FK only: no physics step, no DDS, no GPU.
    ~/dev/g1-ext/.venv/bin/python scripts/87_arm_path_clearance.py --report runs/arm_path_clearance.json \
        --demo tests/data/replay_demo.npz
    ~/dev/g1-ext/.venv/bin/python scripts/87_arm_path_clearance.py --merge OUT.json IN1.json IN2.json ...   # runtime fallback
Block 1 (ready_paths): ready <-> hanging pose h(e, s). Block 2 (policy_exits, needs --demo): from movement-spaced demo poses,
the soft exit to the ready pose and the fault exits to every h(e, s) (worst of three orderings).
--arms-up-first (layout variant, 2026-10-27): the arms rise to the ready pose away from the table, then the table is pushed in
(or the robot is placed at it). Block 1 then never runs towards the table, so the table position no longer depends on the
factory elbow: block 2 checks every elbow at the nominal table and adds `86 release` (the 0.3 rad/s line to h(e, s)).
--waypoint-paths (2026-11-04): checks the waypoint paths of g1sim/arm_paths.py instead (g1sim/arm_paths_clearance.py) over the
torso lean grid and the table height range; needs --demo. .planning/real-robot/SAFE-PATHS.md reads its report.
--factory-pose runs/factory_pose_robot.json (with --waypoint-paths, on site after M0): the same check on the one recorded
factory pose instead of the h(e, s) grid, because grid rows do not stand in for poses between rows or inside the rule 16 (a)
tolerance (review safe-paths B3).
"""
import argparse
import json
import pathlib
import sys
import time

import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim import arm_clearance as ac  # noqa: E402
from g1sim import arm_clearance_report as report_lib  # noqa: E402
from g1sim import arm_paths_clearance as wp  # noqa: E402
from g1sim.arm_clearance_fk import ClearanceModel, check_scene  # noqa: E402

ROUND = 4   # decimals of clearances in the rows (0.1 mm)


def parse_range(text: str) -> list:
    """"0.0:1.6:0.1" (inclusive) or "-0.1,0.1,0.3" -> list of floats, rounded so equal grid values compare equal."""
    if ":" in text:
        lo, hi, step = (float(v) for v in text.split(":"))
        return [round(lo + k * step, 4) for k in range(int(round((hi - lo) / step)) + 1)]
    return [round(float(v), 4) for v in text.split(",")]


def rounded(clearance: dict) -> dict:
    return {name: round(float(clearance[name]), ROUND) for name in ac.OBJECTS}


class Counter:
    """Samples per block and the largest step left between two checked poses (asserted by the report, R4-1)."""

    def __init__(self):
        self.samples, self.max_step = {"ready_paths": 0, "policy_exits": 0}, 0.0

    def add(self, block: str, n: int, step: float) -> None:
        self.samples[block] += n
        self.max_step = max(self.max_step, step)


def ready_paths(cm: ClearanceModel, settings: dict, counter: Counter) -> list:
    """Block 1: real_safety.move(h(e, s), ready) - the exact to_ready line of `86 ready`; to_start/release run it backwards."""
    rows = []
    for setback in settings["setbacks"]:
        cm.set_setback(setback)
        for e in settings["elbows"]:
            for s in settings["shoulder_pitches"]:
                path, step = ac.densify_measured(ac.move_path(ac.hanging_pose(e, s), ac.READY17), cm.step_of)
                counter.add("ready_paths", len(path), step)
                worst = min(range(len(path)), key=lambda i: min(cm.clearance_of(path[i]).values()))
                rows.append({"elbow": e, "shoulder_pitch": s, "setback_m": setback,
                             "min_clearance_m": round(min(cm.clearance_of(path[worst]).values()), ROUND),
                             "worst_pair": cm.worst_pair(path[worst]), "worst_tick": worst})
    cm.set_setback(0.0)
    return rows


def release_row(cm: ClearanceModel, start: np.ndarray, goal: np.ndarray, start_clear: dict, counter: Counter) -> dict:
    """`86 release` from a policy pose (real_safety.release_plan): the same 0.3 rad/s joint line 86 sends to the factory pose.
    The fault exit's worst ordering brackets it from below; its own row tells whether the release step after a crash
    at the table is clear even when a torque-level fault blend is not (arms-up-first variant)."""
    path, step = ac.densify_measured(ac.move_path(start, goal), cm.step_of)
    counter.add("policy_exits", len(path), step)
    along, worst = ac.path_clearance(path, cm.clearance_of)
    return {"exit": "release", "ordering": "straight", "start_clearance_m": rounded(start_clear),
            "min_clearance_m": rounded(along), "passed": ac.exit_passes(start_clear, along), "worst_pair": cm.worst_pair(path[worst])}


def exits_from(cm: ClearanceModel, start: np.ndarray, elbows: list, settings: dict, counter: Counter) -> list:
    """Block 2 rows of one start pose at the current setback: the soft exit to the ready pose, and the fault exits to h(e, s)
    for the given elbows (those whose table position is this setback) and every shoulder pitch; with --arms-up-first also
    the `86 release` line to each h(e, s)."""
    start_clear = cm.clearance_of(start)
    to_ready, step = ac.densify_measured(ac.move_path(start, ac.READY17), cm.step_of)
    counter.add("policy_exits", len(to_ready), step)
    along, worst = ac.path_clearance(to_ready, cm.clearance_of)
    rows = [{"exit": "to_ready", "ordering": "straight", "elbow": None, "shoulder_pitch": None,
             "start_clearance_m": rounded(start_clear), "min_clearance_m": rounded(along),
             "passed": ac.exit_passes(start_clear, along), "worst_pair": cm.worst_pair(to_ready[worst])}]
    for e in elbows:
        for s in settings["shoulder_pitches"]:
            fault = ac.worst_fault_exit(start, ac.hanging_pose(e, s), start_clear, cm.step_of, cm.clearance_of)
            counter.add("policy_exits", fault["samples"], fault["max_step"])
            rows.append({"exit": "to_hanging", "ordering": fault["ordering"], "elbow": e, "shoulder_pitch": s,
                         "start_clearance_m": rounded(start_clear), "min_clearance_m": rounded(fault["along"]),
                         "passed": fault["passed"], "worst_pair": None})
            if settings["arms_up_first"]:
                rows.append({**release_row(cm, start, ac.hanging_pose(e, s), start_clear, counter), "elbow": e, "shoulder_pitch": s})
    return rows


def policy_exits(cm: ClearanceModel, state: np.ndarray, settings: dict, elbows_at: dict, counter: Counter) -> tuple:
    """Block 2 for every waist limit and block-2 setback: (rows, start poses per waist limit, max_start_step_m).

    elbows_at = {setback: elbows whose table position it is}: fault exits are computed only at an elbow's own setback,
    the one rule 16 (c) reads (fault_exits_clear); soft exits (to_ready) at every block-2 setback."""
    rows, start_counts, max_start, done = [], {}, 0.0, {}
    for limit in settings["waist_limits"]:
        starts, start_step = ac.select_start_poses(ac.demo_frames(state, limit), cm.step_of)
        start_counts[report_lib.key(limit)], max_start = len(starts), max(max_start, start_step)
        for setback, elbows in elbows_at.items():
            cm.set_setback(setback)
            for start in starts:
                pose_key = (setback, np.round(start["q"], 6).tobytes())   # reused when clipping left the pose unchanged
                if pose_key not in done:
                    done[pose_key] = exits_from(cm, start["q"], elbows, settings, counter)
                rows += [{**row, "frame": round(start["frame"], 3), "waist_limit": limit, "setback_m": setback}
                         for row in done[pose_key]]
        print(f"  waist {limit}: {len(starts)} start poses, max start step {start_step * 100:.2f} cm", flush=True)
    cm.set_setback(0.0)
    return rows, start_counts, max_start


def scene_settings(cm: ClearanceModel, args: argparse.Namespace) -> dict:
    setbacks = parse_range(args.setbacks)
    return {"elbows": parse_range(args.elbows), "shoulder_pitches": parse_range(args.shoulder_pitches),
            "setbacks": setbacks, "waist_limits": parse_range(args.waist_limits),
            "demo": None if args.demo is None else str(args.demo), "arms_up_first": args.arms_up_first,
            "CLEARANCE_MIN_M": ac.CLEARANCE_MIN_M,
            "MAX_SAMPLE_STEP_M": ac.MAX_SAMPLE_STEP_M, "GRID_BOUNDS": ac.GRID_BOUNDS,
            "fault_exits_at": "each elbow's own table setback (the one rule 16 (c) reads)",
            "stand_on_table": {report_lib.key(s): cm.stand_on_table(s) for s in setbacks},
            "ball_distance_m": {report_lib.key(s): round(cm.ball_distance(s), ROUND) for s in setbacks}}


def compute(args: argparse.Namespace) -> dict:
    started = time.monotonic()
    cm = ClearanceModel()
    check_scene(cm)
    settings, counter = scene_settings(cm, args), Counter()
    n_paths = len(settings["setbacks"]) * len(settings["elbows"]) * len(settings["shoulder_pitches"])
    print(f"block 1: {n_paths} ready paths", flush=True)
    ready_rows = ready_paths(cm, settings, counter)
    needed = report_lib.setback_needed(ready_rows, settings["elbows"], settings["shoulder_pitches"], settings["setbacks"])
    setbacks2 = sorted({0.0} | {s for s in needed.values() if s is not None})
    elbows_at = {sb: [e for e in settings["elbows"] if needed[f"{e:.1f}"] == sb] for sb in setbacks2}
    if args.arms_up_first:                   # the table is pushed in only once the arms are up: every elbow meets it at nominal
        elbows_at = {0.0: settings["elbows"]}
    policy, start_counts, max_start = None, {}, 0.0
    if args.demo is not None:
        state = np.load(args.demo)["state"]
        print(f"block 2: {len(state)} demo frames, setbacks {setbacks2}, waist limits {settings['waist_limits']}", flush=True)
        policy_rows, start_counts, max_start = policy_exits(cm, state, settings, elbows_at, counter)
        policy = {"rows": policy_rows}
    stats = {"samples": counter.samples, "start_poses": start_counts, "max_sample_step_m": round(counter.max_step, 5),
             "max_start_step_m": round(max_start, 5), "elapsed_s": round(time.monotonic() - started, 1)}
    report = {"settings": settings, "stats": stats, "ready_paths": {"rows": ready_rows}, "policy_exits": policy,
              "summary": report_lib.summarize(ready_rows, policy["rows"] if policy else None, settings, stats)}
    if policy is None:
        report["reason"] = "pending integration: run with --demo tests/data/replay_demo.npz (policy_exits not computed)"
    return report


def compute_waypoints(args: argparse.Namespace) -> dict:
    """--waypoint-paths: way_in rows for every h(e, s), setback and scene case; lift_home rows from the demo start poses."""
    from g1sim.real_preflight import LIMITS_HI, LIMITS_LO   # 86's joint limits (needs the sim env: real_io imports cyclonedds)
    started, cm, sampling = time.monotonic(), ClearanceModel(), wp.Sampling()
    check_scene(cm)
    settings = {"elbows": parse_range(args.elbows), "shoulder_pitches": parse_range(args.shoulder_pitches),
                "setbacks": parse_range(args.setbacks), "waist_limits": parse_range(args.waist_limits),
                "demo": str(args.demo), "scene_cases": wp.scene_cases(), "CLEARANCE_MIN_M": ac.CLEARANCE_MIN_M,
                "factory_pose": None if args.factory_pose is None else str(args.factory_pose)}
    if args.factory_pose is None:
        factories = wp.grid_factories(settings["elbows"], settings["shoulder_pitches"])
    else:
        factories = wp.measured_factory(json.loads(args.factory_pose.read_text())["q"])
        settings["elbows"], settings["shoulder_pitches"] = [factories[0][0]], [factories[0][1]]
        settings["factory_q"] = [round(float(v), ROUND) for v in factories[0][2]]
    print(f"way in + recover: {len(settings['setbacks'])} setbacks x {len(settings['scene_cases'])} cases", flush=True)
    in_rows = wp.way_in_rows(cm, factories, settings["setbacks"], sampling)
    recover_rows = wp.recover_rows(cm, factories, settings["setbacks"], sampling)
    state, starts, max_start = np.load(args.demo)["state"], {}, 0.0
    for limit in settings["waist_limits"]:
        starts[limit], step = ac.select_start_poses(ac.demo_frames(state, limit), cm.step_of)
        max_start = max(max_start, step)
    settings["max_start_step_m"] = round(max_start, 5)
    print(f"lift home: start poses {[len(v) for v in starts.values()]}", flush=True)
    home_rows = wp.lift_home_rows(cm, starts, LIMITS_LO, LIMITS_HI, sampling)
    return {"settings": settings, "way_in": {"rows": in_rows}, "recover": {"rows": recover_rows},
            "lift_home": {"rows": home_rows},
            "summary": wp.summarize(in_rows, recover_rows, home_rows, settings, sampling),
            "elapsed_s": round(time.monotonic() - started, 1)}


def print_waypoint_summary(report: dict) -> None:
    """One line per elbow (way in, recover from the way in, factory pose, way out per waist limit), then one per waist limit
    (lift home)."""
    s = report["summary"]
    for row in s["elbows"]:
        out = ", ".join(f"{k} {'clear' if v else 'NOT clear'}" for k, v in row["way_out_clear"].items())
        print(f"elbow {row['elbow']:g}: way in {cm_text(row['way_in_clear'], row['way_in']['value_m'])} "
              f"[{row['way_in']['worst_pair']}]; recover from the way in {'clear' if row['recover_clear'] else 'NOT clear'} "
              f"(worst margin {row['recover']['value_m'] * 100:.1f} cm); "
              f"factory pose min {row['factory_pose_min_m'] * 100:.1f} cm; "
              f"smallest clear setback {row['smallest_clear_setback_m']}; way out per waist: {out}; "
              f"way in {row['seconds_in']:.1f} s")
    for k, v in s["lift_home"].items():
        print(f"lift home waist {k}: {v['passed']}/{v['rows']} pass, worst margin {v['worst']['value_m'] * 100:.1f} cm "
              f"[{v['worst']['worst_pair']}] at {v['worst']['case']}; longest {v['seconds_max']:.1f} s")
    print(f"max sample step {s['max_sample_step_m'] * 100:.2f} cm, max start step {s['max_start_step_m'] * 100:.2f} cm, "
          f"samples {s['samples']}, elapsed {report['elapsed_s']} s")


def print_summary(report: dict) -> None:
    """One line per elbow and waist limit (PLAN.md 3.5), then the sampling numbers and the elapsed time."""
    s = report["summary"]
    for row in s["elbows"]:
        table = {"nominal": "table at nominal", "skip": "skip table steps"}.get(row["table_steps"]) or \
            f"move the table back {round(row['setback_m'] * 100)} cm"
        for limit in s["WAIST_LIMITS"]:
            k, sb = report_lib.key(limit), row["setback_m"]
            replay = s["replay_exits_clear"].get(report_lib.key(sb), {}).get(k) if sb is not None else None
            inside = s["frames_inside"].get(report_lib.key(sb), {}).get(k, {}).get("table") if sb is not None else None
            print(f"elbow {row['elbow']:.1f} waist {limit}: {table}; replay exits {'clear' if replay else 'NOT clear'}; "
                  f"fault exits {'clear' if row['fault_exits_clear'][k] else 'NOT clear'}; frames inside the table {inside}")
    print(f"max sample step {s['max_sample_step_m'] * 100:.2f} cm (limit {s['MAX_SAMPLE_STEP_M'] * 100:.0f} cm), max start "
          f"step {s['max_start_step_m'] * 100:.2f} cm, start poses {s['start_poses']}, samples {s['samples']}")
    print(f"setback keeps layout up to {s['setback_max_keep_layout_m']} m, ball in reach up to {s['setback_max_policy_m']} m; "
          f"elapsed {report['stats']['elapsed_s']} s")
    if "arms_up_first" in s:
        print_arms_up_first(s["arms_up_first"])


def cm_text(clear: bool, min_m: float) -> str:
    """"clear (min 6.2 cm)" or "NOT clear (min -1.4 cm)"."""
    return f"{'clear' if clear else 'NOT clear'} (min {min_m * 100:.1f} cm)"


def print_arms_up_first(variant: dict) -> None:
    """The variant's verdict per exit at the nominal table: one line per waist limit, then one per elbow and waist limit."""
    print("arms-up-first (table pushed in after the arms are up; everything at the nominal table):")
    for k, w in variant["waists"].items():
        path = ", ".join(f"{name} {v['min_m'] * 100:.1f} cm @frame {v['frame']}" for name, v in w["demo_path"].items())
        print(f"  waist {k}: demo path (ready->grasp->basket) min {path}; start poses inside {w['frames_inside']}; "
              f"soft exit to ready {cm_text(w['to_ready']['clear'], w['to_ready']['min_m'])}")
    for row in variant["elbows"]:
        ready = row["ready_to_factory"]
        for k, w in row["waists"].items():
            print(f"  elbow {row['elbow']:.1f} waist {k}: ready->factory (soft exit leg 2, release at ready) "
                  f"{cm_text(ready['clear'], ready['min_m'])}; fault exits {cm_text(w['fault']['clear'], w['fault']['min_m'])}; "
                  f"release {cm_text(w['release']['clear'], w['release']['min_m'])}")


def check_sampling(report: dict) -> None:
    """Fail loudly if the report breaks the 2 cm sampling rule (R4-1, R5-1): never trust such a lookup."""
    s = report["summary"]
    for name in ("max_sample_step_m", "max_start_step_m"):
        if s[name] > s["MAX_SAMPLE_STEP_M"]:
            raise RuntimeError(f"87: {name} {s[name]} > {s['MAX_SAMPLE_STEP_M']} - the sampling is too coarse; this is a bug")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="FK clearance of the robot-day arm paths (PLAN.md 3.5)")
    parser.add_argument("--report", type=pathlib.Path, help="output JSON, e.g. runs/arm_path_clearance.json")
    parser.add_argument("--elbows", default="0.0:1.6:0.1")
    parser.add_argument("--shoulder-pitches", default="-0.1,0.1,0.3")
    parser.add_argument("--setbacks", default="0.0:0.15:0.01")
    parser.add_argument("--waist-limits", default=",".join(str(w) for w in ac.WAIST_LIMITS))
    parser.add_argument("--demo", type=pathlib.Path, help="demo npz whose states stand in for the policy's poses")
    parser.add_argument("--arms-up-first", action="store_true",
                        help="layout variant: arms rise to the ready pose away from the table, then the table is pushed in")
    parser.add_argument("--waypoint-paths", action="store_true",
                        help="check g1sim/arm_paths.py's waypoint paths over lean and table height cases (needs --demo)")
    parser.add_argument("--factory-pose", type=pathlib.Path,
                        help="with --waypoint-paths: check the M0 record (84 --record-factory-pose JSON) instead of the grid")
    parser.add_argument("--merge", nargs="+", type=pathlib.Path, metavar=("OUT", "IN"), help="merge partial reports")
    args = parser.parse_args(argv)
    if args.merge:
        if len(args.merge) < 3:
            parser.error("--merge needs OUT.json and at least two input reports")
        report = report_lib.merge_reports([json.loads(p.read_text()) for p in args.merge[1:]])
        out = args.merge[0]
    elif args.report and args.waypoint_paths:
        if args.demo is None:
            parser.error("--waypoint-paths needs --demo (the start poses of the way out)")
        report, out = compute_waypoints(args), args.report
    elif args.report:
        report, out = compute(args), args.report
    else:
        parser.error("give --report PATH (or --merge OUT IN1 IN2 ...)")
    check_sampling(report)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report))
    (print_waypoint_summary if args.waypoint_paths else print_summary)(report)
    print(f"REPORT {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
