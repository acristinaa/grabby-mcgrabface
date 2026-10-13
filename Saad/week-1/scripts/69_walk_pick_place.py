"""Level 1 + walk, in ONE continuous sim: walk up to the table, switch GR00T off, pick the volleyball up and put it in the basket.

Per trial (g1sim/walk_grasp.py has the details of the handover):
  1. walk exactly as scripts/68_walk_to_table.py (random scenario: table ahead, behind, beside, turned away, obstacles):
     arms down while walking, align at the checkpoint, arms up to BALL_READY_POSE, straight in, dock. Ball on its stand,
     stand and basket on the table.
  2. handover: GR00T switched off (the sim thread is held), pelvis welded where the walk stopped, every joint held.
  3. the two-hand teacher (g1sim/ball_grasp.py) from where the robot really stands: grasp, lift, turn, drop into the basket,
     arms back to the start pose, waist to the front, 3 s placement check.
  4. hand back to GR00T; the next trial walks off from there.
Checks per trial, in this order (the first failed one is the reason):
  walk:   no_fall, no_furniture_contact, arrived_in_time, dock_within_tolerance   (as 68)
          ball_untouched_by_walk  the ball moved < 1 cm on its stand until the handover
  grasp:  the checks of 67 (no_forbidden_contacts ... hands_off_ball)
Environment ~/dev/g1-ext (as 68), never together with another DDS sim:
    DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib PYTHONUNBUFFERED=1 \\
        ~/dev/g1-ext/.venv/bin/python scripts/69_walk_pick_place.py [--trials 6] [--seed 0] [--scenarios a,b] [--video]
Watch: http://127.0.0.1:8020 (camera + top view, the whole run) and the lab view (scripts/65_labview.py, :8030).
Output: runs/level1_walk_pick_place.json; with --video docs/videos/level1_walk_pick_place.mp4 (walk and grasp in one film).
"""
import argparse
import importlib.util
import json
import pathlib
import sys
import time
import traceback

import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim.walk_sim import WalkSim  # noqa: E402  (first: sets up DDS before LeRobot)
from g1sim import approach, approach_world, ball_grasp, labview, walk_grasp  # noqa: E402

_spec = importlib.util.spec_from_file_location("walk_to_table", REPO / "scripts" / "68_walk_to_table.py")
walk = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(walk)

OUT = REPO / "runs/level1_walk_pick_place.json"
VIDEO = REPO / "docs/videos/level1_walk_pick_place.mp4"
BALL_MOVED_MAX = 0.01  # m, ball movement on its stand during the walk that still counts as untouched
WALK_CHECKS = (*walk.CHECKS, "ball_untouched_by_walk")
GRASP_CHECKS = ("no_forbidden_contacts", "ball_lifted", "ball_carried_to_basket", "ball_in_basket", "ball_on_basket_floor",
                "ball_at_rest", "hands_off_ball")


class _WalkPart:
    """The lab view reporter for run_trial of 68: passes everything on except trial_end (the trial goes on with the grasp)."""

    def __init__(self, reporter):
        self.reporter = reporter

    def trial_start(self, *a, **k):
        self.reporter.trial_start(*a, **k)

    def on_tick(self, phase):
        self.reporter.on_tick(phase)

    def trial_end(self, *a, **k):
        pass


def grasp(sim, env, reporter, label):
    """Handover, teacher, hand back. Returns the grasp part of the result."""
    handover = env.take_over()
    try:
        def on_tick(phase):
            if reporter:
                reporter.on_tick(f"grasp_{phase}")

        try:
            result = ball_grasp.TwoHandBallGrasp(env, on_tick=on_tick).run()
        except Exception as error:  # one trial must not end the run
            traceback.print_exc()
            result = {"error": f"{type(error).__name__}: {error}", "checks": {"run_without_error": False},
                      "passed": False, "first_failed": "run_without_error"}
        result["handover"] = handover
        result["sim_s"] = round(float(env.data.time - env._sim0), 1)
        return result
    finally:
        env.hand_back()


def main():
    ap = argparse.ArgumentParser(description="Walk up to the table, then pick and place the volleyball, in one sim")
    ap.add_argument("--trials", type=int, default=6)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--scenarios", default=",".join(approach.SCENARIOS),
                    help=f"comma-separated, cycled in order: {','.join(approach.SCENARIOS)}")
    ap.add_argument("--speed", type=float, default=1.0, help="grasp pacing: 1 = real time (as the walk), 0 = as fast as possible")
    ap.add_argument("--video", nargs="?", const=str(VIDEO), default=None, help="render an mp4 (no path: default path)")
    ap.add_argument("--no-live", action="store_true", help="no live view in the browser")
    ap.add_argument("--no-labview", action="store_true", help="write no lab view files (runs/labview/)")
    ap.add_argument("--out", default=str(OUT))
    args = ap.parse_args()
    scenarios = args.scenarios.split(",")
    unknown = set(scenarios) - set(approach.SCENARIOS)
    if unknown:
        ap.error(f"unknown scenarios {sorted(unknown)}")

    rng = np.random.default_rng(args.seed)
    sim = WalkSim(scene=approach_world.write(volleyball=True), live=not args.no_live, video=args.video,
                  title="Walk up and pick and place")
    furniture = walk_grasp.VolleyballFurniture(sim.model, sim.data, sim.sim._step_lock)
    furniture.park()
    phase_now = {"label": ""}
    env = walk_grasp.HandoverEnv(sim, speed=args.speed, on_step=lambda: sim.show(Phase=phase_now["label"]))
    trial_t0 = [sim.time]
    reporter = None if args.no_labview else labview.Reporter(
        script="69", task="walk_pick_place", object="volleyball", trials=args.trials, argv=sys.argv[1:],
        frame_getter=(lambda: sim.live.frame) if sim.live else None, sim_time_getter=lambda: sim.time - trial_t0[0])
    results, done_lines = [], []
    arms = {"now": np.zeros(walk.action_space.DIM, dtype=np.float32)}
    wall0, sim0 = time.perf_counter(), sim.time
    try:
        ok = sim.release_band() and walk.move_arms(sim, arms, walk.ARMS_DOWN, "Arms to the walking pose")
        for i in range(args.trials if ok else 0):
            trial_t0[0] = sim.time
            scenario = scenarios[i % len(scenarios)]
            try:
                w = walk.run_trial(sim, furniture, reporter and _WalkPart(reporter), rng, i, args.trials, scenario,
                                   done_lines, "smooth", arms, walk.ARMS_DOWN, True)
            except RuntimeError as error:
                w = {"trial": i, "scenario": scenario, "passed": False, "checks": {}, "first_failed": "arrived_in_time",
                     "error": f"{type(error).__name__}: {error}"}
            moved = furniture.ball_moved()
            w.setdefault("checks", {})["ball_untouched_by_walk"] = moved < BALL_MOVED_MAX
            for k in ("dock_log", "trail", "contacts"):  # big, and in 68's output already
                w.pop(k, None)
            result = {"trial": i, "scenario": scenario, "walk": w, "ball_moved_by_walk_m": round(moved, 4), "grasp": None}
            if w.get("passed") and moved < BALL_MOVED_MAX:
                phase_now["label"] = "pick and place (GR00T off, pelvis held)"
                result["grasp"] = grasp(sim, env, reporter, phase_now["label"])
                arms["now"] = np.array(env.ready_pose, dtype=np.float32)  # the teacher ends in its start pose
                sim.send(0.0, 0.0, 0.0, arms=arms["now"])
                first_failed = result["grasp"]["first_failed"]
            else:
                first_failed = next((k for k in WALK_CHECKS if not w["checks"].get(k, False)), "arrived_in_time")
            result["passed"] = first_failed is None
            result["first_failed"] = first_failed
            results.append(result)
            if reporter:
                reporter.trial_end(result["passed"], first_failed, {"error": w.get("error") or (result["grasp"] or {}).get("error")})
            e = w.get("error_m_m_deg")
            g = result["grasp"]
            line = (f"{i + 1} {scenario}: {'ok' if result['passed'] else 'FAIL ' + str(first_failed)}"
                    + (f" | docked {100 * e[0]:+.1f} / {100 * e[1]:+.1f} cm, {e[2]:+.1f} deg" if e else "")
                    + (f" | lift {100 * g['lift_height_m']:.1f} cm, end {100 * g['ball_to_basket_centre_m']:.1f} cm from basket centre"
                       if g and "lift_height_m" in g else "")
                    + (f" | handover pelvis z {g['handover']['pelvis_xyz'][2]:.3f} m" if g else ""))
            done_lines.append(line)
            print(f"Trial {line}" + (f" - {w['error']}" if "error" in w else ""), flush=True)
            if w.get("checks", {}).get("no_fall") is False and not walk.recover(sim, furniture, arms):
                print("could not stand up again after the fall - stopping")
                break
            if not sim.wait(1.0, "standing after the grasp"):
                print(f"fell after the hand back to GR00T: {sim.fault}")
                result["fell_after_hand_back"] = True
                if not walk.recover(sim, furniture, arms):
                    break
        realtime = round((sim.time - sim0) / (time.perf_counter() - wall0), 3)
    finally:
        env.hand_back()
        sim.close()
    passed = sum(r["passed"] for r in results)
    failures = {}
    for r in results:
        if r["first_failed"]:
            failures[r["first_failed"]] = failures.get(r["first_failed"], 0) + 1
    report = {"seed": args.seed, "scenarios": scenarios, "trials": args.trials, "passed_trials": passed,
              "success_rate": passed / args.trials if args.trials else None, "failures_by_first_failed_check": failures,
              "walk_passed": sum(bool(r["walk"].get("passed")) for r in results),
              "grasp_passed_of_handovers": f"{sum(bool(r['grasp'] and r['grasp']['passed']) for r in results)}/"
                                           f"{sum(r['grasp'] is not None for r in results)}",
              "realtime_factor": realtime if results else None, "results": results, "passed": passed == args.trials}
    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=1, ensure_ascii=False, default=str))
    if reporter:
        reporter.finish(summary={k: report[k] for k in ("walk_passed", "grasp_passed_of_handovers")}, report_path=out)
    print(json.dumps({k: report[k] for k in ("passed_trials", "trials", "walk_passed", "grasp_passed_of_handovers",
                                             "failures_by_first_failed_check", "realtime_factor")}, indent=1))
    print(f"saved: {out}" + (f", {args.video}" if args.video else ""))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
