"""Replay the grasp of scripts/69_walk_pick_place.py offline from many dock offsets: where does it pass in GR00T's crouch?

No walking, no LeRobot/DDS: g1sim/walk_grasp.offline_handover puts the walk world (approach_world, volleyball) into the state
GR00T leaves at the table (GROOT_HANDOVER: pelvis 0.750 m, leaning back, waist ~3 deg) with the robot at a chosen offset from
the dock pose, then runs the two-hand teacher exactly as 69 does after the handover. A grid over forward/sideways/yaw offsets
shows the window the walk has to hit.
    ~/dev/g1-ext/.venv/bin/python scripts/70_handover_sweep.py [--dx -0.03,0,0.03] [--dy -0.03,0,0.03] [--dyaw -3,0,3]
Output: runs/handover_sweep.json and a table per offset.
"""
import argparse
import itertools
import json
import math
import pathlib
import sys

import mujoco

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim import SIM_DIR, approach_world, ball_grasp, walk_grasp  # noqa: E402
from g1sim.navigation import Pose2D  # noqa: E402


def floats(text):
    return [float(v) for v in text.split(",")]


def main():
    ap = argparse.ArgumentParser(description="grasp after the handover, offline, over a grid of dock offsets")
    ap.add_argument("--dx", type=floats, default=[-0.03, 0.0, 0.03], help="m, towards the table")
    ap.add_argument("--dy", type=floats, default=[-0.03, 0.0, 0.03], help="m, to the left")
    ap.add_argument("--dyaw", type=floats, default=[-3.0, 0.0, 3.0], help="deg")
    ap.add_argument("--out", type=pathlib.Path, default=REPO / "runs/handover_sweep.json")
    args = ap.parse_args()
    model = mujoco.MjModel.from_xml_path(str(SIM_DIR / approach_world.write(volleyball=True)))
    data = mujoco.MjData(model)
    table = Pose2D(3.0, 1.0, math.radians(20.0))  # any pose: the grasp works in the table frame
    rows = []
    for dx, dy, dyaw in itertools.product(args.dx, args.dy, args.dyaw):
        env = walk_grasp.offline_handover(model, data, table, (dx, dy, math.radians(dyaw)))
        env.take_over()
        try:
            r = ball_grasp.TwoHandBallGrasp(env).run()
        finally:
            env.hand_back()
        forbidden = sorted({f"{b}->{o}" for v in r["forbidden_contacts"].values() for b, o in v})
        rows.append({"dx_m": dx, "dy_m": dy, "dyaw_deg": dyaw, "passed": r["passed"], "first_failed": r["first_failed"],
                     "ball_moved_before_touch_m": r["ball_moved_before_touch_m"], "touch_error_m": r["touch_error_m"],
                     "lift_height_m": r["lift_height_m"], "forbidden": forbidden[:6]})
        print(f"dx {dx * 100:+5.1f} cm  dy {dy * 100:+5.1f} cm  yaw {dyaw:+4.1f}: "
              f"{'ok  ' if r['passed'] else 'FAIL ' + str(r['first_failed'])}  moved before touch "
              f"{r['ball_moved_before_touch_m'] * 100:.1f} cm  {forbidden[:3] if forbidden else ''}", flush=True)
    passed = sum(r["passed"] for r in rows)
    print(f"{passed}/{len(rows)} passed")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({"passed": passed, "total": len(rows), "rows": rows}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
