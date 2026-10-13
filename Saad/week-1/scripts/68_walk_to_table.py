"""Level 4 step - walk up to the table from random start situations and dock at the grasp pose (g1sim/approach.py).

Per trial the table (and 0-3 obstacles) is put at a random pose relative to where the robot stands now, from one of the
scenarios in approach.SCENARIOS (cycled in order, so every kind comes up):
  facing           table ahead, work side towards the robot
  behind_table     robot behind the table: walk all the way round to the work side
  turned_away      table behind the robot: turn round, then walk up
  beside           table to the left or right, any orientation
  obstacle_course  table 3-4 m ahead, 1-3 obstacles in between
  anywhere         any direction, distance and orientation, 0-2 obstacles
Sequence per trial (the user's spec, 2026-10-09):
  1. arms down (ARMS_DOWN, hanging by the body like a person) and walk: A* map + path around table and obstacles (as in L2),
     pulled taut and smoothed, ending with a straight lead-in onto the line into the table -> calm path following (pure
     pursuit with a speed profile, navigation.SmoothPathFollower; --follower grid = the L2 follower on the raw grid path)
  2. checkpoint, 0.75 m (~3-4 steps) before the grasp pose: align on the line, facing the table (approach.Docker with ALIGN,
     +-3 cm / 2.5 deg, again if > 4 cm / 3 deg after standing 1 s) - small side steps are fine here, away from the table
  3. standing still: arms up to action_space.BALL_READY_POSE (the volleyball teacher's start pose) in 2 s
  4. straight in (approach.StraightIn): forward steps only, no side steps - no "penguin walk" with the body turned away;
     the heading only steers back onto the line and straightens over the last 0.3 m; stop, stand 1 s, measure, go again
     forwards/backwards if more than 4 cm off along the table axis (up to 3 attempts)
The next trial starts where this one ended (arms go down again first).
The robot's position comes from the sim (known pose, as L1/L2); localization is L7.
Checks per trial, in this order (cause -> effect, the first failed one is the reason):
  1. no_fall                 pelvis never below 0.5 m
  2. no_furniture_contact    no robot part touched the table or an obstacle
  3. arrived_in_time         docked within path length / 0.2 m/s + 30 s sim time
  4. dock_within_tolerance   after 1 s standing: pelvis within +-10 cm (each axis) and +-10 deg of the grasp pose, in the
                             table frame - the stand offsets the volleyball teacher handles (100/100, 2026-10-08)
Environment ~/dev/g1-ext (as 61/62), never together with another DDS sim:
    DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib PYTHONUNBUFFERED=1 \
        ~/dev/g1-ext/.venv/bin/python scripts/68_walk_to_table.py [--trials 12] [--seed 0] [--scenarios a,b] [--video]
Watch: http://127.0.0.1:8020 (camera + top view; next free port if taken) and the lab view (scripts/65_labview.py, :8030).
Output: runs/level4_walk_to_table.json, all trials in the table frame docs/level4_walk_to_table.png, with --video
docs/videos/level4_walk_to_table.mp4.
"""
import argparse
import dataclasses
import json
import math
import os
import pathlib
import sys
import time
import traceback

import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim.walk_sim import WalkSim  # noqa: E402  (first: sets up DDS before LeRobot)
from g1sim import action_space, approach, approach_world, arm_swing, labview, occupancy  # noqa: E402
from g1sim.navigation import CONTROL_HZ, PathFollower, SmoothPathFollower  # noqa: E402

OUT = REPO / "runs/level4_walk_to_table.json"
PLOT = REPO / "docs/level4_walk_to_table.png"
VIDEO = REPO / "docs/videos/level4_walk_to_table.mp4"
# Arms: down by the body while walking (natural, like a person), up to the grasp start pose at the checkpoint, before the
# robot walks into the table. Pose found by forward kinematics at walking height (2026-10-09): hands beside the hips at
# ~0.47 m, 12 cm in front of the pelvis, 8 cm from the legs. Right arm mirrored (shoulder roll negative).
ARM_DOWN = (0.2, 0.3, 0.0, 1.2, 0.0, 0.0, 0.0)  # shoulder pitch, roll, yaw, elbow, wrist roll, pitch, yaw (rad)
ARMS_DOWN = np.array(ARM_DOWN + (ARM_DOWN[0], -ARM_DOWN[1]) + ARM_DOWN[2:] + (0.0,) * 12 + (0.0,), dtype=np.float32)  # hands open, waist 0
ARMS_UP = action_space.BALL_READY_POSE  # the volleyball teacher's start pose (elbows beside the body, hands at chest height)
ARMS_RAMP_S = 2.0       # standing still: arms from one pose to the other
SWITCH_TO_ALIGN = 0.20  # m from the checkpoint (or the end of the path): stop following, align
MEASURE_S = 1.0         # stopped: stand this long, then measure (and go again if needed)
MIN_SPEED = 0.2         # m/s for the time limit: path length / MIN_SPEED + TIME_EXTRA
TIME_EXTRA = 45.0
TRAIL_POINTS = 800
MAP_INTERVAL_S = 0.2    # wall seconds between two top-view updates (the browser polls every 0.5 s)
ORANGE, LIGHT_ORANGE = (0.91, 0.40, 0.0, 0.8), (0.96, 0.66, 0.38, 0.6)
CHECKS = ("no_fall", "no_furniture_contact", "arrived_in_time", "dock_within_tolerance")


def top_view(robot, setup, path, trail, dock, check):
    """Map for the live page (g1sim/liveview.py "_map"): square bounds around everything, outlines, path, poses
    ("pre" = the checkpoint)."""
    table = approach.corners(*approach.table_rect(setup.table))
    obstacles = [approach.corners(*approach.obstacle_rect(o)) for o in setup.obstacles]
    pts = np.array([(robot.x, robot.y), *table, *(p for o in obstacles for p in o)])
    lo, hi = pts.min(axis=0) - 1.0, pts.max(axis=0) + 1.0
    centre, half = (lo + hi) / 2, float(max(hi - lo)) / 2
    r = lambda v: [round(float(c), 3) for c in v]  # noqa: E731
    return {"bounds": r([centre[0] - half, centre[0] + half, centre[1] - half, centre[1] + half]),
            "table": [r(p) for p in table], "obstacles": [[r(p) for p in o] for o in obstacles],
            "path": [r(p) for p in path[::max(1, len(path) // 80)]], "trail": [r(p) for p in trail[::max(1, len(trail) // 200)]],
            "robot": r([robot.x, robot.y, robot.yaw]), "dock": r([dock.x, dock.y, dock.yaw]), "pre": r([check.x, check.y, check.yaw])}


def path_deviation(trail, path):
    """Distance (m) of each walked point to the path it should follow (path densified to ~1 cm)."""
    path = np.asarray(path, float)
    dense = np.concatenate([np.linspace(a, b, max(2, int(np.linalg.norm(b - a) / 0.01)), endpoint=False)
                            for a, b in zip(path[:-1], path[1:])] + [path[-1:]])
    walked = np.asarray(trail, float)
    return np.array([np.min(np.linalg.norm(dense - p, axis=1)) for p in walked])


def run_trial(sim, furniture, reporter, rng, i, n, scenario, done_lines, follower_kind="smooth", arms=None, walk_arms=None,
              swing_on=True):
    """One trial: arms down, draw, place, plan, walk, align at the checkpoint, arms up, straight in, measure.
    arms: {"now": 27-D vector} - the arm pose currently held (shared with main). walk_arms: pose while walking (ARMS_DOWN;
    ARMS_UP only for the comparison --walk-arms up). Returns the result dict."""
    walk_arms = ARMS_DOWN if walk_arms is None else walk_arms
    swing = arm_swing.ArmSwing() if swing_on and np.allclose(walk_arms, ARMS_DOWN) else None
    offsets, last_t = ((0.0, 0.0), (0.0, 0.0)), None
    reporter and reporter.trial_start(i, {"scenario": scenario})
    furniture.park()
    if not np.allclose(arms["now"], walk_arms) and not move_arms(sim, arms, walk_arms, "Arms to the walking pose"):
        raise RuntimeError("fell while moving the arms")
    start = sim.pose()
    setup = approach.sample(scenario, rng, start)
    path = approach.plan_smooth(start, setup) if follower_kind == "smooth" else approach.plan(start, setup)
    furniture.place(setup)
    dock, check = approach.dock_pose(setup.table), approach.checkpoint_pose(setup.table)
    length = occupancy.path_length(path)
    limit = length / MIN_SPEED + TIME_EXTRA
    follower = SmoothPathFollower(path, check) if follower_kind == "smooth" else PathFollower(path, check)
    aligner = approach.Docker(setup.table, approach.ALIGN, dist=approach.CHECKPOINT_DIST)
    straight = approach.StraightIn(setup.table)
    description = approach.SCENARIOS[scenario][1]
    t0 = next_t = sim.time
    phase, outcome, stopped_at, raise_from = "walk", None, None, None
    cmd, touched, trail, fell = (0.0, 0.0, 0.0), set(), [], False
    walk_log = []  # (t, x, y, vx, vy, omega) while following the path
    dock_log = []  # (t, phase, dx, dy, dyaw deg, vx, vy, omega) from the checkpoint on, in the table frame
    phase_t = {}   # phase -> sim time it started
    align_error = None
    contact_log = {}  # (piece geom, robot body) -> [phase, sim time] of the first touch
    start_error = approach.table_frame_error(start, setup.table)
    shown = {"map": None, "next": 0.0}
    dense_path = np.concatenate([np.linspace(a, b, max(2, int(np.linalg.norm(np.subtract(b, a)) / 0.02)), endpoint=False)
                                 for a, b in zip(path[:-1], path[1:])] + [np.asarray(path[-1:], float)])
    off_path = []  # distance to the path at each walking tick (m), for the live view
    swing_log, arm_leg_hits = [], 0

    def look(phase):
        """Pose, contacts, live view and lab view for this loop pass. Returns the pose."""
        pose = sim.pose()
        if time.monotonic() >= shown["next"]:  # the map is not needed on every pass of a 2 ms loop
            shown["map"], shown["next"] = top_view(pose, setup, path, trail, dock, check), time.monotonic() + MAP_INTERVAL_S
        for piece, geom, body in furniture.contacts():
            touched.add(piece)
            contact_log.setdefault(f"{body} -> {geom}", [phase, round(sim.time - t0, 2)])
        error = approach.table_frame_error(pose, setup.table)
        sim.show(markers=[(dock.x, dock.y, dock.yaw, ORANGE), (check.x, check.y, check.yaw, LIGHT_ORANGE)], path=path,
                 top_view=shown["map"],
                 Trial=f"{i + 1} of {n}", Scenario=f"{scenario}: {description}", Phase=labview.phase_label(phase),
                 Path=f"{length:.1f} m planned (A*), {math.hypot(start.x - check.x, start.y - check.y):.1f} m straight line",
                 **{"To the grasp pose": f"{100 * error[0]:+.0f} / {100 * error[1]:+.0f} cm, {math.degrees(error[2]):+.0f} deg"},
                 Command=f"vx {cmd[0]:+.2f} m/s · vy {cmd[1]:+.2f} m/s · ω {cmd[2]:+.2f} rad/s",
                 **{"Off the path": (f"now {100 * off_path[-1]:.1f} cm · mean {100 * np.mean(off_path):.1f} cm · max "
                                     f"{100 * max(off_path):.1f} cm") if off_path else "-"},
                 Arms=("down" if np.allclose(arms["now"], ARMS_DOWN) else ("up" if np.allclose(arms["now"], ARMS_UP) else "moving"))
                 + (f", swinging (left arm {arm_swing.swing_deg(offsets):+.0f}°)" if swing else ""),
                 Contacts=", ".join(sorted(touched)) or "none", Done=done_lines or ["-"])
        if reporter:
            reporter.on_tick(phase)
        return pose

    def enter(new_phase, t):
        nonlocal phase, stopped_at
        phase, stopped_at = new_phase, None
        phase_t.setdefault(new_phase, t)

    phase_t["walk"] = t0
    while outcome is None:
        pose = look(phase)
        if not sim.ok(f"trial {i + 1}"):
            fell, outcome = True, "fell"
            break
        t = sim.time
        if t >= next_t:
            next_t = max(next_t + 1.0 / CONTROL_HZ, t)
            if phase == "walk":
                off_path.append(float(np.min(np.linalg.norm(dense_path - (pose.x, pose.y), axis=1))))
                near = math.hypot(pose.x - check.x, pose.y - check.y) < SWITCH_TO_ALIGN
                if near or (follower_kind == "smooth" and follower.progress() > 0.98):
                    enter("align", t)
                    aligner.cmd = np.array(follower.cmd)  # continue from the current command, no jump
                else:
                    cmd = follower.step(pose, t) if follower_kind == "smooth" else follower.step(pose)
                    walk_log.append([round(t - t0, 3), round(pose.x, 4), round(pose.y, 4), *(round(c, 3) for c in cmd)])
            if phase == "align":
                cmd = aligner.step(pose, t)
                if aligner.done:
                    stopped_at = t if stopped_at is None else stopped_at
                    if t - stopped_at >= MEASURE_S:
                        error = approach.checkpoint_error(pose, setup.table)
                        if aligner.needs_redock(error):
                            aligner.redock()
                            stopped_at = None
                        else:
                            align_error = error
                            enter("raise_arms", t)
                            raise_from = arms["now"].copy()
            if phase == "raise_arms":  # standing still: arms up in ARMS_RAMP_S
                cmd = (0.0, 0.0, 0.0)
                share = min(1.0, (t - phase_t["raise_arms"]) / ARMS_RAMP_S)
                arms["now"] = raise_from + (0.5 - 0.5 * math.cos(math.pi * share)) * (ARMS_UP - raise_from)
                if share >= 1.0:
                    arms["now"] = ARMS_UP.copy()
                    enter("walk_in", t)
            elif phase in ("walk_in", "stand_still"):
                cmd = straight.step(pose, t)
                if straight.done:
                    phase = "stand_still"
                    stopped_at = t if stopped_at is None else stopped_at
                    if t - stopped_at >= MEASURE_S:  # stood long enough: measure, go again if the distance is off
                        if straight.needs_redo(approach.table_frame_error(pose, setup.table)):
                            straight.redo()
                            phase, stopped_at = "walk_in", None
                        else:
                            outcome = "docked"
            send_arms = arms["now"]
            if swing is not None and phase in ("walk", "align"):  # arms down: swing them with the steps
                dt = 1.0 / CONTROL_HZ if last_t is None else min(0.2, t - last_t)
                offsets = swing.step(sim.foot_gap(), dt, walking=float(np.hypot(cmd[0], cmd[1])) > 0.05)
                send_arms = swing.apply(arms["now"], offsets)
                swing_log.append(arm_swing.swing_deg(offsets))
                arm_leg_hits += sim.arm_leg_contacts()
            last_t = t
            sim.send(*cmd, arms=send_arms)
            if phase != "walk":
                e = approach.table_frame_error(pose, setup.table)
                dock_log.append([round(t - t0, 2), phase, round(e[0], 4), round(e[1], 4), round(math.degrees(e[2]), 2),
                                 *(round(c, 3) for c in cmd)])
            trail.append((round(pose.x, 4), round(pose.y, 4)))
            del trail[:-TRAIL_POINTS]
            if t - t0 > limit:
                outcome = "timeout"
        time.sleep(0.002)
    cmd = (0.0, 0.0, 0.0)
    sim.send(*cmd, arms=arms["now"])
    end = sim.pose()
    error = approach.table_frame_error(end, setup.table)
    checks = {"no_fall": not fell, "no_furniture_contact": not touched, "arrived_in_time": outcome == "docked",
              "dock_within_tolerance": outcome == "docked" and approach.accepted(error)}
    passed = all(checks.values())
    first_failed = next((k for k in CHECKS if not checks[k]), None)
    result = {"trial": i, "scenario": scenario, "setup": setup.as_dict(), "passed": passed, "checks": checks,
              "first_failed": first_failed, "outcome": outcome,
              "start": [round(start.x, 4), round(start.y, 4), round(math.degrees(start.yaw), 2)],
              "start_in_table_frame": [round(start_error[0], 3), round(start_error[1], 3), round(math.degrees(start_error[2]), 1)],
              "error_m_m_deg": [round(error[0], 4), round(error[1], 4), round(math.degrees(error[2]), 2)],
              "dock_dist_m": approach.DOCK_DIST,  # error is against this stop; the grasp scene's pelvis is at TABLE_CENTRE_X
              "walking": walking_metrics(walk_log, path, follower_kind),
              "align": None if align_error is None else {
                  "attempts": aligner.attempts, "s": round(phase_t.get("raise_arms", sim.time) - phase_t["align"], 1),
                  "error_at_checkpoint_cm_cm_deg": [round(100 * align_error[0], 1), round(100 * align_error[1], 1),
                                                    round(math.degrees(align_error[2]), 1)]},
              "approach": approach_metrics(dock_log, phase_t, straight, outcome),
              "arm_swing": None if swing is None else {"max_deg": round(float(np.max(np.abs(swing_log))), 1) if swing_log else 0.0,
                                                         "arm_leg_contact_ticks": arm_leg_hits},
              "path_m": round(length, 2), "sim_s": round(sim.time - t0, 1), "time_limit_s": round(limit, 1),
              "touched": sorted(touched), "contacts": contact_log, "dock_log": dock_log, "trail": trail, "fault": sim.fault}
    reporter and reporter.trial_end(passed, first_failed, {"error": sim.fault})
    return result


def walking_metrics(walk_log, path, follower_kind):
    """Path deviation, calmness and speed while following the path."""
    if len(walk_log) <= 2:
        return {}
    w = np.array(walk_log)
    deviation = path_deviation(w[:, 1:3], path)
    duration = float(w[-1, 0] - w[0, 0])
    return {"follower": follower_kind, "path_deviation_cm": {"mean": round(100 * float(deviation.mean()), 1),
                                                             "p90": round(100 * float(np.percentile(deviation, 90)), 1),
                                                             "max": round(100 * float(deviation.max()), 1)},
            # how calm the commands are: mean change of the turn rate per second (rad/s^2)
            "omega_change_per_s": round(float(np.abs(np.diff(w[:, 5])).sum() / max(duration, 1e-6)), 3),
            "walk_s": round(duration, 1),
            "mean_speed_m_s": round(float(np.linalg.norm(np.diff(w[:, 1:3], axis=0), axis=1).sum() / max(duration, 1e-6)), 3)}


def approach_metrics(dock_log, phase_t, straight, outcome):
    """The walk into the table: how long, how far off the line and the table axis, how much of it went sideways.
    sideways share = sideways / total pelvis travel in the table frame, on the trajectory averaged over ~2 steps (unaveraged,
    the step-by-step sway of the pelvis made it ~0.44 for every kind of approach)."""
    rows = [r for r in dock_log if r[1] in ("walk_in", "stand_still")]
    if len(rows) < 3:
        return None
    a = np.array([r[2:5] for r in rows], dtype=float)  # dx, dy, dyaw deg
    k = min(12, len(a))  # ~0.6 s at 20 Hz: average out the sway of each step, keep the real drift
    smooth = np.column_stack([np.convolve(a[:, j], np.ones(k) / k, mode="valid") for j in (0, 1)])
    step = np.abs(np.diff(smooth, axis=0))
    return {"s": round(rows[-1][0] - rows[0][0], 1), "attempts": straight.attempts, "outcome": outcome,
            "max_off_line_cm": round(100 * float(np.abs(a[:, 1]).max()), 1),
            "max_heading_off_deg": round(float(np.abs(a[:, 2]).max()), 1),
            "sideways_share": round(float(step[:, 1].sum() / max(step.sum(), 1e-9)), 3),
            "max_commanded_vy": round(float(max(abs(r[6]) for r in rows)), 3)}


def move_arms(sim, arms, goal, label):
    """Standing still: arms from arms["now"] to goal within ARMS_RAMP_S (smooth cosine). False if the robot falls."""
    begin, t0 = arms["now"].copy(), sim.time
    while sim.time < t0 + ARMS_RAMP_S:
        if not sim.ok(label):
            return False
        share = 0.5 - 0.5 * math.cos(math.pi * min(1.0, (sim.time - t0) / ARMS_RAMP_S))
        arms["now"] = begin + share * (goal - begin)
        sim.send(0.0, 0.0, 0.0, arms=arms["now"])
        sim.show(Phase=label)
        time.sleep(1.0 / CONTROL_HZ)
    arms["now"] = np.array(goal, dtype=np.float32)
    sim.send(0.0, 0.0, 0.0, arms=arms["now"])
    return sim.wait(0.5, label)


def recover(sim, furniture, arms):
    """After a fall: furniture away, robot back to the start pose on the elastic band, let go again. False if that fails."""
    furniture.park()
    sim.send(0.0, 0.0, 0.0)
    sim.sim.reset()
    sim.sim.elastic_band.enable = True
    sim.fault = None
    sim.watch.last_change = time.perf_counter()
    arms["now"] = np.zeros(action_space.DIM, dtype=np.float32)  # LeRobot's zero pose after the reset
    return sim.release_band() and move_arms(sim, arms, ARMS_DOWN, "Arms down for walking")


def main():
    ap = argparse.ArgumentParser(description="Level 4 step: walk up to the table from random start situations")
    ap.add_argument("--trials", type=int, default=12)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--scenarios", default=",".join(approach.SCENARIOS),
                    help=f"comma-separated, cycled in order: {','.join(approach.SCENARIOS)}")
    ap.add_argument("--follower", choices=("smooth", "grid"), default="smooth",
                    help="smooth: taut + rounded path, speed profile, pure pursuit; grid: raw A* path + L2 follower")
    ap.add_argument("--walk-arms", choices=("down", "up"), default="down",
                    help="arm pose while walking: down (natural, raised at the checkpoint) or up (BALL_READY_POSE all the way, comparison)")
    ap.add_argument("--no-swing", action="store_true", help="arms down but held still (no arm swing), for comparison")
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
    sim = WalkSim(scene=approach_world.write(), live=not args.no_live, video=args.video, title="Walk up to the table")
    furniture = approach_world.Furniture(sim.model, sim.data, getattr(sim.sim, "_step_lock", None))
    furniture.park()
    trial_t0 = [sim.time]
    reporter = None if args.no_labview else labview.Reporter(
        script="68", task="walk_to_table", object="table", trials=args.trials, argv=sys.argv[1:],
        frame_getter=(lambda: sim.live.frame) if sim.live else None, sim_time_getter=lambda: sim.time - trial_t0[0])
    results, done_lines = [], []
    arms = {"now": np.zeros(action_space.DIM, dtype=np.float32)}  # LeRobot's zero pose after standing up
    report = {"seed": args.seed, "scenarios": scenarios, "trials": args.trials, "follower": args.follower,
              "walk_arms": args.walk_arms, "arm_swing": args.walk_arms == "down" and not args.no_swing}
    wall0 = time.perf_counter()
    sim0 = sim.time
    try:
        ok = sim.release_band() and move_arms(sim, arms, ARMS_DOWN if args.walk_arms == "down" else ARMS_UP,
                                              "Arms to the walking pose")
        for i in range(args.trials if ok else 0):
            trial_t0[0] = sim.time
            scenario = scenarios[i % len(scenarios)]
            try:
                result = run_trial(sim, furniture, reporter, rng, i, args.trials, scenario, done_lines, args.follower, arms,
                                   ARMS_DOWN if args.walk_arms == "down" else ARMS_UP, not args.no_swing)
            except RuntimeError as error:  # e.g. no valid setup: report it, go on
                result = {"trial": i, "scenario": scenario, "passed": False, "first_failed": "arrived_in_time",
                          "error": f"{type(error).__name__}: {error}"}
                reporter and reporter.trial_end(False, "arrived_in_time", {"error": result["error"]})
            results.append(result)
            e = result.get("error_m_m_deg")
            line = (f"{i + 1} {scenario}: {'ok' if result['passed'] else 'FAIL ' + str(result['first_failed'])}"
                    + (f" ({100 * e[0]:+.1f} / {100 * e[1]:+.1f} cm, {e[2]:+.1f} deg, {result['path_m']} m)" if e else ""))
            done_lines.append(line)
            print(f"Trial {line}" + (f" - {result['error']}" if "error" in result else ""), flush=True)
            if result.get("checks", {}).get("no_fall") is False and not recover(sim, furniture, arms):
                print("could not stand up again after the fall - stopping")
                break
        report["realtime_factor"] = round((sim.time - sim0) / (time.perf_counter() - wall0), 3)
    finally:
        sim.close()
    passed = sum(r["passed"] for r in results)
    errors = np.array([r["error_m_m_deg"] for r in results if r.get("checks", {}).get("arrived_in_time")])
    report.update({
        "passed_trials": passed, "success_rate": passed / args.trials if args.trials else None,
        "failures_by_first_failed_check": {k: sum(r["first_failed"] == k for r in results) for k in CHECKS
                                           if any(r["first_failed"] == k for r in results)},
        "per_scenario": {s: f"{sum(r['passed'] for r in results if r['scenario'] == s)}/"
                            f"{sum(r['scenario'] == s for r in results)}" for s in scenarios},
        "walking": _walking_summary(results),
        "checkpoint_and_approach": _approach_summary(results),
        "arrival_error": None if not len(errors) else {
            "mean_abs_cm": [round(100 * float(v), 1) for v in np.abs(errors[:, :2]).mean(axis=0)],
            "max_abs_cm": [round(100 * float(v), 1) for v in np.abs(errors[:, :2]).max(axis=0)],
            "mean_abs_deg": round(float(np.abs(errors[:, 2]).mean()), 1), "max_abs_deg": round(float(np.abs(errors[:, 2]).max()), 1)},
        "criteria": {"accept_xy_m": approach.ACCEPT_XY, "accept_yaw_deg": math.degrees(approach.ACCEPT_YAW),
                     "checkpoint_back_m": approach.CHECKPOINT_BACK, "lead_in_m": approach.LEAD_IN,
                     "arms_down": list(ARM_DOWN), "align": _params(approach.ALIGN), "straight_in": _params(approach.STRAIGHT)},
        "results": results, "passed": passed == args.trials,
    })
    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=1, ensure_ascii=False))
    if reporter:
        reporter.finish(summary={k: report[k] for k in ("per_scenario", "arrival_error")}, report_path=out)
    plot(results)
    print(json.dumps({k: report[k] for k in ("passed_trials", "trials", "per_scenario", "failures_by_first_failed_check",
                                             "walking", "checkpoint_and_approach", "arrival_error", "realtime_factor")
                      if k in report}, indent=1))
    print(f"saved: {out}, {PLOT.relative_to(REPO)}" + (f", {args.video}" if args.video else ""))
    print("ACCEPTANCE PASSED" if report["passed"] else "ACCEPTANCE FAILED")
    return 0 if report["passed"] else 1


def _params(params):
    """Dataclass of controller settings as a dict, angles in degrees."""
    return {k: round(math.degrees(v), 2) if "yaw" in k or "heading" in k else v for k, v in dataclasses.asdict(params).items()}


def _approach_summary(results):
    """Checkpoint error after aligning and the straight walk into the table, over all trials."""
    al = [r["align"] for r in results if r.get("align")]
    ap = [r["approach"] for r in results if r.get("approach")]
    if not al or not ap:
        return None
    e = np.abs(np.array([a["error_at_checkpoint_cm_cm_deg"] for a in al]))
    return {"checkpoint_error_max_cm_cm_deg": [round(float(v), 1) for v in e.max(axis=0)],
            "align_s_mean": round(float(np.mean([a["s"] for a in al])), 1),
            "approach_s_mean": round(float(np.mean([a["s"] for a in ap])), 1),
            "approach_max_off_line_cm": max(a["max_off_line_cm"] for a in ap),
            "approach_max_heading_off_deg": max(a["max_heading_off_deg"] for a in ap),
            "approach_sideways_share_mean": round(float(np.mean([a["sideways_share"] for a in ap])), 3),
            "approach_max_commanded_vy": max(a["max_commanded_vy"] for a in ap)}


def _walking_summary(results):
    """Path deviation, calmness and speed over all trials (the per-trial "walking" blocks)."""
    w = [r["walking"] for r in results if r.get("walking")]
    if not w:
        return None
    mean = lambda key, sub=None: round(float(np.mean([x[key][sub] if sub else x[key] for x in w])), 3)  # noqa: E731
    return {"path_deviation_cm": {"mean": mean("path_deviation_cm", "mean"), "p90": mean("path_deviation_cm", "p90"),
                                  "max": max(x["path_deviation_cm"]["max"] for x in w)},
            "omega_change_per_s": mean("omega_change_per_s"), "mean_speed_m_s": mean("mean_speed_m_s"),
            "walk_s": mean("walk_s")}


def plot(results):
    """All trials in the table frame (dock at 0, table to +x): walked paths from their starts, and the arrival errors."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax, ax_err) = plt.subplots(1, 2, figsize=(14, 6.5), gridspec_kw={"width_ratios": [1.6, 1]})
    hx, hy = approach.TABLE_HALF
    cx = approach.DOCK_DIST  # plot in the dock frame
    ax.add_patch(plt.Rectangle((cx - hx, -hy), 2 * hx, 2 * hy, color="#8c6b4d", label="table"))
    colors = plt.cm.tab10(np.linspace(0, 1, 10))
    names = list(approach.SCENARIOS)
    for r in results:
        if "trail" not in r:
            continue
        t = r["setup"]["table"]
        table = approach.Pose2D(t[0], t[1], math.radians(t[2]))
        dock = approach.dock_pose(table)
        c, s = math.cos(table.yaw), math.sin(table.yaw)
        pts = np.array(r["trail"]) - (dock.x, dock.y)
        local = np.column_stack([c * pts[:, 0] + s * pts[:, 1], -s * pts[:, 0] + c * pts[:, 1]])
        color = colors[names.index(r["scenario"]) % 10]
        ax.plot(local[:, 0], local[:, 1], color=color, lw=1.2, alpha=0.85)
        ax.plot(local[0, 0], local[0, 1], "o", color=color, ms=5)
        for o in r["setup"]["obstacles"]:
            ox, oy = o["xy"][0] - dock.x, o["xy"][1] - dock.y
            oyaw = math.radians(o["yaw_deg"]) - table.yaw
            corner = approach.corners(c * ox + s * oy, -s * ox + c * oy, oyaw, approach.OBSTACLES[o["name"]][0])
            ax.add_patch(plt.Polygon(corner, color="#7a8794", alpha=0.35))
    for k, name in enumerate(names):
        if any(r["scenario"] == name for r in results):
            ax.plot([], [], color=colors[k % 10], label=name)
    ax.plot(0, 0, "x", color="#e76500", ms=10, mew=2, label="grasp pose (dock)")
    ax.set_aspect("equal")
    ax.set_xlabel("x towards the table (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("All trials in the table frame: starts (dots) and walked paths; obstacles grey")
    ax.legend(loc="lower left", fontsize=8)
    lim = approach.ACCEPT_XY * 100
    ax_err.add_patch(plt.Rectangle((-lim, -lim), 2 * lim, 2 * lim, fill=False, ls="--", color="#e76500",
                                   label="accepted (+-10 cm)"))
    for r in results:
        if "error_m_m_deg" in r:
            e = r["error_m_m_deg"]
            ax_err.plot(100 * e[1], 100 * e[0], "o" if r["passed"] else "x", color=colors[names.index(r["scenario"]) % 10])
            ax_err.annotate(f"{e[2]:+.0f}°", (100 * e[1], 100 * e[0]), textcoords="offset points", xytext=(4, 4), fontsize=7)
    ax_err.set_xlim(-1.5 * lim, 1.5 * lim)
    ax_err.set_ylim(-1.5 * lim, 1.5 * lim)
    ax_err.set_aspect("equal")
    ax_err.set_xlabel("sideways error dy (cm, + = left)")
    ax_err.set_ylabel("forward error dx (cm, + = closer to the table)")
    ax_err.set_title("Arrival error after 1 s standing (label: heading error)")
    ax_err.legend(loc="lower left", fontsize=8)
    PLOT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(PLOT, dpi=110, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    # CycloneDDS and sim threads keep the interpreter alive (see 50) - always exit hard
    try:
        code = main()
    except BaseException:
        traceback.print_exc()
        code = 2
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
