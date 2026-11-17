"""Level 1 - two-hand volleyball pick-and-place without AI: acceptance and evaluation of the sim teacher.

Runs g1sim/ball_grasp.py (TwoHandBallGrasp) several times on the volleyball scene and checks per trial, in this order
(cause -> effect; the first failed check is the failure reason):
  1. no_forbidden_contacts   no hand or arm contact with the table, the stand or the basket during the whole run
  2. ball_lifted             ball centre >= 6 cm above its rest point after the lift
  3. ball_carried_to_basket  still >= 6 cm up at the end of the waist turn
  4. ball_in_basket          after the release, for 3 s: ball centre inside the basket (inner box minus the radius)
  5. ball_on_basket_floor    ... ball bottom within 5 mm of the basket floor
  6. ball_at_rest            ... slower than 1 cm/s and 0.1 rad/s
  7. hands_off_ball          ... no robot contact with the ball
--jitter J moves ball and stand together by an offset drawn uniformly from +-J m in x and y (--seed); 0 = nominal start.
--distractors puts 1-3 distractors on the table per trial (configs/randomization.json, group "distractors"); touching one
counts as a forbidden contact.
--randomize draws a full variation per trial (TableEnv.reset(randomize=True), configs/randomization.json with the volleyball
ranges of its "scenes" block): ball + stand offset, robot stand pose, physics, light/colours/camera/noise and 1-3 distractors.
--vary GROUPS limits that to some groups (comma-separated, e.g. --vary stand,physics). Replaces --jitter and --distractors.
--scene volleyball_walk runs the teacher in the walk world (free-standing robot in GR00T's crouch, pelvis welded, Level 1
Stage 3): the robot stands at the stop pose of the layout (navigation.table_stop_pose); --randomize draws layout, arrival error,
physics and image (randomization.sample_walk). Each result has a "walk" block: stop pose, stance, arrival error, FK clearance,
distance and bearing of ball and basket. --map (walk world only) runs a fixed grid instead of random trials: stances around the
nominal stop (back, sideways, turned) and layouts with other ball-basket separations; it sets the ranges in the config group "walk".
Environment ~/dev/g1-ext:
    acceptance, nominal start, every trial must pass:
        ~/dev/g1-ext/.venv/bin/python scripts/67_volleyball_pick_place.py --trials 10
    evaluation with start offsets, the result is the success rate:
        ~/dev/g1-ext/.venv/bin/python scripts/67_volleyball_pick_place.py --trials 50 --jitter 0.02 --seed 0
    ... with distractors on the table:
        ~/dev/g1-ext/.venv/bin/python scripts/67_volleyball_pick_place.py --trials 50 --jitter 0.02 --distractors --seed 0
    ... with the full randomization (what the demo factory should record):
        ~/dev/g1-ext/.venv/bin/python scripts/67_volleyball_pick_place.py --trials 50 --randomize --seed 0
    walk world: reach map, then the random sweep (Level 1 Stage 3):
        ~/dev/g1-ext/.venv/bin/python scripts/67_volleyball_pick_place.py --scene volleyball_walk --map --no-labview --out runs/l1w_s3_map.json
        ~/dev/g1-ext/.venv/bin/python scripts/67_volleyball_pick_place.py --scene volleyball_walk --randomize --trials 200 --seed 0 --no-labview --out runs/l1w_s3_sweep.json
    walk world: how close the hands pass table, stand and basket (g1sim/hand_clearance.py, FINGER-PLAN.md gate G7):
        ~/dev/g1-ext/.venv/bin/python scripts/67_volleyball_pick_place.py --scene volleyball_walk --randomize --trials 50 --seed 10 --clearance --no-labview --out runs/l1h_finger_walk_s10.json
Output, named after --out: runs/<name>.json; docs/volleyball_teacher_squeeze.png (trial 0 at the end of the squeeze,
front and side view; walk world: runs/<name>_squeeze.png); with --video docs/videos/<name>.mp4 (trial 0); with --watch (and no --video) a replay video of trial 0
in runs/labview/videos/<run_id>.mp4 for the lab view's play button. One renderer for everything: the head-camera
renderer of TableEnv (640 x 400) also draws the lab view image, the squeeze image and the video.
"""
import argparse
import collections
import json
import pathlib
import sys
import warnings

import cv2
import mink
import mujoco
import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim import ball_grasp, hand_clearance, labview, randomization, table_env, table_scene  # noqa: E402
from g1sim.video import VideoRecorder  # noqa: E402

SCENES = ("volleyball", "volleyball_walk")
BALL = ball_grasp.BALL
OUT = REPO / "runs" / "level1_volleyball_pick_place.json"
SQUEEZE_IMAGE = REPO / "docs" / "volleyball_teacher_squeeze.png"
HEAD_SIZE = (400, 640)  # (height, width) of the one renderer
# Squeeze image: taken when the fingers have closed (last squeeze tick); front of the turned torso and from its side
SQUEEZE_TICK = ball_grasp.ticks(ball_grasp.SQUEEZE_S) + ball_grasp.ticks(ball_grasp.SQUEEZE_HOLD_S)
VIEWS = {"front": 215.0, "side": 125.0}  # azimuth (deg) of a free camera, lookat/distance/elevation below
VIEW_LOOKAT, VIEW_DISTANCE, VIEW_ELEVATION = [0.20, 0.12, 0.95], 1.25, -15.0

warnings.filterwarnings("ignore", message=".*parent has")
# numpy 2 with Apple Accelerate reports false overflow warnings in matrix products; the results are correct
warnings.filterwarnings("ignore", message=".*encountered in matmul")


def map_items():
    """Reach map of the walk world: (label, ball shift, basket shift, arrival error). 45 stances around the nominal stop
    (back 0-8 cm, sideways +-4 cm, turned +-6 deg) and 20 layouts: separation 0.40-0.60 m, the ball-basket line turned
    +-10 deg or moved 6 cm back from the table edge."""
    items = []
    for back in (0.0, 0.02, 0.04, 0.06, 0.08):
        for side in (-0.04, 0.0, 0.04):
            for yaw in (-6.0, 0.0, 6.0):
                items.append((f"stance back {back:.2f} side {side:+.2f} yaw {yaw:+.0f}", (0.0, 0.0), (0.0, 0.0),
                              {"back_m": back, "side_m": side, "yaw_deg": yaw}))
    mid = np.array(table_scene.VOLLEYBALL_OBJECTS[BALL][4]) * [1.0, 0.0]  # middle between ball and basket
    for separation in (0.40, 0.44, 0.50, 0.56, 0.60):
        for turn, back in ((0.0, 0.0), (10.0, 0.0), (-10.0, 0.0), (0.0, 0.06)):
            t = np.radians(turn)
            half = 0.5 * separation * np.array([-np.sin(t), np.cos(t)])  # middle -> ball
            centre = mid + [back, 0.0]
            items.append((f"layout separation {separation:.2f} turn {turn:+.0f} back {back:.2f}",
                          tuple(centre + half - table_scene.VOLLEYBALL_OBJECTS[BALL][4]),
                          tuple(centre - half - table_scene.BASKET_CENTER), None))
    return items


def walk_geometry(env, variation):
    """Walk world: stop pose, stance, arrival error, FK clearance of the stance, and distance (m) and bearing (deg, + = left of
    the stance heading) of ball stand and basket from the pelvis."""
    stance = variation.get("stance", table_scene.WALK_STANCE)
    out = {"stop": variation.get("stop"), "stance": stance, "arrival": variation.get("arrival"),
           "stance_clearance_m": round(env.stance_clearance(stance), 4)}
    pelvis = np.array([stance["x_m"], stance["y_m"]])
    for name in ("volleyball_stand", "basket"):
        rel = env.data.xpos[env.model.body(name).id][:2] - pelvis
        out[f"{name}_r_m"] = round(float(np.linalg.norm(rel)), 4)
        out[f"{name}_bearing_deg"] = round(float((np.degrees(np.arctan2(rel[1], rel[0])) - stance["yaw_deg"] + 180) % 360 - 180), 2)
    return out


def free_camera(azimuth):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = VIEW_LOOKAT
    cam.distance, cam.azimuth, cam.elevation = VIEW_DISTANCE, azimuth, VIEW_ELEVATION
    return cam


def squeeze_image(renderer, data):
    tiles = []
    for name, azimuth in VIEWS.items():
        renderer.update_scene(data, camera=free_camera(azimuth))
        tile = renderer.render().copy()
        cv2.putText(tile, name, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        tiles.append(tile)
    return np.concatenate(tiles, 1)


def main():
    parser = argparse.ArgumentParser(description="Level 1 - two-hand volleyball pick-and-place without AI")
    parser.add_argument("--scene", choices=SCENES, default="volleyball",
                        help="volleyball = fixed robot (default); volleyball_walk = free-standing robot in the walk world")
    parser.add_argument("--map", action="store_true", help="walk world: run the reach-map grid instead of --trials")
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--jitter", type=float, default=0.0, metavar="METERS",
                        help="ball + stand start offset, uniform +-METERS in x and y (0 = nominal start)")
    parser.add_argument("--distractors", action="store_true", help="1-3 distractors on the table per trial")
    parser.add_argument("--randomize", action="store_true", help="full random variation per trial (all enabled groups)")
    parser.add_argument("--vary", help=f"with --randomize: only these groups, comma-separated: {','.join(randomization.GROUPS)}")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", type=pathlib.Path, default=OUT)
    parser.add_argument("--video", action="store_true", help="save the first trial as a video in docs/videos/")
    parser.add_argument("--watch", type=float, metavar="SPEED",
                        help="lab view: slow down to SPEED x real time (1 = real time) so the phases can be followed")
    parser.add_argument("--no-labview", action="store_true", help="write no lab view files (runs/labview/)")
    parser.add_argument("--clearance", action="store_true", help="measure the smallest distance of every arm part to table, "
                        "stand, basket; ~0.5 s more per trial")
    args = parser.parse_args()
    if args.randomize and (args.jitter > 0 or args.distractors):
        parser.error("--randomize already varies the ball and the distractors: leave out --jitter and --distractors")
    if args.vary and not args.randomize:
        parser.error("--vary needs --randomize")
    walk = args.scene == "volleyball_walk"
    if args.map and (not walk or args.randomize or args.jitter > 0):
        parser.error("--map needs --scene volleyball_walk, without --randomize and --jitter")
    if walk and args.distractors:
        parser.error("the walk world has no distractors yet")
    scene = args.scene
    items = map_items() if args.map else None
    if items:
        args.trials = len(items)
    squeeze_path = REPO / "runs" / f"{args.out.stem}_squeeze.png" if walk else SQUEEZE_IMAGE
    rng = np.random.default_rng(args.seed)
    config = randomization.load(groups=args.vary.split(",") if args.vary else None)
    distractor_config = config["distractors"]

    table_scene.write(scene=scene)
    env = table_env.TableEnv(scene=scene, image_sizes={"head_camera": HEAD_SIZE}, seed=args.seed, variation_config=config)
    env.reset()  # renders the head camera once: from here on env owns exactly one renderer
    renderer = env._renderers[HEAD_SIZE]
    reporter = None if args.no_labview else labview.Reporter(
        script="67", task="pick_place", object=scene, trials=args.trials, argv=sys.argv[1:],
        renderer_getter=lambda: renderer, sim_time_getter=lambda: env.data.time, watch=args.watch, data=env.data,
        track_body=BALL)
    if args.video:
        video_path = REPO / "docs" / "videos" / f"{args.out.stem}.mp4"
    elif args.watch and reporter:  # a watched run keeps a replay for the lab view
        video_path = labview.VIDEO_DIR / f"{reporter.run_id}.mp4"
    else:
        video_path = None
    results = []
    for trial in range(args.trials):
        if args.randomize:
            env.reset(randomize=True)
            variation = env.variation
            offset = np.array([variation["objects"][BALL]["dx_m"], variation["objects"][BALL]["dy_m"]])
        else:
            variation = randomization.nominal(env.objects)
            offset = rng.uniform(-args.jitter, args.jitter, 2) if args.jitter > 0 else np.zeros(2)
            variation["objects"][BALL]["dx_m"], variation["objects"][BALL]["dy_m"] = (float(v) for v in offset)
            if args.distractors:
                variation["distractors"] = randomization.sample_distractors(distractor_config, rng, scene)
            if walk:  # the robot stands at the stop pose of this layout (map: plus the map's shifts and arrival error)
                ball_shift, basket_shift, arrival = (items[trial][1:] if items else (offset, (0.0, 0.0), None))
                variation["objects"][BALL]["dx_m"], variation["objects"][BALL]["dy_m"] = (float(v) for v in ball_shift)
                offset = np.asarray(ball_shift, float)
                layout = randomization.walk_layout(ball_shift, basket_shift, arrival)
                variation.update({key: layout[key] for key in ("basket", "stop", "arrival", "stance")})
            env.reset(variation=variation)  # the stand moves with the ball (SCENES["volleyball"]["holders"])
        stand = variation["stand"]
        if reporter:
            reporter.trial_start(trial, {"offset_m": offset.round(4).tolist(), "distractors": sorted(variation["distractors"]),
                                         "stand": {k: round(val, 3) for k, val in stand.items()}})
        video = (VideoRecorder(env.model, env.data, video_path, track_body=BALL, **labview.CAMERA, renderer=renderer)
                 if video_path and trial == 0 else None)
        phase_ticks = collections.Counter()
        meter = hand_clearance.ClearanceMeter(env) if args.clearance else None

        def on_tick(phase, video=video, first=trial == 0):
            if video:
                video.grab()
            if meter:
                meter.update(phase)
            phase_ticks[phase] += 1
            if first and phase == "squeeze" and phase_ticks[phase] == SQUEEZE_TICK:
                squeeze_path.parent.mkdir(exist_ok=True)
                cv2.imwrite(str(squeeze_path), squeeze_image(renderer, env.data)[:, :, ::-1])
            if reporter:
                reporter.on_tick(phase)

        try:
            result = ball_grasp.TwoHandBallGrasp(env, on_tick=on_tick).run()
        except (RuntimeError, mink.exceptions.NoSolutionFound) as error:  # one trial must not end the evaluation
            result = {"error": f"{type(error).__name__}: {error}", "checks": {"run_without_error": False},
                      "passed": False, "first_failed": "run_without_error"}
        finally:
            if video:
                video.close()
        result["ball_offset_m"] = offset.round(4).tolist()
        if meter:
            result["clearance"] = meter.result()
        if walk:
            result["walk"] = walk_geometry(env, variation)
            if items:
                result["map"] = items[trial][0]
        if args.randomize or walk:
            result["variation"] = variation  # enough to replay this start: env.reset(variation=...)
        result["distractors"] = variation["distractors"]
        result["distractors_moved_m"] = {name: round(float(np.linalg.norm(env.object_position(name)[:2] - [p["x_m"], p["y_m"]])), 4)
                                         for name, p in variation["distractors"].items()}
        results.append(result)
        if reporter:
            reporter.trial_end(result["passed"], result["first_failed"], {"error": result.get("error")})
        summary = f"offset {offset[0] * 100:+.1f}/{offset[1] * 100:+.1f} cm, {len(variation['distractors'])} distractors | "
        if walk:
            g = result["walk"]
            summary = ((f"{items[trial][0]} | " if items else "")
                       + (f"arrival back {g['arrival']['back_m'] * 100:.1f} side {g['arrival']['side_m'] * 100:+.1f} cm "
                          f"{g['arrival']['yaw_deg']:+.1f} deg, " if g["arrival"] else "")
                       + f"clearance {g['stance_clearance_m'] * 100:.1f} cm, ball r {g['volleyball_stand_r_m']:.3f} m "
                       f"{g['volleyball_stand_bearing_deg']:+.0f} deg, basket r {g['basket_r_m']:.3f} m "
                       f"{g['basket_bearing_deg']:+.0f} deg | ")
        elif args.randomize:
            summary = (f"stand {stand['dx_m'] * 100:+.1f}/{stand['dy_m'] * 100:+.1f} cm {stand['yaw_deg']:+.1f} deg, "
                       f"friction x{variation['physics']['friction_scale']:.2f}, latency "
                       f"{variation['physics']['latency_s'] * 1000:.0f} ms, " + summary)
        if "error" in result:
            summary += "ERROR " + result["error"]
        else:
            fingers = " ".join(f"{side[0].upper()}:{','.join(parts)}"
                               for side, parts in result["ball_touched_by"]["squeeze"].items())
            summary += (f"lift {result['lift_height_m'] * 100:.1f} cm, over basket after turn "
                        f"{np.round(np.array(result['carried_ball_over_basket_m']) * 100, 1).tolist()} cm, "
                        f"end {result['ball_to_basket_centre_m'] * 100:.1f} cm from centre, "
                        f"touching the ball at squeeze {fingers}, forbidden contacts {result['forbidden_contacts'] or 'none'}")
        print(f"Trial {trial + 1}: {'ok  ' if result['passed'] else 'FAIL ' + result['first_failed']} {summary}",
              flush=True)
    env.close()

    passed = sum(r["passed"] for r in results)
    failures = collections.Counter(r["first_failed"] for r in results if not r["passed"])
    carried = [r for r in results if r.get("checks", {}).get("ball_carried_to_basket")]  # trials that carried the ball over
    over = [float(np.linalg.norm(r["carried_ball_over_basket_m"])) for r in carried]
    aligned = [float(np.linalg.norm(r["aligned_ball_over_basket_m"])) for r in carried]
    report = {
        "scene": scene,
        "map": args.map,
        "carried_ball_over_basket_p95_m": round(float(np.percentile(over, 95)), 4) if over else None,
        "aligned_ball_over_basket_p95_m": round(float(np.percentile(aligned, 95)), 4) if aligned else None,
        "trials": args.trials,
        "passed_trials": passed,
        "success_rate": passed / args.trials,
        "jitter_m": args.jitter,
        "distractors": args.distractors,
        "randomize": args.randomize,
        "varied_groups": randomization.enabled(config) if args.randomize else [],
        "seed": args.seed,
        "failures_by_first_failed_check": dict(failures),
        "criteria": {"min_lift_m": ball_grasp.MIN_LIFT, "check_s": ball_grasp.CHECK_S,
                     "in_basket_margin_m": ball_grasp.IN_BASKET_MARGIN, "floor_tolerance_m": ball_grasp.FLOOR_TOLERANCE,
                     "max_speed_m_s": ball_grasp.MAX_SPEED, "max_spin_rad_s": ball_grasp.MAX_SPIN,
                     "forbidden": list(ball_grasp.FORBIDDEN)},
        "fingers_curled": ball_grasp.FINGERS_CURLED.tolist(),
        "results": results,
        "passed": passed == args.trials,
    }
    if args.clearance:
        report["clearance_summary"] = hand_clearance.summarize(results)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False))
    if reporter:
        reporter.finish(report_path=args.out, video_path=video_path)
    print(f"{passed}/{args.trials} trials passed ({report['success_rate']:.0%})")
    if failures:
        print("Failures by first failed check:", dict(failures))
    video_note = f", {video_path.relative_to(REPO)}" if video_path else ""
    if over:
        print(f"ball over the basket after the turn: p95 {report['carried_ball_over_basket_p95_m'] * 100:.1f} cm from the centre, "
              f"after centring {report['aligned_ball_over_basket_p95_m'] * 100:.1f} cm")
    if args.clearance and report["clearance_summary"]["n"]:
        c = report["clearance_summary"]
        print(f"clearance (passed trials): any arm part p5 {c['p5_m'] * 100:.2f} cm, min {c['min_m'] * 100:.2f} cm; "
              f"pinky p5 {c['pinky_p5_m'] * 100:.2f} cm; closest most often: {c['closest_counts']}")
    print(f"saved: {args.out}, {squeeze_path.relative_to(REPO)}{video_note}")
    if args.jitter > 0 or args.distractors or args.randomize or args.map:
        return 0  # evaluation: the result is the rate, not pass/fail
    print("ACCEPTANCE PASSED" if report["passed"] else "ACCEPTANCE FAILED")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
