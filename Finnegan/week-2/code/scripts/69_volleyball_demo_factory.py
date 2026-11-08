"""Level 1 Step 5 (plan Stage 6) - volleyball demo factory: the sim teacher records episodes for the SmolVLA dataset.

The teacher is g1sim/ball_grasp.py (TwoHandBallGrasp, the natural two-hand cradle grasp, waist turn and centre phase; 199/200 in
the walk world, Level 1 Step 1). It runs in the walk world (scene "volleyball_walk") in NO-WALK mode: TableEnv.reset(randomize=True)
puts the robot at the layout's stop pose plus a random arrival error inside the ranges the reach map passed (config group "walk"),
pelvis welded - the "handover" moment at which a walk-up would hand over to the arms. Walk mode (Saad's walk-in first) is not
built yet; sidecars say so in "mode" and "stance_source" ("provisional" = not a measured walk-up arrival distribution).
Episode = from the handover to the end of face_front + 30 ticks (plan D, I). Per tick: head image (200 x 320) + 27-D state before
the action, and the 27-D action commanded (g1sim/episode.py). The settle and the 3 s placement check after that run but are not
recorded. Only successes are saved (67's checks) that also pass episode.validate, 600-800 frames and waist tracking; others are
logged with their first failed check and not counted.
Per demo: episode_s<seed>_t<trial>.npz (frames + handover snapshot, see episode.handover_arrays) and a sidecar .json (variation,
stance, arrival error, geometry, teacher result, handover roll/pitch, timestep, frames, wall time, waist tracking).
Per process: factory_s<seed>.json, rewritten after every trial. Seeds >= 100 (0 and 1 are the eval seeds). Several processes:
one per seed, same folder; a lock file refuses a second process with the same seed in the same folder (same seed = same demos).
Lab view (scripts/65_labview.py, http://127.0.0.1:8030 -> Live): one card per process with phase, demo count and a live image -
left the head-camera frame being recorded, right the robot from outside (the same renderer, one image every 2 s;
rendering does not touch the physics, the demos stay bit-exact). --no-labview turns it off.
Environment ~/dev/g1-ext:
    smoke (Step 5):
        ~/dev/g1-ext/.venv/bin/python scripts/69_volleyball_demo_factory.py --demos 5 --seed 100 --name smoke_v1
    dataset (Step 6), 4 processes in the background, seeds 101-104 (100 is the smoke seed: its demos would be copies):
        ~/dev/g1-ext/.venv/bin/python scripts/69_volleyball_demo_factory.py --demos 75 --seed 101 --name level1_walkup_v1
"""
import argparse
import collections
import hashlib
import importlib.metadata
import importlib.util
import json
import os
import pathlib
import platform
import sys
import time
import warnings

import mink
import mujoco
import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim import action_space, ball_grasp, episode, labview, policy_wire, randomization, table_env, table_scene  # noqa: E402

warnings.filterwarnings("ignore", message=".*parent has")
warnings.filterwarnings("ignore", message=".*encountered in matmul")  # false alarm numpy 2 + Apple Accelerate

# walk geometry from 67, not rebuilt
_spec = importlib.util.spec_from_file_location("pick_place", REPO / "scripts" / "67_volleyball_pick_place.py")
pick_place = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pick_place)

SCENE = "volleyball_walk"
TASKS = policy_wire.TASKS  # the demo's sentence: TASKS[(seed + trial) % len(TASKS)], see task_for; TASKS[0] is the eval prompt
OUT = pathlib.Path("/Volumes/T7/Robot Project SAP/DATA Collection/g1-demos")
TAIL_TICKS = 30  # recorded settle ticks after face_front
WAIST = action_space.SLICES["waist"].start
WAIST_TRACK_FROM = 10  # ticks; plan Stage 6: state[:, 26] follows actions[:, 26] within WAIST_TRACK_MAX after that
WAIST_TRACK_MAX = 0.05  # rad
FRAMES_RANGE = (600, 800)
LIVE = {"head": None, "next": 0.0}  # head-camera frame last recorded, for the lab view
LIVE_IMAGE_S = 2.0  # s between lab view images
STANCE_SOURCE = "provisional"  # plan D: "measured" only once the stance comes from a measured walk-up arrival distribution
STANCE_NOTE = "no-walk: stop pose + arrival error from the reach-map ranges (config group 'walk'), not a measured walk-up"
GIT = labview.git_state()
# packages whose version can change a demo bit for bit (requirements-record.txt pins them for a second recording machine)
VERSIONED = ("mujoco", "mink", "numpy")  # + OpenCV, read from cv2 itself (provenance)


def write_json(path, data):
    episode.write_verified(path, json.dumps(data, indent=2, ensure_ascii=False, default=float).encode())


def lock(out_dir, seed):
    """Refuse a second live process with this seed in this folder (it would write the same files). Returns the lock path."""
    path = out_dir / f".lock_s{seed}"
    try:
        other = int(path.read_text())
        os.kill(other, 0)
        raise SystemExit(f"process {other} already records seed {seed} into {out_dir} - pick another seed")
    except (FileNotFoundError, ValueError, ProcessLookupError):
        pass  # no lock, or its process is gone
    path.write_text(str(os.getpid()))
    return path


def task_for(seed, trial):
    """The sentence of demo (seed, trial): every phrasing equally often over consecutive trials, and the same demo always gets
    the same one (a re-recording gives identical files)."""
    return TASKS[(seed + trial) % len(TASKS)]


def provenance(config):
    """What a demo needs to be re-made exactly: sha256 of the effective randomization config (as loaded) and the versions of
    Python, of the VERSIONED packages and of OpenCV. OpenCV comes from cv2.__version__, not a distribution name: the wheel
    may be opencv-python or opencv-python-headless, and a missing name would crash after the npz is written (review m3)."""
    import cv2  # lazy, as in g1sim/labview.py

    digest = hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()
    versions = {"python": platform.python_version(), **{name: importlib.metadata.version(name) for name in VERSIONED},
                "opencv": cv2.__version__}
    return {"randomization_sha256": digest, "versions": versions}


def handover_pose(env):
    """Pelvis height (m), roll and pitch (deg) now."""
    pelvis = env.model.body("pelvis").id
    r = env.data.xmat[pelvis].reshape(3, 3)
    return {"pelvis_z_m": round(float(env.data.xpos[pelvis][2]), 4), "roll_deg": round(float(np.degrees(np.arctan2(r[2, 1], r[2, 2]))), 3),
            "pitch_deg": round(float(np.degrees(-np.arcsin(np.clip(r[2, 0], -1.0, 1.0)))), 3)}


def waist_tracking(ep):
    """Largest |measured waist - commanded waist of the tick before| (rad) from tick WAIST_TRACK_FROM on."""
    error = np.abs(ep["state"][WAIST_TRACK_FROM:, WAIST] - ep["actions"][WAIST_TRACK_FROM - 1:-1, WAIST])
    return float(error.max())


def live_frame(env, camera):
    """Lab view image: the last recorded head-camera frame next to an outside view from the head-camera renderer. At most one
    per LIVE_IMAGE_S: at the lab view's 10 per second, 4 parallel recorders ran >= 2.5x slower (2026-10-14)."""
    renderer = env._renderers.get(env.image_sizes["head_camera"])
    now = time.monotonic()
    if renderer is None or LIVE["head"] is None or now < LIVE["next"]:
        return None
    LIVE["next"] = now + LIVE_IMAGE_S
    renderer.update_scene(env.data, camera=camera)
    outside = renderer.render()
    # The first head-camera image after another camera differs by up to 7 grey levels (state left in the GL context; the
    # second one is exact again, render_probe 2026-10-14). One throw-away head render keeps the recorded frames bit-exact.
    renderer.update_scene(env.data, camera="head_camera")
    renderer.render()
    return labview.encode_jpeg(np.concatenate([LIVE["head"], outside], 1))


def outside_camera(env):
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    camera.trackbodyid = env.model.body(ball_grasp.BALL).id
    camera.distance, camera.azimuth, camera.elevation = 1.4, 215.0, -20.0
    return camera


def record(env, seed, trial, out_dir, reporter=None):
    """One teacher run from the current handover. Returns the log entry (dict)."""
    variation, task = env.variation, task_for(seed, trial)
    entry = {"trial": trial, "draws": env.draws}
    snapshot = env.snapshot_handover()  # before the first image: a restore then renders the same first frame
    handover = handover_pose(env)
    recorder = episode.EpisodeRecorder(task, table_env.CONTROL_HZ)
    pending = [env.observation()]  # observation before the next action
    LIVE["head"] = pending[0]["head_image"]
    tail = [0]

    def on_tick(phase):  # called after env.apply(): g.action is the action just executed
        if reporter:
            reporter.on_tick(phase)
        if phase == "settle":
            tail[0] += 1
            if tail[0] > TAIL_TICKS:
                return
        recorder.add(pending[0], g.action.copy())
        if tail[0] < TAIL_TICKS:
            pending[0] = env.observation()
            LIVE["head"] = pending[0]["head_image"]

    g = ball_grasp.TwoHandBallGrasp(env, on_tick=on_tick)
    start = time.perf_counter()
    try:
        result = g.run()
    except (RuntimeError, mink.exceptions.NoSolutionFound) as error:
        result = {"error": f"{type(error).__name__}: {error}", "passed": False, "first_failed": "run_without_error"}
    entry["wall_s"] = time.perf_counter() - start
    if not result["passed"]:
        return {**entry, "result": result["first_failed"], "error": result.get("error")}

    ep = recorder.episode(success=True)
    problems = episode.validate(ep)
    frames = len(recorder)
    if not FRAMES_RANGE[0] <= frames <= FRAMES_RANGE[1]:
        problems.append(f"{frames} frames, expected {FRAMES_RANGE[0]}-{FRAMES_RANGE[1]}")
    waist_error = waist_tracking(ep)
    if waist_error > WAIST_TRACK_MAX:
        problems.append(f"waist tracking {waist_error:.3f} rad > {WAIST_TRACK_MAX}")
    if problems:
        return {**entry, "result": "invalid", "error": "; ".join(problems)}
    path = out_dir / f"episode_s{seed}_t{trial:04d}.npz"
    recorder.save(path, success=True, extra=episode.handover_arrays(snapshot))
    problems = episode.validate(episode.load(path))  # read back from the disk
    if problems:
        raise OSError(f"{path} reads back invalid: {problems}")
    info = {"task": task, "success": True, "source": "scripts/69_volleyball_demo_factory.py", "scene": SCENE,
            "mode": "no-walk", "stance_source": STANCE_SOURCE, "stance_note": STANCE_NOTE, "git": GIT, "seed": seed, "trial": trial, "draws": env.draws,
            "frames": frames, "sim_s": frames / table_env.CONTROL_HZ, "wall_s": round(entry["wall_s"], 1),
            "timestep": float(env.model.opt.timestep), "image_shape": list(ep["head_image"].shape[1:]),
            "handover": handover,
            "walkup": None, "detection": None,  # no walk-up and no camera estimate in no-walk mode (the teacher uses sim truth)
            "walk": pick_place.walk_geometry(env, variation), "waist_tracking_max_rad": round(waist_error, 4),
            "teacher": result, "validate": problems, "variation": variation, **provenance(env.variation_config)}
    write_json(path.with_suffix(".json"), info)
    return {**entry, "result": "saved", "path": str(path), "frames": frames, "size_mb": path.stat().st_size / 1e6,
            "waist_tracking_max_rad": waist_error, "validate": problems}


def main():
    parser = argparse.ArgumentParser(description="Level 1 - volleyball demo factory (no-walk mode)")
    parser.add_argument("--demos", type=int, default=5, help="save this many successful episodes")
    parser.add_argument("--seed", type=int, default=100, help=">= 100 (0 and 1 are the eval seeds)")
    parser.add_argument("--max-trials", type=int, help="draw at most this many starts (default 2 per demo + 10)")
    parser.add_argument("--name", default="smoke_v1", help="folder under --out")
    parser.add_argument("--out", type=pathlib.Path, default=OUT)
    parser.add_argument("--no-labview", action="store_true", help="write no lab view files (runs/labview/)")
    parser.add_argument("--randomization", type=pathlib.Path, default=randomization.CONFIG,
                        help="randomization config (default configs/randomization.json)")
    args = parser.parse_args()
    if args.seed < 100:
        parser.error("--seed must be >= 100: seeds 0 and 1 are kept for the eval")
    parts = args.out.resolve().parts
    if len(parts) > 2 and parts[1] == "Volumes" and not pathlib.Path("/Volumes", parts[2]).is_dir():
        parser.error(f"/Volumes/{parts[2]} is not mounted - plug the SSD in first")
    out_dir = args.out / args.name
    out_dir.mkdir(parents=True, exist_ok=True)
    lock_path = lock(out_dir, args.seed)
    max_trials = args.max_trials or 2 * args.demos + 10

    table_scene.write(scene=SCENE)
    env = table_env.TableEnv(scene=SCENE, seed=args.seed, variation_config=randomization.load(args.randomization))
    camera = outside_camera(env)
    reporter = None if args.no_labview else labview.Reporter(
        script="69", task="pick_place", object=f"demos_{args.name}_s{args.seed}", trials=args.demos, argv=sys.argv[1:],
        sim_time_getter=lambda: env.data.time, frame_getter=lambda: live_frame(env, camera))
    log, saved = [], 0
    report_path = out_dir / f"factory_s{args.seed}.json"
    for trial in range(max_trials):
        if saved == args.demos:
            break
        env.reset(randomize=True)
        if reporter:
            reporter.trial_start(trial, {"saved": saved})
        entry = record(env, args.seed, trial, out_dir, reporter)
        saved += entry["result"] == "saved"
        log.append(entry)
        if reporter:
            reporter.trial_end(entry["result"] == "saved", entry["result"], {"error": entry.get("error")})
        detail = (f"{entry['frames']} frames, {entry['size_mb']:.0f} MB, waist tracking {entry['waist_tracking_max_rad']:.3f} rad"
                  if "path" in entry else entry.get("error") or "")
        print(f"Trial {trial}: {entry['result']} ({detail}, {entry['wall_s']:.0f} s)", flush=True)
        report = summary(args, log)
        write_json(report_path, report)
    env.close()
    lock_path.unlink(missing_ok=True)
    demos = [e for e in log if "path" in e]
    if reporter:
        reporter.finish(report_path=report_path)
    print(f"{len(demos)} demos from {len(log)} trials -> {out_dir}")
    if demos:
        print(f"per demo: {report['frames_per_demo']:.0f} frames = {report['frames_per_demo'] / table_env.CONTROL_HZ:.1f} s sim, "
              f"{report['wall_s_per_demo']:.0f} s wall (incl. failures), {report['mb_per_demo']:.0f} MB; "
              f"100 demos: {report['wall_s_per_demo'] * 100 / 60:.0f} min in one process, {report['mb_per_demo'] * 100 / 1000:.1f} GB")
    return 0 if len(demos) == args.demos else 1


def summary(args, log):
    """factory_s<seed>.json: counts, failure reasons, per-demo averages and the log."""
    demos = [e for e in log if "path" in e]
    return {
        "seed": args.seed, "git": GIT, "scene": SCENE, "mode": "no-walk", "trials": len(log), "saved": len(demos),
        "success_rate": len(demos) / len(log) if log else None,
        "results": dict(collections.Counter(e["result"] for e in log)),
        "wall_s_per_demo": sum(e["wall_s"] for e in log) / len(demos) if demos else None,
        "mb_per_demo": sum(e["size_mb"] for e in demos) / len(demos) if demos else None,
        "frames_per_demo": sum(e["frames"] for e in demos) / len(demos) if demos else None,
        "log": log,
    }


if __name__ == "__main__":
    sys.exit(main())
