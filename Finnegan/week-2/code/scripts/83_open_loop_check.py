"""Level 1 Step 4 (plan Stage 5) - open-loop check: SmolVLA checkpoints against recorded teacher frames, no sim, no server.

F frames (--frames, default 20): F demos evenly spaced over the sorted --demos folder, frame i at tick
round(1 + i * (T - 52) / (F - 1)) of its demo. Each frame (head image + state + policy_wire.TASK) goes through the SmolVLAPredictor
of scripts/81_smolvla_server.py (same preprocessing, same run1 and MPS guards) with fixed noise (reset seed = i), identical for every
checkpoint. The predicted 50-step chunk is compared with the recorded actions[t:t+50], per group (arms 0-13 rad, hands 14-25,
waist 26 rad):
  hold_mae   per frame: MAE over steps 1-25 of repeating actions[t-1] (the "hold still" baseline)
  motion frame: hold_mae >= HOLD_FLOOR (the teacher moves there), else a still frame
  mae_all (steps 1-50), mae_exec (1-25) over all frames; mae_motion, hold_motion (1-25, means over motion frames);
  skill = 1 - mae_motion / hold_motion; mae_still (model drift where the teacher holds still, reported only);
  spread = std of the recorded target rows over all F x 50 rows, mean over the group's dims (reported only)
Gate (diagnostic, does not block Stage 9), per group with at least MIN_MOTION_FRAMES motion frames: skill > 0 at the last checkpoint
and its mae_exec <= 1.1x the previous checkpoint's. Groups with fewer motion frames print "n/a" and are not gated. The run is exit 0
whenever it completes, whatever the gate says; exit 2 = refused (run1 or MPS guard) or a checkpoint/demo problem.
Output: runs/l1w_open_loop_<label>.json and a printed table. One model in memory at a time. Demo npz files are only read.
Run1 rules while lerobot-train runs: checkpoints inside its --output_dir are refused (R4 not approved) unless --allow-during-training;
with it, a checkpoint that is not complete or younger than 60 s is skipped with its reason, never read. --device mps is refused
while a training runs (unless --force-mps). Compare checkpoints only on the same device.
Environment ~/dev/g1-ext/smolvla-test/.venv:
    ~/dev/g1-ext/smolvla-test/.venv/bin/python scripts/83_open_loop_check.py \\
        --checkpoints ~/dev/g1-ext/smolvla-runs/smoke/checkpoints/000200/pretrained_model \\
        --demos "/Volumes/T7/Robot Project SAP/DATA Collection/g1-demos/smoke_v1" --frames 4 --device cpu --threads 4 --label s4_smoke
    after run1:
    ~/dev/g1-ext/smolvla-test/.venv/bin/python scripts/83_open_loop_check.py --run ~/dev/g1-ext/smolvla-runs/run1 --label run1
"""
import argparse
import gc
import importlib.util
import json
import pathlib
import sys
import time

import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim import episode, policy_wire  # noqa: E402

# SmolVLAPredictor from 81, not rebuilt: same preprocessing and the same guards
_spec = importlib.util.spec_from_file_location("smolvla_server", REPO / "scripts" / "81_smolvla_server.py")
smolvla_server = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(smolvla_server)

DEMOS = pathlib.Path("/Volumes/T7/Robot Project SAP/DATA Collection/g1-demos/level1_walkup_v1")
GROUPS = {"arms": slice(0, 14), "hands": slice(14, 26), "waist": slice(26, 27)}
EXEC = 25  # steps 1-25 = what 82 executes per chunk
HOLD_FLOOR = 0.005  # hold_mae below this: the teacher holds still on this frame for this group
MIN_MOTION_FRAMES = 3  # fewer motion frames: the group is n/a, not gated
TREND_MAX = 1.1  # mae_exec may grow at most 10 % from the previous checkpoint


def fail(message):
    print(message, flush=True)
    raise SystemExit(2)


def at_least_two(text):
    value = int(text)
    if value < 2:
        raise argparse.ArgumentTypeError("--frames must be >= 2")
    return value


def demo_files(folder):
    return sorted(p for p in pathlib.Path(folder).glob("*.npz") if not p.name.startswith("._"))


def run_checkpoints(run_dir):
    """checkpoints/<step>/pretrained_model of a training run, sorted by step; "last" is skipped (a link to one of them)."""
    found = [p for p in (pathlib.Path(run_dir) / "checkpoints").glob("[0-9]*") if p.name.isdigit()]
    return [p / "pretrained_model" for p in sorted(found, key=lambda p: int(p.name))]


def load_frames(folder, n):
    """n frames as in the docstring. Only state, actions and head_image are read (each decompressed in full by np.load)."""
    files = demo_files(folder)
    if not files:
        fail(f"no demos (*.npz) in {folder}")
    frames = []
    for i, x in enumerate(np.linspace(0, len(files) - 1, n)):
        path = files[int(round(x))]
        with np.load(path, allow_pickle=False) as f:
            actions = f["actions"]
            t = int(round(1 + i * (len(actions) - 52) / (n - 1)))
            frames.append({"demo": path.name, "tick": t, "state": f["state"][t].copy(), "head_image": f["head_image"][t].copy(),
                           "hold": actions[t - 1].copy(), "target": actions[t:t + 50].copy()})
    return frames


def predict(predictor, frames):
    """One chunk per frame, noise seed = frame index. Returns (F, 50, 27) and the mean ms per chunk."""
    chunks, start = [], time.perf_counter()
    for i, frame in enumerate(frames):
        obs = episode.policy_observation(frame, policy_wire.TASK)
        problems = policy_wire.check_observation(obs)
        if problems:
            fail(f"frame {i} ({frame['demo']} tick {frame['tick']}): {problems}")
        predictor.reset(None, i)
        chunk, _ = predictor.infer(obs, frame["tick"])
        chunks.append(np.asarray(chunk, dtype=np.float32))
    return np.stack(chunks), (time.perf_counter() - start) * 1000 / len(frames)


def group_metrics(chunks, frames):
    """Per group: the fields of the docstring. chunks (F, 50, 27) predicted, frames from load_frames."""
    target = np.stack([f["target"] for f in frames]).astype(np.float64)
    hold = np.stack([np.repeat(f["hold"][None], 50, 0) for f in frames]).astype(np.float64)
    pred = chunks.astype(np.float64)
    out = {}
    for name, s in GROUPS.items():
        err, hold_err = np.abs(pred[..., s] - target[..., s]), np.abs(hold[..., s] - target[..., s])
        hold_frame, err_frame = hold_err[:, :EXEC].mean((1, 2)), err[:, :EXEC].mean((1, 2))
        motion = hold_frame >= HOLD_FLOOR
        k = int(motion.sum())
        row = {"motion_frames": k, "still_frames": len(frames) - k,
               "mae_all": float(err.mean()), "mae_exec": float(err[:, :EXEC].mean()),
               "mae_motion": float(err_frame[motion].mean()) if k else None,
               "hold_motion": float(hold_frame[motion].mean()) if k else None,
               "mae_still": float(err_frame[~motion].mean()) if k < len(frames) else None,
               "spread": float(target[..., s].reshape(-1, s.stop - s.start).std(0).mean()),
               "gated": k >= MIN_MOTION_FRAMES}
        row["skill"] = 1 - row["mae_motion"] / row["hold_motion"] if k and row["hold_motion"] > 0 else None
        out[name] = row
    return out


def gate(results):
    """Gate per group on the last checkpoint; also fills vs_prev (mae_exec / previous mae_exec) on every checkpoint."""
    for prev, cur in zip([None] + results[:-1], results):
        for name, row in cur["groups"].items():
            row["vs_prev"] = row["mae_exec"] / prev["groups"][name]["mae_exec"] if prev else None
    verdict = {}
    for name, row in results[-1]["groups"].items():
        if not row["gated"]:
            verdict[name] = {"pass": None, "note": f"n/a (hold-still ~0 on {row['still_frames']}/{len(results[-1]['ticks'])} frames)"}
            continue
        problems = []
        if not row["skill"] > 0:
            problems.append(f"skill {row['skill']:.3f} <= 0")
        if row["vs_prev"] is not None and row["vs_prev"] > TREND_MAX:
            problems.append(f"mae_exec {row['vs_prev']:.2f}x the previous checkpoint > {TREND_MAX}")
        verdict[name] = {"pass": not problems, "note": "; ".join(problems) or "ok"}
    return verdict


def fmt(value, spec=".4f"):
    return "-" if value is None else format(value, spec)


def print_table(results):
    print(f"{'step':>8} {'group':5} {'motion':>6} {'mae_all':>8} {'mae_exec':>8} {'mae_mot':>8} {'hold_mot':>8} "
          f"{'mae_still':>9} {'spread':>7} {'vs_prev':>7}  skill")
    for res in results:
        for name, row in res["groups"].items():
            skill = fmt(row["skill"], "+.3f") if row["gated"] else f"n/a (hold-still ~0 on {row['still_frames']}/" \
                                                                     f"{row['motion_frames'] + row['still_frames']} frames)"
            print(f"{str(res['step']):>8} {name:5} {row['motion_frames']:>6} {fmt(row['mae_all']):>8} {fmt(row['mae_exec']):>8} "
                  f"{fmt(row['mae_motion']):>8} {fmt(row['hold_motion']):>8} {fmt(row['mae_still']):>9} {fmt(row['spread']):>7} "
                  f"{fmt(row['vs_prev'], '.2f'):>7}  skill {skill}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    which = ap.add_mutually_exclusive_group(required=True)
    which.add_argument("--checkpoints", nargs="+", type=pathlib.Path, help="pretrained_model dirs (or their parents), in order")
    which.add_argument("--run", type=pathlib.Path, help="training run dir: all checkpoints/<step>/pretrained_model by step")
    ap.add_argument("--demos", type=pathlib.Path, default=DEMOS)
    ap.add_argument("--frames", type=at_least_two, default=20)
    ap.add_argument("--device", choices=("cpu", "mps"), default="cpu")
    ap.add_argument("--force-mps", action="store_true", help="--device mps even while lerobot-train runs")
    ap.add_argument("--allow-during-training", action="store_true",
                    help="read checkpoints of a running training (only complete ones older than 60 s); needs the user's OK (R4)")
    ap.add_argument("--threads", type=int, default=None)
    ap.add_argument("--label", default=None, help="output runs/l1w_open_loop_<label>.json (default: run dir name or 'checkpoints')")
    args = ap.parse_args()

    # run1 guard before anything is listed or loaded. With --allow-during-training checkpoint_problem only reports "not ready",
    # which the loop below turns into a skip.
    cmdlines = policy_wire.training_cmdlines()
    if not args.allow_during_training:
        for path in [args.run] if args.run else args.checkpoints:
            problem = policy_wire.checkpoint_problem(path.expanduser(), cmdlines, False, time.time())
            if problem:
                fail(f"REFUSED {path}: {problem}")
    checkpoints = run_checkpoints(args.run.expanduser()) if args.run else [p.expanduser() for p in args.checkpoints]
    if not checkpoints:
        fail(f"no checkpoints/<step>/pretrained_model under {args.run}")
    label = args.label or (args.run.name if args.run else "checkpoints")

    frames, results, skipped = None, [], []
    for ckpt in checkpoints:
        problem = policy_wire.checkpoint_problem(ckpt, cmdlines, args.allow_during_training, time.time())
        if problem:  # only with --allow-during-training (otherwise refused above): not complete or younger than 60 s
            print(f"SKIP {ckpt}: {problem}", flush=True)
            skipped.append({"checkpoint": str(ckpt), "reason": problem})
            continue
        predictor = smolvla_server.SmolVLAPredictor(ckpt, device=args.device, threads=args.threads, force_mps=args.force_mps,
                                                    allow_during_training=args.allow_during_training)
        if frames is None:  # after the first model: a refused device never reads the demos
            frames = load_frames(args.demos, args.frames)
            print(f"frames: {[(f['demo'], f['tick']) for f in frames]}", flush=True)
        chunks, chunk_ms = predict(predictor, frames)
        meta = predictor.metadata
        results.append({"checkpoint": str(meta.get("checkpoint", ckpt)), "step": meta.get("step"), "load_s": predictor.load_s,
                        "chunk_ms": chunk_ms, "groups": group_metrics(chunks, frames),
                        "ticks": [f["tick"] for f in frames]})
        print(f"step {meta.get('step')}: load {predictor.load_s:.1f} s, {chunk_ms:.0f} ms per chunk", flush=True)
        del predictor
        gc.collect()
    if not results:
        fail(f"no checkpoint ready: {skipped}")

    verdict = gate(results)
    print_table(results)
    line = ", ".join(f"{name} {v['note']}" if v["pass"] is None else f"{name} {'PASS' if v['pass'] else 'FAIL'} ({v['note']})"
                     for name, v in verdict.items())
    print(f"GATE (diagnostic, step {results[-1]['step']}): {line}")
    report = {"device": args.device, "threads": args.threads, "frames": args.frames, "demos": str(args.demos),
              "frame_list": [{"demo": f["demo"], "tick": f["tick"]} for f in frames], "ticks": [f["tick"] for f in frames],
              "prompt": policy_wire.TASK, "hold_floor": HOLD_FLOOR, "min_motion_frames": MIN_MOTION_FRAMES,
              "trend_max": TREND_MAX, "checkpoints": results, "skipped": skipped, "gate": verdict, "argv": sys.argv[1:]}
    out = REPO / "runs" / f"l1w_open_loop_{label}.json"
    episode.write_verified(out, json.dumps(report, indent=2, default=float).encode())
    print(f"report: {out}")


if __name__ == "__main__":
    main()
