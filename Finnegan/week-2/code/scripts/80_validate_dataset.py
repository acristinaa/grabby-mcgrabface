"""Level 1 Step 6 (plan Stage 7) - validate the recorded demos and their LeRobot v3.0 dataset before training.

Demo level (every npz the converter mapped): episode.validate, success, handover snapshot, sidecar with seed >= 100, stance_source,
timestep 1/300; no variation twice; waist |q| < 2.6 rad; with folders given, every npz there must be in the map. Dataset level:
no macOS "._*" file under --root (lerobot would crash on it); LeRobotDataset loads; episode and frame counts equal the npz files;
every episode equals its npz in full - length, every state and action value bit-equal, its first frame's image and --frames more
random images bit-equal (lossless: uint8 round trip is exact); stats of state and action equal numpy over all npz frames; task string
= the npz's task and one of policy_wire.TASKS. Prints the problems; exit 0 only with none.
Environment ~/dev/g1-ext/smolvla-test/.venv:
    ~/dev/g1-ext/smolvla-test/.venv/bin/python scripts/80_validate_dataset.py \\
        --root "/Volumes/T7/Robot Project SAP/DATA Collection/lerobot/g1_volleyball_walkup_v1"
"""
import argparse
import hashlib
import json
import os
import pathlib
import sys

os.environ.setdefault("HF_HUB_OFFLINE", "1")

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

import numpy as np  # noqa: E402
from lerobot.datasets.lerobot_dataset import LeRobotDataset  # noqa: E402

from g1sim import action_space, episode, policy_wire  # noqa: E402

IMAGE_KEY = "observation.images.camera1"
WAIST = action_space.SLICES["waist"].start


def check_demos(paths):
    problems, seen = [], {}
    for path in paths:
        ep = episode.load(path)
        bad = list(episode.validate(ep))
        if not ep["success"]:
            bad.append("success=False")
        if "handover_json" not in ep:
            bad.append("no handover snapshot")
        if np.abs(ep["state"][:, WAIST]).max() >= 2.6 or np.abs(ep["actions"][:, WAIST]).max() >= 2.6:
            bad.append("waist beyond 2.6 rad")
        side = path.with_suffix(".json")
        if not side.is_file():
            bad.append("no sidecar")
        else:
            info = json.loads(side.read_text())
            if not isinstance(info.get("seed"), int) or info["seed"] < 100:
                bad.append(f"seed {info.get('seed')} < 100")
            if info.get("stance_source") not in ("provisional", "measured"):
                bad.append(f"stance_source {info.get('stance_source')!r}")
            if abs(info.get("timestep", 0) - 1 / 300) > 1e-12:
                bad.append(f"timestep {info.get('timestep')}")
            key = hashlib.sha1(json.dumps(info.get("variation"), sort_keys=True).encode()).hexdigest()
            if key in seen:
                bad.append(f"same variation as {seen[key]}")
            seen[key] = path.name
        problems += [f"{path.name}: {b}" for b in bad]
    return problems


def check_dataset(root, repo_id, mapping, frames, rng):
    twins = [str(p) for p in pathlib.Path(root).rglob("._*")]
    if twins:
        return [f"{len(twins)} macOS ._ files under the dataset (lerobot crashes on them), e.g. {twins[0]}"], None
    problems = []
    ds = LeRobotDataset(repo_id, root=root)
    if ds.num_episodes != len(mapping):
        problems.append(f"{ds.num_episodes} episodes in the dataset, {len(mapping)} demos converted")
    total = sum(m["frames"] for m in mapping)
    if ds.num_frames != total:
        problems.append(f"{ds.num_frames} frames in the dataset, {total} in the demos")
    starts = np.cumsum([0] + [m["frames"] for m in mapping])
    columns = ds.hf_dataset.select_columns(["episode_index", "frame_index", "observation.state", "action"]).with_format("numpy")
    episode_index, frame_index = np.asarray(columns["episode_index"]), np.asarray(columns["frame_index"])
    state, action = np.stack(columns["observation.state"]), np.stack(columns["action"])
    all_state, all_action = [], []
    for m in mapping:
        ep = episode.load(m["npz"])
        rows = np.flatnonzero(episode_index == m["episode_index"])
        where = f"episode {m['episode_index']} ({pathlib.Path(m['npz']).name})"
        if not len(rows) == len(ep["timestamp"]) == m["frames"]:
            problems.append(f"{where}: {len(rows)} frames in the dataset, {len(ep['timestamp'])} in the npz, {m['frames']} in the map")
            continue
        rows = rows[np.argsort(frame_index[rows])]
        if not np.array_equal(frame_index[rows], np.arange(len(rows))):
            problems.append(f"{where}: frame_index not 0..{len(rows) - 1}")
        if not np.array_equal(state[rows], ep["state"]):
            problems.append(f"{where}: state differs from the npz")
        if not np.array_equal(action[rows], ep["actions"]):
            problems.append(f"{where}: action differs from the npz")
        all_state.append(ep["state"])
        all_action.append(ep["actions"])
    for key, values in (("observation.state", all_state), ("action", all_action)):
        values = np.concatenate(values).astype(np.float64)
        stats = ds.meta.stats.get(key, {})
        for name, expected in (("mean", values.mean(0)), ("std", values.std(0)), ("min", values.min(0)), ("max", values.max(0))):
            got = np.asarray(stats.get(name, []), dtype=np.float64).reshape(-1)
            if got.shape != expected.shape or not np.allclose(got, expected, rtol=1e-4, atol=1e-5):
                problems.append(f"stats {key} {name} differ from numpy over the npz frames")
    image_frames = [int(starts[e]) for e in range(len(mapping))]  # the first frame of every episode
    image_frames += [int(i) for i in rng.choice(ds.num_frames, size=min(frames, ds.num_frames), replace=False)]
    for index in sorted(set(image_frames)):
        e = int(np.searchsorted(starts, index, side="right") - 1)
        t = int(index - starts[e])
        item = ds[int(index)]
        ep = episode.load(mapping[e]["npz"])
        image = (item[IMAGE_KEY].numpy().transpose(1, 2, 0) * 255.0).round().astype(np.uint8)
        where = f"frame {index} (episode {e} t {t}, {pathlib.Path(mapping[e]['npz']).name})"
        if int(item["episode_index"]) != e or int(item["frame_index"]) != t:
            problems.append(f"{where}: dataset says episode {int(item['episode_index'])} frame {int(item['frame_index'])}")
        if not np.array_equal(image, ep["head_image"][t]):
            problems.append(f"{where}: image differs (max {np.abs(image.astype(int) - ep['head_image'][t]).max()} grey levels)")
        if item["task"] != ep["task"] or item["task"] not in policy_wire.TASKS:  # 69 records one of TASKS per demo
            problems.append(f"{where}: task {item['task']!r}, npz {ep['task']!r}")
    return problems, ds


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("folders", nargs="*", type=pathlib.Path, help="demo folders: every npz there must be in the dataset")
    parser.add_argument("--root", type=pathlib.Path, required=True, help="LeRobot dataset folder written by 79")
    parser.add_argument("--frames", type=int, default=100, help="random image frames compared with the npz (plus each first frame)")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    report = json.loads(args.root.with_name(args.root.name + ".convert.json").read_text())
    mapping = report["map"]
    demo_problems = check_demos([pathlib.Path(m["npz"]) for m in mapping])
    mapped = {pathlib.Path(m["npz"]).resolve() for m in mapping}
    demo_problems += [f"{p.name}: in {p.parent} but not in the dataset" for f in args.folders
                      for p in sorted(f.glob("episode_*.npz")) if p.resolve() not in mapped]
    dataset_problems, ds = check_dataset(args.root, report["repo_id"], mapping, args.frames, np.random.default_rng(args.seed))
    problems = demo_problems + dataset_problems
    if ds is not None:
        print(f"demos {len(mapping)}, dataset {ds.num_episodes} episodes / {ds.num_frames} frames, fps {ds.fps}, "
              f"features {sorted(ds.features)}; all state/action values + {len(mapping)} first frames + {args.frames} random "
              f"images compared; skipped by the converter: {len(report['skipped'])}")
    print(f"{len(problems)} problems" + ("" if not problems else ":\n  " + "\n  ".join(problems[:30])))
    return 0 if not problems and not report["skipped"] else 1


if __name__ == "__main__":
    sys.exit(main())
