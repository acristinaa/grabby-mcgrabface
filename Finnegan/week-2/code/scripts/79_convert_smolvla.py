"""Level 1 Step 6 (plan Stage 7) - recorded volleyball demos (.npz, g1sim/episode.py) -> LeRobot dataset v3.0 for SmolVLA.

Runs in the SmolVLA environment (lerobot 0.6.1 with the [dataset] extra). One LeRobot episode per demo, in the order seed, trial.
Columns: observation.images.camera1 (head camera 200 x 320, the first camera of lerobot/smolvla_base, so no rename map),
observation.state and action (27-D, names from g1sim/action_space.py), task = the demo's own sentence (npz "task"), fps 30.
Images are stored lossless (dtype "image", no video codec): the frames carry sensor noise, and training reads no video decoder.
Only demos that pass episode.validate, have success=True and a sidecar are written; macOS "._*" files and "*.part" are skipped.
On the exFAT T7 macOS writes a "._<name>" twin of every new file; lerobot globs "*/*.parquet" and crashes on "._file-000.parquet"
(review level1-step6-converter F1). So after finalize() every "._*" under --root is deleted, and 80 reports any that come back.
Next to the dataset: <root>.convert.json with the episode index -> npz file map (the validator 80 and the eval need it).
Environment ~/dev/g1-ext/smolvla-test/.venv:
    ~/dev/g1-ext/smolvla-test/.venv/bin/python scripts/79_convert_smolvla.py \\
        "/Volumes/T7/Robot Project SAP/DATA Collection/g1-demos/level1_walkup_v1" \\
        --root "/Volumes/T7/Robot Project SAP/DATA Collection/lerobot/g1_volleyball_walkup_v1"
"""
import argparse
import json
import os
import pathlib
import shutil
import sys
import time

os.environ.setdefault("HF_HUB_OFFLINE", "1")  # a local dataset: never ask the Hub

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from lerobot.datasets.lerobot_dataset import LeRobotDataset  # noqa: E402

from g1sim import action_space, episode  # noqa: E402

REPO_ID = "local/g1_volleyball_walkup_v1"
ROBOT_TYPE = "unitree_g1_brainco"
IMAGE_KEY = "observation.images.camera1"
IMAGE_SHAPE = (200, 320, 3)


def features():
    vector = {"dtype": "float32", "shape": (action_space.DIM,), "names": list(action_space.NAMES)}
    return {IMAGE_KEY: {"dtype": "image", "shape": IMAGE_SHAPE, "names": ["height", "width", "channel"]},
            "observation.state": dict(vector), "action": dict(vector)}


def demo_files(folders):
    """Episode files of all folders, sorted by seed and trial (file name), without macOS "._" twins and temp files."""
    return sorted((p for f in folders for p in pathlib.Path(f).glob("episode_*.npz")),
                  key=lambda p: p.name)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("folders", nargs="+", type=pathlib.Path, help="demo folders from 69")
    parser.add_argument("--root", type=pathlib.Path, required=True, help="dataset folder to create")
    parser.add_argument("--repo-id", default=REPO_ID)
    parser.add_argument("--overwrite", action="store_true", help="replace an existing LeRobot dataset at --root")
    args = parser.parse_args()
    if args.root.exists():
        if not args.overwrite:
            parser.error(f"{args.root} exists - --overwrite replaces it")
        if not (args.root / "meta" / "info.json").is_file():
            parser.error(f"{args.root} is not a LeRobot dataset - not deleted")
        shutil.rmtree(args.root)
    args.root.parent.mkdir(parents=True, exist_ok=True)

    dataset = LeRobotDataset.create(repo_id=args.repo_id, fps=30, features=features(), root=args.root, robot_type=ROBOT_TYPE,
                                    use_videos=False, image_writer_threads=4)
    written, skipped, frames, start = [], {}, 0, time.perf_counter()
    for path in demo_files(args.folders):
        ep = episode.load(path)
        problems = episode.validate(ep)
        if not ep["success"]:
            problems.append("success=False")
        if ep["fps"] != dataset.fps:
            problems.append(f"fps {ep['fps']} instead of {dataset.fps}")
        if ep["head_image"].shape[1:] != IMAGE_SHAPE:
            problems.append(f"head_image {ep['head_image'].shape[1:]} instead of {IMAGE_SHAPE}")
        if not path.with_suffix(".json").is_file():
            problems.append("no sidecar")
        if problems:
            skipped[str(path)] = problems
            continue
        for t in range(len(ep["timestamp"])):
            dataset.add_frame({IMAGE_KEY: ep["head_image"][t], "observation.state": ep["state"][t],
                               "action": ep["actions"][t], "task": ep["task"]})
        dataset.save_episode()
        written.append({"episode_index": len(written), "npz": str(path), "frames": len(ep["timestamp"])})
        frames += len(ep["timestamp"])
        print(f"episode {len(written) - 1}: {path.name} ({len(ep['timestamp'])} frames, "
              f"{time.perf_counter() - start:.0f} s so far)", flush=True)
    dataset.finalize()
    twins = [p for p in args.root.rglob("._*") if p.is_file()]
    for p in twins:
        p.unlink()
    print(f"removed {len(twins)} macOS ._ files under {args.root}")
    report = {"repo_id": args.repo_id, "root": str(args.root), "episodes": len(written), "frames": frames,
              "wall_s": round(time.perf_counter() - start, 1), "skipped": skipped, "map": written}
    report_path = args.root.with_name(args.root.name + ".convert.json")
    episode.write_verified(report_path, json.dumps(report, indent=2).encode())
    print(f"{len(written)} episodes, {frames} frames -> {args.root} ({report['wall_s']:.0f} s); skipped {len(skipped)}: {skipped}")
    print(f"report: {report_path}")
    return 0 if written and not skipped else 1


if __name__ == "__main__":
    sys.exit(main())
