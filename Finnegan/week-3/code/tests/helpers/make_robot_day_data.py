"""Build the robot-day test data from dataset episode 0 (internal disk, never the T7).

Why: the operators' laptops have no dataset. The self-test 89, the camera overlay (84) and the slow replay (86 replay, 81 --replay)
need one real training episode in small files that travel with the repo:
  tests/data/selftest_episode.npz       frames 0-2 with images (81 --self-test, fake_camera)
  tests/data/replay_demo.npz            all actions and states, head images blank (81 --replay only looks at images for diagnostics)
  docs/robot_day/reference_head_frame.png   frame 0, the picture the real head camera must line up with
Runs in the SmolVLA env (it has pyarrow and PIL; the sim env does not):
    ~/dev/g1-ext/smolvla-test/.venv/bin/python tests/helpers/make_robot_day_data.py
"""
import argparse
import io
import json
import pathlib
import sys

import numpy as np
import pyarrow.parquet as pq
from PIL import Image

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from g1sim import episode, policy_wire  # noqa: E402

DATASET = pathlib.Path.home() / "dev" / "g1-ext" / "datasets" / "g1_volleyball_walkup_v1"  # LeRobot v3.0, PLAN.md header
EPISODE = 0                      # PLAN.md section 1: dataset episode 0 is a successful level-1 demo
SELFTEST_FRAMES = 3              # PLAN.md Track E1 step 1: frames 0-2
IMAGE_FEATURE = "observation.images.camera1"   # dataset meta/info.json; PNG bytes per frame
MAX_FILE_BYTES = 1_000_000       # PLAN.md frontmatter: each npz < 1 MB so it fits in the repo
REPLAY_NOTE = ("head_image is all zeros on purpose: 81 --replay serves the recorded actions and uses images only for "
               "diagnostics, so the real camera picture does not matter for the slow replay.")
OUT_SELFTEST = REPO / "tests" / "data" / "selftest_episode.npz"
OUT_REPLAY = REPO / "tests" / "data" / "replay_demo.npz"
OUT_FRAME = REPO / "docs" / "robot_day" / "reference_head_frame.png"


def episode_rows(dataset: pathlib.Path, index: int) -> dict:
    """The episode's row in meta/episodes (length, data file, source stem if the converter stored one)."""
    for path in sorted((dataset / "meta" / "episodes").glob("*/*.parquet")):
        for row in pq.read_table(path).to_pylist():
            if row["episode_index"] == index:
                return row
    raise SystemExit(f"episode {index} not in {dataset}/meta/episodes - is the dataset complete? (scripts/80_validate_dataset.py)")


def dataset_fps(dataset: pathlib.Path) -> int:
    """The recording rate from meta/info.json, so the replay timestamps match the speed the demo was recorded at."""
    return int(json.loads((dataset / "meta" / "info.json").read_text())["fps"])


def read_frames(dataset: pathlib.Path, meta: dict) -> list[dict]:
    """All frames of the episode from its data parquet file, in frame order."""
    path = dataset / "data" / f"chunk-{meta['data/chunk_index']:03d}" / f"file-{meta['data/file_index']:03d}.parquet"
    rows = [row for row in pq.read_table(path).to_pylist() if row["episode_index"] == meta["episode_index"]]
    rows.sort(key=lambda row: row["frame_index"])
    if len(rows) != meta["length"]:
        raise SystemExit(f"{path}: {len(rows)} frames for episode {meta['episode_index']}, meta says {meta['length']}")
    return rows


def decode_image(cell: dict) -> np.ndarray:
    """PNG bytes of one frame -> (200, 320, 3) uint8 RGB; refuses any other size (the policy only takes this one)."""
    image = np.asarray(Image.open(io.BytesIO(cell["bytes"])).convert("RGB"), dtype=np.uint8)
    if image.shape != policy_wire.IMAGE_SHAPE:
        raise SystemExit(f"head image is {image.shape}, expected {policy_wire.IMAGE_SHAPE} - wrong dataset?")
    return image


def build_episode(rows: list[dict], images: np.ndarray, task: str, fps: int) -> dict:
    """Frames -> the g1sim/episode.py layout that 81, 84 and fake_camera read."""
    return {"head_image": images,
            "state": np.asarray([row["observation.state"] for row in rows], dtype=np.float32),
            "actions": np.asarray([row["action"] for row in rows], dtype=np.float32),
            "timestamp": np.arange(len(rows), dtype=np.float64) / fps,
            "task": np.str_(task), "fps": np.int64(fps), "success": np.bool_(True)}


def save(path: pathlib.Path, ep: dict) -> int:
    """Write and re-check one episode; fails loudly if it is invalid or too large for the repo."""
    problems = episode.validate(ep)
    if problems:
        raise SystemExit(f"{path.name}: {problems}")
    buffer = io.BytesIO()
    np.savez_compressed(buffer, **ep)
    if buffer.getbuffer().nbytes >= MAX_FILE_BYTES:
        raise SystemExit(f"{path.name} would be {buffer.getbuffer().nbytes} bytes (limit {MAX_FILE_BYTES})")
    episode.write_verified(path, buffer.getbuffer())
    return path.stat().st_size


def main() -> None:
    parser = argparse.ArgumentParser(description="robot-day test data from dataset episode 0")
    parser.add_argument("--dataset", type=pathlib.Path, default=DATASET)
    args = parser.parse_args()

    meta = episode_rows(args.dataset, EPISODE)
    task = meta["tasks"][0]
    if task != policy_wire.TASK:
        raise SystemExit(f"episode task {task!r} differs from policy_wire.TASK {policy_wire.TASK!r}")
    rows = read_frames(args.dataset, meta)
    fps = dataset_fps(args.dataset)
    print(f"episode {EPISODE}: {len(rows)} frames at {fps} fps, task {task!r}, source stem {meta.get('source_stem', 'not recorded')}")

    first = np.stack([decode_image(rows[i][IMAGE_FEATURE]) for i in range(SELFTEST_FRAMES)])
    size = save(OUT_SELFTEST, build_episode(rows[:SELFTEST_FRAMES], first, task, fps))
    print(f"{OUT_SELFTEST.relative_to(REPO)}: {SELFTEST_FRAMES} frames, {size} bytes")

    blank = np.zeros((len(rows), *policy_wire.IMAGE_SHAPE), dtype=np.uint8)
    replay = build_episode(rows, blank, task, fps)
    replay["note"] = np.str_(REPLAY_NOTE)
    size = save(OUT_REPLAY, replay)
    print(f"{OUT_REPLAY.relative_to(REPO)}: {len(rows)} frames, blank images, {size} bytes")

    OUT_FRAME.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(first[0]).save(OUT_FRAME)
    print(f"{OUT_FRAME.relative_to(REPO)}: frame 0, {first[0].shape}")


if __name__ == "__main__":
    main()
