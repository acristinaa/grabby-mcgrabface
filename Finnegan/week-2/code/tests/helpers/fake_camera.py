"""Fake head camera: streams the head_image frames of an episode .npz in the grabber's byte format (g1sim/real_camera.py).

Why it exists: 84, 86 and 89 read the D455 through a command's stdout. With `--camera-cmd "python tests/helpers/fake_camera.py
--npz FILE"` they run the same code path without a robot PC. --stop-after S ends the stream after S seconds, to test what
happens when the camera dies.
    python tests/helpers/fake_camera.py --npz tests/data/selftest_episode.npz [--fps 15] [--stop-after 5]
"""
import argparse
import json
import pathlib
import sys
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
from g1sim.real_camera import CAPTURE_H, CAPTURE_W, DEFAULT_FPS, pack_frame  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Fake head camera from an episode .npz")
    parser.add_argument("--npz", required=True, help="episode file with head_image (T, 200, 320, 3) uint8 RGB")
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS)
    parser.add_argument("--stop-after", type=float, default=None, help="end the stream after this many seconds")
    args = parser.parse_args()
    with np.load(args.npz, allow_pickle=False) as f:
        frames = f["head_image"]
    info = {"device": f"fake:{args.npz}", "width": CAPTURE_W, "height": CAPTURE_H, "fps": args.fps,
            "python": sys.version.split()[0]}
    print(json.dumps(info), flush=True)
    out, start, index = sys.stdout.buffer, time.monotonic(), 0
    while args.stop_after is None or time.monotonic() - start < args.stop_after:
        try:
            out.write(pack_frame(frames[index % len(frames)], time.time(), False))
            out.flush()
        except BrokenPipeError:
            return 0  # the reader closed the stream
        index += 1
        time.sleep(1.0 / args.fps)
    return 0


if __name__ == "__main__":
    sys.exit(main())
