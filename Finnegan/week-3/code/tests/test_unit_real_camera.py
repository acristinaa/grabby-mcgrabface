"""Unit: head-camera framing and the grabber byte stream (g1sim/real_camera.py, tests/helpers/fake_camera.py).

No robot PC: the stream test runs the fake camera as a subprocess, which uses the same pack_frame() as the real grabber.
"""
import ast
import pathlib
import sys

import numpy as np
import pytest

pytest.importorskip("cv2")
from g1sim import real_camera  # noqa: E402

pytestmark = pytest.mark.unit

FAKE_CAMERA = pathlib.Path(__file__).resolve().parent / "helpers" / "fake_camera.py"


def test_to_policy_frame_flags_640x480_as_cropped():
    """Proves: a 640 x 480 (4:3) frame is flagged cropped, a 1280 x 800 (16:10) frame is not; both come out 200 x 320.
    Catches: a D455 left in 640 x 480 passing the camera check - the policy would see a narrower view than in training."""
    small, cropped_small = real_camera.to_policy_frame(np.zeros((480, 640, 3), dtype=np.uint8))
    full, cropped_full = real_camera.to_policy_frame(np.zeros((800, 1280, 3), dtype=np.uint8))

    assert cropped_small and not cropped_full
    assert small.shape == full.shape == (200, 320, 3)


def test_to_policy_frame_converts_bgr_to_rgb():
    """Proves: a pure blue OpenCV (BGR) frame comes out as blue in RGB order.
    Catches: a missing BGR -> RGB swap - the policy would see a red ball blue and a blue sky orange."""
    blue_bgr = np.zeros((800, 1280, 3), dtype=np.uint8)
    blue_bgr[..., 0] = 255

    rgb, _ = real_camera.to_policy_frame(blue_bgr)

    assert (rgb[..., 2] == 255).all() and (rgb[..., 0] == 0).all()


def test_grabber_parses_as_python_3_8():
    """Proves: the source sent to the robot PC parses as Python 3.8 and starts with the annotations import.
    Catches: newer syntax in to_policy_frame / pack_frame that would crash the grabber on the robot PC's older python3."""
    tree = ast.parse(real_camera.GRABBER, feature_version=(3, 8))

    assert isinstance(tree.body[0], ast.ImportFrom) and tree.body[0].module == "__future__"


def test_frame_bytes_round_trip_through_fake_camera(tmp_path):
    """Proves: frames written by fake_camera.py (grabber byte format) arrive in FrameStream bit-exact, with the info line.
    Catches: a header field out of step (length, width, height) - frames would be garbled or the stream would stall."""
    rng = np.random.default_rng(0)
    frames = rng.integers(0, 256, (3, 200, 320, 3), dtype=np.uint8)
    npz = tmp_path / "frames.npz"
    np.savez(npz, head_image=frames)

    received, stats = real_camera.grab([sys.executable, str(FAKE_CAMERA), "--npz", str(npz), "--fps", "30"], count=3,
                                       timeout_s=10)

    assert len(received) == 3 and stats["info"]["device"].startswith("fake:")
    assert all(any(np.array_equal(frame, original) for original in frames) for frame in received)
