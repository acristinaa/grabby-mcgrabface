"""Real G1 head camera (D455): a grabber that runs on the robot PC over SSH, the frame stream reader, and framing checks.

Why it exists: the policy was trained on 200 x 320 RGB frames of a 1280 x 800 view (PLAN.md 2.4). A different resolution
(640 x 480 crops the view) or a BGR/RGB swap would show the policy a world it never saw. The robot PC downsizes each frame
with the SAME to_policy_frame() as this laptop (its source is shipped inside GRABBER), so both sides cannot drift apart.
Frame bytes: b"FRM1" + >I payload length + >d robot time + >H width + >H height + >B cropped + RGB bytes.
No DDS, no writer: the camera is read over an SSH pipe (stdout of the grabber).
"""
import inspect
import json
import shlex
import struct
import subprocess
import threading
import time
from collections import deque

import cv2
import numpy as np

POLICY_H, POLICY_W = 200, 320                       # head_image size of the dataset (g1sim/table_env.py IMAGE_SIZES)
CAPTURE_W, CAPTURE_H = 1280, 800                    # D455 colour mode with the sim's 16:10 field of view (PLAN.md 2.4)
POLICY_ASPECT = POLICY_W / POLICY_H                 # 1.6 = 16:10
ASPECT_TOL = 0.01                                   # a 4:3 frame (1.33) is far outside, rounding is not
MAGIC = b"FRM1"
HEADER = ">IdHHB"                                   # payload length, robot time, width, height, cropped flag
HEADER_SIZE = struct.calcsize(HEADER)
DEFAULT_FPS = 15                                    # D455 colour rate used on the robot day (PLAN.md 2.4)
INFO_TIMEOUT_S = 30.0                               # SSH may wait for the password typed in the terminal
RATE_WINDOW_S = 2.0                                 # receive times kept for the measured fps
STDERR_KEEP_LINES = 50                              # grabber/ssh stderr lines kept for error messages
STDERR_TAIL_CHARS = 500                             # how much of that stderr a CameraError shows
CLOSE_WAIT_S = 5.0                                  # grabber gets this long to stop after SIGTERM, then it is killed
SSH_CONNECT_TIMEOUT_S = 5                           # a wrong robot-PC address fails fast instead of hanging
GRAB_POLL_S = 0.005                                 # grab() looks for a new frame this often (15 fps = one per 67 ms)
DATASET_IMAGE_STATS = {"mean": (0.378, 0.479, 0.551), "std": (0.272, 0.252, 0.230)}   # RGB 0..1, $DATASET/meta/stats.json


class CameraError(RuntimeError):
    """The camera stream did not start or broke; the message says what to check."""


def to_policy_frame(bgr):
    """OpenCV BGR frame -> (RGB uint8 (200, 320, 3), cropped). cropped = the frame was not 16:10, so it was centre-cut and
    shows less than the sim camera did; the policy must not run on such frames."""
    h, w = bgr.shape[:2]
    cropped = abs(w / h - POLICY_ASPECT) > ASPECT_TOL
    if cropped:
        if w / h > POLICY_ASPECT:
            keep = int(round(h * POLICY_ASPECT))
            bgr = bgr[:, (w - keep) // 2:(w - keep) // 2 + keep]
        else:
            keep = int(round(w / POLICY_ASPECT))
            bgr = bgr[(h - keep) // 2:(h - keep) // 2 + keep]
    small = cv2.resize(bgr, (POLICY_W, POLICY_H), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(small, cv2.COLOR_BGR2RGB), cropped


def pack_frame(rgb, robot_time, cropped):
    """One frame in the stream format (module docstring)."""
    h, w = rgb.shape[:2]
    payload = np.ascontiguousarray(rgb, dtype=np.uint8).tobytes()
    return MAGIC + struct.pack(HEADER, len(payload), robot_time, w, h, int(cropped)) + payload


_GRABBER_MAIN = '''
def main():
    import argparse, sys, time
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", type=int, required=True)
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS)
    parser.add_argument("--count", type=int, default=0, help="frames to send, 0 = until stopped")
    args = parser.parse_args()
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_H)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    ok, bgr = cap.read()
    if not ok:
        print(json.dumps({"error": "no frame from /dev/video%d (wrong number or camera busy)" % args.device}), flush=True)
        sys.exit(1)
    info = {"device": "/dev/video%d" % args.device, "width": bgr.shape[1], "height": bgr.shape[0], "fps": args.fps,
            "python": sys.version.split()[0]}
    print(json.dumps(info), flush=True)
    out, sent = sys.stdout.buffer, 0
    while ok and (args.count == 0 or sent < args.count):
        rgb, cropped = to_policy_frame(bgr)
        out.write(pack_frame(rgb, time.time(), cropped))
        out.flush()
        sent += 1
        ok, bgr = cap.read()
    cap.release()


main()
'''
# The robot PC has no copy of this repo, so the grabber's functions and constants travel to it as Python source text
# (piped into `python3 -`). The constants are written out as Python assignments with the values of this module
_GRABBER_CONSTANTS = "\n".join(f"{name} = {globals()[name]!r}" for name in (
    "POLICY_H", "POLICY_W", "CAPTURE_W", "CAPTURE_H", "POLICY_ASPECT", "ASPECT_TOL", "MAGIC", "HEADER", "DEFAULT_FPS"))
# Python source the robot PC runs with `python3 - --device N` (python3 >= 3.8, numpy + cv2 only)
GRABBER = "\n".join(["from __future__ import annotations", "import json, struct", "import cv2", "import numpy as np",
                     _GRABBER_CONSTANTS, inspect.getsource(to_policy_frame), inspect.getsource(pack_frame), _GRABBER_MAIN])


def ssh_cmd(host: str, *remote) -> list:
    """ssh command line for the robot PC; a short connect timeout so a wrong address fails fast."""
    return ["ssh", "-o", f"ConnectTimeout={SSH_CONNECT_TIMEOUT_S}", host, *map(str, remote)]


def grabber_cmd(robot_pc: str, device: int, fps: int = DEFAULT_FPS) -> list:
    """Command that starts GRABBER on the robot PC (pass GRABBER as stdin_text)."""
    return ssh_cmd(robot_pc, "python3", "-", "--device", device, "--fps", fps)


def camera_cmd(cmd: str) -> list:
    """--camera-cmd "CMD" -> argument list (e.g. the fake camera in tests)."""
    return shlex.split(cmd)


class FrameStream:
    """Runs a grabber command and keeps the newest frame. .info = the grabber's first JSON line."""

    def __init__(self, cmd, stdin_text=None):
        self.cmd = list(cmd)
        self._proc = subprocess.Popen(self.cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            if stdin_text is not None:
                self._proc.stdin.write(stdin_text.encode())
            self._proc.stdin.close()  # `python3 -` reads its script until end of input
        except BrokenPipeError:
            pass  # the command already ended; the missing info line below reports it with its stderr
        self._lock = threading.Lock()
        self._ready = threading.Event()
        self.info, self.error = None, None
        self._latest, self._received_at, self._cropped, self._frames = None, None, None, 0
        self._times = deque()
        self._stderr_lines = deque(maxlen=STDERR_KEEP_LINES)
        # stderr is read all the time: a full 64 KB pipe would block the grabber and stop the frames
        self._stderr_thread = threading.Thread(target=self._drain_stderr, daemon=True)
        self._stderr_thread.start()
        threading.Thread(target=self._read_loop, daemon=True).start()
        if not self._ready.wait(INFO_TIMEOUT_S) or self.info is None:
            self.close()
            raise CameraError(f"camera did not start ({self.error or 'no info line'}). Command: {' '.join(self.cmd)}. "
                              f"stderr: {self._stderr_text()}")

    def _read_loop(self) -> None:
        out = self._proc.stdout
        try:
            info = json.loads(out.readline() or b"{}")
            if "error" in info or not info:
                self.error = info.get("error", "grabber ended before its info line")
                return
            self.info = info
            self._ready.set()
            while self._read_one(out):
                pass
        except (ValueError, OSError, struct.error) as exc:   # struct.error: the stream ended inside a frame header
            self.error = str(exc)
        finally:
            self._ready.set()

    def _read_one(self, out) -> bool:
        magic = out.read(len(MAGIC))
        if magic != MAGIC:
            if magic:
                self.error = f"stream out of step (got {magic!r} instead of {MAGIC!r})"
            return False
        length, _robot_time, w, h, cropped = struct.unpack(HEADER, out.read(HEADER_SIZE))
        payload = out.read(length)
        if len(payload) != length or length != w * h * 3:
            self.error = f"frame cut short ({len(payload)} of {length} bytes)"
            return False
        frame = np.frombuffer(payload, dtype=np.uint8).reshape(h, w, 3)
        now = time.monotonic()
        with self._lock:
            self._latest, self._received_at, self._cropped = frame, now, bool(cropped)
            self._frames += 1
            self._times.append(now)
            while self._times and now - self._times[0] > RATE_WINDOW_S:
                self._times.popleft()
        return True

    def latest(self) -> tuple:
        """(newest RGB frame or None, its age in seconds on this laptop's clock, or inf)."""
        with self._lock:
            if self._latest is None:
                return None, float("inf")
            return self._latest, time.monotonic() - self._received_at

    def stats(self) -> dict:
        """frames received, measured fps over the last seconds, cropped flag of the newest frame, grabber info, error."""
        with self._lock:
            span = self._times[-1] - self._times[0] if len(self._times) > 1 else 0.0
            fps = (len(self._times) - 1) / span if span > 0 else 0.0
            return {"frames": self._frames, "fps": fps, "cropped": self._cropped, "info": self.info, "error": self.error}

    def close(self) -> None:
        """Stop the grabber (SIGTERM, then wait)."""
        if self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=CLOSE_WAIT_S)
            except subprocess.TimeoutExpired:
                self._proc.kill()
                self._proc.wait()

    def _drain_stderr(self) -> None:
        try:
            for line in self._proc.stderr:
                self._stderr_lines.append(line.decode(errors="replace"))
        except (OSError, ValueError):
            pass  # the pipe closed; what was read so far stays available

    def _stderr_text(self) -> str:
        self._stderr_thread.join(timeout=CLOSE_WAIT_S)  # after close() the process is gone, so the rest arrives quickly
        return "".join(self._stderr_lines).strip()[-STDERR_TAIL_CHARS:]


def grab(cmd, stdin_text=None, count: int = 45, timeout_s: float = 20.0) -> tuple:
    """Collect `count` distinct frames (or what arrives within timeout_s). Returns (frames, stats of the stream)."""
    stream = FrameStream(cmd, stdin_text)
    frames, last = [], None
    deadline = time.monotonic() + timeout_s
    try:
        while len(frames) < count and time.monotonic() < deadline:
            frame, _age = stream.latest()
            if frame is not None and frame is not last:
                frames.append(frame)
                last = frame
            time.sleep(GRAB_POLL_S)
        return frames, stream.stats()
    finally:
        stream.close()


def image_stats(frames) -> dict:
    """Mean and std per RGB channel on 0..1, to compare real light and colours with DATASET_IMAGE_STATS."""
    pixels = np.stack(frames).reshape(-1, 3).astype(np.float64) / 255.0
    return {"mean": tuple(float(v) for v in pixels.mean(axis=0)), "std": tuple(float(v) for v in pixels.std(axis=0))}


def overlay(real, ref):
    """50/50 blend of a real frame and the dataset reference frame: table edge and stand must line up (D455 tilt check)."""
    real, ref = np.asarray(real), np.asarray(ref)
    if real.shape != ref.shape:
        raise ValueError(f"overlay needs equal shapes, got {real.shape} and {ref.shape}")
    return ((real.astype(np.uint16) + ref.astype(np.uint16)) // 2).astype(np.uint8)
