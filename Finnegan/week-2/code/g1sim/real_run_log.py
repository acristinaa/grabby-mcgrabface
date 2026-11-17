"""Real G1: the log of one 86 run - every message sent, the episode frames, and the sidecar JSON (PLAN.md 3.4).

Why it exists: (1) the sidecar is a SAFETY INPUT, not only a log: 84 reads it (real_io.arms_may_be_held_by_us) to decide
whether our last run may still hold the arms. So it is written twice - once right before the first DDS writer exists
("exit_code": null) and once at exit - always through a temp file + rename, so a kill never leaves half a file (R2-2).
(2) The npz is real robot data for dataset v2, in the g1sim/episode.py layout plus the 50 Hz record of what was sent.
Never overwritten: one <YYYYmmdd_HHMMSS>_<mode> name per run. numpy + stdlib only.
"""
import datetime
import io
import json
import os
import pathlib
import subprocess
import time

import numpy as np

from g1sim import episode, policy_wire

REPO = pathlib.Path(__file__).resolve().parent.parent
LOG_FPS = 30                    # episode frames are one per policy action, as in the dataset (fps 30, PLAN.md 3.4)
NAME_CLASH_WAIT_S = 0.25        # two runs started in the same second: wait for the next stamp instead of overwriting
IMAGE_SHAPE = policy_wire.IMAGE_SHAPE


class RunLog:
    """Collects one run in memory and writes <out>/<stamp>_<mode>.npz + .json."""

    def __init__(self, out_dir, mode: str, source: str, args: dict, gate: dict):
        out = pathlib.Path(out_dir)
        self.out = out if out.is_absolute() else REPO / out
        self.out.mkdir(parents=True, exist_ok=True)
        self.npz_path, self.json_path = self._free_names(mode)
        self.sidecar = {"source": source, "mode": mode, "exit_code": None, "exit_reason": None, "events": {},
                        "args": args, "git": git_info(), "gate": gate, "loaded_files_at_exit": None,
                        "arm_sdk_writer": False, "last_weight_sent": None, "unprinted_messages": [], "npz_error": None,
                        "arm_path": None,
                        "started_at": datetime.datetime.now().isoformat(timespec="seconds"), "log": self.npz_path.name}
        self.sent = {key: [] for key in ("sent_t", "sent_phase", "sent_weight", "sent_q", "sent_kp", "sent_kd", "measured_q")}
        self.frames = {key: [] for key in ("head_image", "state", "actions", "policy_actions")}

    def _free_names(self, mode: str) -> tuple:
        while True:
            stem = f"{time.strftime('%Y%m%d_%H%M%S')}_{mode}"
            npz, sidecar = self.out / f"{stem}.npz", self.out / f"{stem}.json"
            if not npz.exists() and not sidecar.exists():
                return npz, sidecar
            time.sleep(NAME_CLASH_WAIT_S)

    def add_message(self, t: float, phase: str, weight: float, q17, kp17, kd17, measured17) -> None:
        """One rt/arm_sdk message as sent (50 Hz)."""
        for key, value in (("sent_t", t), ("sent_phase", phase), ("sent_weight", weight), ("sent_q", q17), ("sent_kp", kp17),
                           ("sent_kd", kd17), ("measured_q", measured17)):
            self.sent[key].append(value)

    def add_frame(self, image, state27, command27, policy_action27) -> None:
        """One episode frame: observation, the filtered command sent (27-D) and the raw policy action."""
        for key, value in (("head_image", image), ("state", state27), ("actions", command27),
                           ("policy_actions", policy_action27)):
            self.frames[key].append(value)

    def max_policy_lead(self) -> tuple | None:
        """(largest |sent_q - measured_q|, its index in the 17-vector) over the messages of the policy phase at weight 1, or
        None when there was none. Rule 16 (c) needs this M3 number before M4 at the table; the ramp (weight < 1) and the
        scripted moves are left out, because their lead says nothing about how the real arm follows the policy."""
        rows = [i for i, (phase, weight) in enumerate(zip(self.sent["sent_phase"], self.sent["sent_weight"]))
                if phase == "policy" and weight == 1.0]
        if not rows:
            return None
        gap = np.abs(np.asarray(self.sent["sent_q"])[rows] - np.asarray(self.sent["measured_q"])[rows])
        row, joint = np.unravel_index(np.argmax(gap), gap.shape)
        return float(gap[row, joint]), int(joint)

    def note_event(self, name: str, t: float, phase: str | None) -> None:
        """Count an event; the first occurrence keeps its time and phase."""
        event = self.sidecar["events"].setdefault(name, {"count": 0, "first_t": round(t, 3), "first_phase": phase})
        event["count"] += 1

    def note_unprinted(self, text: str) -> None:
        """Keep an operator message that could not be printed (terminal closed, broken pipe), so the log still has it."""
        self.sidecar["unprinted_messages"].append(text)

    def note_arm_path(self, arm_path: dict) -> None:
        """Where the arms are on our waypoint paths (real_run.Runner.log_arm_path), written at once: after a crash `86
        recover` reads it from this sidecar (real_io.last_arm_path) to find the way home (SAFE-PATHS.md S4)."""
        self.sidecar["arm_path"] = {key: _plain(value) for key, value in arm_path.items()}
        self.write_sidecar()

    def mark_writer_coming(self, arm_sdk_writer: bool) -> None:
        """First sidecar write, right BEFORE the first DDS writer is created (R2-2 (ii)): a kill after this point leaves
        "exit_code": null, which 84 reads as "our arms may still be held"."""
        self.sidecar["arm_sdk_writer"] = arm_sdk_writer
        self.write_sidecar()

    def write_sidecar(self) -> None:
        """Write the sidecar atomically (temp file, then rename)."""
        tmp = self.json_path.with_name(f"{self.json_path.name}.{os.getpid()}.part")
        tmp.write_text(json.dumps(self.sidecar, indent=2, default=str))
        os.replace(tmp, self.json_path)

    def finish(self, exit_code: int, reason: str, last_weight_sent: float | None, loaded_files_at_exit: list,
               task: str) -> pathlib.Path:
        """Write the npz, then the final sidecar. Returns the npz path. A failed npz write (disk full) is noted in the
        sidecar instead of raising, so 86 still prints its two last lines and 84 still reads how the run ended."""
        try:
            self._write_npz(task)
        except OSError as error:
            self.sidecar["npz_error"] = f"{type(error).__name__}: {error}"
        self.sidecar.update(exit_code=exit_code, exit_reason=reason, last_weight_sent=last_weight_sent,
                            loaded_files_at_exit=[str(p) for p in loaded_files_at_exit],
                            finished_at=datetime.datetime.now().isoformat(timespec="seconds"))
        self.write_sidecar()
        return self.npz_path

    def _write_npz(self, task: str) -> None:
        buffer = io.BytesIO()
        np.savez_compressed(buffer, **self._episode_arrays(task), **self._sent_arrays())
        episode.write_verified(self.npz_path, buffer.getbuffer())

    def _stack_frames(self, key: str, shape: tuple, dtype) -> np.ndarray:
        """One frame field as an array; an empty (0, *shape) array when the run logged no frame (ready, release)."""
        if not self.frames[key]:
            return np.zeros((0, *shape), dtype)
        return np.asarray(self.frames[key], dtype=dtype)

    def _episode_arrays(self, task: str) -> dict:
        n = len(self.frames["state"])
        return {"head_image": self._stack_frames("head_image", IMAGE_SHAPE, np.uint8),
                "state": self._stack_frames("state", (27,), np.float32),
                "actions": self._stack_frames("actions", (27,), np.float32),
                "policy_actions": self._stack_frames("policy_actions", (27,), np.float32),
                "timestamp": np.arange(n, dtype=np.float64) / LOG_FPS,
                "task": np.str_(task), "fps": np.int64(LOG_FPS), "success": np.bool_(False)}

    def _sent_arrays(self) -> dict:
        n = len(self.sent["sent_t"])
        vectors = {key: (np.asarray(self.sent[key], dtype=np.float64) if n else np.zeros((0, 17)))
                   for key in ("sent_q", "sent_kp", "sent_kd", "measured_q")}
        return {"sent_t": np.asarray(self.sent["sent_t"], dtype=np.float64),
                "sent_phase": np.asarray(self.sent["sent_phase"], dtype=np.str_),
                "sent_weight": np.asarray(self.sent["sent_weight"], dtype=np.float64), **vectors}


def _plain(value):
    """A sidecar value as JSON: poses (arrays, lists of arrays) as nested float lists; names, numbers and None as they are."""
    if value is None or isinstance(value, (str, int)):
        return value
    return np.asarray(value, dtype=float).tolist()


def git_info() -> dict:
    """{"commit", "dirty"} of the checkout 86 runs from (null when git is missing)."""
    def git(*args):
        try:
            result = subprocess.run(["git", "-C", str(REPO), *args], capture_output=True, text=True)
        except OSError:
            return None
        return result.stdout.strip() if result.returncode == 0 else None
    status = git("status", "--porcelain", "--", "g1sim", "scripts")
    return {"commit": git("rev-parse", "HEAD"), "dirty": None if status is None else bool(status)}
