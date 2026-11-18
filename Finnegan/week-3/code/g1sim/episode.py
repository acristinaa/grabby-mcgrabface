"""Rohaufnahme einer Episode (Stufe 2.1) - Zwischenformat zwischen Sim-Umgebung und openpi-Umwandlung.

Eine .npz-Datei je Episode (komprimiert). Nur einfache Datentypen, damit die Sim-Umgebung (numpy 2) schreiben und die
openpi-Umgebung (numpy 1) lesen kann:
  head_image                                       (T, 200, 320, 3) uint8 - nur die Kopfkamera (D455, 16:10)
  state, actions                                   (T, 27) float32, Layout g1sim/action_space.py
  timestamp                                        (T,) float64, Sekunden ab Episodenbeginn
  task (str), fps (int), success (bool)
  handover_state, handover_head_eq_data, handover_json   optional (volleyball demo factory 69): TableEnv.snapshot_handover()
                                                   at the first frame, see handover_arrays / handover_snapshot
openpi_g1/convert.py macht daraus einen LeRobot-Datensatz v2.1.
Dieselben Namen mit Praefix "observation/" sind das Eingangsformat des Policy-Servers (policy_observation, POLICY_KEYS).
"""
import io
import json
import os
import pathlib

import numpy as np

from g1sim import action_space

HANDS = slice(action_space.SLICES["left_hand"].start, action_space.SLICES["right_hand"].stop)  # both hands, 14:26

# Nur die Kopfkamera (Entscheidung D3, 2026-10-02): Der echte G1 hat keine Handgelenkkameras. Die Sim hat sie noch, aber nur
# zum Anschauen (Teleop-Seite, Bilder in 54) - im Datensatz wuerde pi0.5 sonst mit Bildern lernen, die es am Roboter nie gibt.
IMAGE_KEYS = ("head_image",)
HAND_TOLERANCE = 0.05  # normierte Handwerte duerfen durch Ueberschwingen knapp ausserhalb 0..1 liegen

# Schluessel, unter denen die Sim Bilder und Zustand an den Policy-Server schickt (openpi_g1/policy.py liest sie)
POLICY_KEYS = {key: f"observation/{key}" for key in (*IMAGE_KEYS, "state")}


def policy_observation(frame, prompt):
    """Beobachtung (TableEnv.observation() oder ein Frame einer Episode) -> Eingang des Policy-Servers."""
    obs = {POLICY_KEYS[key]: frame[key] for key in IMAGE_KEYS}
    obs[POLICY_KEYS["state"]] = np.asarray(frame["state"], dtype=np.float32)
    obs["prompt"] = prompt
    return obs


class EpisodeRecorder:
    """Sammelt Beobachtung + Aktion je Regeltakt und schreibt die Episode."""

    def __init__(self, task, fps):
        self.task, self.fps = task, int(fps)
        self.frames = {key: [] for key in (*IMAGE_KEYS, "state", "actions", "timestamp")}

    def add(self, obs, actions):
        for key in IMAGE_KEYS:
            self.frames[key].append(np.asarray(obs[key], dtype=np.uint8))
        self.frames["state"].append(np.asarray(obs["state"], dtype=np.float32))
        self.frames["actions"].append(np.asarray(actions, dtype=np.float32))
        self.frames["timestamp"].append(len(self.frames["timestamp"]) / self.fps)

    def __len__(self):
        return len(self.frames["timestamp"])

    def episode(self, success):
        ep = {key: np.stack(values) for key, values in self.frames.items()}
        ep.update(task=np.str_(self.task), fps=np.int64(self.fps), success=np.bool_(success))
        return ep

    def save(self, path, success, extra=None):
        """extra: more arrays for the npz, e.g. handover_arrays(snapshot)."""
        buffer = io.BytesIO()
        np.savez_compressed(buffer, **self.episode(success), **(extra or {}))
        return write_verified(path, buffer.getbuffer())


def write_verified(path, data, tries=3):
    """Write bytes to path so that a file under that name is always complete: temp name (with the process id), fsync, size
    check, rename, size check again. A reader then never sees a half-written or emptied episode (2026-10-14: a 0-byte episode
    on the T7, most likely a second process writing the same file name at the same time). Raises OSError after `tries`
    failed attempts."""
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f"{path.name}.{os.getpid()}.part")
    for _ in range(tries):
        with open(tmp, "wb") as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        if os.path.getsize(tmp) == len(data):
            os.replace(tmp, path)
            if os.path.getsize(path) == len(data):
                return path
    raise OSError(f"{path}: {tries} writes did not leave {len(data)} bytes on disk")


def handover_arrays(snapshot):
    """TableEnv.snapshot_handover() -> npz entries without pickle: the two float arrays as they are, the rest as JSON
    (variation dicts, controller targets and the PCG64 state of the camera noise, whose 128-bit ints JSON keeps exact)."""
    rest = {key: value for key, value in snapshot.items() if key not in ("state", "head_eq_data")}
    plain = json.dumps(rest, default=lambda value: value.tolist() if hasattr(value, "tolist") else str(value))
    return {"handover_state": np.asarray(snapshot["state"], dtype=float),
            "handover_head_eq_data": np.asarray(snapshot["head_eq_data"], dtype=float), "handover_json": np.str_(plain)}


def handover_snapshot(ep):
    """Inverse of handover_arrays: an episode from load() -> the snapshot for TableEnv.restore_handover()."""
    snapshot = json.loads(str(ep["handover_json"]))
    snapshot.update(state=ep["handover_state"], head_eq_data=ep["handover_head_eq_data"])
    return snapshot


def load(path):
    with np.load(path, allow_pickle=False) as f:
        ep = {key: f[key] for key in f.files}
    ep["task"], ep["fps"], ep["success"] = str(ep["task"]), int(ep["fps"]), bool(ep["success"])
    return ep


def validate(ep):
    """Pruefungen aus Masterplan §9.2. Rueckgabe: Liste der Probleme (leer = in Ordnung)."""
    problems = []
    n = len(ep.get("timestamp", []))
    if n == 0:
        return ["keine Frames"]
    for key in (*IMAGE_KEYS, "state", "actions"):
        if key not in ep:
            problems.append(f"{key} fehlt")
        elif len(ep[key]) != n:
            problems.append(f"{key}: {len(ep[key])} Frames statt {n}")
    for key in IMAGE_KEYS:
        if key in ep and (ep[key].dtype != np.uint8 or ep[key].ndim != 4 or ep[key].shape[-1] != 3):
            problems.append(f"{key}: erwartet (T, H, W, 3) uint8, ist {ep[key].shape} {ep[key].dtype}")
    for key in ("state", "actions"):
        if key in ep:
            if ep[key].shape[1:] != (action_space.DIM,):
                problems.append(f"{key}: erwartet (T, {action_space.DIM}), ist {ep[key].shape}")
            elif not np.isfinite(ep[key]).all():
                problems.append(f"{key}: NaN oder Inf")
            else:
                hands = ep[key][:, HANDS]
                if hands.min() < -HAND_TOLERANCE or hands.max() > 1 + HAND_TOLERANCE:
                    problems.append(f"{key}: Handwerte ausserhalb 0..1 ({hands.min():.3f} .. {hands.max():.3f})")
    step = np.diff(ep["timestamp"])
    if len(step) and not np.allclose(step, 1.0 / ep.get("fps", 1), atol=1e-6):
        problems.append("Zeitstempel nicht gleichmaessig im Takt 1/fps (verlorene oder doppelte Frames)")
    if not str(ep.get("task", "")).strip():
        problems.append("kein Aufgabentext")
    return problems
