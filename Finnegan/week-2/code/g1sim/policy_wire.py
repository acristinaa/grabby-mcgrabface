"""Level 1 Step 4 - wire protocol between the eval (82, sim env) and the policy server (81, SmolVLA env), plus the run1 guards.

Must stay importable in the SmolVLA env (~/dev/g1-ext/smolvla-test/.venv): stdlib + numpy + g1sim.episode only, no mujoco and
no torch. Transport: plain TCP on localhost, no pickle (STEP4-PLAN decision (1)).
One message = 4-byte big-endian header length + UTF-8 JSON header + the raw C-order bytes of each array listed in
header["arrays"] = [{"name", "dtype", "shape"}, ...], in that order. recv checks every bound before it reads the body: header
<= MAX_HEADER, at most MAX_ARRAYS arrays, dtype uint8 or float32, rank <= MAX_RANK, non-negative int shapes, total <= MAX_BYTES.
Messages:
  server -> client on connect  {"type": "hello", "mode", "chunk_size", "action_dim", "image_shape", "checkpoint", "step",
                                "device", "source"}
  {"type": "reset", "episode", "seed"}                     -> {"type": "ok"}
  {"type": "infer", "tick", "prompt"} + observation arrays -> {"type": "actions", "server_ms", "checks"} + array "actions"
  any problem                                              -> {"type": "error", "message"} (PolicyClient raises PolicyServerError)
Observation keys = episode.POLICY_KEYS + "prompt", the same payload openpi_g1/policy.py reads.
Tick convention: tick = number of actions applied since the handover (frame `tick` = observation before action `tick`).
The run1 guards (training_cmdlines ... checkpoint_problem) keep 81 and 83 off MPS and out of a running training's output dir.
"""
import json
import math
import os
import pathlib
import socket
import struct
import subprocess
import sys
import time
import zlib

import numpy as np

from g1sim import episode

TASK = "put the volleyball in the basket"  # the eval prompt of 82 and 83, equal to 69's first phrasing (pinned by a test)
# Every sentence 69 may record for a demo, TASK first (plan humming-squishing-treasure A5, 2026-11-01): one fixed sentence lets
# a policy ignore the words; a few phrasings of the same task prepare Level 2, where the sentence says which ball. 80 and 82
# --from-handover accept exactly these.
TASKS = (
    TASK,
    "place the volleyball into the basket",
    "pick up the volleyball and put it in the basket",
    "put the ball in the basket",
    "move the ball into the basket",
)
CHUNK = 50
ACTION_DIM = 27
IMAGE_SHAPE = (200, 320, 3)
MAX_BYTES = 16 * 2**20  # all arrays of one message; one 200 x 320 x 3 image is 192 kB
MAX_HEADER = 64 * 2**10
MAX_ARRAYS = 8
MAX_RANK = 4
DTYPES = ("uint8", "float32")  # anything else is refused on send and on receive (no object arrays)
DEFAULT_PORT = 8765
IMAGE_KEY, STATE_KEY = episode.POLICY_KEYS["head_image"], episode.POLICY_KEYS["state"]
CHECKPOINT_FILES = ("config.json", "model.safetensors", "policy_preprocessor.json", "policy_postprocessor.json")


class PolicyServerError(RuntimeError):
    """The server answered with an error message."""


# --- framing ---------------------------------------------------------------------------------------------------------------

def _array_specs(specs):
    """Validate header["arrays"]; returns the total body size in bytes. ValueError on any bound."""
    if not isinstance(specs, list) or len(specs) > MAX_ARRAYS:
        raise ValueError(f"arrays: expected a list of at most {MAX_ARRAYS} entries")
    total = 0
    for spec in specs:
        if not isinstance(spec, dict) or not isinstance(spec.get("name"), str):
            raise ValueError(f"array spec without a name: {spec!r}")
        if spec.get("dtype") not in DTYPES:
            raise ValueError(f"{spec['name']}: dtype {spec.get('dtype')!r} not in {DTYPES}")
        shape = spec.get("shape")
        if (not isinstance(shape, list) or len(shape) > MAX_RANK
                or not all(type(n) is int and 0 <= n <= MAX_BYTES for n in shape)):
            raise ValueError(f"{spec['name']}: shape {shape!r} is not a list of at most {MAX_RANK} ints in 0..{MAX_BYTES}")
        total += math.prod(shape) * np.dtype(spec["dtype"]).itemsize  # Python ints: cannot overflow
        if total > MAX_BYTES:
            raise ValueError(f"arrays larger than {MAX_BYTES} bytes")
    return total


def send(sock, header, arrays=None):
    """One message: header dict (JSON) + arrays (uint8/float32 only, sent as raw C-order bytes)."""
    arrays = {name: np.ascontiguousarray(value) for name, value in (arrays or {}).items()}
    specs = [{"name": name, "dtype": str(value.dtype), "shape": list(value.shape)} for name, value in arrays.items()]
    _array_specs(specs)
    head = json.dumps({**header, "arrays": specs}).encode()
    if len(head) > MAX_HEADER:
        raise ValueError(f"header {len(head)} bytes > {MAX_HEADER}")
    sock.sendall(b"".join([struct.pack(">I", len(head)), head, *(value.tobytes() for value in arrays.values())]))


def _read(sock, n):
    buffer = bytearray(n)
    view, got = memoryview(buffer), 0
    while got < n:
        k = sock.recv_into(view[got:], n - got)
        if k == 0:
            raise ConnectionError(f"connection closed after {got} of {n} bytes")
        got += k
    return buffer


def recv(sock):
    """One message -> (header without "arrays", {name: array}). ConnectionError on EOF, ValueError on any bound (checked before
    the body is read; the stream is out of step after that, so the caller closes the connection)."""
    (length,) = struct.unpack(">I", _read(sock, 4))
    if length > MAX_HEADER:
        raise ValueError(f"header {length} bytes > {MAX_HEADER}")
    try:
        header = json.loads(_read(sock, length).decode())
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError(f"header is not JSON: {error}") from None
    if not isinstance(header, dict):
        raise ValueError("header is not a JSON object")
    specs = header.pop("arrays", [])
    _array_specs(specs)
    arrays = {}
    for spec in specs:
        dtype = np.dtype(spec["dtype"])
        data = _read(sock, int(np.prod(spec["shape"], dtype=np.int64)) * dtype.itemsize)
        arrays[spec["name"]] = np.frombuffer(data, dtype=dtype).reshape(spec["shape"])
    return header, arrays


# --- observation, seeds, replay --------------------------------------------------------------------------------------------

def check_observation(obs):
    """Problems of one policy observation ([] = valid): head image IMAGE_SHAPE uint8, state (27,) float32 finite, prompt text."""
    problems = []
    image, state, prompt = obs.get(IMAGE_KEY), obs.get(STATE_KEY), obs.get("prompt")
    if not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.shape != IMAGE_SHAPE:
        problems.append(f"{IMAGE_KEY}: expected {IMAGE_SHAPE} uint8, got "
                        f"{getattr(image, 'shape', None)} {getattr(image, 'dtype', type(image).__name__)}")
    if not isinstance(state, np.ndarray) or state.dtype != np.float32 or state.shape != (ACTION_DIM,):
        problems.append(f"{STATE_KEY}: expected ({ACTION_DIM},) float32, got "
                        f"{getattr(state, 'shape', None)} {getattr(state, 'dtype', type(state).__name__)}")
    elif not np.isfinite(state).all():
        problems.append(f"{STATE_KEY}: NaN or Inf")
    if not isinstance(prompt, str) or not prompt.strip():
        problems.append("prompt: expected a non-empty string")
    return problems


def noise_seed(stem):
    """Reset seed for a recorded episode: the same stem always gets the same noise, whichever subset is evaluated."""
    return zlib.crc32(stem.encode())


def replay_chunk(actions, tick, chunk=CHUNK):
    """actions[tick:tick+chunk], padded with the last recorded row (the teacher holds its last command)."""
    if tick < 0:
        raise ValueError(f"tick {tick} < 0")
    rows = np.minimum(np.arange(tick, tick + chunk), len(actions) - 1)
    return np.asarray(actions[rows], dtype=np.float32)


def _episode_files(sources):
    if isinstance(sources, (str, os.PathLike)):
        sources = [sources]
    files = {}
    for source in map(pathlib.Path, sources):
        paths = sorted(source.iterdir()) if source.is_dir() else [source]
        for path in paths:
            if path.name.startswith("._") or path.name.endswith(".part") or path.suffix != ".npz":
                continue  # macOS twins on the exFAT T7, half-written temp files
            if path.stem in files:
                raise ValueError(f"episode {path.stem} in two sources: {files[path.stem]} and {path}")
            files[path.stem] = path
    if not files:
        raise ValueError(f"no episodes in {[str(s) for s in sources]}")
    return files


class ReplayPredictor:
    """Serves the recorded actions of one episode, chunk by chunk, and measures how far each observation the client sends is
    from the recorded frame of that tick (`checks`; the thresholds live in g1sim/vla_eval.py). No torch."""

    def __init__(self, sources):
        self.files = _episode_files(sources)
        self.metadata = {"mode": "replay", "chunk_size": CHUNK, "action_dim": ACTION_DIM, "image_shape": list(IMAGE_SHAPE),
                         "checkpoint": None, "step": None, "device": "cpu",
                         "source": [str(s) for s in ([sources] if isinstance(sources, (str, os.PathLike)) else sources)],
                         "episodes": len(self.files)}
        self.ep = None

    def reset(self, episode_stem, seed):
        if episode_stem not in self.files:
            raise ValueError(f"unknown episode {episode_stem!r} ({len(self.files)} episodes served)")
        ep = episode.load(self.files[episode_stem])  # loaded here, not at start: 300 episodes in the training folder
        self.ep = {key: ep[key] for key in ("actions", "state", "head_image", "task")}
        self.stem, self.seed = episode_stem, seed

    def infer(self, obs, tick):
        if self.ep is None:
            raise ValueError("no episode: send reset first")
        chunk = replay_chunk(self.ep["actions"], tick)
        checks = {"recorded_tick": None, "obs_prompt_ok": obs["prompt"] == self.ep["task"]}
        if tick < len(self.ep["actions"]):
            image = np.abs(obs[IMAGE_KEY].astype(np.int16) - self.ep["head_image"][tick].astype(np.int16))
            checks.update(recorded_tick=int(tick),
                          obs_state_err=float(np.max(np.abs(obs[STATE_KEY] - self.ep["state"][tick]))),
                          obs_image_maxdiff=int(image.max()), obs_image_mad=float(image.mean()))
        return chunk, checks


# --- server and client -----------------------------------------------------------------------------------------------------

def _error(conn, message):
    send(conn, {"type": "error", "message": message})


def _handle(conn, predictor, verbose):
    send(conn, {"type": "hello", **predictor.metadata})
    while True:
        try:
            header, arrays = recv(conn)
        except ConnectionError:
            return
        except ValueError as error:  # malformed frame: the stream is out of step, answer and drop the client
            _error(conn, f"bad message: {error}")
            return
        kind = header.get("type")
        try:
            if kind == "reset":
                seed = header.get("seed", 0)
                if type(seed) is not int:
                    raise ValueError(f"seed {seed!r} is not an int")
                predictor.reset(header.get("episode"), seed)
                send(conn, {"type": "ok"})
            elif kind == "infer":
                obs = {**arrays, "prompt": header.get("prompt")}
                problems = check_observation(obs)
                tick = header.get("tick")
                if type(tick) is not int or tick < 0:
                    problems.append(f"tick {tick!r} is not an int >= 0")
                if problems:
                    raise ValueError("; ".join(problems))
                start = time.perf_counter()
                actions, checks = predictor.infer(obs, tick)
                server_ms = (time.perf_counter() - start) * 1000
                send(conn, {"type": "actions", "server_ms": server_ms, "checks": checks},
                     {"actions": np.asarray(actions, dtype=np.float32)})
                if verbose:
                    print(f"tick {tick} server_ms {server_ms:.1f}", flush=True)
            else:
                raise ValueError(f"unknown message type {kind!r}")
        except ValueError as error:
            _error(conn, str(error))
        except RuntimeError as error:  # e.g. from torch inside predictor.infer: report it, keep serving
            _error(conn, f"{type(error).__name__}: {error}")


def serve(predictor, host, port, on_listen=print, verbose=False):
    """Bind, report "LISTENING <port>" through on_listen, then serve one client at a time until the process ends."""
    with socket.create_server((host, port)) as server:
        on_listen(f"LISTENING {server.getsockname()[1]}")
        while True:
            conn, peer = server.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            if verbose:
                print(f"client {peer[0]}:{peer[1]} connected", flush=True)
            try:
                with conn:
                    _handle(conn, predictor, verbose)
            except OSError as error:  # client vanished mid-reply
                if verbose:
                    print(f"client error: {error}", flush=True)
            if verbose:
                print(f"client disconnected; torch loaded: {'torch' in sys.modules}", flush=True)


class PolicyClient:
    """Client side (82). Retries the connect for up to connect_wait_s, so a server that is still loading is waited for."""

    def __init__(self, host, port, timeout_s=120, connect_wait_s=60):
        deadline = time.monotonic() + connect_wait_s
        while True:
            try:
                self.sock = socket.create_connection((host, port), timeout=timeout_s)
                break
            except OSError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(0.1)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.metadata = self._reply("hello")[0]

    def _reply(self, expected):
        header, arrays = recv(self.sock)
        if header.get("type") == "error":
            raise PolicyServerError(header.get("message"))
        if header.get("type") != expected:
            raise PolicyServerError(f"expected {expected!r}, got {header.get('type')!r}")
        return header, arrays

    def reset(self, episode=None, seed=0):
        send(self.sock, {"type": "reset", "episode": episode, "seed": int(seed)})
        self._reply("ok")

    def infer(self, obs, tick):
        start = time.perf_counter()
        send(self.sock, {"type": "infer", "tick": int(tick), "prompt": obs["prompt"]},
             {IMAGE_KEY: obs[IMAGE_KEY], STATE_KEY: obs[STATE_KEY]})
        header, arrays = self._reply("actions")
        return {"actions": arrays["actions"], "server_ms": header["server_ms"],
                "round_trip_ms": (time.perf_counter() - start) * 1000, "checks": header["checks"]}

    def close(self):
        self.sock.close()


# --- run1 guards (no MPS and no reads inside a running training's output dir) ----------------------------------------------

def training_cmdlines():
    """`pgrep -fl lerobot-train` lines (empty if no training runs)."""
    out = subprocess.run(["pgrep", "-fl", "lerobot-train"], capture_output=True, text=True).stdout
    return [line for line in out.splitlines() if line.strip()]


def training_output_dirs(cmdlines):
    """Resolved --output_dir of each training command line; None if any line has none (then nothing counts as outside)."""
    dirs = []
    for line in cmdlines:
        parts = line.split()
        value = next((p.split("=", 1)[1] for p in parts if p.startswith("--output_dir=")), None)
        if value is None and "--output_dir" in parts[:-1]:
            value = parts[parts.index("--output_dir") + 1]
        if not value:
            return None
        dirs.append(pathlib.Path(os.path.realpath(os.path.expanduser(value))))
    return dirs


def checkpoint_ready(ckpt, now, min_age_s=60):
    """None if the checkpoint dir (pretrained_model or its parent) has all files and its newest file is older than min_age_s;
    else the reason it is not ready."""
    ckpt = pathlib.Path(ckpt)
    if (ckpt / "pretrained_model").is_dir():
        ckpt = ckpt / "pretrained_model"
    if not ckpt.is_dir():
        return f"not ready: {ckpt} is not a directory"
    missing = [name for name in CHECKPOINT_FILES if not (ckpt / name).is_file()]
    if missing:
        return f"not ready: {ckpt} misses {', '.join(missing)}"
    newest = max(os.stat(os.path.join(root, name)).st_mtime for root, _, names in os.walk(ckpt) for name in names)
    if now - newest < min_age_s:
        return f"not ready: newest file in {ckpt} is {now - newest:.0f} s old (< {min_age_s} s)"
    return None


def device_problem(device, cmdlines, force_mps):
    """Refusal message for MPS while a training runs on it (unless force_mps), else None."""
    if str(device).startswith("mps") and cmdlines and not force_mps:
        return ("refusing --device mps: lerobot-train is running and owns the GPU (STEP4-PLAN §0) - use --device cpu, "
                "or --force-mps once you are sure")
    return None


def checkpoint_problem(ckpt, cmdlines, allow_during_training, now):
    """Refusal for a checkpoint inside a running training's output dir (or any checkpoint when that dir is unknown), decided
    from the resolved path before any file is read. With allow_during_training: checkpoint_ready(...) instead."""
    if not cmdlines:
        return None
    path = pathlib.Path(os.path.realpath(os.path.expanduser(str(ckpt))))
    dirs = training_output_dirs(cmdlines)
    if dirs is None:
        reason = "lerobot-train is running and its --output_dir could not be read"
    else:
        inside = [d for d in dirs if path == d or d in path.parents]
        if not inside:
            return None
        reason = f"{path} is inside a running training's output dir {inside[0]}"
    if not allow_during_training:
        return f"refusing checkpoint: {reason} - pass --allow-during-training only with the user's OK (R4)"
    return checkpoint_ready(path, now)
