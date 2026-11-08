"""Real G1: chunk timing and staleness rules of the 86 runner - pure logic, re-exported by g1sim/real_safety.py (PLAN.md 3.3).

Why it exists: the policy answers with 50-action chunks at 30 Hz while 86 sends at 50 Hz, and the answer arrives late
(network, inference). The scheduler plays each chunk from the action that matches "now" (latency skip) and refuses a chunk
that is entirely in the past. stale_reason() stops a run when an input is too old to steer the arm on (rule 10).
"""
import math

import numpy as np

SEND_HZ = 50                            # Unitree example rate, line 69
# seconds; lowstate -> fault exit, the others -> soft exit (rule 10)
STALE_S = {"lowstate": 0.1, "camera": 1.0, "hand_state": 0.5, "chunk_ran_out": 1.0}
POLICY_TIMEOUT_S = 2.0                  # policy error or socket timeout -> soft exit (rule 10)
CAMERA_HAND_MODES = ("shadow", "run")   # camera and hand-state staleness stop only these modes (rule 10; replay only warns)
CHUNK_SIZE = 50                         # actions per policy chunk (policy_wire.CHUNK; 86 refuses another server hello)
EXEC_STEPS = 25                         # actions of a chunk played before the next request goes out (rule 14)
ACTION_HZ = 30                          # policy actions per second at speed 1.0 (dataset fps 30, rule 14)


class InvalidTarget(ValueError):
    """A target contains NaN or Inf; the runner turns it into a fault exit before anything is sent (rule 11)."""


class ChunkTooLate(RuntimeError):
    """A chunk arrived after all its 50 actions were already in the past; the runner makes a soft exit (rule 14)."""


def require_finite(values, what: str) -> np.ndarray:
    """values as a float array; raises InvalidTarget naming `what` if any entry is NaN or Inf."""
    array = np.asarray(values, dtype=float)
    if not np.all(np.isfinite(array)):
        raise InvalidTarget(f"{what} contains NaN or Inf - the policy output is broken; this run stops (fault exit)")
    return array


class ChunkScheduler:
    """Plays policy chunks at 30 Hz x speed and tells the runner when to ask for the next one (rule 14).

    `position` counts actions since the policy phase began. Action j of a chunk requested at position p plays at p + j, so
    the actions that passed while the server was thinking are skipped instead of replayed late.
    """

    def __init__(self, speed: float, chunk_size: int = CHUNK_SIZE, exec_steps: int = EXEC_STEPS,
                 action_hz: int = ACTION_HZ):
        self.speed, self.chunk_size, self.exec_steps, self.action_hz = speed, chunk_size, exec_steps, action_hz
        self.position = 0.0
        self._chunk = None
        self._start = 0.0
        self._pending = False

    def advance(self, dt: float) -> None:
        """Move the clock on by dt seconds of wall time."""
        self.position += dt * self.action_hz * self.speed

    def add_chunk(self, chunk, requested_at: float) -> None:
        """Make `chunk` (50, 27) the current one; raises InvalidTarget on NaN, ChunkTooLate when it lies fully in the past."""
        chunk = require_finite(chunk, "policy chunk")
        if chunk.shape != (self.chunk_size, 27):
            raise ValueError(f"policy chunk has shape {chunk.shape}, expected ({self.chunk_size}, 27) - wrong server?")
        self._pending = False
        if self.position - requested_at >= self.chunk_size:
            raise ChunkTooLate(f"chunk arrived {self.position - requested_at:.1f} actions after its request "
                               f"(all {self.chunk_size} in the past) - the policy server or network is too slow")
        self._chunk, self._start = chunk, requested_at

    def target(self) -> np.ndarray | None:
        """27-D target at the current position (linear between actions); None before the first chunk or after it ran out."""
        if self._chunk is None or self._offset() > self.chunk_size - 1:
            return None
        offset = self._offset()
        lower = math.floor(offset)
        upper = min(lower + 1, self.chunk_size - 1)
        fraction = offset - lower
        return (1 - fraction) * self._chunk[lower] + fraction * self._chunk[upper]

    def needs_request(self) -> bool:
        """True when no request is pending and the current chunk is >= exec_steps actions in (or there is none yet)."""
        if self._pending:
            return False
        return self._chunk is None or self._offset() >= self.exec_steps

    def mark_requested(self) -> float:
        """Note that a request goes out now; returns the position to pass back as requested_at (its int is the tick)."""
        self._pending = True
        return self.position

    def seconds_since_ran_out(self) -> float | None:
        """Wall-clock seconds since the current chunk's last action played, or None while it still plays."""
        if self._chunk is None:
            return None
        over = self._offset() - (self.chunk_size - 1)
        return over / (self.action_hz * self.speed) if over > 0 else None

    def _offset(self) -> float:
        return self.position - self._start


def stale_reason(lowstate_age: float | None, camera_age: float | None, hand_age: float | None,
                 ran_out_s: float | None, mode: str) -> str | None:
    """The first input too old to steer on, or None. Ages in seconds; None = input not used (no camera, --no-hands,
    no policy) - except LowState, where None means no message yet and counts as stale."""
    if lowstate_age is None or lowstate_age > STALE_S["lowstate"]:
        return "lowstate_stale"
    if mode in CAMERA_HAND_MODES:
        if camera_age is not None and camera_age > STALE_S["camera"]:
            return "camera_stale"
        if hand_age is not None and hand_age > STALE_S["hand_state"]:
            return "hand_state_stale"
    if ran_out_s is not None and ran_out_s > STALE_S["chunk_ran_out"]:
        return "policy_stale"
    return None
