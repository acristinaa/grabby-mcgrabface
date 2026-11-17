"""Fakes for the 86 runner tests (tests/test_unit_real_run.py): a simulated arm behind LowState and rt/arm_sdk, hand fakes,
camera, policy client, clock and signals. No DDS, no robot: g1sim/real_run.RunIO takes these instead of the real readers.

The fake arm follows the weighted blend the plan assumes for Unitree's controller (PLAN.md section 1): measured =
weight * our target + (1 - weight) * its hold pose, instantly. It can sag (while weight < 1), get stuck (contact, optionally
pushing some joints off by `stuck_drift`), or stop publishing (stale LowState) - each from `*_after` on, until `*_until`
(None = for good).
"""
import datetime
import json
import types

import numpy as np
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_

from g1sim import action_space, policy_wire, real_run, real_safety as rs, real_send

# A plausible factory hold pose (17-vector, motors 12..28): forearms angled down, waist roll/pitch slightly off 0 so a
# command that forgets to hold them is visible
FACTORY = np.array([0.0, 0.03, -0.02,
                    0.2, 0.2, 0.0, 0.9, 0.0, 0.0, 0.0,
                    0.2, -0.2, 0.0, 0.9, 0.0, 0.0, 0.0])
START_T = 1000.0                 # fake monotonic clock start


class FakeClock:
    """Monotonic time that only moves when the runner sleeps."""

    def __init__(self):
        self.t = START_T

    def __call__(self) -> float:
        return self.t

    def sleep(self, seconds: float) -> None:
        self.t += seconds


class FakeArm:
    """LowState reader AND arm_sdk sender of one simulated arm. Records every message as the DDS sample it would be."""

    def __init__(self, clock: FakeClock, q17=FACTORY):
        self.clock, self.hold_pose, self.q = clock, np.array(q17, float), np.array(q17, float)
        self.sag_per_read, self.sag = 0.0, 0.0
        self.stuck_after = self.stale_after = None
        self.stuck_until = self.stale_until = None   # the contact clears / LowState comes back (None: never)
        self.stuck_drift = np.zeros(17)   # added to the measured angles once stuck (a joint pushed off by the contact)
        self.stale_age = 0.5          # between the 0.1 s limit and 1.0 s, so a loosened limit is caught
        self.messages = []            # dicts: weight, q (17, from the filled LowCmd), kp13_14
        self.weight = 0.0
        self.on_create = None

    def latest(self):
        if self.weight < 1.0:
            self.sag += self.sag_per_read     # an arm with low weight sags under gravity
        stale = _active(self.clock(), self.stale_after, self.stale_until)
        return self.lowstate(), (self.stale_age if stale else 0.0)

    def lowstate(self):
        motors = [types.SimpleNamespace(q=0.0) for _ in range(35)]
        measured = self.q.copy()
        measured[3] -= self.sag              # left shoulder pitch drifts
        if _active(self.clock(), self.stuck_after, self.stuck_until):
            measured += self.stuck_drift
        for i, motor in enumerate(rs.SDK_MOTORS):
            motors[motor].q = float(measured[i])
        return types.SimpleNamespace(motor_state=motors)

    def rate_hz(self, window_s: float) -> float:
        return 500.0                          # robot rate (PLAN.md 2.5)

    def make_sender(self):
        if self.on_create:
            self.on_create()
        return self

    def send(self, weight, q17, kp17, kd17) -> None:
        cmd = real_send.fill_arm_sdk(unitree_hg_msg_dds__LowCmd_(), weight, q17, kp17, kd17)
        self.messages.append({"weight": cmd.motor_cmd[real_send.WEIGHT_SLOT].q,
                              "q": np.array([cmd.motor_cmd[m].q for m in rs.SDK_MOTORS]),
                              "kp13_14": (cmd.motor_cmd[13].kp, cmd.motor_cmd[14].kp)})
        self.weight = weight
        if not _active(self.clock(), self.stuck_after, self.stuck_until):
            self.q = weight * np.asarray(q17) + (1 - weight) * self.hold_pose


def _active(now: float, after: float | None, until: float | None) -> bool:
    """A fake fault is on from `after` (None: never) until `until` (None: for good)."""
    return after is not None and now >= after and (until is None or now < until)


class FakeHand:
    """Hand-state reader and hand sender in one; state None = brainco_hand_service not running. With a clock and
    `stale_after`, its newest state is `stale_age` old from then on."""

    def __init__(self, q=0.1, present=True, clock=None):
        self.state = types.SimpleNamespace(states=[types.SimpleNamespace(q=q) for _ in range(6)]) if present else None
        self.sent = []
        self.clock, self.stale_after = clock, None
        self.stale_age = 0.9          # above the 0.5 s hand-state limit (real_timing.STALE_S)

    def latest(self):
        if self.state is None:
            return None, float("inf")
        stale = self.stale_after is not None and self.clock() >= self.stale_after
        return self.state, (self.stale_age if stale else 0.0)

    def send(self, targets6, speed) -> None:
        real_send.fill_hand_cmd(types.SimpleNamespace(), targets6, speed)   # the real range checks
        self.sent.append((np.array(targets6), speed))


class FakeFrames:
    """Camera stream whose newest frame is always `age_s` old."""

    def __init__(self, age_s=0.0):
        self.age_s = age_s

    def latest(self):
        return np.zeros(policy_wire.IMAGE_SHAPE, np.uint8), self.age_s


class FakePolicy:
    """Policy client that serves rows of `actions` from the requested tick on (like 81's replay)."""

    def __init__(self, actions, mode="replay"):
        self.actions = np.asarray(actions, np.float32)
        self.metadata = {"mode": mode, "chunk_size": 50, "action_dim": 27, "image_shape": [200, 320, 3],
                         "checkpoint": None if mode == "replay" else "/fake/040000/pretrained_model"}
        self.ticks, self.states, self.resets = [], [], []

    def reset(self, episode=None, seed=0):
        self.resets.append((episode, seed))

    def infer(self, obs, tick):
        self.ticks.append(tick)
        self.states.append(np.array(obs[policy_wire.STATE_KEY]))
        return {"actions": policy_wire.replay_chunk(self.actions, tick)}


class InlineWorker:
    """Runs each policy request at once, so the tests are deterministic (the real worker is a thread)."""

    def __init__(self):
        self.result = None

    def submit(self, job) -> None:
        self.result = ("ok", job())

    def poll(self):
        result, self.result = self.result, None
        return result


def signals_at(clock: FakeClock, *times: float):
    """signal_count() that reports one more stop signal at each of `times` (seconds after START_T)."""
    return lambda: sum(clock() >= START_T + t for t in times)


def factory_record(q17=FACTORY) -> dict:
    now = datetime.datetime.now()
    return {"q": [float(v) for v in q17], "iface": "en5", "mode_note": "R2+A (owner)",
            "recorded_at": now.replace(hour=9, minute=5).isoformat(timespec="seconds"), "date": now.date().isoformat()}


def demo_actions(rows=300, shoulder_pitch=-0.25) -> np.ndarray:
    """Policy actions: the ready pose with both shoulder pitches at `shoulder_pitch` (inside the demo envelope)."""
    actions = np.tile(action_space.BALL_READY_POSE, (rows, 1))
    actions[:, [0, 7]] = shoulder_pitch
    return actions


def make_io(clock: FakeClock, arm: FakeArm, hands=None, **overrides) -> real_run.RunIO:
    """RunIO with every fake wired in; overrides replace single fields."""
    hands = {"left": FakeHand(), "right": FakeHand()} if hands is None else hands
    fields = dict(lowstate=arm, hands=hands, frames=None, make_arm_sender=arm.make_sender,
                  make_hand_senders=lambda: hands, policy=None,
                  foreign_samples=lambda: {"rt/arm_sdk": {"samples": 0, "max_weight": None},
                                           "rt/lowcmd": {"samples": 0, "max_weight": None}},
                  factory_record=factory_record(arm.hold_pose), clock=clock, sleep=clock.sleep, make_worker=InlineWorker)
    fields.update(overrides)
    return real_run.RunIO(**fields)


def run(mode: str, io: real_run.RunIO, out_dir, iface="lo0", **args) -> real_run.RunResult:
    """Run one 86 mode with the fakes; logs go to `out_dir` (a test's tmp_path)."""
    return real_run.run_mode(mode, io, real_run.RunArgs(iface=iface, out=out_dir, **args))


def sent(result: real_run.RunResult, key: str) -> np.ndarray:
    """One array of the run's npz log, e.g. "sent_q", "sent_phase", "sent_weight"."""
    with np.load(result.log_path) as log:
        return log[key]


def read_sidecar(result: real_run.RunResult) -> dict:
    """The run's final sidecar JSON."""
    return json.loads(result.log_path.with_suffix(".json").read_text())


def last_index(phases, phase: str) -> int:
    """Index of the last message whose phase is `phase`."""
    return max(i for i, name in enumerate(phases) if name == phase)
