"""Real G1: read-only DDS access - motor layout, 27-D state mapping, LowState/hand readers, writer-sample check, publisher guard.

Why this module exists: the robot day bring-up (84) and the self-test (89) must only LISTEN to the robot. Everything here reads;
nothing here creates a DDS writer. install_publisher_guard() makes any attempt to create one fail loudly, so a mistake in a
read-only script can never send a command to the robot. The factory-pose record lives in g1sim/real_factory_pose.py and is
re-exported here, because PLAN.md section 3.1 names it as part of this module's contract.
Robot env: Python 3.10, numpy, cyclonedds 0.10.2, unitree_sdk2py at 65691c8 (.planning/real-robot/PLAN.md section 2.7).
"""
import atexit
import importlib
import re
import subprocess
import threading
import time
from collections import deque

import numpy as np
from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
from cyclonedds.qos import Policy, Qos
from unitree_sdk2py.core.channel import ChannelFactory, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

from g1sim import brainco
from g1sim.real_factory_pose import (  # noqa: F401 - part of the real_io contract (PLAN.md 3.1)
    BRINGUP_REPORT_DIR, FACTORY_POSE_TOL, REPO, RUN_LOG_DIR, FactoryPoseConflict, arms_may_be_held_by_us,
    earliest_factory_arms_today, factory_arm_summary, factory_pose_path, last_arm_path, load_factory_pose, load_factory_record,
    save_factory_pose)

LOOPBACK = ("lo0", "lo")                    # macOS, Linux loopback interface names
ROBOT_NET_PREFIX = "192.168.123."           # G1 robot network (scripts/72_real_check.py docstring)
SDK_MOTORS = tuple(range(12, 29))           # motors rt/arm_sdk commands: waist 12-14 + arms 15-28 (Unitree arm7 example 40-64)
WAIST_YAW = 12
WAIST_HOLD = (13, 14)                       # waist roll/pitch: held, never steered by the policy (PLAN.md 2.1)
ARM_MOTORS = tuple(range(15, 29))
HAND_MOTOR_COUNT = 6                        # BrainCo Revo2 motors per hand (g1sim/brainco.py MOTORS)
MOTOR_NAMES = (                             # unitree_hg order, equal to 72_real_check.py JOINTS
    "LeftHipPitch", "LeftHipRoll", "LeftHipYaw", "LeftKnee", "LeftAnklePitch", "LeftAnkleRoll",
    "RightHipPitch", "RightHipRoll", "RightHipYaw", "RightKnee", "RightAnklePitch", "RightAnkleRoll",
    "WaistYaw", "WaistRoll", "WaistPitch",
    "LeftShoulderPitch", "LeftShoulderRoll", "LeftShoulderYaw", "LeftElbow", "LeftWristRoll", "LeftWristPitch", "LeftWristYaw",
    "RightShoulderPitch", "RightShoulderRoll", "RightShoulderYaw", "RightElbow", "RightWristRoll", "RightWristPitch",
    "RightWristYaw")
# rad, joint ranges of ~/dev/g1-ext/unitree-g1-mujoco/assets/g1_29dof_no_hand.xml (test-pinned in test_unit_real_io.py)
JOINT_LIMITS = {
    12: (-2.618, 2.618), 13: (-0.52, 0.52), 14: (-0.52, 0.52),
    15: (-3.0892, 2.6704), 16: (-1.5882, 2.2515), 17: (-2.618, 2.618), 18: (-1.0472, 2.0944),
    19: (-1.97222, 1.97222), 20: (-1.61443, 1.61443), 21: (-1.61443, 1.61443),
    22: (-3.0892, 2.6704), 23: (-2.2515, 1.5882), 24: (-2.618, 2.618), 25: (-1.0472, 2.0944),
    26: (-1.97222, 1.97222), 27: (-1.61443, 1.61443), 28: (-1.61443, 1.61443),
}
# Hz: below "refuse" = FAIL, below "warn" = warning. Sim publishes 127.8 Hz on lo0, the robot about 500 Hz (PLAN.md 2.5, review M3)
LOWSTATE_HZ = {"loopback": {"refuse": 100, "warn": 100}, "robot": {"refuse": 200, "warn": 400}}
ARM_CMD_TOPICS = ("rt/arm_sdk", "rt/lowcmd")   # LowCmd_ type; a sample here = someone steers the arms (PLAN.md 3.1)
# MotorCmds_ type; an owner's hand tool would fight our hand commands (PLAN.md 3.1, R3-10 (c))
HAND_CMD_TOPICS = (brainco.TOPIC_CMD.format(side="left"), brainco.TOPIC_CMD.format(side="right"))
WEIGHT_SLOT = 29                            # motor_cmd[29].q carries the arm_sdk weight (Unitree arm7 example line 64)
RATE_WINDOW_S = 2.0                         # receive times kept for rate_hz()
MAX_DISCOVERY_SAMPLES = 10000               # DCPSPublication entries read at once (as 71_real_probe.py)
MIRRORED_SIGN = np.array([1, -1, -1, 1, -1, 1, -1])   # roll and yaw joints change sign between arms (action_space.MIRROR)
# Attributes a DDS writer is built from (PLAN.md 2.6): channel.py imports DataWriter by name, so both bindings are patched
PUBLISHER_ATTRIBUTES = (("unitree_sdk2py.core.channel", "ChannelPublisher"), ("unitree_sdk2py.core.channel", "DataWriter"),
                        ("cyclonedds.pub", "DataWriter"))

_dds_iface = None
# Every reader channel, kept until close_dds(). A dropped SDK channel sits in a reference cycle; when the cyclic garbage collector
# frees it, cyclonedds 0.10.2 skips dds_delete (its weak entity map is cleared first) and the DDS receive thread calls the freed
# Python callback: SIGSEGV (push-1 check, runs/real_a_fu_drop_reader_why.log).
_open_channels = []


class PublisherBlocked(RuntimeError):
    """Raised when read-only code tries to create a DDS writer."""


def is_loopback(iface: str) -> bool:
    """True for the loopback interface, where only our own test publishers and the sim live."""
    return iface in LOOPBACK


def lowstate_hz_check(iface: str, hz: float) -> str:
    """'ok' | 'warn' | 'refuse' for a measured LowState rate. A slow LowState means stale joint angles."""
    bounds = LOWSTATE_HZ["loopback" if is_loopback(iface) else "robot"]
    if hz < bounds["refuse"]:
        return "refuse"
    return "warn" if hz < bounds["warn"] else "ok"


def interface_addresses() -> dict:
    """{interface: [IPv4 addresses]} from `ip` (Linux) or `ifconfig` (macOS); no extra Python package needed."""
    pairs = _ip_addr_pairs() or _ifconfig_pairs()
    addresses = {}
    for name, address in pairs:
        addresses.setdefault(name, []).append(address)
    return addresses


def _ip_addr_pairs() -> list:
    try:
        out = subprocess.run(["ip", "-4", "-o", "addr", "show"], capture_output=True, text=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError):
        return []
    return re.findall(r"^\d+:\s+(\S+)\s+inet\s+([\d.]+)", out, flags=re.M)


def _ifconfig_pairs() -> list:
    out = subprocess.run(["ifconfig", "-a"], capture_output=True, text=True, check=True).stdout
    pairs, name = [], None
    for line in out.splitlines():
        if line and not line[0].isspace():
            name = line.split(":")[0]
        match = re.match(r"\s+inet\s+([\d.]+)", line)
        if match and name:
            pairs.append((name, match.group(1)))
    return pairs


def robot_net_present() -> bool:
    """True when any interface has a 192.168.123.x address, i.e. a robot cable may be plugged in."""
    return any(a.startswith(ROBOT_NET_PREFIX) for addresses in interface_addresses().values() for a in addresses)


def state27(lowstate, hand_left, hand_right) -> np.ndarray:
    """LowState + both hand states -> the 27-D policy state (g1sim/action_space.py). A missing hand gives NaN in its slots,
    so nobody mistakes "no data" for "hand open"."""
    q = [float(lowstate.motor_state[m].q) for m in ARM_MOTORS]
    hands = [_hand_q(hand) for hand in (hand_left, hand_right)]
    return np.array([*q, *hands[0], *hands[1], float(lowstate.motor_state[WAIST_YAW].q)], dtype=np.float32)


def _hand_q(hand_state) -> list:
    if hand_state is None:
        return [float("nan")] * HAND_MOTOR_COUNT
    return [float(m.q) for m in hand_state.states[:HAND_MOTOR_COUNT]]


def sdk_q(lowstate) -> np.ndarray:
    """Measured angles of motors 12..28 (the 17 motors rt/arm_sdk addresses), in that order."""
    return np.array([float(lowstate.motor_state[m].q) for m in SDK_MOTORS])


def joint_rows(lowstate) -> list:
    """One row per motor for the 84 joint table: index, name, angle, limits (motors 12..28 only), inside, temperature."""
    rows = []
    for index, name in enumerate(MOTOR_NAMES):
        motor = lowstate.motor_state[index]
        lo, hi = JOINT_LIMITS.get(index, (None, None))
        q = float(motor.q)
        temperature = motor.temperature if hasattr(motor.temperature, "__len__") else [motor.temperature]
        rows.append({"index": index, "name": name, "q": q, "limit_lo": lo, "limit_hi": hi,
                     "inside": lo is None or lo <= q <= hi, "temperature": [int(t) for t in temperature]})
    return rows


def mirror_gap(state: np.ndarray) -> np.ndarray:
    """Left arm minus the mirrored right arm (7 values, rad). Near 0 for a symmetric pose; a sign flip shows up as 2x the angle."""
    state = np.asarray(state)
    return state[0:7] - MIRRORED_SIGN * state[7:14]


def init_dds(iface: str) -> None:
    """Start DDS on one interface, once per process. The SDK silently keeps the first interface, so a second one is refused."""
    global _dds_iface
    if _dds_iface is None:
        ChannelFactoryInitialize(0, iface)
        _dds_iface = iface
        atexit.register(close_dds)   # runs before interpreter teardown, while Python can still take a DDS callback
    elif _dds_iface != iface:
        raise RuntimeError(f"DDS already runs on {_dds_iface}; restart the script to use {iface}")


def subscribe_topic(topic: str, msg_type, handler) -> object:
    """Reader with KEEP_LAST 1 QoS (only the newest sample matters; review r5-3). The channel stays open until close_dds(),
    even when the caller drops it (see _open_channels: a garbage-collected reader crashes the process)."""
    channel = ChannelFactory().CreateChannel(topic, msg_type)
    channel.SetReader(Qos(Policy.History.KeepLast(1)), handler, 0)
    _open_channels.append(channel)
    return channel


def close_dds() -> None:
    """Delete the SDK's participant and with it every reader while Python is fully alive (atexit, from init_dds). Otherwise
    teardown frees the readers through the garbage collector and crashes after the report is written (exit 139, not 0/1).
    Entity.__del__ (dds_delete + forget the handle) is cyclonedds 0.10.2's only close; it waits for a running callback."""
    participant = dds_participant()
    if participant is not None:
        participant.__del__()


class LatestReader:
    """Keeps the newest sample of one topic and its receive times, for age and rate checks."""

    def __init__(self, iface: str, topic: str, msg_type):
        init_dds(iface)
        self.topic = topic
        self._lock = threading.Lock()
        self._latest, self._received_at = None, None
        self._times = deque()   # receive times of the last RATE_WINDOW_S seconds
        self._channel = subscribe_topic(topic, msg_type, self._on_sample)

    def _on_sample(self, msg) -> None:
        now = time.monotonic()
        with self._lock:
            self._latest, self._received_at = msg, now
            self._times.append(now)
            while self._times and now - self._times[0] > RATE_WINDOW_S:
                self._times.popleft()

    def latest(self) -> tuple:
        """(newest message or None, its age in seconds or inf)."""
        with self._lock:
            if self._latest is None:
                return None, float("inf")
            return self._latest, time.monotonic() - self._received_at

    def rate_hz(self, window_s: float = 1.0) -> float:
        """Samples per second over the last window_s seconds (at most RATE_WINDOW_S): gaps between the samples in the window
        / time from the first of them until now, so a reader younger than window_s is not under-counted (push-1 check: 84
        said 4 Hz for a 78 Hz hand stream). Fewer than 2 samples give 0; a stream that stopped reads lower and lower."""
        now = time.monotonic()
        with self._lock:
            recent = [t for t in self._times if now - t <= window_s]
        if len(recent) < 2:
            return 0.0
        return (len(recent) - 1) / (now - recent[0])


class LowStateReader(LatestReader):
    """rt/lowstate: joint angles and IMU of the G1 (unitree_hg, not unitree_go - CLAUDE.md macOS notes)."""

    def __init__(self, iface: str):
        super().__init__(iface, "rt/lowstate", LowState_)


class HandReader(LatestReader):
    """rt/brainco/<side>/state from brainco_hand_service on PC2 (q 0 = open .. 1 = closed, PLAN.md 2.2)."""

    def __init__(self, iface: str, side: str):
        super().__init__(iface, brainco.TOPIC_STATE.format(side=side), MotorStates_)


def command_msg_type(topic: str):
    """The message type of a command topic. A reader with the wrong type silently receives nothing, so an unknown topic
    is refused instead of guessed."""
    if topic in ARM_CMD_TOPICS:
        return LowCmd_
    if topic in HAND_CMD_TOPICS:
        return MotorCmds_
    raise ValueError(f"{topic} is not a known command topic; use one of {ARM_CMD_TOPICS + HAND_CMD_TOPICS}")


class SampleCounter:
    """Counts the SAMPLES that arrive on a command topic. A discovered writer that never sends is harmless; a sample on
    rt/arm_sdk means someone else steers the arms right now (PLAN.md 4.1 rule 2, review M4). Hand topics carry no
    arm_sdk weight, so their max_weight stays None."""

    def __init__(self, topic: str):
        self.topic, self.samples, self.max_weight = topic, 0, None
        self._reads_weight = topic in ARM_CMD_TOPICS
        self._lock = threading.Lock()
        self._channel = subscribe_topic(topic, command_msg_type(topic), self._on_sample)

    def _on_sample(self, msg) -> None:
        with self._lock:
            self.samples += 1
            if self._reads_weight:
                weight = float(msg.motor_cmd[WEIGHT_SLOT].q)
                self.max_weight = weight if self.max_weight is None else max(self.max_weight, weight)

    def summary(self) -> dict:
        """{"samples": int, "max_weight": float | None}."""
        with self._lock:
            return {"samples": self.samples, "max_weight": self.max_weight}


def active_writers(iface: str, topics=ARM_CMD_TOPICS + HAND_CMD_TOPICS, seconds: float = 2.0) -> dict:
    """Listen `seconds` on each command topic: {topic: {"samples", "max_weight"}}. Counts samples, not discovered writers."""
    init_dds(iface)
    counters = [SampleCounter(topic) for topic in topics]
    time.sleep(seconds)
    return {counter.topic: counter.summary() for counter in counters}


def dds_participant():
    """The SDK's DomainParticipant. The SDK keeps it private, so it is read by its mangled name (as 71_real_probe.py does)."""
    return ChannelFactory()._ChannelFactory__participant


def own_writers(participant) -> int:
    """Number of DDS writers this participant owns (DCPSPublication entries with its key). Must be 0 at the end of every 84 run."""
    discovery = BuiltinDataReader(participant, BuiltinTopicDcpsPublication)
    return sum(1 for s in discovery.take(N=MAX_DISCOVERY_SAMPLES) if s.participant_key == participant.guid)


def _blocked_publisher(*args, **kwargs):
    raise PublisherBlocked("this script is read-only and may not create a DDS writer. Do not work around this: "
                           "commands to the robot only go through 86_real_run.py (runbook Part C)")


def install_publisher_guard() -> None:
    """Replace every class a DDS writer is built from with one that raises (PLAN.md 2.6, 4.1 rule 1 (c)). Idempotent."""
    for module_name, attribute in PUBLISHER_ATTRIBUTES:
        setattr(importlib.import_module(module_name), attribute, _blocked_publisher)


def publisher_guard_installed() -> bool:
    """True when every writer class is replaced by the guard."""
    return all(getattr(importlib.import_module(m), a) is _blocked_publisher for m, a in PUBLISHER_ATTRIBUTES)
