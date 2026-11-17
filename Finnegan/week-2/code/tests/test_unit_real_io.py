"""Unit: real_io (motor layout, 27-D state, LowState rate, writer samples on the arm and hand command topics).

Fakes only (no DDS, no robot): LowState / hand messages are SimpleNamespaces with the fields real_io reads
(tests/helpers/real_fakes.py). The factory-pose record: test_unit_real_factory_pose.py; 84's checks: test_unit_real_bringup.py.
"""
import gc
import pathlib
import re
import weakref
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("unitree_sdk2py")
pytest.importorskip("cv2")
from g1sim import real_bringup as rb, real_io  # noqa: E402
from helpers.real_fakes import MOTOR_COUNT, FakeClock, fake_hand, fake_lowstate  # noqa: E402

pytestmark = pytest.mark.unit

MJCF = pathlib.Path.home() / "dev" / "g1-ext" / "unitree-g1-mujoco" / "assets" / "g1_29dof_no_hand.xml"
STREAM_HZ = 100.0          # brainco_hand_service's hand state rate (PLAN.md 2.2)


# --- layout and mapping ----------------------------------------------------------------------------------------------------

def test_state27_maps_motors_into_policy_order():
    """Proves: state27 puts motors 15-21, 22-28, left hand, right hand, motor 12 into slots 0-6, 7-13, 14-19, 20-25, 26.
    Catches: swapped arms or the waist taken from motor 13 instead of 12 - the policy would see a mirrored robot."""
    lowstate = fake_lowstate(lambda i: i / 100)
    left, right = fake_hand([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]), fake_hand([0.7, 0.8, 0.9, 1.0, 0.05, 0.15])

    state = real_io.state27(lowstate, left, right)

    expected = [i / 100 for i in range(15, 29)] + [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 0.05, 0.15] + [0.12]
    np.testing.assert_allclose(state, expected, rtol=1e-6)


def test_state27_marks_missing_hands_as_nan():
    """Proves: a missing hand fills its 6 slots with NaN, the other hand keeps its values.
    Catches: a missing hand reported as 0 = "fully open", which the policy would trust."""
    lowstate = fake_lowstate(lambda i: 0.0)

    state = real_io.state27(lowstate, None, fake_hand([0.3] * 6))

    assert np.isnan(state[14:20]).all() and np.allclose(state[20:26], 0.3)


@pytest.mark.skipif(not MJCF.exists(), reason=f"{MJCF} not found (LeRobot G1 sim model)")
def test_joint_limits_match_the_mjcf():
    """Proves: JOINT_LIMITS for motors 12..28 equal the joint ranges of the G1 MJCF the policy was trained with.
    Catches: a typo in a literal limit, which would let 84 pass an out-of-range joint or fail a good one."""
    mujoco = pytest.importorskip("mujoco", reason="mujoco is not installed (the operators' robot env has no MuJoCo)")
    model = mujoco.MjModel.from_xml_path(str(MJCF))

    def mjcf_range(motor):
        name = re.sub(r"(?<!^)([A-Z])", r"_\1", real_io.MOTOR_NAMES[motor]).lower() + "_joint"
        return tuple(model.jnt_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)])

    for motor in real_io.SDK_MOTORS:
        np.testing.assert_allclose(real_io.JOINT_LIMITS[motor], mjcf_range(motor), atol=1e-6, err_msg=f"motor {motor}")


def test_lowstate_hz_check_refuses_below_200_on_robot_and_100_on_loopback():
    """Proves: the refuse bound is 200 Hz on a robot interface and 100 Hz on loopback.
    Catches: the loopback bound used on the robot, which would accept a LowState that is 2-3x too slow."""
    verdicts = [real_io.lowstate_hz_check(iface, hz) for iface, hz in (("en5", 199), ("en5", 200), ("lo0", 99), ("lo0", 100))]

    assert verdicts == ["refuse", "warn", "refuse", "ok"]


def test_rate_hz_of_a_young_reader_is_the_stream_rate(monkeypatch):
    """Proves: a reader that has heard a 100 Hz stream for only 0.2 s reports about 100 Hz, and after 2 s more still about 100.
    Catches: dividing by the whole 1 s window although the reader listened only 0.2 s - 84 printed "slow hand state (4 Hz)"
    for a 78 Hz stream in the push-1 check, which would send the operators after a healthy hand service."""
    clock = FakeClock()
    monkeypatch.setattr(real_io.time, "monotonic", clock.now)
    monkeypatch.setattr(real_io, "init_dds", lambda iface: None)
    monkeypatch.setattr(real_io, "subscribe_topic", lambda topic, msg_type, handler: None)
    reader = real_io.LatestReader("lo0", "rt/brainco/left/state", msg_type=None)

    def hear(seconds):
        for _ in range(int(round(seconds * STREAM_HZ))):
            reader._on_sample(fake_hand([0.0] * 6))
            clock.sleep(1.0 / STREAM_HZ)

    hear(0.2)
    young = reader.rate_hz()
    hear(2.0)
    settled = reader.rate_hz()

    assert young == pytest.approx(STREAM_HZ, rel=0.06) and settled == pytest.approx(STREAM_HZ, rel=0.06)


def test_a_dropped_reader_stays_subscribed(monkeypatch):
    """Proves: a reader its owner drops (84's step_hands does) keeps its DDS channel referenced after a garbage collection.
    Catches: letting Python's cyclic garbage collector free an SDK reader - cyclonedds 0.10.2 then skips dds_delete and its
    receive thread calls a freed callback: the intermittent 84 segfault of the push-1 check
    (runs/real_a_fu_drop_reader_why.log)."""
    class FakeChannel:
        def SetReader(self, qos, handler, queue_len):
            self.handler = handler   # the SDK keeps the handler too, so reader and channel form a reference cycle
    monkeypatch.setattr(real_io, "init_dds", lambda iface: None)
    monkeypatch.setattr(real_io, "ChannelFactory", lambda: SimpleNamespace(CreateChannel=lambda topic, msg_type: FakeChannel()))
    monkeypatch.setattr(real_io, "_open_channels", [])
    reader = real_io.HandReader("lo0", "left")
    channel = weakref.ref(reader._channel)

    del reader
    gc.collect()

    assert channel() is not None


def test_active_writers_counts_samples_not_discovered_writers(monkeypatch):
    """Proves: active_writers reports the number of SAMPLES and the largest weight per topic; a silent topic reports 0.
    Catches: counting writers instead of samples (one writer = 1), which hides how much someone else is sending."""
    sent = {"rt/arm_sdk": [0.0, 0.4, 0.2], "rt/lowcmd": []}

    def fake_subscribe(topic, msg_type, handler):
        for weight in sent[topic]:
            handler(SimpleNamespace(motor_cmd=[SimpleNamespace(q=weight if i == real_io.WEIGHT_SLOT else 0.0)
                                               for i in range(MOTOR_COUNT)]))
    monkeypatch.setattr(real_io, "init_dds", lambda iface: None)
    monkeypatch.setattr(real_io, "subscribe_topic", fake_subscribe)

    writers = real_io.active_writers("lo0", topics=real_io.ARM_CMD_TOPICS, seconds=0.0)

    assert writers == {"rt/arm_sdk": {"samples": 3, "max_weight": 0.4}, "rt/lowcmd": {"samples": 0, "max_weight": None}}


def test_active_writers_listens_on_hand_command_topics_with_their_own_type(monkeypatch):
    """Proves: by default active_writers also counts rt/brainco/<side>/cmd, subscribed as MotorCmds_, with max_weight None.
    Catches: hand topics read as LowCmd_ (a reader of the wrong type silently receives nothing), so an owner's hand tool
    that would fight our hand commands goes unseen (R3-10 (c))."""
    types = {}

    def fake_subscribe(topic, msg_type, handler):
        types[topic] = msg_type
        if topic == "rt/brainco/left/cmd":
            handler(SimpleNamespace(cmds=[SimpleNamespace(q=0.5)] * 6))   # MotorCmds_ has no motor_cmd / weight slot
    monkeypatch.setattr(real_io, "init_dds", lambda iface: None)
    monkeypatch.setattr(real_io, "subscribe_topic", fake_subscribe)

    writers = real_io.active_writers("lo0", seconds=0.0)

    assert writers["rt/brainco/left/cmd"] == {"samples": 1, "max_weight": None}
    assert types["rt/brainco/right/cmd"] is real_io.MotorCmds_ and types["rt/arm_sdk"] is real_io.LowCmd_


def test_yes_sim_refused_off_loopback():
    """Proves: --yes-sim on a robot interface is refused with a message naming loopback.
    Catches: a sim flag that would skip robot checks while the robot cable is plugged in."""
    problem = rb.sim_flags_problem("en5", sim=True, yes_sim=True, net_present=False)

    assert problem is not None and "loopback" in problem
