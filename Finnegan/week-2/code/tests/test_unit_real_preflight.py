"""Unit: the preflight of 86 - every refusal and warning before 86 creates its first writer (g1sim/real_preflight.py, rule 2).

Each test drives the real runner with the fakes of tests/helpers/fake_real_run.py and checks that a refused run created no
writer and sent nothing. No DDS, no robot, seconds.
"""
import json
import time

import numpy as np
import pytest

pytest.importorskip("unitree_sdk2py")

from g1sim import real_safety as rs  # noqa: E402
from g1sim import real_io  # noqa: E402
from helpers.fake_real_run import (FACTORY, FakeArm, FakeClock, FakeFrames, FakeHand, FakePolicy, demo_actions,  # noqa: E402
                                   factory_record, make_io, read_sidecar, run)

pytestmark = pytest.mark.unit

ELBOW_POS = 6                                  # left elbow (motor 18, limits -1.0472..2.0944) in a 17-vector
NO_FOREIGN = {"samples": 0, "max_weight": None}


@pytest.fixture(autouse=True)
def run_log_dir(tmp_path, monkeypatch):
    """Refusals of robot-interface runs log to RUN_LOG_DIR; point it into tmp_path so no test writes into runs/real."""
    path = tmp_path / "runs_real"
    monkeypatch.setattr(real_io, "RUN_LOG_DIR", path)
    return path


def io_recording_writers(clock, arm, created, **overrides):
    """make_io whose sender factories also note in `created` that a writer was built."""
    def make_arm_sender():
        created.append("arm")
        return arm
    def make_hand_senders():
        created.append("hands")
        return {}
    return make_io(clock, arm, make_arm_sender=make_arm_sender, make_hand_senders=make_hand_senders, **overrides)


def stale_lowstate(clock):
    arm = FakeArm(clock)
    arm.stale_after = clock()                  # the newest LowState is already 0.5 s old
    return "ready", arm, {}, {}


def joint_outside_limits(clock):
    q = FACTORY.copy()
    q[ELBOW_POS] = 2.5                         # above the elbow's 2.0944 limit: a wrong reading or a broken joint
    return "ready", FakeArm(clock, q), {}, {}


def replay_server_for_run(clock):
    return "run", FakeArm(clock), {"frames": FakeFrames(), "policy": FakePolicy(demo_actions(), mode="replay")}, {}


def stale_camera_for_shadow(clock):
    policy = FakePolicy(demo_actions(), mode="smolvla")
    return "shadow", FakeArm(clock), {"frames": FakeFrames(age_s=1.0), "policy": policy}, {}


def wrong_chunk_size_in_hello(clock):
    policy = FakePolicy(demo_actions(), mode="smolvla")
    policy.metadata["chunk_size"] = 16          # another server or version: 86's scheduler assumes 50
    return "run", FakeArm(clock), {"frames": FakeFrames(), "policy": policy}, {}


def no_hand_state_for_run(clock):
    policy = FakePolicy(demo_actions(), mode="smolvla")
    hands = {"left": FakeHand(present=False), "right": FakeHand(present=False)}   # brainco_hand_service not running
    return "run", FakeArm(clock), {"frames": FakeFrames(), "policy": policy, "hands": hands}, {}


def speed_above_robot_limit(clock):
    policy = FakePolicy(demo_actions(), mode="smolvla")
    return "run", FakeArm(clock), {"frames": FakeFrames(), "policy": policy}, {"iface": "en5", "speed": 0.8}


def waist_max_above_ceiling_on_robot(clock):
    policy = FakePolicy(demo_actions(), mode="smolvla")
    robot_run = {"iface": "en5", "out": real_io.RUN_LOG_DIR, "waist_max": 0.9}   # the runbook's --out: only waist refuses
    return "run", FakeArm(clock), {"frames": FakeFrames(), "policy": policy}, robot_run


def checkpoint_server_for_replay(clock):
    return "replay", FakeArm(clock), {"policy": FakePolicy(demo_actions(), mode="smolvla")}, {}


def no_factory_record_for_ready(clock):
    return "ready", FakeArm(clock), {"factory_record": None}, {}


@pytest.mark.parametrize("case, expected", [
    pytest.param(stale_lowstate, "no fresh rt/lowstate", id="stale_lowstate"),
    pytest.param(joint_outside_limits, "outside its limits", id="joint_outside_limits"),
    pytest.param(replay_server_for_run, "needs a checkpoint server", id="replay_server_for_run"),
    pytest.param(stale_camera_for_shadow, "no camera frame younger than", id="stale_camera_for_shadow"),
    pytest.param(wrong_chunk_size_in_hello, "policy server sends chunk_size = 16", id="wrong_chunk_size_in_hello"),
    pytest.param(no_hand_state_for_run, "no hand state from left, right", id="no_hand_state_for_run"),
    pytest.param(speed_above_robot_limit, f"above {rs.ROBOT_MAX_SPEED_FACTOR} on the robot", id="speed_above_robot_limit"),
    pytest.param(waist_max_above_ceiling_on_robot, f"above the robot ceiling {rs.ROBOT_WAIST_MAX_CEILING}",
                 id="waist_max_above_ceiling_on_robot"),
    pytest.param(checkpoint_server_for_replay, "86 replay needs a replay server", id="checkpoint_server_for_replay"),
    pytest.param(no_factory_record_for_ready, "record the factory pose first", id="no_factory_record_for_ready"),
])
def test_preflight_refuses_before_any_writer(tmp_path, case, expected):
    """Proves: each rule-2 problem refuses the run (exit 2) with its own message, and no writer is ever created.
    Catches: a preflight check dropped from run_preflight or from its call site, so 86 would start the ramp-in on stale
    angles, a broken joint, a swapped or wrong-format server, a blind camera, no hand state, a too-fast speed, a waist
    turning the torso past the 0.6 rad ceiling, or without a factory record that a later `86 release` needs
    [mutation, one per case]."""
    clock = FakeClock()
    mode, arm, io_overrides, args = case(clock)
    created = []
    io = io_recording_writers(clock, arm, created, **io_overrides)

    result = run(mode, io, args.pop("out", tmp_path), **args)

    assert result.exit_code == 2 and expected in result.reason, result.reason
    assert created == [] and arm.messages == []


def test_out_dir_outside_run_log_dir_refused_off_loopback(tmp_path, run_log_dir):
    """Proves: a robot run (iface en5) with --out outside runs/real is refused (exit 2) before any writer exists, and its
    refusal log goes to runs/real, not into the refused folder.
    Catches: a robot run whose sidecar 84 never reads, so a killed run's held arms would look free (R3-5) [mutation]."""
    refused = tmp_path / "elsewhere"
    clock = FakeClock()
    arm = FakeArm(clock)
    created = []

    result = run("ready", io_recording_writers(clock, arm, created), refused, iface="en5")

    assert result.exit_code == 2 and "robot runs log only to runs/real" in result.reason
    assert created == [] and arm.messages == []
    assert result.log_path.parent == run_log_dir and not refused.exists()


def test_foreign_hand_commands_warn_and_are_logged(tmp_path, capsys):
    """Proves: samples from another program on rt/brainco/left/cmd print the plan's warning and log foreign_hand_cmd,
    while the run itself goes on (rule 2: warn, not refuse).
    Catches: an owner's hand tool and our 50 Hz hand commands fighting over the fingers without any message (R3-10 (c))
    [mutation]."""
    clock = FakeClock()
    samples = {"rt/arm_sdk": NO_FOREIGN, "rt/lowcmd": NO_FOREIGN, "rt/brainco/left/cmd": {"samples": 40, "max_weight": None}}

    result = run("ready", make_io(clock, FakeArm(clock), foreign_samples=lambda: samples), tmp_path, hold=0.2)

    assert result.exit_code == 0
    assert "another program sends hand commands" in capsys.readouterr().out
    assert read_sidecar(result)["events"]["foreign_hand_cmd"]["count"] == 1


def test_foreign_arm_sdk_samples_refuse_before_own_writer_exists(tmp_path):
    """Proves: samples from another rt/arm_sdk writer refuse the run (exit 2) before 86 creates any writer.
    Catches: a check that only runs after our writer exists, so two writers fight over the arms (review M4) [mutation]."""
    clock = FakeClock()
    arm = FakeArm(clock)
    created = []
    io = io_recording_writers(clock, arm, created,
                              foreign_samples=lambda: {"rt/arm_sdk": {"samples": 12, "max_weight": 1.0}})

    result = run("ready", io, tmp_path)

    assert result.exit_code == 2 and "someone else is sending on rt/arm_sdk" in result.reason
    assert created == []


def test_start_refused_when_arms_off_factory_pose(tmp_path):
    """Proves: arms 0.2 rad away from today's factory record refuse `ready` (exit 2) and no writer is created.
    Catches: a start from a pose our crashed run still holds - the ramp-out would then swing the gap (rule 2, N4)."""
    clock = FakeClock()
    arm = FakeArm(clock, FACTORY + np.r_[np.zeros(6), 0.2, np.zeros(10)])
    io = make_io(clock, arm, factory_record=factory_record())

    result = run("ready", io, tmp_path)

    assert result.exit_code == 2 and result.reason.startswith("arms are 0.20 rad from the factory pose")
    assert arm.messages == []


def test_release_refused_without_factory_record(tmp_path):
    """Proves: `release` with no factory record today is refused and sends the operators to the owner's remote.
    Catches: a blind release that ramps out wherever the arms are (ruling R12, review N2)."""
    clock = FakeClock()
    arm = FakeArm(clock)

    result = run("release", make_io(clock, arm, factory_record=None), tmp_path)

    assert result.exit_code == 2 and "remote" in result.reason
    assert arm.messages == []


@pytest.mark.parametrize("held", [True, False])
def test_factory_gap_refusal_offers_m0_only_when_arms_are_free(tmp_path, held):
    """Proves: the factory-gap refusal names `86 release` while our last run may hold the arms, else only the M0 re-record.
    Catches: a message that offers re-recording a pose our own code still holds (R2-2 (iii))."""
    if held:  # our last run today was killed after its first publish
        killed = {"arm_sdk_writer": True, "exit_code": None, "last_weight_sent": None}
        (tmp_path / f"{time.strftime('%Y%m%d')}_000001_run.json").write_text(json.dumps(killed))
    clock = FakeClock()
    io = make_io(clock, FakeArm(clock, FACTORY + 0.3), factory_record=factory_record())

    result = run("ready", io, tmp_path)

    assert result.exit_code == 2
    assert ("run `86 release` first" in result.reason) is held
    assert ("re-record the factory pose (runbook M0)" in result.reason) is not held
