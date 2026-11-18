"""Unit: the only publishing module (g1sim/real_send.py) and the static ban on controller-switching code in every motion file.

No DDS writer is created: fill_arm_sdk is tested on a plain LowCmd_ message, the ban reads the source files.
"""
import ast
import pathlib
import types

import numpy as np
import pytest

pytest.importorskip("unitree_sdk2py")

from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_  # noqa: E402

from g1sim import real_io, real_safety as rs, real_send  # noqa: E402

pytestmark = pytest.mark.unit

REPO = pathlib.Path(__file__).resolve().parent.parent
MOTION_CODE = [REPO / "g1sim" / f"{name}.py" for name in
               ("real_safety", "real_timing", "real_send", "real_run", "real_run_exits", "real_run_modes", "real_run_policy",
                "real_run_log", "real_preflight", "motion_gate")] + [REPO / "scripts" / "86_real_run.py"]
BANNED_NAMES = {"Damp", "ZeroTorque", "ReleaseMode", "MotionSwitcher", "MotionSwitcherClient", "run_g1_server", "LocoClient",
                "DataWriter", "Publisher"}
PUBLISHER_NAME = "ChannelPublisher"           # allowed only in real_send.py


def test_fill_arm_sdk_writes_17_motors_and_weight_slot_29():
    """Proves: fill_arm_sdk sets q/kp/kd of motors 12..28, dq = tau = 0, the weight in motor_cmd[29].q, and no leg motor.
    Catches: an off-by-one in the motor layout (a leg joint commanded, or the weight in the wrong slot)."""
    q17 = np.linspace(-0.5, 0.5, 17)

    cmd = real_send.fill_arm_sdk(unitree_hg_msg_dds__LowCmd_(), 0.4, q17, rs.SIM_GAINS_KP, rs.SIM_GAINS_KD)

    assert [cmd.motor_cmd[m].q for m in rs.SDK_MOTORS] == pytest.approx(q17)
    assert [cmd.motor_cmd[m].kp for m in rs.SDK_MOTORS] == list(rs.SIM_GAINS_KP)
    assert all(cmd.motor_cmd[m].dq == 0 and cmd.motor_cmd[m].tau == 0 for m in rs.SDK_MOTORS)
    assert cmd.motor_cmd[29].q == pytest.approx(0.4)
    assert all(cmd.motor_cmd[m].kp == 0 for m in range(12))


def test_fill_arm_sdk_refuses_gains_above_sim_gains():
    """Proves: a kp above the sim gains is refused before it reaches a message (rule 8).
    Catches: a stiffer arm than the policy was trained with, which presses harder on contact."""
    stiff = rs.SIM_GAINS_KP.copy()
    stiff[5] += 1.0

    with pytest.raises(ValueError, match="above the sim gains"):
        real_send.fill_arm_sdk(unitree_hg_msg_dds__LowCmd_(), 1.0, np.zeros(17), stiff, rs.SIM_GAINS_KD)


@pytest.mark.parametrize("send, expected", [
    pytest.param(lambda: real_send.fill_arm_sdk(unitree_hg_msg_dds__LowCmd_(), 1.2, np.zeros(17), rs.SIM_GAINS_KP,
                                                rs.SIM_GAINS_KD), r"outside \[0, 1\]", id="weight_1.2"),
    pytest.param(lambda: real_send.fill_hand_cmd(types.SimpleNamespace(), np.full(6, 0.7), rs.HAND_SPEED),
                 "hand targets", id="hand_target_0.7"),
    pytest.param(lambda: real_send.fill_hand_cmd(types.SimpleNamespace(), np.zeros(6), 0.6), "hand speed",
                 id="hand_speed_0.6"),
    pytest.param(lambda: real_send.fill_arm_sdk(unitree_hg_msg_dds__LowCmd_(), 0.5, np.r_[np.nan, np.zeros(16)],
                                                rs.SIM_GAINS_KP, rs.SIM_GAINS_KD), "q contains NaN", id="nan_in_q"),
    pytest.param(lambda: real_send.fill_arm_sdk(unitree_hg_msg_dds__LowCmd_(), 1.0, np.zeros(17), rs.SIM_GAINS_KP,
                                                rs.SIM_GAINS_KD + 1.0), "above the sim gains", id="kd_above_sim_gains"),
    pytest.param(lambda: real_send.fill_hand_cmd(types.SimpleNamespace(), np.full(6, -0.1), rs.HAND_SPEED),
                 "hand targets", id="hand_target_negative"),
])
def test_send_refuses_values_outside_the_safe_range(send, expected):
    """Proves: an arm_sdk weight above 1, a NaN target, a kd above the sim gains, a hand target above HAND_CAP (0.6) or
    below 0 and a hand speed above HAND_SPEED (0.5) are refused before they reach a message (rules 8, 9, ruling R6).
    The NaN check is the only NaN guard of the UNFILTERED fault ramp-out besides fault_targets.
    Catches: a last line of defence that lost one of its range checks, so a planner bug reaches the motors [mutation,
    one per case]."""
    with pytest.raises(ValueError, match=expected):
        send()


def test_sdk_motor_layout_equal_in_real_io_and_real_safety():
    """Proves: the readers (real_io) and the sender side (real_safety) agree on motors 12..28.
    Catches: a 17-vector read in one order and sent in another (a waist angle sent to a shoulder)."""
    assert real_io.SDK_MOTORS == rs.SDK_MOTORS == tuple(range(12, 29))


def docstring_nodes(tree: ast.AST) -> set:
    """ids of the docstring constants (module, class, function): text there may name rt/lowcmd to forbid it."""
    ids = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.FunctionDef, ast.ClassDef)) and node.body:
            first = node.body[0]
            if isinstance(first, ast.Expr) and isinstance(first.value, ast.Constant):
                ids.add(id(first.value))
    return ids


def names_used(node: ast.AST) -> set:
    """Every name a node uses: a plain name, an attribute, or the last part of an imported name."""
    names = {getattr(node, "id", None), getattr(node, "attr", None)}
    if isinstance(node, (ast.Import, ast.ImportFrom)):
        names |= {alias.name.split(".")[-1] for alias in node.names}
    return names


def motion_code_problems(paths) -> list:
    """Banned names (Damp, LocoClient, writers outside real_send, ...) and rt/lowcmd strings in real_send's code."""
    problems = []
    for path in paths:
        tree = ast.parse(path.read_text())
        docstrings = docstring_nodes(tree)
        banned = BANNED_NAMES if path.name == "real_send.py" else BANNED_NAMES | {PUBLISHER_NAME}
        for node in ast.walk(tree):
            for name in names_used(node) & banned:
                problems.append(f"{path.name}:{node.lineno} uses {name}")
            is_code_string = isinstance(node, ast.Constant) and isinstance(node.value, str) and id(node) not in docstrings
            if path.name == "real_send.py" and is_code_string and "rt/lowcmd" in node.value:
                problems.append(f"{path.name}:{node.lineno} names rt/lowcmd")
    return problems


def test_motion_code_never_uses_damp_or_lowcmd():
    """Proves: no motion file uses Damp, ZeroTorque, ReleaseMode, MotionSwitcher, run_g1_server, LocoClient or a raw writer,
    ChannelPublisher appears only in real_send, and real_send never names rt/lowcmd (rule 4.2.1).
    Catches: code that switches off Unitree's walking controller or writes rt/lowcmd and drops the robot [mutation]."""
    assert motion_code_problems(MOTION_CODE) == []
