"""Real G1: the ONLY module that sends commands - rt/arm_sdk (waist + arms) and rt/brainco/<side>/cmd (hands).

Why it exists: every message that can move the robot goes through the two fill functions below, so the last line of defence
(weight in [0, 1], 17 finite targets, gains never above the sim gains the policy learned with, hand targets inside the hand cap)
sits in one short file that a static test can pin (PLAN.md 4.2 rules 1 and 8). Nothing here decides WHAT to send; that is
g1sim/real_run.py. Never Damp, ZeroTorque, ReleaseMode or rt/lowcmd: those switch off or replace Unitree's walking controller.
Robot env: Python 3.10, numpy, cyclonedds 0.10.2, unitree_sdk2py at 65691c8 (PLAN.md 2.7).
"""
import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_, unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

from g1sim import brainco, real_io, real_safety

ARM_SDK_TOPIC = "rt/arm_sdk"     # Unitree arm7 example: the controller blends this topic into its own arm command
WEIGHT_SLOT = 29                 # kNotUsedJoint: motor_cmd[29].q carries the blend weight (Unitree arm7 example line 64)
HAND_MOTORS = 6                  # BrainCo Revo2 motors per hand (PLAN.md 2.2, g1sim/brainco.py MOTORS)
FLOAT_SLACK = 1e-9               # float slack when comparing gains with SIM_GAINS and hand targets with HAND_CAP


def fill_arm_sdk(cmd, weight: float, q17, kp17, kd17):
    """Write one rt/arm_sdk command into `cmd` (a unitree_hg LowCmd_) and return it. Refuses (ValueError) anything a safe
    command can never contain, because a wrong value here reaches the motors 20 ms later."""
    q17, kp17, kd17 = (np.asarray(v, dtype=float) for v in (q17, kp17, kd17))
    _check_arm_values(weight, q17, kp17, kd17)
    for i, motor in enumerate(real_safety.SDK_MOTORS):
        m = cmd.motor_cmd[motor]
        m.q, m.dq, m.tau, m.kp, m.kd = float(q17[i]), 0.0, 0.0, float(kp17[i]), float(kd17[i])
    cmd.motor_cmd[WEIGHT_SLOT].q = float(weight)
    return cmd


def _check_arm_values(weight: float, q17: np.ndarray, kp17: np.ndarray, kd17: np.ndarray) -> None:
    if not (np.isfinite(weight) and 0.0 <= weight <= 1.0):
        raise ValueError(f"arm_sdk weight {weight} is outside [0, 1]")
    for name, values in (("q", q17), ("kp", kp17), ("kd", kd17)):
        if values.shape != (len(real_safety.SDK_MOTORS),):
            raise ValueError(f"arm_sdk {name} needs 17 values (motors 12..28), got shape {values.shape}")
        if not np.isfinite(values).all():
            raise ValueError(f"arm_sdk {name} contains NaN or Inf")
    if (kp17 > real_safety.SIM_GAINS_KP + FLOAT_SLACK).any() or (kd17 > real_safety.SIM_GAINS_KD + FLOAT_SLACK).any():
        raise ValueError("arm_sdk gains above the sim gains (PLAN.md 4.2 rule 8): stiffer than the policy was trained with")


class ArmSdkSender:
    """Publisher on rt/arm_sdk. Creating it creates a DDS writer, so 86 only builds it after every preflight check passed
    and after the run sidecar marks "arm_sdk_writer": true (PLAN.md 3.4, R2-2)."""

    def __init__(self, iface: str):
        real_io.init_dds(iface)
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._crc = CRC()
        self._publisher = ChannelPublisher(ARM_SDK_TOPIC, LowCmd_)
        self._publisher.Init()

    def send(self, weight: float, q17, kp17, kd17) -> None:
        """Fill, checksum and publish one command (the controller drops messages with a wrong CRC)."""
        fill_arm_sdk(self._cmd, weight, q17, kp17, kd17)
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._publisher.Write(self._cmd)


def fill_hand_cmd(msg, targets6, speed: float):
    """Write one BrainCo command (6 motors, q 0 = open .. 1 = closed, dq = speed) into `msg` (MotorCmds_) and return it.
    Refuses targets above HAND_CAP and speeds above HAND_SPEED (ruling R6): the service has no timeout of its own."""
    targets6 = np.asarray(targets6, dtype=float)
    if targets6.shape != (HAND_MOTORS,) or not np.isfinite(targets6).all():
        raise ValueError(f"hand command needs 6 finite targets, got {targets6}")
    if (targets6 < 0).any() or (targets6 > real_safety.HAND_CAP + FLOAT_SLACK).any():
        raise ValueError(f"hand targets {targets6} outside [0, {real_safety.HAND_CAP}] (ruling R6)")
    if not (0.0 < speed <= real_safety.HAND_SPEED):
        raise ValueError(f"hand speed {speed} outside (0, {real_safety.HAND_SPEED}] (ruling R6)")
    msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(HAND_MOTORS)]  # always 6 entries: the service does not check
    for cmd, target in zip(msg.cmds, targets6):
        cmd.q, cmd.dq = float(target), float(speed)
    return msg


class HandSender:
    """Publisher on rt/brainco/<side>/cmd. DDS must already be initialised (86 builds the LowState reader first)."""

    def __init__(self, side: str):
        self._publisher = ChannelPublisher(brainco.TOPIC_CMD.format(side=side), MotorCmds_)
        self._publisher.Init()

    def send(self, targets6, speed: float) -> None:
        """Publish one hand command; the hand keeps it until the next one (PLAN.md 2.2: no timeout)."""
        self._publisher.Write(fill_hand_cmd(MotorCmds_(), targets6, speed))
