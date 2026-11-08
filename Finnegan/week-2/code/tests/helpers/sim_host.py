"""Loopback-only sim host for Track C: the standing G1 sim that 85 wraps, and a foreign rt/arm_sdk test publisher.

Why it exists: 86 is proven against a sim that obeys rt/arm_sdk (scripts/85_arm_sdk_sim.py) before anyone runs it on the robot.
Both things here publish on DDS (the sim publishes rt/lowstate, the test publisher rt/arm_sdk), so both must never reach the
robot network: they refuse unless the interface is loopback and no 192.168.123.x address exists, and the publisher re-checks
that address once per second (PLAN.md 3.5, review r5-2).
Library (85 imports start_standing_g1) and a small CLI for the tests, so the foreign writer runs in its own process:
    DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib ~/dev/g1-ext/.venv/bin/python tests/helpers/sim_host.py \
        arm-sdk --iface lo0 --weight 1.0 --seconds 3 --pose ready
Sim env only (~/dev/g1-ext/.venv: LeRobot, unitree_sdk2py, cyclonedds).
"""
import argparse
import json
import os
import pathlib
import subprocess
import sys
import time
from typing import Callable, Optional

import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from g1sim import action_space, real_io, real_safety  # noqa: E402

SEND_HZ = real_safety.SEND_HZ        # 50 Hz, the rate 86 sends at (Unitree arm7 example line 69)
NET_CHECK_S = 1.0                    # review r5-2: look for the robot address once per second while publishing
BAND_S = 3.0                         # wall seconds at the elastic band before it is released (scripts/52_brainco_hand_dds.py)
BALANCE_SIM_S = 3.0                  # sim seconds standing free before the sim counts as ready (g1sim/walk_sim.py BALANCE_S)
FALL_HEIGHT = 0.5                    # pelvis below this = fallen (g1sim/walk_sim.py)
STAND_TIMEOUT_S = 120.0              # wall seconds the free stand may take on this Mac (the sim runs slower than real time)
EXIT_REFUSED = 2
# Target poses the test publisher can send (17-vectors, motors 12..28). "ready" = the ready pose 86 rises to.
POSES = {"ready": real_safety.sdk_targets(action_space.BALL_READY_POSE, np.zeros(17)), "zero": np.zeros(17)}


class UnsafeInterface(RuntimeError):
    """Raised when a DDS publisher could reach the robot network."""


def interface_problem(iface: str, robot_net: bool) -> Optional[str]:
    """Why a sim or test publisher must not start on `iface`, or None. Pure, so the refusal is testable without DDS."""
    if not real_io.is_loopback(iface):
        return f"refused: {iface!r} is not a loopback interface - the sim runs only on lo0 (macOS) or lo (Linux)"
    if robot_net:
        return "refused: a 192.168.123.x address exists (robot network) - unplug the robot cable, then run again"
    return None


def refuse_unless_safe_loopback(iface: str) -> None:
    """Raise UnsafeInterface unless `iface` is loopback and no robot address exists right now."""
    problem = interface_problem(iface, real_io.robot_net_present())
    if problem:
        raise UnsafeInterface(problem)


class NetWatch:
    """Re-checks once per second that no robot address appeared (a cable plugged in while a sim publishes)."""

    def __init__(self, clock: Callable[[], float] = time.monotonic, net_check: Callable[[], bool] = real_io.robot_net_present):
        self._clock, self._net_check = clock, net_check
        self._next = clock()

    def robot_appeared(self) -> bool:
        now = self._clock()
        if now < self._next:
            return False
        self._next = now + NET_CHECK_S
        return self._net_check()


def start_standing_g1(band: bool, iface: str = "lo0"):
    """The LeRobot G1 sim (BrainCo hands, GR00T walking controller, no cameras), standing. Returns the connected UnitreeG1.

    band=True keeps the elastic band on (the sim robot hangs, as in a gantry); band=False releases it and waits until the
    robot stands on its own feet. DDS starts on the loopback interface BEFORE LeRobot is imported, because LeRobot
    initialises on "lo", which does not exist on macOS (PLAN.md 2.5, scripts/52_brainco_hand_dds.py)."""
    refuse_unless_safe_loopback(iface)
    real_io.init_dds(iface)
    from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config

    from g1sim.brainco_env import use_brainco_sim

    use_brainco_sim(cameras=False)   # no renderer: two OpenGL threads can hang this Mac in the graphics driver
    robot = UnitreeG1(UnitreeG1Config(is_simulation=True, controller="GrootLocomotionController"))
    robot.connect()
    if not band:
        _release_band(robot.sim_env.sim_env)
    return robot


def _release_band(sim) -> None:
    """Stand at the band, release it, stand free; fail loudly if the robot falls or the sim stalls."""
    time.sleep(BAND_S)
    sim.elastic_band.enable = False
    data, deadline = sim.mj_data, time.monotonic() + STAND_TIMEOUT_S
    end = float(data.time) + BALANCE_SIM_S
    while float(data.time) < end:
        if data.qpos[2] < FALL_HEIGHT:
            raise RuntimeError(f"sim G1 fell after the band was released (pelvis {data.qpos[2]:.2f} m) - re-run; "
                               "if it repeats, check scripts/52_brainco_hand_dds.py")
        if time.monotonic() > deadline:
            raise RuntimeError(f"sim time did not reach {end:.1f} s within {STAND_TIMEOUT_S:.0f} s - the sim stalls")
        time.sleep(0.01)


def publish_test_arm_sdk(weight: float, q17, seconds: float, iface: str = "lo0") -> str:
    """Send rt/arm_sdk at 50 Hz for `seconds` (weight, q17, the sim gains), as a foreign writer would. Loopback only.

    Returns "done", or "robot_network" when a robot address appeared (publishing stops at once)."""
    refuse_unless_safe_loopback(iface)
    from unitree_sdk2py.core.channel import ChannelPublisher
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from unitree_sdk2py.utils.crc import CRC

    from g1sim import real_send

    real_io.init_dds(iface)
    publisher = ChannelPublisher(real_send.ARM_SDK_TOPIC, LowCmd_)
    publisher.Init()
    cmd, crc = unitree_hg_msg_dds__LowCmd_(), CRC()
    real_send.fill_arm_sdk(cmd, weight, q17, real_safety.SIM_GAINS_KP, real_safety.SIM_GAINS_KD)
    cmd.crc = crc.Crc(cmd)
    watch, start = NetWatch(), time.monotonic()
    next_tick = start
    while time.monotonic() - start < seconds:
        if watch.robot_appeared():
            return "robot_network"
        publisher.Write(cmd)
        next_tick += 1.0 / SEND_HZ
        time.sleep(max(0.0, next_tick - time.monotonic()))
    return "done"


# --- process helpers for the tests (each test starts what it needs and stops it, PLAN.md section 0) ------------------

SIM85 = REPO / "scripts" / "85_arm_sdk_sim.py"
CYCLONEDDS_LIB = pathlib.Path.home() / "dev" / "g1-ext" / "cyclonedds" / "install" / "lib"   # as tests/conftest.py
READY_TIMEOUT_S = 180.0              # wall seconds for LeRobot import + band + free stand on this Mac


def dds_env() -> dict:
    """Environment for a DDS subprocess: unbuffered output, the CycloneDDS library on macOS."""
    env = dict(os.environ, PYTHONUNBUFFERED="1")
    env.setdefault("DYLD_LIBRARY_PATH", str(CYCLONEDDS_LIB))
    return env


def start_85(report: pathlib.Path, seconds: float, *flags: str) -> subprocess.Popen:
    """Start 85 and wait for its READY line; fail loudly with its output when it does not come."""
    proc = subprocess.Popen([sys.executable, str(SIM85), "--seconds", str(seconds), "--report", str(report), *flags],
                            cwd=REPO, env=dds_env(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    deadline, lines = time.monotonic() + READY_TIMEOUT_S, []
    for line in proc.stdout:
        lines.append(line)
        if line.strip() == "READY":
            return proc
        if time.monotonic() > deadline:
            break
    stop_process(proc)
    raise RuntimeError("85 did not print READY:\n" + "".join(lines[-40:]))


def finish_85(proc: subprocess.Popen, report: pathlib.Path, timeout_s: float) -> dict:
    """Wait until 85 ends by itself (its --seconds), then read its report."""
    output = proc.stdout.read()
    code = proc.wait(timeout=timeout_s)
    if code != 0 or not report.exists():
        raise RuntimeError(f"85 ended with exit {code}:\n{output[-3000:]}")
    return json.loads(report.read_text())


def stop_process(proc: subprocess.Popen) -> None:
    """SIGTERM, then wait (85 writes its report on SIGTERM); SIGKILL only if it hangs."""
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()


def run_arm_sdk_publisher(weight: float, seconds: float, pose: str = "ready") -> subprocess.CompletedProcess:
    """Run this file's foreign rt/arm_sdk writer as its own process (DDS threads stay out of the pytest process)."""
    return subprocess.run([sys.executable, __file__, "arm-sdk", "--weight", str(weight), "--seconds", str(seconds),
                           "--pose", pose], cwd=REPO, env=dds_env(), capture_output=True, text=True, timeout=seconds + 60)


def run_script(args: list, timeout_s: float) -> subprocess.CompletedProcess:
    """Run a repo script with the sim env's Python and the DDS environment (84 in the e2e)."""
    return subprocess.run([sys.executable, *map(str, args)], cwd=REPO, env=dds_env(), capture_output=True, text=True,
                          timeout=timeout_s)


def record_sim_factory_pose() -> subprocess.CompletedProcess:
    """84 records the standing sim's arm pose as today's sim factory pose (runbook M0 on loopback; PLAN.md 3.5 m6-4)."""
    return run_script([REPO / "scripts" / "84_real_bringup.py", "--iface", "lo0", "--sim", "--yes-sim", "--steps", "joints",
                       "--record-factory-pose", "--overwrite-factory-pose"], timeout_s=120)


def start_86(mode: str, out: pathlib.Path, *args: str) -> subprocess.Popen:
    """86 <mode> on lo0 with the sim bypass and its log in `out` (never runs/real, PLAN.md 3.5 m6-4)."""
    return subprocess.Popen([sys.executable, str(REPO / "scripts" / "86_real_run.py"), mode, "--iface", "lo0", "--yes-sim",
                             "--out", str(out), *args], cwd=REPO, env=dds_env(), stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)


def finish_86(proc: subprocess.Popen, timeout_s: float) -> dict:
    """{"code", "stdout", "log" (npz arrays or None), "sidecar" (dict or None)} of a finished 86 run (its LOG line)."""
    stdout, _ = proc.communicate(timeout=timeout_s)
    log_lines = [line[4:].strip() for line in stdout.splitlines() if line.startswith("LOG ")]
    result = {"code": proc.returncode, "stdout": stdout, "log": None, "sidecar": None}
    if log_lines and pathlib.Path(log_lines[-1]).exists():
        npz = pathlib.Path(log_lines[-1])
        result["log"] = dict(np.load(npz, allow_pickle=False))
        result["sidecar"] = json.loads(npz.with_suffix(".json").read_text())
    return result


def start_81_replay(episode: pathlib.Path) -> tuple:
    """(process, port) of the replay policy server 81 serving `episode` (no torch; sim env)."""
    proc = subprocess.Popen([sys.executable, str(REPO / "scripts" / "81_smolvla_server.py"), "--replay", str(episode),
                             "--port", "0"], cwd=REPO, env=dds_env(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    for line in proc.stdout:
        if line.startswith("LISTENING "):
            return proc, int(line.split()[1])
    stop_process(proc)
    raise RuntimeError("81 --replay did not print LISTENING")


def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(description="loopback-only foreign rt/arm_sdk writer for Track C's tests")
    commands = parser.add_subparsers(dest="command", required=True)
    arm = commands.add_parser("arm-sdk", help="publish rt/arm_sdk at 50 Hz")
    arm.add_argument("--iface", default="lo0", help="lo0 (macOS) or lo (Linux); anything else is refused")
    arm.add_argument("--weight", type=float, required=True, help="arm_sdk weight in [0, 1]")
    arm.add_argument("--seconds", type=float, required=True, help="how long to publish (always bounded)")
    target = arm.add_mutually_exclusive_group(required=True)
    target.add_argument("--pose", choices=sorted(POSES), help="a named 17-vector target")
    target.add_argument("--q", help="JSON list of 17 targets for motors 12..28")
    args = parser.parse_args(argv)
    q17 = POSES[args.pose] if args.pose else np.array(json.loads(args.q), dtype=float)
    try:
        print(f"PUBLISHING rt/arm_sdk weight {args.weight} for {args.seconds:.1f} s on {args.iface}", flush=True)
        reason = publish_test_arm_sdk(args.weight, q17, args.seconds, args.iface)
    except UnsafeInterface as error:
        print(error, file=sys.stderr, flush=True)
        return EXIT_REFUSED
    if reason == "robot_network":
        print("STOPPED: a 192.168.123.x address appeared (robot cable?) - publishing stopped at once", file=sys.stderr, flush=True)
        return EXIT_REFUSED
    print("DONE", flush=True)
    return 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)   # DDS threads keep the interpreter alive otherwise (scripts/52_brainco_hand_dds.py)
