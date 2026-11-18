"""Sim stand-in for Unitree's arm_sdk: the LeRobot G1 sim (GR00T walking controller) that obeys rt/arm_sdk like the robot.

Why it exists: 86 must be proven on a sim before the robot day, but the LeRobot sim only listens to rt/lowcmd. On the real G1,
Unitree's controller blends rt/arm_sdk into its own command by the weight in motor_cmd[29].q and keeps the last message when
messages stop (PLAN.md 1, assumption; not published by Unitree). 85 emulates exactly that, so a sim run proves our code path,
timing and safety behaviour - not Unitree's controller:
  - every rt/lowcmd message GR00T writes is copied; in the COPY motors 12-28 get q, kp, kd = w * arm_sdk + (1 - w) * GR00T's
    (w clipped to [0, 1]); the copy gets a new CRC and goes to the sim. GR00T's own message stays untouched (review m6: an
    in-place blend would leak our targets into GR00T's next command);
  - the last rt/arm_sdk message is latched;
  - --no-blend only counts rt/arm_sdk samples (84's read-only check: 84 must send nothing);
  - --pause-lowstate-at T --pause-for D stops rt/lowstate for every outside reader D seconds, T seconds after the first
    rt/arm_sdk message (a pulled laptop cable). GR00T keeps its state in-process, as the robot's own controller would.
Loopback only (lo0 / lo), refused while a 192.168.123.x address exists, re-checked every second. Sim env:
    DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib ~/dev/g1-ext/.venv/bin/python scripts/85_arm_sdk_sim.py \
        --seconds 60 --report runs/sim85.json [--band] [--no-blend] [--pause-lowstate-at 10 --pause-for 1]
Prints READY once the robot stands and every reader is up, then REPORT <path> at the end. Exit 0, 2 refused, 1 sim failure.
Ctrl+C / SIGTERM end the run early and still write the report.
"""
import argparse
import copy
import json
import os
import pathlib
import signal
import sys
import threading
import time
import traceback

import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "tests" / "helpers"))

import sim_host  # noqa: E402
from g1sim import brainco, real_io  # noqa: E402

SDK_MOTORS = list(real_io.SDK_MOTORS)   # motors 12-28: waist + arms, the ones rt/arm_sdk addresses (Unitree arm7 example)
WEIGHT_SLOT = real_io.WEIGHT_SLOT       # motor_cmd[29].q carries the blend weight (Unitree arm7 example line 64)
MEASURE_HZ = 50                         # PLAN.md 3.5: measured joint angles in the report at 50 Hz
SIM85_MARK = 0x85                       # reserve[0] of every LowCmd copy 85 writes, so a foreign rt/lowcmd writer stands out
BLEND_SAMPLE_EVERY = 5                  # every 5th blended copy goes into the report (10 Hz at GR00T's 50 Hz)
EXIT_FAILED, EXIT_REFUSED = 1, 2


def blend(own: np.ndarray, sdk: np.ndarray, weight: float) -> np.ndarray:
    """w * sdk + (1 - w) * own with w clipped to [0, 1] - the linear blend assumed for Unitree's controller (PLAN.md 1)."""
    w = float(np.clip(weight, 0.0, 1.0))
    return w * np.asarray(sdk, dtype=float) + (1.0 - w) * np.asarray(own, dtype=float)


def motor_values(msg, field: str) -> np.ndarray:
    """q, kp or kd of motors 12..28 of a unitree_hg LowCmd_."""
    return np.array([getattr(msg.motor_cmd[m], field) for m in SDK_MOTORS], dtype=float)


def max_gap_s(times: list) -> float:
    """Largest gap between consecutive receive times (0 for fewer than 2): a gap > 60 ms means 86 stalled (rule 15)."""
    return float(np.max(np.diff(times))) if len(times) > 1 else 0.0


class ArmSdkLatch:
    """Keeps the newest rt/arm_sdk message (the latch) and every message as received, for the report."""

    def __init__(self):
        self._lock = threading.Lock()
        self.latest = None          # (weight, q17, kp17, kd17) of the newest message, or None before the first
        self.received = []          # report entries {t, weight, q, kp, kd}

    def on_sample(self, msg) -> None:
        entry = {"t": time.time(), "weight": float(msg.motor_cmd[WEIGHT_SLOT].q)}
        values = {field: motor_values(msg, field) for field in ("q", "kp", "kd")}
        with self._lock:
            self.latest = (entry["weight"], values["q"], values["kp"], values["kd"])
            self.received.append({**entry, **{k: v.round(6).tolist() for k, v in values.items()}})

    def snapshot(self):
        with self._lock:
            return self.latest


class LowCmdBlender:
    """Wraps GR00T's rt/lowcmd Write: writes a marked COPY, blended with the latched arm_sdk message unless no_blend."""

    def __init__(self, robot, latch: ArmSdkLatch, no_blend: bool):
        from unitree_sdk2py.utils.crc import CRC
        self._write, self._latch, self._no_blend, self._crc = robot.lowcmd_publisher.Write, latch, no_blend, CRC()
        self.samples, self._count = [], 0     # report entries {t, weight, own_q, sent_q}
        robot.lowcmd_publisher.Write = self.write

    def write(self, msg, timeout=None):
        sent = copy.deepcopy(msg)
        latched = None if self._no_blend else self._latch.snapshot()
        if latched is not None:
            weight, q, kp, kd = latched
            for field, sdk in (("q", q), ("kp", kp), ("kd", kd)):
                for motor, value in zip(SDK_MOTORS, blend(motor_values(msg, field), sdk, weight)):
                    setattr(sent.motor_cmd[motor], field, float(value))
        sent.reserve[0] = SIM85_MARK
        sent.crc = self._crc.Crc(sent)
        self._sample(msg, sent, latched)
        return self._write(sent, timeout)

    def _sample(self, own, sent, latched) -> None:
        self._count += 1
        if self._count % BLEND_SAMPLE_EVERY == 0:
            self.samples.append({"t": time.time(), "weight": None if latched is None else latched[0],
                                 "own_q": motor_values(own, "q").round(6).tolist(),
                                 "sent_q": motor_values(sent, "q").round(6).tolist()})


class LowStatePause:
    """Stops rt/lowstate for outside readers while paused. GR00T's own reader then gets the sim's message in-process.

    Both wrappers run in LeRobot's subscribe thread, one sim step after the other (Write, then Read), so Read follows what
    the last Write did: a blocking DDS Read after a skipped Write would freeze the sim thread."""

    def __init__(self, robot, bridge):
        self.paused, self._skipped = False, False
        self._message = bridge.low_state
        self._write, self._read = bridge.low_state_puber.Write, robot.lowstate_subscriber.Read
        bridge.low_state_puber.Write = self.write
        robot.lowstate_subscriber.Read = self.read

    def write(self, msg, timeout=None):
        self._skipped = self.paused
        return True if self._skipped else self._write(msg, timeout)

    def read(self, timeout=None):
        return self._message if self._skipped else self._read(timeout)


class CommandCounter:
    """Counts samples on a command topic; for hand topics also keeps the last 6 targets (the e2e checks "hands opened")."""

    def __init__(self, topic: str, msg_type, keep_hand_targets: bool = False):
        self.samples, self.foreign, self.last = 0, 0, None
        self._keep = keep_hand_targets
        self._channel = real_io.subscribe_topic(topic, msg_type, self.on_sample)

    def on_sample(self, msg) -> None:
        self.samples += 1
        if self._keep:
            self.last = [round(float(c.q), 6) for c in msg.cmds[:real_io.HAND_MOTOR_COUNT]]
        elif msg.reserve[0] != SIM85_MARK:
            self.foreign += 1


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="LeRobot G1 sim that obeys rt/arm_sdk (loopback only)")
    parser.add_argument("--seconds", type=float, required=True, help="wall seconds to run after READY")
    parser.add_argument("--report", type=pathlib.Path, required=True, help="report JSON (PLAN.md 3.5)")
    parser.add_argument("--iface", default="lo0", help="lo0 (macOS) or lo (Linux); anything else is refused")
    parser.add_argument("--band", action="store_true", help="keep the elastic band on (the sim robot hangs)")
    parser.add_argument("--no-blend", action="store_true", help="count rt/arm_sdk samples, never obey them")
    parser.add_argument("--pause-lowstate-at", type=float, help="seconds after the first rt/arm_sdk message")
    parser.add_argument("--pause-for", type=float, default=1.0, help="length of the rt/lowstate pause in seconds")
    return parser.parse_args(argv)


def pause_window(first_arm_sdk_t, at_s, for_s):
    """(start, end) wall time of the LowState pause, or None (no pause asked, or no rt/arm_sdk message yet)."""
    if at_s is None or first_arm_sdk_t is None:
        return None
    return first_arm_sdk_t + at_s, first_arm_sdk_t + at_s + for_s


def run(args: argparse.Namespace, stop: threading.Event) -> dict:
    """Stand the sim up, wrap its DDS traffic, record for args.seconds; returns the report dict."""
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

    robot = sim_host.start_standing_g1(band=args.band, iface=args.iface)
    sim = robot.sim_env.sim_env
    latch = ArmSdkLatch()
    blender = LowCmdBlender(robot, latch, args.no_blend)
    pause = LowStatePause(robot, sim.unitree_bridge)
    real_io.subscribe_topic("rt/arm_sdk", LowCmd_, latch.on_sample)
    lowcmd = CommandCounter("rt/lowcmd", LowCmd_)
    hands = {side: CommandCounter(brainco.TOPIC_CMD.format(side=side), MotorCmds_, keep_hand_targets=True)
             for side in brainco.SIDES}
    report = {"source": "sim85", "iface": args.iface, "band": args.band, "no_blend": args.no_blend, "measured": [],
              "pelvis_z_min": float(sim.mj_data.qpos[2]), "lowstate_pause": None, "stopped": "seconds"}
    watch, start = sim_host.NetWatch(), time.time()
    next_tick = start
    print("READY", flush=True)
    try:
        while time.time() - start < args.seconds and not stop.is_set():
            if watch.robot_appeared():
                report["stopped"] = "robot_network"
                break
            now, data = time.time(), sim.mj_data
            first = latch.received[0]["t"] if latch.received else None
            window = pause_window(first, args.pause_lowstate_at, args.pause_for)
            pause.paused = window is not None and window[0] <= now < window[1]
            if window is not None:
                report["lowstate_pause"] = {"start_t": window[0], "end_t": window[1]}
            report["measured"].append({"t": now, "q": data.qpos[sim.body_qadr].round(6).tolist()})
            report["pelvis_z_min"] = min(report["pelvis_z_min"], float(data.qpos[2]))
            next_tick += 1.0 / MEASURE_HZ   # deadline scheduling: the loop's own work does not lower the rate
            time.sleep(max(0.0, next_tick - time.time()))
        if stop.is_set():
            report["stopped"] = "signal"
    finally:
        pause.paused = False
        robot.disconnect()
    received = list(latch.received)
    report.update(arm_sdk=received, arm_sdk_samples=len(received), lowcmd_samples=lowcmd.samples,
                  lowcmd_foreign_samples=lowcmd.foreign,
                  max_gap_s=max_gap_s([r["t"] for r in received]), blend_samples=blender.samples,
                  hand_cmd={side: {"samples": c.samples, "last": c.last} for side, c in hands.items()},
                  run_s=round(time.time() - start, 3))
    return report


def main(argv=None) -> int:
    args = parse_args(argv)
    problem = sim_host.interface_problem(args.iface, real_io.robot_net_present())
    if problem:
        print(problem, file=sys.stderr, flush=True)
        return EXIT_REFUSED
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    report = run(args, stop)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report))
    print(f"arm_sdk samples {report['arm_sdk_samples']}, foreign rt/lowcmd samples {report['lowcmd_foreign_samples']}, "
          f"max gap {report['max_gap_s'] * 1000:.0f} ms, pelvis z min {report['pelvis_z_min']:.3f} m, run {report['run_s']} s")
    print(f"REPORT {args.report}", flush=True)
    if report["stopped"] == "robot_network":
        print("STOPPED: a 192.168.123.x address appeared (robot cable?) - the sim stopped publishing", file=sys.stderr)
        return EXIT_REFUSED
    return 0


if __name__ == "__main__":
    try:
        code = main()
    except Exception:   # the sim fell or stalled: the message says what to do
        traceback.print_exc()
        code = EXIT_FAILED
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)   # DDS and sim threads keep the interpreter alive otherwise (scripts/52_brainco_hand_dds.py)
