"""Real G1 read-only bring-up checklist (robot day, runbook Part B and M0): listens to the robot, never commands it.

Why it exists: before anything moves, the operators check network, joint data, hands and camera, ask the owner Q1-Q12,
record where the factory controller holds the arms (M0), and record the robot as it is (data for dataset v2). The very first
thing main() does is install the publisher guard, so this script cannot create a DDS writer even by mistake; at the end it
counts its own writers (must be 0).
Each step prints `STEP <name>: PASS|FAIL|SKIP|NOT RUN <reason>` and lands in real_io.BRINGUP_REPORT_DIR/real_bringup_<stamp>.json
(runs/ of the repo, whatever the cwd); exit 0 only if no
step FAILs (exit 2 = refused before any step).
Robot env (Python 3.10), from the repo root:
    python scripts/84_real_bringup.py --iface en5 --steps net,probe,joints
    python scripts/84_real_bringup.py --iface en5 --steps joints --record-factory-pose --mode-note "<owner's Q8 answer>"
    python scripts/84_real_bringup.py --iface en5 --steps camera,record --robot-pc unitree@192.168.123.164 --camera-device 4
    python scripts/84_real_bringup.py --dry-run         # no DDS: lists the steps
Sim (loopback only): --iface lo0 --sim --yes-sim --camera-cmd "python tests/helpers/fake_camera.py --npz FILE"
"""
import argparse
import datetime
import faulthandler
import io
import json
import pathlib
import subprocess
import sys
import time

import cv2
import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
from g1sim import episode, policy_wire, real_bringup as rb, real_camera, real_io  # noqa: E402

REFERENCE_FRAME = REPO / "docs" / "robot_day" / "reference_head_frame.png"   # frame 0 of dataset episode 0 (Track E1)
PROBE_SCRIPT = REPO / "scripts" / "71_real_probe.py"
OBSERVE_DIR = REPO / "runs" / "real_observe"
WAIT_FOR_DATA_S = 3.0           # first LowState / hand state must arrive within this time
POLL_S = 0.05                   # how often wait_for_sample looks
WRITER_LISTEN_S = 2.0           # command-topic listen window (PLAN.md 3.1 active_writers)
HAND_RATE_LISTEN_S = 1.0        # hand readers listen this long before the rate is read: real_io rate_hz's default window
PROBE_TIMEOUT_EXTRA_S = 30.0    # 71 gets --seconds plus this before it counts as hung
PING_TIMEOUT_S = 3.0            # one ping to PC2; a healthy robot network answers in milliseconds
POLICY_TIMEOUT_S = 10.0         # one policy chunk; 81 on CPU needs about 2.7 s (Track E1 self-test, STATUS 17:51)
POLICY_CONNECT_WAIT_S = 10.0    # 81 must already be running when 84 asks; a short wait fails a wrong --policy fast
ERROR_TAIL_CHARS = 300          # how much of 71's stderr a FAIL line shows
EXIT_REFUSED = 2
STEP_HELP = {
    "net": "static IP 192.168.123.222 on --iface + one ping to PC2 192.168.123.164 (SKIP with --sim)",
    "probe": "71_real_probe.py (topics, LowState rate) + rate/age of our own reader + samples on the arm and hand command "
             "topics",
    "joints": "all joint angles, limits check, FACTORY ARMS line; with --record-factory-pose: the M0 record",
    "questions": "owner questions Q1-Q12 (typed answers are saved)",
    "hand-state": "rt/brainco/left|right/state arrive: fresh, rate, 6 finite values each",
    "camera": "45 head-camera frames: framing, fps, colours, overlay with the dataset reference frame",
    "record": "observe-only recording of --seconds (we command nothing); --policy adds predictions afterwards",
}


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Real G1: read-only bring-up checklist (never sends anything)")
    parser.add_argument("--iface", help="network interface of the robot cable, e.g. en5 (lo0 / lo only with --sim --yes-sim)")
    parser.add_argument("--steps", default=",".join(rb.STEPS), help=f"comma list from {', '.join(rb.STEPS)}")
    parser.add_argument("--dry-run", action="store_true", help="list the steps, start no DDS")
    parser.add_argument("--sim", action="store_true", help="run against a loopback sim publisher")
    parser.add_argument("--yes-sim", action="store_true", help="confirm: no robot connected (loopback only)")
    parser.add_argument("--camera-cmd", help="command whose stdout is a frame stream (fake camera in tests)")
    parser.add_argument("--robot-pc", help="USER@HOST of the robot PC with the D455, e.g. unitree@192.168.123.164")
    parser.add_argument("--camera-device", type=int, help="N of /dev/videoN on the robot PC (72 kamera --suchen)")
    parser.add_argument("--policy", help="HOST:PORT of 81; record then also stores the policy's predictions")
    parser.add_argument("--seconds", type=float, default=5.0, help="probe listen time and record length")
    parser.add_argument("--label", default="", help="free text for the record file; owner_motion = owner moves the arms")
    parser.add_argument("--record-factory-pose", action="store_true", help="joints step: save the factory pose (runbook M0)")
    parser.add_argument("--mode-note", default="", help="owner's answer to Q8: which robot mode the robot is in")
    parser.add_argument("--overwrite-factory-pose", action="store_true", help="replace today's record (runbook M0 rules)")
    args = parser.parse_args(argv)
    args.steps = [s.strip() for s in args.steps.split(",") if s.strip()]
    unknown = [s for s in args.steps if s not in rb.STEPS]
    if unknown:
        parser.error(f"unknown step(s) {unknown}; choose from {', '.join(rb.STEPS)}")
    if not args.dry_run and not args.iface:
        parser.error("--iface is required (the robot cable's interface, e.g. en5)")
    return args


def wait_for_sample(reader, seconds: float = WAIT_FOR_DATA_S) -> tuple:
    """Wait until the reader has a message; returns reader.latest()."""
    deadline = time.monotonic() + seconds
    while reader.latest()[0] is None and time.monotonic() < deadline:
        time.sleep(POLL_S)
    return reader.latest()


def run_probe_script(iface: str, seconds: float) -> dict:
    """71_real_probe.py in its own process (topics, LowState rate); returns its runs/real_probe.json."""
    proc = subprocess.run([sys.executable, str(PROBE_SCRIPT), "--iface", iface, "--seconds", str(seconds)], cwd=REPO,
                          capture_output=True, text=True, timeout=seconds + PROBE_TIMEOUT_EXTRA_S)
    if proc.returncode != 0:
        raise rb.StepFailed(f"71_real_probe.py exit {proc.returncode}: {proc.stderr.strip()[-ERROR_TAIL_CHARS:]}")
    return json.loads((REPO / "runs" / "real_probe.json").read_text())


def pc2_answers_ping() -> bool:
    """One ping to PC2 (`ping -c 1` works on Linux and macOS); a timeout counts as no answer."""
    try:
        return subprocess.run(["ping", "-c", "1", rb.PC2_IP], capture_output=True, timeout=PING_TIMEOUT_S).returncode == 0
    except subprocess.TimeoutExpired:
        return False


def step_probe(args, lowstate) -> rb.StepResult:
    probe = run_probe_script(args.iface, args.seconds)
    _, age = wait_for_sample(lowstate)
    writers = real_io.active_writers(args.iface, seconds=WRITER_LISTEN_S)
    result = rb.check_probe(args.iface, lowstate.rate_hz(), age, writers, args.label)
    result.details.update(probe_hz=probe.get("lowstate_hz"), topics=len(probe.get("topics", [])))
    return result


def step_joints(args, lowstate) -> rb.StepResult:
    msg, age = wait_for_sample(lowstate)
    if msg is None or age > rb.LOWSTATE_STALE_S:
        return rb.StepResult(rb.FAIL, "no fresh rt/lowstate: run the probe step first")
    result = rb.check_joints(msg)
    print(f"FACTORY ARMS: {result.details['factory_arms']}")
    for row in result.details["rows"]:
        limits = "" if row["limit_lo"] is None else f"  [{row['limit_lo']:+.3f}, {row['limit_hi']:+.3f}]"
        print(f"  {row['index']:2d} {row['name']:18s} {row['q']:+7.3f}{limits}{'' if row['inside'] else '  OUTSIDE'}")
    if result.result == rb.PASS and args.record_factory_pose:
        writers = real_io.active_writers(args.iface, topics=("rt/arm_sdk",), seconds=WRITER_LISTEN_S)
        earliest = real_io.earliest_factory_arms_today(robot=not real_io.is_loopback(args.iface))
        record = rb.record_factory_pose(real_io.factory_pose_path(args.iface), real_io.sdk_q(msg), args.iface,
                                        args.mode_note, args.overwrite_factory_pose, writers, earliest)
        for line in record.details.get("part_b", []):
            print(line)
        record.details.update(result.details)
        return record
    return result


def step_hands(args) -> rb.StepResult:
    readers = {side: real_io.HandReader(args.iface, side) for side in ("left", "right")}
    for reader in readers.values():
        wait_for_sample(reader)
    time.sleep(HAND_RATE_LISTEN_S)   # a rate read right after the first sample counts only a few samples
    states = {side: (*reader.latest(), reader.rate_hz()) for side, reader in readers.items()}
    return rb.check_hands(states)


def camera_source(args) -> tuple:
    """(command, stdin text) for the frame stream: --camera-cmd, or the grabber on the robot PC over SSH."""
    if args.camera_cmd:
        return real_camera.camera_cmd(args.camera_cmd), None
    if args.robot_pc and args.camera_device is not None:
        return real_camera.grabber_cmd(args.robot_pc, args.camera_device), real_camera.GRABBER
    raise rb.StepFailed("pass --robot-pc USER@HOST and --camera-device N (or --camera-cmd)")


def step_camera(args, stamp: str) -> rb.StepResult:
    cmd, stdin_text = camera_source(args)
    frames, stats = real_camera.grab(cmd, stdin_text)
    reference = None
    if REFERENCE_FRAME.exists():
        reference = cv2.cvtColor(cv2.imread(str(REFERENCE_FRAME)), cv2.COLOR_BGR2RGB)
    result = rb.check_camera(frames, stats, reference)
    for name, image in (("frame", frames[-1] if frames else None), ("overlay", result.details.pop("overlay", None))):
        if image is not None:
            path = REPO / "runs" / f"real_camera_{name}_{stamp}.png"
            cv2.imwrite(str(path), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            result.details[f"{name}_png"] = str(path.relative_to(REPO))
            print(f"  saved {path.relative_to(REPO)}")
    return result


def step_record(args, lowstate, stamp: str) -> rb.StepResult:
    cmd, stdin_text = camera_source(args)
    stream = real_camera.FrameStream(cmd, stdin_text)
    hands = {side: real_io.HandReader(args.iface, side) for side in ("left", "right")}
    counters = [real_io.SampleCounter(topic) for topic in real_io.ARM_CMD_TOPICS]
    try:
        wait_for_sample(lowstate)
        wait_for_sample(stream)  # the first frame takes a moment after the grabber starts
        ep = rb.record_observation(lowstate, hands, stream, args.seconds)
    finally:
        stream.close()
    writers = {c.topic: c.summary() for c in counters}
    ep.update(label=np.str_(args.label), foreign_writers=np.str_(json.dumps(writers)))
    path = OBSERVE_DIR / f"{stamp}_{args.label or 'observe'}.npz"
    save_episode(path, ep)  # first, so a policy problem below never loses the recording
    if args.policy:
        try:
            ep.update(ask_policy(args.policy, ep))
        except (policy_wire.PolicyServerError, ValueError, OSError) as problem:
            return rb.StepResult(rb.FAIL, f"policy {args.policy} failed ({problem}); the recording is saved without "
                                          f"predictions: {path.name}; check that 81 runs there", {"writers": writers})
        save_episode(path, ep)
    foreign = writers["rt/arm_sdk"]["samples"]
    if foreign and args.label != rb.OWNER_MOTION_LABEL:
        return rb.StepResult(rb.FAIL, f"{foreign} foreign rt/arm_sdk samples while recording (saved anyway: {path.name}); "
                                      "ask the owner what moved the arms", {"writers": writers})
    return rb.StepResult(rb.PASS, f"{len(ep['timestamp'])} frames -> {path.relative_to(REPO)}", {"writers": writers})


def save_episode(path: pathlib.Path, ep: dict) -> None:
    """Write the recording as a compressed npz that a reader never sees half-written (episode.write_verified)."""
    buffer = io.BytesIO()
    np.savez_compressed(buffer, **ep)
    episode.write_verified(path, buffer.getbuffer())


def ask_policy(policy: str, ep: dict) -> dict:
    """The policy's predictions for a finished recording, from 81 at HOST:PORT."""
    host, port = policy.rsplit(":", 1)
    client = policy_wire.PolicyClient(host, int(port), timeout_s=POLICY_TIMEOUT_S, connect_wait_s=POLICY_CONNECT_WAIT_S)
    try:
        return rb.policy_predictions(client, ep)
    finally:
        client.close()


def run_step(name: str, args, lowstate, stamp: str) -> rb.StepResult:
    """One step; a StepFailed, a camera, policy or OS error becomes FAIL with its message, so the report is still written."""
    try:
        if name == "net":
            return (rb.StepResult(rb.SKIP, "sim run") if args.sim else
                    rb.check_net(args.iface, real_io.interface_addresses(), pc2_answers_ping()))
        if name == "probe":
            return step_probe(args, lowstate)
        if name == "joints":
            return step_joints(args, lowstate)
        if name == "questions":
            return rb.ask_questions()
        if name == "hand-state":
            return step_hands(args)
        if name == "camera":
            return step_camera(args, stamp)
        return step_record(args, lowstate, stamp)
    except (rb.StepFailed, real_camera.CameraError, policy_wire.PolicyServerError, ValueError, subprocess.TimeoutExpired,
            OSError) as problem:
        return rb.StepResult(rb.FAIL, str(problem))


def write_report(path: pathlib.Path, report: dict) -> None:
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(report, indent=2, default=str))
    print(f"REPORT {path.relative_to(REPO)}")


def main(argv=None) -> int:
    real_io.install_publisher_guard()  # first: nothing below can create a DDS writer
    args = parse_args(argv)
    if args.dry_run:
        for name in rb.STEPS:
            print(f"STEP {name}: {'selected' if name in args.steps else 'not selected'} - {STEP_HELP[name]}")
        print("DRY RUN: no DDS started, nothing sent")
        return 0
    problem = rb.sim_flags_problem(args.iface, args.sim, args.yes_sim, real_io.robot_net_present())
    if problem:
        print(f"REFUSED: {problem}")
        return EXIT_REFUSED
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    lowstate = real_io.LowStateReader(args.iface)
    report = {"iface": args.iface, "sim": args.sim, "label": args.label, "started_at": stamp, "steps": {}}
    for name in rb.STEPS:
        result = (run_step(name, args, lowstate, stamp) if name in args.steps else
                  rb.StepResult(rb.NOT_RUN, "not selected (--steps)"))
        print(f"STEP {name}: {result.result} {result.reason}")
        report["steps"][name] = {"result": result.result, "reason": result.reason, "details": result.details}
    report["own_writers"] = real_io.own_writers(real_io.dds_participant())
    print(f"OWN WRITERS: {report['own_writers']} (must be 0)")
    write_report(real_io.BRINGUP_REPORT_DIR / f"real_bringup_{stamp}.json", report)
    failed = report["own_writers"] != 0 or any(s["result"] == rb.FAIL for s in report["steps"].values())
    return 1 if failed else 0


if __name__ == "__main__":
    # A crash (e.g. inside the DDS library) then still leaves the STEP lines printed so far and a Python trace on stderr,
    # even when stdout goes to a file. Here, not in main(): pytest's captured streams have no file descriptor.
    faulthandler.enable()
    sys.stdout.reconfigure(line_buffering=True)
    sys.exit(main())
