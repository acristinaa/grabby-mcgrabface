"""Robot-day self-test: does this laptop have everything for Monday, with no robot connected? Ends with SELFTEST: PASS|FAIL.

Why: Cristina and Saad set up their laptops alone on Sunday evening. This one command tells them whether the robot env, DDS on
loopback, the camera byte path, the read-only checklist 84 and (on the laptop that will serve the model) the policy all work,
so Monday's hour at the robot is not spent on installs. It sends nothing to any robot:
  - the no-writer guard is installed before anything else (PLAN.md 4.1 rule 1 (c));
  - the DDS check refuses while a 192.168.123.x address exists and uses the loopback helper (a separate process) as its source.
Checks: versions, dds (loopback LowState + hand state through 84's readers), camera (fake_camera through real_camera),
bringup (84 --dry-run), imports (no motion module loaded here), policy (81 --self-test from the policy env, unless --skip-policy),
motion (86 gate in its own process, if 86 is in this checkout: its MOTION line is information, never a FAIL).
    source <prefix>/robot-env.sh
    python scripts/89_robot_day_selftest.py [--policy-device mps|cpu|cuda] [--skip-policy]
"""
import argparse
import json
from importlib import metadata
import os
import pathlib
import platform
import shutil
import subprocess
import sys
import time

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim import real_io  # noqa: E402

real_io.install_publisher_guard()  # first: from here on nothing in this process can create a DDS writer

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from g1sim import episode, real_camera  # noqa: E402

PINNED = {"python": "3.10", "numpy": "2.2.6", "cyclonedds": "0.10.2", "cv2": "4.13.0"}   # robot env pins, PLAN.md 2.7
FORBIDDEN_MODULES = ("g1sim.real_send", "g1sim.real_run", "g1sim.motion_gate", "g1sim.brainco_env", "g1sim.teleop",
                     "lerobot")                 # PLAN.md 4.1 rule 1 (b): read-only tools never load motion code
LOOPBACK_IFACE = "lo0" if sys.platform == "darwin" else "lo"   # macOS names it lo0, Linux lo
PUB_SECONDS = 10.0          # loopback helper run time; covers discovery + the rate window with margin
FIRST_SAMPLE_WAIT_S = 5.0   # discovery on loopback takes well under 1 s; 5 s = clearly broken
RATE_WINDOW_S = 2.0         # rate measured over 2 s (same window 84 uses for writer samples, PLAN.md 3.1)
CAMERA_WAIT_S = 10.0        # fake_camera starts in about 1 s; 10 s = clearly broken
DRY_RUN_TIMEOUT_S = 60.0    # 84 --dry-run only imports and lists its steps (a few s); 60 s = clearly hung
POLICY_TIMEOUT_S = 1200.0   # first CPU load of the 2.7 GB backbone on a slow laptop; normal is 1-3 min
GATE_TIMEOUT_S = 60.0       # 86 gate only reads the unlock file and runs git (a few s, no DDS); 60 s = clearly hung
SELFTEST_NPZ = REPO / "tests" / "data" / "selftest_episode.npz"
LOOPBACK_PUB = REPO / "tests" / "helpers" / "loopback_lowstate_pub.py"
FAKE_CAMERA = REPO / "tests" / "helpers" / "fake_camera.py"
BRINGUP = REPO / "scripts" / "84_real_bringup.py"
SERVER = REPO / "scripts" / "81_smolvla_server.py"
MOTION_RUN = REPO / "scripts" / "86_real_run.py"    # only in the checkout when push 2 carried the motion code (PLAN.md 6.4)
PREFIX_ENV = "ROBOT_DAY_PREFIX"                      # exported by <prefix>/robot-env.sh (scripts/setup_robot_laptop.sh)
DEFAULT_PREFIX = pathlib.Path.home() / "g1-robot-day"   # setup_robot_laptop.sh default --prefix
CHECKPOINT = pathlib.Path("smolvla-runs/run1/checkpoints/040000/pretrained_model")   # where the release is unpacked
HF_CACHE = pathlib.Path("hf_cache/hub")             # the prefix's own model cache; policy-env.sh exports it (PLAN.md R3-1)
BACKBONE_DIR = "models--HuggingFaceTB--SmolVLM2-500M-Video-Instruct"   # SmolVLA's vision-language base model (PLAN.md R3-1)


def check_versions() -> tuple[str, str]:
    """The robot env's pinned versions: cyclonedds 0.10.2 only has wheels up to Python 3.10 (PLAN.md 2.7)."""
    found = {"python": platform.python_version(), "numpy": np.__version__,
             "cyclonedds": metadata.version("cyclonedds"), "cv2": cv2.__version__}
    wrong = [f"{name} {found[name]} (want {want})" for name, want in PINNED.items() if not found[name].startswith(want)]
    detail = ", ".join(f"{name} {value}" for name, value in found.items())
    if wrong:
        return "FAIL", f"{', '.join(wrong)} - run 89 from the robot env: source <prefix>/robot-env.sh"
    return "PASS", detail


def check_dds() -> tuple[str, str]:
    """84's readers receive the loopback helper's LowState and hand state at a rate 84 accepts."""
    if real_io.robot_net_present():
        return "FAIL", "a 192.168.123.x address exists - unplug the robot cable (89 runs with no robot), then run 89 again"
    helper = subprocess.Popen([sys.executable, str(LOOPBACK_PUB), "--iface", LOOPBACK_IFACE, "--seconds", str(PUB_SECONDS),
                               "--hands"], cwd=REPO, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        lowstate = real_io.LowStateReader(LOOPBACK_IFACE)
        hands = [real_io.HandReader(LOOPBACK_IFACE, side) for side in ("left", "right")]
        deadline = time.monotonic() + FIRST_SAMPLE_WAIT_S
        while lowstate.latest()[0] is None and time.monotonic() < deadline:
            time.sleep(0.1)
        if lowstate.latest()[0] is None:
            return "FAIL", f"no LowState within {FIRST_SAMPLE_WAIT_S:.0f} s on {LOOPBACK_IFACE} (helper: {exit_text(helper)})"
        time.sleep(RATE_WINDOW_S)
        hz = lowstate.rate_hz(RATE_WINDOW_S)
        state = real_io.state27(lowstate.latest()[0], hands[0].latest()[0], hands[1].latest()[0])
    finally:
        helper.terminate()
        helper.wait()
    if real_io.lowstate_hz_check(LOOPBACK_IFACE, hz) == "refuse":
        return "FAIL", f"LowState at {hz:.0f} Hz, below what 84 accepts on loopback - laptop overloaded? close other programs"
    if not np.isfinite(state).all():
        return "FAIL", "LowState arrived but no BrainCo hand state - firewall blocking? (runbook Part A, firewall step)"
    return "PASS", f"LowState {hz:.0f} Hz + hand state on {LOOPBACK_IFACE}, 27-D state finite"


def check_camera() -> tuple[str, str]:
    """A recorded frame survives the grabber byte format and real_camera's reader unchanged (the D455 path minus the D455)."""
    expected = episode.load(SELFTEST_NPZ)["head_image"]
    stream = real_camera.FrameStream([sys.executable, str(FAKE_CAMERA), "--npz", str(SELFTEST_NPZ), "--stop-after",
                                      str(CAMERA_WAIT_S)])
    try:
        deadline = time.monotonic() + CAMERA_WAIT_S
        frame = None
        while frame is None and time.monotonic() < deadline:
            frame, _ = stream.latest()
            time.sleep(0.1)
    finally:
        stream.close()
    if frame is None:
        return "FAIL", f"no frame from fake_camera within {CAMERA_WAIT_S:.0f} s"
    if not any(np.array_equal(frame, recorded) for recorded in expected):
        return "FAIL", f"frame {getattr(frame, 'shape', '?')} differs from every recorded frame - byte format broken"
    return "PASS", f"frame {frame.shape} {frame.dtype} identical to the recording"


def check_bringup() -> tuple[str, str]:
    """84 --dry-run starts in this env and lists its steps (exit 0)."""
    proc = subprocess.run([sys.executable, str(BRINGUP), "--dry-run"], cwd=REPO, capture_output=True, text=True,
                          timeout=DRY_RUN_TIMEOUT_S)
    if proc.returncode != 0:
        return "FAIL", f"84 --dry-run exit {proc.returncode}: {last_line(proc.stderr or proc.stdout)}"
    return "PASS", "84 --dry-run exit 0"


def check_imports() -> tuple[str, str]:
    """No motion module was loaded by anything this self-test ran in its own process."""
    loaded = [name for name in FORBIDDEN_MODULES if name in sys.modules]
    if loaded:
        return "FAIL", f"motion modules loaded: {loaded} - do not use this laptop's checkout, tell Finnegan"
    return "PASS", "none of " + ", ".join(FORBIDDEN_MODULES)


def default_policy_device() -> str:
    """mps on an Apple-silicon Mac, cuda when an NVIDIA driver is installed, else cpu (ruling R11: where 81 can run)."""
    if sys.platform == "darwin" and platform.machine() == "arm64":
        return "mps"
    return "cuda" if shutil.which("nvidia-smi") else "cpu"


def check_policy(device: str, prefix: pathlib.Path) -> tuple[str, str]:
    """The released 40k checkpoint answers one recorded frame through 81 --self-test from the policy env."""
    python, checkpoint, cache = prefix / "policy-env" / "bin" / "python", prefix / CHECKPOINT, prefix / HF_CACHE
    if not python.exists() or not checkpoint.is_dir():
        return "FAIL", (f"policy env or model missing under {prefix} - run setup_robot_laptop.sh --policy-env --release "
                        f"robot-day-2026-10-18, or use --skip-policy on a laptop that will not run 81")
    if not (cache / BACKBONE_DIR).is_dir():
        return "FAIL", f"backbone not in the prefix ({cache / BACKBONE_DIR} missing) - re-run setup_robot_laptop.sh --policy-env"
    env = dict(os.environ, HF_HUB_CACHE=str(cache), HF_HUB_OFFLINE="1")
    log = REPO / "runs" / f"robot_day_selftest_policy_{device}.log"
    log.parent.mkdir(exist_ok=True)
    start = time.monotonic()
    with open(log, "w") as out:
        code = subprocess.run([str(python), str(SERVER), "--checkpoint", str(checkpoint), "--self-test", str(SELFTEST_NPZ),
                               "--device", device], cwd=REPO, env=env, stdout=out, stderr=subprocess.STDOUT,
                              timeout=POLICY_TIMEOUT_S).returncode
    text = log.read_text(errors="replace")
    timing = next((line for line in text.splitlines() if line.startswith("load_s")), "no timing line")
    if code != 0 or "SELF-TEST PASS" not in text:
        return "FAIL", f"81 --self-test on {device} exit {code}; full output in {log.relative_to(REPO)}"
    return "PASS", f"81 --self-test on {device} in {time.monotonic() - start:.0f} s ({timing}), cache {cache}"


def check_motion_gate() -> tuple[str, str]:
    """86 gate's MOTION line, so the operators see at 08:00 whether Part C is allowed today. Runs 86 as a separate process, so
    no motion module enters this one; LOCKED is a normal day (Parts A+B), so this check never FAILs."""
    if not MOTION_RUN.exists():
        return "INFO", "86 not in this checkout (motion code not pushed) - today is Parts A and B only"
    proc = subprocess.run([sys.executable, str(MOTION_RUN), "gate"], cwd=REPO, capture_output=True, text=True,
                          timeout=GATE_TIMEOUT_S)
    motion = next((line for line in proc.stdout.splitlines() if line.startswith("MOTION:")), None)
    if motion is None:
        return "INFO", f"86 gate printed no MOTION line (exit {proc.returncode}: {last_line(proc.stderr or proc.stdout)})"
    return "INFO", motion


def skipped_policy() -> tuple[str, str]:
    """The policy check's result on a laptop that will not serve the model (--skip-policy)."""
    return "SKIP", "--skip-policy"


def exit_text(proc: subprocess.Popen) -> str:
    """The helper's state for an error message: still running, or its exit code and last line."""
    if proc.poll() is None:
        return "still running"
    return f"exit {proc.returncode}: {last_line(proc.stdout.read() if proc.stdout else '')}"


def last_line(text: str) -> str:
    """The last non-empty line of a program's output, so a FAIL line shows the actual error."""
    lines = [line for line in text.strip().splitlines() if line.strip()]
    return lines[-1] if lines else "(no output)"


def main() -> int:
    parser = argparse.ArgumentParser(description="robot-day self-test (no robot connected; sends nothing)")
    parser.add_argument("--policy-device", choices=("mps", "cpu", "cuda"), default=default_policy_device())
    parser.add_argument("--skip-policy", action="store_true", help="on a laptop that will not run the policy server 81")
    args = parser.parse_args()
    prefix = pathlib.Path(os.environ.get(PREFIX_ENV, DEFAULT_PREFIX))

    def policy_check() -> tuple[str, str]:
        """The policy check with this run's device and prefix, or SKIP on a laptop that will not run 81."""
        return skipped_policy() if args.skip_policy else check_policy(args.policy_device, prefix)

    checks = {"versions": check_versions, "dds": check_dds, "camera": check_camera, "bringup": check_bringup,
              "imports": check_imports, "policy": policy_check, "motion": check_motion_gate}
    results = {}
    for name, check in checks.items():
        try:
            results[name] = check()
        except Exception as error:  # one broken check must not hide the others; it still fails the self-test
            results[name] = ("FAIL", f"{type(error).__name__}: {error}")
        print(f"CHECK {name}: {results[name][0]} {results[name][1]}", flush=True)

    passed = all(status != "FAIL" for status, _ in results.values())
    report = REPO / "runs" / f"robot_day_selftest_{time.strftime('%Y%m%d_%H%M%S')}.json"
    report.parent.mkdir(exist_ok=True)
    report.write_text(json.dumps({"passed": passed, "prefix": str(prefix), "policy_device": args.policy_device,
                                  "checks": {name: {"status": s, "detail": d} for name, (s, d) in results.items()}}, indent=2))
    print(f"report: {report.relative_to(REPO)}")
    print("SELFTEST:", "PASS" if passed else "FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
