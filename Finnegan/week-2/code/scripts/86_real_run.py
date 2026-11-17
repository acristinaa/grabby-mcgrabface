"""Real G1 motion adapter (robot day, runbook Part C): moves the arms and hands over rt/arm_sdk and rt/brainco, behind the
motion gate. The ONLY script that sends commands to the robot.

Why it exists: the operators run the sim-trained policy (or a slow replay of a demo) on the real G1 without having written
this code. So every mode except `gate` first checks the motion gate (Finnegan's unlock for today and this exact code, both
operators by name, HANGING), then the preflight (fresh LowState, nobody else on rt/arm_sdk, joints in range, arms at the
factory pose recorded at M0), and only then creates a writer. Every run ends with the arms handed back to Unitree's controller.
Modes (PLAN.md 3.4):
    gate                       no DDS: MOTION: UNLOCKED | LOCKED + one line per problem (exit 0 / 2)
    release    --iface IF      weight 1 at the measured angles, slowly to the factory pose, ramp out (first step after a crash)
    recover    --iface IF      like release, but home on our waypoint path from where the last run's log says the arms were
    hands-open --iface IF      both hands open at speed 0.3 for 2 s, no arm command
    ready      --iface IF [--hold 5]                      ramp in, rise to the ready pose, hold, go home
    replay     --iface IF --server HOST:PORT --episode STEM [--speed 0.25] [--max-seconds 120] camera opts | --no-camera
    shadow     --iface IF --server HOST:PORT [--max-seconds 30] camera opts       policy runs, arms hold the ready pose
    run        --iface IF --server HOST:PORT [--speed 0.25] [--max-seconds 60] camera opts
    camera opts: --robot-pc USER@HOST --camera-device N | --camera-cmd "CMD"
    common: --waist-max RAD (0.15, at most 0.6) | --waist-hold; --no-hands; --out DIR (runs/real); --label TEXT; --yes-sim
Last lines: `86 <MODE>: DONE | REFUSED <reason> | SOFT EXIT <reason> | FAULT EXIT <reason>` and `LOG <npz>`; exit 0/2/3/4.
Press Ctrl+C ONCE to stop (slow way home); twice freezes the arms (only if an arm moves towards a person); once more in a
freeze hands them back where they are (real_safety.FAULT_ACTIONS); never kill -9 or close the terminal.
Robot env (Python 3.10), from the repo root, e.g.:  caffeinate -i python scripts/86_real_run.py ready --iface en5
"""
import argparse
import signal
import sys
import pathlib

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
# Every module 86 runs is imported here, at the top (no lazy imports): the motion gate compares exactly these files with
# the unlocked commit (PLAN.md 4.3 (b)).
from g1sim import motion_gate, policy_wire, real_camera, real_io, real_preflight, real_run, real_safety, real_send  # noqa: E402

POLICY_CONNECT_WAIT_S = 10.0     # PolicyClient retries the connect this long (task B2: connect_wait_s=10)
STOP_SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)   # Ctrl+C, kill, terminal closed: all mean "stop" (rule 12)
MODES = ("gate", "release", "recover", "hands-open", "ready", "replay", "shadow", "run")
CODE_WORDS = {real_safety.EXIT_CODES["done"]: "DONE", real_safety.EXIT_CODES["refused"]: "REFUSED",
              real_safety.EXIT_CODES["soft"]: "SOFT EXIT", real_safety.EXIT_CODES["fault"]: "FAULT EXIT"}  # runbook's last line
POLICY_MODES = ("replay", "shadow", "run")


class SignalCounter:
    """Counts stop signals instead of letting them kill the process mid-message; the runner reads the count each tick."""

    def __init__(self):
        self.count = 0

    def install(self) -> None:
        for sig in STOP_SIGNALS:
            signal.signal(sig, self._on_signal)

    def _on_signal(self, signum, frame) -> None:
        self.count += 1


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Real G1 motion adapter (runbook Part C) - only behind the motion gate")
    modes = parser.add_subparsers(dest="mode", required=True)
    modes.add_parser("gate", help="show whether motion is unlocked (no DDS)")
    common = _common_options()
    camera = _camera_options()
    modes.add_parser("release", parents=[common], help="hand the arms back slowly via the factory pose")
    modes.add_parser("recover", parents=[common], help="after a crash at the table: home on our waypoint path, hand back")
    modes.add_parser("hands-open", parents=[common], help="open both hands")
    ready = modes.add_parser("ready", parents=[common], help="rise to the ready pose, hold, go home")
    ready.add_argument("--hold", type=float, default=5.0, help="seconds at the ready pose (default 5)")
    replay = modes.add_parser("replay", parents=[common, camera], help="play a recorded demo slowly")
    _policy_options(replay, speed=True, max_seconds=120.0)
    replay.add_argument("--episode", required=True, help="episode stem the replay server serves, e.g. replay_demo")
    replay.add_argument("--no-camera", action="store_true", help="send a blank image (81 replay ignores images)")
    _policy_options(modes.add_parser("shadow", parents=[common, camera], help="policy runs, nothing it says is sent"),
                    speed=False, max_seconds=30.0)
    _policy_options(modes.add_parser("run", parents=[common, camera], help="the VLA moves the arms"),
                    speed=True, max_seconds=60.0)
    args = parser.parse_args(argv)
    problem = _option_problem(args)
    if problem:
        parser.error(problem)
    return args


def _common_options() -> argparse.ArgumentParser:
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--iface", required=True, help="robot cable interface, e.g. en5 (lo0 / lo for the sim)")
    waist = common.add_mutually_exclusive_group()
    waist.add_argument("--waist-max", type=float, default=real_safety.ROBOT_WAIST_MAX_DEFAULT,
                       help=f"largest waist yaw in rad (default {real_safety.ROBOT_WAIST_MAX_DEFAULT}, "
                            f"at most {real_safety.ROBOT_WAIST_MAX_CEILING} on the robot)")
    waist.add_argument("--waist-hold", action="store_true", help="keep the waist where it is (policy waist logged only)")
    common.add_argument("--no-hands", action="store_true", help="no hand state, no hand commands")
    common.add_argument("--out", default=real_io.RUN_LOG_DIR, help=f"log folder (default {real_io.RUN_LOG_DIR})")
    common.add_argument("--label", default="", help="free text stored in the log")
    common.add_argument("--yes-sim", action="store_true", help="loopback sim run: skips the unlock and the confirmations")
    return common


def _camera_options() -> argparse.ArgumentParser:
    camera = argparse.ArgumentParser(add_help=False)
    camera.add_argument("--robot-pc", help="USER@HOST of the robot PC with the D455, e.g. unitree@192.168.123.164")
    camera.add_argument("--camera-device", type=int, help="N of /dev/videoN on the robot PC")
    camera.add_argument("--camera-cmd", help="command whose stdout is a frame stream (fake camera in the sim)")
    return camera


def _policy_options(parser: argparse.ArgumentParser, speed: bool, max_seconds: float) -> None:
    parser.add_argument("--server", required=True, help="HOST:PORT of the policy server 81")
    parser.add_argument("--max-seconds", type=float, default=max_seconds, help=f"policy phase length (default {max_seconds:g})")
    if speed:
        parser.add_argument("--speed", type=float, default=0.25, help="playback speed factor (default 0.25, at most 0.5)")


def _option_problem(args: argparse.Namespace) -> str | None:
    if args.mode not in POLICY_MODES:
        return None
    if ":" not in args.server:
        return "--server must be HOST:PORT, e.g. 127.0.0.1:8765"
    has_camera = bool(args.camera_cmd) or args.robot_pc is not None
    if args.robot_pc is not None and args.camera_device is None:
        return "--robot-pc needs --camera-device N"
    if not has_camera and not getattr(args, "no_camera", False):
        return "give camera options (--robot-pc + --camera-device, or --camera-cmd)" + \
            (" or --no-camera" if args.mode == "replay" else "")
    return None


def run_args(args: argparse.Namespace, gate: dict) -> real_run.RunArgs:
    """argparse options -> the runner's RunArgs. A mode without --hold, --speed or --max-seconds never reads that field,
    so its fallback value here is never used (shadow plays at real_run_policy.SHADOW_SPEED)."""
    return real_run.RunArgs(iface=args.iface, hold=getattr(args, "hold", 5.0), speed=getattr(args, "speed", 1.0),
                            max_seconds=getattr(args, "max_seconds", 0.0), episode=getattr(args, "episode", None),
                            waist_max=args.waist_max, waist_hold=args.waist_hold, no_hands=args.no_hands, out=args.out,
                            label=args.label, gate=gate)


def gate_mode() -> int:
    """`86 gate`: no DDS, only the unlock file and the loaded files."""
    problems = motion_gate.gate_problems(REPO, motion_gate.loaded_repo_files(REPO))
    print("MOTION: LOCKED" if problems else "MOTION: UNLOCKED")
    for problem in problems:
        print(f"  {problem}")
    return real_safety.EXIT_CODES["refused"] if problems else real_safety.EXIT_CODES["done"]


def check_gate(args: argparse.Namespace, loaded: list) -> tuple:
    """(gate dict for the sidecar, problem or None): the sim bypass, or checks (a)-(d) of PLAN.md 4.3."""
    files = [str(p) for p in loaded]
    if args.yes_sim:
        problem = motion_gate.sim_bypass_problem(args.iface, args.yes_sim)
        return {"checks": "sim", "operators": None, "loaded_files": files}, problem
    gate = {"checks": "unlocked", "operators": None, "loaded_files": files}
    problems = motion_gate.gate_problems(REPO, loaded)
    if problems:
        return gate, "MOTION: LOCKED - " + "; ".join(problems)
    answers = []
    problem = motion_gate.confirm_operators(lambda prompt: _remember(answers, input(prompt)))
    gate["operators"] = answers[:2]
    return gate, problem or motion_gate.confirm_hanging(input)   # input looked up now, so a test can type the answer


def _remember(answers: list, text: str) -> str:
    answers.append(text.strip())
    return text


def print_table_note(args: argparse.Namespace) -> None:
    """replay/shadow/run, before the prompts: the FACTORY ARMS line and the table reminder (RV6-1; 86 cannot see the table)."""
    if args.mode not in POLICY_MODES:
        return
    record = real_io.load_factory_record(real_io.factory_pose_path(args.iface))
    summary = real_io.factory_arm_summary(record["q"]) if record else "no factory pose recorded today (runbook M0)"
    print(f"FACTORY ARMS: {summary}")
    print("table steps: check the runbook M4 lookup")


def build_io(args: argparse.Namespace, signals: SignalCounter) -> real_run.RunIO:
    """The real readers, camera and policy client; senders only as factories (created after preflight)."""
    iface = args.iface
    lowstate = real_io.LowStateReader(iface)    # first: it starts DDS on this interface
    hands = {} if args.no_hands else {side: real_io.HandReader(iface, side) for side in ("left", "right")}
    policy = open_policy(args)                   # before the camera: a refused connect leaves no camera process behind
    return real_run.RunIO(
        lowstate=lowstate, hands=hands, frames=open_camera(args), policy=policy,
        make_arm_sender=lambda: real_send.ArmSdkSender(iface),
        make_hand_senders=lambda: {side: real_send.HandSender(side) for side in ("left", "right")},
        foreign_samples=lambda: real_io.active_writers(iface),
        factory_record=real_io.load_factory_record(real_io.factory_pose_path(iface)),
        signal_count=lambda: signals.count,
        last_arm_path=real_io.last_arm_path(real_preflight.resolve_out_dir(args.out)))


def open_camera(args: argparse.Namespace):
    """The head-camera stream for replay/shadow/run (the D455 grabber over SSH, or --camera-cmd); None otherwise."""
    if args.mode not in POLICY_MODES or getattr(args, "no_camera", False):
        return None
    if args.camera_cmd:
        return real_camera.FrameStream(real_camera.camera_cmd(args.camera_cmd))
    return real_camera.FrameStream(real_camera.grabber_cmd(args.robot_pc, args.camera_device), real_camera.GRABBER)


def open_policy(args: argparse.Namespace):
    """The policy client for replay/shadow/run, with the 2 s socket timeout of rule 10; None otherwise."""
    if args.mode not in POLICY_MODES:
        return None
    host, port = args.server.rsplit(":", 1)
    return policy_wire.PolicyClient(host, int(port), timeout_s=real_safety.POLICY_TIMEOUT_S,
                                    connect_wait_s=POLICY_CONNECT_WAIT_S)


def report(mode: str, result: real_run.RunResult) -> int:
    """The two last lines the runbook reads."""
    done = result.exit_code == real_safety.EXIT_CODES["done"]
    print(f"86 {mode.upper()}: {CODE_WORDS[result.exit_code]}" + ("" if done else f" {result.reason}"))
    print(f"LOG {result.log_path}")
    return result.exit_code


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.mode == "gate":
        return gate_mode()
    gate = {"checks": None, "operators": None, "loaded_files": []}
    problem = real_preflight.argument_problem(args.mode, run_args(args, gate))
    if problem is None:
        print_table_note(args)
        gate, problem = check_gate(args, motion_gate.loaded_repo_files(REPO))
    if problem:
        return report(args.mode, real_run.record_refusal(args.mode, run_args(args, gate), problem))
    signals = SignalCounter()
    try:
        io = build_io(args, signals)
    except (OSError, real_camera.CameraError) as error:
        reason = f"could not start: {error} - is the policy server / camera running? (runbook M3-M6)"
        return report(args.mode, real_run.record_refusal(args.mode, run_args(args, gate), reason))
    signals.install()              # from here on a signal means "stop slowly", never "die mid-message"
    try:
        return report(args.mode, real_run.run_mode(args.mode, io, run_args(args, gate)))
    finally:
        for resource in (io.frames, io.policy):
            if resource is not None:
                resource.close()


if __name__ == "__main__":
    sys.exit(main())
