"""Loopback-only stand-in for the robot's sensor stream: rt/lowstate (and with --hands rt/brainco/*/state), every angle 0.

Why it exists: the self-test 89 and the push-1 check must see 84's readers receive data without a robot and without MuJoCo.
It is the ONLY LowState/hand-state test publisher in the repo (PLAN.md 3.6), and it must never put a fake rt/lowstate on the
robot network - the real controller and every reader there would take it for the robot. So it refuses unless the interface
is loopback and no 192.168.123.x address exists, checks after DDS start-up that CycloneDDS really bound the loopback
interface, and re-checks the robot address once per second, stopping when the cable appears (reviews R4-4, r5-2).
    source <prefix>/robot-env.sh
    python tests/helpers/loopback_lowstate_pub.py --iface lo0 --seconds 120 --hands      # macOS (Linux: --iface lo)
"""
import argparse
import pathlib
import sys
import time
from typing import Callable, Optional

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher  # noqa: E402
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorState_, unitree_hg_msg_dds__LowState_  # noqa: E402
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_  # noqa: E402
from unitree_sdk2py.utils.crc import CRC  # noqa: E402

from g1sim import brainco  # noqa: E402
from g1sim.real_io import is_loopback, robot_net_present  # noqa: E402

LOWSTATE_TOPIC = "rt/lowstate"
DEFAULT_HZ = 200.0         # PLAN.md 3.6; 2x the loopback refuse limit (real_io.py), held by deadline scheduling in run_loop
HAND_HZ = 100.0            # brainco_hand_service main.cpp publishes hand state at 100 Hz (PLAN.md 2.2)
NET_CHECK_S = 1.0          # review r5-2: look for the robot address once per second while publishing
CYCLONE_TRACE_LOG = pathlib.Path("/tmp/cdds.LOG")   # unitree_sdk2py core/channel_config.py: the SDK's trace file
SELECTED_PREFIX = "selected interfaces:"            # CycloneDDS 0.10 trace line naming the bound interface(s)
EXIT_REFUSED = 2


def start_problem(iface: str, robot_net: bool) -> Optional[str]:
    """Why publishing must not start, or None. Pure, so the refusal can be tested without DDS."""
    if not is_loopback(iface):
        return f"refused: {iface!r} is not a loopback interface - use --iface lo0 (macOS) or --iface lo (Linux)"
    if robot_net:
        return "refused: a 192.168.123.x address exists (robot network) - unplug the robot cable, then run again"
    return None


def bound_interfaces(trace_text: str) -> list[str]:
    """Interface names from CycloneDDS's last "selected interfaces:" trace line (empty when the line is missing)."""
    lines = [line for line in trace_text.splitlines() if SELECTED_PREFIX in line]
    if not lines:
        return []
    names = lines[-1].split(SELECTED_PREFIX, 1)[1]
    return [part.strip().split(" ")[0] for part in names.split(",") if part.strip()]


def run_loop(seconds: float, hz: float, publish_lowstate: Callable[[], None], publish_hands: Optional[Callable[[], None]],
             net_check: Callable[[], bool], clock: Callable[[], float] = time.monotonic,
             sleep: Callable[[float], None] = time.sleep) -> str:
    """Publish at `hz` (hands at HAND_HZ) for `seconds`. Returns "done", or "robot_network" when the cable appeared.

    Each publish is scheduled against a fixed deadline, so time spent in the CRC and the write (and sleep overshoot) does not
    lower the rate - a slow laptop must not fail 89's rate check with a healthy setup."""
    start = clock()
    next_net_check = start
    next_hands = start
    next_tick = start
    while clock() - start < seconds:
        now = clock()
        if now >= next_net_check:
            if net_check():
                return "robot_network"
            next_net_check = now + NET_CHECK_S
        publish_lowstate()
        if publish_hands is not None and now >= next_hands:
            publish_hands()
            next_hands += 1.0 / HAND_HZ
        next_tick += 1.0 / hz
        sleep(max(0.0, next_tick - clock()))
    return "done"


def lowstate_publish_fn() -> Callable[[], None]:
    """A function that sends one LowState_ with every motor at q = 0 (inside JOINT_LIMITS) and a rising tick."""
    pub = ChannelPublisher(LOWSTATE_TOPIC, LowState_)
    pub.Init()
    msg, crc = unitree_hg_msg_dds__LowState_(), CRC()

    def publish() -> None:
        msg.tick = (msg.tick + 1) % 2**32
        msg.crc = crc.Crc(msg)
        pub.Write(msg)
    return publish


def hands_publish_fn() -> Callable[[], None]:
    """A function that sends one open-hand state (q = 0 for all 6 motors) on both rt/brainco/<side>/state topics."""
    pubs = []
    for side in brainco.SIDES:
        pub = ChannelPublisher(brainco.TOPIC_STATE.format(side=side), MotorStates_)
        pub.Init()
        pubs.append(pub)
    msg = MotorStates_(states=[unitree_go_msg_dds__MotorState_() for _ in brainco.MOTORS])

    def publish() -> None:
        for pub in pubs:
            pub.Write(msg)
    return publish


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="loopback-only fake rt/lowstate (+ BrainCo hand state)")
    parser.add_argument("--iface", required=True, help="lo0 (macOS) or lo (Linux); anything else is refused")
    parser.add_argument("--seconds", type=float, required=True, help="how long to publish (always bounded)")
    parser.add_argument("--hz", type=float, default=DEFAULT_HZ)
    parser.add_argument("--hands", action="store_true", help="also publish rt/brainco/left|right/state at 100 Hz")
    args = parser.parse_args(argv)

    problem = start_problem(args.iface, robot_net_present())
    if problem:
        print(problem, file=sys.stderr, flush=True)
        return EXIT_REFUSED
    ChannelFactoryInitialize(0, args.iface)
    bound = bound_interfaces(CYCLONE_TRACE_LOG.read_text(errors="replace")) if CYCLONE_TRACE_LOG.exists() else []
    if bound != [args.iface]:
        print(f"refused: CycloneDDS bound {bound or 'no interface it reported'}, not only {args.iface!r} "
              f"(read from {CYCLONE_TRACE_LOG}) - nothing was published", file=sys.stderr, flush=True)
        return EXIT_REFUSED

    publish_hands = hands_publish_fn() if args.hands else None
    print(f"PUBLISHING {LOWSTATE_TOPIC} at {args.hz:.0f} Hz on {args.iface} for {args.seconds:.0f} s"
          f"{' + hand state at 100 Hz' if args.hands else ''}", flush=True)
    reason = run_loop(args.seconds, args.hz, lowstate_publish_fn(), publish_hands, robot_net_present)
    if reason == "robot_network":
        print("STOPPED: a 192.168.123.x address appeared (robot cable?) - publishing stopped at once", file=sys.stderr, flush=True)
        return EXIT_REFUSED
    print("DONE", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
