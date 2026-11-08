"""Real G1: the 50 Hz phase runner of 86 - preflight, ramps, scripted moves, the policy phase and every exit (PLAN.md 3.4, 4.2).

Why it exists: this is the code that decides what goes out on rt/arm_sdk each 20 ms. All I/O (LowState and hand readers,
camera, senders, policy client, clock, signals) comes in through RunIO, so the unit tests drive the very same code with
fakes. Every way out ends with the arms handed back to Unitree's controller at weight 0, or held still until the operator
acts: a soft exit walks home slowly on the waypoint paths of g1sim/arm_paths.py (RV6-1, SAFE-PATHS.md), a fault holds,
recovers on those paths or ramps out, by reason (real_safety.FAULT_ACTIONS); both exit paths live in g1sim/real_run_exits.py.
"""
import dataclasses
import pathlib
import time
import traceback
from typing import Callable

import numpy as np

from g1sim import action_space, motion_gate, policy_wire, real_io, real_preflight as pre, real_run_exits as exits
from g1sim import real_run_modes as modes, real_safety as rs
from g1sim.real_run_exits import ExitRequest
from g1sim.real_run_log import RunLog
from g1sim.real_run_policy import ThreadedWorker
from g1sim.real_send import FLOAT_SLACK

REPO = pathlib.Path(__file__).resolve().parent.parent
DT = 1.0 / rs.SEND_HZ
LEAD_MODES = ("replay", "run")                      # modes that print the MAX LEAD line (PLAN.md task B2 step 1b)


@dataclasses.dataclass
class RunIO:
    """Everything the runner talks to. Senders are FACTORIES: the runner creates them only after preflight passed and the
    sidecar marked the coming writer (R2-2 (ii)), so a refused run never had a DDS writer."""
    lowstate: object                                # .latest() -> (LowState | None, age_s); .rate_hz(window_s)
    hands: dict                                     # side -> hand-state reader; {} with --no-hands
    frames: object | None                           # camera FrameStream (.latest()); None = no camera
    make_arm_sender: Callable[[], object]           # -> .send(weight, q17, kp17, kd17)
    make_hand_senders: Callable[[], dict]           # -> {side: .send(targets6, speed)}
    policy: object | None                           # PolicyClient (.metadata, .reset, .infer) for replay/shadow/run
    foreign_samples: Callable[[], dict]             # real_io.active_writers(iface): listens 2 s
    factory_record: dict | None                     # real_io.load_factory_record(...)
    clock: Callable[[], float] = time.monotonic
    sleep: Callable[[float], None] = time.sleep
    signal_count: Callable[[], int] = lambda: 0     # Ctrl+C / SIGTERM / SIGHUP seen so far
    make_worker: Callable[[], object] = ThreadedWorker
    last_arm_path: dict | None = None               # real_io.last_arm_path(...): where our last run left the arms (recover)


@dataclasses.dataclass
class RunArgs:
    """The 86 options the runner uses (defaults as the 86 CLI, PLAN.md 3.4)."""
    iface: str
    hold: float = 5.0
    speed: float = 0.25
    max_seconds: float = 60.0
    episode: str | None = None
    waist_max: float = rs.ROBOT_WAIST_MAX_DEFAULT
    waist_hold: bool = False
    no_hands: bool = False
    out: pathlib.Path | str = real_io.RUN_LOG_DIR      # off loopback only RUN_LOG_DIR itself is allowed (R3-5)
    label: str = ""
    gate: dict = dataclasses.field(default_factory=dict)


@dataclasses.dataclass
class RunResult:
    """How one 86 run ended: exit code (real_safety.EXIT_CODES), the reason the runbook shows, and the npz log path."""
    exit_code: int
    reason: str
    log_path: pathlib.Path | None


def weight_step_problem(previous: float, weight: float, release_first: bool) -> str | None:
    """m6-8: no message may change the weight by more than MAX_WEIGHT_STEP - except release's first message, the jump to
    weight 1 at the measured angles (Unitree arm7 example line 135). A bigger step would snap the arm."""
    if release_first or abs(weight - previous) <= rs.MAX_WEIGHT_STEP + FLOAT_SLACK:
        return None
    return f"weight step {previous:.3f} -> {weight:.3f} is larger than {rs.MAX_WEIGHT_STEP}"


def run_mode(mode: str, io: RunIO, args: RunArgs) -> RunResult:
    """Run one 86 mode (release, hands-open, ready, replay, shadow, run) to its end; never raises for a robot problem."""
    return Runner(mode, io, args).run()


def max_lead_line(lead: tuple | None) -> str:
    """The `MAX LEAD <rad> (<joint>)` line of replay/run: the runbook copies it from M3, and M4 at the table needs it
    below 0.15 rad (PLAN.md 4.2 rule 16 (c)). `lead` comes from RunLog.max_policy_lead()."""
    if lead is None:
        return "MAX LEAD none (no policy message was sent at weight 1)"
    value, joint = lead
    return f"MAX LEAD {value:.3f} rad ({real_io.MOTOR_NAMES[rs.SDK_MOTORS[joint]]})"


def record_refusal(mode: str, args: RunArgs, reason: str) -> RunResult:
    """Log a run that 86 refused before the runner started (gate, prompts, server); no writer ever existed."""
    log = RunLog(_log_dir(args), mode, _source(args.iface), _logged_args(args), args.gate)
    path = log.finish(rs.EXIT_CODES["refused"], reason, None, motion_gate.loaded_repo_files(REPO), policy_wire.TASK)
    return RunResult(rs.EXIT_CODES["refused"], reason, path)


def _log_dir(args: RunArgs):
    """args.out - unless the run is refused for its --out (R3-5): then the log goes to RUN_LOG_DIR, so the refused folder
    gets no files at all (B2 review r2 minor 3)."""
    if pre.out_dir_problem(args.out, real_io.is_loopback(args.iface)):
        return real_io.RUN_LOG_DIR
    return args.out


def _source(iface: str) -> str:
    return "sim85" if real_io.is_loopback(iface) else "real"


def _logged_args(args: RunArgs) -> dict:
    """The options for the sidecar's "args" (the gate has its own field)."""
    return {key: value for key, value in dataclasses.asdict(args).items() if key != "gate"}


class Runner:
    """One run. Phases call tick() once per 50 Hz message; tick raises ExitRequest when the run must end, and the exits of
    g1sim/real_run_exits.py walk the arms home."""

    def __init__(self, mode: str, io: RunIO, args: RunArgs):
        self.mode, self.io, self.args = mode, io, args
        self.log = RunLog(_log_dir(args), mode, _source(args.iface), _logged_args(args), args.gate)
        self.arm, self.hand_senders = None, {}
        self.weight, self.sent_arm = 0.0, False      # last weight sent; whether any arm message went out
        self.command = self.measured = self.q0 = self.ready17 = self.filter = None
        self.hand_target, self.hand_speed = None, rs.HAND_SPEED
        self.phase, self.in_exit, self.in_fault, self.in_release = None, False, False, False
        self.fault_stage = None                      # in a fault: "hold", "recover" or "ramp_out" (real_run_exits.fault_exit)
        self.arm_path = None                         # where the arms are on our paths (log_arm_path)
        self.signals_at_start = self.signals_seen = io.signal_count()
        self.exit_deadline, self.lead = None, 0.0
        self.start_t = self.next_t = io.clock()

    # --- top level ------------------------------------------------------------------------------------------------------

    def run(self) -> RunResult:
        problem = pre.run_preflight(self.mode, self.io, self.args, self.note_event, self.say)
        if problem is None and self.io.signal_count() > self.signals_seen:
            problem = "stopped by the operator before anything was sent"
        if problem is not None:
            return self._finish("refused", problem)
        if self.mode == "recover":                  # a kill before recover logs its own path must not lose the old one
            self.log.sidecar["arm_path"] = self.io.last_arm_path
        self.log.mark_writer_coming(arm_sdk_writer=self.mode != "hands-open")
        kind, reason = "fault", "internal_error"
        try:
            kind, reason = self._drive()
        finally:
            result = self._finish(kind, reason)
        return result

    def _drive(self) -> tuple:
        try:
            self._create_writers()
            modes.MODE_BODIES[self.mode](self)
        except ExitRequest as request:
            kind, reason = request.kind, request.reason
        except Exception as error:  # a bug must still end with the arms handed back
            self.say(traceback.format_exc())
            kind, reason = "fault", f"internal_error: {type(error).__name__}: {error}"
        if kind == "soft":
            kind, reason = exits.soft_exit(self, reason)
        if kind == "fault":
            exits.fault_exit(self, reason)
        return kind, reason

    def _create_writers(self) -> None:
        if self.mode != "hands-open":
            self.arm = self.io.make_arm_sender()
        if self.mode not in modes.ARM_ONLY_MODES and not self.args.no_hands:
            self.hand_senders = self.io.make_hand_senders()

    def _finish(self, kind: str, reason: str) -> RunResult:
        if kind == "refused":
            code = rs.EXIT_CODES["refused"]
        elif kind == "fault":
            code = rs.EXIT_CODES["fault"]
        else:
            code = rs.EXIT_CODES["done"] if reason == "done" else rs.EXIT_CODES["soft"]
        if self.mode in LEAD_MODES and self.sent_arm:
            self.say(max_lead_line(self.log.max_policy_lead()))
        path = self.log.finish(code, reason, self.weight if self.sent_arm else None,
                               motion_gate.loaded_repo_files(REPO), policy_wire.TASK)
        return RunResult(code, reason, path)

    # --- building blocks used by g1sim/real_run_modes.py and PolicyPhase ------------------------------------------------

    def new_filter(self, q0, waist_max: float | None = None, waist_hold: bool | None = None) -> rs.SafetyFilter:
        """A SafetyFilter with the robot's joint limits and this run's waist options."""
        return rs.SafetyFilter(pre.LIMITS_LO, pre.LIMITS_HI, q0,
                               self.args.waist_max if waist_max is None else waist_max,
                               self.args.waist_hold if waist_hold is None else waist_hold)

    def tick(self, phase: str, weight: float, target17, release_first: bool = False, filtered: bool = True) -> None:
        """One 50 Hz message: staleness and signal checks, SafetyFilter, weight-step check, send, log, wait."""
        measured = self.measured_now()
        self.check_signals()
        command = np.asarray(target17, dtype=float)
        if filtered:
            try:
                command, events = self.filter.step(target17, measured, phase, weight)
            except rs.InvalidTarget:
                raise ExitRequest("fault", "invalid_target") from None
            self._react(events, phase)
        problem = weight_step_problem(self.weight, weight, release_first)
        if problem:
            raise ExitRequest("fault", f"weight_step: {problem}")
        if self.exit_deadline is not None and self.io.clock() > self.exit_deadline:
            raise ExitRequest("fault", "exit_path_overrun")
        self.arm.send(weight, command, rs.SIM_GAINS_KP, rs.SIM_GAINS_KD)
        self.weight, self.command, self.sent_arm, self.phase = weight, command, True, phase
        self.lead = float(np.max(np.abs(command - measured)))
        self.log.add_message(self.io.clock() - self.start_t, phase, weight, command, rs.SIM_GAINS_KP, rs.SIM_GAINS_KD, measured)
        self.send_hands()
        self.wait()

    def measured_now(self) -> np.ndarray:
        """Measured angles of motors 12..28; stale LowState (> 0.1 s) is a fault exit, except while a fault holds still or
        ramps out (neither steers on the angles). A fault recover moves the arm, so there stale LowState stops it."""
        msg, age = self.io.lowstate.latest()
        if msg is None or age > rs.STALE_S["lowstate"]:
            if self.fault_stage in ("hold", "ramp_out"):
                return self.measured                 # keeps going on the last known angles
            raise ExitRequest("fault", "lowstate_stale")
        self.measured = real_io.sdk_q(msg)
        return self.measured

    def check_signals(self) -> None:
        """Rule 12: the run's first stop signal starts the slow way home - or, when a soft exit already walks home (normal
        end, stale input), changes nothing. Any later signal is a fault. In release (itself an exit path) any signal is a
        fault: a soft exit from there would walk back to the crash pose. In a fault hold or recover a signal ends that stage
        (real_run_exits.fault_exit: a hold ramps out, a recover holds); in the final ramp-out it is ignored."""
        count = self.io.signal_count()
        new, self.signals_seen = count - self.signals_seen, count
        if new <= 0:
            return
        if self.in_fault:
            if self.fault_stage in ("hold", "recover"):
                raise ExitRequest("fault", "signal_in_fault")
            return
        if self.in_release:
            raise ExitRequest("fault", "signal_in_release")
        if count - self.signals_at_start >= 2:
            raise ExitRequest("fault", "second_signal")
        if self.in_exit:
            self.say("stop requested: already on the slow way home. Press Ctrl+C again ONLY if an arm moves towards a person"
                     " (it freezes the arms).")
            return
        self.say("stop requested: slow exit. Press Ctrl+C again ONLY if an arm moves towards a person (it freezes the arms).")
        raise ExitRequest("soft", "operator_stop")

    def say(self, text: str) -> None:
        """Print an operator message. A closed terminal (SIGHUP) or a broken pipe makes print() raise OSError; that must
        never stop an exit path from handing the arms back (rule 12), so the text goes into the sidecar instead."""
        try:
            print(text, flush=True)
        except OSError:
            self.log.note_unprinted(text)

    def _react(self, events: list, phase: str) -> None:
        for event in events:
            self.note_event(event)
        if "contact_fault" in events:
            raise ExitRequest("fault", "contact_fault")
        if "envelope_streak" in events:
            raise ExitRequest("soft", "envelope_streak")

    def command_hands(self, left6, right6, speed: float) -> None:
        """Set the hand command sent with every following tick (the hand service keeps the last one, PLAN.md 2.2)."""
        self.hand_target, self.hand_speed = (np.asarray(left6, float), np.asarray(right6, float)), speed

    def hand_command(self) -> tuple:
        """(left6, right6) last hand command; open when the hands were never commanded."""
        return self.hand_target if self.hand_target is not None else (exits.OPEN_HANDS, exits.OPEN_HANDS)

    def command27(self) -> np.ndarray:
        """The command actually sent, in the 27-D policy layout (the log's `actions`)."""
        left, right = self.hand_command()
        return np.concatenate([self.command[3:17], left, right, self.command[:1]]).astype(np.float32)

    def note_event(self, name: str) -> None:
        """Count a safety or data event in the sidecar (first time and phase kept)."""
        self.log.note_event(name, self.io.clock() - self.start_t, self.phase)

    def send_hands(self) -> None:
        """Send the current hand command to both hands (nothing before the first command, nothing in a fault)."""
        if self.hand_target is None or self.in_fault:
            return                                   # a fault keeps the fingers as they are (rule 12)
        for side, targets in zip(action_space.SIDES, self.hand_target):
            if side in self.hand_senders:
                self.hand_senders[side].send(targets, self.hand_speed)

    def wait(self) -> None:
        """Sleep to the next 50 Hz tick; after an overrun, restart the schedule instead of sending a burst."""
        self.next_t += DT
        delay = self.next_t - self.io.clock()
        if delay > 0:
            self.io.sleep(delay)
        elif delay < -DT:
            self.next_t = self.io.clock()

    def log_arm_path(self, phase: str, plan: str | None = None, waypoints=None, leg: int | None = None) -> None:
        """Note where the arms are on our paths and write it to the sidecar at once (SAFE-PATHS.md S4): a recover in this
        run, or `86 recover` after a crash, picks the way home from it (arm_paths.home). Without a plan only the phase
        changes (hold, shadow, policy keep the way-in plan they came by)."""
        if plan is not None:
            self.arm_path = {"plan": plan, "waypoints": [np.asarray(w, dtype=float) for w in waypoints], "leg": leg}
        self.arm_path = {**self.arm_path, "phase": phase, "q0": self.q0, "ready17": self.ready17}
        self.log.note_arm_path(self.arm_path)

    def drive_path(self, plan: str, waypoints: list, targets: list, weight: float = 1.0) -> None:
        """Send (phase, leg, target) triples (real_safety.move_through, soft_exit_plan); at each new leg log_arm_path."""
        leg_now = None
        for phase, leg, q in targets:
            if leg != leg_now:
                self.log_arm_path(phase, plan, waypoints, leg)
                leg_now = leg
            self.tick(phase, weight, q)

    def start_exit_clock(self, ticks: int) -> None:
        """An exit path of `ticks` messages must finish within its planned time + EXIT_PATH_MARGIN_S (a stuck loop faults)."""
        self.exit_deadline = self.io.clock() + ticks * DT + rs.EXIT_PATH_MARGIN_S
