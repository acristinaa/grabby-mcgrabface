"""Real G1 read-only bring-up: the checks behind each step of scripts/84_real_bringup.py, with the readers passed in.

Why it exists: Monday's Part B (runbook) decides whether the robot, its hands and its camera deliver what the policy needs,
without moving anything. Each check returns PASS / FAIL / SKIP with a reason that tells the operator what to do. The checks
take readers and clocks as arguments, so tests drive them with fakes instead of a robot. No DDS writer is created here.
"""
import dataclasses
import time

import numpy as np

from g1sim import episode, policy_wire, real_camera, real_io

STEPS = ("net", "probe", "joints", "questions", "hand-state", "camera", "record")
PASS, FAIL, SKIP, NOT_RUN = "PASS", "FAIL", "SKIP", "NOT RUN"
ROBOT_LAPTOP_IP = "192.168.123.222"          # static IP of the robot laptop (scripts/72_real_check.py docstring)
PC2_IP = "192.168.123.164"                   # PC2, the robot's user computer (scripts/72_real_check.py:10)
LOWSTATE_STALE_S = 0.1                       # older LowState = FAIL (PLAN.md 4.1 rule 2)
HAND_STATE_STALE_S = 0.5                     # hand state older than this counts as missing (PLAN.md 4.2 rule 10)
HAND_STATE_MIN_HZ = 50.0                     # half of brainco_hand_service's 100 Hz (PLAN.md 2.2); slower = warning only
CAMERA_STALE_S = 1.0                         # camera frame older than this stops a recording (PLAN.md 4.2 rule 10)
OWNER_MOTION_LABEL = "owner_motion"          # the only label under which foreign rt/arm_sdk samples are expected (4.1 rule 2)
MIN_CAMERA_FPS = 10.0                        # below this a 15 fps D455 is overloaded or on USB 2 (warning only)
STATS_MEAN_TOL = 0.15                        # RGB mean further than this from the dataset = very different light (warning)
RECORD_FPS = 30                              # dataset rate (episode.py, fps 30)
PART_B_WARNING = ("WARNING: differs from the Part B reading by more than 0.10 rad - if the owner did not change the robot "
                  "mode, the owner's tool may still hold the arms: the owner re-sets the robot (or confirms their tool ended "
                  "at weight 0), then record again")   # PLAN.md 5 Track A action 2 (R3-9), word for word
ARMS_HELD_BY_US = ("our last 86 run may still hold the arms (it did not end with weight 0): run `86 release` first, then "
                   "re-record; if release is refused, ask the owner to use the remote; do not overwrite")   # PLAN.md 3.1 (R2-2)
POLICY_EVERY = 25                            # frames between two policy chunks = 86's exec_steps (PLAN.md 3.3)
QUESTIONS = (                                # PLAN.md section 9; Q8 and Q10 must be answered before Part C
    "Q1 Backpack PC: which computer is in the backpack, and may we use it?",
    "Q2 D455: which computer is it plugged into, USB 3 or 2, and is another program using it?",
    "Q3 Head pitch: is the head camera mount tilted, and by how many degrees?",
    "Q4 BrainCo hands: does brainco_hand_service run on PC2 (rt/brainco/*)?",
    "Q5 SSH/IP: user and IP of the robot PC with the camera and hands (default unitree@192.168.123.164)?",
    "Q6 Waist: are waist roll and pitch locked on this robot?",
    "Q7 Built-in arm service: does a built-in service already command the arms?",
    "Q8 arm_sdk: is rt/arm_sdk allowed on this firmware, and in which robot mode (remote combo)?",
    "Q9 Robot time: how long may the robot run today, and when are the breaks?",
    "Q10 Firmware: which firmware version runs on the robot?",
    "Q11 arm_sdk stops: what does the robot do when rt/arm_sdk messages stop?",
    "Q12 Owner motion: may we record while the owner moves the arms with their own tools?",
)


@dataclasses.dataclass
class StepResult:
    """One step's verdict for the 84 report: result, a reason the operator can act on, and the measured details."""
    result: str
    reason: str = ""
    details: dict = dataclasses.field(default_factory=dict)


class StepFailed(RuntimeError):
    """A step cannot continue; the message is the operator's next action."""


def sim_flags_problem(iface: str, sim: bool, yes_sim: bool, net_present: bool) -> str | None:
    """Why the --sim / --yes-sim combination is refused, or None. A sim run must never touch a robot network."""
    if yes_sim and not real_io.is_loopback(iface):
        return f"--yes-sim is only for the loopback interface (lo0 / lo), not {iface}: on the robot run without it"
    if yes_sim and net_present:
        return "a 192.168.123.x address is up: unplug the robot cable before a sim run"
    if sim and not yes_sim:
        return "--sim needs --yes-sim (confirms that no robot is connected)"
    if real_io.is_loopback(iface) and not sim:
        return f"{iface} is the loopback (no robot there): add --sim --yes-sim for a sim run, or pass the robot cable's interface"
    return None


def check_net(iface: str, addresses: dict, pc2_answers: bool) -> StepResult:
    """The robot laptop has its static IP on the cable's interface. A silent PC2 is only a warning: the LowState in the
    probe step is the proof that matters (PLAN.md 5 Track A action 2, R3-6)."""
    found = addresses.get(iface, [])
    details = {"addresses": found, "pc2_answers": pc2_answers}
    if ROBOT_LAPTOP_IP not in found:
        return StepResult(FAIL, f"{iface} has {found or 'no IPv4 address'}: set {ROBOT_LAPTOP_IP}/24 on the robot cable's "
                                "interface (runbook Part B step 0)", details)
    if not pc2_answers:
        return StepResult(PASS, f"{iface} = {ROBOT_LAPTOP_IP}; warning: PC2 {PC2_IP} does not answer ping (the probe "
                                "step decides)", details)
    return StepResult(PASS, f"{iface} = {ROBOT_LAPTOP_IP}, PC2 {PC2_IP} answers", details)


def check_probe(iface: str, hz: float, age_s: float, writers: dict, label: str) -> StepResult:
    """LowState fresh and fast enough; nobody else sends on rt/arm_sdk (rt/lowcmd samples only warn, review M4)."""
    details = {"lowstate_hz": hz, "lowstate_age_s": age_s, "writers": writers}
    if age_s > LOWSTATE_STALE_S:
        return StepResult(FAIL, f"no fresh rt/lowstate on {iface} (age {age_s:.2f} s): check cable, IP and the robot is on",
                          details)
    verdict = real_io.lowstate_hz_check(iface, hz)
    if verdict == "refuse":
        return StepResult(FAIL, f"rt/lowstate only {hz:.0f} Hz: a busy network or laptop; close other programs, re-run", details)
    arm_sdk = writers.get("rt/arm_sdk", {}).get("samples", 0)
    if arm_sdk and label != OWNER_MOTION_LABEL:
        return StepResult(FAIL, f"{arm_sdk} samples on rt/arm_sdk: someone else commands the arms - ask the owner to stop "
                                "it; never start 86 while this happens", details)
    warnings = []
    if verdict == "warn":
        warnings.append(f"rt/lowstate {hz:.0f} Hz is below {real_io.LOWSTATE_HZ['robot']['warn']} Hz")
    if writers.get("rt/lowcmd", {}).get("samples", 0):
        warnings.append("samples on rt/lowcmd (logged; tell Finnegan)")
    hand_cmd = [topic for topic in real_io.HAND_CMD_TOPICS if writers.get(topic, {}).get("samples", 0)]
    if hand_cmd:
        warnings.append(f"samples on {', '.join(hand_cmd)}: another program sends hand commands - ask the owner to stop "
                        "their hand tool before Part C")
    return StepResult(PASS, "; ".join(warnings) or f"rt/lowstate {hz:.0f} Hz, no foreign arm_sdk samples", details)


def check_joints(lowstate) -> StepResult:
    """Every SDK motor (12..28) inside its joint limits; always reports the FACTORY ARMS line for the table lookup."""
    rows = real_io.joint_rows(lowstate)
    outside = [r["name"] for r in rows if not r["inside"]]
    q17 = real_io.sdk_q(lowstate)
    summary = real_io.factory_arm_summary(q17)
    details = {"rows": rows, "factory_arms": summary, "q17": [float(v) for v in q17],
               "mirror_gap": [float(v) for v in real_io.mirror_gap(real_io.state27(lowstate, None, None))]}
    if outside:
        return StepResult(FAIL, f"outside the joint limits: {', '.join(outside)} - stop, show the owner", details)
    return StepResult(PASS, f"FACTORY ARMS: {summary}", details)


def part_b_lines(q17, earliest) -> list:
    """The PART B ARMS line (gap to today's first joints reading) and, above FACTORY_POSE_TOL, the warning that an owner
    tool may still hold the arms (R3-9). A warning, not a refusal: a mode change may legitimately move the hold pose (RV6-2)."""
    if earliest is None:
        return ["PART B ARMS: no joints reading earlier today to compare with (runbook Part B step 0 was skipped)"]
    q_then, hhmm = earliest
    gap = float(np.max(np.abs(np.asarray(q17, dtype=float) - q_then)))
    lines = [f"PART B ARMS: gap {gap:.2f} rad to the reading at {hhmm}"]
    if gap > real_io.FACTORY_POSE_TOL:
        lines.append(PART_B_WARNING)
    return lines


def record_factory_pose(path, q17, iface: str, mode_note: str, overwrite: bool, writers: dict, earliest,
                        log_dir=real_io.RUN_LOG_DIR) -> StepResult:
    """Save today's factory pose (runbook M0). Refused while someone else sends on rt/arm_sdk (their pose is not the factory
    pose, R3-9) and while our last 86 run may hold the arms (then the pose is OURS, R2-2); never silently over a different
    record. writers = active_writers() output, earliest = earliest_factory_arms_today()."""
    if writers.get("rt/arm_sdk", {}).get("samples", 0):
        return StepResult(FAIL, "another program steers the arms right now - its pose is not the factory pose; ask the "
                                "owner to stop it", {"writers": writers})
    if real_io.arms_may_be_held_by_us(log_dir):
        return StepResult(FAIL, ARMS_HELD_BY_US)
    details = {"part_b": part_b_lines(q17, earliest)}
    try:
        real_io.save_factory_pose(path, q17, iface, mode_note, overwrite=overwrite)
    except real_io.FactoryPoseConflict as conflict:
        return StepResult(FAIL, f"{conflict} if the owner changed the robot mode, re-run with --overwrite-factory-pose after "
                                "the owner confirms the arms are in the controller's hold pose (runbook M0); otherwise ask "
                                "the owner", details)
    except ValueError as problem:
        return StepResult(FAIL, str(problem), details)
    return StepResult(PASS, f"factory pose recorded in {path} (mode: {mode_note or 'sim'})", details)


def ask_questions(ask=input) -> StepResult:
    """Ask the owner Q1-Q12 and keep the answers; Q8 and Q10 must not stay empty before Part C."""
    answers = {}
    try:
        for question in QUESTIONS:
            answers[question.split()[0]] = ask(question + "\n> ").strip()
    except EOFError:
        return StepResult(NOT_RUN, "no keyboard input (run this step in a terminal)", {"answers": answers})
    missing = [key for key in ("Q8", "Q10") if not answers.get(key)]
    if missing:
        return StepResult(FAIL, f"{' and '.join(missing)} unanswered: Part C needs them", {"answers": answers})
    return StepResult(PASS, "answers saved", {"answers": answers})


def check_hands(states: dict) -> StepResult:
    """Both hands report fresh, NaN-free state (6 values each) from brainco_hand_service; a slow rate only warns.
    states = {side: (msg | None, age_s, rate_hz)}."""
    details, missing, broken, slow = {}, [], [], []
    for side, (msg, age, hz) in states.items():
        if msg is None or age > HAND_STATE_STALE_S:
            missing.append(side)
            continue
        values = [float(m.q) for m in msg.states[:real_io.HAND_MOTOR_COUNT]]
        details[side] = {"q": values, "hz": hz}
        if len(values) != real_io.HAND_MOTOR_COUNT or not np.isfinite(values).all():
            broken.append(side)
        if hz < HAND_STATE_MIN_HZ:
            slow.append(f"{side} {hz:.0f} Hz")
    if missing:
        return StepResult(FAIL, f"no hand state from {', '.join(missing)}: ask the owner to start brainco_hand_service on "
                                "PC2 (Q4); 72 hand --pruefen shows whether it runs", details)
    if broken:
        return StepResult(FAIL, f"hand state of {', '.join(broken)} is not 6 finite values: ask the owner to restart "
                                "brainco_hand_service on PC2 (Q4)", details)
    warning = f"; warning: slow hand state ({', '.join(slow)})" if slow else ""
    return StepResult(PASS, f"both hands report (0 = open, 1 = closed){warning}", details)


def check_camera(frames: list, stats: dict, reference) -> StepResult:
    """Frames arrive in the policy's framing (200 x 320, not cropped); light and fps are reported, the tilt is checked by eye
    on the overlay with the dataset reference frame."""
    if not frames:
        return StepResult(FAIL, f"no frames ({stats.get('error') or 'none arrived'}): check --camera-device with "
                                "72 kamera --suchen and that no other program uses the D455 (Q2)", {"stats": stats})
    shape = frames[-1].shape
    colours = real_camera.image_stats(frames)
    details = {"stats": stats, "shape": list(shape), "image_stats": colours}
    if stats.get("cropped") or shape != (real_camera.POLICY_H, real_camera.POLICY_W, 3):
        return StepResult(FAIL, f"camera delivers {stats.get('info')}: it must run at 1280 x 800 (16:10); a 640 x 480 "
                                "stream shows a cropped view the policy never saw", details)
    warnings = []
    if stats.get("fps", 0.0) < MIN_CAMERA_FPS:
        warnings.append(f"only {stats.get('fps', 0.0):.1f} fps (USB 2 or busy camera?)")
    gap = np.max(np.abs(np.subtract(colours["mean"], real_camera.DATASET_IMAGE_STATS["mean"])))
    if gap > STATS_MEAN_TOL:
        warnings.append(f"colours differ from the training data by {gap:.2f} (light?)")
    if reference is not None:
        details["overlay"] = real_camera.overlay(frames[-1], reference)
    return StepResult(PASS, "; ".join(warnings) or "framing OK; check the overlay by eye", details)


def record_observation(lowstate_reader, hand_readers: dict, frames, seconds: float, clock=time.monotonic,
                       sleep=time.sleep) -> dict:
    """Record what the robot does on its own (we command nothing) at 30 fps, as an episode dict in g1sim/episode.py layout.
    actions = the measured state (the robot's own motion is the only "action" there is); missing hands are 0 and flagged."""
    recorder = episode.EpisodeRecorder(policy_wire.TASK, RECORD_FPS)
    hands_present = []
    next_tick = clock()
    for _ in range(int(round(seconds * RECORD_FPS))):
        lowstate, low_age = lowstate_reader.latest()
        frame, frame_age = frames.latest()
        if lowstate is None or low_age > LOWSTATE_STALE_S:
            raise StepFailed(f"rt/lowstate stopped during the recording (age {low_age:.2f} s): check the cable, re-run")
        if frame is None or frame_age > CAMERA_STALE_S:
            raise StepFailed(f"camera stopped during the recording (age {frame_age:.2f} s): re-run the camera step")
        hands = {side: _fresh_hand(reader) for side, reader in hand_readers.items()}
        state = real_io.state27(lowstate, hands.get("left"), hands.get("right"))
        hands_present.append(bool(np.isfinite(state).all()))
        state = np.nan_to_num(state, nan=0.0)
        recorder.add({"head_image": frame, "state": state}, actions=state)
        next_tick += 1.0 / RECORD_FPS
        sleep(max(0.0, next_tick - clock()))
    ep = recorder.episode(success=False)
    ep.update(commanded_by_us=np.bool_(False), source=np.str_("real_observe"), hands_present=np.array(hands_present))
    return ep


def _fresh_hand(reader):
    msg, age = reader.latest()
    return msg if age <= HAND_STATE_STALE_S else None


def policy_predictions(client, ep: dict) -> dict:
    """Ask the policy server for a chunk every POLICY_EVERY frames of a finished recording (after it, so the recording keeps
    its 30 fps). Returns npz entries policy_ticks (K,) and policy_chunks (K, 50, 27)."""
    ticks = list(range(0, len(ep["timestamp"]), POLICY_EVERY))
    chunks = []
    for tick in ticks:
        obs = {policy_wire.IMAGE_KEY: ep["head_image"][tick], policy_wire.STATE_KEY: ep["state"][tick],
               "prompt": policy_wire.TASK}
        chunks.append(client.infer(obs, tick=tick)["actions"])
    return {"policy_ticks": np.array(ticks), "policy_chunks": np.stack(chunks).astype(np.float32)}
