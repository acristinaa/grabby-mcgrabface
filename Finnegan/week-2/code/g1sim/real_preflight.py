"""Real G1: the preflight checks of 86 - everything that must be true BEFORE 86 creates its first DDS writer (PLAN.md 4.2 rule 2).

Why it exists: a refusal before the first writer is free (nothing was ever sent, exit 2), while a problem found after the
ramp-in costs a slow exit with the arms in motion. Each check returns the operator-facing problem text or None; the runner
turns the first problem into "86 <MODE>: REFUSED <problem>". Reads only: the injected readers, the factory record, the
policy server's hello. numpy + repo modules only (the readers come in through g1sim/real_run.RunIO).
"""
import pathlib

import numpy as np

from g1sim import arm_paths, policy_wire, real_io, real_safety

ARM_MODES = ("ready", "replay", "shadow", "run")        # modes that move the arms away from the factory pose
POLICY_MODES = ("replay", "shadow", "run")
LIVE_POLICY_MODES = ("shadow", "run")                   # need a fresh camera frame and hand state (rule 2)
CAMERA_READY_S = 0.5                                    # newest camera frame must be younger than this to start (rule 2)
HELLO = {"chunk_size": policy_wire.CHUNK, "action_dim": policy_wire.ACTION_DIM,
         "image_shape": list(policy_wire.IMAGE_SHAPE)}  # what 86's scheduler and observation assume (PLAN.md 3.4)
LIMITS_LO = np.array([real_io.JOINT_LIMITS[m][0] for m in real_safety.SDK_MOTORS])
LIMITS_HI = np.array([real_io.JOINT_LIMITS[m][1] for m in real_safety.SDK_MOTORS])
RECOVER_FROM_POLICY_LOG = False     # ruling pending (Finnegan), see SAFE-PATHS.md: the policy phase logs no arm pose, so
                                    # `86 recover` refuses its logs (review safe-paths-code B1-r (b)); True = accept them unchecked


def run_preflight(mode: str, io, args, note_event, say) -> str | None:
    """All checks of rule 2 in order, for one 86 run (io = real_run.RunIO, args = real_run.RunArgs). Returns the first
    problem, or None when 86 may create its writers. hands-open moves no arm, so it only needs the argument check.
    `say` prints an operator message (real_run.Runner.say: it survives a closed terminal)."""
    problem = argument_problem(mode, args)
    if problem or mode == "hands-open":
        return problem
    samples = io.foreign_samples()              # listens 2 s; the readers fill meanwhile
    warn_foreign_commands(samples, note_event, say)
    msg, age = io.lowstate.latest()
    rate = io.lowstate.rate_hz(1.0)
    problem = lowstate_problem(args.iface, age, rate) or foreign_writer_problem(samples)
    if problem:
        return problem
    if real_io.lowstate_hz_check(args.iface, rate) == "warn":
        say(f"warning: rt/lowstate at {rate:.0f} Hz (expected about 500 on the robot)")
    measured = real_io.sdk_q(msg)
    out_dir = resolve_out_dir(args.out)
    problem = (joint_limit_problem(measured) or factory_problem(mode, io.factory_record, measured, out_dir)
               or arm_path_problem(mode, io.last_arm_path, measured, io.factory_record,
                                   real_io.arms_may_be_held_by_us(out_dir)))
    if problem or mode not in POLICY_MODES:
        return problem
    problem = hello_problem(mode, io.policy.metadata)
    if problem or mode not in LIVE_POLICY_MODES:
        return problem
    camera_age = io.frames.latest()[1] if io.frames is not None else None
    hand_ages = None if args.no_hands else {side: reader.latest()[1] for side, reader in io.hands.items()}
    return live_input_problem(camera_age, hand_ages)


def warn_foreign_commands(samples: dict, note_event, say) -> None:
    """Commands from another program that do not refuse the run, but must be seen: rt/lowcmd (review M4) and the hand
    command topics (R3-10 (c): an owner's hand tool and our 50 Hz hand commands would fight over the fingers)."""
    if samples.get("rt/lowcmd", {}).get("samples", 0) > 0:
        say("warning: samples on rt/lowcmd (logged; only rt/arm_sdk samples refuse, review M4)")
        note_event("lowcmd_samples")
    hand_topics = [topic for topic in real_io.HAND_CMD_TOPICS if samples.get(topic, {}).get("samples", 0) > 0]
    if hand_topics:
        say(f"warning: samples on {', '.join(hand_topics)}: another program sends hand commands - ask the owner to stop "
            f"their hand tool, or run with --no-hands")
        note_event("foreign_hand_cmd")


def argument_problem(mode: str, args) -> str | None:
    """The option checks that need no robot (args = real_run.RunArgs): motion gate (e) --speed <= 0.5 and --waist-max
    <= 0.6 rad off loopback (rules 6, 6b), the log folder (R3-5), and hands-open needs the hands."""
    on_loopback = real_io.is_loopback(args.iface)
    if mode in ("replay", "run"):
        problem = real_safety.speed_refusal(args.speed, on_loopback)
        if problem:
            return problem
    problem = real_safety.waist_max_refusal(args.waist_max, on_loopback) or out_dir_problem(args.out, on_loopback)
    if problem:
        return problem
    if mode == "hands-open" and args.no_hands:
        return "hands-open needs the hands; drop --no-hands"
    return None


def resolve_out_dir(out_dir) -> pathlib.Path:
    """--out as an absolute path: a relative folder is taken from the repo root, never from the current folder (R3-5)."""
    path = pathlib.Path(out_dir)
    return (path if path.is_absolute() else real_io.REPO / path).resolve()


def out_dir_problem(out_dir, on_loopback: bool) -> str | None:
    """Off loopback the log folder must be exactly RUN_LOG_DIR (R3-5): 84 (`arms_may_be_held_by_us`) and the next 86 run
    read only the sidecars directly in that folder, so a killed robot run logged anywhere else would look like "arms free"."""
    if on_loopback or resolve_out_dir(out_dir) == pathlib.Path(real_io.RUN_LOG_DIR).resolve():
        return None
    return (f"--out {out_dir}: robot runs log only to runs/real, so 84 can see whether our last run may still hold the "
            f"arms - drop --out")


def lowstate_problem(iface: str, age_s: float, rate_hz: float) -> str | None:
    """LowState must be fresh (0.1 s) and fast enough (100 Hz loopback, 200 Hz robot): stale angles steer blind."""
    if age_s > real_safety.STALE_S["lowstate"]:
        return (f"no fresh rt/lowstate (newest is {age_s:.2f} s old) - check the cable and the robot network "
                f"(runbook Part B step 0)")
    if real_io.lowstate_hz_check(iface, rate_hz) == "refuse":
        return f"rt/lowstate arrives at {rate_hz:.0f} Hz, too slow to steer on - check the cable; do not continue"
    return None


def foreign_writer_problem(samples: dict) -> str | None:
    """Samples on rt/arm_sdk mean someone else steers the arms right now; two writers would fight (review M4)."""
    arm = samples.get("rt/arm_sdk", {})
    if arm.get("samples", 0) > 0:
        return (f"someone else is sending on rt/arm_sdk ({arm['samples']} messages in 2 s, weight up to "
                f"{arm.get('max_weight')}) - stop every other program and ask the owner; do not continue")
    return None


def joint_limit_problem(measured17) -> str | None:
    """Every measured motor 12..28 inside JOINT_LIMITS; outside means a wrong reading or a broken joint."""
    for i, motor in enumerate(real_safety.SDK_MOTORS):
        if not LIMITS_LO[i] <= measured17[i] <= LIMITS_HI[i]:
            return (f"motor {motor} ({real_io.MOTOR_NAMES[motor]}) reads {measured17[i]:.3f} rad, outside its limits "
                    f"{real_io.JOINT_LIMITS[motor]} - do not move; tell the owner")
    return None


def factory_problem(mode: str, record: dict | None, measured17, out_dir) -> str | None:
    """Arm modes start only from the factory pose recorded today; release only needs the record to exist (ruling R12)."""
    if record is None:
        if mode == "release":
            return ("no factory pose recorded today, so 86 cannot know where to put the arms - tell the owner to use the "
                    "remote (runbook box); we do nothing more")
        return "record the factory pose first (runbook M0)"
    if mode not in ARM_MODES:
        return None
    gap = real_safety.factory_pose_gap(measured17, record["q"])
    if gap <= real_io.FACTORY_POSE_TOL:
        return None
    recorded_hhmm = record["recorded_at"][11:16]        # ISO "YYYY-MM-DDTHH:MM:SS" -> "HH:MM"
    start = f"arms are {gap:.2f} rad from the factory pose recorded at {recorded_hhmm} (mode: {record['mode_note']})."
    if real_io.arms_may_be_held_by_us(out_dir):
        return f"{start} our last run may still hold the arms: run `86 release` first"
    return (f"{start} our code does not hold the arms. If the owner changed the robot mode, re-record the factory pose "
            f"(runbook M0); otherwise ask the owner")


def arm_path_problem(mode: str, arm_path: dict | None, measured17, record: dict | None,
                     held_by_us: bool) -> str | None:
    """`86 recover` drives home from the arm path our last run logged (SAFE-PATHS.md S4); without one 86 cannot know which
    way home misses the table, and a guess (e.g. the way out from a pose low on the way in) can go through it.
    The log is only valid while the arms are still where it says (review safe-paths-code B1, B1-r): after a hand-back
    (freeze + Ctrl+C), a kill Unitree did not latch, or the owner's remote, the arms are elsewhere, and the way home from
    the log then starts with an unchecked line (3.6 cm from the basket from the factory pose, down to -15 cm into the
    table). So recover refuses when that run already handed the arms back (held_by_us False:
    real_io.arms_may_be_held_by_us), when the arms are where it took them over, when they are off the ready pose a
    hold/shadow log names or off the leg a waypoint log names, and for a policy log, which names no pose
    (RECOVER_FROM_POLICY_LOG)."""
    if mode != "recover":
        return None
    if arm_path is None:
        return ("our last run logged no arm path, so 86 cannot know which way home misses the table - tell the owner to "
                "use the remote (runbook box), or pull the table back and run `86 release`")
    if not held_by_us:
        return ("our last run already handed the arms back (weight 0), so Unitree's controller holds them and no log says "
                "where they are - nothing to recover; if they are near the table, tell the owner to use the remote "
                "(runbook box)")
    homes = [arm_path["q0"]] + ([] if record is None else [record["q"]])
    if min(real_safety.factory_pose_gap(measured17, home) for home in homes) <= real_io.FACTORY_POSE_TOL:
        return ("the arms are where our last run took them over (its q0: the factory pose, or the crash pose if that run "
                "was `86 release`), so there is no way home to recover along - run `86 release` (pull the table back "
                "first if the hands are near it)")
    phase = arm_path["phase"]
    if phase == "policy":
        if RECOVER_FROM_POLICY_LOG:
            return None
        return ("our last run stopped in the policy phase, which logs no arm pose, so 86 cannot check a way home from "
                "where the arms are - tell the owner to use the remote (runbook box), or pull the table back and run "
                "`86 release`")
    if phase in arm_paths.FREE_PHASES:                  # hold/shadow: 86 sent the logged ready pose the whole phase
        off = real_safety.factory_pose_gap(measured17, arm_path["ready17"])
        where = f"the ready pose of its {phase} phase"
    else:
        off = leg_distance(measured17, arm_path["waypoints"], arm_path["leg"])
        where = f"{arm_path['plan']} leg {arm_path['leg']}"
    if off > real_safety.LEAD_LIMIT:
        return (f"the arms are {off:.2f} rad from where our last run logged them ({where}), so its way home is not "
                f"checked from here - tell the owner to use the remote (runbook box); we do nothing more")
    return None


def leg_distance(measured17, waypoints, leg: int) -> float:
    """Largest joint gap (rad) from measured17 to the nearest message 86 sends on leg `leg` (1-based, waypoints[leg - 1]
    -> waypoints[leg], the real_safety.move line). Within LEAD_LIMIT = the arm still sits on that leg, the same bound the
    lead monitor allows between command and measured angle (rule 7)."""
    start = np.asarray(waypoints[leg - 1], dtype=float)
    line = np.vstack([start] + real_safety.move(start, np.asarray(waypoints[leg], dtype=float)))
    return float(np.min(np.max(np.abs(line - np.asarray(measured17, dtype=float)), axis=1)))


def hello_problem(mode: str, metadata: dict) -> str | None:
    """The policy server must speak our format, and replay must talk to a replay server, shadow/run to a checkpoint
    (catches a swapped server: a replay played as if it were the VLA, or the reverse)."""
    for key, wanted in HELLO.items():
        if metadata.get(key) != wanted:
            return f"policy server sends {key} = {metadata.get(key)}, expected {wanted} - wrong server or version"
    server_mode = metadata.get("mode")
    if mode == "replay" and server_mode != "replay":
        return f"86 replay needs a replay server (81 --replay ...), this one is {server_mode!r}"
    if mode in LIVE_POLICY_MODES and (server_mode == "replay" or not metadata.get("checkpoint")):
        return f"86 {mode} needs a checkpoint server (81 --checkpoint ...), this one is {server_mode!r}"
    return None


def live_input_problem(camera_age: float | None, hand_ages: dict | None) -> str | None:
    """shadow/run start only with a fresh camera frame and hand state from both hands (unless --no-hands: hand_ages None)."""
    if camera_age is None or camera_age >= CAMERA_READY_S:
        return f"no camera frame younger than {CAMERA_READY_S} s - check the camera command (runbook Part B step 3)"
    if hand_ages is not None:
        missing = [side for side, age in hand_ages.items() if age > real_safety.STALE_S["hand_state"]]
        if missing:
            return (f"no hand state from {', '.join(missing)} - is brainco_hand_service running? (runbook Part B step 2) "
                    f"or pass --no-hands")
    return None
