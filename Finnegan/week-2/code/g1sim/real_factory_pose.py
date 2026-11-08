"""Real G1: the factory-pose record - where Unitree's controller holds the arms before we take over (PLAN.md 3.1, review N4).

Why it exists: `86 release` and every ramp-out hand the arms back to the factory controller. Without knowing its hold pose a
release could swing the arms (ruling R12), so 84 records that pose once per day, in the robot mode Part C uses (RV6-2), and
refuses to overwrite it silently. arms_may_be_held_by_us() reads 86's run sidecars to decide whether an overwrite is safe:
while our last run may still hold the arms, the measured pose is OUR pose, not the factory one (R2-2).
earliest_factory_arms_today() finds today's first 84 joints reading (Part B step 0, before anyone moved the arms), so a later
record can be compared with it (R3-9). Every file path is anchored to the repo root, never to the cwd (R3-5).
Pure stdlib + numpy, so every env can import it; g1sim/real_io.py re-exports it.
"""
import datetime
import json
import pathlib
import re

import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
FACTORY_POSE_TOL = 0.10        # rad; a larger gap = the arms are not where Unitree's controller keeps them (PLAN.md 3.1)
RUN_LOG_DIR = REPO / "runs" / "real"   # 86's default --out (PLAN.md 3.4); absolute (R3-5): 86 started elsewhere still writes here
BRINGUP_REPORT_DIR = REPO / "runs"     # 84's real_bringup_<stamp>.json reports (PLAN.md 3.1, R3-5, R3-9)
LOOPBACK = ("lo0", "lo")       # same names as real_io.LOOPBACK (real_io imports this module, so it cannot import real_io)
SIDECAR_NAME = re.compile(r"^(\d{8})_\d{6}_[a-z-]+\.json$")   # <YYYYmmdd_HHMMSS>_<mode>.json (PLAN.md 3.4)
REPORT_NAME = re.compile(r"^real_bringup_(\d{8})_(\d{2})(\d{2})\d{2}\.json$")   # 84's report name: date, hour, minute
# Positions in a 17-vector (index i = motor 12 + i) for the summary line: left arm = motors 15-21, right arm = 22-28
ARM_START = {"left": 3, "right": 10}
WAIST_YAW_INDEX = 0                                               # motor 12 is the first entry of a 17-vector
SHOULDER_PITCH, SHOULDER_ROLL, SHOULDER_YAW, ELBOW = 0, 1, 2, 3   # offsets inside one arm (action_space.ARM_JOINTS)
WRISTS = slice(4, 7)


class FactoryPoseConflict(RuntimeError):
    """A different factory pose was already recorded today; overwriting it needs an explicit --overwrite-factory-pose."""

    def __init__(self, message: str, record: dict):
        super().__init__(message)
        self.record = record


def factory_pose_path(iface: str) -> pathlib.Path:
    """runs/factory_pose_robot.json, or runs/factory_pose_sim.json on loopback, so a sim run never replaces the robot record."""
    name = "factory_pose_sim.json" if iface in LOOPBACK else "factory_pose_robot.json"
    return REPO / "runs" / name


def save_factory_pose(path, q17, iface: str, mode_note: str, overwrite: bool = False) -> None:
    """Write the record. Refuses an empty mode note on the robot and a silent change of today's record (FactoryPoseConflict)."""
    q17 = np.asarray(q17, dtype=float)
    if q17.shape != (17,) or not np.isfinite(q17).all():
        raise ValueError(f"factory pose needs 17 finite angles (motors 12..28), got shape {q17.shape}")
    if iface not in LOOPBACK and not mode_note.strip():
        raise ValueError("ask the owner which mode the robot is in (Q8) and pass --mode-note")
    old = load_factory_record(path)
    if old is not None and not overwrite and np.max(np.abs(np.asarray(old["q"]) - q17)) > FACTORY_POSE_TOL:
        raise FactoryPoseConflict(f"a different factory pose was recorded today at {old['recorded_at'][11:16]} "
                                  f"(mode: {old['mode_note']}).", old)
    now = datetime.datetime.now()
    record = {"q": [float(v) for v in q17], "iface": iface, "mode_note": mode_note,
              "recorded_at": now.isoformat(timespec="seconds"), "date": now.date().isoformat()}
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".part")
    tmp.write_text(json.dumps(record, indent=2))
    tmp.replace(path)  # rename: a reader never sees half a record


def load_factory_record(path) -> dict | None:
    """Today's whole record (q, iface, mode_note, recorded_at, date), or None when missing or from another day."""
    path = pathlib.Path(path)
    if not path.exists():
        return None
    record = json.loads(path.read_text())
    return record if record.get("date") == datetime.date.today().isoformat() else None


def load_factory_pose(path) -> np.ndarray | None:
    """Today's factory pose as a (17,) array, or None (missing or recorded on another day: the mode may have changed)."""
    record = load_factory_record(path)
    return None if record is None else np.asarray(record["q"], dtype=float)


def earliest_factory_arms_today(report_dir=BRINGUP_REPORT_DIR, robot: bool = True) -> tuple | None:
    """(q17, "HH:MM") of today's first 84 joints reading on a robot (robot=True) or loopback interface, or None.
    Why: Part B step 0 reads the joints before any owner motion, so a later factory-pose record that differs from it may
    show an owner tool still holding the arms (R3-9)."""
    today = datetime.date.today().strftime("%Y%m%d")
    for path in sorted(pathlib.Path(report_dir).glob("real_bringup_*.json")):   # the stamp in the name sorts by time
        match = REPORT_NAME.match(path.name)
        if not match or match.group(1) != today:
            continue
        report = json.loads(path.read_text())
        q17 = report.get("steps", {}).get("joints", {}).get("details", {}).get("q17")
        if q17 is not None and (report.get("iface") not in LOOPBACK) == robot:
            return np.asarray(q17, dtype=float), f"{match.group(2)}:{match.group(3)}"
    return None


def arms_may_be_held_by_us(log_dir=RUN_LOG_DIR) -> bool:
    """True when today's newest 86 run that created an arm_sdk writer did not hand the arms back (R2-2): it never finished
    (exit_code null: killed, crashed, power lost) or its last message had weight > 0. An unreadable sidecar counts as
    "may be held", because guessing "free" could let a held pose be recorded as the factory pose."""
    sidecar = _newest_arm_writer_sidecar(_resolve(log_dir))
    if sidecar is None:
        return False
    if sidecar.get("unreadable") or sidecar.get("exit_code") is None:
        return True
    return (sidecar.get("last_weight_sent") or 0.0) > 0.0


def last_arm_path(log_dir=RUN_LOG_DIR) -> dict | None:
    """The arm path (plan, leg, waypoints, phase, q0, ready17) that today's newest 86 run with an arm_sdk writer logged
    last, or None (no such run, an unreadable sidecar, or a run that logged none). `86 recover` drives home from it and
    refuses without it: guessing the way home could sweep the hands through the table (SAFE-PATHS.md S4)."""
    sidecar = _newest_arm_writer_sidecar(_resolve(log_dir))
    return None if sidecar is None else sidecar.get("arm_path")


def _resolve(log_dir) -> pathlib.Path:
    path = pathlib.Path(log_dir)
    return path if path.is_absolute() else REPO / path


def _newest_arm_writer_sidecar(log_dir: pathlib.Path) -> dict | None:
    today = datetime.date.today().strftime("%Y%m%d")
    todays = []
    for path in log_dir.glob("*.json"):
        match = SIDECAR_NAME.match(path.name)
        if match and match.group(1) == today:
            todays.append(path)
    for path in sorted(todays, key=lambda p: p.name, reverse=True):  # the stamp in the name sorts by time
        try:
            sidecar = json.loads(path.read_text())
        except (OSError, json.JSONDecodeError):
            return {"unreadable": True}
        if sidecar.get("arm_sdk_writer"):
            return sidecar
    return None


def factory_arm_summary(q17) -> str:
    """One line with everything the runbook's table-clearance lookup needs (rule 16 (a)): elbow, shoulder pitch, roll and yaw
    of both sides, the largest wrist angle and the waist yaw (the lookup has one column per waist limit, R3-2)."""
    q17 = np.asarray(q17, dtype=float)
    left, right = (q17[ARM_START[side]:ARM_START[side] + 7] for side in ("left", "right"))
    pairs = [("elbow", ELBOW), ("shoulder pitch", SHOULDER_PITCH), ("shoulder roll", SHOULDER_ROLL),
             ("shoulder yaw", SHOULDER_YAW)]
    parts = [f"{name} {left[i]:.2f}/{right[i]:.2f}" for name, i in pairs]
    largest_wrist = max(np.max(np.abs(left[WRISTS])), np.max(np.abs(right[WRISTS])))
    return (f"left/right {', '.join(parts)}, largest wrist {largest_wrist:.2f}, "
            f"waist yaw {q17[WAIST_YAW_INDEX]:.2f} rad")
