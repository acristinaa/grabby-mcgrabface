"""Fakes for the Track A unit tests: LowState / hand messages, readers, a clock, 86 sidecars and 84 reports.

Why it exists: the read-only bring-up is tested without DDS and without a robot. The messages are SimpleNamespaces with only
the fields real_io reads; sidecars and reports are written with the file names 86 and 84 use (PLAN.md 3.1, 3.4).
"""
import datetime
import json
import pathlib
from types import SimpleNamespace

MOTOR_COUNT = 35  # unitree_hg LowState_ motor_state array length


def fake_lowstate(q_of_motor) -> SimpleNamespace:
    """LowState whose motor i reads q_of_motor(i)."""
    return SimpleNamespace(motor_state=[SimpleNamespace(q=q_of_motor(i), temperature=[30, 31]) for i in range(MOTOR_COUNT)])


def fake_hand(values) -> SimpleNamespace:
    """Hand state (MotorStates_) with one motor per value."""
    return SimpleNamespace(states=[SimpleNamespace(q=v) for v in values])


class FakeReader:
    """Stands in for LowStateReader / HandReader / FrameStream: always the same message at the same age."""

    def __init__(self, msg, age=0.0):
        self.msg, self.age = msg, age

    def latest(self):
        return self.msg, self.age


class FakeClock:
    """Monotonic clock that only moves when sleep() is called, so a 30 fps recording runs instantly."""

    def __init__(self):
        self.t = 0.0

    def now(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        self.t += dt


def today_stamp(hhmmss: str) -> str:
    """<YYYYmmdd>_<hhmmss> for today, the stamp 84 and 86 put into their file names."""
    return f"{datetime.date.today().strftime('%Y%m%d')}_{hhmmss}"


def write_sidecar(log_dir: pathlib.Path, hhmmss: str = "120000", mode: str = "replay", **fields) -> pathlib.Path:
    """One 86 run sidecar from today (PLAN.md 3.4); arm_sdk_writer defaults to True like every run that sent."""
    path = pathlib.Path(log_dir) / f"{today_stamp(hhmmss)}_{mode}.json"
    path.write_text(json.dumps({"arm_sdk_writer": True, **fields}))
    return path


def write_bringup_report(report_dir: pathlib.Path, hhmmss: str, iface: str, q17) -> pathlib.Path:
    """One 84 report from today whose joints step stored q17 (what earliest_factory_arms_today reads)."""
    path = pathlib.Path(report_dir) / f"real_bringup_{today_stamp(hhmmss)}.json"
    report = {"iface": iface, "steps": {"joints": {"result": "PASS", "details": {"q17": [float(v) for v in q17]}}}}
    path.write_text(json.dumps(report))
    return path
