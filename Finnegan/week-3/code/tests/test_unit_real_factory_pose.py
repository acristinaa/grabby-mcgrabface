"""Unit: the factory-pose record (runbook M0) - overwrite rules, 86 sidecars, the Part B comparison, the FACTORY ARMS line.

Why these tests matter: `86 release` hands the arms back to the pose recorded here. A record taken while OUR run or an
owner tool still holds the arms would make every later release "release" to the wrong pose (PLAN.md 3.1, R2-2, R3-9).
Fakes only: sidecars and reports are small JSON files in a temp folder (tests/helpers/real_fakes.py).
"""
import pathlib

import numpy as np
import pytest

pytest.importorskip("unitree_sdk2py")
pytest.importorskip("cv2")
from g1sim import real_bringup as rb, real_io  # noqa: E402
from helpers.real_fakes import write_bringup_report, write_sidecar  # noqa: E402

pytestmark = pytest.mark.unit

NO_WRITERS = {"rt/arm_sdk": {"samples": 0, "max_weight": None}}


def record(path, q17, log_dir, overwrite=False, writers=NO_WRITERS, earliest=None, mode_note="R2+A"):
    """84's --record-factory-pose with the DDS listening and the report lookup already done (passed in as fakes)."""
    return rb.record_factory_pose(path, q17, "en5", mode_note, overwrite, writers, earliest, log_dir=log_dir)


@pytest.fixture
def log_dir(tmp_path) -> pathlib.Path:
    folder = tmp_path / "real"
    folder.mkdir()
    return folder


def test_factory_pose_record_refuses_silent_overwrite(tmp_path):
    """Proves: a second record today that differs by more than 0.10 rad raises FactoryPoseConflict and keeps the first.
    Catches: a re-run of M0 with the arms elsewhere silently replacing the pose that 86 release relies on."""
    path, q = tmp_path / "factory_pose_robot.json", np.zeros(17)
    real_io.save_factory_pose(path, q, "en5", "R2+A")

    with pytest.raises(real_io.FactoryPoseConflict):
        real_io.save_factory_pose(path, q + 0.2, "en5", "R2+A")
    np.testing.assert_allclose(real_io.load_factory_pose(path), q)


@pytest.mark.parametrize("sidecar", [{"exit_code": None, "last_weight_sent": None},
                                     {"exit_code": 0, "last_weight_sent": 0.4}], ids=["unfinished", "ended_at_weight_0.4"])
def test_factory_pose_overwrite_refused_while_our_last_run_may_hold_the_arms(tmp_path, log_dir, sidecar):
    """Proves: --overwrite-factory-pose is refused (pointing to `86 release`) while today's last 86 run may still hold the arms.
    Catches: recording OUR held pose as the factory pose; a later release would then never hand the arms back."""
    path = tmp_path / "factory_pose_robot.json"
    write_sidecar(log_dir, **sidecar)
    real_io.save_factory_pose(path, np.zeros(17), "en5", "R2+A")

    result = record(path, np.full(17, 0.3), log_dir, overwrite=True)

    assert result.result == rb.FAIL and "86 release" in result.reason
    np.testing.assert_allclose(real_io.load_factory_pose(path), np.zeros(17))


def test_factory_pose_first_record_refused_while_our_last_run_may_hold_the_arms(tmp_path, log_dir):
    """Proves: even the first record of the day is refused while our last 86 run is unfinished.
    Catches: the R2-2 gap for a first record - the measured pose would be OURS, not the factory controller's."""
    path = tmp_path / "factory_pose_robot.json"
    write_sidecar(log_dir, exit_code=None, last_weight_sent=None)

    result = record(path, np.zeros(17), log_dir)

    assert result.result == rb.FAIL and "86 release" in result.reason and not path.exists()


def test_factory_pose_overwrite_allowed_after_a_run_that_ended_at_weight_0(tmp_path, log_dir):
    """Proves: after a run that ended with weight 0 sent, --overwrite-factory-pose replaces today's record.
    Catches: the rev 6.1 dead end - after a mode change the factory pose could never be re-recorded that day."""
    path = tmp_path / "factory_pose_robot.json"
    write_sidecar(log_dir, exit_code=3, last_weight_sent=0.0)
    real_io.save_factory_pose(path, np.zeros(17), "en5", "R2+A")

    result = record(path, np.full(17, 0.3), log_dir, overwrite=True, mode_note="R1+X")

    assert result.result == rb.PASS
    np.testing.assert_allclose(real_io.load_factory_pose(path), np.full(17, 0.3))


def test_newest_arm_writer_run_decides_and_refused_runs_are_skipped(log_dir):
    """Proves: with a clean run (weight 0) at 10:00, a killed run at 11:00 and a refused run (no arm_sdk writer) at 12:00,
    the arms count as held - the newest run that sent decides, refused runs do not count.
    Catches: the oldest sidecar deciding, or a refused run counted as "ended cleanly"; either lets OUR held pose be saved
    as the factory pose after a killed run (review A-1 major 2)."""
    write_sidecar(log_dir, "100000", "ready", exit_code=0, last_weight_sent=0.0)
    write_sidecar(log_dir, "110000", "replay", exit_code=None, last_weight_sent=None)
    write_sidecar(log_dir, "120000", "replay", arm_sdk_writer=False, exit_code=2, last_weight_sent=None)

    held = real_io.arms_may_be_held_by_us(log_dir)

    assert held is True


def test_sidecar_found_when_86_ran_from_another_cwd(tmp_path, monkeypatch):
    """Proves: a sidecar that 86 writes to its default RUN_LOG_DIR while started from another folder is the one 84 reads.
    Catches: a relative RUN_LOG_DIR - 86 would write under its own cwd, 84 would find no sidecar and allow an overwrite
    while our run holds the arms (fails open, R3-5)."""
    monkeypatch.chdir(tmp_path)                                   # 86 started from some other folder
    sidecar_dir = pathlib.Path(real_io.RUN_LOG_DIR)               # what 86 uses as its default --out
    sidecar_dir.mkdir(parents=True, exist_ok=True)
    sidecar = write_sidecar(sidecar_dir, "235959", "test-cwd", exit_code=None, last_weight_sent=None)   # newest of today
    monkeypatch.chdir(real_io.REPO)                               # 84 runs from the repo root
    try:
        held = real_io.arms_may_be_held_by_us()
    finally:
        sidecar.unlink()

    assert held is True


def test_factory_pose_record_refused_while_a_foreign_arm_sdk_writer_sends(tmp_path, log_dir):
    """Proves: --record-factory-pose is refused while samples arrive on rt/arm_sdk, and nothing is saved.
    Catches: recording an owner tool's pose as the factory pose (R3-9); 86 release would later hand the arms to it."""
    path = tmp_path / "factory_pose_robot.json"
    writers = {"rt/arm_sdk": {"samples": 40, "max_weight": 1.0}}

    result = record(path, np.zeros(17), log_dir, writers=writers)

    assert result.result == rb.FAIL and "another program steers the arms" in result.reason and not path.exists()


def test_factory_pose_record_warns_when_it_differs_from_the_part_b_reading(tmp_path, log_dir):
    """Proves: a record 0.3 rad away from today's first joints reading (09:30) prints the PART B gap and the warning.
    Catches: an owner tool that still holds the arms at M0 going unnoticed, because the record itself only checks today's
    earlier record, not the untouched Part B reading (R3-9)."""
    write_bringup_report(tmp_path, "093000", "en5", np.zeros(17))
    write_bringup_report(tmp_path, "101500", "en5", np.full(17, 0.3))   # a later reading must not be the reference
    earliest = real_io.earliest_factory_arms_today(tmp_path, robot=True)

    result = record(tmp_path / "factory_pose_robot.json", np.full(17, 0.3), log_dir, earliest=earliest)

    assert result.details["part_b"] == ["PART B ARMS: gap 0.30 rad to the reading at 09:30", rb.PART_B_WARNING]


def test_factory_pose_record_stores_mode_note(tmp_path):
    """Proves: the record keeps the owner's Q8 mode note.
    Catches: a record without its robot mode - the hold pose can change with the mode (issues #156/#170)."""
    path = tmp_path / "factory_pose_robot.json"

    real_io.save_factory_pose(path, np.zeros(17), "en5", "R2+A after L2+up")

    assert real_io.load_factory_record(path)["mode_note"] == "R2+A after L2+up"


def test_factory_pose_record_refuses_an_empty_mode_note_on_the_robot(tmp_path):
    """Proves: an empty (or blank) mode note is refused off loopback with a message naming Q8.
    Catches: an operator skipping the owner question, so nobody knows later which mode the pose belongs to."""
    with pytest.raises(ValueError, match="Q8"):
        real_io.save_factory_pose(tmp_path / "factory_pose_robot.json", np.zeros(17), "en5", "  ")


def test_factory_pose_paths_differ_for_robot_and_sim():
    """Proves: loopback runs write factory_pose_sim.json, robot runs factory_pose_robot.json, both under the repo root.
    Catches: a sim run on Sunday night replacing Monday's robot record (or the other way round)."""
    assert real_io.factory_pose_path("lo0") == real_io.REPO / "runs" / "factory_pose_sim.json"
    assert real_io.factory_pose_path("en5") == real_io.REPO / "runs" / "factory_pose_robot.json"


def test_factory_arm_summary_lists_roll_yaw_wrist_and_waist():
    """Proves: the FACTORY ARMS line shows elbow, shoulder pitch, roll, yaw of both sides, the largest wrist and the waist yaw.
    Catches: a summary that hides a factory shoulder roll or waist yaw, which the table-clearance lookup must see (R2-3, R3-2)."""
    q17 = np.zeros(17)
    q17[0] = 0.01                      # waist yaw (motor 12)
    q17[[6, 13]] = 0.95, 0.97          # elbows (motors 18, 25)
    q17[[4, 11]] = 0.25, -0.24         # shoulder roll (motors 16, 23)
    q17[[5, 12]] = 0.01, -0.02         # shoulder yaw (motors 17, 24)
    q17[15] = -0.05                    # right wrist pitch (motor 27)

    summary = real_io.factory_arm_summary(q17)

    assert summary == ("left/right elbow 0.95/0.97, shoulder pitch 0.00/0.00, shoulder roll 0.25/-0.24, "
                       "shoulder yaw 0.01/-0.02, largest wrist 0.05, waist yaw 0.01 rad")


def test_earliest_reading_ignores_sim_reports_on_the_robot(tmp_path):
    """Proves: on the robot the Part B reference is the first ROBOT joints reading of today, never a loopback one.
    Catches: Sunday-night or Monday-morning sim runs becoming the reference, which would make every robot record warn."""
    write_bringup_report(tmp_path, "070000", "lo0", np.full(17, 0.5))
    write_bringup_report(tmp_path, "093000", "en5", np.zeros(17))

    q17, hhmm = real_io.earliest_factory_arms_today(tmp_path, robot=True)

    assert hhmm == "09:30" and not q17.any()
