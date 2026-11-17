"""Track C: 87's table clearance - the FK scene and distance code, the pass rule, the fault orderings, sampling by arm movement,
start poses by arm movement, and the merge of partial runs (PLAN.md 3.5, rule 16). FK only (no physics, no DDS), seconds.
The sampling tests use a planted one-joint "arm" in pure numpy: a point 0.5 m from its axis, like shoulder to fingertip.
    ~/dev/g1-ext/.venv/bin/python -m pytest -m integration tests/test_integration_arm_path_clearance.py -v
"""
import importlib.util
import json
import pathlib

import numpy as np
import pytest

from g1sim import arm_clearance as ac
from g1sim import arm_clearance_report as report_lib

pytestmark = pytest.mark.integration

ARM_RADIUS_M = 0.5          # planted arm: shoulder to fingertip
EDGE_OUTSIDE_M = 0.01       # planted edge point 1 cm outside the fingertip arc
SWEEP_RAD = 2.4             # planted sweep, about a fault drop from a raised arm
FAR_M = 1.0                 # planted clearance of objects the test does not look at
CLOSE_PASS_MAX_M = 0.015    # a densified check must see the 1 cm pass within this (PLAN.md 3.5: <= 1.5 cm)
INSIDE_TABLE_ELBOW = 0.3    # h(0.3, 0): fingertips 3 cm inside the table top at the nominal stop (FK probe 2026-10-18)
NEAR_TABLE_ELBOW = 0.5      # h(0.5, 0): hand in the table; a setback must reduce the overlap
SCRIPT_87 = pathlib.Path(__file__).resolve().parent.parent / "scripts" / "87_arm_path_clearance.py"


def tip(q: np.ndarray) -> np.ndarray:
    """Fingertip of the planted one-joint arm; its angle is position 0 of the 17-vector."""
    return ARM_RADIUS_M * np.array([np.cos(q[0]), np.sin(q[0])])


def tip_step(qa: np.ndarray, qb: np.ndarray) -> float:
    return float(np.linalg.norm(tip(qb) - tip(qa)))


def edge_at(angle: float) -> np.ndarray:
    return (ARM_RADIUS_M + EDGE_OUTSIDE_M) * np.array([np.cos(angle), np.sin(angle)])


def joint_pose(angle: float) -> np.ndarray:
    pose = np.zeros(17)
    pose[0] = angle
    return pose


@pytest.fixture(scope="module")
def scene():
    """The 87 scene (TableEnv volleyball_walk, nominal layout); built once, setback reset after each use."""
    fk = pytest.importorskip("g1sim.arm_clearance_fk", reason="needs the sim env ~/dev/g1-ext/.venv (TableEnv, MuJoCo 3.13)")
    return fk.ClearanceModel()


def test_ready_pose_clears_table_stand_and_basket_at_nominal_stop(scene):
    """Proves: the scene, stance and distance code agree with the measured clearance of BALL_READY_POSE (>= 4.9 cm).
    Catches: a wrong scene, stance (pelvis not at the walk stop) or geom set behind every number of the lookup."""
    clearance = scene.clearance_of(ac.READY17)

    assert min(clearance.values()) >= ac.CLEARANCE_MIN_M, clearance


def test_clearance_detects_a_path_through_the_table(scene):
    """Proves: a planted path from the ready pose to a hanging pose whose fingers end 3 cm inside the table top reads
    negative table clearance.
    Catches: a distance call that never sees the table (then every table step would look clear)."""
    path = ac.densify(ac.move_path(ac.READY17, ac.hanging_pose(INSIDE_TABLE_ELBOW, 0.0)), scene.step_of)

    along, _ = ac.path_clearance(path, scene.clearance_of)

    assert along["table"] < 0, along


def test_setback_increases_clearance(scene):
    """Proves: moving table, stand and basket back (+x, away from the robot) increases the hand's table clearance.
    Catches: a setback applied with the wrong sign, which would make the M0 lookup move the table the wrong way."""
    pose = ac.hanging_pose(NEAR_TABLE_ELBOW, 0.0)
    nominal = scene.clearance_of(pose)["table"]

    scene.set_setback(0.05)
    moved_back = scene.clearance_of(pose)["table"]
    scene.set_setback(0.0)

    assert moved_back > nominal + 0.01, (nominal, moved_back)


def test_clearance_is_computed_with_the_torso_sagged_forward(scene):
    """Proves: whatever waist roll/pitch the 17-vector carries (86 sends q0), the FK leans the torso to WAIST_PITCH_CHECKED.
    Catches: a return to the upright torso of sdk_targets(q0=zeros), which reads clearances 2-6 cm too optimistic (review B1)."""
    upright = ac.hanging_pose(NEAR_TABLE_ELBOW, 0.0)
    leaning = upright.copy()
    leaning[2] = ac.WAIST_PITCH_CHECKED          # 17-vector position of motor 14, waist pitch

    assert scene.clearance_of(upright) == scene.clearance_of(leaning)
    assert scene.data.qpos[scene.waist_roll_pitch_qadr] == pytest.approx([0.0, ac.WAIST_PITCH_CHECKED])


def test_policy_exit_pass_rule_compares_with_start_clearance():
    """Proves: an exit that starts 1 cm from the stand and moves away passes; one that starts clear and dips to 1 cm from the
    table edge fails.
    Catches: a plain 5 cm rule that fails every grasp frame, and an "only the start matters" rule that misses the dip."""
    grasp_start = {"table": 0.20, "stand": 0.01, "basket": 0.30}
    moving_away = {"table": 0.20, "stand": 0.012, "basket": 0.30}
    clear_start = {"table": 0.12, "stand": 0.20, "basket": 0.30}
    dipping = {"table": 0.01, "stand": 0.20, "basket": 0.30}

    assert ac.exit_passes(grasp_start, moving_away)
    assert not ac.exit_passes(clear_start, dipping)


def test_fault_exit_counts_the_worst_of_straight_and_corner_orderings():
    """Proves: a start/goal pair whose straight line keeps clear but whose distal_first corner passes 1 cm from an edge fails.
    Catches: an 87 that assumes a q-only blend (straight line only) while a torque-level blend may bend the path."""
    start, goal = np.zeros(17), np.zeros(17)
    goal[3], goal[6] = 1.0, 1.0                 # shoulder pitch (proximal) and elbow (distal) of the left arm
    edge = np.array([0.0, 1.0])                 # sits at the distal_first corner (shoulder still 0, elbow done)

    def clearance_of(q):
        return {"table": float(np.linalg.norm(q[[3, 6]] - edge)) - 0.01, "stand": FAR_M, "basket": FAR_M}

    result = ac.worst_fault_exit(start, goal, clearance_of(start), lambda a, b: float(np.linalg.norm(b - a)), clearance_of)

    assert ac.exit_passes(clearance_of(start), ac.path_clearance(ac.straight_path(start, goal), clearance_of)[0])
    assert result["ordering"] == "distal_first" and not result["passed"]


def test_policy_exits_are_reported_per_waist_limit(scene):
    """Proves: a demo frame turned 0.6 rad gives different start poses and start clearances for waist limits 0.15 and 0.6.
    Catches: a lookup that reuses the 0.6 numbers for a run at 0.15 (the waist columns of rule 16 (b))."""
    state = np.asarray(ac.action_space.BALL_READY_POSE, dtype=float)[None].copy()
    state[0, 26] = 0.6

    at_015, at_06 = ac.demo_frames(state, 0.15)[0], ac.demo_frames(state, 0.6)[0]

    assert at_015[0] == pytest.approx(0.15) and at_06[0] == pytest.approx(0.6)
    assert scene.clearance_of(at_015) != scene.clearance_of(at_06)


def test_fault_exit_sampling_cannot_skip_a_close_pass():
    """Proves: densify turns a coarse leg (10 even samples 13.3 cm apart) into steps <= 2 cm, so a planted edge 1 cm outside
    the fingertip arc, midway between samples 4 and 5, reads <= 1.5 cm instead of ~6.8 cm.
    Catches: an 87 that keeps the coarse corner sampling and passes a real 1 cm brush past the table edge."""
    even = [joint_pose(SWEEP_RAD * k / 9) for k in range(10)]
    edge = edge_at(SWEEP_RAD * 4.5 / 9)

    dense = ac.densify(even, tip_step)

    def closest(path):
        return min(float(np.linalg.norm(tip(q) - edge)) for q in path)
    assert closest(even) > ac.CLEARANCE_MIN_M
    assert closest(dense) <= CLOSE_PASS_MAX_M
    assert max(tip_step(a, b) for a, b in zip(dense, dense[1:])) <= ac.MAX_SAMPLE_STEP_M


def test_start_poses_follow_arm_movement_not_frame_count():
    """Proves: start poses picked from 21 demo frames 6 cm apart are <= 2 cm of fingertip travel apart, so a planted edge
    1 cm outside the arc between frames 14 and 15 reads <= 1.5 cm (every-10th-frame selection would read ~27 cm).
    Catches: a return to fixed frame steps, which hides an unchecked start pose closer to the table edge."""
    frames = [joint_pose(SWEEP_RAD * k / 20) for k in range(21)]
    edge = edge_at(SWEEP_RAD * 14.5 / 20)

    starts, max_start_step = ac.select_start_poses(frames, tip_step)

    assert min(float(np.linalg.norm(tip(s["q"]) - edge)) for s in starts) <= CLOSE_PASS_MAX_M
    assert max_start_step <= ac.MAX_SAMPLE_STEP_M
    assert starts[0]["frame"] == 0 and starts[-1]["frame"] == 20


def planted_part(waist_limit: float, passed: bool, max_step: float = ac.MAX_SAMPLE_STEP_M) -> dict:
    """A minimal 87 report for one waist limit: one elbow (0.9, clear at the nominal table), one to_ready and one fault row."""
    settings = {"elbows": [0.9], "shoulder_pitches": [0.1], "setbacks": [0.0], "waist_limits": [waist_limit],
                "demo": "replay_demo.npz", "CLEARANCE_MIN_M": ac.CLEARANCE_MIN_M, "MAX_SAMPLE_STEP_M": max_step,
                "GRID_BOUNDS": ac.GRID_BOUNDS, "stand_on_table": {"0.00": True}, "ball_distance_m": {"0.00": 0.458}}
    clear = {"table": 0.2, "stand": 0.2, "basket": 0.2}
    common = {"start_clearance_m": clear, "min_clearance_m": clear, "frame": 0.0, "waist_limit": waist_limit, "setback_m": 0.0}
    rows = [{**common, "exit": "to_ready", "ordering": "straight", "elbow": None, "shoulder_pitch": None, "passed": True},
            {**common, "exit": "to_hanging", "ordering": "straight", "elbow": 0.9, "shoulder_pitch": 0.1, "passed": passed}]
    stats = {"samples": {"ready_paths": 1, "policy_exits": 2}, "start_poses": {report_lib.key(waist_limit): 1},
             "max_sample_step_m": 0.01, "max_start_step_m": 0.01, "elapsed_s": 1.0}
    ready = [{"elbow": 0.9, "shoulder_pitch": 0.1, "setback_m": 0.0, "min_clearance_m": 0.08}]
    return {"settings": settings, "stats": stats, "ready_paths": {"rows": ready}, "policy_exits": {"rows": rows}}


def test_merged_report_keeps_every_waist_limit_and_refuses_mixed_settings():
    """Proves: merging two one-waist-limit runs keeps the rows and fault_exits_clear of both limits, and parts with a
    different MAX_SAMPLE_STEP_M are refused.
    Catches: a merge that silently mixes runs with different sampling, or drops one waist column of the lookup."""
    parts = [planted_part(0.15, passed=True), planted_part(0.6, passed=False)]

    merged = report_lib.merge_reports(parts)

    assert merged["summary"]["elbows"][0]["fault_exits_clear"] == {"0.15": True, "0.60": False}
    assert len(merged["policy_exits"]["rows"]) == 4
    with pytest.raises(ValueError, match="MAX_SAMPLE_STEP_M"):
        report_lib.merge_reports([planted_part(0.15, True), planted_part(0.6, True, max_step=0.04)])


def run_87(tmp_path: pathlib.Path, *extra: str) -> dict:
    """87 on a one-frame demo (the ready pose), elbow NEAR_TABLE_ELBOW, one pitch, setback 0, waist 0.15 -> its report."""
    pytest.importorskip("g1sim.arm_clearance_fk", reason="needs the sim env ~/dev/g1-ext/.venv (TableEnv, MuJoCo 3.13)")
    spec = importlib.util.spec_from_file_location("script_87_clearance", SCRIPT_87)
    script = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(script)
    demo, out = tmp_path / "demo.npz", tmp_path / "report.json"
    np.savez(demo, state=np.asarray(ac.action_space.BALL_READY_POSE, dtype=float)[None])
    script.main(["--report", str(out), "--demo", str(demo), "--elbows", str(NEAR_TABLE_ELBOW), "--shoulder-pitches", "0.0",
                 "--setbacks", "0.0", "--waist-limits", "0.15", *extra])
    return json.loads(out.read_text())


def test_arms_up_first_checks_fault_and_release_exits_of_a_skipped_elbow_at_the_nominal_table(tmp_path):
    """Proves: with --arms-up-first, an elbow whose ready path never clears the table (h(0.5, 0): hand in the table, so the
    default lookup says "skip") still gets its fault exits and its `86 release` line checked at the nominal table, and the
    variant summary reads them (release from the ready pose into the table: NOT clear, with a negative distance).
    Catches: the default rule (fault exits only at the elbow's own setback), which checks nothing for a skipped elbow - the
    variant would then report its exits without ever having computed them."""
    default = run_87(tmp_path)
    variant = run_87(tmp_path, "--arms-up-first")

    exits = {row["exit"] for row in variant["policy_exits"]["rows"] if row["elbow"] == NEAR_TABLE_ELBOW}
    release = variant["summary"]["arms_up_first"]["elbows"][0]["waists"]["0.15"]["release"]
    assert not [row for row in default["policy_exits"]["rows"] if row["elbow"] == NEAR_TABLE_ELBOW]
    assert exits == {"to_hanging", "release"}
    assert release["clear"] is False and release["min_m"] < 0, release
