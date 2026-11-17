"""Waypoint arm paths (g1sim/arm_paths.py) and their 87 check (g1sim/arm_paths_clearance.py): the waypoints are what keeps
the hands off table, stand and basket, the lift stays inside 86's joint limits, and the lean/table-height grid reaches the FK.
FK only (no physics, no DDS), seconds. .planning/real-robot/SAFE-PATHS.md has the full 87 run.
    ~/dev/g1-ext/.venv/bin/python -m pytest -m integration tests/test_integration_arm_paths.py -v
"""
import numpy as np
import pytest

from g1sim import arm_clearance as ac
from g1sim import arm_paths, real_safety
from g1sim import arm_paths_clearance as wp

pytestmark = pytest.mark.integration

FACTORY = ac.hanging_pose(1.6, 0.1)     # arms hanging nearly straight: a factory pose clear of the table (SAFE-PATHS.md)
STALE_AT_S = 2.3                        # s after 86 starts: preflight + 2 s ramp-in, then ~10 % into way-in leg 1 (review B1)
STALE_FOR_S = 0.5                       # s without LowState; it is back well inside STALE_RETURN_WAIT_S, so 86 recovers
HANG_GUARD_S = 60.0                     # s: a stop signal only a fault hold still waiting would see (a broken build ends red)
GRASP_FRAME = 153                       # demo frame, both hands at the ball on its stand (runs/safe_paths_probes/probe14.txt)
DEMO = "tests/data/replay_demo.npz"


@pytest.fixture(scope="module")
def scene():
    """The 87 scene (TableEnv volleyball_walk, nominal layout); built once, reset to the default case after each use."""
    fk = pytest.importorskip("g1sim.arm_clearance_fk", reason="needs the sim env ~/dev/g1-ext/.venv (TableEnv, MuJoCo 3.13)")
    return fk.ClearanceModel()


@pytest.fixture(scope="module")
def limits():
    preflight = pytest.importorskip("g1sim.real_preflight", reason="needs the sim env (real_io imports cyclonedds)")
    return preflight.LIMITS_LO, preflight.LIMITS_HI


def path_min(scene, waypoints) -> float:
    """Smallest clearance to any object along the waypoint legs, densified as 87 does, start pose included."""
    dense = [q for a, b in zip(waypoints, waypoints[1:]) for q in ac.densify(ac.move_path(a, b), scene.step_of)]
    return min(min(scene.clearance_of(q).values()) for q in dense)


def test_way_in_clears_the_table_where_the_straight_line_does_not(scene):
    """Proves: from a factory pose that is itself clear, 86's straight line to the ready pose comes closer than 5 cm to table,
    stand or basket, while the waypoint way in keeps >= 5 cm all the way.
    Catches: a waypoint edited back towards a forward sweep (e.g. SIDE_UP equal to the ready pose), which 87 would then report
    as not clear - and a scene in which the straight line is already clear, so the waypoints would prove nothing."""
    straight = path_min(scene, [FACTORY, ac.READY17])
    waypoints = path_min(scene, arm_paths.way_in(FACTORY, ac.READY17))

    assert straight < ac.CLEARANCE_MIN_M, straight
    assert waypoints >= ac.CLEARANCE_MIN_M, waypoints


def test_lift_home_leaves_the_ball_stand_where_the_straight_exit_does_not(scene, limits):
    """Proves: from the demo pose with both hands at the ball, the straight soft-exit line to the ready pose breaks the exit
    rule (closer than 5 cm or than the start), while lift -> waist home -> ready keeps it.
    Catches: a lift that does nothing (LIFT_DELTA_RIGHT emptied or zero) or a way out that turns the waist first."""
    start = ac.demo_frames(np.load(DEMO)["state"], 0.6)[GRASP_FRAME]
    start_clear = scene.clearance_of(start)

    def along(waypoints):
        dense = [start] + [q for a, b in zip(waypoints, waypoints[1:]) for q in ac.densify(ac.move_path(a, b), scene.step_of)[1:]]
        return ac.path_clearance(dense, scene.clearance_of)[0]

    straight = along([start, ac.READY17])
    lifted = along(arm_paths.way_out(start, FACTORY, ac.READY17, *limits)[:4])

    assert not ac.exit_passes(start_clear, straight), straight
    assert ac.exit_passes(start_clear, lifted), lifted


def test_lift_stays_inside_the_joint_limits_86_clips_to(limits):
    """Proves: lifting a pose whose elbows are already bent near their limit commands the elbow exactly LIMIT_MARGIN inside it.
    Catches: an unclipped lift - 86's SafetyFilter would clip it and send a different line than the one 87 checked."""
    lo, hi = limits
    bent = np.array(ac.READY17)
    bent[[arm_paths.LEFT_ARM.start + arm_paths.ELBOW, arm_paths.RIGHT_ARM.start + arm_paths.ELBOW]] = -0.9

    lifted = arm_paths.lift(bent, lo, hi)

    elbows = [arm_paths.LEFT_ARM.start + arm_paths.ELBOW, arm_paths.RIGHT_ARM.start + arm_paths.ELBOW]
    assert lifted[elbows] == pytest.approx(lo[elbows] + real_safety.LIMIT_MARGIN)
    assert np.all(lifted >= lo + real_safety.LIMIT_MARGIN - 1e-12) and np.all(lifted <= hi - real_safety.LIMIT_MARGIN + 1e-12)


def test_lean_and_table_height_change_the_clearance_87_reads(scene):
    """Proves: a pose with the fingers over the table edge (elbow 0) reads a different table clearance at lean pitch 0.041 vs
    0.126 and at table +1 cm vs -4 cm, and the default case reads its old value again afterwards.
    Catches: lean or table height missing from the FK cache key or never applied - 87's 27 cases would then all be one case."""
    pose = ac.hanging_pose(0.0, 0.1)
    default = scene.clearance_of(pose)["table"]
    readings = {}
    for name, roll, pitch, dz in (("upright", 0.0, 0.041, 0.0), ("lean", 0.0, 0.126, 0.0), ("high", 0.0, 0.126, 0.01),
                                  ("low", 0.0, 0.126, -0.04)):
        scene.set_lean(roll, pitch)
        scene.set_table_height(dz)
        readings[name] = scene.clearance_of(pose)["table"]
    scene.set_lean(0.0, ac.WAIST_PITCH_CHECKED)
    scene.set_table_height(0.0)

    assert readings["upright"] != pytest.approx(readings["lean"], abs=1e-3)
    assert readings["high"] != pytest.approx(readings["low"], abs=1e-3)
    assert scene.clearance_of(pose)["table"] == default


def test_recover_on_the_way_in_goes_back_where_lift_and_ready_would_hit_the_table(scene, limits):
    """Proves: a fault early on way-in leg 1 (hands low beside the table edge), then arm_paths.recover, keeps >= 5 cm all the
    way back to the factory pose, while the old prescription from there (lift -> waist home -> ready) goes into the table.
    Catches: recover on the way in replaced by way_out from the measured pose (review safe-paths B1) - and a scene in which
    the old prescription is clear anyway, so the test would prove nothing."""
    plan = arm_paths.way_in(FACTORY, ac.READY17)
    start = plan[0] + (plan[1] - plan[0]) * 0.1

    old = path_min(scene, arm_paths.way_out(start, FACTORY, ac.READY17, *limits)[:4])
    new = path_min(scene, arm_paths.recover(start, arm_paths.WAY_IN, plan, 1))

    assert old < 0.0, old
    assert new >= ac.CLEARANCE_MIN_M, new


def test_lean_cases_reach_the_upright_end_where_the_factory_pose_is_closest(scene):
    """Proves: over 87's lean cases the factory pose h(1.1, 0.1) reads a smaller table clearance than in the cases with
    pitch >= 0.041 (the upright end counts: the hanging hands are closest to the table at the smallest q0 pitch).
    Catches: LEAN_PITCHES starting at 0.041 again (review safe-paths B2) - 87 would read 5.6 cm where the upright robot
    has 4 cm."""
    pose = ac.hanging_pose(1.1, 0.1)

    def table_clearance(case):
        wp.set_case(scene, case)
        return scene.clearance_of(pose)["table"]

    every_case = min(table_clearance(c) for c in wp.scene_cases())
    from_q0_max = min(table_clearance(c) for c in wp.scene_cases() if c["lean_pitch"] >= wp.WAIST_PITCH_Q0_MAX)
    wp.reset_scene(scene)

    assert every_case < from_q0_max - 0.005, (every_case, from_q0_max)


def test_measured_factory_pose_outside_the_checked_lean_is_refused():
    """Proves: 87 --factory-pose checks the recorded pose itself, and refuses a recorded q0 waist pitch below the lean cases.
    Catches: a missing refusal - 87 would report 'clear' for a torso more upright than it checked (review safe-paths B2/B3)."""
    record = ac.hanging_pose(1.1, 0.1)
    record[wp.WAIST_PITCH_POS] = 0.0
    assert wp.measured_factory(record)[0][2] == pytest.approx(record)

    record[wp.WAIST_PITCH_POS] = wp.WAIST_PITCH_Q0_MIN - 0.01
    with pytest.raises(ValueError, match="skip table steps"):
        wp.measured_factory(record)


def test_86_way_in_and_fault_recover_keep_5_cm_from_the_table(scene, tmp_path):
    """Proves: what 86 itself sends from the factory pose h(1.6, 0.1) keeps every arm and hand geom >= 5 cm from table,
    stand and basket by 87's FK: a whole `ready` (way in, hold, back along the way in), and a run with a fault low on
    way-in leg 1 (LowState gone for 0.5 s), its hold and its recover back to the factory pose. Both hand the arms back
    only at the factory pose.
    Catches: the old straight line put back into move_to_ready, the recover from there replaced by the way out (lift ->
    ready sweeps the low hands into the table, review safe-paths B1), or a fault that ramps out away from the factory pose
    (Unitree's own path then crosses the table) [mutation]."""
    pytest.importorskip("unitree_sdk2py", reason="the 86 runner fakes need unitree_sdk2py (sim env)")
    from helpers.fake_real_run import FakeArm, FakeClock, make_io, run, sent, signals_at

    def ready_run(name, stale_at=None):
        clock = FakeClock()
        arm = FakeArm(clock, FACTORY)
        if stale_at is not None:
            arm.stale_after, arm.stale_until = clock() + stale_at, clock() + stale_at + STALE_FOR_S
        result = run("ready", make_io(clock, arm, signal_count=signals_at(clock, HANG_GUARD_S)), tmp_path / name, hold=0.5)
        return result, sent(result, "sent_phase"), sent(result, "sent_q")

    whole, whole_phases, whole_q = ready_run("whole")
    fault, fault_phases, fault_q = ready_run("fault", STALE_AT_S)

    assert whole.exit_code == 0 and (fault.exit_code, fault.reason) == (4, "lowstate_stale")
    assert "to_factory" in fault_phases and "hold" not in fault_phases, "the fault did not strike on the way in"
    for phases, q in ((whole_phases, whole_q), (fault_phases, fault_q)):
        clearance = min(min(scene.clearance_of(pose).values()) for pose in q)
        assert clearance >= ac.CLEARANCE_MIN_M, clearance
        assert np.allclose(q[phases == "ramp_out"][-1], FACTORY, atol=1e-9), "handed back away from the factory pose"
