"""Unit: walk up to the table (g1sim/approach.py) - dock geometry, table-frame error, scenarios, paths, docking controller.

A sign error in the table frame would make 68 report a good arrival for a robot standing beside the table; a scenario that
silently draws the easy case ("behind the table" with the dock on the near side) would make the evaluation look better than
it is. Both show up here in seconds instead of after minutes of walking sim.
"""
import math

import numpy as np
import pytest

from g1sim import approach, occupancy
from g1sim.navigation import Pose2D, wrap

pytestmark = pytest.mark.unit

ROBOT = Pose2D(0.3, -0.2, 0.4)
SAMPLES = 15


def test_dock_pose_is_the_grasp_scene_stand():
    """Dock = pelvis DOCK_DIST in front of the table centre, looking at it: CROUCH_BACK behind the volleyball scene's pelvis."""
    table = Pose2D(2.0, 1.0, math.radians(30))
    dock = approach.dock_pose(table)
    assert math.hypot(table.x - dock.x, table.y - dock.y) == pytest.approx(approach.DOCK_DIST)
    assert math.atan2(table.y - dock.y, table.x - dock.x) == pytest.approx(table.yaw)
    assert approach.table_frame_error(dock, table) == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_dock_matches_the_volleyball_scene():
    table_scene = pytest.importorskip("g1sim.table_scene")
    centre_x, half = table_scene.SCENES["volleyball"]["table"]
    assert approach.TABLE_CENTRE_X == pytest.approx(centre_x)
    assert approach.DOCK_DIST == pytest.approx(centre_x + approach.CROUCH_BACK)
    assert approach.TABLE_HALF == pytest.approx(half)
    assert approach.TABLE_TOP == pytest.approx(table_scene.TABLE_TOP)


def test_table_frame_error_signs():
    """Same meaning as the stand offset of the grasp scene: +dx closer to the table, +dy to the left, +yaw turned left."""
    table = Pose2D(1.0, 2.0, math.radians(90))  # robot docks looking along +y
    dock = approach.dock_pose(table)
    closer = Pose2D(dock.x, dock.y + 0.05, dock.yaw)
    left = Pose2D(dock.x - 0.03, dock.y, dock.yaw)  # looking along +y, left is -x
    turned = Pose2D(dock.x, dock.y, dock.yaw + 0.1)
    assert approach.table_frame_error(closer, table) == pytest.approx((0.05, 0.0, 0.0), abs=1e-12)
    assert approach.table_frame_error(left, table) == pytest.approx((0.0, 0.03, 0.0), abs=1e-12)
    assert approach.table_frame_error(turned, table) == pytest.approx((0.0, 0.0, 0.1), abs=1e-12)


def test_acceptance_is_the_teacher_range():
    assert approach.accepted((0.099, -0.099, math.radians(9.9)))
    assert not approach.accepted((0.11, 0.0, 0.0))
    assert not approach.accepted((0.0, 0.0, math.radians(10.5)))


def test_rect_gap():
    a = (0.0, 0.0, 0.0, (0.5, 0.5))
    assert approach.rect_gap(a, (2.0, 0.0, 0.0, (0.5, 0.5))) == pytest.approx(1.0)
    assert approach.rect_gap(a, (0.8, 0.0, 0.0, (0.5, 0.5))) == 0.0
    # turned 45 deg: its edge x + y = 4 - 0.5 sqrt(2) faces the corner (0.5, 0.5) of a
    assert approach.rect_gap(a, (2.0, 2.0, math.pi / 4, (0.5, 0.5))) == pytest.approx((3.0 - 0.5 * math.sqrt(2)) / math.sqrt(2))


@pytest.mark.parametrize("scenario", list(approach.SCENARIOS))
def test_scenario_draws_are_valid(scenario):
    """Robot clear of the table and obstacles, obstacles apart from table, dock and each other, and a path exists."""
    rng = np.random.default_rng(0)
    for _ in range(SAMPLES):
        setup = approach.sample(scenario, rng, ROBOT)
        assert setup.scenario == scenario
        assert approach.rect_distance(ROBOT.x, ROBOT.y, *approach.table_rect(setup.table)) >= approach.ROBOT_CLEAR
        rects = [approach.obstacle_rect(o) for o in setup.obstacles]
        assert len({o.name for o in setup.obstacles}) == len(setup.obstacles)
        for k, r in enumerate(rects):
            assert approach.rect_distance(ROBOT.x, ROBOT.y, *r) >= approach.ROBOT_CLEAR
            assert approach.rect_gap(r, approach.table_rect(setup.table)) >= approach.OBSTACLE_GAP
            for other in rects[k + 1:]:
                assert approach.rect_gap(r, other) >= approach.OBSTACLE_GAP
        assert approach.plan(ROBOT, setup) is not None


def bearing(setup):
    """Direction of the table centre in the robot frame (rad)."""
    return wrap(math.atan2(setup.table.y - ROBOT.y, setup.table.x - ROBOT.x) - ROBOT.yaw)


def test_behind_table_puts_the_dock_on_the_far_side():
    """The robot stands behind the table: the dock is further away than the table centre, and the path goes round it."""
    rng = np.random.default_rng(1)
    for _ in range(SAMPLES):
        setup = approach.sample("behind_table", rng, ROBOT)
        dock = approach.dock_pose(setup.table)
        assert abs(bearing(setup)) <= math.radians(25) + 1e-9
        assert math.hypot(dock.x - ROBOT.x, dock.y - ROBOT.y) > math.hypot(setup.table.x - ROBOT.x, setup.table.y - ROBOT.y)
        pre = approach.checkpoint_pose(setup.table)
        straight = math.hypot(pre.x - ROBOT.x, pre.y - ROBOT.y)
        assert occupancy.path_length(approach.plan(ROBOT, setup)) > straight + 0.3


def test_turned_away_puts_the_table_behind_the_robot():
    rng = np.random.default_rng(2)
    for _ in range(SAMPLES):
        setup = approach.sample("turned_away", rng, ROBOT)
        assert abs(bearing(setup)) >= math.radians(150) - 1e-9
        dock = approach.dock_pose(setup.table)  # work side towards the robot: dock nearer than the table centre
        assert math.hypot(dock.x - ROBOT.x, dock.y - ROBOT.y) < math.hypot(setup.table.x - ROBOT.x, setup.table.y - ROBOT.y)


def test_obstacle_course_has_obstacles_and_paths_avoid_them():
    rng = np.random.default_rng(3)
    for _ in range(SAMPLES):
        setup = approach.sample("obstacle_course", rng, ROBOT)
        assert 1 <= len(setup.obstacles) <= 3
        path = approach.plan(ROBOT, setup)
        for o in setup.obstacles:
            assert min(approach.rect_distance(x, y, *approach.obstacle_rect(o)) for x, y in path) >= 0.25


def test_same_seed_same_setups():
    draw = lambda seed: [approach.sample(s, np.random.default_rng(seed), ROBOT) for s in approach.SCENARIOS]  # noqa: E731
    assert draw(5) == draw(5)


def simulate_dock(table, start, steps=1500, lag=0.045, settle_back=0.02):
    """Point robot with GR00T's measured forward lag (actual = cmd - lag, at least 0) and settle-back after a stop.
    Returns the final pose and the docker."""
    docker = approach.Docker(table)
    pose, dt = start, 1.0 / 20
    for _ in range(steps):
        vx, vy, omega = docker.step(pose)
        vx = math.copysign(max(0.0, abs(vx) - lag), vx)
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        pose = Pose2D(pose.x + dt * (c * vx - s * vy), pose.y + dt * (s * vx + c * vy), wrap(pose.yaw + dt * omega))
        if docker.done:
            pose = Pose2D(pose.x - settle_back * c, pose.y - settle_back * s, pose.yaw)  # settles back once
            error = approach.table_frame_error(pose, table)
            if not docker.needs_redock(error):
                return pose, docker
            docker.redock()
    return pose, docker


@pytest.mark.parametrize("offset", [(-0.40, 0.10, 15.0), (-0.30, -0.15, -20.0), (-0.10, 0.0, 0.0), (0.06, 0.05, 5.0)])
def test_docker_reaches_the_grasp_pose_despite_the_dead_band(offset):
    """A plain P controller stalls 4-6 cm short (68, first try): the docker gets within its re-dock tolerance."""
    table = Pose2D(1.5, 0.5, math.radians(40))
    dx, dy, dyaw = offset
    dock = approach.dock_pose(table)
    c, s = math.cos(table.yaw), math.sin(table.yaw)
    start = Pose2D(dock.x + c * dx - s * dy, dock.y + s * dx + c * dy, table.yaw + math.radians(dyaw))
    pose, docker = simulate_dock(table, start)
    ex, ey, eyaw = approach.table_frame_error(pose, table)
    assert docker.done
    assert abs(ex) <= approach.DOCK.redock_xy and abs(ey) <= approach.DOCK.redock_xy
    assert abs(eyaw) <= approach.DOCK.redock_yaw


def test_docker_adds_the_dead_band_only_while_clearly_off():
    table = Pose2D(0.0, 0.0, 0.0)
    far = approach.Docker(table)
    for _ in range(40):  # let the acceleration ramp settle
        cmd = far.step(approach.along(table, approach.DOCK_DIST + 0.2))
    assert cmd[0] == pytest.approx(min(approach.DOCK.max_vx, approach.DOCK.k_lin * 0.22 + approach.DOCK.vx_dead_band))
    near = approach.Docker(table)
    for _ in range(40):
        cmd = near.step(approach.along(table, approach.DOCK_DIST - approach.DOCK.aim_deeper))
    assert cmd == (0.0, 0.0, 0.0) and near.done


def test_redock_is_limited():
    docker = approach.Docker(Pose2D(0.0, 0.0, 0.0))
    off = (0.06, 0.0, 0.0)
    for _ in range(approach.DOCK.max_attempts - 1):
        assert docker.needs_redock(off)
        docker.redock()
    assert not docker.needs_redock(off)
    assert not approach.Docker(Pose2D(0.0, 0.0, 0.0)).needs_redock((0.03, -0.03, math.radians(3.0)))


# --- checkpoint + straight in (2026-10-09, the user's spec) ---------------------------------------------------------------
def test_checkpoint_is_on_the_line_three_to_four_steps_out():
    table = Pose2D(2.0, 1.0, math.radians(-50))
    check, dock = approach.checkpoint_pose(table), approach.dock_pose(table)
    assert math.hypot(check.x - dock.x, check.y - dock.y) == pytest.approx(approach.CHECKPOINT_BACK)
    assert approach.table_frame_error(check, table)[1:] == pytest.approx((0.0, 0.0), abs=1e-12)
    assert approach.checkpoint_error(check, table) == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


@pytest.mark.parametrize("scenario", list(approach.SCENARIOS))
def test_path_ends_on_the_line_facing_the_table(scenario):
    rng = np.random.default_rng(6)
    for _ in range(6):
        setup = approach.sample(scenario, rng, ROBOT)
        path = np.asarray(approach.plan_smooth(ROBOT, setup))
        check = approach.checkpoint_pose(setup.table)
        assert np.allclose(path[-1], (check.x, check.y))
        # the join onto the line is rounded (a sharp corner made the robot cut it), the last 25 cm are on the line
        tail = [approach.table_frame_error(Pose2D(x, y, setup.table.yaw), setup.table) for x, y in path[-6:]]
        assert max(abs(e[1]) for e in tail) < 0.01
        assert all(b[0] > a[0] for a, b in zip(tail[:-1], tail[1:]))  # and moving towards the table


def test_corridor_into_the_table_stays_free():
    rng = np.random.default_rng(8)
    for scenario in ("obstacle_course", "anywhere"):
        for _ in range(SAMPLES):
            setup = approach.sample(scenario, rng, ROBOT)
            for o in setup.obstacles:
                for d in np.arange(approach.DOCK_DIST, approach.LEAD_IN_DIST, 0.05):
                    p = approach.along(setup.table, d)
                    assert approach.rect_distance(p.x, p.y, *approach.obstacle_rect(o)) >= approach.CORRIDOR_CLEAR - 0.05


@pytest.mark.parametrize("offset", [(0.0, 0.0, 0.0), (0.0, 0.03, 3.0), (0.04, -0.03, -2.5), (-0.03, 0.02, 2.0)])
def test_straight_in_reaches_the_grasp_pose_without_side_steps(offset):
    """From a slightly misaligned checkpoint: arrives along the axis within the redo tolerance, never commands vy, keeps the
    body pointed at the table (no penguin walk), and gets back onto the line."""
    table = Pose2D(1.0, -0.5, math.radians(120))
    check = approach.checkpoint_pose(table)
    c, s = math.cos(table.yaw), math.sin(table.yaw)
    dx, dy, dyaw = offset
    pose = Pose2D(check.x + c * dx - s * dy, check.y + s * dx + c * dy, table.yaw + math.radians(dyaw))
    straight, dt, headings = approach.StraightIn(table), 0.05, []
    for k in range(2000):
        vx, vy, omega = straight.step(pose, k * dt)
        assert vy == 0.0
        v = math.copysign(max(0.0, abs(vx) - 0.045), vx)  # GR00T's forward lag
        pose = Pose2D(pose.x + dt * v * math.cos(pose.yaw), pose.y + dt * v * math.sin(pose.yaw), wrap(pose.yaw + dt * omega))
        headings.append(abs(approach.table_frame_error(pose, table)[2]))
        if straight.done:
            break
    ex, ey, eyaw = approach.table_frame_error(pose, table)
    assert straight.done
    assert abs(ex) <= approach.STRAIGHT.redo_along
    assert abs(ey) < abs(dy) + 0.005  # not further off the line than at the checkpoint
    assert max(headings) <= math.radians(max(abs(dyaw), 0.0)) + approach.STRAIGHT.line_heading_max + 1e-6
    assert abs(eyaw) < math.radians(4.0)


def test_straight_in_redo_only_along_the_axis():
    straight = approach.StraightIn(Pose2D(0.0, 0.0, 0.0))
    assert straight.needs_redo((0.06, 0.0, 0.0))
    assert not straight.needs_redo((0.02, 0.08, math.radians(8)))  # sideways / heading: not by side steps at the table
