"""Unit: calm path following (2026-10-09) - occupancy.smooth_path, navigation.speed_profile, navigation.SmoothPathFollower.

With the L2 follower on the raw grid path the robot wobbled 9-10 cm around the path (max 28 cm, 68 seeds 1 and 3). Guarded
here: smoothing never brings the path closer to obstacles than A* did and keeps both ends; the speed profile slows down in
curves and stays above GR00T's dead band; the follower walks and turns at the same time, turns on the spot only when the
path starts behind it, and changes its commands only gradually.
"""
import math

import numpy as np
import pytest

from g1sim import approach, occupancy
from g1sim import navigation as nav
from g1sim.navigation import Pose2D

pytestmark = pytest.mark.unit

ROBOT = Pose2D(0.0, 0.0, 0.0)


def max_cost(points, cost, grid):
    return max(float(cost[grid.to_cell(*p)]) for p in points)


@pytest.mark.parametrize("scenario", list(approach.SCENARIOS))
def test_smoothed_path_is_shorter_straighter_and_not_closer_to_obstacles(scenario):
    rng = np.random.default_rng(4)
    for _ in range(6):
        setup = approach.sample(scenario, rng, ROBOT)
        raw, smooth = approach.plan(ROBOT, setup), approach.plan_smooth(ROBOT, setup)
        grid = approach.grid_for(ROBOT, setup)
        cost = grid.costmap()
        assert np.allclose(smooth[0], raw[0]) and np.allclose(smooth[-1], raw[-1])
        assert occupancy.path_length(smooth) <= occupancy.path_length(raw) + 1e-9
        assert max_cost(smooth, cost, grid) <= max(5.0, max_cost(raw, cost, grid)) + 1e-9
        steps = np.linalg.norm(np.diff(np.asarray(smooth), axis=0), axis=1)
        assert steps.max() <= occupancy.RESOLUTION + 1e-9  # finely sampled for the follower


def test_shortcut_keeps_straight_lines_and_goes_round_a_wall():
    grid = occupancy.Grid(((-1.0, 3.0), (-1.0, 3.0)))
    cost = grid.costmap()  # no walls: everything costs 1
    zigzag = [(0.0, 0.0), (0.5, 0.05), (1.0, 0.0), (1.5, 0.05), (2.0, 0.0)]
    assert occupancy.shortcut(zigzag, cost, grid) == [(0.0, 0.0), (2.0, 0.0)]
    grid.add_box((1.0, 0.0), (0.05, 0.8))  # wall across the straight line
    cost = grid.costmap()
    around = [(0.0, 0.0), (0.0, 1.8), (2.0, 1.8), (2.0, 0.0)]
    kept = occupancy.shortcut(around, cost, grid)
    assert kept[0] == (0.0, 0.0) and kept[-1] == (2.0, 0.0) and len(kept) >= 3
    assert all(occupancy.segment_cost(cost, grid, a, b) < math.inf for a, b in zip(kept[:-1], kept[1:]))


def test_speed_profile_slows_in_curves_and_respects_limits():
    p = nav.SmoothParams()
    straight = occupancy.resample([(0.0, 0.0), (4.0, 0.0)])
    corner = occupancy.smooth([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)])
    v_straight, v_corner = nav.speed_profile(straight, p), nav.speed_profile(corner, p)
    assert v_straight.max() == pytest.approx(p.v_max)
    assert v_corner.min() >= p.v_min and v_straight.min() >= p.v_min
    middle = len(corner) // 2
    assert v_corner[middle] < 0.8 * p.v_max  # slower in the bend
    assert v_straight[0] <= p.v_min + 1e-9 and v_straight[-1] == pytest.approx(p.v_end)  # gentle start and end
    ds = occupancy.RESOLUTION
    assert np.all(np.abs(np.diff(v_straight ** 2)) <= 2 * p.a_lin * ds + 1e-9)


def run(follower, start, steps=600, lag=0.045, dt=0.05):
    """Unicycle with GR00T's forward lag. Returns the poses and the commands."""
    pose, poses, cmds = start, [], []
    for k in range(steps):
        vx, vy, omega = follower.step(pose, k * dt)
        cmds.append((vx, vy, omega))
        v = max(0.0, vx - lag)
        pose = Pose2D(pose.x + dt * v * math.cos(pose.yaw), pose.y + dt * v * math.sin(pose.yaw), nav.wrap(pose.yaw + dt * omega))
        poses.append(pose)
        if math.hypot(pose.x - follower.goal.x, pose.y - follower.goal.y) < 0.25:
            break
    return poses, np.array(cmds)


def test_follower_walks_and_turns_at_the_same_time_and_stays_on_the_path():
    path = occupancy.smooth([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)])
    goal = Pose2D(2.0, 2.0, math.pi / 2)
    poses, cmds = run(nav.SmoothPathFollower(path, goal), ROBOT)
    assert math.hypot(poses[-1].x - goal.x, poses[-1].y - goal.y) < 0.3
    dense = np.asarray(occupancy.resample(path, 0.01))
    deviation = [np.min(np.linalg.norm(dense - (p.x, p.y), axis=1)) for p in poses]
    assert max(deviation) < 0.10
    assert np.any((cmds[:, 0] > 0.1) & (np.abs(cmds[:, 2]) > 0.1))  # walking while turning in the bend
    assert np.all(cmds[:, 1] == 0.0)  # never sideways while walking


def test_follower_turns_on_the_spot_only_when_the_path_starts_behind():
    behind = occupancy.resample([(0.0, 0.0), (-2.0, 0.0)])
    follower = nav.SmoothPathFollower(behind, Pose2D(-2.0, 0.0, math.pi))
    _, cmds = run(follower, ROBOT, steps=10)
    assert np.all(np.abs(cmds[:, 0]) <= nav.SmoothParams().hold_max) and np.all(cmds[:, 2] != 0.0)  # at most holding the spot
    ahead = occupancy.resample([(0.0, 0.0), (2.0, 0.3)])
    _, cmds = run(nav.SmoothPathFollower(ahead, Pose2D(2.0, 0.3, 0.0)), ROBOT, steps=40)
    assert np.all(cmds[1:, 0] > 0.0)


def test_follower_commands_change_gradually():
    p = nav.SmoothParams()
    path = occupancy.smooth([(0.0, 0.0), (1.5, 0.0), (1.5, -1.5), (3.0, -1.5)])
    _, cmds = run(nav.SmoothPathFollower(path, Pose2D(3.0, -1.5, 0.0), p), ROBOT, dt=0.05)
    assert np.abs(np.diff(cmds[:, 0])).max() <= p.acc_v * 0.05 + 1e-9
    assert np.abs(np.diff(cmds[:, 2])).max() <= p.acc_omega * 0.05 + 1e-9


def test_turning_on_the_spot_steps_back_to_where_the_turn_started():
    """GR00T drifts while it turns on the spot: the follower sends the robot back towards the start of the turn."""
    behind = occupancy.resample([(0.0, 0.0), (-2.0, 0.0)])
    follower = nav.SmoothPathFollower(behind, Pose2D(-2.0, 0.0, math.pi))
    follower.step(ROBOT, 0.0)  # starts turning here
    for k in range(1, 40):
        vx, vy, _ = follower.step(Pose2D(0.0, 0.15, 0.0), 0.05 * k)  # drifted 15 cm to the left, still facing +x
    assert vy < 0.0 and abs(vy) == pytest.approx(nav.SmoothParams().hold_max)
