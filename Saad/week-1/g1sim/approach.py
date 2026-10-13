"""Level 4 step - walk up to the table from random start situations, then dock at the grasp pose (pure numpy, no MuJoCo).

The walk ends where Level 1 begins: the pelvis DOCK_DIST in front of the table centre (CROUCH_BACK behind the volleyball
scene's stand pose, because the robot stands in GR00T's crouch),
looking at the table. Arrival error = stand offset of that scene (randomization group "stand"): the two-hand teacher was
checked for +-10 cm and +-10 degrees there (100/100, 2026-10-08), so that is the acceptance range here too.

Each trial puts the table (and 0-3 obstacles) at a random pose RELATIVE TO WHERE THE ROBOT STANDS NOW, drawn from a
scenario (SCENARIOS): table ahead, robot behind the table (walk around it), robot turned away (turn round first), table
beside, obstacles in the way, anything. Moving the world instead of the robot keeps LeRobot's sim untouched (no teleport of
a walking robot) and lets the trials chain: the next start is wherever the last trial ended.

Chain per trial (scripts/68_walk_to_table.py), as the user specified it on 2026-10-09:
  sample(scenario, rng, robot) -> Setup (table pose, obstacles)
  plan(robot, setup)           -> A* path (g1sim/occupancy.py) around table and obstacles to the LEAD-IN point, then a
  plan_smooth(robot, setup)       straight line to the CHECKPOINT (smooth: the A* part pulled taut, corners rounded)
  walk  (arms down)            -> navigation.SmoothPathFollower: a natural walk, the robot arrives on the line into the table
  align (arms down)            -> Docker at the checkpoint, CHECKPOINT_BACK (~3-4 steps) before the grasp pose: on the line,
                                  facing the table, a few cm / degrees - small side steps are fine here, away from the table
  raise the arms               -> standing still, arms to BALL_READY_POSE (they must not swing into the table)
  approach                     -> StraightIn: forward steps only, straight into the table, no side steps (no "penguin walk"
                                  with the body turned away); the heading only steers the robot back onto the line
  table_frame_error(...)       -> (dx, dy, dyaw) of the pelvis against the dock pose, in the table frame
Table frame like the grasp scene: x from the robot's dock towards the table, y to the left, origin at the dock pose.
"""
import collections
import dataclasses
import math

import numpy as np

from g1sim import occupancy
from g1sim.navigation import CONTROL_HZ, Pose2D, to_body, wrap

# Table of the volleyball scene (table_scene.SCENES["volleyball"]["table"]): centre 0.525 m in front of the pelvis,
# half edges 0.325 x 0.45 m, top at 0.74 m. Copied, not imported: this module must run without MuJoCo.
TABLE_HALF = (0.325, 0.45)
TABLE_TOP = 0.74
TABLE_CENTRE_X = 0.525   # m, pelvis -> table centre in the volleyball grasp scene (front edge 0.20 m away)
# The walk stops this much further back than the grasp scene's pelvis: GR00T leaves the robot in its crouch (pelvis 0.75 m,
# not 0.793 m as in the grasp scene), and there the two-hand teacher only passes with the pelvis 3-7 cm behind the grasp
# scene's spot, 9/9 at 5 cm (sweep in table_scene's "volleyball_walk" scene, +-3 cm sideways, +-3 deg, 2026-10-09). At 0 cm
# the hands hit table and stand and knock the ball off.
CROUCH_BACK = 0.05
DOCK_DIST = TABLE_CENTRE_X + CROUCH_BACK  # m, pelvis -> table centre where the walk stops
CHECKPOINT_BACK = 0.75   # m, checkpoint before the grasp pose: ~3-4 steps of the slow final approach
CHECKPOINT_DIST = DOCK_DIST + CHECKPOINT_BACK
LEAD_IN = 0.70           # m, the planned path reaches the line this far before the checkpoint, so it arrives facing the table
                         # (0.5 m: rounding a sharp join onto the line left only the last 20 cm exactly on it)
LEAD_IN_DIST = CHECKPOINT_DIST + LEAD_IN

# Acceptance at the table = what the two-hand teacher handles (randomization group "stand", 100/100 on 2026-10-08)
ACCEPT_XY = 0.10         # m, per axis in the table frame
ACCEPT_YAW = math.radians(10.0)

# Obstacles: (half x, half y, height, rgba). Taller than the hands (~0.85 m at the zero arm pose, reaching 0.42 m in front
# of the pelvis) get HAND_MARGIN more room in the map, lower ones the hands pass over like the table top.
OBSTACLES = {
    "obstacle_crate": ((0.22, 0.22), 0.45, (0.62, 0.48, 0.30, 1.0)),
    "obstacle_cabinet": ((0.30, 0.20), 1.20, (0.45, 0.50, 0.58, 1.0)),
    "obstacle_bin": ((0.18, 0.18), 0.70, (0.25, 0.42, 0.30, 1.0)),
}
HAND_HEIGHT = 0.80
HAND_MARGIN = 0.15

# Minimum gaps while sampling (m)
ROBOT_CLEAR = 0.60       # robot centre -> table or obstacle edge at the start (hands reach 0.42 m)
OBSTACLE_GAP = 0.85      # obstacle edge -> table edge and between obstacles: a corridor wider than the robot
DOCK_CLEAR = 0.90        # dock and checkpoint -> obstacle edge
CORRIDOR_CLEAR = 0.75    # line lead-in -> dock -> obstacle edge
MAP_MARGIN = 2.0         # map extends this far beyond robot, table and obstacles
MAX_DRAWS = 500



@dataclasses.dataclass(frozen=True)
class DockParams:
    """Docking at the table. Numbers from a GR00T tracking probe (2026-10-08, robot standing, arms in READY_POSE, 4 s per
    command): forward it walks ~0.045 m/s slower than commanded (0.05 -> 0.009, 0.10 -> 0.061, 0.20 -> 0.154 m/s), sideways
    nearly as commanded (0.10 -> 0.086); after a slow stop it settles ~2 cm back. With a plain P controller the robot hovered
    4-6 cm short for 30 s: the small commands near the goal were inside that dead band."""
    k_lin: float = 0.8              # 1/s
    k_yaw: float = 1.2              # 1/s
    max_vx: float = 0.20            # m/s forward (backward: half)
    max_vy: float = 0.12            # m/s
    max_omega: float = 0.30         # rad/s
    max_accel: float = 0.4          # m/s^2 (rad/s^2)
    vx_dead_band: float = 0.045     # m/s added in the direction of travel (forward lag of GR00T)
    vy_dead_band: float = 0.015
    omega_dead_band: float = 0.04
    aim_deeper: float = 0.02        # m, aim this much closer to the table: the robot settles back ~2 cm after stopping
    filter_s: float = 0.6           # s, average the pelvis pose over ~2 steps: the pelvis sways +-2 cm per step
    pos_tol: float = 0.03           # m, filtered error per axis to stop
    yaw_tol: float = math.radians(3.0)
    hold_s: float = 0.3             # inside the tolerance this long, then stop
    redock_xy: float = 0.04         # m, after standing: dock again if further off than this (per axis) ...
    redock_yaw: float = math.radians(4.0)  # ... or turned more than this
    max_attempts: int = 3


DOCK = DockParams()
# Aligning at the checkpoint: same controller, aimed at the checkpoint itself, a bit tighter sideways and in heading, because
# from here on the robot only walks straight
ALIGN = DockParams(aim_deeper=0.0, pos_tol=0.03, yaw_tol=math.radians(2.5), redock_xy=0.04, redock_yaw=math.radians(3.0))


class Docker:
    """Last metres to the grasp pose: always facing the table, steps sideways when needed (no turning towards the goal).
    step(pose, t) once per control tick; returns (vx, vy, omega). done is True once it has stopped inside the tolerance;
    the caller then lets the robot stand, measures, and calls redock() if the standing error is too big.
    Time-based (t = sim time): the filter window and the acceleration ramp use the real tick spacing. With a live view the
    loop ran at ~8 Hz instead of 20 Hz, and a filter counted in ticks became 1.6 s long - the robot overshot +-20 cm back
    and forth (68, 2026-10-08). Without t it assumes ticks at `hz` (unit tests)."""

    MAX_DT = 0.2  # s, a longer gap (e.g. the loop stalled) counts as this

    def __init__(self, table, params=DOCK, hz=CONTROL_HZ, dist=DOCK_DIST):
        """dist: from the table centre - DOCK_DIST (the grasp pose) or CHECKPOINT_DIST (aligning at the checkpoint)."""
        self.table, self.p, self.nominal_dt = table, params, 1.0 / hz
        self.goal = along(table, dist - params.aim_deeper)
        self.poses = collections.deque()  # (t, pose) of the last filter_s seconds
        self.t = None
        self.cmd = np.zeros(3)
        self.inside_s, self.done, self.attempts = 0.0, False, 1

    def filtered(self):
        """Mean pelvis pose over the filter window (yaw averaged as a unit vector)."""
        a = np.array([(p.x, p.y, math.cos(p.yaw), math.sin(p.yaw)) for _, p in self.poses]).mean(axis=0)
        return Pose2D(float(a[0]), float(a[1]), math.atan2(a[3], a[2]))

    def errors(self, pose):
        """Goal relative to the robot, robot frame: (forward m, left m, turn rad)."""
        ex, ey = to_body(self.goal.x - pose.x, self.goal.y - pose.y, pose.yaw)
        return ex, ey, wrap(self.goal.yaw - pose.yaw)

    def step(self, pose, t=None):
        p = self.p
        t = (self.nominal_dt if self.t is None else self.t + self.nominal_dt) if t is None else float(t)
        dt = self.nominal_dt if self.t is None else float(np.clip(t - self.t, 1e-3, self.MAX_DT))
        self.t = t
        self.poses.append((t, pose))
        while self.poses[0][0] < t - p.filter_s:
            self.poses.popleft()
        ex, ey, eyaw = self.errors(self.filtered())
        if self.done:
            want = np.zeros(3)
        elif abs(ex) < p.pos_tol and abs(ey) < p.pos_tol and abs(eyaw) < p.yaw_tol:
            self.inside_s += dt
            want = np.zeros(3) if self.inside_s >= p.hold_s else self.cmd.copy()
            self.done = self.inside_s >= p.hold_s
        else:
            self.inside_s = 0.0
            want = np.array([_with_dead_band(p.k_lin * ex, p.vx_dead_band, abs(ex) > p.pos_tol / 2, -p.max_vx / 2, p.max_vx),
                             _with_dead_band(p.k_lin * ey, p.vy_dead_band, abs(ey) > p.pos_tol / 2, -p.max_vy, p.max_vy),
                             _with_dead_band(p.k_yaw * eyaw, p.omega_dead_band, abs(eyaw) > p.yaw_tol / 2,
                                             -p.max_omega, p.max_omega)])
        step = p.max_accel * dt
        self.cmd = want if not want.any() else self.cmd + np.clip(want - self.cmd, -step, step)
        return tuple(float(v) for v in self.cmd)

    def needs_redock(self, error):
        """error: table_frame_error after standing. True if it is worth another attempt (and attempts are left)."""
        dx, dy, dyaw = error
        off = abs(dx) > self.p.redock_xy or abs(dy) > self.p.redock_xy or abs(dyaw) > self.p.redock_yaw
        return off and self.attempts < self.p.max_attempts

    def redock(self):
        self.attempts += 1
        self.done, self.inside_s = False, 0.0
        self.poses.clear()
        self.t = None
        self.cmd = np.zeros(3)


@dataclasses.dataclass(frozen=True)
class StraightParams:
    """Straight into the table from the aligned checkpoint: forward steps only (vy = 0), heading steers back onto the line."""
    v_max: float = 0.18             # m/s commanded (GR00T walks ~0.045 slower)
    k_lin: float = 0.8              # 1/s on the distance still to go
    vx_dead_band: float = 0.045
    v_back: float = 0.08            # m/s, at most this fast backwards (only if it overshot)
    k_line: float = 1.5             # rad per m sideways off the line -> heading towards the line ...
    line_heading_max: float = math.radians(6.0)  # ... at most this much, fading out over the last FADE m
    fade: float = 0.30
    k_yaw: float = 1.2
    max_omega: float = 0.20
    omega_dead_band: float = 0.03
    max_accel: float = 0.3
    aim_deeper: float = 0.02        # m, the robot settles back ~2 cm after stopping
    filter_s: float = 0.6
    pos_tol: float = 0.03           # m to go (filtered) to stop
    hold_s: float = 0.3
    redo_along: float = 0.04        # m: after standing, go again (forwards or backwards) if further off than this
    max_attempts: int = 3


STRAIGHT = StraightParams()


class StraightIn:
    """Final approach from the checkpoint: walk straight into the table, never sideways. step(pose, t) -> (vx, 0, omega).
    done once it stopped within pos_tol of the grasp pose along the table axis. Sideways errors are not stepped out: they are
    steered out on the way (heading towards the line, straightening over the last `fade` m) - that is why the robot aligns at
    the checkpoint first."""

    def __init__(self, table, params=STRAIGHT, hz=CONTROL_HZ):
        self.table, self.p, self.nominal_dt = table, params, 1.0 / hz
        self.goal = along(table, DOCK_DIST - params.aim_deeper)
        self.poses = collections.deque()
        self.t = None
        self.cmd = np.zeros(3)
        self.inside_s, self.done, self.attempts = 0.0, False, 1

    filtered = Docker.filtered

    def line_errors(self, pose):
        """(m still to go along the table axis, m left of the line, heading rad against the table axis)."""
        c, s = math.cos(self.table.yaw), math.sin(self.table.yaw)
        ex, ey = pose.x - self.goal.x, pose.y - self.goal.y
        return -(c * ex + s * ey), -s * ex + c * ey, wrap(pose.yaw - self.table.yaw)

    def step(self, pose, t=None):
        p = self.p
        t = (self.nominal_dt if self.t is None else self.t + self.nominal_dt) if t is None else float(t)
        dt = self.nominal_dt if self.t is None else float(np.clip(t - self.t, 1e-3, Docker.MAX_DT))
        self.t = t
        self.poses.append((t, pose))
        while self.poses[0][0] < t - p.filter_s:
            self.poses.popleft()
        to_go, left, heading = self.line_errors(self.filtered())
        if self.done:
            want = np.zeros(3)
        elif abs(to_go) < p.pos_tol:
            self.inside_s += dt
            want = np.zeros(3) if self.inside_s >= p.hold_s else self.cmd.copy()
            self.done = self.inside_s >= p.hold_s
        else:
            self.inside_s = 0.0
            vx = _with_dead_band(p.k_lin * to_go, p.vx_dead_band, True, -p.v_back, p.v_max)
            fade = min(1.0, max(to_go, 0.0) / p.fade)  # aim along the line itself at the end: arrive straight
            towards_line = -math.copysign(1.0, vx) * float(np.clip(p.k_line * left, -p.line_heading_max, p.line_heading_max))
            omega = _with_dead_band(p.k_yaw * wrap(towards_line * fade - heading), p.omega_dead_band,
                                    abs(wrap(towards_line * fade - heading)) > math.radians(1.0), -p.max_omega, p.max_omega)
            want = np.array([vx, 0.0, omega])
        step = p.max_accel * dt
        self.cmd = want if not want.any() else self.cmd + np.clip(want - self.cmd, -step, step)
        return tuple(float(v) for v in self.cmd)

    def needs_redo(self, error):
        """error: table_frame_error after standing. True if the distance along the axis is off and attempts are left
        (sideways and heading are not corrected here: that would need side steps right at the table)."""
        return abs(error[0]) > self.p.redo_along and self.attempts < self.p.max_attempts

    def redo(self):
        self.attempts += 1
        self.done, self.inside_s = False, 0.0
        self.poses.clear()
        self.t = None
        self.cmd = np.zeros(3)


def _with_dead_band(v, dead_band, moving, low, high):
    """P command plus the dead band in its direction (only while still clearly off), clipped."""
    if moving and v != 0.0:
        v += math.copysign(dead_band, v)
    return float(np.clip(v, low, high))


@dataclasses.dataclass(frozen=True)
class Placed:
    """A box on the floor: name, centre (x, y), yaw (rad)."""
    name: str
    x: float
    y: float
    yaw: float


@dataclasses.dataclass(frozen=True)
class Setup:
    """One trial's world: scenario name, table pose (x, y = centre, yaw = direction the robot looks at the dock), obstacles."""
    scenario: str
    table: Pose2D
    obstacles: tuple = ()

    def as_dict(self):
        return {"scenario": self.scenario, "table": [round(self.table.x, 4), round(self.table.y, 4),
                                                     round(math.degrees(self.table.yaw), 2)],
                "obstacles": [{"name": o.name, "xy": [round(o.x, 4), round(o.y, 4)], "yaw_deg": round(math.degrees(o.yaw), 2)}
                              for o in self.obstacles]}


# --- geometry -------------------------------------------------------------------------------------------------------------
def along(table, dist):
    """Pose `dist` m in front of the table centre on its work side, looking at the table."""
    return Pose2D(table.x - dist * math.cos(table.yaw), table.y - dist * math.sin(table.yaw), table.yaw)


def dock_pose(table):
    return along(table, DOCK_DIST)


def checkpoint_pose(table):
    """Where the robot aligns and raises its arms: on the line into the table, CHECKPOINT_BACK before the grasp pose."""
    return along(table, CHECKPOINT_DIST)


def lead_in_pose(table):
    """Where the planned path joins the line into the table (LEAD_IN before the checkpoint)."""
    return along(table, LEAD_IN_DIST)


def table_frame_error(pose, table):
    """Pelvis pose against the dock pose, in the table frame: (dx towards the table, dy to the left, dyaw), m and rad.
    Same meaning as the stand offset (dx_m, dy_m, yaw_deg) of the volleyball grasp scene."""
    dock = dock_pose(table)
    c, s = math.cos(table.yaw), math.sin(table.yaw)
    ex, ey = pose.x - dock.x, pose.y - dock.y
    return c * ex + s * ey, -s * ex + c * ey, wrap(pose.yaw - table.yaw)


def checkpoint_error(pose, table):
    """Pelvis pose against the checkpoint, in the table frame: (dx towards the table, dy to the left, dyaw)."""
    dx, dy, dyaw = table_frame_error(pose, table)
    return dx + CHECKPOINT_BACK, dy, dyaw


def accepted(error):
    dx, dy, dyaw = error
    return abs(dx) <= ACCEPT_XY and abs(dy) <= ACCEPT_XY and abs(dyaw) <= ACCEPT_YAW


def corners(x, y, yaw, half):
    """Four corners (x, y) of a rectangle."""
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * u - s * v, y + s * u + c * v) for u, v in ((half[0], half[1]), (-half[0], half[1]),
                                                               (-half[0], -half[1]), (half[0], -half[1]))]


def rect_distance(px, py, x, y, yaw, half):
    """Distance from point (px, py) to a rectangle (0 inside)."""
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = px - x, py - y
    u, v = abs(c * dx + s * dy) - half[0], abs(-s * dx + c * dy) - half[1]
    return math.hypot(max(u, 0.0), max(v, 0.0))


def rect_gap(a, b):
    """Smallest distance between two rectangles a, b = (x, y, yaw, half) (0 if they overlap). Exact for convex polygons
    that do not overlap: the closest pair always involves a corner of one of them."""
    ca, cb = corners(*a), corners(*b)
    if any(rect_distance(px, py, *b) == 0.0 for px, py in ca) or any(rect_distance(px, py, *a) == 0.0 for px, py in cb):
        return 0.0
    edge = lambda p, q, r: _segment_distance(r, p, q)  # noqa: E731
    return min(min(edge(ca[i], ca[(i + 1) % 4], p) for i in range(4) for p in cb),
               min(edge(cb[i], cb[(i + 1) % 4], p) for i in range(4) for p in ca))


def _segment_distance(p, a, b):
    p, a, b = (np.asarray(v, float) for v in (p, a, b))
    t = np.clip(np.dot(p - a, b - a) / max(np.dot(b - a, b - a), 1e-12), 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * (b - a))))


def table_rect(table):
    return table.x, table.y, table.yaw, TABLE_HALF


def obstacle_rect(o, margin=0.0):
    (hx, hy), *_ = OBSTACLES[o.name]
    return o.x, o.y, o.yaw, (hx + margin, hy + margin)


# --- scenarios ------------------------------------------------------------------------------------------------------------
def _relative(robot, bearing, dist):
    """World point at `bearing` (rad, robot frame, 0 = straight ahead) and distance dist from the robot."""
    a = robot.yaw + bearing
    return robot.x + dist * math.cos(a), robot.y + dist * math.sin(a)


def _table(robot, rng, bearing, dist, facing_deg, turn=0.0):
    """Table centre at bearing/dist from the robot. Its work side faces the robot +- facing_deg, turned by `turn` (rad)
    on top: 0 = dock on the near side, pi = dock on the far side (the robot is behind the table)."""
    x, y = _relative(robot, bearing, dist)
    to_table = math.atan2(y - robot.y, x - robot.x)
    return Pose2D(x, y, wrap(to_table + turn + math.radians(rng.uniform(-facing_deg, facing_deg))))


def _deg(rng, low, high):
    return math.radians(rng.uniform(low, high))


def _facing(rng, robot):
    return _table(robot, rng, _deg(rng, -25, 25), rng.uniform(2.4, 3.8), 30), 0


def _behind_table(rng, robot):
    """Robot stands behind the table, looking across it: it has to walk all the way round to the work side."""
    return _table(robot, rng, _deg(rng, -25, 25), rng.uniform(1.4, 2.4), 30, turn=math.pi), 0


def _turned_away(rng, robot):
    """Table behind the robot, work side towards it: turn round first, then walk up."""
    return _table(robot, rng, math.pi + _deg(rng, -30, 30), rng.uniform(2.4, 3.8), 30), 0


def _beside(rng, robot):
    side = 1.0 if rng.random() < 0.5 else -1.0
    return _table(robot, rng, side * _deg(rng, 60, 120), rng.uniform(2.2, 3.6), 180), 0


def _obstacle_course(rng, robot):
    """Table ahead and further away, 1-3 obstacles in between."""
    return _table(robot, rng, _deg(rng, -20, 20), rng.uniform(3.6, 4.6), 30), int(rng.integers(1, 4))


def _anywhere(rng, robot):
    return _table(robot, rng, _deg(rng, -180, 180), rng.uniform(2.2, 4.6), 180), int(rng.integers(0, 3))


SCENARIOS = {  # name -> (sampler, description)
    "facing": (_facing, "table ahead, work side towards the robot"),
    "behind_table": (_behind_table, "robot behind the table: walk all the way round to the work side"),
    "turned_away": (_turned_away, "table behind the robot: turn round, then walk up"),
    "beside": (_beside, "table to the left or right, any orientation"),
    "obstacle_course": (_obstacle_course, "table 3.6-4.6 m ahead, 1-3 obstacles in between"),
    "anywhere": (_anywhere, "any direction, distance and orientation, 0-2 obstacles"),
}


def sample(scenario, rng, robot):
    """Draw a valid Setup for `scenario` relative to the robot's pose: robot clear of everything, obstacles apart from the
    table, each other and the straight corridor lead-in -> grasp pose, and a path to the checkpoint exists."""
    sampler, _ = SCENARIOS[scenario]
    for _ in range(MAX_DRAWS):
        table, n_obstacles = sampler(rng, robot)
        if rect_distance(robot.x, robot.y, *table_rect(table)) < ROBOT_CLEAR:
            continue
        obstacles = _sample_obstacles(rng, robot, table, n_obstacles, between=scenario == "obstacle_course")
        if obstacles is None:
            continue
        setup = Setup(scenario, table, tuple(obstacles))
        if plan(robot, setup) is not None:
            return setup
    raise RuntimeError(f"no valid setup for scenario '{scenario}' in {MAX_DRAWS} draws")


def _sample_obstacles(rng, robot, table, n, between):
    """n obstacles (distinct kinds), each clear of robot, table, dock and each other; None if they do not fit."""
    names = [list(OBSTACLES)[i] for i in rng.choice(len(OBSTACLES), n, replace=False)] if n else []
    placed = []
    goal = lead_in_pose(table)
    corridor = [along(table, d) for d in np.arange(DOCK_DIST, LEAD_IN_DIST + 1e-9, 0.1)]
    for name in names:
        for _ in range(100):
            if between:  # on the line robot -> lead-in point, 30-70 % of the way, up to 0.6 m to the side
                t, side = rng.uniform(0.3, 0.7), rng.uniform(-0.6, 0.6)
                heading = math.atan2(goal.y - robot.y, goal.x - robot.x)
                x = robot.x + t * (goal.x - robot.x) - side * math.sin(heading)
                y = robot.y + t * (goal.y - robot.y) + side * math.cos(heading)
            else:
                a, d = rng.uniform(-math.pi, math.pi), rng.uniform(1.0, 3.5)
                x, y = robot.x + d * math.cos(a), robot.y + d * math.sin(a)
            o = Placed(name, float(x), float(y), float(rng.uniform(-math.pi, math.pi)))
            rect = obstacle_rect(o)
            dock, check = dock_pose(table), checkpoint_pose(table)
            if (rect_distance(robot.x, robot.y, *rect) >= ROBOT_CLEAR
                    and rect_gap(rect, table_rect(table)) >= OBSTACLE_GAP
                    and all(rect_gap(rect, obstacle_rect(p)) >= OBSTACLE_GAP for p in placed)
                    and rect_distance(dock.x, dock.y, *rect) >= DOCK_CLEAR
                    and rect_distance(check.x, check.y, *rect) >= DOCK_CLEAR
                    and min(rect_distance(c.x, c.y, *rect) for c in corridor) >= CORRIDOR_CLEAR):
                placed.append(o)
                break
        else:
            return None
    return placed


# --- map and path ---------------------------------------------------------------------------------------------------------
def grid_for(robot, setup, resolution=occupancy.RESOLUTION):
    """Occupancy grid around robot, table and obstacles: the table as its footprint, tall obstacles HAND_MARGIN larger."""
    pts = [(robot.x, robot.y), *corners(*table_rect(setup.table))]
    for o in setup.obstacles:
        pts += corners(*obstacle_rect(o))
    pts = np.array(pts)
    lo, hi = pts.min(axis=0) - MAP_MARGIN, pts.max(axis=0) + MAP_MARGIN
    grid = occupancy.Grid(((lo[0], hi[0]), (lo[1], hi[1])), resolution)
    grid.add_box((setup.table.x, setup.table.y), TABLE_HALF, setup.table.yaw)
    for o in setup.obstacles:
        x, y, yaw, half = obstacle_rect(o, HAND_MARGIN if OBSTACLES[o.name][1] > HAND_HEIGHT else 0.0)
        grid.add_box((x, y), half, yaw)
    return grid


def plan(robot, setup):
    """A* path (world points) from the robot to the lead-in point, then straight to the checkpoint; None if there is none."""
    planned = _astar(robot, setup)
    return None if planned is None else planned[0] + planned[3]


def plan_smooth(robot, setup):
    """As plan(), with the A* part pulled taut and its corners rounded, every 5 cm (world points); None if there is no path."""
    planned = _astar(robot, setup)
    if planned is None:
        return None
    path, cost, grid, line = planned
    return occupancy.smooth_path(path, cost, grid, tail=line[:1] + line[-1:])  # round the join onto the line too


def _astar(robot, setup):
    """(A* path to the lead-in point, cost map, grid, straight line lead-in -> checkpoint). If the
    robot already stands between the lead-in point and the checkpoint, the path goes straight to the checkpoint."""
    grid = grid_for(robot, setup)
    cost = grid.costmap()
    lead, check = lead_in_pose(setup.table), checkpoint_pose(setup.table)
    c, s = math.cos(setup.table.yaw), math.sin(setup.table.yaw)
    forward = c * (robot.x - lead.x) + s * (robot.y - lead.y)  # along the line, from the lead-in point towards the table
    sideways = -s * (robot.x - lead.x) + c * (robot.y - lead.y)
    beyond_lead_in = 0.0 < forward < LEAD_IN and abs(sideways) < 0.3  # already on the line, between lead-in and checkpoint
    goal = check if beyond_lead_in else lead
    cells = occupancy.astar(cost, grid.to_cell(robot.x, robot.y), grid.to_cell(goal.x, goal.y))
    if cells is None:
        return None
    line = [] if beyond_lead_in else occupancy.resample([(lead.x, lead.y), (check.x, check.y)])  # from the lead-in point on
    return [grid.to_world(*c) for c in cells], cost, grid, line
