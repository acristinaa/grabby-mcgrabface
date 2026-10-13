"""Level 1 - two-hand volleyball pick-and-place without AI: the sim teacher that generates demos for a VLA.

Both palms cradle the ball from its sides, 20 degrees below the equator with the palm normal through the ball centre,
and all five fingers of each hand curl gently around it. The waist carries the ball: the robot grasps with the waist
turned +35 degrees (ball on its stand 0.42 m from the waist axis), lifts it 9 cm, turns the waist to -35 degrees with
the arms held still and lets go over the basket. Recipe and numbers from the feasibility probe of 2026-10-07
("variant B", 120/120 drops in sim).
  face_ball (waist +35) -> reach (one straight move to beside the ball: palms 4.5 cm out and 2 cm up, already turned
  to the ball, hands opening; measure arm sag) -> touch (palms on the ball, measure sag) -> squeeze (palms 2.5 cm in, then
  fingers curl) -> lift (9 cm, sag fix) -> turn (waist to -35) -> release (fingers open, palms 7 cm out and 2 cm up)
  -> retract (arms back to the start pose) -> face_front (waist to 0) -> settle (wait 3 s, then the ball must lie still in
  the basket for 3 s)
Start and end pose = the scene's start pose (env.ready_pose, action_space.BALL_READY_POSE): elbows low, palms facing each
other, fingers relaxed; the hands open while they approach. Ending where it started looks natural and lets tasks chain.
Every move speeds up and slows down smoothly (cosine profile). The reach is a straight line like a human arm's (user,
2026-10-08); the earlier path "8 cm above the ball, then down" made the hands hang over the ball with bent wrists.
The start pose is high enough to turn the waist without touching the ball: a lower one swept the left fingertips through
it (13 of 50 runs knocked the ball off its stand).
The arm sag under gravity is measured and removed from the following targets (as in g1sim/grasp.py). After the lift
the hands are corrected in the frame of the turned torso: forward and up per hand, sideways only by the mean error of
both hands, because the left-right difference is the squeeze itself and must stay. (Split in world axes as in the probe,
the turned squeeze axis leaked into the per-hand part: the left palm lost the ball and only its thumb touched it.)
The ball position comes from the sim (no perception): this is a teacher, not a policy.
Randomized starts (TableEnv.reset(randomize=True)): the robot's stand may be shifted and turned and the ball moved. The
teacher reads the pelvis pose and turns the waist by as much more or less as the ball (and later the basket) moved around
the waist axis; on the nominal start that is exactly +35 / -35 degrees as before. Hand orientations are built in world
coordinates from the torso yaw = pelvis yaw + waist yaw.
"""
import mink
import mujoco
import numpy as np

from g1sim import action_space, table_env, table_scene
from g1sim.grasp import interpolate, shifted
from g1sim.grasp_ik import ArmIK, hand_frame, pose

SIDES = ("left", "right")
ARM = {s: action_space.SLICES[f"{s}_arm"] for s in SIDES}
HAND = {s: action_space.SLICES[f"{s}_hand"] for s in SIDES}
WAIST = action_space.SLICES["waist"]
BALL = "volleyball"
RADIUS = table_scene.VOLLEYBALL_OBJECTS[BALL][1][0]

# Waist yaw while grasping and while placing (deg). At 30 degrees the right wrist touched the basket in 36 of 60 probe
# runs.
GRASP_WAIST_DEG = 35.0
PLACE_WAIST_DEG = -35.0
# Waist limit (tried 2026-10-09 for the real robot: "turn the waist less"): the waist turns at most this far and the arms carry
# the ball the rest of the arc (IK). Measured on the nominal layout, straight and in GR00T's crouch: every limit below 35 deg
# FAILS (the arms cannot reach both sides of a ball 35 deg off the torso axis: hands 10-54 cm short, they hit the stand), and
# the centre of mass of robot + ball moves MORE (2.8-3.8 cm) than with the full waist turn (2.3 / 3.0 cm) - turning the torso
# keeps arms and ball close to the body. So the default is no limit (None, the original teacher); a limit only makes sense with
# ball and basket closer together (layout) or with the legs turning the body.
MAX_WAIST_DEG = None
# Hand pose on the ball. The open Revo2 hand is a shallow dish: a 21-24 cm ball first touches it with its centre at
# (PALM_X + r, 0, PALM_Z) in hand-base coordinates (palm normal +x, fingers +z), measured 8-10 mm on both hands.
PALM_X = 0.009
PALM_Z = 0.09
FINGER_TILT_DEG = -15.0  # fingers point forward and 15 degrees down, thumb side up
# Palms 20 degrees below the equator, palm normal through the ball centre: held 36/36 in the probe. Palms at the equator
# held 8/36 - the curved palm acts as a wedge and squeezes the ball out downwards.
CRADLE_DEG = 20.0
APPROACH_OUT = 0.045    # m, palms this far outside the ball surface before the touch (3 cm: pinky hit the stand)
REACH_ABOVE = 0.02      # m, the reach ends this far above that: the arm sags before it is measured (sag fix after the reach)
# Robust to where the walk stops (2026-10-09, GR00T's crouch: pelvis 4.3 cm lower than the grasp scene, leaning back 2.5 deg;
# offline sweep scripts/70_handover_sweep.py): only 13/25 docks within +-3 cm passed. Too close, the hands turning with the
# torso in face_ball swept through the ball (fingertips 0.98 m, ball top 1.015 m); too far, the stretched arm sagged 7 cm under
# its own weight at the end of the reach (hand 0.83 m for a 0.90 m target) and the pinky hit the table (0.74 m). Hence:
FACE_LIFT = 0.06        # m, the hands rise this much while the waist turns to the ball (IK, turning with the torso)
# Gravity offset: each arm target is raised by the joint controller's static sag at that pose (gravity torque / kp), so the
# arm holds where the IK put it however far it reaches; the measured sag correction below only removes what is left (the
# ball's weight, contacts). The same feedforward is what the real arm needs with Unitree's PD gains.
GRAVITY_OFFSET = True
SQUEEZE = 0.025         # m, palms this far inside the first-touch pose; 1.5-4 cm all held in the probe
LIFT = 0.09             # m; the ball bottom is then ~5 cm above the basket wall
RELEASE_OUT = 0.07      # m, palms move out along their normals ...
RELEASE_UP = 0.02       # ... and up
SAG_FIX_ROUNDS = 3

# Revo2 targets while holding, order action_space.HAND_MOTORS (Thumb rotation, ThumbAux, Index, Middle, Ring, Pinky),
# 0 = open, 1 = closed. All five fingers curl around the ball after the palms have squeezed it. Measured 2026-10-07 with
# the palms in the squeeze pose: the open hand is a dish that already follows the ball, so index, middle, ring and
# pinky touch it at 0.03-0.06 (each closing alone). Beyond that the four fingers push the ball back towards the wrists,
# ~1 cm per 0.1 (fingers 0.6: 6-7 cm, the left palm lost the ball). The thumb (rotation motor 0.5 Nm) rests on top of
# the ball without pushing it. Over 20 starts with +-2 cm offset: 20/20 placed, the ball moved <= 1.5 mm in the hands
# during lift and turn (thumb 0/0 with fingers 0.12: up to 9.8 mm).
FINGERS_CURLED = np.array([0.3, 0.3, 0.12, 0.12, 0.12, 0.12])

FACE_S, REACH_S, TOUCH_S, SQUEEZE_S, SQUEEZE_HOLD_S = 2.0, 2.5, 1.5, 1.0, 0.5
LIFT_S, SAG_FIX_S, TURN_S, TURN_HOLD_S = 2.5, 0.3, 4.0, 0.5
OPEN_S, RELEASE_S, RETRACT_S, SETTLE_S, CHECK_S = 0.3, 0.8, 1.2, 3.0, 3.0
SETTLE_ROUNDS, SETTLE_ROUND_S = 3, 0.5

# Success check after the release (placement_checks), every tick over CHECK_S
MIN_LIFT = 0.06          # m, ball centre above its rest point after the lift and at the end of the turn
IN_BASKET_MARGIN = 0.005  # m, the ball centre may be this far beyond the point where the ball touches an inner wall
FLOOR_TOLERANCE = 0.005  # m, ball bottom from the basket floor
MAX_SPEED = 0.01         # m/s
MAX_SPIN = 0.1           # rad/s (a ball rolling at 1 cm/s spins 0.095 rad/s)
# contact with the ball is allowed; a demo that bumps a distractor is a bad demo
FORBIDDEN = ("table", "volleyball_stand", "basket", *table_scene.VOLLEYBALL_DISTRACTORS)


def ticks(seconds):
    return int(round(seconds * table_env.CONTROL_HZ))


def smooth(t):
    """Cosine profile 0..1 for t in 0..1: starts and ends at zero speed."""
    return 0.5 - 0.5 * np.cos(np.pi * t)


def hand_rotation(side, waist_yaw):
    """Hand-base rotation (columns = hand axes in world) for the cradle grasp with the torso turned to waist_yaw (rad, world:
    pelvis yaw + waist joint).

    Palm normal (x) towards the ball centre, fingers (z) forward and FINGER_TILT_DEG down, thumb side up (y for the
    right hand, -y for the left: the hands are mirror images); then tilted CRADLE_DEG about the forward axis so the palm
    sits below the equator, then turned with the waist.
    """
    s = 1.0 if side == "right" else -1.0
    a = np.radians(FINGER_TILT_DEG)
    x = np.array([0.0, s, 0.0])
    z = np.array([np.cos(a), 0.0, np.sin(a)])
    level = np.column_stack([x, np.cross(z, x), z])
    cradle = mink.SO3.from_x_radians(s * np.radians(CRADLE_DEG)).as_matrix()
    return mink.SO3.from_z_radians(waist_yaw).as_matrix() @ cradle @ level


def rotated_about(target, axis_xy, angle, spin=None):
    """target (mink.SE3) moved by angle (rad) round the vertical line through axis_xy (world) and turned by spin (rad, default
    angle) about the vertical: angle = spin is what a waist turn would do to a hand held still relative to the torso."""
    turn = mink.SO3.from_z_radians(angle).as_matrix()
    spin = turn if spin is None else mink.SO3.from_z_radians(spin).as_matrix()
    a = np.array([axis_xy[0], axis_xy[1], 0.0])
    return pose(spin @ target.rotation().as_matrix(), a + turn @ (target.translation() - a))


def palm_pose(side, centre, waist_yaw, squeeze):
    """Hand-base pose (mink.SE3) with the palm contact point `squeeze` m inside the surface of a ball centred at centre
    (negative = outside)."""
    rotation = hand_rotation(side, waist_yaw)
    return pose(rotation, np.asarray(centre, float) - rotation @ [PALM_X + RADIUS - squeeze, 0.0, PALM_Z])


def placement_checks(env):
    """Placement of the ball in the current sim state: {check: bool}, in cause -> effect order."""
    m, d = env.model, env.data
    ball = env.object_position(BALL)
    dof = m.jnt_dofadr[m.joint(f"{BALL}_joint").id]
    b = m.body("basket").id
    basket = d.xpos[b]
    free = table_scene.BASKET_INNER_HALF - RADIUS + IN_BASKET_MARGIN
    floor = basket[2] + table_scene.BASKET_WALL
    inside = d.xmat[b].reshape(3, 3).T @ (ball - basket)  # basket frame: the walk world turns the table with the basket on it
    return {
        "ball_in_basket": bool(np.all(np.abs(inside[:2]) <= free)),
        "ball_on_basket_floor": bool(abs(ball[2] - RADIUS - floor) <= FLOOR_TOLERANCE),
        "ball_at_rest": bool(np.linalg.norm(d.qvel[dof:dof + 3]) < MAX_SPEED
                             and np.linalg.norm(d.qvel[dof + 3:dof + 6]) < MAX_SPIN),
        "hands_off_ball": not any(other == BALL for _, other in env.robot_contacts()),
    }


def ball_placed(env):
    """True if the ball lies still in the basket, untouched by the robot. One sim state; run() checks CHECK_S worth."""
    return all(placement_checks(env).values())


def finger_of(body_name):
    """Robot body name -> "palm", "thumb", "index", ..., or None if it is not a hand body."""
    if not body_name.startswith(("lh_", "rh_")):
        return None
    for finger in ("thumb", "index", "middle", "ring", "pinky"):
        if f"_{finger}_" in body_name:
            return finger
    return "palm"


class TwoHandBallGrasp:
    """One pick-and-place on a freshly reset TableEnv(scene="volleyball").

    on_tick(phase) is called after every control tick, e.g. for video, lab view or camera images.
    """

    def __init__(self, env, on_tick=None, max_waist_deg=MAX_WAIST_DEG):
        """max_waist_deg: waist limit (MAX_WAIST_DEG, None = none); the arms reach the rest of the way."""
        self.env = env
        self.max_waist = np.inf if max_waist_deg is None else np.radians(max_waist_deg)
        m = env.model
        self.iks = {s: ArmIK(m, s, dt=1.0 / table_env.CONTROL_HZ) for s in SIDES}
        self.hand_body = {s: m.body(hand_frame(s)).id for s in SIDES}
        self.waist_qadr = env.body_qadr[table_env.WAIST_YAW]
        d = env.data
        pelvis = m.body("pelvis").id
        self.base_xy = d.xpos[pelvis][:2].copy()
        self.base_yaw = float(np.arctan2(d.xmat[pelvis][3], d.xmat[pelvis][0]))  # stand yaw of this variation
        waist_joint = int(np.flatnonzero(m.jnt_qposadr == self.waist_qadr)[0])
        self.waist_axis = self._in_base(d.xanchor[waist_joint][:2])  # waist axis, pelvis frame (x, y)
        self.on_tick = on_tick or (lambda phase: None)

    def run(self):
        """Whole pick-and-place. Returns the result dict (passed, checks, first_failed, measurements)."""
        env = self.env
        self.action = env.ready_pose.copy()
        self.contacts = {}  # phase -> {(robot body, scene part)}
        self._reset_ik()
        rest = env.object_position(BALL)
        basket_xy = env.data.xpos[env.model.body("basket").id][:2]
        # waist joint targets: the nominal angles plus how far ball and basket moved around the waist axis in this variation
        # (the static stand, not the ball: the ball settles a fraction of a mm off its spot, the nominal turn must stay exact)
        stand_xy = env.data.xpos[env.model.body("volleyball_stand").id][:2]
        grasp_yaw = np.radians(GRASP_WAIST_DEG) + self._bearing_change(stand_xy, table_scene.VOLLEYBALL_OBJECTS[BALL][4])
        place_yaw = np.radians(PLACE_WAIST_DEG) + self._bearing_change(basket_xy, table_scene.BASKET_CENTER)
        # the waist turns only as far as allowed; the arms reach across the rest (rotated hand targets, IK)
        grasp_waist, place_waist = (float(np.clip(a, -self.max_waist, self.max_waist)) for a in (grasp_yaw, place_yaw))
        # hand orientations in world: squeeze axis = the torso's sideways axis (a sphere can be squeezed along any horizontal
        # axis); facing the ball fully with the torso turned less twisted the wrists and the hand hit the stand
        torso_yaw = self.base_yaw + grasp_waist
        self.com = {"start": self._com_xy(), "max_shift_m": 0.0}

        waist_now = float(self.env.waist_yaw_target)
        self._carry("face_ball", waist_now, grasp_waist, waist_now, grasp_waist, FACE_S, lift=FACE_LIFT)
        self._reset_ik()
        pre = {s: palm_pose(s, rest, torso_yaw, -APPROACH_OUT) for s in SIDES}
        self._move("reach", {s: shifted(pre[s], [0.0, 0.0, REACH_ABOVE]) for s in SIDES}, REACH_S,
                   fingers=np.zeros(6))  # open hands: the grasp below is measured with the open hand as a dish
        self._settle("reach", SETTLE_ROUNDS)

        centre = env.object_position(BALL)  # look again: the true position (a camera estimate on the robot)
        moved_before_touch = float(np.linalg.norm(centre - rest))
        touch = {s: palm_pose(s, centre, torso_yaw, 0.0) for s in SIDES}
        self._move("touch", touch, TOUCH_S)
        self._settle("touch", 2)
        touch_error = {s: self._hand_error(s, touch[s]) for s in SIDES}

        squeeze = {s: palm_pose(s, centre, torso_yaw, SQUEEZE) for s in SIDES}
        self._move("squeeze", squeeze, SQUEEZE_S)
        self._move("squeeze", squeeze, SQUEEZE_HOLD_S, fingers=FINGERS_CURLED)
        posture = self._posture()
        touched_by = self._ball_touched_by()

        lift = {s: shifted(squeeze[s], [0.0, 0.0, LIFT]) for s in SIDES}
        self._move("lift", lift, LIFT_S)
        self._correct_sag("lift", lift)
        lift_height = env.lift_height(BALL)
        touched_by_lifted = self._ball_touched_by()

        if grasp_waist == grasp_yaw and place_waist == place_yaw:
            self._turn_waist("turn", place_yaw, TURN_S)  # original teacher: arm targets held, no IK while turning
        else:
            self._carry("turn", grasp_yaw, place_yaw, grasp_waist, place_waist, TURN_S)
        self._hold_action("turn", TURN_HOLD_S)
        carried = env.object_position(BALL)
        basket = env.data.xpos[env.model.body("basket").id].copy()

        self._reset_ik()  # IK from the commanded arm angles, waist now at PLACE_WAIST_DEG
        for s in SIDES:
            self.action[HAND[s]] = 0.0
        self._hold_action("release", OPEN_S)
        opened = {s: shifted(self.target[s], -RELEASE_OUT * self.target[s].rotation().as_matrix()[:, 0]
                             + [0.0, 0.0, RELEASE_UP]) for s in SIDES}
        self._move("release", opened, RELEASE_S)
        self._move_joints("retract", env.ready_pose, RETRACT_S)  # fingers back to relaxed as well
        self._turn_waist("face_front", float(env.ready_pose[WAIST][0]), FACE_S)
        self._hold_action("settle", SETTLE_S)
        window = []
        for _ in range(ticks(CHECK_S)):
            self._tick("settle")
            window.append(placement_checks(env))
        placed = {k: all(w[k] for w in window) for k in window[0]}

        all_contacts = set().union(*self.contacts.values())
        forbidden = {p: sorted(c for c in cs if c[1] in FORBIDDEN) for p, cs in self.contacts.items()}
        forbidden = {p: cs for p, cs in forbidden.items() if cs}
        end = env.object_position(BALL)
        checks = {
            "no_forbidden_contacts": not forbidden,
            "ball_lifted": lift_height >= MIN_LIFT,
            "ball_carried_to_basket": bool(carried[2] - rest[2] >= MIN_LIFT),
            **placed,
        }
        return {
            "passed": all(checks.values()),
            "checks": checks,
            "first_failed": next((k for k, ok in checks.items() if not ok), None),
            "ball_start": rest.round(4).tolist(),
            "ball_moved_before_touch_m": round(moved_before_touch, 4),
            "lift_height_m": round(float(lift_height), 4),
            "touch_error_m": {s: round(e, 4) for s, e in touch_error.items()},
            "ball_touched_by": {"squeeze": touched_by, "lifted": touched_by_lifted},
            "carried_ball_over_basket_m": (carried[:2] - basket[:2]).round(4).tolist(),
            "carried_ball_bottom_above_wall_m": round(float(carried[2] - RADIUS - basket[2] - table_scene.BASKET_WALL
                                                            - table_scene.BASKET_WALL_HEIGHT), 4),
            "ball_end": end.round(4).tolist(),
            "ball_to_basket_centre_m": round(float(np.linalg.norm(end[:2] - basket[:2])), 4),
            "ball_bottom_above_floor_m": round(float(end[2] - RADIUS - basket[2] - table_scene.BASKET_WALL), 4),
            "forbidden_contacts": forbidden,
            "contacts": sorted(all_contacts),
            "posture_at_squeeze": posture,
            "sim_time_s": round(float(env.data.time), 3),
            "waist_deg": {"grasp": round(float(np.degrees(grasp_waist)), 1), "place": round(float(np.degrees(place_waist)), 1),
                          "limit": None if np.isinf(self.max_waist) else round(float(np.degrees(self.max_waist)), 1)},
            "arm_reach_deg": {"grasp": round(float(np.degrees(grasp_yaw - grasp_waist)), 1),
                              "place": round(float(np.degrees(place_yaw - place_waist)), 1)},
            "com_max_shift_m": round(self.com["max_shift_m"], 4),
        }

    # --- building blocks ----------------------------------------------------------------------------------------------
    def _reset_ik(self):
        """Set both IKs to the commanded arm angles and waist yaw; targets = those hand poses, no sag correction."""
        q = self.env.data.qpos.copy()
        q[self.waist_qadr] = self.env.waist_yaw_target
        for s, ik in self.iks.items():
            q[ik.arm_qadr] = self.action[ARM[s]]
        for ik in self.iks.values():
            ik.reset(q)
        self.target = {s: ik.hand_pose() for s, ik in self.iks.items()}
        self.sag = {s: mink.SE3.identity() for s in SIDES}  # measured hand sag (hand frame), removed from the targets

    def _turn_waist(self, phase, goal, seconds):
        """Turn the waist to goal (rad) with a smooth (cosine) profile; the arm and hand targets stay as they are."""
        start, n = float(self.action[WAIST][0]), ticks(seconds)
        for i in range(1, n + 1):
            self.action[WAIST] = start + (goal - start) * smooth(i / n)
            self._tick(phase)

    def _carry(self, phase, yaw_from, yaw_to, waist_from, waist_to, seconds, lift=0.0):
        """Carry the held ball along its arc round the waist axis from yaw_from to yaw_to (rad, as a full waist turn would)
        while the waist itself only goes from waist_from to waist_to: the hand targets are the held poses turned about the
        waist axis, the IK follows the waist each tick. Same smooth (cosine) profile for both. lift: the hands rise this much
        (m) on the way."""
        axis = self._to_world(self.waist_axis)
        start = dict(self.target)  # after the sag fix: the commanded poses that hold the ball
        n = ticks(seconds)
        for i in range(1, n + 1):
            alpha = smooth(i / n)
            waist = waist_from + (waist_to - waist_from) * alpha
            self.action[WAIST] = waist
            self._set_ik_waist(waist)
            turn = (yaw_to - yaw_from) * alpha
            self._command({s: shifted(rotated_about(start[s], axis, turn, waist - waist_from), [0.0, 0.0, lift * alpha])
                           for s in SIDES})
            self._tick(phase)
        self.target = {s: shifted(rotated_about(start[s], axis, yaw_to - yaw_from, waist_to - waist_from), [0.0, 0.0, lift])
                       for s in SIDES}

    def _set_ik_waist(self, waist):
        """Tell both IKs the waist angle (they only move the arm joints)."""
        for ik in self.iks.values():
            q = ik.configuration.q.copy()
            q[self.waist_qadr] = waist
            ik.configuration.update(q)

    def _to_world(self, xy):
        """Pelvis frame (x, y) of this variation's stand -> world (x, y)."""
        c, s = np.cos(self.base_yaw), np.sin(self.base_yaw)
        return self.base_xy + np.array([c * xy[0] - s * xy[1], s * xy[0] + c * xy[1]])

    def _com_xy(self):
        """Horizontal centre of mass of robot + ball (ball only once it is off its stand, i.e. carried), world (x, y)."""
        m, d = self.env.model, self.env.data
        pelvis = m.body("pelvis").id
        mass, com = m.body_subtreemass[pelvis], d.subtree_com[pelvis][:2] * m.body_subtreemass[pelvis]
        if self.env.lift_height(BALL) > 0.02:
            ball_mass = m.body_mass[m.body(BALL).id]
            mass, com = mass + ball_mass, com + ball_mass * self.env.object_position(BALL)[:2]
        return com / mass

    def _move(self, phase, goals, seconds, fingers=None):
        """Both hands in `seconds` along straight lines to the poses goals, speeding up and slowing down smoothly (cosine
        profile); fingers (6-D, optional) change along the same profile."""
        start, n = dict(self.target), ticks(seconds)
        hand_start = {s: self.action[HAND[s]].copy() for s in SIDES}
        for i in range(1, n + 1):
            alpha = smooth(i / n)
            if fingers is not None:
                for s in SIDES:
                    self.action[HAND[s]] = hand_start[s] + (fingers - hand_start[s]) * alpha
            self._command({s: interpolate(start[s], goals[s], alpha) for s in SIDES})
            self._tick(phase)
        self.target = dict(goals)

    def _move_joints(self, phase, pose, seconds):
        """Arms and hands to the 27-D pose (waist excluded) in joint space with a smooth (cosine) profile, no IK: the
        relaxed way back to the start pose."""
        parts = np.r_[0:WAIST.start]
        start, n = self.action[parts].copy(), ticks(seconds)
        for i in range(1, n + 1):
            self.action[parts] = start + (pose[parts] - start) * smooth(i / n)
            self._tick(phase)

    def _hold(self, phase, seconds):
        for _ in range(ticks(seconds)):
            self._command(self.target)
            self._tick(phase)

    def _hold_action(self, phase, seconds):
        """Keep sending the current action without IK (the IK does not know the waist moved)."""
        for _ in range(ticks(seconds)):
            self._tick(phase)

    def _settle(self, phase, rounds):
        """Hold, measure how far each hand sags from its target and remove that from the targets, `rounds` times."""
        for _ in range(rounds):
            self._hold(phase, SETTLE_ROUND_S)
            for s in SIDES:
                self.sag[s] = self.sag[s] @ (self.target[s].inverse() @ self._measured_hand(s))

    def _correct_sag(self, phase, goals):
        """Shift the targets by the measured position error to goals, in the frame of the turned torso: forward (x) and
        up (z) per hand, sideways (y, the squeeze axis) by the mean of both hands."""
        torso = mink.SO3.from_z_radians(self.base_yaw + self.env.waist_yaw_target).as_matrix()
        for _ in range(SAG_FIX_ROUNDS):
            self._hold(phase, SAG_FIX_S)
            error = {s: torso.T @ (goals[s].translation() - self._measured_hand(s).translation()) for s in SIDES}
            y = 0.5 * (error["left"][1] + error["right"][1])
            for s in SIDES:
                self.target[s] = shifted(self.target[s], torso @ [error[s][0], y, error[s][2]])

    def _command(self, targets):
        """Arm angles for the hand poses targets into the action (IK goal = target without the measured sag), plus the
        gravity offset (GRAVITY_OFFSET)."""
        q = {s: ik.step(targets[s] @ self.sag[s].inverse()) for s, ik in self.iks.items()}
        offset = self._gravity_offset(q) if GRAVITY_OFFSET else {s: 0.0 for s in SIDES}
        for s in SIDES:
            self.action[ARM[s]] = q[s] + offset[s]

    def _gravity_offset(self, q):
        """{side: 7 angles (rad)} the arm joints sag under gravity at the arm angles q (PD at rest: kp * error = gravity
        torque), from the current state with the arms at q, at rest."""
        m = self.env.model
        if not hasattr(self, "_scratch"):
            self._scratch = mujoco.MjData(m)
            self._arm_dof = {s: m.jnt_dofadr[[m.joint(n).id for n in action_space.SIM_ARM_JOINTS[s]]] for s in SIDES}
            self._arm_kp = {s: np.asarray(self.env.kp)[list(action_space.ARM_MOTOR_INDEX[s])] for s in SIDES}
        d = self._scratch
        d.qpos[:] = self.env.data.qpos
        d.qvel[:] = 0.0
        for s, ik in self.iks.items():
            d.qpos[ik.arm_qadr] = q[s]
        mujoco.mj_forward(m, d)
        return {s: d.qfrc_bias[self._arm_dof[s]] / self._arm_kp[s] for s in SIDES}

    def _tick(self, phase):
        self.env.apply(self.action)
        self.contacts.setdefault(phase, set()).update(self.env.robot_contacts())
        if getattr(self, "com", None):
            shift = float(np.linalg.norm(self._com_xy() - self.com["start"]))
            self.com["max_shift_m"] = max(self.com["max_shift_m"], shift)
        self.on_tick(phase)

    def _in_base(self, xy):
        """World (x, y) -> pelvis frame (x, y) of this variation's stand."""
        c, s = np.cos(self.base_yaw), np.sin(self.base_yaw)
        dx, dy = np.asarray(xy, float) - self.base_xy
        return np.array([c * dx + s * dy, -s * dx + c * dy])

    def _bearing_change(self, xy, nominal_xy):
        """Angle (rad, + = left) by which xy lies further round the waist axis than nominal_xy does on the nominal stand.
        0 on the nominal start, so the teacher then turns exactly as before."""
        now, nominal = self._in_base(xy) - self.waist_axis, np.asarray(nominal_xy, float) - self.waist_axis
        return float(np.arctan2(now[1], now[0]) - np.arctan2(nominal[1], nominal[0]))

    def _measured_hand(self, side):
        d, b = self.env.data, self.hand_body[side]
        return pose(d.xmat[b].reshape(3, 3), d.xpos[b])

    def _hand_error(self, side, goal):
        """Distance (m) of the measured hand base from the pose goal."""
        return float(np.linalg.norm(self._measured_hand(side).translation() - goal.translation()))

    def _ball_touched_by(self):
        """{side: sorted hand parts (palm, thumb, index, ...) touching the ball now}."""
        parts = {s: set() for s in SIDES}
        for robot, other in self.env.robot_contacts():
            if other == BALL and finger_of(robot):
                parts["left" if robot.startswith("lh_") else "right"].add(finger_of(robot))
        return {s: sorted(p) for s, p in parts.items()}

    def _posture(self):
        """Arm posture now, per side: elbow height (m) against shoulder and wrist, how far the elbow sticks out."""
        m, d = self.env.model, self.env.data
        torso = d.xmat[m.body("torso_link").id].reshape(3, 3)
        out = {}
        for s in SIDES:
            shoulder, elbow, wrist = (d.xpos[m.body(f"{s}_{link}").id]
                                      for link in ("shoulder_pitch_link", "elbow_link", "wrist_roll_link"))
            sideways = 1.0 if s == "left" else -1.0  # torso +y is left
            out[s] = {
                "elbow_minus_shoulder_z_m": round(float(elbow[2] - shoulder[2]), 3),
                "elbow_minus_wrist_z_m": round(float(elbow[2] - wrist[2]), 3),
                # elbow further out from the body than the shoulder (+) or tucked in (-), torso frame
                "elbow_out_of_shoulder_m": round(float(sideways * (torso.T @ (elbow - shoulder))[1]), 3),
                "elbow_angle_deg": round(float(np.degrees(self.action[ARM[s]][3])), 1),
                "arm_deg": np.degrees(self.action[ARM[s]]).round(1).tolist(),
            }
        return out
