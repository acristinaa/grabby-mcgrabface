"""Continuous walk + pick-and-place in ONE sim: GR00T walks the robot to the table, then it is switched off and the two-hand
volleyball teacher (g1sim/ball_grasp.py) runs on the very same MuJoCo state (scripts/69_walk_pick_place.py).

World: approach_world.write(volleyball=True) - the walk world with stand and basket on the movable table and the ball as a
free body (place_ball puts it on its stand wherever the table stands).

Handover (user's ruling 2026-10-09: "once GR00T has got us there, disable it and start the pick-and-place code"):
  1. take the sim thread's step lock and keep it: the LeRobot sim thread blocks before its next step, so GR00T's leg and
     waist torques (rt/lowcmd) no longer reach the robot. GR00T itself keeps running but has no effect.
  2. weld the pelvis where it stands (table_scene's pelvis hold, a virtual harness) and hold all 29 joints and both hands
     at their current angles with TableEnv's PD (same gains): the legs stay in GR00T's last stance.
  3. step the physics here, TableEnv style: 1/300 s, 10 substeps per 1/30 s control tick; elliptic friction cone and
     impratio 10 as in the grasp scene (the walk runs with the sim's own settings).
  4. the teacher reads the pelvis pose and turns the waist after where ball and basket really are, so the docking error
     of the walk is simply a stand offset for it.
hand_back() undoes all of it and lets the sim thread (and GR00T) go on - the next walk starts from where the grasp ended.
This environment has TableEnv's interface as far as ball_grasp uses it (model, data, apply, body_qadr, ready_pose,
waist_yaw_target, object_position, lift_height, robot_contacts).
"""
import math
import threading
import time

import mujoco
import numpy as np

from g1sim import action_space, approach, approach_world, brainco, table_env, table_scene

BALL = "volleyball"
SUBSTEPS = table_env.SUBSTEPS
GRASP_TIMESTEP = 1.0 / (table_env.CONTROL_HZ * SUBSTEPS)


def stand_xy(table_pose):
    """World (x, y) of the ball's stand for a table at table_pose (approach.Table or anything with x, y, yaw)."""
    bx, by = table_scene.VOLLEYBALL_OBJECTS[BALL][4]
    lx, ly = bx - approach.TABLE_CENTRE_X, by
    c, s = math.cos(table_pose.yaw), math.sin(table_pose.yaw)
    return table_pose.x + c * lx - s * ly, table_pose.y + s * lx + c * ly


class VolleyballFurniture(approach_world.Furniture):
    """Furniture that also puts the ball on its stand with the table, and parks it with the table."""

    def __init__(self, model, data, lock=None):
        super().__init__(model, data, lock)
        joint = model.joint(f"{BALL}_joint").id
        self.ball_qadr, self.ball_vadr = model.jnt_qposadr[joint], model.jnt_dofadr[joint]
        self.ball_start = None

    def place(self, setup):
        super().place(setup)
        x, y = stand_xy(setup.table)
        self._put_ball([x, y, table_scene.object_rest_height(BALL) + 0.002])

    def park(self):
        super().park()
        self._put_ball([approach_world.PARK[0], approach_world.PARK[1] - 3.0, table_scene.VOLLEYBALL_OBJECTS[BALL][1][0] + 0.002])

    def _put_ball(self, xyz):
        with self.lock if self.lock is not None else approach_world._nolock():
            a, v = self.ball_qadr, self.ball_vadr
            self.data.qpos[a:a + 7] = [*xyz, 1.0, 0.0, 0.0, 0.0]
            self.data.qvel[v:v + 6] = 0.0
        self.ball_start = np.array(xyz, dtype=float)

    def ball_moved(self):
        """How far (m) the ball has moved since it was put down."""
        return float(np.linalg.norm(self.data.qpos[self.ball_qadr:self.ball_qadr + 3] - self.ball_start))


class HandoverEnv:
    """TableEnv's interface for ball_grasp on the running walk sim, valid between take_over() and hand_back()."""

    def __init__(self, walk_sim, speed=1.0, on_step=None):
        """walk_sim: g1sim.walk_sim.WalkSim on approach_world.write(volleyball=True). speed: wall-clock pacing while the
        grasp runs (1 = real time, so the live view shows it like the walk; 0 = as fast as possible). on_step(): called
        once per control tick, e.g. the live view."""
        s = walk_sim.sim
        self._setup(walk_sim.model, walk_sim.data, s._step_lock, s.body_qadr, s.body_vadr, s.body_act, s.hand, speed, on_step)

    @classmethod
    def offline(cls, model, data):
        """The same environment on a plain MjModel/MjData of the walk world (no LeRobot, no DDS): replaying handovers fast
        (OfflineHandover). Joint mapping as TableEnv: the 29 non-hand actuators in model order are the G1 motors."""
        env = cls.__new__(cls)
        acts = [a for a in range(model.nu) if not model.actuator(a).name.startswith(tuple(brainco.PREFIX.values()))]
        joints = [int(model.actuator_trnid[a, 0]) for a in acts]
        act_of_joint = {int(j): a for a, j in enumerate(model.actuator_trnid[:, 0])}
        hand = {}
        for side in brainco.SIDES:
            hj = [model.joint(n).id for n in brainco.joint_names(side)]
            hand[side] = {"act": np.array([act_of_joint[j] for j in hj]), "qadr": model.jnt_qposadr[hj],
                          "vadr": model.jnt_dofadr[hj]}
        env._setup(model, data, threading.RLock(), model.jnt_qposadr[joints], model.jnt_dofadr[joints], np.array(acts),
                   hand, 0.0, None)
        return env

    def _setup(self, model, data, lock, body_qadr, body_vadr, body_act, hand, speed, on_step):
        self.model, self.data, self.lock = model, data, lock
        m = self.model
        self.body_qadr, self.body_vadr, self.body_act = body_qadr, body_vadr, body_act
        self.hand = hand
        self.arm_slot = {s: np.array(action_space.ARM_MOTOR_INDEX[s]) for s in action_space.SIDES}
        self.limit = m.actuator_ctrlrange[:, 1].copy()
        self.ready_pose = action_space.BALL_READY_POSE
        self.scene = {"target": BALL}
        joint = m.joint(f"{BALL}_joint").id
        self.ball_qadr = m.jnt_qposadr[joint]
        self.scene_bodies = {m.body(approach_world.TABLE).id: "table", m.body("volleyball_stand").id: "volleyball_stand",
                             m.body("basket").id: "basket", m.body(BALL).id: BALL}
        self.hold_mocap = m.body_mocapid[m.body(table_scene.PELVIS_HOLD).id]
        self.hold_eq = m.equality(table_scene.PELVIS_HOLD).id
        self.pelvis_qadr = m.jnt_qposadr[m.body_jntadr[m.body("pelvis").id]]
        self.speed, self.on_step = speed, on_step or (lambda: None)
        self.active = False

    # --- handover -------------------------------------------------------------------------------------------------
    def take_over(self):
        """Stop GR00T's influence, weld the pelvis, hold every joint where it is. Returns the handover state (dict)."""
        m, d = self.model, self.data
        self.lock.acquire()  # the sim thread now waits before its next step
        self.active = True
        self.saved = (m.opt.timestep, m.opt.cone, m.opt.impratio)
        m.opt.timestep, m.opt.cone, m.opt.impratio = GRASP_TIMESTEP, mujoco.mjtCone.mjCONE_ELLIPTIC, 10.0
        a = self.pelvis_qadr
        d.mocap_pos[self.hold_mocap] = d.qpos[a:a + 3]
        d.mocap_quat[self.hold_mocap] = d.qpos[a + 3:a + 7]
        d.eq_active[self.hold_eq] = 1
        self.body_target = d.qpos[self.body_qadr].copy()
        self.hand_target = {s: brainco.to_norm(d.qpos[h["qadr"]]) for s, h in self.hand.items()}
        self.waist_yaw_target = float(self.body_target[table_env.WAIST_YAW])
        self.kp, self.kd = table_env.KP.copy(), table_env.KD.copy()
        self._wall0, self._sim0 = time.perf_counter(), d.time
        mujoco.mj_forward(m, d)
        roll, pitch, yaw = _euler(d.qpos[a + 3:a + 7])
        return {"pelvis_xyz": [round(float(v), 4) for v in d.qpos[a:a + 3]],
                "pelvis_roll_pitch_yaw_deg": [round(math.degrees(v), 2) for v in (roll, pitch, yaw)],
                "legs_rad": [round(float(v), 3) for v in self.body_target[:12]],
                "waist_rad": [round(float(v), 3) for v in self.body_target[12:15]]}

    def hand_back(self):
        """Weld off, the walk sim's physics settings back, the sim thread (and GR00T) runs again."""
        if not self.active:
            return
        m, d = self.model, self.data
        d.eq_active[self.hold_eq] = 0
        m.opt.timestep, m.opt.cone, m.opt.impratio = self.saved
        self.active = False
        self.lock.release()

    # --- TableEnv interface ---------------------------------------------------------------------------------------
    def apply(self, actions):
        """One 1/30 s control tick: 27-D absolute targets (arms rad, hands normalized, waist yaw rad); legs and waist
        roll/pitch stay at their handover angles."""
        actions = np.asarray(actions, dtype=float)
        for side, q in action_space.arm_targets(actions).items():
            self.body_target[self.arm_slot[side]] = q
        self.body_target[table_env.WAIST_YAW] = self.waist_yaw_target = float(actions[action_space.SLICES["waist"]][0])
        self.hand_target = action_space.hand_targets(actions)
        m, d = self.model, self.data
        for _ in range(SUBSTEPS):
            ctrl = np.zeros(m.nu)
            q, dq = d.qpos[self.body_qadr], d.qvel[self.body_vadr]
            ctrl[self.body_act] = self.kp * (self.body_target - q) - self.kd * dq
            for side, h in self.hand.items():
                q, dq = d.qpos[h["qadr"]], d.qvel[h["vadr"]]
                ctrl[h["act"]] = brainco.HAND_KP * (brainco.to_rad(self.hand_target[side]) - q) - brainco.HAND_KD * dq
            d.ctrl[:] = np.clip(ctrl, -self.limit, self.limit)
            mujoco.mj_step(m, d)
        self.on_step()
        if self.speed > 0:  # pace like the walk, so the live view shows the grasp in real time
            ahead = (d.time - self._sim0) / self.speed - (time.perf_counter() - self._wall0)
            if ahead > 0:
                time.sleep(ahead)

    def object_position(self, name):
        a = self.ball_qadr if name == BALL else None
        if a is None:
            raise KeyError(name)
        return self.data.qpos[a:a + 3].copy()

    def lift_height(self, name=None):
        return float(self.object_position(BALL)[2] - table_scene.object_rest_height(BALL))

    def robot_contacts(self):
        """Contacts between the robot and table, stand, basket or ball: sorted [(robot body, scene part)]."""
        m, d = self.model, self.data
        pairs = set()
        for i in range(d.ncon):
            b1, b2 = int(m.geom_bodyid[d.contact[i].geom1]), int(m.geom_bodyid[d.contact[i].geom2])
            for robot, other in ((b1, b2), (b2, b1)):
                if other in self.scene_bodies and robot != 0 and robot not in self.scene_bodies and m.body_mocapid[robot] < 0:
                    pairs.add((m.body(robot).name, self.scene_bodies[other]))
        return sorted(pairs)


def _euler(quat):
    w, x, y, z = quat
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


# Handover as GR00T leaves it (measured in the walk world, 69 seed 0/1, 2026-10-09: pelvis 0.750 m, leaning back ~2.5 deg,
# waist yaw ~3 deg, legs in its crouch); used to replay the grasp offline from any dock offset.
GROOT_HANDOVER = {
    "pelvis_z": 0.7497, "roll_deg": 0.1, "pitch_deg": -2.5,
    "legs": (-0.38, 0.003, 0.036, 0.698, -0.275, 0.017, -0.362, -0.006, -0.046, 0.714, -0.309, -0.024),
    "waist": (0.003, -0.008, 0.052),
}


def offline_handover(model, data, table, offset=(0.0, 0.0, 0.0), handover=GROOT_HANDOVER):
    """Put the walk world into the state of a handover without walking: table (approach.Pose2D-like: x, y, yaw) with stand,
    basket and ball, the robot docked at offset (dx towards the table, dy left, dyaw rad - as approach.table_frame_error)
    from the dock pose, in handover's posture with the arms in BALL_READY_POSE. Returns HandoverEnv.offline on it."""
    mujoco.mj_resetData(model, data)
    env = HandoverEnv.offline(model, data)
    furniture = VolleyballFurniture(model, data)
    furniture.place(approach.Setup("offline", table))
    dock = approach.dock_pose(table)
    c, s = math.cos(table.yaw), math.sin(table.yaw)
    dx, dy, dyaw = offset
    x, y, yaw = dock.x + c * dx - s * dy, dock.y + s * dx + c * dy, dock.yaw + dyaw
    a = env.pelvis_qadr
    data.qpos[a:a + 3] = [x, y, handover["pelvis_z"]]
    data.qpos[a + 3:a + 7] = _quat_rpy(math.radians(handover["roll_deg"]), math.radians(handover["pitch_deg"]), yaw)
    data.qpos[env.body_qadr[:12]] = handover["legs"]
    data.qpos[env.body_qadr[12:15]] = handover["waist"]
    for side, q in action_space.arm_targets(action_space.BALL_READY_POSE).items():
        data.qpos[env.body_qadr[env.arm_slot[side]]] = q
    for side, q in action_space.hand_targets(action_space.BALL_READY_POSE).items():
        data.qpos[env.hand[side]["qadr"]] = brainco.to_rad(q)
    mujoco.mj_forward(model, data)
    return env


def _quat_rpy(roll, pitch, yaw):
    """Quaternion (w, x, y, z) of yaw about z, then pitch about y, then roll about x (the inverse of _euler)."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2),
                              math.cos(yaw / 2), math.sin(yaw / 2))
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]
