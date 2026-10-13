"""Stufe 1.4 - Inverse Kinematik (IK) fuer einen Arm am Tisch, mit mink.

IK rechnet aus einer Soll-Pose der Handbasis die 7 Armwinkel. mink loest dafuer je Schritt ein kleines quadratisches
Programm (Loeser daqp): Hand zur Soll-Pose, Arm nah an seiner Startpose; Gelenkgrenzen, Abstand von Unterarm und Hand
zur Tischplatte, Abstand des Arms zum Rumpf und Gelenkgeschwindigkeit sind harte Grenzen. Alle Gelenke ausser dem
einen Arm sind gesperrt.

Die IK rechnet auf ihrer eigenen Kopie der Gelenkwinkel (den kommandierten Sollwerten), nie auf dem gemessenen Zustand
der Sim - sonst sacken die Arme ab (Befund Stufe 1.1).
"""
import mink
import numpy as np

from g1sim import action_space, brainco

SOLVER = "daqp"
TABLE_CLEARANCE = 0.02  # m, Mindestabstand Unterarm und Hand zur Tischplatte
# m, Soll-Abstand Arm zu Rumpf und Becken. Befund 1.5b: Ohne diese Grenze steckte der Oberarm bei Flaschen nahe der
# Koerpermitte 14-21 mm im Rumpf, am Roboter blieb die Hand 1-7 cm vor der Greifpose haengen. In READY_POSE: 9 mm.
# Die Grenze wirkt auf Geschwindigkeiten (linear) und kann den Soll-Abstand unterschreiten (gemessen bis 0,4 mm),
# ein Eindringen verhindert sie
BODY_CLEARANCE = 0.005
TRUNK_BODIES = ("torso_link", "pelvis")
# Arm-Bodies, die gegen den Rumpf geprueft werden, plus Handbasis. Die Finger kommen dem Rumpf nicht nahe, waeren aber
# 21 von 27 Pruefungen und machten die IK ~10x langsamer
ARM_LINKS = ("shoulder_yaw_link", "elbow_link", "wrist_roll_link", "wrist_pitch_link", "wrist_yaw_link")
# rad/s je Armgelenk und Regeltakt. Ohne Grenze sprang die IK bei unerreichbarem Ziel in einem Schritt 9,6 cm tief durch
# den Rumpf und blieb dort am Gelenkanschlag haengen (Befund 1.5b). Der Greifablauf braucht < 1 rad/s
MAX_JOINT_SPEED = 3.0
ITERATIONS = 2  # IK-Schritte je Regeltakt, jeder rechnet mit dt - die Geschwindigkeitsgrenze wird darauf aufgeteilt
POSTURE_COST = 1e-2     # haelt die freie siebte Richtung des Arms nah an der Startpose
WRIST_JOINTS = ("wrist_pitch_joint", "wrist_yaw_joint")  # fehlen am G1 mit 23 DoF


def _colliding(model, geoms):
    """Nur Geoms, die an Kontakten teilnehmen (Sichtgeometrie ohne contype/conaffinity zaehlt nicht)."""
    return [g for g in geoms if model.geom_contype[g] or model.geom_conaffinity[g]]


def _body_geoms(model, bodies):
    """Kontakt-Geoms genau dieser Bodies (ohne ihre Kinder)."""
    return _colliding(model, [g for b in bodies for g in mink.get_body_geom_ids(model, model.body(b).id)])


def hand_frame(side):
    """Name der Handbasis (Body), deren Pose die IK regelt."""
    return f"{brainco.PREFIX[side]}{side}_base_link"


def pose(rotation, translation):
    """Rotationsmatrix + Position -> mink.SE3."""
    return mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(np.asarray(rotation, float)),
                                                  np.asarray(translation, float))


class ArmIK:
    def __init__(self, model, side="right", lock_wrist=False, dt=1.0 / 30):
        """lock_wrist: Handgelenk Pitch/Yaw sperren wie am G1 mit 23 DoF."""
        self.model, self.dt = model, dt
        self.frame = hand_frame(side)
        joints = action_space.SIM_ARM_JOINTS[side]
        ids = [model.joint(n).id for n in joints]
        self.arm_qadr = model.jnt_qposadr[ids]
        free_dofs = {int(model.jnt_dofadr[j]) for j, n in zip(ids, joints) if not (lock_wrist and n.endswith(WRIST_JOINTS))}

        self.configuration = mink.Configuration(model)
        self.hand_task = mink.FrameTask(self.frame, "body", position_cost=1.0, orientation_cost=1.0, lm_damping=1e-3)
        self.posture_task = mink.PostureTask(model, cost=POSTURE_COST)
        self.constraints = [mink.DofFreezingTask(model, [i for i in range(model.nv) if i not in free_dofs])]
        forearm = _colliding(model, mink.get_subtree_geom_ids(model, model.body(f"{side}_elbow_link").id))
        arm = _body_geoms(model, [f"{side}_{link}" for link in ARM_LINKS] + [self.frame])
        trunk = _body_geoms(model, TRUNK_BODIES)
        self.limits = [
            mink.ConfigurationLimit(model),
            mink.VelocityLimit(model, {n: MAX_JOINT_SPEED / ITERATIONS for n in joints}),
            mink.CollisionAvoidanceLimit(model, [(forearm, ["table_top"])],
                                         minimum_distance_from_collisions=TABLE_CLEARANCE,
                                         collision_detection_distance=TABLE_CLEARANCE + 0.05),
            mink.CollisionAvoidanceLimit(model, [(arm, trunk)], minimum_distance_from_collisions=BODY_CLEARANCE,
                                         collision_detection_distance=BODY_CLEARANCE + 0.05),
        ]
        self._limited = model.jnt_limited.astype(bool) & (model.jnt_type >= 2)  # Scharnier- und Schubgelenke

    def reset(self, qpos):
        """IK auf eine Stellung setzen (volles qpos). Die Armwerte darin muessen die kommandierten Sollwerte sein."""
        q = self._clipped(qpos)
        self.configuration.update(q)
        self.posture_task.set_target(q)

    def _clipped(self, qpos):
        """qpos mit allen begrenzten Gelenken in ihrem Bereich. Schon 1e-10 rad ausserhalb macht das QP unloesbar, wenn die
        Rueckrichtung zugleich von einer Kollisionsgrenze gesperrt ist (Befund 1.5b)."""
        q = np.array(qpos, dtype=float)
        adr = self.model.jnt_qposadr[self._limited]
        q[adr] = np.clip(q[adr], *self.model.jnt_range[self._limited].T)
        return q

    @property
    def arm_q(self):
        """Die 7 Armwinkel (rad) in ARM_JOINTS-Reihenfolge."""
        return self.configuration.q[self.arm_qadr].copy()

    def hand_pose(self):
        """Pose der Handbasis in der aktuellen IK-Stellung (mink.SE3)."""
        return self.configuration.get_transform_frame_to_world(self.frame, "body")

    def step(self, target, iterations=ITERATIONS):
        """Stellung Richtung Soll-Pose (mink.SE3) bewegen. Rueckgabe: die 7 Armwinkel."""
        self.hand_task.set_target(target)
        for _ in range(iterations):
            velocity = mink.solve_ik(self.configuration, [self.hand_task, self.posture_task], self.dt, SOLVER,
                                     damping=1e-3, limits=self.limits, constraints=self.constraints)
            self.configuration.integrate_inplace(velocity, self.dt)
            self.configuration.update(self._clipped(self.configuration.q))
        return self.arm_q

    def solve(self, target, max_iterations=300, pos_tol=1e-3, rot_tol=np.deg2rad(1.0)):
        """So lange rechnen, bis die Soll-Pose erreicht ist oder max_iterations. Rueckgabe: (Armwinkel, Positionsfehler m,
        Winkelfehler rad)."""
        for _ in range(max_iterations):
            self.step(target, iterations=1)
            pos_err, rot_err = self.error(target)
            if pos_err < pos_tol and rot_err < rot_tol:
                break
        return self.arm_q, pos_err, rot_err

    def error(self, target):
        """(Positionsfehler m, Winkelfehler rad) der aktuellen IK-Stellung zur Soll-Pose."""
        current = self.hand_pose()
        pos_err = float(np.linalg.norm(current.translation() - target.translation()))
        rot_err = float(np.linalg.norm((current.rotation().inverse() @ target.rotation()).log()))
        return pos_err, rot_err
