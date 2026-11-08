"""27-D action space `g1_brainco` + waist yaw - the single source for order and units (Stufe 1.2, Level 1 walk-up).

Indices 0-25 as unitree_lerobot `Unitree_G1_Brainco` (unitree_lerobot/utils/constants.py, Commit 41c2805), gleich fuer
G1 mit 23 und 29 DoF; index 26 (waist yaw) is our addition, so the VLA can carry the volleyball with the waist:
  0-6    linker Arm    Schulter Pitch/Roll/Yaw, Ellbogen, Handgelenk Roll/Pitch/Yaw   rad   (G1-Motoren 15-21)
  7-13   rechter Arm   dito                                                          rad   (G1-Motoren 22-28)
  14-19  linke Hand    Daumen, Daumen-Aux, Zeige, Mittel, Ring, Klein                0 = offen .. 1 = zu
  20-25  rechte Hand   dito
  26     waist yaw     + = turn left, absolute                                       rad   (G1 motor 12)
Zustand (state) und Aktion (actions) haben dasselbe Layout. Legs, waist roll and waist pitch belong to the walking controller.
Actions are absolute targets. For training, openpi turns the arm joints and the waist yaw into deltas (DELTA_MASK).
"""
import numpy as np

from g1sim import brainco

DIM = 27
SIDES = ("left", "right")

ARM_JOINTS = ("ShoulderPitch", "ShoulderRoll", "ShoulderYaw", "Elbow", "WristRoll", "WristPitch", "WristYaw")
HAND_MOTORS = ("Thumb", "ThumbAux", "Index", "Middle", "Ring", "Pinky")  # gleiche Reihenfolge wie brainco.MOTORS

NAMES = (
    *(f"kLeft{j}" for j in ARM_JOINTS),
    *(f"kRight{j}" for j in ARM_JOINTS),
    *(f"kLeftHand{m}" for m in HAND_MOTORS),
    *(f"kRightHand{m}" for m in HAND_MOTORS),
    "kWaistYaw",
)
SLICES = {"left_arm": slice(0, 7), "right_arm": slice(7, 14), "left_hand": slice(14, 20), "right_hand": slice(20, 26),
          "waist": slice(26, 27)}

# G1-Motorindizes der Arme (LeRobot G1_29_JointArmIndex) und MuJoCo-Gelenknamen, je Seite in ARM_JOINTS-Reihenfolge
ARM_MOTOR_INDEX = {"left": tuple(range(15, 22)), "right": tuple(range(22, 29))}
SIM_ARM_JOINTS = {
    side: tuple(f"{side}_{j}_joint" for j in
                ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"))
    for side in SIDES
}

# Armgelenke als Delta zum aktuellen Zustand trainieren, Haende absolut - wie Greifer in openpis ALOHA-Beispielen; the waist
# yaw is a joint angle like the arms, so it is a delta too
DELTA_MASK = (True,) * 14 + (False,) * 12 + (True,)

# Startpose am Tisch, gemessen 2026-09-15 (PROGRESS.md): Unterarme angehoben, offene Haende knapp vor der Tischkante ueber
# den Objekten, >= 12 cm Abstand zu Tisch und Objekten. In Nullstellung steckten die rechten Finger 2,1 cm in der Flasche.
# Gilt fuer die Sim (g1sim/table_env.py), die Policy-Server-Metadaten (openpi_g1/config.py) und spaeter den echten Roboter.
# Roll/Yaw = 0, daher fuer beide Arme gleich.
READY_ARM = (0.0, 0.0, 0.0, -0.6, 0.0, -0.3, 0.0)  # rad, Reihenfolge ARM_JOINTS
READY_POSE = np.array(READY_ARM * 2 + (0.0,) * 12 + (0.0,), dtype=np.float32)  # hands open, waist yaw 0

# Start and end pose of the volleyball scenes (Level 1, 2026-10-08): ready to pick up a ball - upper arms hanging slightly
# forward, elbows beside the body (6.5 cm in front of the shoulder line, 5.3 cm out; the first try had them behind the
# torso), forearms raised, palms facing each other, fingers forward and 25 deg up, relaxed. Found by a joint grid search
# with forward kinematics. The hands sit at chest height on purpose: >= 4.9 cm from ball, stand, basket and table at the
# nominal walk stop, >= 4.5 cm while the waist turns -40..40 deg with the ball up to 3 cm off; lower hands touched the ball
# (the ball top is at 1.02 m). Right arm; the left arm is its mirror image (shoulder roll/yaw and wrist roll/yaw negated).
BALL_READY_RIGHT_ARM = (-0.25, -0.05, 0.0, -0.8, 0.0, 0.6, 0.0)  # rad, order ARM_JOINTS
BALL_READY_HAND = (0.1, 0.1, 0.15, 0.15, 0.15, 0.15)  # relaxed fingers, order HAND_MOTORS
MIRROR = np.array([1, -1, -1, 1, -1, 1, -1])  # right arm -> left arm: roll and yaw joints change sign
BALL_READY_POSE = np.array((*(MIRROR * BALL_READY_RIGHT_ARM), *BALL_READY_RIGHT_ARM, *BALL_READY_HAND * 2, 0.0),
                           dtype=np.float32)


def join(left_arm, right_arm, left_hand, right_hand, waist):
    """Five parts (rad, rad, normalized, normalized, rad) -> 27-D vector (float32)."""
    vec = np.concatenate([left_arm, right_arm, left_hand, right_hand, np.atleast_1d(waist)]).astype(np.float32)
    if vec.shape != (DIM,):
        raise ValueError(f"erwartet {DIM} Werte, bekommen {vec.shape}")
    return vec


def split(vec):
    """27-D vector -> {"left_arm", "right_arm", "left_hand", "right_hand", "waist"}."""
    vec = np.asarray(vec)
    if vec.shape[-1] != DIM:
        raise ValueError(f"erwartet {DIM} Werte, bekommen {vec.shape}")
    return {name: vec[..., s] for name, s in SLICES.items()}


def from_sim(arm_q, hand_q_rad, waist_q):
    """Sim joint angles -> 27-D vector.

    arm_q:      {"left": 7, "right": 7} rad, Reihenfolge ARM_JOINTS
    hand_q_rad: {"left": 6, "right": 6} rad, Reihenfolge brainco.MOTORS
    waist_q:    waist yaw, rad
    """
    return join(arm_q["left"], arm_q["right"], brainco.to_norm(hand_q_rad["left"]), brainco.to_norm(hand_q_rad["right"]),
                waist_q)


def arm_targets(vec):
    """27-D vector -> {"left": 7, "right": 7} Arm-Sollwinkel in rad."""
    parts = split(vec)
    return {"left": parts["left_arm"], "right": parts["right_arm"]}


def hand_targets(vec):
    """27-D vector -> {"left": 6, "right": 6} Hand-Sollwerte normiert, auf 0..1 begrenzt (wie der echte Handdienst)."""
    parts = split(vec)
    return {side: np.clip(parts[f"{side}_hand"], 0.0, 1.0) for side in SIDES}


def lerobot_arm_action(vec):
    """27-D vector -> Arm-Teil als Aktion fuer LeRobots UnitreeG1.send_action ({"kLeftShoulderPitch.q": ...})."""
    return {f"{name}.q": float(v) for name, v in zip(NAMES[:14], np.asarray(vec)[:14])}
