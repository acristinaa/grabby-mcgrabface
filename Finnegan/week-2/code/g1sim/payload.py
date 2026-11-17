"""Anbauteile des echten G1: AGX-Rucksack (Rechner auf dem Ruecken) und schwenkbares Kopfmodul mit RealSense D455.

Modell: models/unitree_g1_payload/payload.xml (Unitree, BSD-3). Gebaut wird es in g1sim/build.py, damit Laufmodell und
Greifszene dieselben Teile tragen: +2,59 kg am Oberkoerper, davon 1,9 kg hinten.

Kopfmodul: Das Servo XL330 dreht die Kamera links/rechts (xl330_joint, +-50 Grad, positiv = nach links), darunter
neigt d455_joint sie (-20 .. +90 Grad, positiv = nach unten). Motoren gibt es dafuer keine - nu bleibt 41, der
26-D-Aktionsraum bleibt gleich. Stattdessen haelt je eine equality-Bedingung den Winkel, wie ein Servo seine Position.

Kameras: head_camera ist die Farbkamera der D455 - die Kopfkamera fuer die Policy (Masterplan 7.3). LeRobots
gleichnamige Platzhalter-Kamera wird entfernt: Sie sass 3 cm ueber der echten D435i, mitten im Kopfmodul, und sah bei
geneigter D455 nur deren Gehaeuse. Der Name bleibt, weil LeRobots Sim und LeRobots G1-Server "head_camera" streamen.
d435i_camera ist die im Kopf eingebaute D435i an ihrer Unitree-Pose (derzeit von keinem Ablauf genutzt).

Anbaurahmen: Die Unitree-URDFs haben ihren torso_link 1 cm tiefer als LeRobots MJCF (Kopf-Mesh bei z -0.044 statt
-0.054, beide Male genau am Becken-Nullpunkt). Mit ANCHOR_POS gelten die Unitree-Zahlen in payload.xml unveraendert.
"""
import pathlib

import mujoco
import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
PAYLOAD_DIR = REPO / "models" / "unitree_g1_payload"
PAYLOAD_XML = PAYLOAD_DIR / "payload.xml"
MESH_DIR = PAYLOAD_DIR / "meshes"

PARENT = "torso_link"
ANCHOR_POS = [0.0, 0.0, -0.010]  # URDF-torso_link in LeRobots torso_link

BODIES = ("backpack_link", "head_servo_link", "xl330_link", "d455_link")
CAMERA = "head_camera"  # D455
D435I_CAMERA = "d435i_camera"
# Body, Gelenk, equality, Drehachse im Body
HEAD = {
    "pan": ("xl330_link", "xl330_joint", "xl330_hold", [0.0, 0.0, 1.0]),
    "tilt": ("d455_link", "d455_joint", "d455_hold", [0.0, 1.0, 0.0]),
}


def attach(spec, pan=0.0, tilt=0.0):
    """Anbauteile an einen G1-MjSpec (LeRobot-Modell) haengen. Kopf startet in (pan, tilt), rad. Rueckgabe: spec."""
    placeholder = spec.camera(CAMERA)
    if placeholder is not None:
        spec.delete(placeholder)
    frame = spec.body(PARENT).add_frame()
    frame.name = "payload_mount"
    frame.pos = ANCHOR_POS
    child = mujoco.MjSpec.from_file(str(PAYLOAD_XML))
    # Ohne Praefix hiesse die Default-Klasse der Datei wie die des G1 ("main") und wird namenlos geschrieben -
    # die geschriebene Szene liesse sich dann nicht mehr laden
    child.default.name = "unitree_payload"
    spec.attach(child, prefix="", frame=frame)  # Unitree-Namen bleiben unveraendert
    # Kamera <-> Kopf: MuJoCo kollidiert mit der konvexen Huelle des Kopf-Meshs, die vorn ueber die Visier-Oeffnung des
    # echten Kopfes reicht. Ab 0,45 rad Neigung steckte die Kamera darin fest; laut Unitree neigt sie frei bis 90 Grad.
    exclude = spec.add_exclude()
    exclude.bodyname1, exclude.bodyname2 = PARENT, "d455_link"
    return set_start_pose(spec, pan, tilt)


def set_start_pose(spec, pan, tilt):
    """Startstellung des Kopfes im noch nicht kompilierten spec (rad).

    Der Body wird um den Winkel gedreht und joint.ref auf den Winkel gesetzt: Das Modell startet dort (qpos0 = ref),
    qpos bleibt der echte Servowinkel (0 = Nullstellung im URDF), die equality haelt genau diese Stellung.
    """
    for key, angle in (("pan", pan), ("tilt", tilt)):
        body_name, joint_name, _, axis = HEAD[key]
        joint, body = spec.joint(joint_name), spec.body(body_name)
        _check_range(key, angle, joint.range)
        if body.alt.type != mujoco.mjtOrientation.mjORIENTATION_QUAT:
            raise ValueError(f"{body_name}: Drehung in payload.xml als quat angeben - sonst gilt body.quat nicht")
        turn = np.zeros(4)
        mujoco.mju_axisAngle2Quat(turn, np.asarray(axis, float), angle - joint.ref)
        quat = np.zeros(4)
        mujoco.mju_mulQuat(quat, np.asarray(body.quat, float), turn)
        body.quat = quat
        joint.ref = angle
    return spec


def set_head(model, pan=None, tilt=None):
    """Neuer Sollwinkel der Servos (rad) im kompilierten Modell. Die equality zieht das Gelenk im naechsten Schritt hin.

    None laesst die Achse, wie sie ist. Rueckgabe: {achse: gesetzter Winkel}.
    """
    out = {}
    for key, angle in (("pan", pan), ("tilt", tilt)):
        if angle is None:
            continue
        _, joint_name, eq_name, _ = HEAD[key]
        joint = model.joint(joint_name)
        _check_range(key, angle, joint.range)
        # equality ohne zweites Gelenk: q = qpos0 + polycoef[0]
        model.eq_data[model.equality(eq_name).id, 0] = angle - model.qpos0[joint.qposadr[0]]
        out[key] = angle
    return out


def head_angles(model, data):
    """Aktuelle Servowinkel {pan, tilt} (rad)."""
    return {key: float(data.qpos[model.joint(joint_name).qposadr[0]]) for key, (_, joint_name, _, _) in HEAD.items()}


def _check_range(key, angle, joint_range):
    lo, hi = joint_range
    if not lo <= angle <= hi:
        raise ValueError(f"Kopf {key} {angle:.3f} rad ausserhalb {lo:.3f} .. {hi:.3f}")
