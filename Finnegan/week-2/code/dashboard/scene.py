"""Szenenaufbau G1 + BrainCo Revo2 - gemeinsame Quelle fuer Skript und Dashboard.

Diese Logik lag urspruenglich in scripts/15_attach_hands.py. Sie ist hierher
gezogen, damit das Dashboard die Szene mit veraenderter Handmontage neu bauen kann,
ohne den Code zu duplizieren. scripts/15_attach_hands.py ruft dieselbe Funktion.

Modell der Haende (ohne Login herunterladbar):
  https://www.brainco-hz.com/docs/revolimb-hand/en/revo2/download.html
  -> Revo2_xml.zip, entpackt nach models/revo2/{xml_left,xml_right}/
"""
import copy
import pathlib
import shutil

import mujoco
import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
MODELS = REPO / "models"
OUT = MODELS / "g1_brainco"
REVO2 = MODELS / "revo2"
SCENE_IN = OUT / "scene_table.xml"
SCENE_OUT = OUT / "scene_table_hands.xml"

HAND_XML = {
    "left": REVO2 / "xml_left" / "brainco-lefthand-v2.xml",
    "right": REVO2 / "xml_right" / "brainco-righthand-v2.xml",
}
PREFIX = {"left": "lh_", "right": "rh_"}

# Adapterflansch Handgelenk -> Revo2, aus Unitrees offiziellem G1-Modell mit BrainCo-Haenden (BSD-3)
MOUNT_DIR = MODELS / "unitree_brainco_mount"
ADAPTER_XML = {"left": MOUNT_DIR / "adapter_left.xml", "right": MOUNT_DIR / "adapter_right.xml"}
ADAPTER_POS = [0.0415, 0.0, 0.0]

# Offizielle Montage der Handbasis relativ zu {side}_wrist_yaw_link: unitreerobotics/unitree_ros, Commit 43e9f3e,
# robots/g1_with_brainco_hand/g1_29dof_mode_15_brainco_hand.urdf, Gelenke {side}_base_joint.
# Die Handbasis-Koordinaten von URDF und Hersteller-MJCF sind identisch (Fingergelenke verglichen 2026-09-14).
URDF_HAND_BASE = {
    "left": {"xyz": [0.0591, 0.0, 0.0], "rpy": [-np.pi / 2, 0.0, -np.pi / 2]},
    "right": {"xyz": [0.0591, 0.0, 0.0], "rpy": [np.pi / 2, 0.0, np.pi / 2]},
}

# Montagerahmen fuer MjSpec.attach, je Seite: Position und Euler-Winkel (rad, XYZ).
# Ergibt die URDF-Montage oben: Das Hersteller-MJCF dreht seine Basis selbst um -90 Grad um z, der Rahmen gleicht das
# aus (links = +90 Grad um y). Bis 2026-09-14 stand hier (0.0415, +-0.003, 0) ohne Drehung: Finger zeigten 90 Grad
# nach oben, der linke Daumen steckte im Handgelenk. Das Dashboard kann weiter feinjustieren, mount_error() misst die
# Abweichung vom URDF.
DEFAULT_MOUNT = {
    "left": {"pos": [0.0591, 0.0, 0.0], "euler": [0.0, np.pi / 2, 0.0]},
    "right": {"pos": [0.0591, 0.0, 0.0], "euler": [0.0, np.pi / 2, -np.pi]},
}

# Offscreen-Framebuffer. MuJoCo begrenzt sonst auf 640x480, was fuers Dashboard
# zu klein ist. Muss vor dem Kompilieren gesetzt sein.
OFFSCREEN = (1280, 960)


def euler_to_quat(euler):
    """XYZ-Euler (rad) -> MuJoCo-Quaternion (w, x, y, z)."""
    q = np.zeros(4)
    seq = "xyz"
    mujoco.mju_euler2Quat(q, np.asarray(euler, dtype=float), seq)
    return q


def default_mount():
    """Frische, unabhaengige Kopie der Standardmontage."""
    return copy.deepcopy(DEFAULT_MOUNT)


def check_hand_models():
    """Fehlt ein Handmodell, klar sagen was zu tun ist."""
    missing = [str(p) for p in HAND_XML.values() if not p.is_file()]
    if missing:
        raise FileNotFoundError(
            "Handmodell(e) fehlen:\n  " + "\n  ".join(missing) + "\n"
            "Revo2_xml.zip von brainco-hz.com laden und nach models/revo2/ entpacken."
        )
    missing = [str(p) for p in ADAPTER_XML.values() if not p.is_file()]
    if missing:
        raise FileNotFoundError("Adapter-Modell(e) fehlen (gehoeren zum Repo):\n  " + "\n  ".join(missing))


def asset_mesh_dirs():
    """Mesh-Ordner aller angebauten Teile (Haende + Adapter). Deren XMLs verweisen auf "meshes/<datei>"."""
    return [p.parent / "meshes" for p in (*HAND_XML.values(), *ADAPTER_XML.values())]


def remove_passive_tip_joints(hand):
    """Lose Fingerspitzen-Gelenke aus einem Revo2-Handmodell entfernen. Rueckgabe: ihre Namen.

    Das rechte Herstellermodell hat an jeder Fingerspitze ein zusaetzliches Gelenk (`*_tip`, `*_tip_joint`):
    ohne Motor, ohne Kopplung, ohne Grenzen. Die Spitzen samt Kontaktflaeche klappen damit frei weg.
    Das linke Modell hat diese Gelenke nicht. Danach sind die Spitzen starr am Endglied, wie links.
    """
    tips = [j for j in hand.joints if j.name.endswith(("_tip", "_tip_joint"))]
    names = [j.name for j in tips]
    for j in tips:
        hand.delete(j)
    return names


def attach_revo2(spec, mount=None, prepare_hand=None):
    """Revo2 an beide Handgelenke eines G1-MjSpec haengen. Veraendert spec und gibt es zurueck.

    Passt fuer das Menagerie-Modell (Tischszene) und fuer LeRobots g1_29dof_no_hand.xml (Laufmodell).
    Die Gummihand-Attrappe ist bei Menagerie ein Geom am Handgelenk, bei LeRobot ein eigener Body -
    beides wird entfernt.

    mount: {"left": {"pos": [x,y,z], "euler": [rx,ry,rz]}, "right": {...}}, None -> DEFAULT_MOUNT.
    prepare_hand(side, hand_spec): optional, passt das Handmodell vor dem Anbauen an (z. B. Aktuatortyp).
    """
    check_hand_models()
    mount = mount or default_mount()

    for side in ("left", "right"):
        wrist = spec.body(f"{side}_wrist_yaw_link")

        # Platzhalter-Gummihand entfernen (reine Sichtgeometrie, keine Gelenke)
        for g in list(wrist.geoms):
            if g.meshname == f"{side}_rubber_hand":
                spec.delete(g)
        rubber = spec.body(f"{side}_rubber_hand")
        if rubber is not None:
            spec.delete(rubber)

        adapter_frame = wrist.add_frame()
        adapter_frame.name = f"{side}_adapter_mount"
        adapter_frame.pos = ADAPTER_POS
        adapter = mujoco.MjSpec.from_file(str(ADAPTER_XML[side]))
        spec.attach(adapter, prefix=f"{PREFIX[side]}{side}_", frame=adapter_frame)

        frame = wrist.add_frame()
        frame.name = f"{side}_hand_mount"
        frame.pos = list(mount[side]["pos"])
        frame.quat = euler_to_quat(mount[side]["euler"])

        hand = mujoco.MjSpec.from_file(str(HAND_XML[side]))
        remove_passive_tip_joints(hand)
        if prepare_hand is not None:
            prepare_hand(side, hand)
        spec.attach(hand, prefix=PREFIX[side], frame=frame)

    return spec


def build_with_hands(mount=None, offscreen=OFFSCREEN):
    """Tischszene laden, Platzhalter-Haende ersetzen, echte Revo2 anbauen.

    mount: {"left": {"pos": [x,y,z], "euler": [rx,ry,rz]}, "right": {...}}
           None -> DEFAULT_MOUNT.
    Rueckgabe: kompilierbares MjSpec (noch nicht kompiliert).
    """
    spec = mujoco.MjSpec.from_file(str(SCENE_IN))

    # Groesserer Offscreen-Puffer, sonst kann das Dashboard nur 640x480 rendern
    if offscreen:
        spec.visual.global_.offwidth = int(offscreen[0])
        spec.visual.global_.offheight = int(offscreen[1])

    return attach_revo2(spec, mount)


def copy_hand_assets():
    """Hand-Meshes in den Asset-Ordner der Szene kopieren.

    Noetig, damit die geschriebene XML eigenstaendig ladbar ist: die Szene hat
    meshdir="assets/", die Hand-XML referenziert "meshes/*.STL".
    """
    dst = OUT / "assets" / "meshes"
    dst.mkdir(parents=True, exist_ok=True)
    n = 0
    for src in asset_mesh_dirs():
        for stl in src.glob("*.STL"):
            shutil.copy2(stl, dst / stl.name)
            n += 1
    return n, dst


def mount_error(model, data):
    """Abweichung jeder Handbasis von der offiziellen URDF-Montage: {side: (Position mm, Winkel Grad)}.

    data muss nach mj_forward stehen. Gemessen relativ zu {side}_wrist_yaw_link, unabhaengig von der Armstellung.
    """
    out = {}
    for side in ("left", "right"):
        wrist = model.body(f"{side}_wrist_yaw_link").id
        base = hand_base_id(model, side)
        pos = (data.xpos[base] - data.xpos[wrist]) @ data.xmat[wrist].reshape(3, 3)

        target = np.zeros(4)
        mujoco.mju_euler2Quat(target, np.asarray(URDF_HAND_BASE[side]["rpy"], dtype=float), "XYZ")  # URDF: feste Achsen
        wrist_inv, rel, target_inv, err = np.zeros(4), np.zeros(4), np.zeros(4), np.zeros(4)
        mujoco.mju_negQuat(wrist_inv, data.xquat[wrist])
        mujoco.mju_mulQuat(rel, wrist_inv, data.xquat[base])
        mujoco.mju_negQuat(target_inv, target)
        mujoco.mju_mulQuat(err, target_inv, rel)
        angle = float(2.0 * np.degrees(np.arccos(min(1.0, abs(float(err[0]))))))
        out[side] = (float(np.linalg.norm(pos - URDF_HAND_BASE[side]["xyz"]) * 1000.0), angle)
    return out


def hand_actuator_ids(model):
    """Indizes der 12 Hand-Aktuatoren (6 je Hand)."""
    return [
        i for i in range(model.nu)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or "").startswith(("lh_", "rh_"))
    ]


def distal_body_ids(model, side):
    """Endglieder einer Hand (5 Stueck).

    ACHTUNG: Die Herstellermodelle sind nicht symmetrisch - die rechte Hand hat
    zusaetzliche `*_tip`-Bodies, die linke endet bei `*_distal_link`. Deshalb
    wird auf `_distal_link` gematcht, das gibt es in beiden.
    """
    pre = PREFIX[side]
    return [
        i for i in range(model.nbody)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "").startswith(pre)
        and (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "").endswith("_distal_link")
    ]


def hand_base_id(model, side):
    return mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, f"{PREFIX[side]}{side}_base_link"
    )
