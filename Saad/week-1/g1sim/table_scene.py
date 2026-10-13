"""Stufe 1.1 - Greifszene auf dem Laufmodell: G1 + Revo2 mit fixiertem Becken vor einem Tisch.

Basis ist dasselbe MJCF wie beim Laufen (g1sim/build.py): gleiche Kinematik, Massen, Drehmomentgrenzen, Handmontage und
Anbauteile (Rucksack, D455-Kopfmodul).
Das Becken ist fixiert - die Demo-Fabrik braucht so keinen Lauf-Controller und kein DDS und laeuft schneller als
Echtzeit. Greifen im Stand mit Lauf-Controller folgt in Stufe 3.4.

Inhalt: Tisch, rote Flasche (0,5 kg), gruene Tasse, blauer Wuerfel; Kopfkamera (D455 am Kopfmodul, nach unten auf den
Tisch geneigt, g1sim/payload.py) und je eine Handgelenkkamera; grasp_site an beiden Haenden; condim 4 an Objekten und
Handkontakten (Masterplan §7.2).
Vorher muss scripts/51_build_brainco_sim.py die Sim unter SIM_DIR gebaut haben (Meshes).

Level 1 adds a second scene, "volleyball" (SCENES): a grey volleyball on a static tube stand left of the robot and a static
open basket on its right, on a table whose front edge is 5 cm closer (x = 0.20). Its own file, VOLLEYBALL_FILE. The bottle
scene stays the default. The volleyball scene also holds VOLLEYBALL_DISTRACTORS (free bodies, parked on the floor until the
randomization puts some on the table).
The Level 1 walk-up uses a third scene, "volleyball_walk" (VOLLEYBALL_WALK_FILE): the same table, ball, stand and basket, but
the pelvis keeps its free joint, so the robot can stand and walk. A mocap body "pelvis_hold" and a weld equality of the same
name hold the pelvis where TableEnv puts it (a virtual harness, off in the file, switched on by TableEnv).
"""
import os

import mujoco
import numpy as np

from dashboard import scene
from g1sim import MANIFEST, SIM_DIR, action_space, build, payload

SCENE_FILE = "assets/scene_29dof_brainco_table.xml"

PELVIS_HEIGHT = 0.793  # Becken in Standhoehe wie im LeRobot-Modell
TABLE_TOP = 0.74
TABLE_CENTER_X = 0.55
TABLE_HALF = (0.30, 0.45)  # Vorderkante bei x = 0.25, die Haende schweben in Nullstellung darueber

CAMERAS = ("head_camera", "left_wrist_camera", "right_wrist_camera")
# Kopfmodul (rad): 0,85 nach unten zeigt den ganzen Tisch, beide Haende und alle Objekte, aber nicht den eigenen Koerper
HEAD_POSE = {"pan": 0.0, "tilt": 0.85}

# name: (Form, Groesse wie MuJoCo, Farbe, Masse kg, Ruheposition x, y auf der Tischplatte)
OBJECTS = {
    "red_bottle": ("cylinder", [0.033, 0.105], [0.85, 0.10, 0.10, 1.0], 0.50, (0.40, -0.16)),
    "green_cup": ("cylinder", [0.040, 0.055], [0.15, 0.65, 0.25, 1.0], 0.25, (0.42, 0.16)),
    "blue_block": ("box", [0.030, 0.030, 0.030], [0.15, 0.30, 0.80, 1.0], 0.20, (0.48, 0.00)),
}

# Volleyball scene (Level 1). Ball: size 5 volleyball, thin shell, condim 6 (rolling friction) and priority 1, so its own
# friction applies in every contact. Grey like the real ball (user, 2026-10-08): it is found by its shape, not its colour.
VOLLEYBALL_FILE = "assets/scene_29dof_brainco_volleyball.xml"
VOLLEYBALL_WALK_FILE = "assets/scene_29dof_brainco_volleyball_walk.xml"
VOLLEYBALL_OBJECTS = {
    "volleyball": ("sphere", [0.105], [0.55, 0.55, 0.55, 1.0], 0.27, (0.344, 0.241)),  # 0.42 m from the waist axis, +35 deg
}
BALL_FRICTION = [0.9, 0.005, 0.001]
STAND_INNER_RADIUS = 0.05  # tube the ball rests on: inner diameter 0.10 m, 0.08 m high, made of box segments
STAND_HEIGHT = 0.08
STAND_WALL = 0.006
STAND_SEGMENTS = 16
BASKET_CENTER = (0.344, -0.241)
BASKET_INNER_HALF = 0.14  # open top, inner 0.28 x 0.28 m
BASKET_WALL_HEIGHT = 0.10
BASKET_WALL = 0.005  # wall and floor thickness

# Distractors for the volleyball scene: things that are not the ball, so the VLA has to find the ball by its shape and not
# just "the red thing" or "the only thing on the table". All are in the model; per episode the randomization puts 1-3 of
# them on the table (configs/randomization.json, group "distractors") and parks the rest on the floor far behind the robot.
# The ball is grey (found by shape), so colour must not give it away: the can is grey too, and the red cylinder is the
# colour the ball had before 2026-10-08. Upright cylinders and boxes only: nothing that rolls.
# name: (shape, size as MuJoCo, colour, mass kg)
VOLLEYBALL_DISTRACTORS = {
    "distractor_box": ("box", [0.06, 0.045, 0.04], [0.72, 0.55, 0.36, 1.0], 0.15),       # cardboard box 12 x 9 x 8 cm
    "distractor_can": ("cylinder", [0.033, 0.061], [0.78, 0.80, 0.84, 1.0], 0.35),       # drink can 6.6 cm x 12.2 cm
    "distractor_red_cylinder": ("cylinder", [0.04, 0.05], [0.85, 0.15, 0.10, 1.0], 0.20),
    "distractor_purple_cylinder": ("cylinder", [0.035, 0.07], [0.50, 0.20, 0.70, 1.0], 0.20),
    "distractor_yellow_block": ("box", [0.03, 0.03, 0.03], [0.95, 0.80, 0.15, 1.0], 0.10),
}
DISTRACTOR_PARK = (-10.0, 0.0)  # x, y on the floor; parked distractors stand in a row along y, 0.5 m apart
DISTRACTOR_PARK_GAP = 0.5
# Keep-out for distractors on the table (m, from the waist axis at x = y = 0): while it is carried, the ball (r 0.105) and both
# hands sweep an arc at 0.42 m from +35 to -35 degrees; 0.62 leaves ~5 cm to the outer edge of the hands.
DISTRACTOR_MIN_REACH = 0.62
# Standing posture of the free robot in "volleyball_walk" = GR00T's measured crouch, not its default angles (those would put
# the pelvis at 0.784). Provisional, from a 4 s GR00T Balance stand at dt 1/300 with READY arms (probe
# stand_0.0033333333333_ready.log, 2026-10-08): pelvis z 0.7505, roll 0.05 deg, pitch -0.96 deg. Stage 4 re-measures it in
# this scene and overwrites these numbers. legs: G1 motors 0-11 (hip pitch/roll/yaw, knee, ankle pitch/roll, left then right).
GROOT_STANCE = {
    "pelvis_z": 0.7505,
    "roll_deg": 0.05,
    "pitch_deg": -0.96,
    "legs": (-0.377, 0.0, 0.0, 0.709, -0.317, 0.0, -0.402, 0.0, 0.0, 0.697, -0.281, 0.0),
    "waist_roll": 0.0,
    "waist_pitch": 0.0,
}
STAND_Z = GROOT_STANCE["pelvis_z"]
# Stop radius (m): pelvis to ball and to basket centre when the robot stops at the table. The hips are at table height, so the
# knee touches the table when the pelvis is closer than ~0.205 m to the front edge (stance_clearance with GROOT_STANCE); at 0.45
# it stays ~0.24 m away, clearance 0.031 m (runs/l1w_s1_fix_clearance.log).
R_STOP = 0.45
# Stop pose of the nominal layout by the R_STOP rule (navigation.table_stop_pose, Level 1 Stage 3): x, y (m), yaw (deg)
WALK_STANCE = {"x_m": -0.036, "y_m": 0.0, "yaw_deg": 0.0}
PELVIS_HOLD = "pelvis_hold"  # mocap body and weld equality

SCENES = {
    # table: (centre x, (half length x, half width y)); static: bodies without joints that belong to the scene
    "bottle": {"file": SCENE_FILE, "objects": OBJECTS, "target": "red_bottle", "table": (TABLE_CENTER_X, TABLE_HALF),
               "static": ()},
    "volleyball": {"file": VOLLEYBALL_FILE, "objects": VOLLEYBALL_OBJECTS, "target": "volleyball",
                   "table": (0.525, (0.325, TABLE_HALF[1])),  # front edge x = 0.20, same back edge 0.85 and top height
                   "static": ("volleyball_stand", "basket"),
                   "holders": {"volleyball": "volleyball_stand"},  # the stand moves with the ball's start offset
                   # target (x, y) distance from the pelvis that a random start must keep, m (nominal 0.42): beyond ~0.53 the
                   # two-hand teacher lifted the ball < 6 cm in 13 of 13 starts (stand + ball offset added up, 2026-10-08)
                   "reach_m": (0.33, 0.52),
                   "distractors": VOLLEYBALL_DISTRACTORS,
                   "ready": action_space.BALL_READY_POSE},  # start pose of arms and hands (default READY_POSE)
    # free_base: keep the pelvis free joint; stance: where TableEnv puts the pelvis unless the variation has a "stance"
    "volleyball_walk": {"file": VOLLEYBALL_WALK_FILE, "objects": VOLLEYBALL_OBJECTS, "target": "volleyball",
                        "table": (0.525, (0.325, TABLE_HALF[1])), "static": ("volleyball_stand", "basket"),
                        "holders": {"volleyball": "volleyball_stand"}, "free_base": True, "stance": WALK_STANCE,
                        "ready": action_space.BALL_READY_POSE},
}
ALL_OBJECTS = {**OBJECTS, **VOLLEYBALL_OBJECTS, **VOLLEYBALL_DISTRACTORS}

GRASP_SITE_POS = [0.11, 0.0, 0.03]  # wie grasp_site im rechten Herstellermodell, Koordinaten der Handbasis
# Handgelenkkamera in Koordinaten der Handbasis: vor dem Handteller nahe am Handgelenk, Blick entlang der Finger
WRIST_CAM_POS = [0.05, 0.0, -0.02]
WRIST_CAM_FORWARD = [0.06, 0.0, 0.10]
WRIST_CAM_FOVY = 80.0


def object_rest_height(name, on=TABLE_TOP):
    """Height of the object's centre at rest. on: height of the surface it stands on (0 = floor; not for the ball)."""
    kind, size, *_ = ALL_OBJECTS[name]
    if kind == "sphere":  # the ball rests on the inner top edge of its stand
        r = size[0]
        return TABLE_TOP + STAND_HEIGHT - (r - np.sqrt(r * r - STAND_INNER_RADIUS ** 2)) + r
    return on + (size[1] if kind == "cylinder" else size[2])


def look_quat(forward, up):
    """Quaternion einer Kamera, die entlang `forward` blickt (MuJoCo: Kamera schaut entlang -z, +y ist oben)."""
    f = np.asarray(forward, float) / np.linalg.norm(forward)
    z = -f
    x = np.cross(np.asarray(up, float), z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.array([x, y, z]).T.flatten())
    return q


def build_spec(snap, scene_name="bottle"):
    """MjSpec der Greifszene. snap: Pfad der LeRobot-Sim (build.snapshot()). scene_name: key of SCENES."""
    setup = SCENES[scene_name]
    table_center_x, table_half = setup["table"]
    spec = build.build_spec(snap)
    payload.set_start_pose(spec, **HEAD_POSE)
    # Reibung zum Greifen: elliptischer Reibungskegel wie in der Wirklichkeit und impratio 10 gegen kuenstliches Rutschen
    # (MuJoCo-Doku). Mit dem Standard (Pyramide, 1) rutschte die Flasche in 36 von 36 Griffvarianten durch (Stufe 1.4).
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = 10.0

    pelvis = spec.body("pelvis")
    if setup.get("free_base"):
        add_pelvis_hold(spec)
    else:
        for joint in list(pelvis.joints):
            if joint.type == mujoco.mjtJoint.mjJNT_FREE:
                spec.delete(joint)
        pelvis.pos = [0.0, 0.0, PELVIS_HEIGHT]

    for side in ("left", "right"):
        base = spec.body(f"{scene.PREFIX[side]}{side}_base_link")
        if side == "left":  # das rechte Herstellermodell bringt seine grasp_site schon mit (rh_grasp_site)
            site = base.add_site()
            site.name = "lh_grasp_site"
            site.pos = GRASP_SITE_POS
            site.group = 4
        cam = base.add_camera()
        cam.name = f"{side}_wrist_camera"
        cam.pos = WRIST_CAM_POS
        cam.quat = look_quat(WRIST_CAM_FORWARD, up=[-1.0, 0.0, 0.0])
        cam.fovy = WRIST_CAM_FOVY

    for geom in spec.geoms:
        if geom.name.startswith(tuple(scene.PREFIX.values())) and geom.name.endswith("_collision"):
            geom.condim = 4

    world = spec.worldbody
    table = world.add_body()
    table.name = "table"
    table.pos = [table_center_x, 0.0, TABLE_TOP / 2.0]
    top = table.add_geom()
    top.name = "table_top"
    top.type = mujoco.mjtGeom.mjGEOM_BOX
    top.size = [table_half[0], table_half[1], TABLE_TOP / 2.0]
    top.rgba = [0.55, 0.42, 0.30, 1.0]
    top.friction = [1.0, 0.02, 0.001]

    for name, (kind, size, rgba, mass, (x, y)) in setup["objects"].items():
        body = world.add_body()
        body.name = name
        body.pos = [x, y, object_rest_height(name) + 0.002]
        joint = body.add_freejoint()
        joint.name = f"{name}_joint"
        geom = body.add_geom()
        geom.name = f"{name}_geom"
        geom.type = GEOM_TYPES[kind]
        geom.size = size + [0.0] * (3 - len(size))
        geom.rgba = rgba
        geom.mass = mass
        if kind == "sphere":  # hollow ball: inertia of a thin shell (2/3 m r^2)
            geom.typeinertia = mujoco.mjtGeomInertia.mjINERTIA_SHELL
            geom.condim = 6
            geom.friction = BALL_FRICTION
            geom.priority = 1
        else:
            geom.condim = 4
            geom.friction = [1.2, 0.05, 0.001]
    for i, (name, (kind, size, rgba, mass)) in enumerate(setup.get("distractors", {}).items()):
        body = world.add_body()
        body.name = name
        body.pos = [*distractor_park(i), object_rest_height(name, on=0.0) + 0.002]
        joint = body.add_freejoint()
        joint.name = f"{name}_joint"
        geom = body.add_geom()
        geom.name = f"{name}_geom"
        geom.type = GEOM_TYPES[kind]
        geom.size = size + [0.0] * (3 - len(size))
        geom.rgba = rgba
        geom.mass = mass
        geom.condim = 4
        geom.friction = [1.2, 0.05, 0.001]
    if scene_name in ("volleyball", "volleyball_walk"):
        add_stand(world, *VOLLEYBALL_OBJECTS["volleyball"][4])
        add_basket(world, *BASKET_CENTER)
    return spec


def distractor_park(index):
    """Floor spot (x, y) of the index-th distractor of a scene while it is not on the table."""
    return DISTRACTOR_PARK[0], DISTRACTOR_PARK[1] + index * DISTRACTOR_PARK_GAP


GEOM_TYPES = {"cylinder": mujoco.mjtGeom.mjGEOM_CYLINDER, "box": mujoco.mjtGeom.mjGEOM_BOX,
              "sphere": mujoco.mjtGeom.mjGEOM_SPHERE}


def _add_box(body, name, half, pos, rgba, yaw=0.0):
    geom = body.add_geom()
    geom.name = name
    geom.type = mujoco.mjtGeom.mjGEOM_BOX
    geom.size = list(half)
    geom.pos = list(pos)
    geom.quat = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
    geom.rgba = rgba


def add_stand(world, x, y):
    """Static tube (no joint) on the table at x, y: STAND_SEGMENTS boxes on a circle, inner faces at STAND_INNER_RADIUS."""
    body = world.add_body()
    body.name = "volleyball_stand"
    body.pos = [x, y, TABLE_TOP]
    step = 2 * np.pi / STAND_SEGMENTS
    radius = STAND_INNER_RADIUS + STAND_WALL / 2
    # tangential half width so neighbouring segments meet at the outer face (no gaps)
    half = [STAND_WALL / 2, (STAND_INNER_RADIUS + STAND_WALL) * np.tan(step / 2), STAND_HEIGHT / 2]
    for i in range(STAND_SEGMENTS):
        a = i * step
        _add_box(body, f"volleyball_stand_{i}", half, [radius * np.cos(a), radius * np.sin(a), STAND_HEIGHT / 2],
                 [0.85, 0.85, 0.82, 1.0], yaw=a)


def add_basket(world, x, y):
    """Static open basket (no joint) on the table at x, y: floor and four walls, inner BASKET_INNER_HALF * 2 square."""
    body = world.add_body()
    body.name = "basket"
    body.pos = [x, y, TABLE_TOP]
    rgba = [0.25, 0.40, 0.60, 1.0]
    outer = BASKET_INNER_HALF + BASKET_WALL
    t = BASKET_WALL / 2
    wall_z = BASKET_WALL + BASKET_WALL_HEIGHT / 2
    _add_box(body, "basket_floor", [outer, outer, t], [0.0, 0.0, t], rgba)
    for sign in (1, -1):
        _add_box(body, f"basket_wall_y{sign:+d}", [outer, t, BASKET_WALL_HEIGHT / 2],
                 [0.0, sign * (BASKET_INNER_HALF + t), wall_z], rgba)
        _add_box(body, f"basket_wall_x{sign:+d}", [t, BASKET_INNER_HALF, BASKET_WALL_HEIGHT / 2],
                 [sign * (BASKET_INNER_HALF + t), 0.0, wall_z], rgba)


def add_pelvis_hold(spec):
    """Mocap body PELVIS_HOLD (no geoms) and a weld of the same name that pins the pelvis to it (relpose identity).

    The weld is off in the file; TableEnv moves the mocap body to the pelvis and switches it on (TableEnv.hold_pelvis).
    """
    hold = spec.worldbody.add_body()
    hold.name = PELVIS_HOLD
    hold.mocap = True
    weld = spec.add_equality()
    weld.name = PELVIS_HOLD
    weld.type = mujoco.mjtEq.mjEQ_WELD
    weld.objtype = mujoco.mjtObj.mjOBJ_BODY
    weld.name1, weld.name2 = "pelvis", PELVIS_HOLD
    weld.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # anchor, relpose (pos, quat), torquescale
    weld.solref = [0.01, 1.0]
    weld.active = False


def write(snap=None, out=SIM_DIR, scene="bottle"):
    """Szene nach out/<Datei der Szene> schreiben und von der Platte laden. Rueckgabe: MjModel. scene: key of SCENES."""
    if not (out / MANIFEST).is_file():
        raise FileNotFoundError(f"{out} ist nicht gebaut - zuerst scripts/51_build_brainco_sim.py")
    spec = build_spec(snap or build.snapshot(), scene)
    spec.compile()
    path = out / SCENES[scene]["file"]
    # Erst eine eigene Datei schreiben, dann umbenennen: Laufen mehrere Skripte parallel, liest keines eine halbe Datei
    tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    tmp.write_text(spec.to_xml())
    os.replace(tmp, path)
    return mujoco.MjModel.from_xml_path(str(path))
