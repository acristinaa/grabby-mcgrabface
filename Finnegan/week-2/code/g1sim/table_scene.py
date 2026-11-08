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
name hold the pelvis where TableEnv puts it (a virtual harness, off in the file, switched on by TableEnv). No distractors there
yet: their keep-out (DISTRACTOR_MIN_REACH) is measured from the fixed robot's waist axis; the walk world adds them later.
"""
import os

import mujoco
import numpy as np

from dashboard import scene
from g1sim import MANIFEST, SIM_DIR, action_space, build, navigation, payload

SCENE_FILE = "assets/scene_29dof_brainco_table.xml"

PELVIS_HEIGHT = 0.793  # Becken in Standhoehe wie im LeRobot-Modell
TABLE_TOP = 0.74
TABLE_CENTER_X = 0.55
TABLE_HALF = (0.30, 0.45)  # Vorderkante bei x = 0.25, die Haende schweben in Nullstellung darueber
TABLE_RGBA = [0.55, 0.42, 0.30, 1.0]  # wood brown of the table top (block and slab)

# Table look (2026-11-01): the head-camera images are training data and a real table is a thin top on four legs, not a solid
# block. The block stays the collision shape (geom "table_top", geom 0 of body "table": same size, place, friction, contacts),
# but sits in HIDDEN_GROUP, which mujoco.Renderer and the viewer do not draw (MjvOption.geomgroup default: groups 0-2 only).
# Visual-only geoms (contype = conaffinity = 0, density 0: no contacts, no mass) draw the top slab and the legs, all inside the
# block, so a clearance measured to the block is never larger than to what the camera sees. Config image.table_style picks
# the look per run (randomization.ModelVariation): "legs" (as built) or "block" (the look run1 was recorded with).
TABLE_STYLES = ("legs", "block")
VISIBLE_GROUP, HIDDEN_GROUP = 0, 3
TABLE_SLAB = 0.03         # top thickness, m: common 25-30 mm desk top
TABLE_LEG = 0.045         # square leg side, m: common 40-50 mm table leg
TABLE_LEG_INSET = 0.04    # gap from each table edge to the leg's outer faces, m
TABLE_LEG_RGBA = [0.32, 0.32, 0.34, 1.0]  # grey steel legs
# Table height randomization (config objects.table_height_m, user decision 2026-11-01): the real table top sits 0.69-0.81 m above
# the robot's feet depending on its hip height (runbook), the sim one always at TABLE_TOP. randomization.ModelVariation moves
# body "table" (collision block and look) up or down; TableEnv moves what stands on it. The robot's stance stays. The block keeps
# its size, so raised by dz it floats dz above the floor and lowered it reaches into it: nothing of the robot comes near the
# floor under the table's front edge (the knee is the closest part, at hip height, navigation.R_STOP). The visual legs grow
# or shrink so they still meet the floor. Largest offset load() accepts: the widest real deviation, 0.81 - TABLE_TOP.
TABLE_HEIGHT_MAX_M = 0.07

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
# knee touches the table when the pelvis is closer than ~0.205 m to the front edge (stance_clearance with GROOT_STANCE);
# clearance grows ~1:1 with the distance (runs/l1w_s1_fix_clearance.log). 0.458 puts the nominal stop 0.2455 m from the edge,
# clearance 0.04 m = 2 cm arrival error + 2 cm margin (0.45 left only 0.031 m).
R_STOP = 0.458
# Stop pose of the nominal layout by the R_STOP rule (navigation.table_stop_pose, Level 1 Stage 3): x, y (m), yaw (deg)
WALK_STANCE = navigation.table_stop_pose(VOLLEYBALL_OBJECTS["volleyball"][4], BASKET_CENTER, R_STOP)
PELVIS_HOLD = "pelvis_hold"  # mocap body and weld equality
# Distractors for the volleyball scene: things that are not the ball, so the VLA has to find the ball by its shape and not
# just "the red thing" or "the only thing on the table". All are in the model; per episode the randomization puts 1-3 of
# them on the table (configs/randomization.json, group "distractors") and parks the rest on the floor far behind the robot.
# The red cylinder is red like the ball on purpose. Upright cylinders and boxes only: nothing that rolls.
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

SCENES = {
    # table: (centre x, (half length x, half width y)); static: bodies without joints that belong to the scene
    "bottle": {"file": SCENE_FILE, "objects": OBJECTS, "target": "red_bottle", "table": (TABLE_CENTER_X, TABLE_HALF),
               "static": ()},
    "volleyball": {"file": VOLLEYBALL_FILE, "objects": VOLLEYBALL_OBJECTS, "target": "volleyball",
                   "table": (0.525, (0.325, TABLE_HALF[1])),  # front edge x = 0.20, same back edge 0.85 and top height
                   "static": ("volleyball_stand", "basket"),
                   "holders": {"volleyball": "volleyball_stand"},  # the stand moves with the ball's start offset
                   "ready": action_space.BALL_READY_POSE,  # start pose of arms and hands (default READY_POSE)
                   # target (x, y) distance from the pelvis that a random start must keep, m (nominal 0.42): beyond ~0.53 the
                   # two-hand teacher lifted the ball < 6 cm in 13 of 13 starts (stand + ball offset added up, 2026-10-08)
                   "reach_m": (0.33, 0.52),
                   "distractors": VOLLEYBALL_DISTRACTORS},
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

# Texture randomization (plan humming-squishing-treasure A3, 2026-11-01): a policy trained on one wooden table and one grey
# floor can latch onto those exact pixels; at SAP the table and floor look different. These materials are in every scene file,
# unused until randomization.ModelVariation assigns one to a surface per episode (config image.textures). MuJoCo builtin
# textures only (checker / gradient / flat, some flat ones with a tile edge or speckles): generated at load, no image files.
# Every colour keeps r <= g and r <= b: times a random table colour (r <= max_red_ratio * g, b) the product stays inside the
# not-red rule, so the red-colour checks still see nothing red on the table. The ball and the basket are never textured.
# name: (builtin, rgb1, rgb2, mark, markrgb, texrepeat per m)
TEXTURE_SIZE = 256  # px per side; 12 RGB textures = 2.4 MB
TEXTURE_SURFACES = ("table_top", "floor")  # geoms the randomization may texture (the scenes have no wall or backdrop)
# Colour and texture drawn for a surface also go to its visual twin, so the table looks the same in either TABLE_STYLES
LOOK_TWINS = {"table_top": "table_slab"}
DR_MATERIALS = {
    "dr_checker_grey": ("checker", (0.55, 0.55, 0.55), (0.35, 0.35, 0.35), "none", (0, 0, 0), 4.0),
    "dr_checker_blue": ("checker", (0.30, 0.40, 0.60), (0.75, 0.80, 0.85), "none", (0, 0, 0), 4.0),
    "dr_checker_green": ("checker", (0.30, 0.45, 0.35), (0.70, 0.75, 0.70), "none", (0, 0, 0), 2.0),
    "dr_checker_dark": ("checker", (0.15, 0.15, 0.18), (0.25, 0.25, 0.28), "none", (0, 0, 0), 8.0),
    "dr_gradient_teal": ("gradient", (0.20, 0.45, 0.45), (0.60, 0.70, 0.70), "none", (0, 0, 0), 1.0),
    "dr_gradient_grey": ("gradient", (0.85, 0.85, 0.85), (0.40, 0.40, 0.42), "none", (0, 0, 0), 1.0),
    "dr_gradient_blue": ("gradient", (0.15, 0.20, 0.40), (0.50, 0.60, 0.75), "none", (0, 0, 0), 1.0),
    "dr_flat_light": ("flat", (0.80, 0.82, 0.82), (0, 0, 0), "none", (0, 0, 0), 1.0),
    "dr_flat_dark": ("flat", (0.20, 0.22, 0.22), (0, 0, 0), "none", (0, 0, 0), 1.0),
    "dr_flat_olive": ("flat", (0.40, 0.45, 0.40), (0, 0, 0), "none", (0, 0, 0), 1.0),
    "dr_flat_speckle": ("flat", (0.55, 0.57, 0.57), (0, 0, 0), "random", (0.30, 0.32, 0.32), 2.0),
    "dr_flat_tiles": ("flat", (0.70, 0.72, 0.74), (0, 0, 0), "edge", (0.35, 0.36, 0.38), 4.0),
}
TEXTURE_SPECKLE = 0.1  # share of pixels the "random" mark colours

# Render look (2026-11-01, user: "the image quality is not great"). Measured in the built scene (runs/l1h2_render_found.log):
# 4x anti-aliasing (offsamples) and a 4096 shadow map are already on; the light is LeRobot's one directional lamp straight down
# plus a strong headlight (ambient 0.3, diffuse 0.6) that lights everything flat from the camera, the floor is a mirror
# (reflectance 0.2: the robot's legs show in it) and every scene geom has MuJoCo's default plastic shine (specular 0.5).
# "natural" = a room with ceiling light: a key lamp from above and a little to the side (the only one with a shadow), a weaker
# fill lamp from the other side, a dim headlight, a matte floor and mild shine. "basic" = the file as built (LeRobot's lamp,
# the look run1 was recorded with). Config image.render_look picks it per run (randomization.ModelVariation); visual only:
# lights and materials never enter the physics. The light randomization (light_scale/tilt/tint) scales and turns every lamp.
RENDER_LOOKS = ("natural", "basic")
# name: (direction, diffuse, specular, castshadow); off in the file, on in "natural". Directions tilt 15-25 deg from vertical,
# as ceiling panels seen from a table; diffuse key + fill + headlight ~ LeRobot's lamp + headlight on the table (~1.0)
LOOK_LIGHTS = {
    "look_key": ((0.25, 0.35, -1.0), 0.7, 0.15, True),
    "look_fill": ((-0.35, -0.25, -1.0), 0.35, 0.0, False),
}
LOOK_LIGHT_POS = (0.4, 0.0, 2.5)  # above the table; a directional lamp's shadow map is centred on it
LOOK_AMBIENT = 0.12  # per lamp: shadowed sides stay grey, not black
LOOK_HEADLIGHT = (0.1, 0.15)  # ambient, diffuse: just enough to see into the shadows, not a flash on the camera
LOOK_FLOOR_REFLECTANCE = 0.0  # a lab floor is matte
LOOK_SPECULAR, LOOK_SHININESS = 0.15, 0.3  # mild satin shine (wood top, synthetic ball) instead of MuJoCo's default 0.5 / 0.5
LOOK_MATERIAL = "look_satin"  # given to every scene geom without a material in "natural"


def object_rest_height(name, on=TABLE_TOP):
    """Height of the object's centre at rest. on: height of the surface it stands on (0 = floor; for the ball: the table top
    its stand stands on)."""
    kind, size, *_ = ALL_OBJECTS[name]
    if kind == "sphere":  # the ball rests on the inner top edge of its stand
        r = size[0]
        return on + STAND_HEIGHT - (r - np.sqrt(r * r - STAND_INNER_RADIUS ** 2)) + r
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
    top.rgba = TABLE_RGBA
    top.friction = [1.0, 0.02, 0.001]
    top.group = HIDDEN_GROUP
    add_table_look(table, table_half)

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
    add_dr_materials(spec)
    add_look(spec)
    hide_from_cameras(spec)
    return spec


# Drawn by every camera before 2026-11-01 but never seen by the real head camera: the LeRobot sim's debug sites (red spheres
# at the centre of mass and feet, green hand markers, grey foot-contact and IMU dots - visible at the robot's feet in the head
# image), and the D455 housing, whose edge crosses the head image as a dark bar once the camera pose randomization
# (+-2.5 cm) moves the camera behind its own front face (found in v2 demo s109_t0004). Sites are markers only and group
# changes do not touch the physics, so every site is hidden (ours, like lh_grasp_site, already sit in group 4).
CAMERA_HOUSING_BODY = "d455_link"


def hide_from_cameras(spec):
    """Move every site and the head camera's own housing into HIDDEN_GROUP (not drawn by any renderer)."""
    for site in spec.sites:
        site.group = max(site.group, HIDDEN_GROUP)
    for geom in spec.body(CAMERA_HOUSING_BODY).geoms:
        geom.group = HIDDEN_GROUP


def add_look(spec):
    """The lamps (off) and the material (unused) of the "natural" RENDER_LOOKS; ModelVariation switches them on. Off in the
    file, so the file itself still renders exactly the "basic" look."""
    for name, (direction, diffuse, specular, shadow) in LOOK_LIGHTS.items():
        light = spec.worldbody.add_light()
        light.name = name
        light.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
        light.pos = list(LOOK_LIGHT_POS)
        light.dir = list(direction)
        light.diffuse = [diffuse] * 3
        light.specular = [specular] * 3
        light.ambient = [LOOK_AMBIENT] * 3
        light.castshadow = shadow
        light.active = False
    material = spec.add_material()
    material.name = LOOK_MATERIAL
    material.specular = LOOK_SPECULAR
    material.shininess = LOOK_SHININESS


def add_table_look(table, half):
    """Visual-only geoms of body "table" (its frame sits at TABLE_TOP / 2): a slab whose upper face is at TABLE_TOP with the
    block's footprint, and four legs from the floor to the slab's underside. No contacts and no mass, so the physics only
    sees the collision block (see TABLE_STYLES)."""
    slab = _add_box(table, "table_slab", [half[0], half[1], TABLE_SLAB / 2],
                    [0.0, 0.0, TABLE_TOP / 2 - TABLE_SLAB / 2], TABLE_RGBA)
    leg_half_z = (TABLE_TOP - TABLE_SLAB) / 2
    corner = np.subtract(half, TABLE_LEG_INSET + TABLE_LEG / 2)
    legs = [_add_box(table, f"table_leg_{i}", [TABLE_LEG / 2, TABLE_LEG / 2, leg_half_z],
                     [sx * corner[0], sy * corner[1], leg_half_z - TABLE_TOP / 2], TABLE_LEG_RGBA)
            for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1)))]
    for geom in (slab, *legs):
        geom.contype = geom.conaffinity = 0
        geom.density = 0.0
        geom.group = VISIBLE_GROUP


def add_dr_materials(spec):
    """One builtin texture and one material per DR_MATERIALS entry, same name; no geom uses them in the file."""
    for name, (builtin, rgb1, rgb2, mark, markrgb, repeat) in DR_MATERIALS.items():
        texture = spec.add_texture()
        texture.name = name
        texture.type = mujoco.mjtTexture.mjTEXTURE_2D
        texture.builtin = getattr(mujoco.mjtBuiltin, f"mjBUILTIN_{builtin.upper()}")
        texture.width = texture.height = TEXTURE_SIZE
        texture.rgb1, texture.rgb2 = list(rgb1), list(rgb2)
        texture.mark = getattr(mujoco.mjtMark, f"mjMARK_{mark.upper()}")
        texture.markrgb = list(markrgb)
        texture.random = TEXTURE_SPECKLE
        material = spec.add_material()
        material.name = name
        material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = name
        material.texuniform = True  # repeat per metre, so a checker square has the same size on the table and the floor
        material.texrepeat = [repeat, repeat]


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
    return geom


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
