"""Phase 1a+1b — Tischszene mit Kameras bauen.

Baut aus dem Menagerie-G1 eine Manipulationsszene:
  - Basis fixiert (Oberkoerper-Manipulation, keine Locomotion -> siehe PLAN.md)
  - Kopf- und Handgelenkkameras (das Menagerie-Modell hat ncam=0!)
  - Tisch + rote Flasche + Distraktoren (gruene Tasse, blauer Wuerfel)

DOF-unabhaengig: laeuft mit 23- wie 29-DOF-Armen.
Ausgabe: models/g1_brainco/scene_table.xml
"""
import pathlib
import shutil
import numpy as np
import mujoco
import mujoco_menagerie as mm

OUT = pathlib.Path("models/g1_brainco"); OUT.mkdir(parents=True, exist_ok=True)

PELVIS_H  = 0.793     # Standhoehe des Beckens
TABLE_X   = 0.55      # Tisch vor dem Roboter
TABLE_TOP = 0.74


def look_quat(forward, up=(0.0, 0.0, 1.0)):
    """Quaternion fuer eine Kamera, die entlang `forward` blickt.
    MuJoCo-Konvention: Kamera schaut entlang -Z, +Y ist oben."""
    f = np.array(forward, float); f /= np.linalg.norm(f)
    u = np.array(up, float)
    z = -f                                  # -Z = Blickrichtung
    x = np.cross(u, z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    mat = np.array([x, y, z]).T.flatten()   # Spalten = Achsen
    q = np.zeros(4); mujoco.mju_mat2Quat(q, mat)
    return q


spec = mm.get("unitree_g1").spec("g1")

# --- 1. Basis fixieren: freejoint entfernen, Becken auf Standhoehe ---
pelvis = spec.body("pelvis")
for j in list(pelvis.joints):
    if j.type == mujoco.mjtJoint.mjJNT_FREE:
        spec.delete(j)
pelvis.pos = [0.0, 0.0, PELVIS_H]

# --- 2. Kameras (Menagerie-G1 hat KEINE) ---
torso = spec.body("torso_link")
cam = torso.add_camera()
cam.name = "head_cam"
cam.pos  = [0.16, 0.0, 0.40]                       # vor dem Kopfmesh, sonst schwarzer Rand
cam.quat = look_quat([1.0, 0.0, -0.70])            # nach vorn-unten auf die Tischplatte
cam.fovy = 58.0

for side in ("left", "right"):
    wc = spec.body(f"{side}_wrist_yaw_link").add_camera()
    wc.name = f"{side}_wrist_cam"
    wc.pos  = [0.06, 0.0, -0.02]
    wc.quat = look_quat([1.0, 0.0, -0.30])
    wc.fovy = 70.0

# --- 3. Tisch (statisch, kein Gelenk) ---
world = spec.body("world")
table = world.add_body()
table.name = "table"
table.pos  = [TABLE_X, 0.0, TABLE_TOP / 2.0]
g = table.add_geom()
g.name = "table_top"; g.type = mujoco.mjtGeom.mjGEOM_BOX
g.size = [0.35, 0.45, TABLE_TOP / 2.0]
g.rgba = [0.55, 0.42, 0.30, 1.0]
g.friction = [1.0, 0.02, 0.001]

# --- 4. Greifbare Objekte (mit freejoint) ---
OBJECTS = [
    # name,          typ,       size,               rgba,                    pos (x,y)
    ("red_bottle",   "cyl",  [0.033, 0.105], [0.85, 0.10, 0.10, 1.0], ( 0.02,  0.00)),
    ("green_cup",    "cyl",  [0.040, 0.055], [0.15, 0.65, 0.25, 1.0], (-0.02,  0.20)),
    ("blue_block",   "box",  [0.030, 0.030, 0.030], [0.15, 0.30, 0.80, 1.0], (0.04, -0.20)),
]
for name, kind, size, rgba, (dx, dy) in OBJECTS:
    b = world.add_body()
    b.name = name
    half_h = size[1] if kind == "cyl" else size[2]
    b.pos = [TABLE_X + dx, dy, TABLE_TOP + half_h + 0.002]
    b.add_freejoint()
    gg = b.add_geom()
    gg.name = f"{name}_geom"
    gg.type = mujoco.mjtGeom.mjGEOM_CYLINDER if kind == "cyl" else mujoco.mjtGeom.mjGEOM_BOX
    gg.size = size + [0.0] if kind == "cyl" else size
    gg.rgba = rgba
    gg.mass = 0.35 if kind == "cyl" else 0.20
    gg.friction = [1.2, 0.05, 0.001]

# --- 5. Boden + Licht ---
fl = world.add_geom()
fl.name = "floor"; fl.type = mujoco.mjtGeom.mjGEOM_PLANE
fl.size = [5.0, 5.0, 0.05]; fl.rgba = [0.35, 0.38, 0.42, 1.0]
lt = world.add_light()
lt.pos = [0.5, 0.0, 3.0]; lt.dir = [0.0, 0.0, -1.0]
lt.diffuse = [0.8, 0.8, 0.8]; lt.specular = [0.2, 0.2, 0.2]

model = spec.compile()
print(f"kompiliert: nq={model.nq} nv={model.nv} nu={model.nu} ncam={model.ncam}")
for i in range(model.ncam):
    print("  cam:", mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i))

# --- 6. Assets mitkopieren, damit das Modell eigenstaendig ist ---
src_assets = pathlib.Path(mm.get("unitree_g1").path()) / "assets"
dst_assets = OUT / "assets"
if src_assets.is_dir():
    shutil.copytree(src_assets, dst_assets, dirs_exist_ok=True)
    print(f"Assets kopiert: {len(list(dst_assets.iterdir()))} Dateien -> {dst_assets}")
else:
    print("WARNUNG: Asset-Ordner nicht gefunden:", src_assets)

xml_path = OUT / "scene_table.xml"
xml_path.write_text(spec.to_xml())
print("geschrieben:", xml_path)

# Gegenprobe: laesst sich die Datei eigenstaendig laden?
check = mujoco.MjModel.from_xml_path(str(xml_path))
print(f"Reload OK: nq={check.nq} nu={check.nu} ncam={check.ncam}")
