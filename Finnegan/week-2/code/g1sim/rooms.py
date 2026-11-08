"""Lernpfad L2 - Zwei-Raum-Szene fuer die Laufwelt: Raum A (Start), Trennwand mit Tuer, Raum B mit Tisch.

Die Geometrie steht nur hier. Die Sim bekommt sie als Waende (write), die Navigation als Belegungskarte (grid) - so koennen
Karte und Welt nicht auseinanderlaufen. Spaeter (L7) baut der Roboter die Karte selbst aus LiDAR-Strahlen.
Der Roboter startet bei (0, 0) mit Blick nach +x, also in Raum A. Der Tisch in Raum B ist hier nur Hindernis; Flasche und
Greifen kommen in L3.
Grundlage ist das gebaute Laufmodell (scripts/51_build_brainco_sim.py); die Szene liegt daneben unter SIM_DIR/SCENE_FILE.
"""
import os

import mujoco

from g1sim import MANIFEST, SIM_DIR, occupancy
from g1sim import SCENE_FILE as WALK_SCENE

SCENE_FILE = "assets/scene_29dof_brainco_rooms.xml"

ROOM_A = ((-1.5, 2.5), (-2.0, 2.0))  # Innenmasse (x, y) in m
ROOM_B = ((2.5, 6.5), (-2.0, 2.0))
DOOR_Y = (0.4, 1.6)                  # Tueroeffnung in der Trennwand bei x = 2,5: 1,2 m breit, nicht mittig
WALL_HALF = 0.05                     # Wandstaerke 10 cm
WALL_HEIGHT = 1.0
TABLE = ((5.0, -1.0), (0.30, 0.45), 0.74)  # Mitte, halbe Kanten, Hoehe - wie der Greiftisch aus Stufe 1.1
MARGIN = 0.3                         # Karte reicht so weit ueber die Aussenwaende hinaus


def boxes():
    """Alle Hindernisse: Name -> ((Mitte x, y), (halbe Kante x, y), Hoehe). Namen beginnen mit wall_ oder obstacle_."""
    (ax0, _), (y0, y1) = ROOM_A
    (_, bx1), _ = ROOM_B
    split = ROOM_A[0][1]
    w = WALL_HALF
    x_mid, x_half = (ax0 + bx1) / 2.0, (bx1 - ax0) / 2.0 + 2 * w
    return {
        "wall_south": ((x_mid, y0 - w), (x_half, w), WALL_HEIGHT),
        "wall_north": ((x_mid, y1 + w), (x_half, w), WALL_HEIGHT),
        "wall_west": ((ax0 - w, 0.0), (w, (y1 - y0) / 2.0), WALL_HEIGHT),
        "wall_east": ((bx1 + w, 0.0), (w, (y1 - y0) / 2.0), WALL_HEIGHT),
        "wall_split_south": ((split, (y0 + DOOR_Y[0]) / 2.0), (w, (DOOR_Y[0] - y0) / 2.0), WALL_HEIGHT),
        "wall_split_north": ((split, (DOOR_Y[1] + y1) / 2.0), (w, (y1 - DOOR_Y[1]) / 2.0), WALL_HEIGHT),
        "obstacle_table": TABLE,
    }


def grid(resolution=occupancy.RESOLUTION):
    """Belegungskarte der Szene."""
    (ax0, _), (y0, y1) = ROOM_A
    (_, bx1), _ = ROOM_B
    g = occupancy.Grid(((ax0 - MARGIN, bx1 + MARGIN), (y0 - MARGIN, y1 + MARGIN)), resolution)
    for center, half, _ in boxes().values():
        g.add_box(center, half)
    return g


def room_of(x, y):
    """'A', 'B' oder None (Wand, Tuer, ausserhalb)."""
    for name, ((x0, x1), (y0, y1)) in (("A", ROOM_A), ("B", ROOM_B)):
        if x0 < x < x1 and y0 < y < y1:
            return name
    return None


def write(sim_dir=SIM_DIR):
    """Laufmodell laden, Waende und Tisch ergaenzen, als SCENE_FILE daneben schreiben. Rueckgabe: SCENE_FILE (relativ)."""
    if not (sim_dir / MANIFEST).is_file():
        raise FileNotFoundError(f"{sim_dir} ist nicht gebaut - zuerst scripts/51_build_brainco_sim.py")
    spec = mujoco.MjSpec.from_file(str(sim_dir / WALK_SCENE))
    for name, ((cx, cy), (hx, hy), height) in boxes().items():
        geom = spec.worldbody.add_geom()
        geom.name = name
        geom.type = mujoco.mjtGeom.mjGEOM_BOX
        geom.pos = [cx, cy, height / 2.0]
        geom.size = [hx, hy, height / 2.0]
        geom.rgba = [0.55, 0.42, 0.30, 1.0] if name.startswith("obstacle_") else [0.85, 0.85, 0.82, 1.0]
    spec.compile()
    path = sim_dir / SCENE_FILE
    tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")  # erst eigene Datei, dann umbenennen (wie table_scene.write)
    tmp.write_text(spec.to_xml())
    os.replace(tmp, path)
    return SCENE_FILE
