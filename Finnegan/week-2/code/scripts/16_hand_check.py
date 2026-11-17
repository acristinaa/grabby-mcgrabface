"""Phase 1c Abnahme — schliessen beide Revo2-Haende unter data.ctrl?

Faehrt alle 12 Hand-Aktuatoren von offen (0) auf zu (ctrlrange-Max), misst die
Bewegung der Fingerspitzen und rendert ein Vorher/Nachher-Bild.

Abnahmekriterium (PLAN.md Phase 1): beide Haende schliessen sichtbar.
Numerisch: jede Fingerspitze bewegt sich > 1 cm.

Normales python reicht (headless). Ausgabe: docs/hands_open_closed.png
"""
import pathlib
import mujoco
import numpy as np
from PIL import Image

SCENE = "models/g1_brainco/scene_table_hands.xml"
SETTLE_STEPS = 1500          # genug Zeit, damit die Positionsregler ankommen
MIN_TIP_TRAVEL = 0.01        # 1 cm

model = mujoco.MjModel.from_xml_path(SCENE)
data = mujoco.MjData(model)

hand_act = [
    i for i in range(model.nu)
    if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or "").startswith(("lh_", "rh_"))
]
# ACHTUNG: Die beiden Herstellermodelle sind NICHT symmetrisch aufgebaut.
# Die rechte Hand hat zusaetzliche `*_tip`-Bodies, die linke endet bei
# `*_distal_link`. Wir messen deshalb an den distalen Gliedern - die gibt es
# in beiden Haenden (je 5, ein Endglied pro Finger).
tip_bodies = [
    i for i in range(model.nbody)
    if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "").endswith("_distal_link")
]
n_left = sum(
    1 for i in tip_bodies
    if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "").startswith("lh_")
)
print(f"{len(hand_act)} Hand-Aktuatoren, {len(tip_bodies)} Endglieder "
      f"({n_left} links / {len(tip_bodies) - n_left} rechts)")


def settle(n=SETTLE_STEPS):
    for _ in range(n):
        mujoco.mj_step(model, data)


def hand_cam(side):
    """Freie Kamera dicht an der jeweiligen Hand."""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                            f"{'lh_left' if side == 'left' else 'rh_right'}_base_link")
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = data.xpos[bid]
    cam.distance = 0.32
    cam.azimuth = 150.0 if side == "left" else 30.0
    cam.elevation = -20.0
    return cam


r = mujoco.Renderer(model, height=480, width=640)

# --- offen ---
data.ctrl[:] = 0.0
settle()
open_pos = data.xpos[tip_bodies].copy()
tiles_open = []
for side in ("left", "right"):
    r.update_scene(data, camera=hand_cam(side))
    tiles_open.append(r.render())

# --- geschlossen ---
for i in hand_act:
    data.ctrl[i] = model.actuator_ctrlrange[i][1]
settle()
closed_pos = data.xpos[tip_bodies].copy()
tiles_closed = []
for side in ("left", "right"):
    r.update_scene(data, camera=hand_cam(side))
    tiles_closed.append(r.render())

# --- Auswertung ---
travel = np.linalg.norm(closed_pos - open_pos, axis=1)
print("\nFingerspitzen-Weg (offen -> zu):")
for bid, d in zip(tip_bodies, travel):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid)
    flag = "ok" if d >= MIN_TIP_TRAVEL else "ZU WENIG"
    print(f"  {name:34s} {d * 100:6.2f} cm  {flag}")
print(f"\nMittel {travel.mean() * 100:.2f} cm | Min {travel.min() * 100:.2f} cm "
      f"| Max {travel.max() * 100:.2f} cm")

out = pathlib.Path("docs"); out.mkdir(exist_ok=True)
grid = np.concatenate([np.concatenate(tiles_open, axis=1),
                       np.concatenate(tiles_closed, axis=1)], axis=0)
Image.fromarray(grid).save(out / "hands_open_closed.png")
print("gespeichert: docs/hands_open_closed.png (oben offen, unten zu; links/rechts)")

if travel.min() < MIN_TIP_TRAVEL:
    raise SystemExit(f"FEHLGESCHLAGEN: kleinster Weg {travel.min() * 100:.2f} cm "
                     f"< {MIN_TIP_TRAVEL * 100:.0f} cm")
print("\nABNAHME BESTANDEN: beide Haende schliessen unter data.ctrl.")
