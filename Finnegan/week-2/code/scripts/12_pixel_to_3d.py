"""Phase 2 Kernstueck — Pixel -> 3D-Weltkoordinate.

Das ist die Bruecke zwischen "VLM sagt: die Flasche ist DA im Bild"
und "Arm, fahr zu diesem Punkt". Reine Geometrie, kein Training.
"""
import sys
import numpy as np
import mujoco

W, H = 640, 480
CAM = "head_cam"


def deproject(model, data, renderer, cam_name, u, v):
    """Bildpixel (u,v) -> 3D-Punkt in Weltkoordinaten, via Tiefenpuffer."""
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)

    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera=cam_name)
    depth = renderer.render()
    renderer.disable_depth_rendering()

    d = float(depth[int(v), int(u)])          # Tiefe entlang der Blickachse

    fovy = float(model.cam_fovy[cid])
    f = (H / 2.0) / np.tan(np.deg2rad(fovy) / 2.0)   # Brennweite in Pixeln

    x_c = (u - W / 2.0) / f
    y_c = -(v - H / 2.0) / f                   # Bild-y zeigt nach unten
    p_cam = np.array([x_c * d, y_c * d, -d])   # Kamera blickt entlang -Z

    cam_pos = data.cam_xpos[cid]
    cam_mat = data.cam_xmat[cid].reshape(3, 3)
    return cam_pos + cam_mat @ p_cam, d


model = mujoco.MjModel.from_xml_path("models/g1_brainco/scene_table.xml")
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
renderer = mujoco.Renderer(model, height=H, width=W)

# --- "VLM-Ausgabe": Pixelkoordinaten der Objekte, abgelesen aus dem Kamerabild ---
VLM_OUTPUT = {
    "red_bottle": (322, 300),
    "green_cup":  (166, 345),
    "blue_block": (462, 330),
}

print(f"{'Objekt':<12} {'VLM-Pixel':<12} {'geschaetzt (x,y,z)':<26} {'wahr (x,y,z)':<26} {'Fehler'}")
print("-" * 92)
worst = 0.0
for name, (u, v) in VLM_OUTPUT.items():
    est, d = deproject(model, data, renderer, CAM, u, v)
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    true = data.xpos[bid]
    err = float(np.linalg.norm(est - true))
    worst = max(worst, err)
    print(f"{name:<12} {str((u,v)):<12} "
          f"({est[0]:.3f},{est[1]:.3f},{est[2]:.3f})      "
          f"({true[0]:.3f},{true[1]:.3f},{true[2]:.3f})      {err*100:.1f} cm")

print("-" * 92)
print(f"groesster Fehler: {worst*100:.1f} cm")
print("\nHinweis: der Punkt liegt auf der VORDERSEITE des Objekts (Tiefenpuffer sieht nur")
print("die sichtbare Oberflaeche). Fuer den Griff muss der Radius addiert werden.")
sys.exit(0 if worst < 0.15 else 1)
