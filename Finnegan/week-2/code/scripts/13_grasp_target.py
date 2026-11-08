"""Phase 2 — von der VLM-Bounding-Box zum Greifpunkt.

Verbesserung gegenueber 12_pixel_to_3d.py: der Tiefenpuffer sieht nur die
VORDERSEITE. Aus der Boxbreite schaetzen wir den Radius und schieben den Punkt
entlang des Sehstrahls zur Objektmitte. Kein Training - reine Geometrie.
"""
import numpy as np
import mujoco

W, H, CAM = 640, 480, "head_cam"

# --- "VLM-Ausgabe": Bounding Boxes, aus dem Kamerabild abgelesen ---
VLM_BOXES = {                 # x0,  y0,  x1,  y1
    "red_bottle": (293, 222, 352, 380),
    "green_cup":  (124, 305, 210, 395),
    "blue_block": (432, 300, 492, 365),
}

model = mujoco.MjModel.from_xml_path("models/g1_brainco/scene_table.xml")
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
r = mujoco.Renderer(model, height=H, width=W)
cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, CAM)

r.enable_depth_rendering(); r.update_scene(data, camera=CAM)
depth = r.render(); r.disable_depth_rendering()

fovy = float(model.cam_fovy[cid])
f = (H / 2.0) / np.tan(np.deg2rad(fovy) / 2.0)
cam_pos = data.cam_xpos[cid]
cam_mat = data.cam_xmat[cid].reshape(3, 3)

print(f"Brennweite f = {f:.1f} px   (fovy={fovy}deg)\n")
print(f"{'Objekt':<12} {'Radius gesch.':<14} {'Greifpunkt (x,y,z)':<26} {'wahr':<26} {'Fehler'}")
print("-" * 92)

errs = []
for name, (x0, y0, x1, y1) in VLM_BOXES.items():
    u, v = (x0 + x1) / 2.0, (y0 + y1) / 2.0

    # robuste Tiefe: Median ueber ein kleines Fenster in der Boxmitte
    patch = depth[int(v)-4:int(v)+5, int(u)-4:int(u)+5]
    d = float(np.median(patch[patch > 0]))

    # Sehstrahl durch den Boxmittelpunkt
    ray = np.array([(u - W/2)/f, -(v - H/2)/f, -1.0])
    ray /= np.linalg.norm(ray)

    # Radius aus der Boxbreite: halbe Breite in Pixeln -> Meter bei Tiefe d
    radius = ((x1 - x0) / 2.0) / f * d

    # Vorderflaeche + Radius entlang des Strahls = Mitte
    p_cam = ray * (d / abs(ray[2])) + ray * radius
    est = cam_pos + cam_mat @ p_cam

    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    true = data.xpos[bid]
    err = float(np.linalg.norm(est - true)); errs.append(err)
    print(f"{name:<12} {radius*100:>6.1f} cm      "
          f"({est[0]:.3f},{est[1]:.3f},{est[2]:.3f})      "
          f"({true[0]:.3f},{true[1]:.3f},{true[2]:.3f})      {err*100:.1f} cm")

print("-" * 92)
print(f"mittlerer Fehler: {np.mean(errs)*100:.1f} cm   max: {np.max(errs)*100:.1f} cm")
