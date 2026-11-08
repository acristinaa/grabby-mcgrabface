"""Phase 1 check — was sieht der Roboter? Rendert alle Kameras.
Normales python reicht (headless)."""
import pathlib
import mujoco
import numpy as np
from PIL import Image

model = mujoco.MjModel.from_xml_path("models/g1_brainco/scene_table.xml")
data  = mujoco.MjData(model)
mujoco.mj_forward(model, data)

out = pathlib.Path("docs"); out.mkdir(exist_ok=True)
r = mujoco.Renderer(model, height=480, width=640)
tiles = []
for i in range(model.ncam):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i)
    r.update_scene(data, camera=name)
    rgb = r.render()
    Image.fromarray(rgb).save(out / f"cam_{name}.png")
    tiles.append(rgb)
    print(f"{name:18s} mean={rgb.mean():6.2f}")

# Uebersicht: freie Kamera
r.update_scene(data)
Image.fromarray(r.render()).save(out / "scene_overview.png")

strip = np.concatenate(tiles, axis=1)
Image.fromarray(strip).save(out / "cams_strip.png")
print("gespeichert: docs/scene_overview.png, docs/cams_strip.png")
