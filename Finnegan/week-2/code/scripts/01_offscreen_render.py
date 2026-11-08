"""Phase 0 check — headless Rendering (Grundlage fuer den VLM-Planner in Phase 2).
Laeuft mit normalem python, KEIN mjpython noetig."""
import mujoco, mujoco_menagerie as mm, numpy as np

model = mm.load("unitree_g1")
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

renderer = mujoco.Renderer(model, height=480, width=640)
renderer.update_scene(data)                 # freie Kamera
rgb = renderer.render()
print("RGB:", rgb.shape, rgb.dtype, "mean:", round(float(rgb.mean()), 2))

renderer.enable_depth_rendering()
renderer.update_scene(data)
depth = renderer.render()
print("Depth:", depth.shape, "min:", round(float(depth.min()),3), "max:", round(float(depth.max()),3))

try:
    from PIL import Image
    Image.fromarray(rgb).save("docs/render_check.png")
    print("saved docs/render_check.png")
except ImportError:
    np.save("docs/render_check.npy", rgb)
    print("PIL fehlt -> docs/render_check.npy gespeichert")
