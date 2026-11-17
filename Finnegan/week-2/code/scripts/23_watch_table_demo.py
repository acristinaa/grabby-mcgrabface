"""Die Projekt-Szene live anschauen - zum Anschauen und Abfilmen.
START:  mjpython scripts/23_watch_table_demo.py

Im Fenster:
  Tab            = Kamera wechseln (frei -> head_cam -> wrist_cams)
  Leertaste      = Pause / Weiter
  Maus ziehen    = drehen,  Scrollen = zoomen
  Strg + ziehen  = Objekte anschubsen
"""
import time
import numpy as np
import mujoco, mujoco.viewer

model = mujoco.MjModel.from_xml_path("models/g1_brainco/scene_table.xml")
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

# Gelenke, die wir sanft bewegen (Schultern + Ellbogen), damit man sieht dass er lebt
ARMS = [15, 16, 17, 18, 22, 23, 24, 25]

print("=" * 62)
print("PROJEKT-SZENE: G1 am Tisch")
print("=" * 62)
print(f"Gelenke mit Motor : {model.nu}")
print(f"Kameras           : {model.ncam}")
for i in range(model.ncam):
    print("   -", mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i))
print(f"Objekte auf dem Tisch:")
for n in ("red_bottle", "green_cup", "blue_block"):
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
    print(f"   - {n:12s} bei {np.round(data.xpos[bid], 3)}")
print("\n>>> TAB druecken, um durch die Kameras zu schalten <<<")
print(">>> 'head_cam' ist das, was der VLM sieht <<<\n")

with mujoco.viewer.launch_passive(model, data) as v:
    t0 = time.time()
    while v.is_running():
        step = time.time()
        t = time.time() - t0
        data.ctrl[:] = 0.0
        for j in ARMS:                      # langsame, ruhige Bewegung
            data.ctrl[j] = 0.30 * np.sin(0.6 * t + j)
        mujoco.mj_step(model, data)
        v.sync()
        rest = model.opt.timestep - (time.time() - step)
        if rest > 0:
            time.sleep(rest)
