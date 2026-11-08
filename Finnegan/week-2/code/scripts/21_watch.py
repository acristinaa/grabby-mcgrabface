"""Szene live anschauen.  START:  mjpython scripts/21_watch.py [szene]

Tastatur im Viewer:
  Leertaste = Pause/Weiter |  Strg+P = Kontaktkraefte |  Tab = Menue
  Doppelklick = Koerper auswaehlen |  Strg+Ziehen = Koerper schubsen
"""
import sys, time
import mujoco, mujoco.viewer

path = sys.argv[1] if len(sys.argv) > 1 else "models/g1_brainco/scene_table.xml"
model = mujoco.MjModel.from_xml_path(path)
data = mujoco.MjData(model)

print(f"Szene: {path}")
print(f"nq={model.nq} nu={model.nu} ncam={model.ncam}")
for i in range(model.ncam):
    print("  Kamera:", mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i))
print("\nIm Viewer: Tab -> Kamera wechseln. Fenster schliessen = Ende.")

with mujoco.viewer.launch_passive(model, data) as v:
    while v.is_running():
        step_start = time.time()
        mujoco.mj_step(model, data)
        v.sync()
        rest = model.opt.timestep - (time.time() - step_start)
        if rest > 0:
            time.sleep(rest)
