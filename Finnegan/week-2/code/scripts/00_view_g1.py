"""Phase 0 — G1 im Viewer. IMMER mit mjpython starten:  mjpython scripts/00_view_g1.py"""
import time
import numpy as np
import mujoco
import mujoco.viewer
import mujoco_menagerie as mm

model = mm.load("unitree_g1")   # Szene inkl. Boden + Licht
data = mujoco.MjData(model)

print("nq:", model.nq, "nv:", model.nv, "nu:", model.nu, "ncam:", model.ncam)

with mujoco.viewer.launch_passive(model, data) as viewer:
    t0 = time.time()
    while viewer.is_running():
        t = time.time() - t0
        data.ctrl[:] = 0.2 * np.sin(2.0 * t)   # nur Demo, keine Policy
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.002)
