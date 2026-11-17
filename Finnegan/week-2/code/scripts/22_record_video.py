"""Video aufnehmen - damit du Ergebnisse anschauen kannst OHNE offenes Fenster.
START:  python scripts/22_record_video.py            (kein mjpython noetig!)
Ausgabe: docs/videos/<name>.mp4
"""
import pathlib
import numpy as np
import mujoco
import imageio.v2 as imageio

OUT = pathlib.Path("docs/videos"); OUT.mkdir(parents=True, exist_ok=True)
SECONDS, FPS = 6.0, 30


def record(name, xml, camera=None, ctrl_fn=None, height=480, width=640):
    model = mujoco.MjModel.from_xml_path(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    r = mujoco.Renderer(model, height=height, width=width)

    frames, n_steps = [], int(SECONDS / model.opt.timestep)
    every = max(1, int((1.0 / FPS) / model.opt.timestep))
    for i in range(n_steps):
        if ctrl_fn is not None:
            ctrl_fn(model, data, i * model.opt.timestep)
        mujoco.mj_step(model, data)
        if i % every == 0:
            r.update_scene(data, camera=camera) if camera else r.update_scene(data)
            frames.append(r.render())

    path = OUT / f"{name}.mp4"
    imageio.mimsave(path, frames, fps=FPS, quality=8)
    print(f"  {path}  ({len(frames)} Bilder, {SECONDS:.0f}s)")
    return path


if __name__ == "__main__":
    print("Nehme auf:")
    TABLE = "models/g1_brainco/scene_table.xml"

    # 1) Tischszene, Uebersicht - Roboter haelt still (fixe Basis)
    record("01_tisch_uebersicht", TABLE)

    # 2) Aus Sicht des Roboters - das sieht der VLM
    record("02_kopfkamera", TABLE, camera="head_cam")

    # 3) Arme bewegen sich - zeigt, dass die Aktuatoren wirken
    def wiggle(model, data, t):
        data.ctrl[:] = 0.0
        for j in (15, 16, 18, 22, 23, 25):      # Schulter + Ellbogen
            if j < model.nu:
                data.ctrl[j] = 0.4 * np.sin(1.5 * t)
    record("03_arme_bewegen", TABLE, ctrl_fn=wiggle)
