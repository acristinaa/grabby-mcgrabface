"""Stufe 1.1 - Greifszene auf dem Laufmodell bauen und pruefen.

Baut assets/scene_29dof_brainco_table.xml in ~/dev/g1-ext/g1_brainco_sim (Rezept: g1sim/table_scene.py) und prueft:
  1. laedt von der Platte: nu 41, drei Kameras, grasp_site an beiden Haenden, Flasche 0,5 kg
  2. 2 s Halten der Startpose mit TableEnv (PD wie am Roboter): Objekte bleiben liegen (< 5 mm), Arme sacken nicht ab
     (< 0,1 rad), zu keinem Zeitpunkt Kontakt Roboter <-> Tisch/Objekte, stabil
  3. Kopfmodul: haelt nach den 2 s seine Startneigung, folgt einem neuen Sollwinkel binnen 1 s (je <= 0,01 rad)
  4. 10 Starts mit zufaellig verschobenen Objekten: kein Kontakt Roboter <-> Tisch/Objekte
  5. Kamerabilder aller drei Kameras (Kopf = D455) + Uebersicht -> docs/stufe1_1_greifszene.png
Braucht vorher scripts/51_build_brainco_sim.py. Umgebung ~/dev/g1-ext:
    ~/dev/g1-ext/.venv/bin/python scripts/53_build_table_scene.py
Ausgabe: runs/stufe1_1_greifszene.json, docs/stufe1_1_greifszene.png
"""
import json
import pathlib
import sys
import warnings

import cv2
import mujoco
import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim import action_space, payload, table_env, table_scene  # noqa: E402

OUT = REPO / "runs" / "stufe1_1_greifszene.json"
IMAGE = REPO / "docs" / "stufe1_1_greifszene.png"
HOLD_S = 2.0
MAX_OBJECT_DRIFT = 0.005
MAX_ARM_SAG = 0.1  # rad; das alte Halten mit "Sollwert = aktuelle Stellung" sackte in 2 s bis 0,38 rad ab
RANDOM_STARTS = 10
HEAD_TEST_POSE = {"pan": 0.4, "tilt": 0.5}  # rad
MAX_HEAD_ERROR = 0.01  # rad

warnings.filterwarnings("ignore", message=".*parent has")


def main():
    model = table_scene.write()
    names = lambda obj, n: {mujoco.mj_id2name(model, obj, i) for i in range(n)}  # noqa: E731
    env = table_env.TableEnv()
    env.reset()
    start = {name: env.object_position(name) for name in table_scene.OBJECTS}
    touched = set(env.robot_contacts())
    for _ in range(int(HOLD_S * table_env.CONTROL_HZ)):
        env.apply(action_space.READY_POSE)  # fester Sollwert
        touched.update(env.robot_contacts())
    drift = {name: float(np.linalg.norm(env.object_position(name) - start[name])) for name in table_scene.OBJECTS}
    arm_sag = float(np.abs(env.state()[:14] - action_space.READY_POSE[:14]).max())

    m, d = env.model, env.data
    stable = bool(np.isfinite(d.qpos).all()) and d.warning[mujoco.mjtWarning.mjWARN_BADQACC].number == 0
    obs = env.observation()
    overview = mujoco.Renderer(m, height=448, width=448)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.35, 0.0, 0.85]
    cam.distance, cam.azimuth, cam.elevation = 1.6, 210.0, -25.0
    overview.update_scene(d, camera=cam)
    tiles = [cv2.resize(overview.render(), (448, 448))] + [cv2.resize(obs[k], (448, 448)) for k in
                                                          ("head_image", "left_wrist_image", "right_wrist_image")]
    IMAGE.parent.mkdir(exist_ok=True)
    cv2.imwrite(str(IMAGE), np.concatenate([np.concatenate(tiles[:2], 1), np.concatenate(tiles[2:], 1)], 0)[:, :, ::-1])
    overview.close()

    head_start = payload.head_angles(m, d)
    payload.set_head(m, **HEAD_TEST_POSE)
    for _ in range(table_env.CONTROL_HZ):
        env.apply(action_space.READY_POSE)
    head_moved = payload.head_angles(m, d)
    payload.set_head(m, **table_scene.HEAD_POSE)
    head_error = lambda got, want: max(abs(got[k] - want[k]) for k in want)  # noqa: E731

    random_touched = set()
    for _ in range(RANDOM_STARTS):
        env.reset(randomize=True)
        random_touched.update(env.robot_contacts())

    checks = {
        "nu_41": m.nu == 41,
        "drei_kameras": set(table_scene.CAMERAS) <= names(mujoco.mjtObj.mjOBJ_CAMERA, m.ncam),
        "grasp_site_beide_haende": {"lh_grasp_site", "rh_grasp_site"} <= names(mujoco.mjtObj.mjOBJ_SITE, m.nsite),
        # aus der Szenen-Datei: env.model traegt nach den Zufallsstarts die variierte Masse (Stufe 1.5b)
        "flasche_0_5_kg": abs(model.body_mass[model.body("red_bottle").id] - 0.5) < 1e-6,
        "objekte_bleiben_liegen": max(drift.values()) < MAX_OBJECT_DRIFT,
        "arme_sacken_nicht_ab": arm_sag < MAX_ARM_SAG,
        "kein_kontakt_roboter_tisch_objekte": not touched,
        "zufallsstarts_ohne_kontakt": not random_touched,
        "kopf_haelt_stellung": head_error(head_start, table_scene.HEAD_POSE) <= MAX_HEAD_ERROR,
        "kopf_folgt_sollwert": head_error(head_moved, HEAD_TEST_POSE) <= MAX_HEAD_ERROR,
        "stabil": stable,
        "bilder_nicht_leer": all(obs[k].std() > 5 for k in ("head_image", "left_wrist_image", "right_wrist_image")),
    }
    result = {
        "nq": m.nq, "nv": m.nv, "nu": m.nu,
        "ready_pose": action_space.READY_POSE.round(3).tolist(),
        "object_drift_mm": {k: round(v * 1000, 2) for k, v in drift.items()},
        "arm_sag_rad": round(arm_sag, 4),
        "head_rad": {"start": head_start, "target": HEAD_TEST_POSE, "after_1s": head_moved},
        "robot_contacts": sorted(touched),
        "random_start_contacts": sorted(random_touched),
        "state": obs["state"].round(3).tolist(),
        "checks": {k: bool(v) for k, v in checks.items()},
        "passed": bool(all(checks.values())),
    }
    env.close()
    OUT.parent.mkdir(exist_ok=True)
    OUT.write_text(json.dumps(result, indent=2, ensure_ascii=False))
    print(json.dumps({k: result[k] for k in ("nq", "nu", "object_drift_mm", "arm_sag_rad", "robot_contacts",
                                             "random_start_contacts")}, ensure_ascii=False))
    for k, v in checks.items():
        print(f"  {'ok  ' if v else 'FAIL'} {k}")
    print(f"gespeichert: {OUT.relative_to(REPO)}, {IMAGE.relative_to(REPO)}")
    print("ABNAHME BESTANDEN" if result["passed"] else "ABNAHME NICHT BESTANDEN")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
