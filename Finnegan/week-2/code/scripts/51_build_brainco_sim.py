"""Stufe 0.3b - LeRobot-G1-Sim mit BrainCo-Revo2 bauen und pruefen.

Baut ~/dev/g1-ext/g1_brainco_sim (Rezept: g1sim/build.py) und prueft das Modell ohne DDS und ohne Controller:
  1. laedt von der Platte: nu = 29 Koerpermotoren + 12 Handmotoren = 41, alle als Drehmoment-<motor>
  2. die LeRobot-Sim findet mit ihrer Namensregel genau 29 Koerpergelenke
  3. Montage der Handbasis wie in Unitrees offiziellem G1+BrainCo-URDF (<= 1 mm, <= 0,5 Grad)
  4. in Nullstellung beruehrt keine Hand den uebrigen Roboter
  5. jeder Handmotor einzeln mit dem PD-Regler der Sim (4 ms, Euler-Integrator): stabil, zu >= 90 %, auf <= 5 %,
     Ueberschwingen <= 10 %
  6. alle Motoren gleichzeitig zu: jede Fingerspitze bewegt sich > 1 cm. Der Schliessgrad wird hier nur berichtet -
     der Daumen darf am schon geschlossenen Zeigefinger anstossen, das waere an der echten Hand genauso
  7. Anbauteile (Rucksack, D455-Kopfmodul): Lage wie im Unitree-URDF, nachgerechnet gegen LeRobots Kopf-Mesh
     (<= 0,1 mm, <= 0,01 Grad), Masse +2,59 kg, im ganzen Schwenkbereich des Kopfes kein Kontakt zum Roboter,
     head_camera sitzt an der D455

Laeuft in der Umgebung ~/dev/g1-ext, braucht kein DDS:
    ~/dev/g1-ext/.venv/bin/python scripts/51_build_brainco_sim.py
Ausgabe: runs/stufe0_3b_brainco_sim.json, docs/stufe0_3b_brainco_haende.png
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

from dashboard import scene  # noqa: E402
from g1sim import SIM_DIR, brainco, build, payload  # noqa: E402

OUT = REPO / "runs" / "stufe0_3b_brainco_sim.json"
IMAGE = REPO / "docs" / "stufe0_3b_brainco_haende.png"

SIM_DT = 0.004  # SIMULATE_DT der LeRobot-Sim
BODY_RULE = ("hip", "knee", "ankle", "waist", "shoulder", "elbow", "wrist")  # Namensregel aus sim/base_sim.py
MOVE_S = 2.5  # Sim-Zeit je Bewegung
MAX_MOUNT_MM, MAX_MOUNT_DEG = 1.0, 0.5
MIN_CLOSED = 0.9
MAX_OPEN = 0.05
MAX_OVERSHOOT = 0.10
MIN_TIP_TRAVEL_CM = 1.0
MAX_PAYLOAD_MM, MAX_PAYLOAD_DEG = 0.1, 0.01

# Unitree-URDF, unabhaengig von payload.xml abgeschrieben: Kind -> (Eltern, xyz, rpy). g1_comp.urdf (Commit 7c40519),
# Rucksack aus robots/g1_description/README.md (Commit d6f13aa).
# head_link = Kopf-Mesh, bei LeRobot ein Geom im torso_link - der Bezug fuer die Messung.
URDF_JOINTS = {
    "head_link": ("torso_link", (0.0039635, 0.0, -0.044), (0.0, 0.0, 0.0)),
    "backpack_link": ("torso_link", (-0.094605, -0.0005, 0.165324), (0.0, 0.0, 0.0)),
    "head_servo_link": ("torso_link", (0.0039635, 0.0, -0.047), (0.0, 0.0, 0.0)),
    "xl330_link": ("head_servo_link", (0.030518, 0.0, 0.52486), (0.0, 0.039968, 0.0)),
    "d455_link": ("xl330_link", (0.0295, 0.0, 0.013), (0.0, 0.0, 0.0)),
}
URDF_MASS_KG = {"backpack_link": 1.9, "head_servo_link": 0.54796, "xl330_link": 0.047, "d455_link": 0.096}

BODY = mujoco.mjtObj.mjOBJ_BODY
MESH = mujoco.mjtObj.mjOBJ_MESH
ACT = mujoco.mjtObj.mjOBJ_ACTUATOR
JNT = mujoco.mjtObj.mjOBJ_JOINT

# Die Handmodelle bringen eigene <option>-Werte mit, beim Anbauen gelten die des G1 - erwuenscht
warnings.filterwarnings("ignore", message=".*parent has")


def name(model, obj, i):
    return mujoco.mj_id2name(model, obj, i) or ""


def subtree(model, root):
    """Body-IDs eines Teilbaums (MuJoCo speichert Eltern vor Kindern)."""
    ids = {root}
    for b in range(root + 1, model.nbody):
        if model.body_parentid[b] in ids:
            ids.add(b)
    return ids


def check_actuators(model):
    hand = [i for i in range(model.nu) if name(model, ACT, i).startswith(build.HAND_PREFIXES)]
    torque_motor = [
        model.actuator_gaintype[i] == mujoco.mjtGain.mjGAIN_FIXED
        and model.actuator_biastype[i] == mujoco.mjtBias.mjBIAS_NONE
        and model.actuator_gainprm[i, 0] == 1.0
        and bool(model.actuator_ctrllimited[i])
        for i in range(model.nu)
    ]
    return {
        "nu": model.nu,
        "body_motors": model.nu - len(hand),
        "hand_motors": len(hand),
        "all_torque_motors": all(torque_motor),
        "hand_torque_limit_nm": {name(model, ACT, i): float(model.actuator_ctrlrange[i, 1]) for i in hand},
    }


def at_rest(model):
    """Montagefehler gegen das URDF und Kontakte Hand <-> restlicher Roboter in Nullstellung."""
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    hand = set().union(*(subtree(model, scene.hand_base_id(model, s)) for s in brainco.SIDES))
    contacts = []
    for i in range(data.ncon):
        c = data.contact[i]
        b1, b2 = model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]
        if (b1 in hand) != (b2 in hand):
            contacts.append(f"{name(model, BODY, b1)} <-> {name(model, BODY, b2)} ({c.dist * 1000:.1f} mm)")
    mount = {s: {"pos_mm": round(e[0], 3), "angle_deg": round(e[1], 3)} for s, e in scene.mount_error(model, data).items()}
    return mount, contacts


def urdf_pose(link):
    """Pose eines Links im URDF-torso_link: (Position, 3x3-Rotation). Kopfgelenke in Nullstellung."""
    if link == "torso_link":
        return np.zeros(3), np.eye(3)
    parent, xyz, rpy = URDF_JOINTS[link]
    pos, rot = urdf_pose(parent)
    quat = np.zeros(4)
    mujoco.mju_euler2Quat(quat, np.array(rpy), "XYZ")  # URDF-rpy = feste Achsen x, y, z
    local = np.zeros(9)
    mujoco.mju_quat2Mat(local, quat)
    return pos + rot @ np.array(xyz), rot @ local.reshape(3, 3)


def check_payload(model):
    """Anbauteile gegen das Unitree-URDF, im Rahmen des Kopf-Meshs gemessen. Kopfgelenke in Nullstellung."""
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    torso = model.body(payload.PARENT).id
    head_geom = next(g for g in range(model.ngeom) if model.geom_bodyid[g] == torso
                     and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
                     and name(model, MESH, model.geom_dataid[g]) == "head_link")
    # MuJoCo legt ein Mesh-Geom in den Schwerpunkt des Meshs (mesh_pos, mesh_quat) - zurueck zum Rahmen der STL-Datei
    mesh = model.geom_dataid[head_geom]
    offset_rot = np.zeros(9)
    mujoco.mju_quat2Mat(offset_rot, model.mesh_quat[mesh])
    ref_rot = data.geom_xmat[head_geom].reshape(3, 3) @ offset_rot.reshape(3, 3).T
    ref_pos = data.geom_xpos[head_geom] - ref_rot @ model.mesh_pos[mesh]
    urdf_head_pos, urdf_head_rot = urdf_pose("head_link")

    error = {}
    for link in payload.BODIES:
        b = model.body(link).id
        pos = ref_rot.T @ (data.xpos[b] - ref_pos)
        rot = ref_rot.T @ data.xmat[b].reshape(3, 3)
        want_pos, want_rot = urdf_pose(link)
        want_pos = urdf_head_rot.T @ (want_pos - urdf_head_pos)
        want_rot = urdf_head_rot.T @ want_rot
        angle = np.degrees(np.arccos(np.clip((np.trace(want_rot.T @ rot) - 1.0) / 2.0, -1.0, 1.0)))
        error[link] = {"pos_mm": round(float(np.linalg.norm(pos - want_pos) * 1000), 4),
                       "angle_deg": round(float(angle), 4)}

    # Kontakte Anbauteile <-> Roboter an Grenzen und Mitte beider Kopfgelenke (Rest in Nullstellung)
    added = set().union(*(subtree(model, model.body(link).id) for link in payload.BODIES))
    adr = {k: model.joint(j).qposadr[0] for k, (_, j, _, _) in payload.HEAD.items()}
    grid = {k: (*model.joint(j).range, 0.5 * sum(model.joint(j).range)) for k, (_, j, _, _) in payload.HEAD.items()}
    contacts = set()
    for pan in grid["pan"]:
        for tilt in grid["tilt"]:
            data.qpos[adr["pan"]], data.qpos[adr["tilt"]] = pan, tilt
            mujoco.mj_forward(model, data)
            for i in range(data.ncon):
                c = data.contact[i]
                b1, b2 = model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]
                if (b1 in added) != (b2 in added):
                    contacts.add(f"{name(model, BODY, b1)} <-> {name(model, BODY, b2)} bei Kopf ({pan:.2f}, {tilt:.2f})")
    cameras = {cam: name(model, BODY, model.cam_bodyid[model.camera(cam).id])
               for cam in (payload.CAMERA, payload.D435I_CAMERA)}
    return {
        "pose_error_vs_urdf": error,
        "mass_kg": {link: round(float(model.body_mass[model.body(link).id]), 5) for link in payload.BODIES},
        "contacts": sorted(contacts),
        "camera_body": cameras,
        "head_joint_range_rad": {k: model.joint(j).range.round(4).tolist() for k, (_, j, _, _) in payload.HEAD.items()},
    }


def hand_cam(data, model, side):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = data.xpos[scene.hand_base_id(model, side)]
    cam.distance, cam.azimuth, cam.elevation = 0.35, (150.0 if side == "left" else 30.0), -20.0
    return cam


def hand_response(model):
    """Handmotoren mit dem PD-Regler der Sim: jeder einzeln, dann alle zusammen.

    Schwerkraft aus: Die Koerpermotoren sind hier aus, so haengen die Arme ruhig. Gemessen wird ohnehin im
    Koordinatensystem der Hand.
    """
    model.opt.timestep = SIM_DT
    model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    limit = model.actuator_ctrlrange[:, 1]
    n_motors = len(brainco.MOTORS)

    idx = {}
    for side in brainco.SIDES:
        joints = [model.joint(n).id for n in brainco.joint_names(side)]
        acts = [int(np.flatnonzero(model.actuator_trnid[:, 0] == j)[0]) for j in joints]
        idx[side] = (np.array(acts), model.jnt_qposadr[joints], model.jnt_dofadr[joints])

    def run(target_norm):
        """Beide Haende auf dasselbe normierte Ziel. Rueckgabe: {side: Verlauf (Schritte x 6), normiert}."""
        n = int(round(MOVE_S / SIM_DT))
        traj = {side: np.zeros((n, n_motors)) for side in brainco.SIDES}
        target = brainco.to_rad(target_norm)
        for k in range(n):
            data.ctrl[:] = 0.0
            for side, (act, qadr, vadr) in idx.items():
                q, dq = data.qpos[qadr], data.qvel[vadr]
                tau = brainco.HAND_KP * (target - q) - brainco.HAND_KD * dq
                data.ctrl[act] = np.clip(tau, -limit[act], limit[act])
                traj[side][k] = brainco.to_norm(q)
            mujoco.mj_step(model, data)
        return traj

    t = np.arange(int(round(MOVE_S / SIM_DT))) * SIM_DT
    single = {side: {"motors": list(brainco.MOTORS), "closed_norm": [], "t90_s": [], "overshoot": [], "open_norm": []}
              for side in brainco.SIDES}
    all_traj = []
    for i in range(n_motors):
        only = np.zeros(n_motors)
        only[i] = 1.0
        closing = run(only)
        opening = run(np.zeros(n_motors))
        all_traj += [*closing.values(), *opening.values()]
        for side in brainco.SIDES:
            c = closing[side][:, i]
            reached = c >= MIN_CLOSED
            single[side]["closed_norm"].append(round(float(c[-1]), 3))
            single[side]["t90_s"].append(round(float(t[np.argmax(reached)]), 3) if reached.any() else None)
            single[side]["overshoot"].append(round(float(c.max() - c[-1]), 3))
            single[side]["open_norm"].append(round(float(opening[side][-1, i]), 3))

    renderer = mujoco.Renderer(model, height=480, width=640)

    def render_both():
        tiles = []
        for side in brainco.SIDES:
            renderer.update_scene(data, camera=hand_cam(data, model, side))
            tiles.append(renderer.render())
        return np.concatenate(tiles, axis=1)

    tips_open = {s: brainco.tips_in_hand_frame(model, data, s) for s in brainco.SIDES}
    img_open = render_both()
    closing = run(np.ones(n_motors))
    tips_closed = {s: brainco.tips_in_hand_frame(model, data, s) for s in brainco.SIDES}
    img_closed = render_both()
    all_traj += [*closing.values(), *run(np.zeros(n_motors)).values()]
    renderer.close()
    IMAGE.parent.mkdir(exist_ok=True)
    cv2.imwrite(str(IMAGE), np.concatenate([img_open, img_closed], axis=0)[:, :, ::-1])

    together = {
        side: {
            "closed_norm": closing[side][-1].round(3).tolist(),
            "tip_travel_cm": (np.linalg.norm(tips_closed[side] - tips_open[side], axis=1) * 100).round(2).tolist(),
        }
        for side in brainco.SIDES
    }
    stable = (np.isfinite(data.qpos).all()
              and data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number == 0
              and all(np.isfinite(v).all() for v in all_traj))
    return {"stable": bool(stable), "move_s": MOVE_S, "single_motor": single, "all_together": together}


def main():
    snap = build.snapshot()
    print(f"LeRobot-Sim: {snap}")
    spec = build.build_spec(snap)
    spec.compile()
    model, manifest = build.write_sim(spec, snap)
    print(f"gebaut: {SIM_DIR}   nq={model.nq} nv={model.nv} nu={model.nu}")

    actuators = check_actuators(model)
    body_rule = sum(1 for j in range(model.njnt) if any(p in name(model, JNT, j) for p in BODY_RULE))
    wrist = {s: model.body(f"{s}_wrist_yaw_link").id for s in brainco.SIDES}
    hand_mass = {s: round(float(model.body_subtreemass[w] - model.body_mass[w]), 3) for s, w in wrist.items()}
    mount, contacts = at_rest(model)
    response = hand_response(model)
    single = response["single_motor"].values()
    attached = check_payload(model)

    checks = {
        "nu_29_plus_12": (actuators["nu"], actuators["body_motors"], actuators["hand_motors"]) == (41, 29, 12),
        "alles_drehmoment_motoren": actuators["all_torque_motors"],
        "sim_findet_29_koerpergelenke": body_rule == 29,
        "montage_wie_unitree_urdf": all(m["pos_mm"] <= MAX_MOUNT_MM and m["angle_deg"] <= MAX_MOUNT_DEG
                                        for m in mount.values()),
        "keine_handkontakte_in_ruhe": not contacts,
        "regelung_stabil": response["stable"],
        "jeder_motor_schliesst": all(min(s["closed_norm"]) >= MIN_CLOSED for s in single),
        "jeder_motor_oeffnet": all(max(s["open_norm"]) <= MAX_OPEN for s in single),
        "ueberschwingen": all(max(s["overshoot"]) <= MAX_OVERSHOOT for s in single),
        "fingerspitzen_1cm": all(min(s["tip_travel_cm"]) >= MIN_TIP_TRAVEL_CM
                                 for s in response["all_together"].values()),
        "anbauteile_wie_unitree_urdf": all(e["pos_mm"] <= MAX_PAYLOAD_MM and e["angle_deg"] <= MAX_PAYLOAD_DEG
                                           for e in attached["pose_error_vs_urdf"].values()),
        "anbauteile_masse_wie_urdf": all(abs(attached["mass_kg"][k] - v) < 1e-6 for k, v in URDF_MASS_KG.items()),
        "anbauteile_ohne_kontakt_im_schwenkbereich": not attached["contacts"],
        "kopfkamera_an_d455": attached["camera_body"] == {payload.CAMERA: "d455_link",
                                                          payload.D435I_CAMERA: "torso_link"},
    }
    result = {
        "manifest": manifest,
        "total_mass_kg": round(float(model.body_subtreemass[0]), 3),
        "hand_mass_incl_adapter_kg": hand_mass,
        "actuators": actuators,
        "body_joints_by_sim_rule": body_rule,
        "mount_error_vs_urdf": mount,
        "hand_contacts_at_rest": contacts,
        "hand_gains": {"kp": brainco.HAND_KP, "kd": brainco.HAND_KD},
        "hand_response": response,
        "payload": attached,
        "checks": checks,
        "passed": all(checks.values()),
    }
    OUT.parent.mkdir(exist_ok=True)
    OUT.write_text(json.dumps(result, indent=2, ensure_ascii=False))

    print(f"Masse gesamt {result['total_mass_kg']} kg, je Hand inkl. Adapter {hand_mass} kg")
    print(f"Montage vs. URDF: {mount}")
    for side, s in response["single_motor"].items():
        print(f"{side:5s} einzeln zu {s['closed_norm']}  t90 {s['t90_s']} s")
        print(f"      einzeln auf {s['open_norm']}  Ueberschwingen {s['overshoot']}")
        a = response["all_together"][side]
        print(f"      alle zu {a['closed_norm']}  Spitzen {a['tip_travel_cm']} cm")
    print(f"Anbauteile vs. URDF: {attached['pose_error_vs_urdf']}")
    if contacts:
        print("Handkontakte in Ruhe:", contacts)
    if attached["contacts"]:
        print("Kontakte Anbauteile:", attached["contacts"])
    for k, v in checks.items():
        print(f"  {'ok  ' if v else 'FAIL'} {k}")
    print(f"gespeichert: {OUT.relative_to(REPO)}, {IMAGE.relative_to(REPO)}")
    print("ABNAHME BESTANDEN" if result["passed"] else "ABNAHME NICHT BESTANDEN")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
