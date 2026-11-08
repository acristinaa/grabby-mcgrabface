"""Stufe 0.2 / 0.3c — laeuft die LeRobot-G1-Sim mit fertigem Lauf-Controller auf dem Mac?

Prueft drei Dinge auf einmal:
  1. Laufen die LeRobot-G1-Sim (lerobot/unitree-g1-mujoco) und CycloneDDS unter macOS?
  2. Haelt der GR00T-Lauf-Controller die Balance, wenn das Gummiband geloest ist?
  3. Laeuft der G1 per Geschwindigkeitsbefehl AUS CODE (kein Gamepad) 10 m ohne Sturz?

Der Lauf-Controller steuert nur Beine und Taille (Gelenke 0-14). Wohin gelaufen wird,
kommt hier aus diesem Skript - spaeter aus der Navigation (siehe PLAN.md, SCOPE UPDATE).

Mit --brainco (Stufe 0.3c) laeuft derselbe Test in der Sim mit BrainCo-Revo2-Haenden statt Dex3
(vorher einmal scripts/51_build_brainco_sim.py). Zusaetzlich gemessen wird, wie genau beide Arme ihrer
Sollstellung folgen - das prueft die Motor-Zuordnung aus g1sim/brainco_env.py.

Laeuft NICHT in der Projekt-venv, sondern in der Spike-Umgebung unter ~/dev/g1-ext
(LeRobot 0.6 + unitree_sdk2py + selbst gebautes CycloneDDS 0.10). Immer ohne Fenster:

    DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib PYTHONUNBUFFERED=1 \
        ~/dev/g1-ext/.venv/bin/python scripts/50_spike_lerobot_walk.py --headless --video [--brainco]

Zum Anschauen rendert --video offscreen ein mp4, die Kamera folgt dem Becken.
NICHT das MuJoCo-Fenster benutzen (mjpython, ohne --headless): Es friert in dieser Umgebung
beim Oeffnen ein und hinterlaesst einen Prozess, der sich erst beim Abmelden beenden laesst
(beobachtet 2026-09-14).

Optionen: --speed 0.5 (Joystick-Achse ly, -1..1)  --meters 10  --timeout 90  --headless  --video [PFAD]
          --brainco  --out PFAD
Ausgabe:  runs/stufe0_2_lerobot_walk.json bzw. runs/stufe0_3c_lerobot_walk_brainco.json,
          Video docs/videos/stufe0_2_g1_laeuft.mp4 bzw. docs/videos/stufe0_3c_g1_brainco_laeuft.mp4
Abnahme:  Zielstrecke erreicht, kein Sturz.
"""
import argparse
import json
import os
import pathlib
import sys
import time
import traceback

import mujoco
import mujoco.viewer
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

# macOS: Die Loopback-Schnittstelle heisst lo0. LeRobot und die Sim initialisieren fest
# mit "lo" (Linux-Name). Die Channel-Factory ist ein Singleton - wer zuerst initialisiert,
# gewinnt, spaetere Aufrufe kehren sofort zurueck. Deshalb hier, vor jedem LeRobot-Import.
ChannelFactoryInitialize(0, "lo0")

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config  # noqa: E402

from g1sim.brainco_env import use_brainco_sim  # noqa: E402
from g1sim.runtime import StallWatch, free_camera_port  # noqa: E402
from g1sim.video import VideoRecorder  # noqa: E402

# (Ergebnis, Video) je Sim
OUTPUTS = {
    "dex3": ("runs/stufe0_2_lerobot_walk.json", "docs/videos/stufe0_2_g1_laeuft.mp4"),
    "brainco": ("runs/stufe0_3c_lerobot_walk_brainco.json", "docs/videos/stufe0_3c_g1_brainco_laeuft.mp4"),
}

FALL_HEIGHT = 0.5     # Becken darunter = gestuerzt (Sim-eigenes check_fall greift erst bei 0.2)
STAND_S = 3.0         # am Gummiband stehen
BALANCE_S = 3.0       # frei stehen, Band geloest
STOP_S = 3.0          # nach dem Laufen wieder frei stehen
ARMS = {"left": range(15, 22), "right": range(22, 29)}  # G1-Motorindizes


def arm_tracking_error(sim):
    """Groesster Abstand Soll (rt/lowcmd) <-> Ist je Arm in rad, nur Gelenke mit kp > 0."""
    cmd = sim.unitree_bridge.low_cmd.motor_cmd
    qadr = getattr(sim, "body_qadr", None)
    if qadr is None:
        qadr = sim.body_joint_index + 6  # Original-Sim: nur Scharniergelenke nach dem freien Gelenk
    q = sim.mj_data.qpos[qadr]
    return {side: max((abs(cmd[i].q - q[i]) for i in idx if cmd[i].kp > 0), default=None)
            for side, idx in ARMS.items()}


def main():
    ap = argparse.ArgumentParser(description="Stufe 0.2 / 0.3c: G1 laeuft in der LeRobot-Sim")
    ap.add_argument("--speed", type=float, default=0.5, help="Joystick-Achse ly (-1..1)")
    ap.add_argument("--meters", type=float, default=10.0, help="Zielstrecke in m")
    ap.add_argument("--timeout", type=float, default=90.0, help="max. Laufzeit in s (Wanduhr)")
    ap.add_argument("--headless", action="store_true",
                    help="ohne Fenster, mit normalem python statt mjpython (empfohlen)")
    ap.add_argument("--video", nargs="?", const="auto", default=None,
                    help="offscreen ein mp4 rendern (ohne Pfad: Standardpfad je Sim)")
    ap.add_argument("--brainco", action="store_true", help="Sim mit BrainCo-Revo2 statt Dex3 (Stufe 0.3c)")
    ap.add_argument("--out", default=None, help="Ergebnis-JSON (Standard je Sim)")
    args = ap.parse_args()

    sim_name = "brainco" if args.brainco else "dex3"
    out = pathlib.Path(args.out) if args.out else REPO / OUTPUTS[sim_name][0]
    video = REPO / OUTPUTS[sim_name][1] if args.video == "auto" else args.video

    # Die Sim ruft mujoco.viewer.launch_passive auf, sobald ENABLE_ONSCREEN in ihrer config.yaml
    # steht. Die liegt im Hugging-Face-Cache und bleibt unveraendert - der Aufruf wird hier umhuellt:
    #  - immer zuerst mj_forward. Sonst sind die Koerper-Quaternionen null und das Gummiband
    #    stuerzt im ersten Schritt ab (scipy: "Found zero norm quaternions", beobachtet 2026-09-14)
    #  - mit --headless kein Fenster (viewer = None), wie im fensterlosen Zweig der Sim
    real_launch_passive = mujoco.viewer.launch_passive

    def launch_passive(model, data, *a, **k):
        mujoco.mj_forward(model, data)
        return None if args.headless else real_launch_passive(model, data, *a, **k)

    mujoco.viewer.launch_passive = launch_passive

    free_camera_port()
    if args.brainco:
        use_brainco_sim()
    robot = UnitreeG1(UnitreeG1Config(is_simulation=True, controller="GrootLocomotionController"))
    t0 = time.perf_counter()
    robot.connect()
    print(f"verbunden nach {time.perf_counter() - t0:.1f} s ({sim_name})")

    sim = robot.sim_env.sim_env            # DefaultEnv: mj_model, mj_data, elastic_band, viewer
    data = sim.mj_data
    rec = VideoRecorder(sim.mj_model, data, video) if video else None
    poll = 0.01 if rec else 0.05           # beim Filmen oefter nachsehen, sonst ruckelt das Video
    result = {"sim": sim_name, "speed_cmd": args.speed, "target_m": args.meters, "fell": False}
    watch = StallWatch(data)

    def pelvis():
        return np.array(data.qpos[:3])

    def hold(seconds, label):
        """Wanduhr abwarten und dabei auf Sturz und Stillstand pruefen. False bei Sturz oder Stillstand."""
        end = time.perf_counter() + seconds
        while time.perf_counter() < end:
            if rec:
                rec.grab()
            z = pelvis()[2]
            if z < FALL_HEIGHT:
                print(f"STURZ waehrend '{label}' (Becken {z:.2f} m)")
                result.update(fell=True, fell_during=label)
                return False
            if watch.stalled():
                result["sim_stalled_during"] = label
                return False
            time.sleep(poll)
        return True

    try:
        ok = hold(STAND_S, "am Gummiband")
        sim.elastic_band.enable = False
        print("Gummiband geloest")
        ok = ok and hold(BALANCE_S, "frei stehen")

        if ok:
            start = pelvis()
            sim_t0, wall_t0 = float(data.time), time.perf_counter()
            robot.send_action({"remote.ly": args.speed})
            print(f"Laufbefehl aus Code: remote.ly = {args.speed}")

            dist, last_print = 0.0, 0.0
            arm_max = {side: None for side in ARMS}
            while dist < args.meters and time.perf_counter() - wall_t0 < args.timeout:
                if rec:
                    rec.grab()
                p = pelvis()
                if p[2] < FALL_HEIGHT:
                    print(f"STURZ beim Laufen nach {dist:.2f} m (Becken {p[2]:.2f} m)")
                    result.update(fell=True, fell_during="laufen")
                    break
                if watch.stalled():
                    result["sim_stalled_during"] = "laufen"
                    break
                for side, err in arm_tracking_error(sim).items():
                    if err is not None:
                        arm_max[side] = err if arm_max[side] is None else max(arm_max[side], err)
                dist = float(np.linalg.norm(p[:2] - start[:2]))
                if time.perf_counter() - last_print > 1.0:
                    last_print = time.perf_counter()
                    print(f"  {dist:5.2f} m   Becken {p[2]:.2f} m   Sim-Zeit {data.time - sim_t0:5.1f} s")
                time.sleep(poll)
            robot.send_action({"remote.ly": 0.0})

            sim_dt = float(data.time) - sim_t0
            wall_dt = time.perf_counter() - wall_t0
            result.update(
                distance_m=round(dist, 3),
                delta_xy_m=[round(float(v), 3) for v in (pelvis()[:2] - start[:2])],
                sim_s=round(sim_dt, 2),
                wall_s=round(wall_dt, 2),
                speed_m_s=round(dist / sim_dt, 3) if sim_dt > 0 else None,
                realtime_factor=round(sim_dt / wall_dt, 3) if wall_dt > 0 else None,
                arm_tracking_max_rad={k: None if v is None else round(float(v), 3) for k, v in arm_max.items()},
            )
            if not result["fell"]:
                hold(STOP_S, "anhalten")

        report(result, args.meters, out)
        if sim.viewer is not None:
            print("Fenster bleibt offen - zum Beenden das MuJoCo-Fenster schliessen.")
            while sim.viewer.is_running():
                time.sleep(0.2)
    finally:
        if rec is not None:
            rec.close()
        robot.disconnect()

    return 0 if result.get("passed") else 1


def report(result, meters, out):
    """Abnahme bewerten, Ergebnis speichern und ausgeben."""
    result["passed"] = (not result["fell"] and "sim_stalled_during" not in result
                        and result.get("distance_m", 0.0) >= meters)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2, ensure_ascii=False))
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print(f"gespeichert: {out}")
    print("ABNAHME BESTANDEN" if result["passed"] else "ABNAHME NICHT BESTANDEN")


if __name__ == "__main__":
    # CycloneDDS- und Sim-Threads halten den Interpreter nach dem Ende am Leben
    # (beobachtet 2026-09-14) - auch nach einem Fehler. Deshalb in jedem Fall hart beenden.
    try:
        code = main()
    except BaseException:
        traceback.print_exc()
        code = 2
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
