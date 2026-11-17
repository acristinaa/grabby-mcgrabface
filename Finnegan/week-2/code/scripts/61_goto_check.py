"""Lernpfad L1 — Gehe zu Punkt: laeuft die G1 mit Rueckkopplung genau an zufaellige Ziele?

Der Folgeregler g1sim/navigation.py misst 20-mal je Sim-Sekunde die Becken-Pose (vorerst direkt aus der Sim), rechnet den
Fehler ins Koerpersystem und schickt (vx, vy, omega) als virtuelle Joystick-Achsen an den GR00T-Lauf-Controller.
Zum Vergleich: scripts/50_spike_lerobot_walk.py schickt den Befehl einmal und driftet ohne Rueckkopplung ab.
Die Ziele liegen nacheinander 1-4 m auseinander, jedes mit zufaelliger End-Blickrichtung (Startwert --seed).

Umgebung ~/dev/g1-ext (wie 50), nie zusammen mit einer anderen DDS-Sim:
    DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib PYTHONUNBUFFERED=1 \
        ~/dev/g1-ext/.venv/bin/python scripts/61_goto_check.py [--goals 10] [--seed 0] [--video] [--no-live]
Zuschauen: http://127.0.0.1:8020 im Browser (g1sim/liveview.py) - Roboter, aktuelles Ziel als Bodenmarke, Fehler und Befehl.

Ausgabe:  runs/lernpfad_l1_goto.json, Draufsicht docs/lernpfad_l1_wege.png, mit --video docs/videos/lernpfad_l1_goto.mp4
Abnahme:  jedes Ziel auf +-10 cm und +-10 Grad erreicht (gemessen nach 1 s Stehen), kein Sturz, kein Zeitlimit gerissen.
"""
import argparse
import json
import math
import os
import pathlib
import sys
import time
import traceback

import mujoco
import mujoco.viewer
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

ChannelFactoryInitialize(0, "lo0")  # macOS-Loopback, vor jedem LeRobot-Import (siehe 50_spike_lerobot_walk.py)

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config  # noqa: E402

from g1sim.brainco_env import use_brainco_sim  # noqa: E402
from g1sim.liveview import LiveView  # noqa: E402
from g1sim.navigation import CONTROL_HZ, GotoController, Pose2D, pose_from_qpos, to_remote  # noqa: E402
from g1sim.runtime import StallWatch, free_camera_port  # noqa: E402

OUT = REPO / "runs/lernpfad_l1_goto.json"
PLOT = REPO / "docs/lernpfad_l1_wege.png"
VIDEO = REPO / "docs/videos/lernpfad_l1_goto.mp4"
FALL_HEIGHT = 0.5     # Becken darunter = gestuerzt
STAND_S = 3.0         # Sim-Sekunden am Gummiband
BALANCE_S = 3.0       # frei stehen vor dem ersten Ziel
SETTLE_S = 1.0        # angekommen und so lange gestanden -> Ziel gilt als erreicht
ARENA = 3.0           # m: Ziele bleiben im Quadrat +-ARENA um den Ursprung (ebener Boden, keine Hindernisse)


def make_goals(rng, start, n):
    """n Ziele nacheinander 1-4 m auseinander, jedes mit zufaelliger End-Blickrichtung."""
    goals, x, y = [], start.x, start.y
    while len(goals) < n:
        d, a = rng.uniform(1.0, 4.0), rng.uniform(-math.pi, math.pi)
        gx, gy = x + d * math.cos(a), y + d * math.sin(a)
        if abs(gx) <= ARENA and abs(gy) <= ARENA:
            goals.append(Pose2D(gx, gy, rng.uniform(-math.pi, math.pi)))
            x, y = gx, gy
    return goals


def main():
    ap = argparse.ArgumentParser(description="Lernpfad L1: Gehe zu Punkt mit Rueckkopplung")
    ap.add_argument("--goals", type=int, default=10)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--video", nargs="?", const=str(VIDEO), default=None, help="mp4 rendern (ohne Pfad: Standardpfad)")
    ap.add_argument("--no-live", action="store_true", help="ohne Live-Ansicht im Browser")
    ap.add_argument("--out", default=str(OUT))
    args = ap.parse_args()

    # Kein MuJoCo-Fenster (friert auf diesem Mac ein) - Begruendung in 50_spike_lerobot_walk.py
    def launch_passive(model, data, *a, **k):
        mujoco.mj_forward(model, data)

    mujoco.viewer.launch_passive = launch_passive
    free_camera_port()
    use_brainco_sim(cameras=False)  # keine Sim-Kamerabilder: sonst rendern zwei Threads (Haenger 2026-10-02, siehe brainco_env)
    robot = UnitreeG1(UnitreeG1Config(is_simulation=True, controller="GrootLocomotionController"))
    robot.connect()
    sim = robot.sim_env.sim_env
    data = sim.mj_data
    # Ein Renderer fuer Browser und Video
    live = LiveView(sim.mj_model, data, video=args.video, serve=not args.no_live) if (args.video or not args.no_live) else None
    watch = StallWatch(data)
    result = {"seed": args.seed, "fell": False, "goals": []}
    path = []  # (Sim-Zeit, x, y, yaw) je Regeltakt

    def broken(label):
        """Sturz oder stehende Sim -> Grund in result eintragen."""
        if data.qpos[2] < FALL_HEIGHT:
            result.update(fell=True, fell_during=label)
            print(f"STURZ waehrend '{label}' (Becken {data.qpos[2]:.2f} m)")
            return True
        if watch.stalled():
            result["sim_stalled_during"] = label
            return True
        return False

    def show(**status):
        """Live-Ansicht und Video nachfuehren."""
        if live:
            live.update({"titel": "L1 Gehe zu Punkt", "Sim-Zeit": f"{float(data.time):.1f} s", **status})

    def wait_sim(seconds, label):
        """Sim-Zeit abwarten (die Sim laeuft auf dem Mac langsamer als die Wanduhr). False bei Sturz/Stillstand."""
        end = float(data.time) + seconds
        while float(data.time) < end:
            show(Phase=label)
            if broken(label):
                return False
            time.sleep(0.005)
        return True

    try:
        ok = wait_sim(STAND_S, "am Gummiband")
        sim.elastic_band.enable = False
        ok = ok and wait_sim(BALANCE_S, "frei stehen")
        start = pose_from_qpos(data.qpos)
        goals = make_goals(np.random.default_rng(args.seed), start, args.goals)
        ctrl = GotoController()
        wall0 = time.perf_counter()
        sim0 = float(data.time)

        for i, goal in enumerate(goals if ok else []):
            pose = pose_from_qpos(data.qpos)
            limit = math.hypot(goal.x - pose.x, goal.y - pose.y) / 0.25 + 20.0  # Sim-Sekunden
            t0 = next_t = float(data.time)
            settled_since, status = None, None
            cmd = (0.0, 0.0, 0.0)
            if live:
                live.update(markers=[(goal.x, goal.y, goal.yaw, (0.91, 0.40, 0.0, 0.8))])
            while status is None:
                dist, yaw_err = ctrl.errors(pose, goal)
                show(Phase=f"Ziel {i + 1} von {len(goals)}", Abstand=f"{100 * dist:.0f} cm",
                     Winkelfehler=f"{math.degrees(abs(yaw_err)):.0f} Grad",
                     Befehl=f"vx {cmd[0]:+.2f} m/s · vy {cmd[1]:+.2f} m/s · ω {cmd[2]:+.2f} rad/s",
                     Erledigt=[f"{k + 1}: {g['status']} {g['fehler_cm']} cm" for k, g in enumerate(result["goals"])])
                t = float(data.time)
                if broken(f"ziel {i + 1}"):
                    status = "sturz"
                elif t >= next_t:
                    next_t = max(next_t + 1.0 / CONTROL_HZ, t)
                    pose = pose_from_qpos(data.qpos)
                    cmd = ctrl.step(pose, goal)
                    robot.send_action(to_remote(*cmd))
                    path.append((round(t - sim0, 3), round(pose.x, 4), round(pose.y, 4), round(pose.yaw, 4)))
                    if ctrl.arrived(pose, goal) and not any(cmd):
                        settled_since = t if settled_since is None else settled_since
                        if t - settled_since >= SETTLE_S:
                            status = "erreicht"
                    else:
                        settled_since = None
                    if t - t0 > limit:
                        status = "zeitlimit"
                time.sleep(0.002)
            robot.send_action(to_remote(0.0, 0.0, 0.0))
            ctrl.stop()
            dist, yaw_err = ctrl.errors(pose_from_qpos(data.qpos), goal)
            entry = {"ziel": [round(goal.x, 3), round(goal.y, 3), round(math.degrees(goal.yaw), 1)],
                     "status": status, "fehler_cm": round(100 * dist, 1),
                     "fehler_grad": round(math.degrees(abs(yaw_err)), 1), "sim_s": round(float(data.time) - t0, 1)}
            result["goals"].append(entry)
            print(f"Ziel {i + 1}/{len(goals)}: {status:9s} {entry['fehler_cm']:5.1f} cm {entry['fehler_grad']:5.1f} Grad "
                  f"in {entry['sim_s']:5.1f} s Sim")
            if status == "sturz" or "sim_stalled_during" in result:
                break

        wall = time.perf_counter() - wall0
        result["realtime_factor"] = round((float(data.time) - sim0) / wall, 3) if wall > 0 else None
        reached = [g for g in result["goals"] if g["status"] == "erreicht"]
        if reached:
            result["mittel_cm"] = round(float(np.mean([g["fehler_cm"] for g in reached])), 1)
            result["max_cm"] = max(g["fehler_cm"] for g in reached)
        result["passed"] = (ok and not result["fell"] and "sim_stalled_during" not in result
                            and len(reached) == args.goals)
        result["path"] = path
        save(result, goals, start, pathlib.Path(args.out))
    finally:
        if live is not None:
            live.close()
        robot.disconnect()
    return 0 if result.get("passed") else 1


def save(result, goals, start, out):
    """Ergebnis speichern, Draufsicht zeichnen, Abnahme ausgeben."""
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=1, ensure_ascii=False))
    summary = {k: v for k, v in result.items() if k not in ("path", "goals")}
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    plot(result["path"], goals, start)
    print(f"gespeichert: {out}, Draufsicht {PLOT}")
    print("ABNAHME BESTANDEN" if result["passed"] else "ABNAHME NICHT BESTANDEN")


def plot(path, goals, start):
    """Draufsicht: gelaufener Weg, Ziele mit 10-cm-Kreis und Soll-Blickrichtung."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 7))
    if path:
        xy = np.array([(p[1], p[2]) for p in path])
        ax.plot(xy[:, 0], xy[:, 1], color="#0070F2", lw=1.5, label="Weg des Beckens")
    ax.plot(start.x, start.y, "ko", label="Start")
    for i, g in enumerate(goals):
        ax.add_patch(plt.Circle((g.x, g.y), 0.10, color="#E76500", fill=False))
        ax.arrow(g.x, g.y, 0.3 * math.cos(g.yaw), 0.3 * math.sin(g.yaw), color="#E76500", head_width=0.08)
        ax.annotate(str(i + 1), (g.x, g.y), textcoords="offset points", xytext=(6, 6))
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("L1 Gehe zu Punkt: Ziele (Kreis = 10 cm, Pfeil = Soll-Blickrichtung)")
    ax.legend(loc="upper right")
    ax.grid(alpha=0.3)
    PLOT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(PLOT, dpi=120, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    # CycloneDDS- und Sim-Threads halten den Interpreter am Leben (siehe 50) - deshalb in jedem Fall hart beenden
    try:
        code = main()
    except BaseException:
        traceback.print_exc()
        code = 2
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
