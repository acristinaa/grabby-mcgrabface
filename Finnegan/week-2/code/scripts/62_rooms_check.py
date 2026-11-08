"""Lernpfad L2 — Karte und Weg: findet die G1 selbst durch die Tuer in den Nachbarraum, ohne eine Wand zu beruehren?

Kette je Raumwechsel:
  Szene (g1sim/rooms.py) -> Belegungskarte + Kostenkarte (g1sim/occupancy.py) -> A* von der aktuellen Pose zum Ziel
  -> Bahnverfolgung (navigation.PathFollower) -> Folgeregler aus L1 -> GR00T-Lauf-Controller
Die Position kommt noch aus der Sim (bekannte Pose, Masterplan Stufe 3.2); L7 ersetzt das durch Lokalisierung.
Die Ziele wechseln zwischen Raum B und Raum A, jedes mindestens 0,7 m von Waenden und Tisch, mit zufaelliger
End-Blickrichtung (Startwert --seed).

Umgebung ~/dev/g1-ext (wie 50/61), nie zusammen mit einer anderen DDS-Sim:
    DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib PYTHONUNBUFFERED=1 \
        ~/dev/g1-ext/.venv/bin/python scripts/62_rooms_check.py [--goals 10] [--seed 0] [--video] [--no-live]
Zuschauen: http://127.0.0.1:8020 - Ziel als orange Bodenmarke, geplanter Weg als blaue Punktspur.

Ausgabe:  runs/lernpfad_l2_rooms.json, Karte mit Wegen docs/lernpfad_l2_karte.png, mit --video docs/videos/lernpfad_l2_rooms.mp4
Abnahme:  jeder Raumwechsel erreicht (+-10 cm, +-10 Grad, nach 1 s Stehen), kein Wandkontakt, kein Sturz, kein Zeitlimit.
"""
import argparse
import json
import math
import os
import pathlib
import sys
import time
import traceback

import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim.walk_sim import WalkSim  # noqa: E402  (zuerst: initialisiert DDS vor LeRobot)
from g1sim import occupancy, rooms  # noqa: E402
from g1sim.navigation import CONTROL_HZ, PathFollower, Pose2D  # noqa: E402

OUT = REPO / "runs/lernpfad_l2_rooms.json"
PLOT = REPO / "docs/lernpfad_l2_karte.png"
VIDEO = REPO / "docs/videos/lernpfad_l2_rooms.mp4"
SETTLE_S = 1.0
GOAL_CLEARANCE = 0.7  # m Mindestabstand der Zufallsziele zu Waenden und Tisch
ORANGE = (0.91, 0.40, 0.0, 0.8)


def make_goals(rng, grid, clear, n):
    """n Ziele abwechselnd in Raum B und Raum A (der Roboter startet in A), jedes mit Abstand zu Waenden und Tisch."""
    goals = []
    while len(goals) < n:
        (x0, x1), (y0, y1) = rooms.ROOM_B if len(goals) % 2 == 0 else rooms.ROOM_A
        x, y = rng.uniform(x0, x1), rng.uniform(y0, y1)
        if clear[grid.to_cell(x, y)] >= GOAL_CLEARANCE:
            goals.append(Pose2D(x, y, rng.uniform(-math.pi, math.pi)))
    return goals


def touching(model, data, obstacle_ids):
    """Namen der Waende/Hindernisse, die gerade ein Teil des Roboters beruehrt (alles, was nicht zur Welt gehoert)."""
    # Die Sim rechnet im eigenen Thread weiter: ncon kann sich zwischen zwei Zugriffen aendern. Deshalb die Kontaktpaare
    # in einem Zugriff als Momentaufnahme kopieren; misslingt das gerade, im naechsten Durchlauf erneut (500-mal je Sekunde).
    try:
        pairs = np.array(data.contact.geom)
    except (IndexError, ValueError):
        return set()
    hits = set()
    for g1, g2 in pairs:
        for wall, other in ((g1, g2), (g2, g1)):
            if wall in obstacle_ids and model.geom_bodyid[other] != 0:
                hits.add(obstacle_ids[wall])
    return hits


def main():
    ap = argparse.ArgumentParser(description="Lernpfad L2: Karte, A* und Bahnverfolgung zwischen zwei Raeumen")
    ap.add_argument("--goals", type=int, default=10)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--video", nargs="?", const=str(VIDEO), default=None, help="mp4 rendern (ohne Pfad: Standardpfad)")
    ap.add_argument("--no-live", action="store_true", help="ohne Live-Ansicht im Browser")
    ap.add_argument("--out", default=str(OUT))
    args = ap.parse_args()

    grid = rooms.grid()
    clear, cost = grid.clearance(), grid.costmap()
    sim = WalkSim(scene=rooms.write(), live=not args.no_live, video=args.video, title="L2 Karte und Weg")
    obstacle_ids = {sim.model.geom(name).id: name for name in rooms.boxes()}
    result = {"seed": args.seed, "goals": []}
    planned, driven = [], []
    try:
        ok = sim.release_band()
        goals = make_goals(np.random.default_rng(args.seed), grid, clear, args.goals)
        sim0, wall0 = sim.time, time.perf_counter()
        for i, goal in enumerate(goals if ok else []):
            pose = sim.pose()
            cells = occupancy.astar(cost, grid.to_cell(pose.x, pose.y), grid.to_cell(goal.x, goal.y))
            room = rooms.room_of(goal.x, goal.y)
            if cells is None:
                result["goals"].append({"ziel": [round(goal.x, 3), round(goal.y, 3)], "raum": room, "status": "kein weg"})
                continue
            path = [grid.to_world(*c) for c in cells]
            planned.append(path)
            follower = PathFollower(path, goal)
            length = occupancy.path_length(path)
            limit = length / 0.25 + 25.0  # Sim-Sekunden
            t0 = next_t = sim.time
            settled_since, status, cmd, touched = None, None, (0.0, 0.0, 0.0), set()
            while status is None:
                pose = sim.pose()
                dist, _ = follower.ctrl.errors(pose, goal)
                touched |= touching(sim.model, sim.data, obstacle_ids)
                sim.show(markers=[(goal.x, goal.y, goal.yaw, ORANGE)], path=path,
                         Phase=f"Raumwechsel {i + 1} von {len(goals)}: nach Raum {room}",
                         Weg=f"{length:.1f} m geplant (A*)", Abstand=f"{100 * dist:.0f} cm",
                         Befehl=f"vx {cmd[0]:+.2f} m/s · vy {cmd[1]:+.2f} m/s · ω {cmd[2]:+.2f} rad/s",
                         Wandkontakte=", ".join(sorted(touched)) or "keine",
                         Erledigt=[f"{k + 1}: {g['status']} {g.get('fehler_cm', '')} cm" for k, g in enumerate(result["goals"])])
                if not sim.ok(f"raumwechsel {i + 1}"):
                    status = "sturz"
                    break
                t = sim.time
                if t >= next_t:
                    next_t = max(next_t + 1.0 / CONTROL_HZ, t)
                    cmd = follower.step(pose)
                    sim.send(*cmd)
                    driven.append((round(t - sim0, 3), round(pose.x, 4), round(pose.y, 4)))
                    if follower.arrived(pose) and not any(cmd):
                        settled_since = t if settled_since is None else settled_since
                        if t - settled_since >= SETTLE_S:
                            status = "erreicht"
                    else:
                        settled_since = None
                    if t - t0 > limit:
                        status = "zeitlimit"
                time.sleep(0.002)
            sim.send(0.0, 0.0, 0.0)
            follower.ctrl.stop()
            dist, yaw_err = follower.ctrl.errors(sim.pose(), goal)
            entry = {"ziel": [round(goal.x, 3), round(goal.y, 3), round(math.degrees(goal.yaw), 1)], "raum": room,
                     "status": status, "fehler_cm": round(100 * dist, 1), "fehler_grad": round(math.degrees(abs(yaw_err)), 1),
                     "weg_m": round(length, 2), "sim_s": round(sim.time - t0, 1), "wandkontakte": sorted(touched),
                     "min_wandabstand_weg_m": round(float(min(clear[c] for c in cells)), 2)}
            result["goals"].append(entry)
            print(f"Raumwechsel {i + 1}/{len(goals)} nach {room}: {status:9s} {entry['fehler_cm']:5.1f} cm "
                  f"{entry['fehler_grad']:5.1f} Grad, Weg {length:4.1f} m in {entry['sim_s']:5.1f} s Sim, "
                  f"Wandkontakte: {', '.join(entry['wandkontakte']) or 'keine'}")
            if status == "sturz":
                break

        wall = time.perf_counter() - wall0
        result["realtime_factor"] = round((sim.time - sim0) / wall, 3) if ok and wall > 0 else None
        reached = [g for g in result["goals"] if g["status"] == "erreicht"]
        if reached:
            result["mittel_cm"] = round(float(np.mean([g["fehler_cm"] for g in reached])), 1)
            result["max_cm"] = max(g["fehler_cm"] for g in reached)
        result["fault"] = sim.fault
        result["passed"] = (ok and sim.fault is None and len(reached) == args.goals
                            and not any(g.get("wandkontakte") for g in result["goals"]))
        result["driven"] = driven
    finally:
        sim.close()
    save(result, grid, cost, planned, driven, goals if ok else [], pathlib.Path(args.out))
    return 0 if result.get("passed") else 1


def save(result, grid, cost, planned, driven, goals, out):
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=1, ensure_ascii=False))
    print(json.dumps({k: v for k, v in result.items() if k not in ("driven", "goals")}, indent=2, ensure_ascii=False))
    plot(grid, cost, planned, driven, goals)
    print(f"gespeichert: {out}, Karte {PLOT}")
    print("ABNAHME BESTANDEN" if result["passed"] else "ABNAHME NICHT BESTANDEN")


def plot(grid, cost, planned, driven, goals):
    """Karte von oben: Waende schwarz, unpassierbarer Rand dunkelgrau, Kosten hell -> dunkel; geplante und gelaufene Wege."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    shade = np.where(np.isfinite(cost), (cost - 1.0) / occupancy.WALL_COST * 0.5, 0.75)
    shade[grid.occupied] = 1.0
    x0, y0 = grid.origin
    extent = [x0, x0 + grid.shape[1] * grid.res, y0, y0 + grid.shape[0] * grid.res]
    fig, ax = plt.subplots(figsize=(11, 6))
    ax.imshow(shade, origin="lower", extent=extent, cmap="Greys", vmin=0, vmax=1)
    for k, path in enumerate(planned):
        p = np.array(path)
        ax.plot(p[:, 0], p[:, 1], "--", color="#E76500", lw=1, alpha=0.7, label="geplant (A*)" if k == 0 else None)
    if driven:
        d = np.array([(x, y) for _, x, y in driven])
        ax.plot(d[:, 0], d[:, 1], color="#0070F2", lw=1.3, label="gelaufen")
    for i, g in enumerate(goals):
        ax.add_patch(plt.Circle((g.x, g.y), 0.10, color="#E76500", fill=False))
        ax.annotate(str(i + 1), (g.x, g.y), textcoords="offset points", xytext=(5, 5))
    ax.plot(0, 0, "ko", label="Start")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("L2: Belegungs- und Kostenkarte (schwarz Wand, dunkelgrau unpassierbar, grau teurer), Wege")
    ax.legend(loc="lower left", fontsize=8)
    PLOT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(PLOT, dpi=110, bbox_inches="tight")
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
