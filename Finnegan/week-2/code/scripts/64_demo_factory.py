"""Lernpfad D4 - Demo-Fabrik: Der Skript-Griff (g1sim/grasp.py) nimmt Episoden fuer den Datensatz auf.

Startlagen wie 54 --randomize (Seed + laufende Nummer, alle Gruppen aus configs/randomization.json). Gegriffen wird nur, wenn die
Flasche erreichbar ist (grasp.reach_error <= 1 cm); bewertet wird mit derselben Erfolgsdefinition wie 54. Gespeichert werden nur
Erfolge, Fehlschlaege und Starts ausser Reichweite stehen mit Ursache im Bericht.
Aufgenommen wird jeder Regeltakt vom Start bis zum Ende des Haltens: Kopfkamerabild + Zustand vor der Aktion und die Aktion
(26-D, g1sim/episode.py) - dasselbe Format wie Teleop (57) und der Policy-Server. Daneben eine JSON-Datei mit Seed, Nummer und
gezogener Variation: TableEnv.reset(variation=...) stellt die Startlage wieder her.
Seeds: Die Auswertungen (54, 63) nutzen Seed 0 und 1. Demos kommen aus anderen Seeds (Standard 100) - sonst pruefte die spaetere
Auswertung von pi0.5 (Stufe 2.3) auf Startlagen, die es aus dem Training kennt. Mehrere Prozesse: je einer pro Seed, gleicher Ordner.
Speicherort runs/demos.nosync/<name>: runs/ ist nicht im Git, ".nosync" nimmt den Ordner vom iCloud-Abgleich aus (~80 MB je Episode).
Umgebung ~/dev/g1-ext:
    ~/dev/g1-ext/.venv/bin/python scripts/64_demo_factory.py --demos 2 --name probe
Umwandlung danach in der openpi-Umgebung (g1sim/episode.py -> LeRobot v2.1):
    ~/dev/g1-ext/openpi/.venv/bin/python -m openpi_g1.convert runs/demos.nosync/probe/*.npz
"""
import argparse
import collections
import importlib.util
import json
import pathlib
import sys
import time
import warnings

import mink

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim import episode, grasp, table_env, table_scene  # noqa: E402
from g1sim.openpi_env import DEFAULT_TASK  # noqa: E402

warnings.filterwarnings("ignore", message=".*parent has")
warnings.filterwarnings("ignore", message=".*encountered in matmul")  # Fehlalarm numpy + Apple Accelerate

# Erfolgsdefinition aus 54 uebernehmen, nicht nachbauen
_spec = importlib.util.spec_from_file_location("grasp_scripted", REPO / "scripts" / "54_grasp_scripted.py")
grasp_scripted = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(grasp_scripted)

OUT = REPO / "runs" / "demos.nosync"


def record(env, seed, trial, out_dir):
    """Ein Versuch auf der aktuellen Startlage. Rueckgabe: Protokolleintrag (dict)."""
    entry = {"trial": trial}
    recorder = episode.EpisodeRecorder(DEFAULT_TASK, table_env.CONTROL_HZ)
    pending = [env.observation()]  # Beobachtung vor der naechsten Aktion

    def on_tick(phase):  # ScriptedGrasp ruft das nach env.apply(): g.action ist die eben ausgefuehrte Aktion
        recorder.add(pending[0], g.action.copy())
        pending[0] = env.observation()

    g = grasp.ScriptedGrasp(env, on_tick=on_tick)
    entry["reach_error_m"] = reach = g.reach_error()
    if reach > grasp_scripted.MAX_GRASP_POS_ERROR:
        g.close()
        return {**entry, "result": "flasche_in_reichweite"}
    start = time.perf_counter()
    try:
        result = g.run()
    except (RuntimeError, mink.exceptions.NoSolutionFound) as error:
        result = {"error": f"{type(error).__name__}: {error}"}
    finally:
        g.close()
    result["reach_error_m"] = reach
    checks = grasp_scripted.evaluate(result)
    entry["wall_s"] = time.perf_counter() - start
    if not all(checks.values()):
        return {**entry, "result": next(k for k, ok in checks.items() if not ok), "error": result.get("error")}

    path = out_dir / f"episode_s{seed}_t{trial:04d}.npz"
    recorder.save(path, success=True)
    problems = episode.validate(episode.load(path))
    info = {"task": DEFAULT_TASK, "success": True, "source": "scripts/64_demo_factory.py", "seed": seed, "trial": trial,
            "frames": len(recorder), "sim_s": len(recorder) / table_env.CONTROL_HZ, "wall_s": round(entry["wall_s"], 1),
            "reach_error_m": reach, "min_lift_m": round(result["min_lift_during_hold_m"], 4),
            "bottle_tilt_deg": round(result["bottle_tilt_deg"], 1), "validate": problems,
            "grasp": {"GRASP_BELOW_TOP": grasp.GRASP_BELOW_TOP, "BOTTLE_IN_HAND": grasp.BOTTLE_IN_HAND.tolist(),
                      "GRASP_ROTATION": grasp.GRASP_ROTATION.round(6).tolist()},
            "variation": env.variation}
    path.with_suffix(".json").write_text(json.dumps(info, indent=2, ensure_ascii=False))
    return {**entry, "result": "gespeichert", "path": str(path), "frames": len(recorder),
            "size_mb": path.stat().st_size / 1e6, "validate": problems}


def main():
    parser = argparse.ArgumentParser(description="D4 - Demo-Fabrik")
    parser.add_argument("--demos", type=int, default=2, help="so viele erfolgreiche Episoden speichern")
    parser.add_argument("--seed", type=int, default=100, help="nicht 0 oder 1 (Auswertung)")
    parser.add_argument("--max-trials", type=int, help="hoechstens so viele Startlagen ziehen (Standard 10 je Demo + 20)")
    parser.add_argument("--name", default="probe")
    parser.add_argument("--out", type=pathlib.Path, default=OUT)
    args = parser.parse_args()
    out_dir = args.out / args.name
    out_dir.mkdir(parents=True, exist_ok=True)
    max_trials = args.max_trials or 10 * args.demos + 20

    table_scene.write()
    env = table_env.TableEnv(seed=args.seed)
    log, saved = [], 0
    for trial in range(max_trials):
        if saved == args.demos:
            break
        env.reset(randomize=True)
        entry = record(env, args.seed, trial, out_dir)
        saved += entry["result"] == "gespeichert"
        log.append(entry)
        detail = (f"{entry['frames']} Frames, {entry['size_mb']:.0f} MB, {entry['wall_s']:.0f} s" if "path" in entry
                  else f"Reichweite {entry['reach_error_m'] * 1000:.0f} mm")
        print(f"Start {trial}: {entry['result']} ({detail})")
    env.close()

    demos = [e for e in log if "path" in e]
    attempted = [e for e in log if "wall_s" in e]
    report = {
        "seed": args.seed, "starts": len(log), "attempted": len(attempted), "saved": len(demos),
        "results": dict(collections.Counter(e["result"] for e in log)),
        "wall_s_per_demo": sum(e["wall_s"] for e in attempted) / len(demos) if demos else None,
        "mb_per_demo": sum(e["size_mb"] for e in demos) / len(demos) if demos else None,
        "frames_per_demo": sum(e["frames"] for e in demos) / len(demos) if demos else None,
        "log": log,
    }
    (out_dir / f"factory_s{args.seed}.json").write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(f"{len(demos)} Demos aus {len(log)} Startlagen ({len(attempted)} gegriffen) -> {out_dir}")
    if demos:
        print(f"je Demo: {report['frames_per_demo']:.0f} Frames = {report['frames_per_demo'] / table_env.CONTROL_HZ:.1f} s Sim, "
              f"{report['wall_s_per_demo']:.0f} s Rechenzeit (inkl. Fehlversuche), {report['mb_per_demo']:.0f} MB; "
              f"hochgerechnet auf 100 Demos: {report['wall_s_per_demo'] * 100 / 60:.0f} min in einem Prozess, "
              f"{report['mb_per_demo'] * 100 / 1000:.1f} GB")
    bad = [e for e in demos if e["validate"]]
    if bad:
        print("episode.validate() meldet Probleme:", {pathlib.Path(e["path"]).name: e["validate"] for e in bad})
    return 0 if len(demos) == args.demos and not bad else 1


if __name__ == "__main__":
    sys.exit(main())
