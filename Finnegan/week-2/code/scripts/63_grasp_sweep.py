"""Lernpfad L3a Teil 2 / D2 - Parameterstudie Greifgeometrie: Welche Griffhaltung haelt die Flasche am geradesten?

Fuehrt den Ablauf aus g1sim/grasp.py fuer mehrere Varianten aus, jede auf denselben Startlagen (gleicher Seed), und bewertet
jeden Versuch mit derselben Erfolgsdefinition wie 54 (evaluate). Eine Variante ueberschreibt Konstanten aus g1sim/grasp.py nur
in diesem Prozess - grasp.py selbst aendert sich erst, wenn ein Messlauf eine bessere Haltung belegt.
Schluessel einer Variante (fehlende bleiben wie in grasp.py):
  GRASP_BELOW_TOP              Greifhoehe unter dem Deckel (m)
  BOTTLE_IN_HAND [x, y, z]     Flaschenachse in Handbasis-Koordinaten (m): x Handflaechennormale, y Daumenseite, z Finger
  ROT_DEG [rx, ry, rz]         Greifhaltung um den Greifpunkt auf der Flaschenachse drehen (Grad), nacheinander um die
                               Handachsen x, y, z. Beim Greifen zeigt y nach oben: ry dreht die Hand um die Flasche herum
  PREGRASP_OFFSET, ABOVE, LIFT, LOAD_SAG (m) · THUMB_S, CLOSE_S, LIFT_S (s) · THUMB_ROTATED, CLOSED (6 Handwerte 0..1)
Bildrauschen bekommt je Versuch einen eigenen Zufallsstrom (Seed, Versuchsnummer): Dadurch ist ein Versuch unabhaengig davon,
in welchem Prozess er laeuft. Die Zahlen sind deshalb vergleichbar, aber nicht bitgleich mit 54.
Umgebung ~/dev/g1-ext:
    ~/dev/g1-ext/.venv/bin/python scripts/63_grasp_sweep.py --variants varianten.json --base --randomize --trials 50 \
        --jobs 6 --out runs/d2_sweep.json
    eine Variante direkt:  --set GRASP_BELOW_TOP=0.09 --set ROT_DEG=0,15,0
varianten.json: {"name": {Schluessel: Wert, ...}, ...}. Ausgabe: Tabelle je Variante + JSON mit allen Versuchen.
"""
import argparse
import collections
import concurrent.futures
import copy
import importlib.util
import json
import multiprocessing
import pathlib
import statistics
import sys
import warnings

import mink
import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim import grasp, randomization, table_env, table_scene  # noqa: E402

warnings.filterwarnings("ignore", message=".*parent has")
warnings.filterwarnings("ignore", message=".*encountered in matmul")  # Fehlalarm numpy + Apple Accelerate

# Erfolgsdefinition aus 54 uebernehmen, nicht nachbauen
_spec = importlib.util.spec_from_file_location("grasp_scripted", REPO / "scripts" / "54_grasp_scripted.py")
grasp_scripted = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(grasp_scripted)

KEYS = ("GRASP_BELOW_TOP", "BOTTLE_IN_HAND", "GRASP_ROTATION", "PREGRASP_OFFSET", "ABOVE", "LIFT", "LOAD_SAG",
        "THUMB_S", "CLOSE_S", "LIFT_S", "THUMB_ROTATED", "CLOSED")
BASE = {key: copy.deepcopy(getattr(grasp, key)) for key in KEYS}
_env = {}  # je Prozess eine Umgebung, wiederverwendet ueber die Aufgaben


def axis_rotation(axis, degrees):
    c, s = np.cos(np.radians(degrees)), np.sin(np.radians(degrees))
    i, j = [(1, 2), (2, 0), (0, 1)][axis]
    r = np.eye(3)
    r[i, i], r[i, j], r[j, i], r[j, j] = c, -s, s, c
    return r


def apply(overrides):
    """Konstanten in g1sim.grasp auf grasp.py zuruecksetzen und die Variante darueberlegen."""
    for key in KEYS:
        setattr(grasp, key, copy.deepcopy(BASE[key]))
    for key, value in overrides.items():
        if key == "ROT_DEG":
            rx, ry, rz = value
            grasp.GRASP_ROTATION = BASE["GRASP_ROTATION"] @ axis_rotation(0, rx) @ axis_rotation(1, ry) @ axis_rotation(2, rz)
        elif key in KEYS and key != "GRASP_ROTATION":
            setattr(grasp, key, np.asarray(value, float) if isinstance(BASE[key], np.ndarray) else float(value))
        else:
            raise KeyError(f"unbekannter Schluessel {key!r}")


def run_chunk(task):
    """Versuche [start, stop) einer Variante. Die Startlagen davor werden nur gezogen, nicht gegriffen."""
    name, overrides, start, stop, seed, randomize, config = task
    apply(overrides)
    key = (seed, json.dumps(config, sort_keys=True))
    if key not in _env:
        _env.clear()
        _env[key] = table_env.TableEnv(seed=seed, variation_config=config)
    env = _env[key]
    env.rng = np.random.default_rng(seed)  # wie frisch gebaut: Versuch i bekommt dieselbe Startlage wie in 54
    for _ in range(start):
        env.reset(randomize=randomize)
    trials = []
    for trial in range(start, stop):
        env.reset(randomize=randomize)
        env.noise_rng = np.random.default_rng([seed, 1, trial])
        bottle_start = env.object_position("red_bottle").round(4).tolist()
        tilt = {}

        def on_tick(phase):
            tilt[phase] = env.object_tilt("red_bottle")

        g = grasp.ScriptedGrasp(env, on_tick=on_tick)
        reach = g.reach_error()
        try:
            result = g.run()
        except (RuntimeError, mink.exceptions.NoSolutionFound) as error:
            result = {"error": f"{type(error).__name__}: {error}"}
        finally:
            g.close()
        result["reach_error_m"] = reach
        checks = grasp_scripted.evaluate(result)
        trials.append({
            "trial": trial,
            "passed": all(checks.values()),
            "first_failed": next((k for k, ok in checks.items() if not ok), None),
            "reach_error_m": reach,
            "tilt_end_close_deg": tilt.get("schliessen"),
            "tilt_end_lift_deg": tilt.get("anheben"),
            "tilt_end_hold_deg": result.get("bottle_tilt_deg"),
            "min_lift_m": result.get("min_lift_during_hold_m"),
            "grasp_pos_error_m": result.get("grasp_pos_error_m"),
            "forbidden_contacts": result.get("forbidden_contacts"),
            # wer die Flasche am Ende des Haltens beruehrt: viele Finger + Handflaeche = Kraftgriff, wenige Spitzen = Pinzettengriff
            "bottle_contacts": sorted({body for body, other in env.robot_contacts() if other == "red_bottle"}),
            "bottle_start": bottle_start,
            "error": result.get("error"),
        })
    return name, trials


def summarize(trials):
    passed = [t for t in trials if t["passed"]]
    reachable = [t for t in trials if t["reach_error_m"] <= grasp_scripted.MAX_GRASP_POS_ERROR]
    tilts = [t["tilt_end_hold_deg"] for t in passed]
    creep = [t["tilt_end_hold_deg"] - t["tilt_end_lift_deg"] for t in passed]
    return {
        "trials": len(trials),
        "passed": len(passed),
        "reachable": len(reachable),
        "reachable_success": len(passed) / len(reachable) if reachable else None,
        "tilt_median_deg": statistics.median(tilts) if tilts else None,
        "tilt_max_deg": max(tilts) if tilts else None,
        "tilt_end_lift_median_deg": statistics.median(t["tilt_end_lift_deg"] for t in passed) if passed else None,
        "creep_during_hold_median_deg": statistics.median(creep) if creep else None,
        "min_lift_m": min(t["min_lift_m"] for t in passed) if passed else None,
        "failures": dict(collections.Counter(t["first_failed"] for t in trials if not t["passed"])),
    }


def parse_value(text):
    parts = [float(p) for p in text.split(",")]
    return parts if len(parts) > 1 else parts[0]


def main():
    parser = argparse.ArgumentParser(description="D2 - Parameterstudie Greifgeometrie")
    parser.add_argument("--variants", type=pathlib.Path, help="JSON {name: {Schluessel: Wert}}")
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="eine Variante direkt angeben")
    parser.add_argument("--base", action="store_true", help="grasp.py unveraendert als Variante 'basis' mitmessen")
    parser.add_argument("--trials", type=int, default=20)
    parser.add_argument("--randomize", action="store_true", help="Zufallsvariation wie 54 --randomize")
    parser.add_argument("--config", type=pathlib.Path, default=randomization.CONFIG)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--jobs", type=int, default=4, help="Prozesse")
    parser.add_argument("--chunk", type=int, default=10, help="Versuche je Aufgabe")
    parser.add_argument("--out", type=pathlib.Path, default=REPO / "runs" / "d2_sweep.json")
    args = parser.parse_args()

    variants = {"basis": {}} if args.base else {}
    if args.variants:
        variants.update(json.loads(args.variants.read_text()))
    if args.set:
        variants["set"] = {k: parse_value(v) for k, v in (s.split("=", 1) for s in args.set)}
    if not variants:
        parser.error("keine Variante: --variants, --set oder --base")
    for overrides in variants.values():
        apply(overrides)  # Tippfehler im Schluessel hier melden, nicht erst im Arbeitsprozess
    apply({})

    config = randomization.load(args.config)
    table_scene.write()
    tasks = [(name, overrides, start, min(start + args.chunk, args.trials), args.seed, args.randomize, config)
             for name, overrides in variants.items() for start in range(0, args.trials, args.chunk)]
    trials = {name: [] for name in variants}
    with concurrent.futures.ProcessPoolExecutor(args.jobs, mp_context=multiprocessing.get_context("spawn")) as pool:
        for name, chunk in pool.map(run_chunk, tasks):
            trials[name].extend(chunk)

    report = {"trials": args.trials, "randomize": args.randomize, "seed": args.seed, "config": str(args.config),
              "variants": {name: {"overrides": variants[name], **summarize(trials[name]), "results": trials[name]}
                           for name in variants}}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False))

    def fmt(value, scale=1.0, digits=0):
        return "-" if value is None else f"{value * scale:.{digits}f}"

    print(f"{'Variante':28s} {'bestanden':>9s} {'erreichbar':>10s} {'Neigung Median/Max':>18s} {'nach Anheben':>12s} "
          f"{'Kriechen':>8s} {'Hub min':>7s}  Fehlschlaege")
    for name, v in report["variants"].items():
        print(f"{name:28s} {v['passed']:>4d}/{v['trials']:<4d} {v['passed']:>4d}/{v['reachable']:<5d} "
              f"{fmt(v['tilt_median_deg']):>9s}/{fmt(v['tilt_max_deg']):<8s} {fmt(v['tilt_end_lift_median_deg']):>12s} "
              f"{fmt(v['creep_during_hold_median_deg'], digits=1):>8s} {fmt(v['min_lift_m'], 100, 1):>7s}  {v['failures']}")
    print(f"gespeichert: {args.out}")


if __name__ == "__main__":
    main()
