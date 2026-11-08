"""Stufe 1.4 / 1.5 - Flasche greifen ohne KI: Abnahme und Auswertung.

Fuehrt den Ablauf aus g1sim/grasp.py mehrmals auf der Greifszene aus und prueft je Versuch (in dieser Reihenfolge):
  0. Flasche in Reichweite: die IK erreicht die Greifpose der wahren Flasche auf <= 1 cm (Eigenschaft des Starts)
  1. Kopfkamera findet den Flaschendeckel beim zweiten Hinsehen auf <= 1,5 cm genau - daraus kommt die Greifpose.
     Das erste Hinsehen (nur grobe Anfahrt; seit L3a mit der Hand aus dem Bild) wird mitgeschrieben, nicht bewertet
  2. Hand steht vor dem Schliessen auf <= 1 cm und <= 10 Grad an der Greifpose
  3. kein Kontakt des Roboters mit Tisch, Tasse oder Wuerfel
  4. Flasche >= 5 cm angehoben und 5 s gehalten (Mindesthoehe ueber die ganze Haltezeit)
Dazu Bilder der rechten Handgelenkkamera im ersten Versuch (nur zur Info, kein Kriterium).
Umgebung ~/dev/g1-ext:
    Abnahme 1.4, feste Position, alle Versuche muessen bestehen:
        ~/dev/g1-ext/.venv/bin/python scripts/54_grasp_scripted.py [--video] [--lock-wrist]
    Auswertung 1.5b, Zufallsvariation nach configs/randomization.json, Ergebnis ist die Erfolgsquote:
        ~/dev/g1-ext/.venv/bin/python scripts/54_grasp_scripted.py --randomize --trials 50 --out runs/stufe1_5b_alles.json
    nur einzelne Gruppen (objects, stand, physics, image), z. B. Fehlschlaege durch die Standposition allein:
        ~/dev/g1-ext/.venv/bin/python scripts/54_grasp_scripted.py --vary stand --trials 50 --out runs/stufe1_5b_stand.json
Ausgabe, benannt nach --out: runs/<name>.json, docs/<name>_handkamera.png, mit --video docs/videos/<name>.mp4
"""
import argparse
import collections
import json
import pathlib
import sys
import warnings

import cv2
import mink
import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim import labview, randomization, table_env, table_scene  # noqa: E402
from g1sim.grasp import ScriptedGrasp  # noqa: E402
from g1sim.perception import red_mask  # noqa: E402
from g1sim.video import VideoRecorder  # noqa: E402

OUT = REPO / "runs" / "stufe1_4_greifen.json"

MIN_LIFT = 0.05
MAX_PERCEPTION_ERROR = 0.015
MAX_GRASP_POS_ERROR = 0.01
MAX_GRASP_ROT_ERROR_DEG = 10.0
# Handgelenkkamera: Bild zu Beginn dieser Phasen = Start, neben der Flasche, vor dem Schliessen, beim Halten
SNAPSHOTS = {"anfahren": "Start", "heranfahren": "neben der Flasche", "daumen_drehen": "vor dem Schliessen",
             "halten": "Halten"}

warnings.filterwarnings("ignore", message=".*parent has")
# numpy 2 mit Apple Accelerate meldet bei Matrixprodukten falsche Ueberlauf-Warnungen; die Ergebnisse stimmen
warnings.filterwarnings("ignore", message=".*encountered in matmul")


def evaluate(result):
    """Kriterien je Versuch, in der Reihenfolge von Ursache zu Folge - das erste nicht erfuellte ist die Fehlerursache."""
    in_reach = {"flasche_in_reichweite": result["reach_error_m"] <= MAX_GRASP_POS_ERROR}
    if "error" in result:
        return {**in_reach, "ablauf_ohne_fehler": False}
    return {
        **in_reach,
        "kopfkamera_findet_flasche": result["perception_error_m"] <= MAX_PERCEPTION_ERROR,
        "hand_an_greifpose": result["grasp_pos_error_m"] <= MAX_GRASP_POS_ERROR
                             and result["grasp_rot_error_deg"] <= MAX_GRASP_ROT_ERROR_DEG,
        "kein_kontakt_tisch_tasse_wuerfel": not result["forbidden_contacts"],
        "flasche_5_cm_angehoben_5_s_gehalten": result["min_lift_during_hold_m"] >= MIN_LIFT,
    }


def main():
    parser = argparse.ArgumentParser(description="Stufe 1.4 / 1.5 - Flasche greifen ohne KI")
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--randomize", action="store_true",
                        help="Zufallsvariation mit allen Gruppen, die in --config eingeschaltet sind (Stufe 1.5b)")
    parser.add_argument("--vary", help=f"nur diese Gruppen variieren, mit Komma getrennt: {','.join(randomization.GROUPS)}")
    parser.add_argument("--config", type=pathlib.Path, default=randomization.CONFIG, help="Bereiche der Zufallsvariation")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--video", action="store_true", help="ersten Versuch als Video speichern")
    parser.add_argument("--lock-wrist", action="store_true", help="Handgelenk Pitch/Yaw sperren (G1 mit 23 DoF)")
    parser.add_argument("--out", type=pathlib.Path, default=OUT)
    parser.add_argument("--watch", type=float, metavar="SPEED",
                        help="lab view: slow down to SPEED x real time (1 = real time) so the phases can be followed")
    parser.add_argument("--no-labview", action="store_true", help="write no lab view files (runs/labview/)")
    args = parser.parse_args()
    image = REPO / "docs" / f"{args.out.stem}_handkamera.png"
    video_path = REPO / "docs" / "videos" / f"{args.out.stem}.mp4"
    groups = args.vary.split(",") if args.vary else None
    config = randomization.load(args.config, groups)
    randomize = args.randomize or groups is not None
    varied = randomization.enabled(config) if randomize else []

    table_scene.write()
    env = table_env.TableEnv(seed=args.seed, variation_config=config)
    shots, results = {}, []
    # Lab view (scripts/65_labview.py): the live image borrows the head-camera renderer of the current trial's Locator
    reporter = None if args.no_labview else labview.Reporter(
        script="54", task="grasp_lift", object="bottle", trials=args.trials, argv=sys.argv[1:],
        renderer_getter=lambda: grasp.locator.renderer, sim_time_getter=lambda: env.data.time, watch=args.watch,
        data=env.data, track_body="red_bottle")
    for trial in range(args.trials):
        env.reset(randomize=randomize)
        if reporter:
            reporter.trial_start(trial)
        bottle_start = env.object_position("red_bottle")
        video = (VideoRecorder(env.model, env.data, video_path, track_body="red_bottle", distance=1.1, azimuth=225.0,
                               elevation=-20.0) if args.video and trial == 0 else None)

        def on_tick(phase, video=video, first=trial == 0):
            if video:
                video.grab()
            if first and phase in SNAPSHOTS and phase not in shots:
                shots[phase] = env.render("right_wrist_camera")
            if reporter:
                reporter.on_tick(phase)

        grasp = ScriptedGrasp(env, lock_wrist=args.lock_wrist, on_tick=on_tick)
        reach_error = grasp.reach_error()
        try:
            result = grasp.run()
        except (RuntimeError, mink.exceptions.NoSolutionFound) as error:  # ein Versuch darf die Auswertung nicht beenden
            result = {"error": f"{type(error).__name__}: {error}"}
        finally:
            grasp.close()
            if video:
                video.close()
        result["reach_error_m"] = reach_error
        result["bottle_start"] = bottle_start.round(4).tolist()
        result["variation"] = env.variation
        result["start_draws"] = env.draws
        result["checks"] = evaluate(result)
        result["passed"] = all(result["checks"].values())
        results.append(result)
        if reporter:
            reporter.trial_end(result["passed"], next((k for k, ok in result["checks"].items() if not ok), None),
                               {"reachable": result["checks"]["flasche_in_reichweite"], "error": result.get("error")})
        stand = env.variation["stand"]
        summary = (f"Reichweite {reach_error * 1000:.0f} mm, Stand {stand['dx_m'] * 100:+.0f}/{stand['dy_m'] * 100:+.0f} cm "
                   f"{stand['yaw_deg']:+.0f} Grad | ")
        summary += ("FEHLER " + result["error"] if "error" in result else
                   f"Hub min {result['min_lift_during_hold_m'] * 100:.1f} cm, "
                   f"Neigung {result['bottle_tilt_deg']:.0f} Grad, "
                   f"Wahrnehmung {result['perception_error_m'] * 1000:.1f} mm "
                   f"(erstes Hinsehen {result['first_look_error_m'] * 1000:.0f} mm), "
                   f"Hand {result['grasp_pos_error_m'] * 1000:.1f} mm / {result['grasp_rot_error_deg']:.1f} Grad, "
                   f"verbotene Kontakte {result['forbidden_contacts']}")
        print(f"Versuch {trial + 1}: {'ok  ' if result['passed'] else 'FAIL'} {summary}")
    env.close()

    tiles = []
    for phase, label in SNAPSHOTS.items():
        tile = cv2.resize(shots[phase], (320, 320)) if phase in shots else np.zeros((320, 320, 3), np.uint8)
        cv2.putText(tile, label, (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        tiles.append(tile)
    image.parent.mkdir(exist_ok=True)
    cv2.imwrite(str(image), np.concatenate(tiles, 1)[:, :, ::-1])

    passed = sum(r["passed"] for r in results)
    failures = collections.Counter(next(k for k, ok in r["checks"].items() if not ok) for r in results if not r["passed"])
    report = {
        "trials": args.trials,
        "passed_trials": passed,
        "success_rate": passed / args.trials,
        "randomize": randomize,
        "varied_groups": varied,
        "variation_config": {"path": str(args.config), **config},
        "rejected_starts": sum(r["start_draws"] - 1 for r in results),
        "seed": args.seed,
        "lock_wrist": args.lock_wrist,
        "failures_by_first_failed_check": dict(failures),
        "criteria": {"min_lift_m": MIN_LIFT, "hold_s": 5.0, "max_perception_error_m": MAX_PERCEPTION_ERROR,
                     "max_grasp_pos_error_m": MAX_GRASP_POS_ERROR, "max_grasp_rot_error_deg": MAX_GRASP_ROT_ERROR_DEG},
        # Anteil roter Pixel = wie viel Flasche die Handgelenkkamera sieht (nur zur Info)
        "wrist_camera_red_fraction": {SNAPSHOTS[p]: round(float(red_mask(img).mean()), 3) for p, img in shots.items()},
        "results": results,
        "passed": passed == args.trials,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False))
    if reporter:
        reporter.finish(report_path=args.out)
    print("Handgelenkkamera, Anteil rote Pixel:", report["wrist_camera_red_fraction"])
    print(f"Variiert: {varied or 'nichts'}; verworfene Starts (zu nah an Tisch/Objekt): {report['rejected_starts']}")
    print(f"{passed}/{args.trials} Versuche bestanden ({report['success_rate']:.0%})")
    if failures:
        print("Fehlschlaege nach Ursache (erstes nicht erfuelltes Kriterium):", dict(failures))
    print(f"gespeichert: {args.out}, {image.relative_to(REPO)}")
    if randomize:
        return 0  # Auswertung: das Ergebnis ist die Quote, kein Bestanden/Nicht bestanden
    print("ABNAHME BESTANDEN" if report["passed"] else "ABNAHME NICHT BESTANDEN")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
