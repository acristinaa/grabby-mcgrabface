"""Stufe 1.4 zum Anschauen: einen Greifversuch rechnen und die Stellung jedes Regeltakts speichern.

Das MuJoCo-Fenster friert in ~/dev/g1-ext ein, die Projekt-Umgebung zeigt es - scripts/56_watch_grasp.py startet dieses
Skript selbst und spielt das Ergebnis ab. Gespeichert werden qpos je Regeltakt (30 pro Sim-Sekunde), die Phase und die
sichtbaren Modellwerte, die reset(randomize=True) veraendern kann, damit die Wiedergabe genau diesen Versuch zeigt.
Umgebung ~/dev/g1-ext:
    ~/dev/g1-ext/.venv/bin/python scripts/55_record_grasp.py [--randomize] [--seed N] [--out runs/live/greifen.npz]
"""
import argparse
import pathlib
import sys
import warnings

import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim import SIM_DIR, table_env, table_scene  # noqa: E402
from g1sim.grasp import ScriptedGrasp  # noqa: E402

OUT = REPO / "runs" / "live" / "greifen.npz"
# Sichtbare Modellwerte, die eine Zufallsvariation aendern kann: Standpose, Farben, Licht, Kameras
MODEL_FIELDS = ("body_pos", "body_quat", "geom_rgba", "light_pos", "light_dir", "light_diffuse", "cam_pos", "cam_quat")

warnings.filterwarnings("ignore", message=".*parent has")
warnings.filterwarnings("ignore", message="Attach conflict")
warnings.filterwarnings("ignore", message=".*encountered in matmul")  # Fehlalarm numpy + Apple Accelerate


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--randomize", action="store_true", help="zufaellige Startlage wie in der Auswertung")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", type=pathlib.Path, default=OUT)
    args = parser.parse_args()

    table_scene.write()
    env = table_env.TableEnv(seed=args.seed)
    env.reset(randomize=args.randomize)
    qpos, times, phases = [env.data.qpos.copy()], [float(env.data.time)], ["start"]

    def on_tick(phase):
        qpos.append(env.data.qpos.copy())
        times.append(float(env.data.time))
        phases.append(phase)

    grasp = ScriptedGrasp(env, on_tick=on_tick)
    try:
        result = grasp.run()
        print(f"Flasche gehalten: min {result['min_lift_during_hold_m'] * 100:.1f} cm ueber dem Tisch, "
              f"Neigung {result['bottle_tilt_deg']:.0f} Grad, verbotene Kontakte {result['forbidden_contacts']}")
    except RuntimeError as error:  # auch ein Fehlversuch ist sehenswert
        print(f"Versuch abgebrochen: {error}")
    finally:
        grasp.close()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.out, qpos=np.array(qpos), time=np.array(times), phase=np.array(phases),
                        scene=str(SIM_DIR / table_scene.SCENE_FILE),
                        **{f"model_{name}": getattr(env.model, name) for name in MODEL_FIELDS})
    env.close()
    print(f"gespeichert: {args.out} ({len(qpos)} Bilder, {times[-1] - times[0]:.1f} s Sim-Zeit)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
