"""Greifen ohne KI (Stufe 1.4) im MuJoCo-Fenster anschauen - mit Rucksack und D455-Kopfmodul.

Kein trainiertes Modell: Datensatz (Stufe 2.1) und Training (2.2) gibt es noch nicht. Gezeigt wird der Greifablauf, aus
dem spaeter die Demos fuer pi0.5 entstehen.

IN ARBEIT: Das Fenster reagierte am 2026-09-16 auch in der Projekt-Umgebung nicht (Prozess liess sich mit kill beenden).
Bis das geklaert ist, zum Anschauen Videos nutzen (54 --video). --no-window prueft nur Aufnahme und Wiedergabe.

Zwei Umgebungen: Das Fenster friert in ~/dev/g1-ext ein, in der Projekt-Umgebung lief es (Phase 0). Die Greif-Rechnung
braucht aber ~/dev/g1-ext (mink, LeRobot-Sim). Deshalb rechnet dieses Skript zuerst einen Versuch mit
scripts/55_record_grasp.py (~10 s) und spielt ihn dann in Echtzeit ab, in Schleife. Die Stellungen stammen aus der
Physik-Rechnung, das Fenster zeigt sie nur.

Start in der Projekt-Umgebung, immer mit mjpython:
    .venv/bin/mjpython scripts/56_watch_grasp.py                feste Flaschenposition
    .venv/bin/mjpython scripts/56_watch_grasp.py --randomize    zufaellige Startlage
    .venv/bin/mjpython scripts/56_watch_grasp.py --replay       letzten Versuch nur abspielen
    Optionen: --seed N   --speed 0.5 (Zeitlupe)   --no-window (nur pruefen, ohne Fenster)
Im Fenster: linke Maustaste drehen, rechte verschieben, Mausrad zoomen. Links unter "Rendering" bei "Camera"
"head_camera" waehlen = Blick der D455. Fenster schliessen beendet das Skript.
"""
import argparse
import contextlib
import pathlib
import subprocess
import sys
import time

import mujoco
import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
RECORDING = REPO / "runs" / "live" / "greifen.npz"
G1EXT_PYTHON = pathlib.Path.home() / "dev" / "g1-ext" / ".venv" / "bin" / "python"
PAUSE_AT_END_S = 1.5


def record(args):
    cmd = [str(G1EXT_PYTHON), str(REPO / "scripts" / "55_record_grasp.py"),
           "--seed", str(args.seed), "--out", str(args.file)]
    if args.randomize:
        cmd.append("--randomize")
    print("Rechne einen Greifversuch in ~/dev/g1-ext (~10 s) ...", flush=True)
    subprocess.run(cmd, cwd=REPO, check=True)


def load(path):
    """Szene laden und auf den aufgenommenen Versuch einstellen. Rueckgabe: model, data, Aufnahme."""
    rec = np.load(path)
    model = mujoco.MjModel.from_xml_path(str(rec["scene"]))
    if model.nq != rec["qpos"].shape[1]:
        raise RuntimeError(f"Szene hat nq {model.nq}, Aufnahme {rec['qpos'].shape[1]} - ohne --replay neu rechnen")
    for key in rec.files:
        if key.startswith("model_"):
            getattr(model, key.removeprefix("model_"))[:] = rec[key]
    return model, mujoco.MjData(model), rec


def frames(model, data, rec, speed, running, lock=contextlib.nullcontext):
    """Aufnahme einmal in Echtzeit / speed abspielen. Liefert je Bild die Phase; lock schuetzt data vor dem Fenster."""
    t = rec["time"] - rec["time"][0]
    start = time.perf_counter()
    for k in range(len(t)):
        if not running():
            return
        with lock():
            data.qpos[:] = rec["qpos"][k]
            data.time = t[k]
            mujoco.mj_forward(model, data)
        yield str(rec["phase"][k])
        wait = start + t[k] / speed - time.perf_counter()
        if wait > 0:
            time.sleep(wait)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--randomize", action="store_true")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--replay", action="store_true", help="nicht neu rechnen, --file abspielen")
    parser.add_argument("--file", type=pathlib.Path, default=RECORDING)
    parser.add_argument("--no-window", action="store_true", help="einmal ohne Fenster durchlaufen (Pruefung)")
    args = parser.parse_args()

    if not args.replay:
        record(args)
    model, data, rec = load(args.file)
    duration = rec["time"][-1] - rec["time"][0]
    print(f"Wiedergabe: {len(rec['qpos'])} Bilder, {duration:.1f} s Sim-Zeit, Tempo {args.speed}x")

    if args.no_window:
        phases = list(dict.fromkeys(frames(model, data, rec, speed=1e9, running=lambda: True)))
        print("Phasen:", " -> ".join(phases))
        return 0

    import mujoco.viewer  # nur mit mjpython

    with mujoco.viewer.launch_passive(model, data) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:] = [0.35, 0.0, 0.95]
            viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 1.8, 210.0, -25.0
        while viewer.is_running():
            for phase in frames(model, data, rec, args.speed, viewer.is_running, viewer.lock):
                with viewer.lock():
                    viewer.set_texts((None, int(mujoco.mjtGridPos.mjGRID_TOPLEFT), "Phase", phase))
                viewer.sync()
            time.sleep(PAUSE_AT_END_S)
    return 0


if __name__ == "__main__":
    sys.exit(main())
