"""Abnahme Teleop (Stufe 2.1): Aufnahme per Tastatur, Browser-Server, iPhone-Protokoll und Umwandlung - ohne Menschen.

Prueft der Reihe nach:
  1. Ein simulierter Bediener greift per Tastendruck (nur KeyPilot-Eingaben wie im Browser) --episodes Mal die Flasche,
     mit Zufallsvariation. Jede Episode: erfolgreich gespeichert, besteht episode.validate(), Aktionen in den Grenzen
     (Gelenkbereich, Haende 0..1, Arme hoechstens grasp_ik.MAX_JOINT_SPEED)
  2. scripts/57_teleop.py als eigener Prozess: Seite und Kamerabild kommen, eine gehaltene Taste startet die Aufnahme
     und bewegt die Hand, "Verwerfen" setzt zurueck
  3. iPhone: eine Nachricht im Format der App "MuJoCo AR" kommt an; 10 cm Handybewegung nach vorn bei gehaltenem Knopf
     = 10 cm Hand nach vorn
  4. openpi-Umgebung: openpi_g1/convert.py macht aus den Episoden einen LeRobot-Datensatz v2.1 (Ordner unter runs/)
Der simulierte Bediener sieht die Flasche wie ein Mensch im Bild (wahre Lage) und korrigiert nach der gemessenen Hand.
Umgebung ~/dev/g1-ext:
    ~/dev/g1-ext/.venv/bin/python scripts/58_teleop_check.py [--episodes 3] [--seed 0]
Ergebnis: runs/teleop_check.json, Episoden in runs/teleop_check/
"""
import argparse
import asyncio
import json
import os
import pathlib
import shutil
import socket
import subprocess
import sys
import time
import types
import urllib.request
import warnings

import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim import action_space, episode, phone, table_env, table_scene, teleop  # noqa: E402
from g1sim.grasp_ik import MAX_JOINT_SPEED  # noqa: E402
from g1sim.grasp import (ABOVE, BOTTLE_IN_HAND, CLOSE_S, GRASP_BELOW_TOP, GRASP_ROTATION, LIFT,  # noqa: E402
                         PREGRASP_OFFSET, bottle_top)

OUT = REPO / "runs" / "teleop_check"
OPENPI_PY = pathlib.Path.home() / "dev" / "g1-ext" / "openpi" / ".venv" / "bin" / "python"
TOLERANCE = 0.005        # m: so genau faehrt der simulierte Bediener an (gemessene Hand)
FINE_BELOW = 0.03        # m: naeher am Ziel mit Feinsteuerung (Umschalttaste)
SETTLED_TICKS = 10       # so viele Takte ohne Taste am Ziel, dann naechster Schritt
MAX_OPERATOR_S = 40.0    # Sim-Sekunden je Episode, dann gilt sie als gescheitert

warnings.filterwarnings("ignore", message=".*parent has")
warnings.filterwarnings("ignore", message="Attach conflict")
warnings.filterwarnings("ignore", message=".*encountered in matmul")


class SimulatedOperator:
    """Drueckt Tasten wie ein Mensch am Browser: Greifhaltung (G), hinfahren (W/A/S/D/Q/E), greifen (Leertaste),
    anheben (E), speichern (Enter). Fahrziele wie in der Demo-Fabrik (g1sim/grasp.py), nur per Tastatur angefahren."""

    def __init__(self, session):
        self.session, self.arm = session, session.arm
        self.hand_body = session.env.model.body(self.arm.ik.frame).id
        rotation = self.arm.body_rot @ GRASP_ROTATION
        axis = bottle_top(session.env) - [0.0, 0.0, GRASP_BELOW_TOP]
        grasp = axis - rotation @ BOTTLE_IN_HAND
        pregrasp = grasp - PREGRASP_OFFSET * rotation[:, 0]
        self.plan = [("haltung", None), ("ueber", pregrasp + [0, 0, ABOVE]), ("vor", pregrasp), ("ran", grasp),
                     ("zu", CLOSE_S), ("hoch", grasp + [0, 0, LIFT]), ("halten", 1.0), ("speichern", None)]
        self.step, self.wait, self.started, self.started_grip = 0, 0, False, False

    def inputs(self):
        """Eingaben fuer den naechsten Takt (teleop.Inputs)."""
        phase, arg = self.plan[self.step]
        if phase == "haltung":
            if not self.started:
                self.started = True
                return teleop.Inputs(events=("grasp_pose",))
            if self.session.keys.preset is None:
                self._next()
            return teleop.Inputs()
        if phase == "zu":  # Leertaste; warten, bis der Greifwert voll ist (Browser: Balken "Greifen"), dann arg s
            if not self.started_grip:
                self.started_grip = True
                return teleop.Inputs(events=("grip",))
            if self.arm.grip < 0.99:
                return teleop.Inputs()
        if phase in ("zu", "halten"):
            self.wait += 1
            if self.wait >= arg * table_env.CONTROL_HZ:
                self._next()
            return teleop.Inputs()
        if phase == "speichern":
            return teleop.Inputs(events=("save",))
        measured = self.session.env.data.xpos[self.hand_body]
        error = self.arm.body_rot.T @ (arg - measured)  # im Koerpersystem, wie die Tasten
        held = set()
        for axis, (plus, minus) in enumerate((("KeyW", "KeyS"), ("KeyA", "KeyD"), ("KeyE", "KeyQ"))):
            if error[axis] > TOLERANCE:
                held.add(plus)
            elif error[axis] < -TOLERANCE:
                held.add(minus)
        if held and np.abs(error).max() < FINE_BELOW:
            held.add("ShiftLeft")  # kurz vor dem Ziel langsam, wie ein Mensch
        self.wait = 0 if held else self.wait + 1
        if self.wait >= SETTLED_TICKS:  # erst weiter, wenn die Hand ruhig am Ziel steht
            self._next()
        return teleop.Inputs(held=frozenset(held))

    def _next(self):
        self.step, self.wait = self.step + 1, 0


def record_episodes(n, seed):
    """Teil 1. Rueckgabe: Liste je Episode (dict)."""
    if OUT.exists():
        shutil.rmtree(OUT)
    table_scene.write()
    env = table_env.TableEnv(seed=seed)
    session = teleop.TeleopSession(env, OUT, randomize=True)
    results = []
    for i in range(n):
        operator, ticks, start = SimulatedOperator(session), 0, time.perf_counter()
        saved_before = len(session.results)
        while len(session.results) == saved_before and ticks < MAX_OPERATOR_S * table_env.CONTROL_HZ:
            session.tick(operator.inputs())
            ticks += 1
        if len(session.results) == saved_before:
            phase = operator.plan[operator.step][0]
            results.append({"episode": i, "error": f"nach {MAX_OPERATOR_S:.0f} s nicht gespeichert, Phase {phase}"})
            session.tick(teleop.Inputs(events=("discard",)))
            continue
        path, success = session.results[-1]
        ep = episode.load(path)
        info = json.loads(path.with_suffix(".json").read_text())
        hands = ep["actions"][:, episode.HANDS]
        joints = [name for side in action_space.SIDES for name in action_space.SIM_ARM_JOINTS[side]]
        lo, hi = np.array([env.model.jnt_range[env.model.joint(name).id] for name in joints]).T
        results.append({
            "episode": i, "path": str(path.relative_to(REPO)), "success": success, "frames": len(ep["timestamp"]),
            "lift_cm": round(info["lift_m"] * 100, 1), "forbidden_contacts": info["forbidden_contacts"],
            "problems": episode.validate(ep),
            "arm_in_limits": bool(np.all((ep["actions"][:, :14] >= lo - 1e-6) & (ep["actions"][:, :14] <= hi + 1e-6))),
            "hands_in_0_1": bool(hands.min() >= 0.0 and hands.max() <= 1.0),
            "max_arm_step_rad": float(np.abs(np.diff(ep["actions"][:, :14], axis=0)).max()),
            "wall_s": round(time.perf_counter() - start, 1),
        })
        print(f"Episode {i}: {'ok  ' if success else 'FAIL'} {results[-1]}")
    env.close()
    return results


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def http_check():
    """Teil 2: 57_teleop.py im eigenen Prozess ansprechen wie ein Browser."""
    port, out = free_port(), OUT / "http"
    cmd = [sys.executable, str(REPO / "scripts" / "57_teleop.py"), "--port", str(port), "--out", str(out), "--no-browser"]
    proc = subprocess.Popen(cmd, cwd=REPO, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    url = f"http://127.0.0.1:{port}"
    checks = {}
    try:
        status = _wait_for(lambda: _get_json(f"{url}/status"), timeout=90)
        checks["server_startet"] = status is not None
        if not checks["server_startet"]:
            return checks, None
        checks["seite_kommt"] = b"<html" in _get(f"{url}/")
        checks["kamerabild_kommt"] = _first_jpeg(f"{url}/stream.mjpg").startswith(b"\xff\xd8")
        before = status["hand_body_m"]
        for _ in range(30):  # 1 s Taste W halten, wie der Browser: alle ~33 ms den Stand schicken
            _post(f"{url}/input", {"held": ["KeyW"], "events": [], "axes": [], "trigger": 0})
            time.sleep(0.033)
        _post(f"{url}/input", {"held": [], "events": [], "axes": [], "trigger": 0})
        moved = _wait_for(lambda: (s := _get_json(f"{url}/status")) and s["state"] == "aufnahme" and s, timeout=10)
        checks["taste_startet_aufnahme"] = bool(moved)
        checks["hand_faehrt_vor"] = bool(moved) and moved["hand_body_m"][0] > before[0] + 0.01
        _post(f"{url}/input", {"held": [], "events": ["discard"], "axes": [], "trigger": 0})
        reset = _wait_for(lambda: (s := _get_json(f"{url}/status")) and s["state"] == "bereit" and s, timeout=10)
        checks["verwerfen_setzt_zurueck"] = bool(reset) and abs(reset["hand_body_m"][0] - before[0]) < 0.005
        return checks, {"hand_vorher": before, "hand_nach_W": moved and moved["hand_body_m"]}
    finally:
        proc.terminate()
        try:
            proc.wait(10)
        except subprocess.TimeoutExpired:
            proc.kill()


def phone_check():
    """Teil 3: Nachricht wie von der App; PhonePilot rechnet 10 cm vor dem Handy in 10 cm vor der Hand um."""
    port = free_port()
    receiver = phone.PhoneReceiver(port, host="127.0.0.1").start()
    try:
        import websockets

        async def send(messages):
            async with websockets.connect(f"ws://127.0.0.1:{port}") as ws:
                for m in messages:
                    await ws.send(json.dumps(m))
                await asyncio.sleep(0.1)

        identity = np.eye(3).tolist()
        asyncio.run(send([{"position": [0.0, 0.0, 0.0], "rotation": identity, "button": True, "toggle": False}]))
        first = receiver.latest()
    finally:
        receiver.close()

    if first is None:
        return {"nachricht_kommt_an": False}
    arm = types.SimpleNamespace(body_rot=np.eye(3), target=teleop._pose(teleop.mink.SO3.identity(), [0.3, -0.2, 0.9]))
    pilot = teleop.PhonePilot()
    pilot.update(arm, first)  # Knopf gedrueckt: hier greift die Kupplung
    forward = {**first, "position": np.array([0.0, 0.0, -0.10])}  # ARKit: -z = vom Nutzer weg = nach vorn
    goal, _, acted = pilot.update(arm, forward)
    released, _, released_acted = pilot.update(arm, {**forward, "button": False})
    moved = goal.translation() - arm.target.translation()
    return {
        "nachricht_kommt_an": first["button"] is True,
        "10cm_vor_ist_10cm_vor": bool(np.allclose(moved, [0.10, 0.0, 0.0], atol=1e-9)) and acted,
        "loslassen_haelt_an": not released_acted
                              and bool(np.allclose(released.translation(), arm.target.translation())),
    }


def convert_check(paths):
    """Teil 4: Umwandlung in der openpi-Umgebung, in einen eigenen Ordner (nicht in den echten Datensatz)."""
    if not OPENPI_PY.is_file():
        return {"openpi_umgebung_da": False}, "openpi-Umgebung fehlt"
    env = dict(os.environ, HF_LEROBOT_HOME=str(OUT / "lerobot"))
    cmd = [str(OPENPI_PY), "-m", "openpi_g1.convert", *map(str, paths), "--repo-id", "g1_brainco/teleop_check",
           "--overwrite"]
    proc = subprocess.run(cmd, cwd=REPO, env=env, capture_output=True, text=True, timeout=600)
    info = OUT / "lerobot" / "g1_brainco" / "teleop_check" / "meta" / "info.json"
    episodes = json.loads(info.read_text())["total_episodes"] if info.is_file() else 0
    return {"umwandlung_ohne_fehler": proc.returncode == 0, "alle_episoden_im_datensatz": episodes == len(paths)}, \
        (proc.stdout + proc.stderr)[-1500:]


def _get(url, timeout=5):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read()


def _get_json(url):
    try:
        return json.loads(_get(url))
    except OSError:
        return None


def _post(url, data):
    req = urllib.request.Request(url, json.dumps(data).encode(), {"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
        return r.read()


def _first_jpeg(url):
    """Erstes vollstaendiges JPEG (Start FFD8 bis Ende FFD9) aus dem MJPEG-Strom."""
    with urllib.request.urlopen(url, timeout=30) as r:
        buf = b""
        while True:
            start = buf.find(b"\xff\xd8")
            end = buf.find(b"\xff\xd9", start + 2) if start >= 0 else -1
            if end >= 0:
                return buf[start:end + 2]
            chunk = r.read(4096)
            if not chunk:
                return b""
            buf += chunk


def _wait_for(fn, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        value = fn()
        if value:
            return value
        time.sleep(0.2)
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--episodes", type=int, default=3)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    print(f"== 1. Simulierter Bediener, {args.episodes} Episoden mit Zufallsvariation")
    episodes = record_episodes(args.episodes, args.seed)
    ok_paths = [REPO / e["path"] for e in episodes if e.get("success")]
    checks = {
        "alle_episoden_erfolgreich": len(ok_paths) == args.episodes,
        "alle_episoden_gueltig": all(e.get("problems") == [] for e in episodes),
        "arme_in_gelenkgrenzen": all(e.get("arm_in_limits") for e in episodes),
        "haende_0_bis_1": all(e.get("hands_in_0_1") for e in episodes),
        "arme_hoechstens_3_rad_s": all(e.get("max_arm_step_rad", 1.0) <= MAX_JOINT_SPEED * teleop.DT + 1e-5
                                       for e in episodes),
    }
    print("== 2. Browser-Server (57_teleop.py)")
    http, http_info = http_check()
    checks.update(http)
    print("== 3. iPhone-Protokoll")
    checks.update(phone_check())
    print("== 4. Umwandlung in LeRobot v2.1 (openpi-Umgebung)")
    convert, convert_log = convert_check(ok_paths) if ok_paths else ({"umwandlung_ohne_fehler": False}, "keine Episoden")
    checks.update(convert)

    report = {"episodes": episodes, "http": http_info, "convert_log": convert_log, "checks": checks,
              "passed": all(checks.values())}
    out = REPO / "runs" / "teleop_check.json"
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False, default=str))
    for name, ok in checks.items():
        print(f"  {'ok  ' if ok else 'FAIL'} {name}")
    print(f"gespeichert: {out.relative_to(REPO)}")
    print("ABNAHME BESTANDEN" if report["passed"] else "ABNAHME NICHT BESTANDEN")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
