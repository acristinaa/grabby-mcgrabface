"""Teleop im Browser (Stufe 2.1): die rechte Hand des G1 per Tastatur, Gamepad oder iPhone fuehren und Demos aufnehmen.

Umgebung ~/dev/g1-ext:
    ~/dev/g1-ext/.venv/bin/python scripts/57_teleop.py [--randomize] [--phone] [--out runs/teleop/<name>]
    -> oeffnet http://127.0.0.1:8010 im Browser. Beenden mit Ctrl+C im Terminal.

Die Sim laeuft in diesem Prozess (Hauptthread: Physik und Bilder, das OpenGL des Macs verlangt einen Thread), der
Browser zeigt die Bilder und schickt die Tasten. Das MuJoCo-Fenster (mjpython) braucht es nicht, es friert auf diesem
Mac ein. Aufgenommen wird in Sim-Zeit mit 30 Hz. Waehrend der Aufnahme rendert jeder Takt drei Kamerabilder, die Sim
laeuft dann langsamer als Echtzeit (Zeitlupe, die Aufnahme stimmt trotzdem).
Sicherheit: Kommt 0,5 s lang keine Eingabe vom Browser (Tab zu, Verbindung weg), bleibt die Hand stehen.
iPhone (--phone): App "MuJoCo AR" im selben WLAN, IP und Port aus der Ausgabe eintragen, die App mit dem Handy nach vorn
(in Blickrichtung des Roboters) starten. Knopf halten = Hand folgt dem Handy, Schalter = greifen.
Episoden: <out>/episode_NNN.npz (+ .json), danach openpi_g1/convert.py (nur erfolgreiche kommen in den Datensatz).
"""
import argparse
import datetime
import http.server
import json
import pathlib
import sys
import threading
import time
import warnings
import webbrowser

import cv2
import mujoco
import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim import phone, table_env, table_scene, teleop  # noqa: E402

PAGE = REPO / "g1sim" / "teleop_page.html"
# Uebersichtskamera: Blickpunkt im Koerpersystem vor dem Roboter auf dem Tisch, Blickwinkel je Ansicht (Grad, m)
LOOK_AT_BODY = np.array([0.45, -0.12, 0.02])
VIEWS = {"schulter": (20.0, -40.0, 1.2), "seite": (90.0, -12.0, 1.1), "oben": (0.0, -89.0, 1.0)}
VIEW_SIZE = (480, 640)   # Hoehe, Breite der Uebersicht
TILE = 240               # Kopf- und Handkamera daneben
VIEW_EVERY = 2           # Browserbild jeden zweiten Takt (15 Bilder/s Sim-Zeit)
INPUT_TIMEOUT = 0.5      # s ohne Eingabe vom Browser -> nichts gedrueckt

warnings.filterwarnings("ignore", message=".*parent has")
warnings.filterwarnings("ignore", message="Attach conflict")
warnings.filterwarnings("ignore", message=".*encountered in matmul")


class Shared:
    """Austausch zwischen Sim-Schleife (Hauptthread) und den HTTP-Threads."""

    def __init__(self):
        self.lock = threading.Condition()
        self.frame, self.frame_id, self.status = b"", 0, {}
        self.held, self.axes, self.trigger, self.input_time = frozenset(), (), 0.0, 0.0
        self.events, self.view = [], "schulter"

    def put_input(self, data):
        with self.lock:
            self.held = frozenset(str(k) for k in data.get("held", []))
            self.axes = tuple(float(a) for a in data.get("axes", [])[:4])
            self.trigger = float(data.get("trigger", 0.0))
            self.events += [e for e in data.get("events", []) if e in teleop.EVENTS]
            if data.get("view") in VIEWS:
                self.view = data["view"]
            self.input_time = time.monotonic()

    def take_input(self):
        """Eingaben fuer einen Takt; veraltete gehaltene Tasten zaehlen nicht (Totmann)."""
        with self.lock:
            fresh = time.monotonic() - self.input_time <= INPUT_TIMEOUT
            events, self.events = tuple(self.events), []
            if not fresh:
                return teleop.Inputs(events=events), self.view
            return teleop.Inputs(held=self.held, events=events, axes=self.axes, trigger=self.trigger), self.view

    def put_frame(self, jpeg):
        with self.lock:
            self.frame, self.frame_id = jpeg, self.frame_id + 1
            self.lock.notify_all()

    def wait_frame(self, last_id, timeout=2.0):
        with self.lock:
            self.lock.wait_for(lambda: self.frame_id != last_id, timeout)
            return self.frame, self.frame_id


def make_handler(shared, page):
    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path == "/":
                self._send(200, page, "text/html; charset=utf-8")
            elif self.path == "/status":
                with shared.lock:
                    body = json.dumps(shared.status).encode()
                self._send(200, body, "application/json")
            elif self.path.startswith("/stream.mjpg"):
                self._stream()
            else:
                self._send(404, b"nicht gefunden", "text/plain")

        def do_POST(self):
            if self.path != "/input":
                return self._send(404, b"nicht gefunden", "text/plain")
            length = int(self.headers.get("Content-Length", 0))
            try:
                shared.put_input(json.loads(self.rfile.read(length) or b"{}"))
            except (ValueError, TypeError):
                return self._send(400, b"kein gueltiges JSON", "text/plain")
            self._send(204, b"", "text/plain")

        def _send(self, code, body, content_type):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last = -1
            try:
                while True:
                    frame, last = shared.wait_frame(last)
                    if frame:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(frame)
                                         + frame + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def letterbox(image, size):
    """Bild ohne Verzerren auf size x size bringen, Rest schwarz - so, wie openpi es fuer pi0.5 tut (resize_with_pad).
    Die Kachel zeigt also genau, was die Policy von der 16:10-Kopfkamera sieht."""
    h, w = image.shape[:2]
    scale = size / max(h, w)
    small = cv2.resize(image, (round(w * scale), round(h * scale)), interpolation=cv2.INTER_AREA)
    tile = np.zeros((size, size, 3), dtype=np.uint8)
    top, left = (size - small.shape[0]) // 2, (size - small.shape[1]) // 2
    tile[top:top + small.shape[0], left:left + small.shape[1]] = small
    return tile


class View:
    """Browserbild: Uebersicht (umschaltbar) links, Kopf- und rechte Handkamera rechts."""

    def __init__(self, env):
        self.env = env
        self.renderer = mujoco.Renderer(env.model, *VIEW_SIZE)
        self.camera = mujoco.MjvCamera()

    def compose(self, session, obs, view):
        arm = session.arm
        self.camera.lookat[:] = arm.body_pos + arm.body_rot @ LOOK_AT_BODY
        self.camera.azimuth, self.camera.elevation, self.camera.distance = VIEWS[view]
        self.renderer.update_scene(self.env.data, self.camera)
        overview = self.renderer.render().copy()
        # Kopfkamera aus der Aufnahme, wenn es eine gibt. Die Handkamera nur zum Anschauen: Sie wird nicht aufgenommen,
        # weil der echte G1 keine hat (Entscheidung D3).
        obs = {"head_image": obs["head_image"] if obs is not None else self.env.render("head_camera"),
               "right_wrist_image": self.env.render("right_wrist_camera")}
        tiles = []
        for key, label in (("head_image", "Kopfkamera D455 (Aufnahme)"), ("right_wrist_image", "Handkamera (nur Anzeige)")):
            tile = letterbox(obs[key], TILE)
            cv2.putText(tile, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            tiles.append(tile)
        if session.state == "aufnahme":
            cv2.circle(overview, (22, 22), 9, (230, 30, 30), -1)
            cv2.putText(overview, "REC", (38, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (230, 30, 30), 2, cv2.LINE_AA)
        canvas = np.concatenate([overview, np.concatenate(tiles, axis=0)], axis=1)
        ok, jpeg = cv2.imencode(".jpg", canvas[..., ::-1], [cv2.IMWRITE_JPEG_QUALITY, 80])
        return jpeg.tobytes()

    def close(self):
        self.renderer.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=8010)
    parser.add_argument("--out", type=pathlib.Path,
                        default=REPO / "runs" / "teleop" / datetime.datetime.now().strftime("%Y-%m-%d_%H%M"))
    parser.add_argument("--randomize", action="store_true", help="Zufallsvariation (configs/randomization.json), nur "
                                                                  "Szenen mit erreichbarer Flasche")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--phone", action="store_true", help="iPhone-App MuJoCo AR annehmen (Port im WLAN offen)")
    parser.add_argument("--phone-port", type=int, default=phone.DEFAULT_PORT)
    parser.add_argument("--phone-scale", type=float, default=1.0, help="Handbewegung je Handybewegung")
    parser.add_argument("--no-browser", action="store_true", help="Browser nicht selbst oeffnen")
    args = parser.parse_args()

    table_scene.write()
    env = table_env.TableEnv(seed=args.seed)
    session = teleop.TeleopSession(env, args.out, randomize=args.randomize, phone_scale=args.phone_scale)
    receiver = phone.PhoneReceiver(args.phone_port).start() if args.phone else None
    phone_address = f"{phone.local_ip()}:{args.phone_port}" if receiver else None
    view = View(env)
    shared = Shared()
    server = http.server.ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(shared, PAGE.read_bytes()))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{args.port}"
    print(f"Teleop: {url}   Episoden: {args.out}", flush=True)
    if receiver:
        print(f"iPhone: App MuJoCo AR -> IP und Port {phone_address} (Mac und Handy im selben WLAN)", flush=True)
    if not args.no_browser:
        webbrowser.open(url)

    tick, rate, last = 0, 1.0, time.perf_counter()
    try:
        while True:
            start = time.perf_counter()
            inputs, view_name = shared.take_input()
            if receiver:
                inputs = teleop.Inputs(inputs.held, inputs.events, inputs.axes, inputs.trigger, receiver.latest())
            obs = session.tick(inputs)
            if receiver and session.touched:
                receiver.vibrate()
            if tick % VIEW_EVERY == 0:
                shared.put_frame(view.compose(session, obs, view_name))
            now = time.perf_counter()
            time.sleep(max(0.0, teleop.DT - (now - start)))
            now, elapsed = time.perf_counter(), time.perf_counter() - last
            rate, last = 0.95 * rate + 0.05 * (teleop.DT / elapsed), now
            with shared.lock:
                shared.status = {**session.status(), "view": view_name, "realtime": round(rate, 2),
                                 "phone": {"enabled": bool(receiver), "connected": bool(receiver and receiver.connected),
                                           "address": phone_address}}
            tick += 1
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        view.close()
        env.close()
        if receiver:
            receiver.close()
    ok = sum(success for _, success in session.results)
    print(f"\n{len(session.results)} Episoden gespeichert, davon {ok} erfolgreich: {args.out}")
    if session.results:
        print("Datensatz daraus (openpi-Umgebung):\n  ~/dev/g1-ext/openpi/.venv/bin/python -m openpi_g1.convert "
              f"{args.out}/episode_*.npz --repo-id g1_brainco/teleop")
    return 0


if __name__ == "__main__":
    sys.exit(main())
