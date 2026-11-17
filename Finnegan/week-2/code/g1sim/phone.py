"""Empfaenger fuer die iPhone-App "MuJoCo AR" (Teleop, g1sim/teleop.py): Pose des Handys + zwei Knoepfe per WebSocket.

Protokoll wie im Python-Paket mujoco_ar 1.4.0 (connector.py), das die App mitbringt: Die App verbindet sich mit
ws://<IP des Macs>:<Port> und schickt je Bild ein JSON-Objekt
    {"position": [x, y, z], "rotation": 3x3, "button": bool, "toggle": bool}
in ARKits Weltsystem (x rechts, y oben, z zum Nutzer; festgelegt beim Start der App). Die Rotation kommt spaltenweise,
das Paket transponiert sie - hier ebenso. Zurueck geht nur {"vibrate": true, ...}.
Das Paket selbst nutzen wir nicht: Beim Start beendet es jeden Prozess auf seinem Port hart (kill -9).
"""
import asyncio
import json
import socket
import threading
import time

import numpy as np
import websockets

DEFAULT_PORT = 8888  # Voreinstellung der App


def local_ip():
    """IP des Macs im WLAN (die das Handy erreicht). Eine UDP-Verbindung verschickt dabei nichts."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.connect(("192.0.2.1", 80))  # Adresse aus dem Dokumentationsbereich, wird nie erreicht
            return s.getsockname()[0]
        except OSError:
            return "127.0.0.1"


def parse(message):
    """Nachricht der App -> {"position" (3,), "rotation" (3, 3), "button", "toggle"}; None, wenn keine Pose drin ist."""
    data = json.loads(message)
    if "position" not in data or "rotation" not in data:
        return None
    return {"position": np.asarray(data["position"], dtype=float)[:3],
            "rotation": np.asarray(data["rotation"], dtype=float).reshape(3, 3).T,
            "button": bool(data.get("button", False)),
            "toggle": bool(data.get("toggle", False))}


class PhoneReceiver:
    """WebSocket-Server in einem eigenen Thread; latest() liefert die juengste Pose."""

    def __init__(self, port=DEFAULT_PORT, host="0.0.0.0"):
        self.port, self.host = port, host
        self._latest, self._time = None, 0.0
        self._lock = threading.Lock()
        self._clients = set()
        self._ready = threading.Event()
        self._error = None
        self.loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()
        self._ready.wait(5.0)
        if self._error is not None:
            raise RuntimeError(f"Port {self.port} fuer das iPhone belegt ({self._error}) - anderen mit --phone-port")
        return self

    def _run(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._serve())

    async def _serve(self):
        try:
            self._server = await websockets.serve(self._handle, self.host, self.port)
        except OSError as error:
            self._error = error
            self._ready.set()
            return
        self._ready.set()
        await self._server.wait_closed()

    async def _handle(self, websocket):
        self._clients.add(websocket)
        try:
            async for message in websocket:
                pose = parse(message)
                if pose is not None:
                    with self._lock:
                        self._latest, self._time = pose, time.monotonic()
        except websockets.ConnectionClosed:
            pass
        finally:
            self._clients.discard(websocket)

    @property
    def connected(self):
        return bool(self._clients)

    def latest(self, max_age=0.5):
        """Juengste Pose, oder None, wenn seit max_age s nichts kam (Verbindung weg = Hand bleibt stehen)."""
        with self._lock:
            return self._latest if time.monotonic() - self._time <= max_age else None

    def vibrate(self, duration=0.05):
        message = json.dumps({"vibrate": True, "duration": duration, "intensity": 1.0, "sharpness": 0.5})
        for client in list(self._clients):
            asyncio.run_coroutine_threadsafe(client.send(message), self.loop)

    def close(self):
        if getattr(self, "_server", None) is not None:
            self.loop.call_soon_threadsafe(self._server.close)
        self._thread.join(2.0)
