"""SimSession - Physik- und Renderschleife in einem eigenen Thread.

Warum alles in EINEM Thread: der OpenGL-Kontext von mujoco.Renderer gehoert dem
Thread, der ihn anlegt. Wird aus einem anderen gerendert, scheitert es auf macOS
auf schwer nachvollziehbare Weise. Also: der Worker steppt UND rendert, alle
anderen reden nur ueber Queue (rein) und gesicherte Slots (raus).
"""
import io
import queue
import threading
import time

from PIL import Image

from .drivers import MujocoDriver

LOOP_HZ = 60.0          # Takt der Schleife
RENDER_EVERY = 2        # jede 2. Runde rendern -> ~30 fps
JPEG_QUALITY = 80


class SimSession:
    def __init__(self, width=960, height=720):
        self.width = width
        self.height = height

        self._cmds = queue.Queue()
        self._lock = threading.Lock()
        self._frame = None          # JPEG-Bytes
        self._state = {}
        self._meta = {}
        self._error = None

        self._paused = False
        self._camera = None
        self._cam_params = {"lookat": [0.35, 0.0, 0.95], "distance": 1.8,
                            "azimuth": 135.0, "elevation": -15.0}

        self._running = threading.Event()
        self._ready = threading.Event()
        self._thread = None

    # --- Steuerung von aussen ---

    def start(self):
        if self._thread is not None:
            return
        self._running.set()
        self._thread = threading.Thread(target=self._run, name="sim", daemon=True)
        self._thread.start()
        self._ready.wait(timeout=30.0)
        if self._error:
            raise RuntimeError(self._error)

    def stop(self):
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None

    def send(self, cmd):
        """Kommando einreihen. Wird im Worker-Thread ausgefuehrt."""
        self._cmds.put(cmd)

    def frame(self):
        with self._lock:
            return self._frame

    def state(self):
        with self._lock:
            return dict(self._state)

    def meta(self):
        with self._lock:
            return dict(self._meta)

    # --- Worker ---

    def _run(self):
        try:
            drv = MujocoDriver()
            self._camera = drv.cameras()[0]
            with self._lock:
                self._meta = {
                    "actuators": drv.actuators(),
                    "cameras": drv.cameras(),
                    "mount": drv.mount,
                    "supports_mount_tuning": drv.supports_mount_tuning,
                }
        except Exception as exc:                       # Aufbau fehlgeschlagen
            self._error = f"{type(exc).__name__}: {exc}"
            self._ready.set()
            return

        self._ready.set()

        substeps = max(1, int(round((1.0 / LOOP_HZ) / drv.model.opt.timestep)))
        period = 1.0 / LOOP_HZ
        tick = 0
        next_t = time.perf_counter()

        while self._running.is_set():
            try:
                self._drain(drv)

                if not self._paused:
                    drv.step(substeps)

                if tick % RENDER_EVERY == 0:
                    rgb = drv.render(self._camera, self.width, self.height,
                                     self._cam_params)
                    buf = io.BytesIO()
                    Image.fromarray(rgb).save(buf, "JPEG", quality=JPEG_QUALITY)
                    with self._lock:
                        self._frame = buf.getvalue()
                        self._state = drv.state()
                        self._state["paused"] = self._paused
                        self._state["camera"] = self._camera
            except Exception as exc:
                with self._lock:
                    self._state = dict(self._state)
                    self._state["error"] = f"{type(exc).__name__}: {exc}"

            tick += 1
            next_t += period
            slack = next_t - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            else:
                next_t = time.perf_counter()      # nicht aufholen, sonst Spirale

        drv.close()

    def _drain(self, drv):
        while True:
            try:
                cmd = self._cmds.get_nowait()
            except queue.Empty:
                return
            self._apply(drv, cmd)

    def _apply(self, drv, cmd):
        kind = cmd.get("type")
        if kind == "ctrl":
            drv.set_ctrl(int(cmd["index"]), float(cmd["value"]))
        elif kind == "hands":
            drv.set_hands(float(cmd["value"]))
        elif kind == "zero":
            drv.zero()
        elif kind == "reset":
            drv.reset()
        elif kind == "pause":
            self._paused = bool(cmd.get("value", True))
        elif kind == "camera":
            self._camera = cmd["value"]
        elif kind == "cam_params":
            self._cam_params.update(cmd["value"])
        elif kind == "resolution":
            self.width = int(cmd["width"])
            self.height = int(cmd["height"])
        elif kind == "mount":
            drv.rebuild(cmd["value"])
            with self._lock:
                self._meta = dict(self._meta)
                self._meta["mount"] = drv.mount
