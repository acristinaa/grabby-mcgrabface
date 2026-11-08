"""Treiber-Naht: dieselbe Schnittstelle fuer Simulation und (spaeter) echten Roboter.

Das ist die Stelle, an der sich das Dashboard mit Johannes' Arbeit trifft. Sein
Steuerstand soll "agnostic to sim or real" sein - das geht nur, wenn beide Seiten
dieselbe Schnittstelle meinen.

Heute: MujocoDriver.
Spaeter: DdsDriver gegen `unitree_hg` (NICHT `unitree_go`, das ist der Go2-Hund -
haeufigste Fehlerquelle beim Uebernehmen von Beispielcode). Dafuer wird
unitree_mujoco gebraucht, das Linux-only ist.
"""
import abc
import warnings

import mujoco
import numpy as np

from . import scene

# Beim Anbauen der Haende meldet MuJoCo Options-Konflikte (timestep, integrator).
# Die Elternwerte gewinnen, das ist gewollt. Meldung waere im Betrieb nur Laerm.
warnings.filterwarnings("ignore", message=".*Attach conflict.*")

FREE_CAMERA = "__frei__"

# Zuordnung Aktuator -> Bedienguppe. Reihenfolge bestimmt die Anzeige.
GROUPS = [
    ("Linker Arm", lambda n: n.startswith("left_") and any(
        k in n for k in ("shoulder", "elbow", "wrist"))),
    ("Rechter Arm", lambda n: n.startswith("right_") and any(
        k in n for k in ("shoulder", "elbow", "wrist"))),
    ("Linke Hand", lambda n: n.startswith("lh_")),
    ("Rechte Hand", lambda n: n.startswith("rh_")),
    ("Taille", lambda n: n.startswith("waist")),
    ("Beine", lambda n: any(k in n for k in ("hip", "knee", "ankle"))),
]


def _group_of(name):
    for label, test in GROUPS:
        if test(name):
            return label
    return "Sonstige"


class RobotDriver(abc.ABC):
    """Minimale gemeinsame Schnittstelle. Bewusst klein gehalten."""

    supports_mount_tuning = False

    @abc.abstractmethod
    def actuators(self):
        """[{index, name, group, low, high}]"""

    @abc.abstractmethod
    def cameras(self):
        """Verfuegbare Kameranamen."""

    @abc.abstractmethod
    def set_ctrl(self, index, value):
        """Einen Stellwert setzen."""

    @abc.abstractmethod
    def state(self):
        """Zustandsabbild fuer die Anzeige."""

    @abc.abstractmethod
    def reset(self):
        """Ausgangszustand."""

    @abc.abstractmethod
    def step(self, n=1):
        """n Simulationsschritte (real: no-op)."""

    @abc.abstractmethod
    def render(self, camera, width, height, cam_params=None):
        """RGB-Bild als uint8-Array."""


class MujocoDriver(RobotDriver):
    """Fahrt die MuJoCo-Szene.

    Renderer wird faul angelegt, damit der GL-Kontext in dem Thread entsteht,
    der auch rendert. Auf macOS fuehrt alles andere zu schwer greifbaren Fehlern.
    """

    supports_mount_tuning = True

    def __init__(self, mount=None):
        self.mount = mount or scene.default_mount()
        self._renderer = None
        self._render_size = None
        self._build()

    # --- Aufbau und Neuaufbau ---

    def _build(self, keep_ctrl=None):
        spec = scene.build_with_hands(self.mount)
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self._drop_renderer()

        self._act = []
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or f"act{i}"
            lo, hi = self.model.actuator_ctrlrange[i]
            self._act.append(
                {"index": i, "name": name, "group": _group_of(name),
                 "low": float(lo), "high": float(hi)}
            )
        self._hand_ids = scene.hand_actuator_ids(self.model)
        self._hand_geom = {
            side: (scene.hand_base_id(self.model, side),
                   scene.distal_body_ids(self.model, side))
            for side in ("left", "right")
        }

        if keep_ctrl is not None and len(keep_ctrl) == self.model.nu:
            self.data.ctrl[:] = keep_ctrl
        mujoco.mj_forward(self.model, self.data)

    def rebuild(self, mount):
        """Montage aendern und Modell neu kompilieren (~180 ms).

        Aktuatoranzahl bleibt gleich, deshalb koennen die Stellwerte uebernommen
        werden - der Roboter springt beim Justieren nicht in die Nullstellung.
        """
        self.mount = mount
        self._build(keep_ctrl=self.data.ctrl.copy())

    # --- Schnittstelle ---

    def actuators(self):
        return self._act

    def cameras(self):
        names = [FREE_CAMERA]
        for i in range(self.model.ncam):
            names.append(mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_CAMERA, i))
        return names

    def set_ctrl(self, index, value):
        if 0 <= index < self.model.nu:
            lo, hi = self.model.actuator_ctrlrange[index]
            self.data.ctrl[index] = float(np.clip(value, lo, hi))

    def set_hands(self, fraction):
        """Beide Haende gemeinsam: 0.0 = offen, 1.0 = zu."""
        for i in self._hand_ids:
            lo, hi = self.model.actuator_ctrlrange[i]
            self.data.ctrl[i] = lo + (hi - lo) * float(np.clip(fraction, 0.0, 1.0))

    def zero(self):
        self.data.ctrl[:] = 0.0

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def step(self, n=1):
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)

    def spread(self, side):
        """Mittlerer Abstand Handbasis -> Endglieder, in cm.

        Offene Hand = grosser Wert, geschlossene = kleiner. Damit ist im
        Dashboard live sichtbar, ob die Haende wirklich greifen - dieselbe
        Groesse, die scripts/16_hand_check.py als Abnahme misst.
        """
        base, distal = self._hand_geom[side]
        if base < 0 or not distal:
            return None
        d = np.linalg.norm(self.data.xpos[distal] - self.data.xpos[base], axis=1)
        return float(d.mean() * 100.0)

    def state(self):
        return {
            "time": float(self.data.time),
            "ctrl": [float(v) for v in self.data.ctrl],
            "spread": {s: self.spread(s) for s in ("left", "right")},
            "nq": int(self.model.nq),
            "nu": int(self.model.nu),
            "mount": self.mount,
        }

    # --- Rendern ---

    def _drop_renderer(self):
        if self._renderer is not None:
            self._renderer.close()
        self._renderer = None
        self._render_size = None

    def _ensure_renderer(self, width, height):
        if self._renderer is None or self._render_size != (width, height):
            if self._renderer is not None:
                self._renderer.close()
            self._renderer = mujoco.Renderer(self.model, height=height, width=width)
            self._render_size = (width, height)
        return self._renderer

    def render(self, camera, width, height, cam_params=None):
        r = self._ensure_renderer(width, height)
        if camera and camera != FREE_CAMERA:
            r.update_scene(self.data, camera=camera)
        else:
            cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            p = cam_params or {}
            cam.lookat[:] = p.get("lookat", [0.35, 0.0, 0.95])
            cam.distance = p.get("distance", 1.8)
            cam.azimuth = p.get("azimuth", 135.0)
            cam.elevation = p.get("elevation", -15.0)
            r.update_scene(self.data, camera=cam)
        return r.render()

    def close(self):
        self._drop_renderer()
