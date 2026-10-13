"""Laufwelt fuer Abnahme-Skripte ab Lernpfad L2: LeRobot-G1-Sim mit BrainCo-Haenden und GR00T-Lauf-Controller.

Buendelt, was jedes Lauf-Skript braucht: verbinden (optional mit anderer Szene), Gummiband loesen, Becken-Pose lesen,
(vx, vy, omega) senden, auf Sturz und stehende Sim pruefen, Sim-Zeit abwarten, Live-Ansicht und Video nachfuehren.
Laeuft nur in ~/dev/g1-ext/.venv. Der Import initialisiert DDS auf der macOS-Loopback lo0 - deshalb dieses Modul vor
jedem LeRobot-Import laden (Begruendung in scripts/50_spike_lerobot_walk.py).
"""
import time

import mujoco
import mujoco.viewer
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

ChannelFactoryInitialize(0, "lo0")

from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config  # noqa: E402

from g1sim.action_space import lerobot_arm_action  # noqa: E402
from g1sim.brainco_env import use_brainco_sim  # noqa: E402
from g1sim.liveview import LiveView  # noqa: E402
from g1sim.navigation import pose_from_qpos, to_remote  # noqa: E402
from g1sim.runtime import StallWatch, free_camera_port  # noqa: E402

FALL_HEIGHT = 0.5   # Becken darunter = gestuerzt
STAND_S = 3.0       # Sim-Sekunden am Gummiband
BALANCE_S = 3.0     # frei stehen nach dem Loesen


class WalkSim:
    def __init__(self, scene=None, live=True, video=None, title="G1 live", cameras=False):
        """cameras=False: die Sim rendert keine Kamerabilder (Lauf-Skripte brauchen sie nicht) - so rendert nur die
        Live-Ansicht, und zwei OpenGL-Threads koennen den Prozess nicht im Grafiktreiber haengen lassen (2026-10-02)."""
        # Kein MuJoCo-Fenster (friert auf diesem Mac ein); mj_forward, damit die Koerperlagen vor dem ersten Schritt stimmen
        mujoco.viewer.launch_passive = lambda model, data, *a, **k: mujoco.mj_forward(model, data)
        free_camera_port()
        use_brainco_sim(scene=scene, cameras=cameras)
        self.robot = UnitreeG1(UnitreeG1Config(is_simulation=True, controller="GrootLocomotionController"))
        self.robot.connect()
        self.sim = self.robot.sim_env.sim_env
        self.model, self.data = self.sim.mj_model, self.sim.mj_data
        self.title = title
        # Ein Renderer fuer Browser und Video
        self.live = LiveView(self.model, self.data, video=video, serve=live) if (live or video) else None
        self.watch = StallWatch(self.data)
        self.fault = None  # z. B. "sturz waehrend ziel 3" oder "sim steht waehrend ..."

    @property
    def time(self):
        return float(self.data.time)

    def pose(self):
        return pose_from_qpos(self.data.qpos)

    def foot_gap(self):
        """Right foot minus left foot, forward distance in the pelvis frame (m): > 0 = right foot in front. Input of the arm
        swing (g1sim/arm_swing.py); on the real robot the same number comes from the leg joint angles."""
        m, d = self.model, self.data
        if not hasattr(self, "_feet"):
            self._feet = (m.body("pelvis").id, m.body("left_ankle_roll_link").id, m.body("right_ankle_roll_link").id)
        pelvis, left, right = self._feet
        forward = d.xmat[pelvis].reshape(3, 3)[:, 0]
        return float(forward @ (d.xpos[right] - d.xpos[left]))

    def arm_leg_contacts(self):
        """Number of contacts between an arm/hand and a leg right now (an arm swinging into the thigh)."""
        m = self.model
        if not hasattr(self, "_limbs"):
            names = [m.body(b).name for b in range(m.nbody)]
            arm = {b for b, n in enumerate(names) if any(k in n for k in ("elbow", "wrist", "lh_", "rh_"))}
            leg = {b for b, n in enumerate(names) if any(k in n for k in ("hip", "knee", "ankle"))}
            self._limbs = (arm, leg)
        arm, leg = self._limbs
        try:
            pairs = np.array(self.data.contact.geom).reshape(-1, 2)
        except (IndexError, ValueError):
            return 0
        bodies = m.geom_bodyid[pairs] if len(pairs) else pairs
        return int(sum((a in arm and b in leg) or (a in leg and b in arm) for a, b in bodies))

    def send(self, vx, vy, omega, arms=None):
        """Walking command; arms: optional 27-D action vector whose arm part (14 angles, rad) the arms then hold
        (action_space.lerobot_arm_action). GR00T keeps the legs and the waist either way."""
        action = to_remote(vx, vy, omega)
        if arms is not None:
            action.update(lerobot_arm_action(arms))
        self.robot.send_action(action)

    def show(self, markers=None, path=None, top_view=None, **status):
        """Live-Ansicht und Video nachfuehren - einmal je Schleifendurchlauf aufrufen. top_view: map for the browser's
        top view (g1sim/liveview.py, key "_map")."""
        if self.live:
            extra = {"_map": top_view} if top_view is not None else {}
            self.live.update({"titel": self.title, "Sim-Zeit": f"{self.time:.1f} s", **status, **extra}, markers, path)

    def ok(self, label):
        """False bei Sturz oder stehender Sim; der Grund steht danach in self.fault."""
        if self.fault is None and self.data.qpos[2] < FALL_HEIGHT:
            self.fault = f"sturz waehrend {label} (Becken {self.data.qpos[2]:.2f} m)"
        elif self.fault is None and self.watch.stalled():
            self.fault = f"sim steht waehrend {label}"
        return self.fault is None

    def wait(self, seconds, label):
        """Sim-Zeit abwarten (die Sim laeuft auf dem Mac langsamer als die Wanduhr)."""
        end = self.time + seconds
        while self.time < end and self.ok(label):
            self.show(Phase=label)
            time.sleep(0.005)
        return self.fault is None

    def release_band(self):
        """Am Gummiband stehen, loesen, frei stehen. False, wenn der Roboter dabei faellt."""
        ok = self.wait(STAND_S, "am Gummiband")
        self.sim.elastic_band.enable = False
        return ok and self.wait(BALANCE_S, "frei stehen")

    def close(self):
        self.send(0.0, 0.0, 0.0)
        if self.live:
            self.live.close()
        self.robot.disconnect()
