"""Teleoperation am Tisch (Stufe 2.1): ein Mensch fuehrt die rechte Hand, jede Aufnahme wird eine Demo fuer den Datensatz.

Kette je Regeltakt (30 Hz Sim-Zeit):
  Eingabe -> Ziel-Pose der Handbasis + Greifwert 0..1      KeyPilot (Tastatur, Gamepad) oder PhonePilot (iPhone)
  -> Grenzen: Tempo, Arbeitsraum, Vorlauf vor der Hand      TeleopArm
  -> IK (g1sim/grasp_ik.py): 7 Armwinkel; Greifwert -> 6 Handwerte (erst Daumen drehen, dann schliessen)
  -> 26-D-Aktion, absolute Sollwerte -> TableEnv.apply
TeleopSession fuehrt die Episoden: Die Aufnahme beginnt mit der ersten Bewegung, Speichern oder Verwerfen startet eine
neue Szene. Gespeichert wird das Format der Demo-Fabrik (g1sim/episode.py: Beobachtung vor der Aktion + Aktion), daneben
eine JSON-Datei mit Variation und Messwerten. Geregelt wird nur der rechte Arm, linker Arm und linke Hand halten READY_POSE.
Richtungen gelten im Koerpersystem des Roboters (Becken): x vor, y links, z oben.
"""
import dataclasses
import json
import pathlib

import mink
import numpy as np

from g1sim import action_space, episode, table_env
from g1sim.grasp import CLOSED, FORBIDDEN, GRASP_ROTATION, THUMB_ROTATED, reach_error
from g1sim.grasp_ik import ArmIK
from g1sim.openpi_env import DEFAULT_TASK

SIDE = "right"
ARM = action_space.SLICES[f"{SIDE}_arm"]
HAND = action_space.SLICES[f"{SIDE}_hand"]
DT = 1.0 / table_env.CONTROL_HZ

MOVE_SPEED = 0.12      # m/s bei gehaltener Taste bzw. vollem Stick (Sim-Zeit)
TURN_SPEED = 0.6       # rad/s
FINE = 0.25            # mit gehaltener Umschalttaste: Feinsteuerung. Der Arm folgt traege (bis 4 cm hinter dem Ziel),
                       # mit vollem Tempo schiesst man kurz vor dem Ziel leicht darueber hinaus
MAX_SPEED = 0.25       # m/s - schneller folgt das Ziel keiner Eingabe, auch keinem ruckartig bewegten Handy
MAX_TURN = 1.2         # rad/s
MAX_LEAD = 0.05        # m, so weit darf das Ziel der Hand vorauslaufen - sonst entfernt sich ein unerreichbares Ziel
MAX_LEAD_TURN = 0.35   # rad, dasselbe fuer die Drehung
GRIP_SPEED = 1.0       # 1/s: Greifwert 0 -> 1 in 1 s
THUMB_SHARE = 0.3      # der erste Teil des Greifwerts dreht nur den Daumen vor die Handflaeche (Befund Stufe 0.3)
THUMB_READY = 0.9      # gemessene Daumendrehung, ab der die Finger schliessen duerfen. Der Daumen braucht dafuer ~1,8 s;
                       # schliessen die Finger frueher, blockieren sie ihn am Zeigefinger und die Flasche rutscht durch
WORKSPACE = np.array([[0.10, 0.60], [-0.50, 0.15], [-0.10, 0.40]])  # Handbasis im Beckensystem (m), [min, max] je Achse
DEADZONE = 0.15        # Gamepad-Sticks
PRESET_DONE = (0.002, 0.02)  # m, rad: Voreinstellung (Greifhaltung, Startpose) gilt als erreicht

MIN_LIFT = 0.05        # m, Erfolg wie in scripts/54_grasp_scripted.py: angehoben und keine verbotenen Kontakte
MAX_EPISODE_S = 60.0
MAX_REACH_ERROR = 0.01  # m, nur Szenen mit erreichbarer Flasche (Befund 1.5b)
MAX_DRAWS = 50

# Tastatur (KeyboardEvent.code, unabhaengig von QWERTZ/QWERTY): Richtung im Koerpersystem
MOVE_KEYS = {"KeyW": (1, 0, 0), "KeyS": (-1, 0, 0), "KeyA": (0, 1, 0), "KeyD": (0, -1, 0),
             "KeyE": (0, 0, 1), "KeyQ": (0, 0, -1)}
TURN_KEYS = {"KeyJ": (0, 0, 1), "KeyL": (0, 0, -1), "KeyI": (0, 1, 0), "KeyK": (0, -1, 0),  # gieren, nicken
             "KeyU": (1, 0, 0), "KeyO": (-1, 0, 0)}  # rollen
EVENTS = ("grip", "grasp_pose", "home", "save", "discard")  # einmalige Befehle (Taste gedrueckt, nicht gehalten)

# ARKit (x rechts, y oben, z zum Nutzer; beim Start der App festgelegt) -> Koerpersystem. Gilt, wenn das Handy beim Start
# der App nach vorn zeigt, also in Blickrichtung des Roboters.
ARKIT_TO_BODY = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


@dataclasses.dataclass(frozen=True)
class Inputs:
    """Eingaben eines Regeltakts. Leer = nichts gedrueckt, die Hand bleibt stehen."""
    held: frozenset = frozenset()  # KeyboardEvent.code der gehaltenen Tasten
    events: tuple = ()             # Befehle aus EVENTS
    axes: tuple = ()               # Gamepad LX, LY, RX, RY (-1..1)
    trigger: float = 0.0           # Gamepad rechter Trigger (0..1), analoger Greifwert
    phone: dict | None = None      # juengste iPhone-Pose (g1sim/phone.py) oder None


def grip_to_hand(grip):
    """Greifwert 0..1 -> 6 Revo2-Werte: bis THUMB_SHARE nur den Daumen drehen, danach alle Finger schliessen."""
    g = float(np.clip(grip, 0.0, 1.0))
    thumb = min(g / THUMB_SHARE, 1.0)
    close = max((g - THUMB_SHARE) / (1.0 - THUMB_SHARE), 0.0)
    return THUMB_ROTATED * thumb + (CLOSED - THUMB_ROTATED) * close


def _toward(a, b, max_step):
    """Punkt von a Richtung b, hoechstens max_step weit."""
    d = np.asarray(b, float) - a
    n = float(np.linalg.norm(d))
    return np.asarray(b, float) if n <= max_step else a + d * (max_step / n)


def _turn_toward(a, b, max_angle):
    """Drehung von a Richtung b (mink.SO3), hoechstens max_angle."""
    delta = (a.inverse() @ b).log()
    n = float(np.linalg.norm(delta))
    return b if n <= max_angle else a @ mink.SO3.exp(delta * (max_angle / n))


def _pose(rotation, translation):
    return mink.SE3.from_rotation_and_translation(rotation, np.asarray(translation, float))


class TeleopArm:
    """Rechter Arm + Hand folgen einer Ziel-Pose mit Grenzen. step() liefert je Takt die 26-D-Aktion."""

    def __init__(self, env):
        self.env = env
        self.ik = ArmIK(env.model, SIDE, dt=DT)
        self.pelvis = env.model.body("pelvis").id

    def reset(self):
        """Nach env.reset(): Arme in READY_POSE, Hand offen, Ziel = Handpose der Startstellung."""
        d = self.env.data
        self.body_rot, self.body_pos = d.xmat[self.pelvis].reshape(3, 3).copy(), d.xpos[self.pelvis].copy()
        self.action = action_space.READY_POSE.copy()
        qpos = d.qpos.copy()
        qpos[self.ik.arm_qadr] = self.action[ARM]
        self.ik.reset(qpos)
        self.home = self.target = self.ik.hand_pose()
        self.grip = 0.0

    def to_world(self, direction):
        """Richtung im Koerpersystem -> Welt."""
        return self.body_rot @ np.asarray(direction, float)

    def in_body(self, point):
        """Punkt in Welt -> Koerpersystem (Becken)."""
        return self.body_rot.T @ (np.asarray(point, float) - self.body_pos)

    def step(self, goal, grip):
        """Ein Regeltakt: Ziel Richtung goal (mink.SE3) mit Grenzen, IK, Hand. Rueckgabe: 26-D-Aktion."""
        self.target = self.limit(goal)
        self.action[ARM] = self.ik.step(self.target)
        grip = float(np.clip(grip, 0.0, 1.0))
        thumb = self.env.state()[HAND][0]  # gemessen; am Roboter aus den Gelenkgebern der Hand
        if grip > self.grip and thumb < THUMB_READY:
            # Finger erst schliessen, wenn der Daumen gedreht ist. Nur weiteres Schliessen anhalten, nie oeffnen:
            # Beim Tragen drueckt die Flasche den Daumen etwas zurueck (gemessen 0,87) - sonst fiele sie heraus
            grip = max(min(grip, THUMB_SHARE), self.grip)
        self.grip += float(np.clip(grip - self.grip, -GRIP_SPEED * DT, GRIP_SPEED * DT))
        self.action[HAND] = grip_to_hand(self.grip)
        return self.action.copy()

    def limit(self, goal):
        """Tempo (vom letzten Ziel aus), Arbeitsraum, Vorlauf (vor der IK-Hand)."""
        pos = _toward(self.target.translation(), goal.translation(), MAX_SPEED * DT)
        pos = self.body_pos + self.body_rot @ np.clip(self.in_body(pos), WORKSPACE[:, 0], WORKSPACE[:, 1])
        hand = self.ik.hand_pose()
        pos = _toward(hand.translation(), pos, MAX_LEAD)
        rot = _turn_toward(self.target.rotation(), goal.rotation(), MAX_TURN * DT)
        rot = _turn_toward(hand.rotation(), rot, MAX_LEAD_TURN)
        return _pose(rot, pos)


class KeyPilot:
    """Tastatur und Gamepad -> Ziel-Pose. Gehaltene Tasten bewegen das Ziel mit MOVE_SPEED bzw. TURN_SPEED.

    Befehle: grip schaltet Greifen an/aus, grasp_pose dreht die Hand in die Greifhaltung der Demo-Fabrik (Handflaeche zur
    Flasche, Finger nach vorn), home faehrt zur Startpose. Der Gamepad-Trigger greift analog.
    """

    def reset(self):
        self.preset, self.grip = None, 0.0

    def update(self, arm, inputs):
        """Rueckgabe: (Ziel-Pose, Greifwert, hat der Bediener etwas getan?)."""
        if "grip" in inputs.events:
            self.grip = 0.0 if self.grip > 0.5 else 1.0
        if "grasp_pose" in inputs.events:
            self.preset = _pose(mink.SO3.from_matrix(arm.body_rot @ GRASP_ROTATION), arm.target.translation())
        if "home" in inputs.events:
            self.preset = arm.home
        move = sum((np.array(MOVE_KEYS[k], float) for k in inputs.held if k in MOVE_KEYS), np.zeros(3))
        turn = sum((np.array(TURN_KEYS[k], float) for k in inputs.held if k in TURN_KEYS), np.zeros(3))
        if len(inputs.axes) >= 4:
            lx, ly, rx, ry = (a if abs(a) > DEADZONE else 0.0 for a in inputs.axes[:4])
            move += [-ly, -lx, -ry]
            turn += [0.0, 0.0, -rx]
        move, turn = np.clip(move, -1.0, 1.0), np.clip(turn, -1.0, 1.0)
        if {"ShiftLeft", "ShiftRight"} & inputs.held:
            move, turn = move * FINE, turn * FINE
        if move.any() or turn.any():
            self.preset = None  # wer selbst steuert, bricht die Voreinstellung ab
        if self.preset is not None and _reached(arm.target, self.preset):
            self.preset = None
        base = arm.target if self.preset is None else self.preset
        goal = _pose(mink.SO3.exp(arm.to_world(turn) * TURN_SPEED * DT) @ base.rotation(),
                     base.translation() + arm.to_world(move) * MOVE_SPEED * DT)
        acted = bool(move.any() or turn.any() or inputs.trigger > 0.05
                     or {"grip", "grasp_pose", "home"} & set(inputs.events))
        return goal, max(self.grip, float(inputs.trigger)), acted


class PhonePilot:
    """iPhone (App "MuJoCo AR") -> Ziel-Pose, mit Kupplung: Nur solange der Knopf in der App gehalten wird, folgt die Hand
    dem Handy - loslassen, Handy neu ansetzen, wieder druecken, wie eine Computermaus anheben. Der Schalter greift.
    """

    def __init__(self, scale=1.0):
        self.scale = scale
        self.reset()

    def reset(self):
        self.clutch = None

    def update(self, arm, phone):
        """Rueckgabe: (Ziel-Pose, Greifwert, bewegt der Bediener gerade?)."""
        grip = 1.0 if phone["toggle"] else 0.0
        if not phone["button"]:
            self.clutch = None
            return arm.target, grip, False
        if self.clutch is None:
            self.clutch = (arm.target, phone["position"].copy(), phone["rotation"].copy())
        hand0, p0, r0 = self.clutch
        to_world = arm.body_rot @ ARKIT_TO_BODY
        pos = hand0.translation() + self.scale * to_world @ (phone["position"] - p0)
        turned = to_world @ (phone["rotation"] @ r0.T) @ to_world.T  # Drehung des Handys seit dem Druecken, in Welt
        return _pose(mink.SO3.from_matrix(turned) @ hand0.rotation(), pos), grip, True


def _reached(a, b):
    pos = float(np.linalg.norm(a.translation() - b.translation()))
    rot = float(np.linalg.norm((a.rotation().inverse() @ b.rotation()).log()))
    return pos < PRESET_DONE[0] and rot < PRESET_DONE[1]


class TeleopSession:
    """Episoden eines Bedieners: neue Szene, Aufnahme ab der ersten Bewegung, speichern oder verwerfen.

    tick() rechnet einen Regeltakt. Zustaende: "bereit" (Szene steht, noch nichts getan), "aufnahme", "voll"
    (MAX_EPISODE_S erreicht: der Roboter folgt weiter, aufgenommen wird nicht mehr).
    """

    def __init__(self, env, out_dir, randomize=False, task=DEFAULT_TASK, phone_scale=1.0):
        self.env, self.out_dir, self.randomize, self.task = env, pathlib.Path(out_dir), randomize, task
        self.arm, self.keys, self.phone = TeleopArm(env), KeyPilot(), PhonePilot(phone_scale)
        self.results = []  # (Pfad, erfolgreich) der in dieser Sitzung gespeicherten Episoden
        self.message = ""
        self.new_episode()

    def new_episode(self):
        """Neue Szene (mit Zufallsvariation nur Starts mit erreichbarer Flasche), Arm in Startpose, noch keine Aufnahme."""
        for _ in range(MAX_DRAWS):
            self.env.reset(randomize=self.randomize)
            self.reach = reach_error(self.env, self.arm.ik)
            if self.reach <= MAX_REACH_ERROR or not self.randomize:
                break
        else:
            raise RuntimeError(f"{MAX_DRAWS} Szenen gezogen, in keiner ist die Flasche erreichbar - Bereiche pruefen")
        self.arm.reset()
        self.keys.reset()
        self.phone.reset()
        self.recorder, self.state, self.source = None, "bereit", None
        self.contacts, self._last_contacts, self.touched = set(), set(), False

    def tick(self, inputs):
        """Ein Regeltakt. Rueckgabe: die aufgenommene Beobachtung (vor der Aktion) oder None."""
        if "save" in inputs.events and self.recorder is not None:
            self.save()
            return None
        if "discard" in inputs.events:
            self.message = "Verworfen, neue Szene" if self.recorder is not None else "Neue Szene"
            self.new_episode()
            return None
        # Mit verbundenem iPhone greift der Schalter der App; die Hand folgt dem Handy, solange der Knopf gehalten wird,
        # sonst der Tastatur
        goal, grip, acted = self.keys.update(self.arm, inputs)
        source = "Tastatur"
        if inputs.phone is not None:
            phone_goal, grip, phone_acted = self.phone.update(self.arm, inputs.phone)
            if phone_acted:
                goal, acted, source = phone_goal, True, "iPhone"
        if self.state == "bereit" and acted:
            self.recorder = episode.EpisodeRecorder(self.task, table_env.CONTROL_HZ)
            self.state, self.source = "aufnahme", source
            self.message = ""
        obs = self.env.observation() if self.state == "aufnahme" else None
        action = self.arm.step(goal, grip)
        if obs is not None:
            self.recorder.add(obs, action)
            if len(self.recorder) >= MAX_EPISODE_S * table_env.CONTROL_HZ:
                self.state, self.message = "voll", f"{MAX_EPISODE_S:.0f} s erreicht - Speichern oder Verwerfen"
        self.env.apply(action)
        now = set(self.env.robot_contacts())
        self.touched = bool({c for c in now if c[1] == "red_bottle"} - self._last_contacts)  # neu beruehrt: vibrieren
        self.contacts |= now
        self._last_contacts = now
        return obs

    def forbidden_contacts(self):
        return sorted({other for _, other in self.contacts if other in FORBIDDEN})

    def save(self):
        """Episode speichern (auch nicht erfolgreiche, der Konverter laesst sie weg), dann neue Szene.

        Rueckgabe: (Pfad, Erfolg).
        """
        lift, forbidden = self.env.lift_height(), self.forbidden_contacts()
        success = lift >= MIN_LIFT and not forbidden
        taken = [int(p.stem.split("_")[1]) for p in self.out_dir.glob("episode_*.npz")]
        path = self.out_dir / f"episode_{max(taken, default=-1) + 1:03d}.npz"
        self.recorder.save(path, success)
        info = {"task": self.task, "success": success, "lift_m": round(lift, 4), "forbidden_contacts": forbidden,
                "bottle_tilt_deg": round(self.env.object_tilt(), 1), "frames": len(self.recorder),
                "sim_s": len(self.recorder) / table_env.CONTROL_HZ, "input": self.source, "reach_error_m": self.reach,
                "randomize": self.randomize, "variation": self.env.variation}
        path.with_suffix(".json").write_text(json.dumps(info, indent=2, ensure_ascii=False))
        self.results.append((path, success))
        problem = f"Kontakt {', '.join(forbidden)}" if forbidden else f"nur {lift * 100:.1f} cm angehoben"
        why = "" if success else f" - NICHT erfolgreich: {problem}"
        self.message = f"{path.name} gespeichert{why}"
        self.new_episode()
        return path, success

    def status(self):
        frames = len(self.recorder) if self.recorder is not None else 0
        return {
            "state": self.state, "message": self.message, "source": self.source,
            "seconds": round(frames / table_env.CONTROL_HZ, 1),
            "saved": len(self.results), "successful": sum(ok for _, ok in self.results),
            "lift_cm": round(self.env.lift_height() * 100, 1), "grip": round(self.arm.grip, 2),
            "forbidden_contacts": self.forbidden_contacts(),
            "hand_body_m": self.arm.in_body(self.arm.target.translation()).round(3).tolist(),
            "out_dir": str(self.out_dir),
        }
