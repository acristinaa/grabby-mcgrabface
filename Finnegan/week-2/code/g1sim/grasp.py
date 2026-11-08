"""Stufe 1.4 - Flasche greifen ohne KI (Track A): Kopfkamera -> IK -> Hand, als Folge fester Sollwerte auf TableEnv.

Seitlicher Griff mit der rechten Hand: Handflaeche zur Flasche, Finger waagerecht nach vorn, um 30 Grad um die Fingerachse
gerollt (Handflaeche schaut etwas nach unten). Der gedrehte Daumen haelt die Flasche von hinten (zum Roboter hin), Zeige- und
Mittelfinger vorn; Ringfinger und kleiner Finger bleiben offen.
  freimachen (Hand aus dem Kamerabild) -> hinsehen -> anfahren (ueber den Vorgriff neben der Flasche) -> absenken
  -> neu hinsehen -> heranfahren -> Daumen drehen -> Finger und Daumen schliessen -> anheben -> halten
Schraeglage der Flasche in der Hand 2-4 Grad (bis D2 am 2026-10-02: ~40 Grad, Log 2026-10-02, scripts/63_grasp_sweep.py).
Vor dem Hinsehen und vor dem Schliessen misst der Ablauf, wie weit die Hand unter der Schwerkraft von ihrer Soll-Pose
absackt, und rechnet das aus den folgenden Sollposen heraus. Am echten Roboter kaeme die gemessene Handpose aus den
Gelenkgebern, hier aus der Sim.
"""
import mink
import numpy as np

from g1sim import action_space, table_env, table_scene
from g1sim.grasp_ik import ArmIK, hand_frame, pose
from g1sim.perception import Locator

SIDE = "right"
ARM = action_space.SLICES[f"{SIDE}_arm"]
HAND = action_space.SLICES[f"{SIDE}_hand"]

# Handbasis beim Greifen, Spalten = Handachsen in Weltkoordinaten. Grundhaltung:
# x (Handflaechennormale) -> +y, y (Daumenseite) -> +z, z (Fingerrichtung) -> +x
SIDE_GRASP_ROTATION = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
# Dazu um die Fingerachse gerollt (D2): Handflaechennormale zeigt 30 Grad nach unten, die Daumenseite zur Flasche hin. Die Drehung
# geht um den Greifpunkt auf der Flaschenachse (grasp_pose). Gemessen -15 .. -35 Grad, alle mit 100 % bei erreichbarer Flasche und
# 2-4 Grad Schraeglage; -30 liegt im Plateau und hat die meiste Hubreserve. Mit geschlossenem Ringfinger hob die Hand ab -40 keine 5 cm mehr.
GRASP_ROLL_DEG = -30.0
GRASP_ROTATION = SIDE_GRASP_ROTATION @ mink.SO3.from_z_radians(np.radians(GRASP_ROLL_DEG)).as_matrix()
# Flaschenachse in Handbasis-Koordinaten beim Greifen (m): 6,6 cm vor der Handbasis, 9 cm entlang der Finger. Gewaehlt aus
# ~150 Sim-Varianten (Log 2026-09-15): haelt bei +-5 mm Versatz 9 von 9 Mal. Naeher an der Handflaeche blockiert die Flasche
# den Daumen, weiter vorn greift nur noch der Mittelfinger.
BOTTLE_IN_HAND = np.array([0.066, 0.0, 0.090])
# Greifhoehe unter dem Deckel (m). Tiefer zum Schwerpunkt hin (7,5-12 cm) kippt die Flasche mehr und weniger Starts sind
# erreichbar, 3-6 cm aendern wenig (D2)
GRASP_BELOW_TOP = 0.06
PREGRASP_OFFSET = 0.06  # Vorgriff: so weit gegen die Handflaechennormale zurueck (m)
ABOVE = 0.12            # Anfahrt ueber dem Vorgriff, damit die Finger die Flasche nicht streifen (m)
LIFT = 0.10  # m, so weit soll die Hand wirklich hoch (Teleop-Bediener in 58 fahren die gemessene Hand dorthin)
# Der Arm sackt unter der Flasche ~2,5 cm ab - die Absackung wird vor dem Schliessen gemessen, also ohne Last. Der Skript-Griff
# kommandiert deshalb LOAD_SAG mehr (D2): kleinster Hub ueber 50 Zufallsstarts 7,7 cm (Seed 0) bzw. 7,4 cm (Seed 1), Kriterium
# 5 cm; ohne Zuschlag hoben Randstarts unter 5 cm. 14 cm fuer die gemessene Hand lagen dagegen bei Randstarts ausser Reichweite.
LOAD_SAG = 0.04

# Revo2, Reihenfolge action_space.HAND_MOTORS: Thumb dreht den Daumen vor die Handflaeche, ThumbAux beugt ihn.
# Erst drehen, dann schliessen - sonst blockiert der Daumen am Zeigefinger (Befund Stufe 0.3).
THUMB_ROTATED = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# Ringfinger und kleiner Finger bleiben offen (D2): Sie drueckten mit je 2 Nm auf die untere Flaschenhaelfte und drehten den Daumen
# zurueck, dessen Drehmotor im BrainCo-Modell nur 0,5 Nm hat (actuatorfrcrange). Die Flasche kippte mit, bis ~40 Grad. Offen bleibt
# der Daumen an seinem Anschlag, Zeige- und Mittelfinger druecken dagegen.
CLOSED = np.array([1.0, 1.0, 1.0, 1.0, 0.0, 0.0])

APPROACH_S, DESCEND_S, REACH_S = 2.5, 1.5, 1.5
SETTLE_S, SETTLE_ROUNDS = 0.5, 3
THUMB_S, CLOSE_S, LIFT_S, HOLD_S = 2.0, 1.5, 2.0, 5.0

# Freimachen (Lernpfad L3a): In READY_POSE sieht die Kopfkamera ~5500 Pixel der eigenen rechten Hand, sie verdeckte das erste
# Hinsehen (4 der 5 Rest-Fehlschlaege in 1.5b). Versatz der Handbasis im Beckensystem (m): 15 cm zurueck, 10 cm nach aussen.
# Gemessen mit Segmentierungsbild (welches Pixel gehoert zu welchem Koerper) ueber 30 zufaellige Starts: 0 Hand-Pixel im Bild,
# Flasche >= 770 Pixel sichtbar, IK <= 1 mm, kein Kontakt. Kuerzester von 27 gepruefen Wegen (Log 2026-10-02).
LOOK_OFFSET = np.array([-0.15, -0.10, 0.0])
FREE_S = 1.5
FORBIDDEN = ("table", "green_cup", "blue_block")  # Kontakt mit der Flasche ist erlaubt


def ticks(seconds):
    return int(round(seconds * table_env.CONTROL_HZ))


def grasp_pose(bottle_top):
    """Soll-Pose der Handbasis (mink.SE3) fuer eine Flasche mit Deckelmitte bottle_top (Weltkoordinaten, m)."""
    axis_point = np.asarray(bottle_top, float) - [0.0, 0.0, GRASP_BELOW_TOP]
    return pose(GRASP_ROTATION, axis_point - GRASP_ROTATION @ BOTTLE_IN_HAND)


def shifted(p, offset):
    """Dieselbe Pose, um offset (Weltkoordinaten, m) verschoben."""
    return mink.SE3.from_rotation_and_translation(p.rotation(), p.translation() + offset)


def interpolate(a, b, alpha):
    """Pose zwischen a und b: Position auf gerader Linie, Drehung gleichmaessig."""
    return mink.SE3.from_rotation_and_translation(a.rotation().interpolate(b.rotation(), alpha),
                                                  (1.0 - alpha) * a.translation() + alpha * b.translation())


def bottle_top(env):
    """Wahre Deckelmitte der Flasche aus der Sim - nur zum Vergleich mit der Wahrnehmung."""
    return env.object_position("red_bottle") + [0.0, 0.0, table_scene.OBJECTS["red_bottle"][1][1]]


def reach_error(env, ik, iterations=150):
    """Wie nah kommt die IK aus der Startpose an die Greifpose der wahren Flasche (m)? Nur Rechnung, keine Physik.

    Die Armlaenge begrenzt das Greiffenster (Messung 1.5b: etwa x 0,30-0,46 m vor dem Becken); ein grosser Wert heisst
    "ausser Reichweite", nicht "Griff misslungen". Die IK steht danach irgendwo - der Aufrufer setzt sie neu auf.
    """
    qpos = env.data.qpos.copy()
    qpos[ik.arm_qadr] = action_space.READY_POSE[ARM]
    ik.reset(qpos)
    goal = grasp_pose(bottle_top(env))
    for _ in range(iterations):
        ik.step(goal)
    return float(np.linalg.norm(ik.hand_pose().translation() - goal.translation()))


class ScriptedGrasp:
    """Ein Greifversuch auf einer frisch zurueckgesetzten TableEnv.

    on_tick(phase) wird nach jedem Regeltakt gerufen, z. B. fuer Video und Kamerabilder.
    """

    def __init__(self, env, lock_wrist=False, on_tick=None):
        self.env = env
        self.ik = ArmIK(env.model, SIDE, lock_wrist, dt=1.0 / table_env.CONTROL_HZ)
        self.locator = Locator(env.model, noise=env.camera_noise)
        self.hand_body = env.model.body(hand_frame(SIDE)).id
        self.on_tick = on_tick or (lambda phase: None)

    def reach_error(self, iterations=150):
        """reach_error() fuer diesen Versuch; run() setzt die IK danach neu auf."""
        return reach_error(self.env, self.ik, iterations)

    def _reset_ik(self):
        qpos = self.env.data.qpos.copy()
        qpos[self.ik.arm_qadr] = action_space.READY_POSE[ARM]
        self.ik.reset(qpos)

    def run(self):
        """Ganzer Ablauf. Rueckgabe: Messwerte (dict); bewertet wird im Abnahme-Skript."""
        env = self.env
        self.action = action_space.READY_POSE.copy()
        self._reset_ik()
        self.target = self.ik.hand_pose()  # wo die Hand tatsaechlich stehen soll
        self.sag = mink.SE3.identity()     # gemessene Absackung der Hand (im Handkoordinatensystem), wird herausgerechnet
        self.contacts = set()
        looks = {}

        pelvis_rot = env.data.xmat[env.model.body("pelvis").id].reshape(3, 3)
        self._move("freimachen", shifted(self.target, pelvis_rot @ LOOK_OFFSET), FREE_S)
        looks["hinsehen"] = self._look()
        pregrasp = shifted(grasp_pose(looks["hinsehen"]["seen"]), -PREGRASP_OFFSET * GRASP_ROTATION[:, 0])
        self._move("anfahren", shifted(pregrasp, [0.0, 0.0, ABOVE]), APPROACH_S)
        self._move("absenken", pregrasp, DESCEND_S)
        self._settle("absenken")

        looks["neu_hinsehen"] = self._look()
        grasp = grasp_pose(looks["neu_hinsehen"]["seen"])
        self._move("heranfahren", grasp, REACH_S)
        self._settle("heranfahren")
        pos_err, rot_err = self._hand_error(grasp)

        self.action[HAND] = THUMB_ROTATED
        self._hold("daumen_drehen", THUMB_S)
        self.action[HAND] = CLOSED
        self._hold("schliessen", CLOSE_S)
        self._move("anheben", shifted(grasp, [0.0, 0.0, LIFT + LOAD_SAG]), LIFT_S)
        lift = []
        for _ in range(ticks(HOLD_S)):
            self._command(self.target)
            self._tick("halten")
            lift.append(env.lift_height())

        return {
            "looks": looks,
            # Die Greifpose kommt allein aus dem zweiten Hinsehen. Das erste bestimmt die grobe Anfahrt; seit dem Freimachen
            # (L3a) ist die Hand dabei aus dem Bild - vorher lag es durch Verdeckung bis 12 cm daneben (Befund 1.5b)
            "perception_error_m": looks["neu_hinsehen"]["error_m"],
            "first_look_error_m": looks["hinsehen"]["error_m"],
            "grasp_pos_error_m": pos_err,
            "grasp_rot_error_deg": float(np.rad2deg(rot_err)),
            "min_lift_during_hold_m": float(min(lift)),
            "bottle_tilt_deg": env.object_tilt("red_bottle"),
            "contacts": sorted(self.contacts),
            "forbidden_contacts": sorted(c for c in self.contacts if c[1] in FORBIDDEN),
            "right_hand_state": env.state()[HAND].round(3).tolist(),
            "sim_time_s": float(env.data.time),
        }

    # --- Bausteine ------------------------------------------------------------------------------------------------
    def _look(self):
        seen = self.locator.locate(self.env.data)
        if seen is None:
            raise RuntimeError("Flasche im Bild der Kopfkamera nicht gefunden")
        true = bottle_top(self.env)
        return {"seen": seen.round(4).tolist(), "true": true.round(4).tolist(),
                "error_m": float(np.linalg.norm(seen - true))}

    def _move(self, phase, goal, seconds):
        """Hand in `seconds` zur Soll-Pose goal fuehren."""
        start, n = self.target, ticks(seconds)
        for i in range(1, n + 1):
            self._command(interpolate(start, goal, i / n))
            self._tick(phase)
        self.target = goal

    def _settle(self, phase):
        """Stehen lassen, Absackung (Position und Drehung) messen und herausrechnen, SETTLE_ROUNDS Mal."""
        for _ in range(SETTLE_ROUNDS):
            self._hold(phase, SETTLE_S)
            self.sag = self.sag @ (self.target.inverse() @ self._measured_hand())

    def _hold(self, phase, seconds):
        for _ in range(ticks(seconds)):
            self._command(self.target)
            self._tick(phase)

    def _command(self, target):
        """Armwinkel fuer die Soll-Pose target in die Aktion schreiben (IK-Ziel = target ohne die gemessene Absackung)."""
        self.action[ARM] = self.ik.step(target @ self.sag.inverse())

    def _tick(self, phase):
        self.env.apply(self.action)
        self.contacts.update(self.env.robot_contacts())
        self.on_tick(phase)

    def _measured_hand(self):
        d = self.env.data
        return pose(d.xmat[self.hand_body].reshape(3, 3), d.xpos[self.hand_body])

    def _hand_error(self, goal):
        """(Positionsfehler m, Winkelfehler rad) der gemessenen Handbasis zur Soll-Pose."""
        measured = self._measured_hand()
        pos_err = float(np.linalg.norm(measured.translation() - goal.translation()))
        return pos_err, float(np.linalg.norm((measured.rotation().inverse() @ goal.rotation()).log()))

    def close(self):
        self.locator.close()
