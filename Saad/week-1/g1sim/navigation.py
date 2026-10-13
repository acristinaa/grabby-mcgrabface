"""Gehe zu Punkt (Lernpfad L1): geschlossener Regelkreis vom Becken zum Ziel, Ausgabe (vx, vy, omega) fuer den Lauf-Controller.

Kette je Regeltakt (CONTROL_HZ, Sim-Zeit):
  Pose messen (x, y, Blickrichtung) -> Fehler in der Welt -> ins Koerpersystem drehen -> P-Regler mit Grenzen -> (vx, vy, omega)
Die Pose liefert vorerst die Sim (qpos des Beckens), spaeter die Lokalisierung (Lernpfad L7). Der Lauf-Controller (GR00T)
macht aus (vx, vy, omega) Schritte und Balance; er bekommt sie als virtuelle Joystick-Achsen (to_remote).
Koerpersystem wie ueberall im Projekt: x vor, y links. omega > 0 dreht nach links (von oben gesehen gegen den Uhrzeigersinn).
Rein numerisch, ohne MuJoCo - laeuft in jeder Umgebung und ist einzeln testbar (tests/test_unit_navigation.py).
"""
import dataclasses
import math

import numpy as np

CONTROL_HZ = 20  # neuer Befehl alle 50 ms - der Lauf-Controller selbst rechnet mit 50 Hz
GROOT_DEADBAND = 0.05  # gr00t_locomotion.py: Befehlsnorm darunter -> Steh-Policy statt Lauf-Policy


@dataclasses.dataclass(frozen=True)
class Pose2D:
    """Lage in der Ebene: Position in m, Blickrichtung yaw in rad (0 = Welt-x, positiv nach links drehend)."""
    x: float
    y: float
    yaw: float


def wrap(angle):
    """Winkel auf -pi..pi: 350 Grad Fehler sind in Wahrheit -10 Grad, der Roboter soll den kurzen Weg drehen."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(quat):
    """Blickrichtung (Drehung um die Hochachse) aus einer MuJoCo-Quaternion (w, x, y, z)."""
    w, x, y, z = (float(v) for v in quat)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_from_qpos(qpos):
    """Becken-Pose aus dem Anfang von qpos: Freigelenk = Position (3) + Quaternion (4)."""
    return Pose2D(float(qpos[0]), float(qpos[1]), yaw_from_quat(qpos[3:7]))


def to_body(dx, dy, yaw):
    """Weltvektor (dx, dy) ins Koerpersystem eines Roboters, der in Richtung yaw schaut (Drehung um -yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def to_remote(vx, vy, omega):
    """(vx, vy, omega) -> virtuelle Joystick-Achsen fuer UnitreeG1.send_action. GR00T rechnet cmd = [ly, -lx, -rx]
    (gr00t_locomotion.py), deshalb hier die Vorzeichen umgekehrt."""
    return {"remote.ly": float(vx), "remote.lx": float(-vy), "remote.rx": float(-omega)}


@dataclasses.dataclass(frozen=True)
class GotoParams:
    k_lin: float = 1.0          # 1/s: 1 m Fehler -> 1 m/s Wunsch (danach begrenzt)
    k_yaw: float = 1.5          # 1/s: 1 rad Fehler -> 1,5 rad/s Wunsch
    max_vx: float = 0.5         # m/s vorwaerts (rueckwaerts die Haelfte)
    max_vy: float = 0.25        # m/s seitwaerts - Humanoide gehen seitlich schlechter als vorwaerts
    max_omega: float = 0.6      # rad/s
    max_accel: float = 0.5      # m/s^2 bzw. rad/s^2: Befehl aendert sich nur langsam, sonst kippt der Schwung
    face_dist: float = 0.4      # m: weiter weg zum Ziel hin drehen, naeher dran die End-Blickrichtung einnehmen
    pos_tol: float = 0.10       # m: angekommen (Masterplan T3)
    yaw_tol: float = math.radians(10)
    min_cmd: float = 0.08       # unterhalb der Totzone (GROOT_DEADBAND) wuerde der Controller nur stehen


class GotoController:
    """P-Regler von der aktuellen Pose zu einer Zielpose. step() einmal je Regeltakt aufrufen."""

    def __init__(self, params=GotoParams(), hz=CONTROL_HZ):
        self.p = params
        self.dt = 1.0 / hz
        self.cmd = np.zeros(3)  # zuletzt gesendet (vx, vy, omega)

    def errors(self, pose, goal):
        """Abstand (m) und Restdrehung zur End-Blickrichtung (rad)."""
        return math.hypot(goal.x - pose.x, goal.y - pose.y), wrap(goal.yaw - pose.yaw)

    def arrived(self, pose, goal):
        dist, yaw_err = self.errors(pose, goal)
        return dist < self.p.pos_tol and abs(yaw_err) < self.p.yaw_tol

    def step(self, pose, goal):
        """Ein Regeltakt: liefert (vx, vy, omega). Angekommen -> (0, 0, 0), der Controller steht dann."""
        p = self.p
        dx, dy = goal.x - pose.x, goal.y - pose.y
        dist = math.hypot(dx, dy)
        if self.arrived(pose, goal):
            want = np.zeros(3)
        else:
            # 1. Fehler aus Sicht des Roboters: "so weit vor, so weit links"
            ex, ey = to_body(dx, dy, pose.yaw)
            # 2. Wohin schauen: weit weg zum Ziel hin (vorwaerts geht am besten), nah dran die gewuenschte End-Richtung
            heading = math.atan2(dy, dx) if dist > p.face_dist else goal.yaw
            yaw_err = wrap(heading - pose.yaw)
            # 3. P-Regler mit Grenzen. Steht das Ziel seitlich oder hinten, erst drehen, dann gehen (cos-Faktor)
            forward_share = max(0.0, math.cos(yaw_err)) if dist > p.face_dist else 1.0
            vx = float(np.clip(p.k_lin * ex * forward_share, -p.max_vx / 2, p.max_vx))
            vy = float(np.clip(p.k_lin * ey * (0.0 if dist > p.face_dist else 1.0), -p.max_vy, p.max_vy))
            omega = float(np.clip(p.k_yaw * yaw_err, -p.max_omega, p.max_omega))
            want = np.array([vx, vy, omega])
            # 4. Totzone: noch nicht angekommen, aber der Befehl waere so klein, dass GR00T nur steht -> auf min_cmd anheben
            norm = float(np.linalg.norm(want))
            if 0.0 < norm < p.min_cmd:
                want *= p.min_cmd / norm
        # 5. Befehl nur langsam aendern (Rampe)
        step = p.max_accel * self.dt
        self.cmd = self.cmd + np.clip(want - self.cmd, -step, step)
        if not want.any() and np.abs(self.cmd).max() < step:
            self.cmd = np.zeros(3)
        return tuple(float(v) for v in self.cmd)

    def stop(self):
        self.cmd = np.zeros(3)


LOOKAHEAD = 0.5  # m: so weit voraus auf dem Weg peilt die Bahnverfolgung - kleiner folgt enger, schwingt aber mehr


class PathFollower:
    """Bahnverfolgung (Pure Pursuit, Lernpfad L2): peilt einen Punkt LOOKAHEAD voraus auf dem Weg an, wie beim Autofahren.

    Der Vorausschaupunkt geht als Zwischenziel an den GotoController aus L1 (weit weg: hindrehen und gehen). Auf den letzten
    LOOKAHEAD Metern wird das echte Ziel mit End-Blickrichtung angesteuert.
    path: Weltpunkte (x, y) vom Start zum Ziel, z. B. aus occupancy.astar.
    """

    WINDOW = 40  # Pfadpunkte: der naechste Punkt wird nur so weit voraus gesucht - sonst springt der Fortschritt zurueck

    def __init__(self, path, goal, controller=None, lookahead=LOOKAHEAD):
        self.path = np.asarray(path, dtype=float).reshape(-1, 2)
        self.goal = goal
        self.ctrl = controller or GotoController()
        self.lookahead = lookahead
        self.i = 0  # Fortschritt auf dem Weg (Index des naechsten Pfadpunkts), waechst nur
        self.s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(self.path, axis=0), axis=1))])  # Bogenlaenge

    def target(self, pose):
        """Zwischenziel fuer diesen Takt."""
        window = self.path[self.i:self.i + self.WINDOW]
        self.i += int(np.argmin(np.linalg.norm(window - (pose.x, pose.y), axis=1)))
        k = int(np.searchsorted(self.s, self.s[self.i] + self.lookahead))
        if k >= len(self.path):
            return self.goal
        (px, py), (x, y) = self.path[k - 1], self.path[k]
        return Pose2D(float(x), float(y), math.atan2(y - py, x - px))

    def step(self, pose):
        return self.ctrl.step(pose, self.target(pose))

    def arrived(self, pose):
        return self.ctrl.arrived(pose, self.goal)

    @property
    def cmd(self):
        """Last command (vx, vy, omega) - same name as SmoothPathFollower.cmd, so a caller can hand over to the next controller."""
        return self.ctrl.cmd


# --- Calm path following (2026-10-09) -------------------------------------------------------------------------------------
# With PathFollower on the raw grid path the robot wobbled 9-10 cm around the path (max 28 cm): the path zig-zags, the
# heading controller turns hard, then corrects, and turn-then-walk brakes at every bend. SmoothPathFollower follows a
# smoothed path (occupancy.smooth_path) with a speed profile and pure pursuit: walk forward and turn at the same time,
# omega from the arc to a point ahead. Only when the path starts behind the robot it turns on the spot first.

@dataclasses.dataclass(frozen=True)
class SmoothParams:
    v_max: float = 0.35         # m/s on straight stretches (GR00T walks ~0.045 m/s slower than commanded)
    v_min: float = 0.12         # m/s, never less while walking: below ~0.1 GR00T steps on the spot
    a_lat: float = 0.10         # m/s^2 sideways in curves: v <= sqrt(a_lat / curvature) (0.5 m radius -> 0.22 m/s)
    a_lin: float = 0.25         # m/s^2 speeding up and slowing down along the path
    v_end: float = 0.15         # m/s at the end of the path (the docking takes over from there)
    lookahead_min: float = 0.25  # m, pure pursuit point ahead: lookahead_min + lookahead_per_speed * v (shorter in curves,
    lookahead_per_speed: float = 0.7  # where the profile is slower: a long lookahead cut the corners round the table)
    lookahead_max: float = 0.65
    omega_max: float = 0.5      # rad/s
    turn_in_place: float = math.radians(60)  # path direction further off than this -> turn on the spot first ...
    walk_again: float = math.radians(25)     # ... until it is within this
    turn_omega_min: float = 0.3  # rad/s while turning on the spot (smaller turns are slow and shaky)
    turn_omega_max: float = 0.6  # rad/s on the spot: GR00T drifts while it turns (~5 cm/s), so do not turn slowly
    hold_gain: float = 1.0       # 1/s: while turning on the spot, step back towards where the turn started ...
    hold_max: float = 0.12       # m/s ... at most this fast (68, seed 3: a 180 deg turn drifted 35 cm sideways without it)
    acc_v: float = 0.3          # m/s^2 command ramp
    acc_omega: float = 0.8      # rad/s^2 command ramp
    vx_dead_band: float = 0.045  # m/s: the robot really walks about this much slower; used to compute the turn rate
    max_dt: float = 0.2


def speed_profile(points, p=SmoothParams()):
    """Target speed (m/s) at each point of a finely sampled path: v_max, slower in curves, ramps at start and end."""
    pts = np.asarray(points, float)
    n = len(pts)
    if n < 2:
        return np.full(n, p.v_end)
    ds = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    heading = np.unwrap(np.arctan2(*np.diff(pts, axis=0).T[::-1]))
    curvature = np.zeros(n)
    curvature[1:-1] = np.abs(np.diff(heading)) / np.maximum(0.5 * (ds[:-1] + ds[1:]), 1e-6)
    # average the curvature over ~0.25 m: the sampled path has small kinks that are not real curves
    k = max(1, int(round(0.25 / max(float(np.median(ds)), 1e-6))))
    curvature = np.convolve(curvature, np.ones(2 * k + 1) / (2 * k + 1), mode="same")
    v = np.minimum(p.v_max, np.sqrt(p.a_lat / np.maximum(curvature, 1e-6)))
    v[-1] = p.v_end
    for i in range(n - 2, -1, -1):  # able to slow down in time
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * p.a_lin * ds[i]))
    v[0] = min(v[0], p.v_min)
    for i in range(1, n):  # and speed up gently
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * p.a_lin * ds[i - 1]))
    return np.maximum(v, p.v_min)


class SmoothPathFollower:
    """Pure pursuit on a smoothed path with a speed profile. step(pose, t) once per control tick -> (vx, vy, omega).
    vy stays 0: humanoids walk best forwards. Commands change at most acc_v / acc_omega per second (sim time)."""

    WINDOW = 40  # path points: the nearest point is only searched this far ahead, so progress never jumps back

    def __init__(self, path, goal, params=SmoothParams()):
        self.path = np.asarray(path, dtype=float).reshape(-1, 2)
        self.goal, self.p = goal, params
        self.speeds = speed_profile(self.path, params)
        self.s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(self.path, axis=0), axis=1))])
        self.i, self.t = 0, None
        self.turning, self.anchor = False, None
        self.cmd = np.zeros(3)

    def target(self, pose):
        """(lookahead point, target speed) for this tick."""
        window = self.path[self.i:self.i + self.WINDOW]
        self.i += int(np.argmin(np.linalg.norm(window - (pose.x, pose.y), axis=1)))
        v_ref = float(self.speeds[self.i])
        ahead = float(np.clip(self.p.lookahead_min + self.p.lookahead_per_speed * v_ref, self.p.lookahead_min,
                              self.p.lookahead_max))
        k = min(int(np.searchsorted(self.s, self.s[self.i] + ahead)), len(self.path) - 1)
        return self.path[k], v_ref

    def step(self, pose, t=None):
        p = self.p
        dt = 1.0 / CONTROL_HZ if (t is None or self.t is None) else float(np.clip(t - self.t, 1e-3, p.max_dt))
        self.t = t
        (tx, ty), v_ref = self.target(pose)
        bx, by = to_body(tx - pose.x, ty - pose.y, pose.yaw)
        alpha = math.atan2(by, bx)
        distance = max(math.hypot(bx, by), 1e-3)
        turning = abs(alpha) > (p.walk_again if self.turning else p.turn_in_place)
        if turning and not self.turning:
            self.anchor = (pose.x, pose.y)  # hold this spot while turning
        self.turning = turning
        if self.turning:
            hx, hy = to_body(self.anchor[0] - pose.x, self.anchor[1] - pose.y, pose.yaw)
            hold = np.clip(p.hold_gain * np.array([hx, hy]), -p.hold_max, p.hold_max)
            omega = math.copysign(max(p.turn_omega_min, min(p.turn_omega_max, 1.2 * abs(alpha))), alpha)
            want = np.array([hold[0], hold[1], omega])
        else:
            vx = v_ref * max(0.4, math.cos(alpha))
            walked = max(vx - p.vx_dead_band, 0.05)  # what the robot really walks: turn for that speed
            omega = float(np.clip(2.0 * walked * math.sin(alpha) / distance, -p.omega_max, p.omega_max))
            want = np.array([vx, 0.0, omega])
        limit = np.array([p.acc_v, p.acc_v, p.acc_omega]) * dt
        self.cmd = self.cmd + np.clip(want - self.cmd, -limit, limit)
        return tuple(float(c) for c in self.cmd)

    def progress(self):
        """Share of the path done (0..1)."""
        return float(self.s[self.i] / self.s[-1]) if self.s[-1] > 0 else 1.0
