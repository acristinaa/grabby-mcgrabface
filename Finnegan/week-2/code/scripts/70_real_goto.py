"""Echter G1: Gehe zu Punkt (Lernpfad L1 am Roboter) - unser Regler g1sim/navigation.py steuert den Werks-Lauf-Controller.

Kette je Regeltakt (10 Hz): Pose schaetzen -> GotoController (derselbe wie in der Sim, 61_goto_check.py) -> (vx, vy, omega)
-> LocoClient.SetVelocity mit 0,5 s Gueltigkeit. Kommt kein neuer Befehl (Programm haengt oder ist beendet), bleibt der Roboter
nach 0,5 s von selbst stehen. Ziele sind relativ zur Startlage: x vor, y links (m), optional End-Blickrichtung in Grad.
Woher die Pose kommt (der Reihe nach, was beim Start Daten liefert):
  odom  - rt/odommodestate (SportModeState_: Position + Blickrichtung aus der Odometrie des Roboters)
  imu   - Blickrichtung aus rt/lowstate (Lagesensor), Position aus den gesendeten Geschwindigkeiten aufaddiert
  dead  - alles aus den gesendeten Geschwindigkeiten aufaddiert (Koppelnavigation, wird mit jedem Meter ungenauer)
Sicherheit: Ziel hoechstens 3 m entfernt, langsam (0,3 m/s, 0,4 rad/s), Zeitlimit je Ziel, Ctrl+C = sofort anhalten. Nie Damp oder
ZeroTorque - der Roboter wuerde schlaff umfallen. Vorher: Roboter gesichert, steht per Fernbedienung, zweite Person an der
Fernbedienung, 3 m frei. Ohne --dry-run fragt das Skript vor jeder Fahrt nach.
Umgebung ~/dev/g1-ext (Mac per Kabel, feste IP 192.168.123.222/24):
    ~/dev/g1-ext/.venv/bin/python scripts/70_real_goto.py --dry-run                 # ohne Roboter: nur rechnen und anzeigen
    DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib \\
        ~/dev/g1-ext/.venv/bin/python scripts/70_real_goto.py --iface en5          # interaktiv: Ziele eintippen
    ... --iface en5 --goal 1.5 0                                                   # ein Ziel, dann Ende
    ... --iface en5 --hin-zurueck 1 0                                              # 1 m vor und von selbst zurueck
"""
import argparse
import dataclasses
import math
import pathlib
import sys
import threading
import time

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim.navigation import GotoController, GotoParams, Pose2D, wrap  # noqa: E402

HZ = 10
COMMAND_HOLD_S = 0.5  # so lange gilt ein Geschwindigkeitsbefehl am Roboter - danach steht er von selbst
MAX_GOAL_M = 3.0
# Langsamer als in der Sim (0,5 m/s): erster Kontakt mit dem echten Roboter
PARAMS = dataclasses.replace(GotoParams(), max_vx=0.3, max_vy=0.15, max_omega=0.4, max_accel=0.3, pos_tol=0.15,
                             yaw_tol=math.radians(15))


class Odometry:
    """Pose relativ zur Startlage. Nimmt die beste Quelle, die Daten liefert, und rechnet sonst die gesendeten Befehle auf."""

    def __init__(self, robot):
        self.lock = threading.Lock()
        self.odom = self.imu_yaw = None  # juengste Messwerte (Welt des Roboters)
        self.source = "dead"
        self.est = Pose2D(0.0, 0.0, 0.0)  # Koppelnavigation, Startsystem
        self.start = None
        if robot:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
            self.subs = [ChannelSubscriber("rt/odommodestate", SportModeState_),
                         ChannelSubscriber("rt/lowstate", LowState_)]
            self.subs[0].Init(self._on_odom, 10)
            self.subs[1].Init(self._on_low, 10)
            time.sleep(2.0)
            with self.lock:
                self.source = "odom" if self.odom else "imu" if self.imu_yaw is not None else "dead"

    def _on_odom(self, msg):
        with self.lock:
            self.odom = (float(msg.position[0]), float(msg.position[1]), float(msg.imu_state.rpy[2]))

    def _on_low(self, msg):
        with self.lock:
            self.imu_yaw = float(msg.imu_state.rpy[2])

    def reset(self):
        """Aktuelle Lage wird zum Ursprung (Ziele sind relativ zu ihr)."""
        with self.lock:
            self.start = self.odom if self.source == "odom" else (0.0, 0.0, self.imu_yaw or 0.0)
            self.est = Pose2D(0.0, 0.0, 0.0)

    def integrate(self, cmd, dt):
        """Gesendeten Befehl (Koerpersystem) aufaddieren - Schaetzung, falls keine Odometrie kommt."""
        vx, vy, om = cmd
        p = self.est
        yaw = self.pose().yaw if self.source == "imu" else p.yaw
        self.est = Pose2D(p.x + (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt,
                          p.y + (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt, wrap(p.yaw + om * dt))

    def pose(self):
        with self.lock:
            if self.source == "odom":
                x0, y0, yaw0 = self.start
                x, y, yaw = self.odom
                c, s = math.cos(-yaw0), math.sin(-yaw0)
                return Pose2D(c * (x - x0) - s * (y - y0), s * (x - x0) + c * (y - y0), wrap(yaw - yaw0))
            if self.source == "imu":
                return Pose2D(self.est.x, self.est.y, wrap(self.imu_yaw - self.start[2]))
            return self.est


def go(goal, odo, loco, dry):
    """Bis zum Ziel fahren. Rueckgabe: True = angekommen."""
    ctrl = GotoController(PARAMS, hz=HZ)
    dist = math.hypot(goal.x, goal.y)
    deadline = time.monotonic() + 15.0 + dist / 0.1
    print(f"Fahre zu x {goal.x:+.2f} m, y {goal.y:+.2f} m, Blick {math.degrees(goal.yaw):+.0f} Grad (Quelle: {odo.source})")
    next_tick, last_print = time.monotonic(), 0.0
    try:
        while True:
            pose = odo.pose()
            if ctrl.arrived(pose, goal):
                break
            if time.monotonic() > deadline:
                print("Zeitlimit - angehalten")
                return False
            cmd = ctrl.step(pose, goal)
            if not dry:
                code = loco.SetVelocity(*cmd, COMMAND_HOLD_S)
                if code != 0:
                    print(f"  SetVelocity Rueckgabe {code} (nicht 0) - Roboter nimmt den Befehl nicht an?")
            if odo.source != "odom":
                odo.integrate(cmd, 1.0 / HZ)
            if time.monotonic() - last_print > 0.5:
                last_print = time.monotonic()
                d, e = ctrl.errors(pose, goal)
                print(f"  Pose x {pose.x:+.2f} y {pose.y:+.2f} Blick {math.degrees(pose.yaw):+4.0f} | Rest {d:.2f} m | "
                      f"Befehl vx {cmd[0]:+.2f} vy {cmd[1]:+.2f} om {cmd[2]:+.2f}")
            next_tick += 1.0 / HZ
            time.sleep(max(0.0, next_tick - time.monotonic()) if not dry else 0.0)
    finally:
        stop(loco, dry)
    print(f"Angekommen: x {pose.x:+.2f} y {pose.y:+.2f} Blick {math.degrees(pose.yaw):+.0f} Grad")
    return True


def stop(loco, dry):
    if not dry:
        for _ in range(3):  # mehrfach, falls ein Paket verloren geht
            loco.StopMove()
            time.sleep(0.05)


def parse_goal(values):
    x, y = float(values[0]), float(values[1])
    yaw = math.radians(float(values[2])) if len(values) > 2 else 0.0
    if math.hypot(x, y) > MAX_GOAL_M:
        raise ValueError(f"Ziel weiter als {MAX_GOAL_M} m - fuer den ersten Test zu weit")
    return Pose2D(x, y, yaw)


def main():
    parser = argparse.ArgumentParser(description="Echter G1: Gehe zu Punkt mit unserem Regler")
    parser.add_argument("--iface", help="Netzwerkadapter am Roboterkabel, z. B. en5 (networksetup -listallhardwareports)")
    parser.add_argument("--goal", nargs="+", metavar="X Y [GRAD]", help="ein Ziel, dann Ende; sonst interaktiv")
    parser.add_argument("--hin-zurueck", nargs=2, type=float, metavar=("X", "Y"),
                        help="zum Punkt gehen und von selbst zurueck zur Startlage (beide Wege im selben Startsystem)")
    parser.add_argument("--dry-run", action="store_true", help="ohne Roboter: nichts senden, Pose nur aufaddieren")
    args = parser.parse_args()
    if not args.dry_run and not args.iface:
        parser.error("--iface fehlt (oder --dry-run)")

    loco = None
    if not args.dry_run:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        ChannelFactoryInitialize(0, args.iface)
        loco = LocoClient()
        loco.SetTimeout(1.0)
        loco.Init()
    odo = Odometry(robot=not args.dry_run)
    print(f"Pose-Quelle: {odo.source}" + ("" if odo.source == "odom" else
          "  (keine Odometrie empfangen - Position wird aus den Befehlen geschaetzt, wird mit jedem Meter ungenauer)"))

    if args.hin_zurueck:
        goal = parse_goal(args.hin_zurueck)
        if args.dry_run or input("Weg frei, Roboter gesichert, Fernbedienung bereit? 'ja' zum Losgehen: ").strip() == "ja":
            odo.reset()  # nur einmal: der Rueckweg zielt auf genau diese Startlage
            if go(goal, odo, loco, args.dry_run):
                time.sleep(1.0)
                go(Pose2D(0.0, 0.0, 0.0), odo, loco, args.dry_run)
        stop(loco, args.dry_run)
        return 0

    goals = [args.goal] if args.goal else None
    while True:
        if goals is not None:
            if not goals:
                break
            values = goals.pop()
        else:
            text = input("\nZiel 'x y [grad]' relativ zur jetzigen Lage (z. B. '1 0' = 1 m vor), leer = Ende: ").strip()
            if not text:
                break
            values = text.split()
        try:
            goal = parse_goal(values)
        except (ValueError, IndexError) as error:
            print(f"Ungueltig: {error}")
            continue
        if not args.dry_run and input("Weg frei, Roboter gesichert, Fernbedienung bereit? 'ja' zum Losgehen: ").strip() != "ja":
            continue
        odo.reset()
        go(goal, odo, loco, args.dry_run)
    stop(loco, args.dry_run)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nCtrl+C - angehalten")  # go() hat im finally schon StopMove gesendet
        sys.exit(130)
