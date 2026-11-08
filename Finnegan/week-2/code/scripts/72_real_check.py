"""Echter G1: die vier Grundlagen fuers Greifen einzeln pruefen - Gelenke lesen, Arm bewegen, Hand bewegen, Kopfkamerabild holen.

    gelenke  alle 29 Gelenkwinkel aus rt/lowstate live anzeigen (bewegt nichts). Arm von Hand bewegen -> Zahl muss sich aendern
    arm      EIN Armgelenk langsam um wenige Grad bewegen und zurueck, ueber rt/arm_sdk (der Werks-Lauf-Controller laeuft weiter)
    hand     BrainCo-Hand: --pruefen (per SSH: laeuft brainco_hand_server auf dem Bordrechner?), ohne Option Zustand lesen,
             --bewegen Zeigefinger langsam halb zu und wieder auf
    kamera   Kopfkamera per SSH auf dem Bordrechner: --suchen (welche /dev/video*, ist die D455 dort?), --geraet N ein Bild holen
Roboter-Modus fuer arm: normaler Bewegungsmodus (Fernbedienung L2+B Daempfung -> L2+hoch Stehen -> R1+X), NICHT Debug-Modus (L2+R2) -
dort hoert niemand auf rt/arm_sdk (Unitree xr_teleoperate). Nie LeRobots run_g1_server.py starten: Es schaltet den Werks-Controller ab.
Hand und Kamera haengen am Bordrechner (192.168.123.164, ssh unitree@..., Passwort laut LeRobot-Doku 123), nicht am Bewegungsrechner.
Arm-Test, anders als Unitrees Beispiel g1_arm7_sdk_dds_example.py: Das faehrt erst alle Arme in die Nullstellung - hier bleiben alle
Arm- und Taillengelenke dort, wo sie beim Start stehen, nur das gewaehlte Gelenk bewegt sich (Standard: rechter Ellbogen +0,25 rad
= 14 Grad). Ablauf: Uebernahme einblenden (arm_sdk-Gewicht 0 -> 1, 2 s) -> hin (3 s) -> halten (1 s) -> zurueck (3 s) -> Uebernahme
ausblenden (1 -> 0, 2 s). Ctrl+C blendet sofort langsam aus statt abzubrechen. Steifigkeit wie im Unitree-Beispiel (kp 60, kd 1,5).
Umgebung ~/dev/g1-ext (Mac per Kabel, feste IP 192.168.123.222/24):
    DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib ~/dev/g1-ext/.venv/bin/python scripts/72_real_check.py gelenke --iface en5
    ... scripts/72_real_check.py arm --iface en5 [--gelenk RightElbow] [--um 0.25]
    ... scripts/72_real_check.py arm --dry-run        # ohne Roboter: nur den geplanten Verlauf zeigen
"""
import argparse
import math
import sys
import threading
import time

# G1-Motorreihenfolge unitree_hg (Unitree g1_arm7_sdk_dds_example.py). Bei 23 DoF gibt es WristPitch/WristYaw und WaistRoll/WaistPitch nicht.
JOINTS = ["LeftHipPitch", "LeftHipRoll", "LeftHipYaw", "LeftKnee", "LeftAnklePitch", "LeftAnkleRoll",
          "RightHipPitch", "RightHipRoll", "RightHipYaw", "RightKnee", "RightAnklePitch", "RightAnkleRoll",
          "WaistYaw", "WaistRoll", "WaistPitch",
          "LeftShoulderPitch", "LeftShoulderRoll", "LeftShoulderYaw", "LeftElbow", "LeftWristRoll", "LeftWristPitch", "LeftWristYaw",
          "RightShoulderPitch", "RightShoulderRoll", "RightShoulderYaw", "RightElbow", "RightWristRoll", "RightWristPitch",
          "RightWristYaw"]
# Taille-Gieren + beide Arme halten wir; WaistRoll/WaistPitch (13, 14) nicht - bei 23 DoF oder gesperrter Taille gibt es sie nicht
ARM_SDK_JOINTS = [12] + list(range(15, 29))
WEIGHT_SLOT = 29                       # motor_cmd[29].q = Gewicht der Uebernahme, 0 = Werks-Controller, 1 = wir
KP, KD, DT = 60.0, 1.5, 0.02
MAX_DELTA = 0.35                       # rad, groesster erlaubter Ausschlag im Test (20 Grad)


class Robot:
    def __init__(self, iface):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        ChannelFactoryInitialize(0, iface)
        self.lock, self.state = threading.Lock(), None
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self._on_state, 10)
        for _ in range(30):
            if self.state is not None:
                return
            time.sleep(0.1)
        raise SystemExit("Keine Daten auf rt/lowstate - Verbindung pruefen (scripts/71_real_probe.py)")

    def _on_state(self, msg):
        with self.lock:
            self.state = msg

    def q(self):
        with self.lock:
            return [float(self.state.motor_state[i].q) for i in range(len(JOINTS))]


def cmd_gelenke(args):
    robot = Robot(args.iface)
    print("Gelenkwinkel live (rad, in Klammern Grad). Bewege einen Arm vorsichtig von Hand - die Zahl muss mitlaufen. Ctrl+C = Ende")
    try:
        while True:
            q = robot.q()
            lines = [f"{n:18s} {v:+7.3f} ({math.degrees(v):+6.1f})" for n, v in zip(JOINTS, q)]
            print("\033[2J\033[H" + "\n".join(f"{i:2d} {line}" for i, line in enumerate(lines)))
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Ende")


def arm_profile(start, delta):
    """Zeitplan (Sekunden, Gewicht, Ausschlag) des Arm-Tests - weich ein- und ausgeblendet."""
    smooth = lambda a: 0.5 - 0.5 * math.cos(math.pi * min(max(a, 0.0), 1.0))  # 0 -> 1 ohne Ruck
    phases = [(2.0, lambda a: (smooth(a), 0.0)), (3.0, lambda a: (1.0, delta * smooth(a))), (1.0, lambda a: (1.0, delta)),
              (3.0, lambda a: (1.0, delta * (1.0 - smooth(a)))), (2.0, lambda a: (1.0 - smooth(a), 0.0))]
    t = 0.0
    for duration, f in phases:
        for k in range(int(duration / DT)):
            weight, d = f(k * DT / duration)
            yield t, weight, start + d
            t += DT
    yield t, 0.0, start


def cmd_arm(args):
    if args.gelenk not in JOINTS or JOINTS.index(args.gelenk) < 15:
        raise SystemExit(f"--gelenk muss ein Armgelenk sein: {', '.join(JOINTS[15:])}")
    if abs(args.um) > MAX_DELTA:
        raise SystemExit(f"--um hoechstens {MAX_DELTA} rad im Test")
    joint = JOINTS.index(args.gelenk)
    if args.dry_run:
        for t, w, target in list(arm_profile(0.0, args.um))[::25]:
            print(f"t {t:5.2f} s  Gewicht {w:.2f}  {args.gelenk} Soll {target:+.3f} rad (relativ zum Start)")
        return

    from unitree_sdk2py.core.channel import ChannelPublisher
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from unitree_sdk2py.utils.crc import CRC
    robot = Robot(args.iface)
    hold = robot.q()  # alle Gelenke bleiben hier - kein Sprung in eine Nullstellung
    print(f"Startwinkel {args.gelenk}: {hold[joint]:+.3f} rad. Plan: {args.um:+.2f} rad ({math.degrees(args.um):+.0f} Grad) hin und zurueck, "
          "ca. 11 s.")
    if input("Arm frei, Roboter gesichert? 'ja' zum Starten: ").strip() != "ja":
        return
    pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
    pub.Init()
    cmd, crc = unitree_hg_msg_dds__LowCmd_(), CRC()

    def send(weight, target):
        cmd.motor_cmd[WEIGHT_SLOT].q = weight
        for i in ARM_SDK_JOINTS:
            m = cmd.motor_cmd[i]
            m.q, m.dq, m.tau, m.kp, m.kd = (target if i == joint else hold[i]), 0.0, 0.0, KP, KD
        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

    weight, max_err = 0.0, 0.0
    next_tick = time.monotonic()
    try:
        for t, weight, target in arm_profile(hold[joint], args.um):
            send(weight, target)
            if weight > 0.99:
                max_err = max(max_err, abs(robot.q()[joint] - target))
            next_tick += DT
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        print("\nCtrl+C - blende die Uebernahme langsam aus (2 s)")
        target = robot.q()[joint]
        for k in range(int(2.0 / DT)):
            send(weight * (1.0 - k * DT / 2.0), target)
            time.sleep(DT)
    send(0.0, hold[joint])
    end = robot.q()[joint]
    print(f"Fertig. {args.gelenk}: groesste Abweichung Soll/Ist unter voller Uebernahme {max_err:.3f} rad, "
          f"Endwinkel {end:+.3f} rad (Start {hold[joint]:+.3f}).")
    print("Bestanden, wenn sich der Arm sichtbar bewegt hat und die Abweichung klein ist (< 0,05 rad)." if max_err < 0.05 else
          "Arm folgte nicht genau - Modus des Roboters pruefen (Werks-Controller aktiv? arm_sdk erlaubt?).")


ROBOT_PC = "unitree@192.168.123.164"
FINGERS = ["Thumb", "ThumbAux", "Index", "Middle", "Ring", "Pinky"]  # Reihenfolge brainco_hand_service (README); 0 = offen, 1 = zu
# Was auf dem Bordrechner nachgesehen wird (nur lesen): serielle Ports der Haende, Dienst, gebaute Programme
HAND_CHECK = ("ls -l /dev/ttyUSB* /dev/ttyHAND* /dev/ttyUN* 2>/dev/null || echo 'keine seriellen Hand-Ports';"
              "systemctl status brainco_hand.service --no-pager 2>&1 | head -5;"
              "pgrep -a brainco_hand || echo 'brainco_hand_server laeuft nicht';"
              "ls ~/brainco_hand_service/bin 2>/dev/null || echo 'kein ~/brainco_hand_service/bin'")
CAMERA_FIND = ("lsusb | grep -i 8086 || echo 'keine Intel-RealSense am Bordrechner (haengt sie am AGX-Rucksack?)';"
               "for d in /dev/video*; do echo \"$d: $(v4l2-ctl -d $d --list-formats 2>/dev/null | grep -oE 'YUYV|MJPG|Z16|GREY' | sort -u | tr '\\n' ' ')\"; done;"
               "python3 -c 'import cv2; print(\"opencv\", cv2.__version__)' 2>&1 | tail -1")
# Laeuft auf dem Bordrechner: ein Farbbild aufnehmen und als Base64-JPEG ausgeben (kein Server, kein offener Port)
CAMERA_GRAB = """
import base64, sys, cv2
c = cv2.VideoCapture({device}, cv2.CAP_V4L2)
c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV')); c.set(3, 640); c.set(4, 480)
ok, f = False, None
for _ in range(15):  # Belichtung einregeln lassen
    ok, f = c.read()
c.release()
if not ok or f is None:
    sys.exit('kein Bild von /dev/video{device}')
print('BILD', f.shape, base64.b64encode(cv2.imencode('.jpg', f)[1].tobytes()).decode())
"""


def ssh(command, stdin=None):
    """Befehl auf dem Bordrechner (fragt nach dem Passwort). Rueckgabe: Ausgabe."""
    import subprocess
    proc = subprocess.run(["ssh", "-o", "ConnectTimeout=5", ROBOT_PC, command], input=stdin, capture_output=True, text=True)
    if proc.returncode != 0 and not proc.stdout:
        print(proc.stderr.strip())
    return proc.stdout


def cmd_hand(args):
    if args.pruefen:
        print(ssh(HAND_CHECK))
        print("Laeuft der Dienst nicht: auf dem Bordrechner github.com/unitreerobotics/brainco_hand_service bauen und\n"
              "  cd ~/brainco_hand_service/bin && sudo ./brainco_hand_server --network eth0   starten (Haende oeffnen sich dabei).")
        return
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
    ChannelFactoryInitialize(0, args.iface)
    latest = {}
    sub = ChannelSubscriber(f"rt/brainco/{args.seite}/state", MotorStates_)
    sub.Init(lambda msg: latest.__setitem__("s", msg), 10)
    time.sleep(2.0)
    if "s" not in latest:
        raise SystemExit(f"Kein rt/brainco/{args.seite}/state - brainco_hand_server laeuft nicht (pruefen mit: hand --pruefen)")

    def show(label):
        s = latest["s"].states[:6]
        print(f"{label:10s} " + "  ".join(f"{n} {m.q:.2f}" for n, m in zip(FINGERS, s)) +
              f"   Strom {max(m.tau_est for m in s):.2f} A")
    show("jetzt")
    if not args.bewegen:
        return
    if input("Finger frei, nichts im Greifbereich? 'ja' zum Bewegen: ").strip() != "ja":
        return
    pub = ChannelPublisher(f"rt/brainco/{args.seite}/cmd", MotorCmds_)
    pub.Init()

    def send(index_q, seconds):
        msg = MotorCmds_()
        msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(6)]  # immer 6 Eintraege - der Dienst prueft das nicht
        for i in range(6):
            msg.cmds[i].q, msg.cmds[i].dq = (index_q if i == 2 else 0.0), 0.3  # langsam (Werkstest nimmt 1,0)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            pub.Write(msg)
            time.sleep(0.02)
    send(0.5, 2.0)
    show("halb zu")
    send(0.0, 2.0)
    show("offen")
    print("Bestanden, wenn der Zeigefinger sichtbar halb zu und wieder auf ging und 'Index' etwa 0,5 bzw. 0 zeigt.")


def cmd_kamera(args):
    import base64
    import pathlib
    if args.geraet is None:
        print(ssh(CAMERA_FIND))
        print("Das Farbbild ist das /dev/videoN mit YUYV. Dann: kamera --geraet N")
        return
    out = ssh("python3 -", stdin=CAMERA_GRAB.format(device=int(args.geraet)))
    line = next((l for l in out.splitlines() if l.startswith("BILD")), None)
    if line is None:
        raise SystemExit(f"Kein Bild. Ausgabe vom Roboter:\n{out}")
    path = pathlib.Path(__file__).resolve().parent.parent / "runs" / "real_head_camera.jpg"
    path.parent.mkdir(exist_ok=True)
    path.write_bytes(base64.b64decode(line.rsplit(" ", 1)[1]))
    print(f"Bild gespeichert: {path} ({line.split(')')[0][5:]})")
    import subprocess
    subprocess.run(["open", str(path)])


def main():
    parser = argparse.ArgumentParser(description="Echter G1: Grundlagen fuers Greifen pruefen")
    sub = parser.add_subparsers(dest="test", required=True)
    p = sub.add_parser("gelenke", help="Gelenkwinkel live lesen (bewegt nichts)")
    p.add_argument("--iface", required=True)
    p.set_defaults(run=cmd_gelenke)
    p = sub.add_parser("arm", help="ein Armgelenk wenige Grad hin und zurueck")
    p.add_argument("--iface")
    p.add_argument("--gelenk", default="RightElbow")
    p.add_argument("--um", type=float, default=0.25, help="Ausschlag in rad (Standard 0,25 = 14 Grad)")
    p.add_argument("--dry-run", action="store_true")
    p.set_defaults(run=cmd_arm)
    p = sub.add_parser("hand", help="BrainCo-Hand pruefen, lesen, Zeigefinger bewegen")
    p.add_argument("--iface")
    p.add_argument("--seite", choices=["left", "right"], default="right")
    p.add_argument("--pruefen", action="store_true", help="per SSH: laeuft brainco_hand_server?")
    p.add_argument("--bewegen", action="store_true", help="Zeigefinger halb zu und wieder auf")
    p.set_defaults(run=cmd_hand)
    p = sub.add_parser("kamera", help="Kopfkamera per SSH: suchen oder ein Bild holen")
    p.add_argument("--geraet", type=int, help="Nummer N von /dev/videoN (aus --suchen); ohne = suchen")
    p.add_argument("--suchen", action="store_true", help="(Standard ohne --geraet)")
    p.set_defaults(run=cmd_kamera)
    args = parser.parse_args()
    if (args.test == "arm" and not args.dry_run or args.test == "hand" and not args.pruefen) and not args.iface:
        parser.error("--iface fehlt")
    args.run(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
