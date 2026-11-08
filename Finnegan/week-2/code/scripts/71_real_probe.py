"""Echter G1: nur zuhoeren, nichts bewegen - welche Datenkanaele sendet der Roboter, kommen Gelenk- und Lagedaten an?

Beantwortet vor allem drei Fragen fuer die naechsten Schritte:
  1. Steht die Verbindung? (rt/lowstate kommt an, mit welcher Rate)
  2. Was gibt es fuer "Umgebung verstehen"? Alle Kanaele (DDS-Topics) im Netz, LiDAR- und Odometrie-Kanaele markiert
  3. 23 oder 29 DoF? Hinweis aus den Handgelenk- und Taillenmotoren (Werte und Temperaturen). Nur ein Hinweis - der sichere Test bleibt:
     Kann die Hand angetrieben am Handgelenk nicken?
Sendet nichts, ruft keine Dienste auf. Umgebung ~/dev/g1-ext (Mac per Kabel, feste IP 192.168.123.222/24):
    DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib ~/dev/g1-ext/.venv/bin/python scripts/71_real_probe.py --iface en5
Ausgabe zusaetzlich in runs/real_probe.json.
"""
import argparse
import json
import pathlib
import time

from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
from unitree_sdk2py.core.channel import ChannelFactory, ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

REPO = pathlib.Path(__file__).resolve().parent.parent
# G1-Motorreihenfolge (unitree_hg, 29 DoF): Taille 12-14, linkes Handgelenk 19-21, rechtes 26-28 (Roll, Pitch, Yaw)
WATCH = {"Taille Gieren": 12, "Taille Rollen": 13, "Taille Nicken": 14,
         "li Handgelenk Rollen": 19, "li Handgelenk Nicken": 20, "li Handgelenk Gieren": 21,
         "re Handgelenk Rollen": 26, "re Handgelenk Nicken": 27, "re Handgelenk Gieren": 28}
HINTS = {"lidar": ("lidar", "cloud", "livox", "mid360", "scan"), "odometrie": ("odom", "sportmode", "pose"),
         "kamera": ("camera", "image", "video", "realsense")}


def main():
    parser = argparse.ArgumentParser(description="Echter G1: nur zuhoeren")
    parser.add_argument("--iface", required=True, help="Netzwerkadapter am Roboterkabel, z. B. en5")
    parser.add_argument("--seconds", type=float, default=5.0)
    args = parser.parse_args()

    ChannelFactoryInitialize(0, args.iface)
    received = {"rt/lowstate": [], "rt/odommodestate": []}
    subs = [ChannelSubscriber("rt/lowstate", LowState_), ChannelSubscriber("rt/odommodestate", SportModeState_)]
    subs[0].Init(lambda msg: received["rt/lowstate"].append(msg), 10)
    subs[1].Init(lambda msg: received["rt/odommodestate"].append(msg), 10)
    # Alle Kanaele im Netz ueber die eingebaute DDS-Entdeckung, ohne ihre Datentypen zu kennen.
    # Das SDK haelt seinen DomainParticipant privat - hier ueber den Python-Namen der privaten Variable.
    participant = ChannelFactory()._ChannelFactory__participant
    discovery = BuiltinDataReader(participant, BuiltinTopicDcpsPublication)
    print(f"Hoere {args.seconds:.0f} s zu auf {args.iface} ...")
    time.sleep(args.seconds)

    topics = sorted({(s.topic_name, s.type_name) for s in discovery.take(N=10000)})
    report = {"iface": args.iface, "seconds": args.seconds, "topics": [{"name": n, "type": t} for n, t in topics]}
    print(f"\n1. Kanaele im Netz: {len(topics)}")
    for name, typ in topics:
        tags = [k for k, words in HINTS.items() if any(w in name.lower() for w in words)]
        print(f"   {name:45s} {typ:50s} {' <- ' + ', '.join(tags) if tags else ''}")
    if not topics:
        print("   KEINE - Verbindung pruefen: Kabel, IP 192.168.123.222/24 am richtigen Adapter, ping 192.168.123.164, Firewall")

    low = received["rt/lowstate"]
    rate = len(low) / args.seconds
    print(f"\n2. rt/lowstate (Gelenke + Lagesensor): {len(low)} Nachrichten = {rate:.0f} Hz")
    report["lowstate_hz"] = rate
    if low:
        msg = low[-1]
        rpy = [round(float(v), 3) for v in msg.imu_state.rpy]
        print(f"   mode_machine {msg.mode_machine} · Lage rpy (rad) {rpy}")
        print("   Motor                   Winkel (rad)  Temperatur")
        motors = {}
        for label, i in WATCH.items():
            m = msg.motor_state[i]
            temp = list(m.temperature) if hasattr(m.temperature, "__len__") else [m.temperature]
            motors[label] = {"index": i, "q": round(float(m.q), 4), "temperature": [int(t) for t in temp]}
            print(f"   {label:22s} {float(m.q):+10.4f}    {temp}")
        report.update(mode_machine=int(msg.mode_machine), imu_rpy=rpy, motors=motors)
        silent = [k for k, v in motors.items() if "Nicken" in k and "Handgelenk" in k and v["q"] == 0.0 and not any(v["temperature"])]
        print("   Hinweis DoF: " + ("Handgelenk-Nicken meldet Winkel 0 und Temperatur 0 -> eher 23 DoF (Motor fehlt)" if silent else
                                  "Handgelenk-Motoren melden Werte -> eher 29 DoF") + " - sicher ist nur der Test von Hand")

    odom = received["rt/odommodestate"]
    print(f"\n3. rt/odommodestate (Odometrie): {len(odom)} Nachrichten")
    if odom:
        p = [round(float(v), 3) for v in odom[-1].position]
        print(f"   Position {p} m - 70_real_goto.py nutzt sie automatisch (Pose-Quelle 'odom')")
        report["odom_position"] = p
    else:
        print("   keine - 70_real_goto.py schaetzt die Position dann aus Blickrichtung (Lagesensor) und Befehlen")
    report["odom_messages"] = len(odom)

    out = REPO / "runs" / "real_probe.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(f"\ngespeichert: {out.relative_to(REPO)}")


if __name__ == "__main__":
    main()
