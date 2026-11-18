"""Real G1 flight recorder: listens next to 84/86 (second terminal) and records, once per second, which layer is healthy.

Why it exists: on the first robot day a failure has several possible causes - the cable/network, the robot's sensor stream,
the hands, someone else steering the arms (an owner tool, a SLAM navigator), or the robot walking. 91 records all of them
side by side with timestamps, so afterwards the FIRST thing that went wrong is known instead of guessed. Read-only: the
publisher guard is installed before DDS starts, so 91 cannot create a writer (as 84). It does not open the camera (one
reader at a time; 84 --steps camera checks the camera, 86 logs its own camera errors).

Layers, each with a state per second (OK / WARN / FAIL):
  NET    the robot-network address on --iface, one ping to --ping (robot PC2) every 2 s, round-trip time
  STATE  rt/lowstate rate and age (the robot's joint/IMU stream; about 500 Hz on the robot)
  HANDS  rt/brainco/left|right/state rate and age
  WRITERS every DDS publisher on the network (discovery); a NEW one is logged with time and topic. Several participants
         registered on rt/arm_sdk or rt/lowcmd = WARN (a fight if both send; ARM shows it). Discovery sees senders,
         not messages; a crashed program stays listed for about 10 s. api/*/request or cmd_vel publishers =
         someone sends walk/sport commands (a navigator)
  ARM    the newest rt/arm_sdk command (weight > 0) vs the measured arm angles: max error in rad. A large error that
         stays = the arm does not follow (blocked, or another controller)
  BASE   rt/odommodestate / rt/sportmodestate velocity and the IMU roll/pitch: is the robot walking or tilting
Output: one ALERT line the moment a layer turns WARN/FAIL, a status line every --print-every s, every row in
runs/watch/watch_<stamp>.jsonl, and at the end a summary per layer (count, first and last problem with times).
Robot env (Python 3.10), from the repo root, in a SECOND terminal while 84 or 86 runs:
    python scripts/91_watch.py --iface en5                     # until Ctrl+C
Sim (loopback only): python scripts/91_watch.py --iface lo0 --ping 127.0.0.1 --seconds 30
"""
import argparse
import datetime
import json
import pathlib
import re
import subprocess
import sys
import threading
import time

import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
from g1sim import real_io  # noqa: E402

real_io.install_publisher_guard()   # before any DDS object exists: 91 can never send

from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication  # noqa: E402
from cyclonedds.core import InstanceState  # noqa: E402
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_  # noqa: E402

OUT_DIR = REPO / "runs" / "watch"
PC2 = "192.168.123.164"
PING_EVERY_S = 2.0
STATE_STALE_S = 0.1          # = 86's own limit (real_timing.STALE_S lowstate): a gap 91 reports is one 86 faults on
HAND_STALE_S = 0.5
ARM_ERR_WARN = 0.30          # rad between our command and the measured angle, held ARM_ERR_HOLD_S = WARN
ARM_ERR_HOLD_S = 1.0
BASE_MOVING = 0.05           # m/s body speed = the robot walks
TILT_WARN = 0.25             # rad IMU roll or pitch
ODOM_TOPICS = ("rt/odommodestate", "rt/sportmodestate")
WATCHED_CMD = ("rt/arm_sdk", "rt/lowcmd")


class Pinger(threading.Thread):
    """Pings `host` every PING_EVERY_S in the background; keeps the last result (ok, ms)."""

    def __init__(self, host):
        super().__init__(daemon=True)
        self.host, self.result = host, (None, None)

    def run(self):
        while True:
            try:
                out = subprocess.run(["ping", "-c", "1", "-t" if sys.platform == "darwin" else "-W", "1", self.host],
                                     capture_output=True, text=True, timeout=3).stdout
                match = re.search(r"time[=<]([\d.]+)", out)
                self.result = (match is not None, float(match.group(1)) if match else None)
            except (subprocess.TimeoutExpired, OSError):
                self.result = (False, None)
            time.sleep(PING_EVERY_S)


class Writers:
    """Every DDS publication on the network from discovery: {key: (topic, participant)}; new ones are returned once."""

    def __init__(self, participant):
        self.own = participant.guid
        self.reader = BuiltinDataReader(participant, BuiltinTopicDcpsPublication)
        self.alive = {}

    def poll(self):
        """(new [(topic, participant)], gone [(topic, participant)]) since the last poll."""
        new, gone = [], []
        for s in self.reader.take(N=real_io.MAX_DISCOVERY_SAMPLES):
            info = getattr(s, "sample_info", None)
            key = str(getattr(s, "key", ""))
            if info is not None and info.instance_state != InstanceState.Alive:
                if key in self.alive:
                    gone.append(self.alive.pop(key))
                continue
            if not getattr(s, "topic_name", None) or s.participant_key == self.own or key in self.alive:
                continue
            self.alive[key] = (s.topic_name, str(s.participant_key)[:8])
            new.append(self.alive[key])
        return new, gone

    def participants_on(self, topic):
        return sorted({p for t, p in self.alive.values() if t == topic})

    def api_requests(self):
        return sorted({t for t, _ in self.alive.values() if re.search(r"api/[^/]+/request|cmd_vel", t)})


def layer(states, name, level, text):
    states[name] = (level, text)


def check(args, readers, pinger, writers, row, hold):
    """Fill `row` with the measurements and return {layer: (level, text)}."""
    states = {}
    has_ip = any(ip.startswith(real_io.ROBOT_NET_PREFIX) for ip in real_io.interface_addresses().get(args.iface, []))
    ok, ms = pinger.result
    row["net"] = {"robot_ip_on_iface": has_ip, "ping_ok": ok, "ping_ms": ms}
    if not args.sim and not has_ip:
        layer(states, "NET", "FAIL", f"no {real_io.ROBOT_NET_PREFIX}x address on {args.iface} (cable/adapter/IP)")
    elif ok is False:
        layer(states, "NET", "FAIL", f"ping {args.ping} lost (cable, switch or robot PC)")
    else:
        layer(states, "NET", "OK", f"ping {ms} ms" if ms is not None else "ping pending")

    low, age = readers["state"].latest()
    hz = readers["state"].rate_hz()
    limit = real_io.LOWSTATE_HZ["loopback" if args.sim else "robot"]
    row["state"] = {"hz": round(hz, 1), "age_s": None if low is None else round(age, 3)}
    if low is None or age > STATE_STALE_S:
        layer(states, "STATE", "FAIL", "rt/lowstate " + ("never arrived" if low is None else f"stopped {age:.2f} s ago"))
    elif hz < limit["warn"]:
        layer(states, "STATE", "WARN", f"rt/lowstate only {hz:.0f} Hz (expected >= {limit['warn']})")
    else:
        layer(states, "STATE", "OK", f"{hz:.0f} Hz")

    hands = []
    for side in ("left", "right"):
        msg, h_age = readers[side].latest()
        row[f"hand_{side}"] = {"hz": round(readers[side].rate_hz(), 1), "age_s": None if msg is None else round(h_age, 3)}
        if msg is None or h_age > HAND_STALE_S:
            hands.append(f"{side} " + ("never arrived" if msg is None else f"stopped {h_age:.1f} s ago"))
    layer(states, "HANDS", "FAIL" if hands else "OK", ", ".join(hands) or
          f"{row['hand_left']['hz']:.0f}/{row['hand_right']['hz']:.0f} Hz")

    new, gone = writers.poll()
    row["writers_new"], row["writers_gone"] = new, gone
    arm_parts = {t: writers.participants_on(t) for t in WATCHED_CMD}
    api = writers.api_requests()
    row["arm_cmd_participants"], row["api_requests"] = arm_parts, api
    fights = [f"{t}: {len(p)} programs" for t, p in arm_parts.items() if len(p) > 1]
    both = arm_parts["rt/arm_sdk"] and arm_parts["rt/lowcmd"] and set(arm_parts["rt/lowcmd"]) - set(arm_parts["rt/arm_sdk"])
    if fights:
        layer(states, "WRITERS", "WARN", "two programs registered as arm senders - " + "; ".join(fights) +
              " (a fight if both send: then ARM warns too; 86 refuses on a second sender's messages)")
    elif api:
        layer(states, "WRITERS", "WARN", "walk/sport command senders active (api request / cmd_vel): " + ", ".join(api) + " (navigator?)")
    else:
        layer(states, "WRITERS", "OK", f"arm_sdk senders {len(arm_parts['rt/arm_sdk'])}" +
              (", lowcmd sender (robot's own controller)" if both else ""))

    cmd, cmd_age = readers["arm_cmd"].latest()
    err = None
    if cmd is not None and low is not None and cmd_age < 0.5 and float(cmd.motor_cmd[real_io.WEIGHT_SLOT].q) > 0:
        err = max(abs(float(cmd.motor_cmd[i].q) - float(low.motor_state[i].q)) for i in real_io.ARM_MOTORS)
    row["arm_err_rad"] = None if err is None else round(err, 3)
    now = time.monotonic()
    if err is not None and err > ARM_ERR_WARN:
        hold.setdefault("since", now)
    else:
        hold.pop("since", None)
    if "since" in hold and now - hold["since"] >= ARM_ERR_HOLD_S:
        layer(states, "ARM", "WARN", f"arm {err:.2f} rad away from our command for {now - hold['since']:.0f} s "
                                     "(blocked, or another controller)")
    else:
        layer(states, "ARM", "OK", "no command" if err is None else f"error {err:.2f} rad")

    speed, src = None, None
    for topic in ODOM_TOPICS:
        msg, o_age = readers[topic].latest()
        if msg is not None and o_age < 1.0:
            speed, src = float(np.linalg.norm(np.asarray(msg.velocity, dtype=float)[:2])), topic
            break
    tilt = real_io.imu_roll_pitch(low) if low is not None else np.full(2, np.nan)
    row["base"] = {"speed_mps": None if speed is None else round(speed, 3), "from": src,
                   "roll_pitch": [None if np.isnan(v) else round(float(v), 3) for v in tilt]}
    if speed is not None and speed > BASE_MOVING:
        layer(states, "BASE", "WARN", f"robot moves {speed:.2f} m/s ({src}) - walking/navigator active")
    elif np.nanmax(np.abs(tilt), initial=0) > TILT_WARN:
        layer(states, "BASE", "WARN", f"torso tilted roll/pitch {np.round(tilt, 2).tolist()} rad")
    else:
        layer(states, "BASE", "OK", "still" if speed is not None else "no odometry topic (IMU only)")
    return states


def main(argv=None):
    parser = argparse.ArgumentParser(description="Real G1 flight recorder (read-only, second terminal next to 84/86)")
    parser.add_argument("--iface", required=True, help="network interface to the robot (84 --show-ip), lo0 for the sim")
    parser.add_argument("--ping", default=PC2, help=f"address to ping (default robot PC2 {PC2})")
    parser.add_argument("--seconds", type=float, default=0, help="stop after this time (0 = until Ctrl+C)")
    parser.add_argument("--print-every", type=float, default=5.0, help="status line interval in seconds")
    args = parser.parse_args(argv)
    args.sim = real_io.is_loopback(args.iface)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = OUT_DIR / f"watch_{stamp}.jsonl"
    readers = {"state": real_io.LowStateReader(args.iface),
               "left": real_io.HandReader(args.iface, "left"), "right": real_io.HandReader(args.iface, "right"),
               "arm_cmd": real_io.LatestReader(args.iface, "rt/arm_sdk", LowCmd_)}
    for topic in ODOM_TOPICS:
        readers[topic] = real_io.LatestReader(args.iface, topic, SportModeState_)
    writers = Writers(real_io.dds_participant())
    pinger = Pinger(args.ping)
    pinger.start()
    print(f"WATCH: recording to {out_path.relative_to(REPO)} - read-only, Ctrl+C to stop", flush=True)
    time.sleep(1.5)                              # first samples, first ping, discovery

    previous, problems, hold = {}, {}, {}
    start, last_print = time.monotonic(), 0.0
    try:
        with out_path.open("w") as out:
            while not args.seconds or time.monotonic() - start < args.seconds:
                clock = datetime.datetime.now().strftime("%H:%M:%S")
                row = {"time": clock}
                states = check(args, readers, pinger, writers, row, hold)
                row["layers"] = {k: list(v) for k, v in states.items()}
                out.write(json.dumps(row) + "\n")
                out.flush()
                for topic, part in row["writers_new"]:
                    print(f"WATCH {clock} NEW SENDER {topic} (program {part})", flush=True)
                for topic, part in row["writers_gone"]:
                    print(f"WATCH {clock} sender gone {topic} (program {part})", flush=True)
                for name, (level, text) in states.items():
                    if level != "OK" and previous.get(name, ("OK",))[0] != level:
                        print(f"ALERT {clock} {name} {level}: {text}", flush=True)
                        problems.setdefault(name, []).append(f"{clock} {level}: {text}")
                    elif level == "OK" and previous.get(name, ("OK",))[0] != "OK":
                        print(f"WATCH {clock} {name} back to OK: {text}", flush=True)
                previous = states
                if time.monotonic() - last_print >= args.print_every:
                    last_print = time.monotonic()
                    print(f"WATCH {clock} " + " | ".join(f"{k} {v[0]}" for k, v in states.items()), flush=True)
                time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    print("\nWATCH SUMMARY (problems per layer; compare the times: the earliest one is usually the cause):")
    for name in ("NET", "STATE", "HANDS", "WRITERS", "ARM", "BASE"):
        seen = problems.get(name, [])
        print(f"  {name:8s} " + ("no problem seen" if not seen else f"{len(seen)}x, first {seen[0]}" +
                                   (f" | last {seen[-1]}" if len(seen) > 1 else "")))
    print(f"  file: {out_path.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
