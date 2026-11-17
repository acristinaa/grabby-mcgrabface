"""Stufe 0.3d - BrainCo-Haende per DDS-Befehl in der LeRobot-G1-Sim.

Prueft die Sim-Bruecke rt/brainco/{left,right}/{cmd,state} - dieselben Topics und Nachrichten wie an den echten
BrainCo-Haenden (unitree_go MotorCmds_/MotorStates_, 6 Motoren, q normiert 0..1, 0 = offen):
  1. G1 steht frei mit dem GR00T-Lauf-Controller (Gummiband geloest)
  2. per DDS, drei Phasen (PHASES): rechte Finger zu + linke Hand halb, dann beide Daumen drehen, dann alles auf
Abnahme: der ueber DDS gelesene Zustand erreicht jedes Ziel auf 0,1 genau und stimmt mit den Sim-Gelenken ueberein,
jede Fingerspitze der rechten Hand bewegt sich > 1 cm, kein Sturz.

Vorher einmal scripts/51_build_brainco_sim.py. Laeuft in der Spike-Umgebung, ohne Fenster:
    DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib PYTHONUNBUFFERED=1 \
        ~/dev/g1-ext/.venv/bin/python scripts/52_brainco_hand_dds.py [--video [PFAD]]
Ausgabe: runs/stufe0_3d_brainco_dds.json, Video docs/videos/stufe0_3d_haende_dds.mp4
"""
import argparse
import json
import os
import pathlib
import sys
import threading
import time
import traceback
from functools import partial

import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber

# macOS: lo0 statt lo, vor jedem LeRobot-Import (Begruendung in 50_spike_lerobot_walk.py)
ChannelFactoryInitialize(0, "lo0")

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config  # noqa: E402
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_  # noqa: E402
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_  # noqa: E402

from g1sim import brainco  # noqa: E402
from g1sim.brainco_env import use_brainco_sim  # noqa: E402
from g1sim.runtime import StallWatch, free_camera_port  # noqa: E402
from g1sim.video import VideoRecorder  # noqa: E402

OUT = REPO / "runs" / "stufe0_3d_brainco_dds.json"
VIDEO = REPO / "docs" / "videos" / "stufe0_3d_haende_dds.mp4"

FALL_HEIGHT = 0.5
BAND_S = 3.0          # Wanduhr am Gummiband
STAND_SIM_S = 1.5     # Sim-Zeit frei stehen vor dem ersten Handbefehl
MOVE_SIM_S = 2.5      # Sim-Zeit je Handbefehl
CMD_PERIOD_S = 0.02   # Befehl wiederholen (Wanduhr)
TIMEOUT_S = 180.0     # Wanduhr gesamt
TOL = 0.1
DDS_VS_SIM_TOL = 0.02
MIN_TIP_TRAVEL_CM = 1.0

# (Name, Ziel links, Ziel rechts), normiert, Reihenfolge wie brainco.MOTORS. Die Daumendrehung hat eine eigene Phase:
# Gleichzeitig mit den Fingern stoesst der Daumen am geschlossenen Zeigefinger an (51_build_brainco_sim.py) - wie an
# einer echten Hand. Zusammen pruefen die Phasen jeden der 12 Motoren.
PHASES = (
    ("zu", [0.5] * 6, [0.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
    ("daumen", [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ("auf", [0.0] * 6, [0.0] * 6),
)


class HandClient:
    """Befehle an rt/brainco/*/cmd senden, Zustand von rt/brainco/*/state lesen."""

    def __init__(self):
        self._lock = threading.Lock()
        self._state = {side: None for side in brainco.SIDES}
        self._pub, self._sub = {}, {}
        for side in brainco.SIDES:
            self._pub[side] = ChannelPublisher(brainco.TOPIC_CMD.format(side=side), MotorCmds_)
            self._pub[side].Init()
            self._sub[side] = ChannelSubscriber(brainco.TOPIC_STATE.format(side=side), MotorStates_)
            self._sub[side].Init(partial(self._on_state, side), 1)

    def _on_state(self, side, msg):
        q = np.array([s.q for s in msg.states], dtype=float)
        with self._lock:
            self._state[side] = q

    def send(self, side, q_norm):
        msg = MotorCmds_(cmds=[unitree_go_msg_dds__MotorCmd_() for _ in brainco.MOTORS])
        for cmd, q in zip(msg.cmds, q_norm):
            cmd.q = float(q)
        self._pub[side].Write(msg)

    def read(self, side):
        with self._lock:
            q = self._state[side]
        return None if q is None else q.copy()


def main():
    ap = argparse.ArgumentParser(description="Stufe 0.3d: BrainCo-Haende per DDS in der G1-Sim")
    ap.add_argument("--video", nargs="?", const=str(VIDEO), default=None, help="offscreen ein mp4 rendern")
    ap.add_argument("--out", default=str(OUT), help=f"Ergebnis-JSON (Standard {OUT.relative_to(REPO)})")
    args = ap.parse_args()
    out = pathlib.Path(args.out)

    free_camera_port()
    use_brainco_sim()
    robot = UnitreeG1(UnitreeG1Config(is_simulation=True, controller="GrootLocomotionController"))
    robot.connect()
    sim = robot.sim_env.sim_env
    model, data = sim.mj_model, sim.mj_data
    watch = StallWatch(data)
    hands = HandClient()
    rec = (VideoRecorder(model, data, args.video, track_body="rh_right_base_link",
                         distance=0.9, azimuth=150.0, elevation=-10.0) if args.video else None)

    targets = {side: np.zeros(len(brainco.MOTORS)) for side in brainco.SIDES}
    result = {"fell": False, "phases": {}}
    wall_start = time.perf_counter()

    def run_for(label, sim_s=None, wall_s=None):
        """Befehle wiederholen, filmen, auf Sturz pruefen. False bei Sturz oder Timeout."""
        sim_end = data.time + sim_s if sim_s is not None else None
        wall_end = time.perf_counter() + wall_s if wall_s is not None else None
        last_cmd = 0.0
        while True:
            now = time.perf_counter()
            if (sim_end is not None and data.time >= sim_end) or (wall_end is not None and now >= wall_end):
                return True
            if now - wall_start > TIMEOUT_S:
                result["timeout_during"] = label
                return False
            if now - last_cmd >= CMD_PERIOD_S:
                for side in brainco.SIDES:
                    hands.send(side, targets[side])
                last_cmd = now
            if rec:
                rec.grab()
            if data.qpos[2] < FALL_HEIGHT:
                result.update(fell=True, fell_during=label)
                return False
            if watch.stalled():
                result["sim_stalled_during"] = label
                return False
            time.sleep(0.005)

    def snapshot():
        phase = {"target": {}, "dds_state": {}, "sim_norm": {}, "dds_vs_sim_max": 0.0}
        for side in brainco.SIDES:
            dds = hands.read(side)
            sim_norm = brainco.to_norm(data.qpos[sim.hand[side]["qadr"]])
            phase["target"][side] = targets[side].round(3).tolist()
            phase["sim_norm"][side] = sim_norm.round(3).tolist()
            phase["dds_state"][side] = None if dds is None else dds.round(3).tolist()
            if dds is not None:
                phase["dds_vs_sim_max"] = max(phase["dds_vs_sim_max"], float(np.abs(dds - sim_norm).max()))
        phase["dds_vs_sim_max"] = round(phase["dds_vs_sim_max"], 4)
        return phase

    try:
        ok = run_for("am Gummiband", wall_s=BAND_S)
        sim.elastic_band.enable = False
        ok = ok and run_for("frei stehen", sim_s=STAND_SIM_S)
        tips_open = brainco.tips_in_hand_frame(model, data, "right")

        for name, left, right in PHASES:
            if not ok:
                break
            targets["left"][:] = left
            targets["right"][:] = right
            ok = run_for(f"Phase {name}", sim_s=MOVE_SIM_S)
            result["phases"][name] = snapshot()
            if name == "zu":
                travel = np.linalg.norm(brainco.tips_in_hand_frame(model, data, "right") - tips_open, axis=1)
                result["right_tip_travel_cm"] = (travel * 100).round(2).tolist()
        result["realtime_factor"] = round(float(data.time) / (time.perf_counter() - wall_start), 3)
    finally:
        if rec is not None:
            rec.close()
        robot.disconnect()

    evaluate(result)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2, ensure_ascii=False))
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print(f"gespeichert: {out}")
    print("ABNAHME BESTANDEN" if result["passed"] else "ABNAHME NICHT BESTANDEN")
    return 0 if result["passed"] else 1


def evaluate(result):
    checks = {"kein_sturz": not result["fell"],
              "sim_laeuft_durch": "timeout_during" not in result and "sim_stalled_during" not in result}
    for name, *_ in PHASES:
        phase = result["phases"].get(name)
        states = phase and [phase["dds_state"][s] for s in brainco.SIDES]
        if not phase or any(s is None for s in states):
            checks[f"{name}_ziel_erreicht"] = False
            checks[f"{name}_dds_gleich_sim"] = False
            continue
        err = max(abs(q - t) for s in brainco.SIDES for q, t in zip(phase["dds_state"][s], phase["target"][s]))
        checks[f"{name}_ziel_erreicht"] = err <= TOL
        checks[f"{name}_dds_gleich_sim"] = phase["dds_vs_sim_max"] <= DDS_VS_SIM_TOL
    checks["fingerspitzen_rechts_1cm"] = min(result.get("right_tip_travel_cm") or [0.0]) >= MIN_TIP_TRAVEL_CM
    result["checks"] = checks
    result["passed"] = all(checks.values())


if __name__ == "__main__":
    # DDS- und Sim-Threads halten den Interpreter am Leben (siehe 50_spike_lerobot_walk.py) -> hart beenden
    try:
        code = main()
    except BaseException:
        traceback.print_exc()
        code = 2
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
