"""Sonde fuer tests/test_e2e_sim.py: UnitreeG1 mit BrainCo-Sim und GR00T-Lauf-Controller verbinden, trennen.

Prueft den Weg, auf dem der Wettlauf Schritt <-> reset() aufgetreten ist: Laeuft die Sim-Zeit nach connect() weiter?
Ausgabe: Zeile "RESULT {json}".
"""
import json
import os
import pathlib
import sys
import time
import traceback

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

ChannelFactoryInitialize(0, "lo0")  # macOS: lo0 statt lo, vor jedem LeRobot-Import

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config  # noqa: E402

from g1sim.brainco_env import use_brainco_sim  # noqa: E402
from g1sim.runtime import free_camera_port  # noqa: E402

SIM_SECONDS = 1.0
WALL_LIMIT_S = 30.0


def main():
    free_camera_port()
    use_brainco_sim()
    robot = UnitreeG1(UnitreeG1Config(is_simulation=True, controller="GrootLocomotionController"))
    t0 = time.perf_counter()
    robot.connect()
    out = {"connect_s": round(time.perf_counter() - t0, 2)}
    try:
        data = robot.sim_env.sim_env.mj_data
        start, wall = float(data.time), time.perf_counter()
        while data.time - start < SIM_SECONDS and time.perf_counter() - wall < WALL_LIMIT_S:
            time.sleep(0.05)
        out["sim_advanced_s"] = round(float(data.time) - start, 3)
        out["pelvis_z"] = round(float(data.qpos[2]), 3)
    finally:
        robot.disconnect()
    print("RESULT " + json.dumps(out), flush=True)


if __name__ == "__main__":
    try:
        main()
        code = 0
    except BaseException:
        traceback.print_exc()
        code = 2
    sys.stdout.flush()
    os._exit(code)
