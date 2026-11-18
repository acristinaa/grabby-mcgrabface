"""Sonde fuer tests/test_integration_sim_env.py: BrainCo-Sim-Umgebung direkt, ohne LeRobot-Roboter und Controller.

Misst drei Dinge und gibt sie als Zeile "RESULT {json}" aus:
  1. Motor-Zuordnung: Nur die rechte Schulter (G1-Motor 22) bekommt einen PD-Befehl - welche Motoren ziehen?
  2. Rechte Hand: BrainCo-Befehl -> Gelenke -> Zustand ueber DDS (rt/brainco/right/state)
  3. Wettlauf: reset() aus einem zweiten Thread, waehrend Schritte laufen (so ruft es UnitreeG1.connect auf)

--ohne-sperre schaltet die Sperre zwischen Schritt und reset() ab - nur um zu zeigen, dass Punkt 3 den Fehler findet.
"""
import contextlib
import json
import os
import pathlib
import sys
import threading
import time
import traceback
from types import SimpleNamespace

import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber

ChannelFactoryInitialize(0, "lo0")  # macOS: lo0 statt lo, vor jedem LeRobot-Import

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

import lerobot.envs  # noqa: E402
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_  # noqa: E402
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_  # noqa: E402

from g1sim import brainco  # noqa: E402
from g1sim.brainco_env import use_brainco_sim  # noqa: E402
from g1sim.runtime import free_camera_port  # noqa: E402

RIGHT_SHOULDER_PITCH = 22  # G1-Motorindex
HAND_TARGET = [0.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # alles ausser Daumendrehung - die blockiert am geschlossenen Zeigefinger
HAND_STEPS = 750  # 3 s Sim-Zeit


def main():
    free_camera_port()
    use_brainco_sim()
    env = lerobot.envs.make_env("lerobot/unitree-g1-mujoco", trust_remote_code=True)["hub_env"][0].envs[0]
    sim = env.sim_env
    model, data, bridge = sim.mj_model, sim.mj_data, sim.unitree_bridge
    if "--ohne-sperre" in sys.argv:
        sim._step_lock = contextlib.nullcontext()
    out = {}
    try:
        # 1. Motor-Zuordnung
        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.motor_cmd[RIGHT_SHOULDER_PITCH].q = 0.5
        cmd.motor_cmd[RIGHT_SHOULDER_PITCH].kp = 20.0
        bridge.low_cmd = cmd
        sim.sim_step()
        nonzero = np.flatnonzero(np.abs(data.ctrl) > 1e-9)
        out["pulling_actuators"] = [model.actuator(int(i)).name for i in nonzero]
        bridge.low_cmd = unitree_hg_msg_dds__LowCmd_()

        # 2. Rechte Hand
        states = {}
        sub = ChannelSubscriber(brainco.TOPIC_STATE.format(side="right"), MotorStates_)
        sub.Init(lambda msg: states.__setitem__("right", [s.q for s in msg.states]), 1)
        bridge._on_brainco_cmd("right", SimpleNamespace(cmds=[SimpleNamespace(q=q) for q in HAND_TARGET]))
        for _ in range(HAND_STEPS):
            sim.sim_step()
        time.sleep(0.3)  # letzte Zustandsnachricht zustellen lassen
        out["hand_target"] = HAND_TARGET
        out["hand_sim_norm"] = brainco.to_norm(data.qpos[sim.hand["right"]["qadr"]]).round(3).tolist()
        out["hand_dds_state"] = [round(q, 3) for q in states.get("right", [])]

        # 3. Wettlauf Schritt <-> reset()
        errors, stop = [], threading.Event()

        def stepper():
            try:
                while not stop.is_set():
                    sim.sim_step()
            except Exception as exc:  # noqa: BLE001 - genau diese Ausnahme wird gemessen
                errors.append(repr(exc))

        thread = threading.Thread(target=stepper)
        thread.start()
        for _ in range(200):
            sim.reset()
            time.sleep(0.005)
            if errors:
                break
        stop.set()
        thread.join(timeout=5)
        out["race_errors"] = errors
    finally:
        env.close()
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
