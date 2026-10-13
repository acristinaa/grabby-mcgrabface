"""BrainCo Revo2 in der LeRobot-G1-Sim: Umgebung und DDS-Bruecke (Stufe 0.3b / 0.3d).

Der Python-Code der LeRobot-Sim bleibt unveraendert (Hugging Face, ohne Lizenzangabe). install() tauscht beim
Laden zwei Klassen in ihrem Modul sim.base_sim aus, bevor BaseSimulator sie erzeugt:

  DefaultEnv        + BraincoEnvMixin
  UnitreeSdk2Bridge + BraincoBridgeMixin

Warum die Sim angepasst werden muss:
  1. Sie ordnet Motoren ueber "Gelenk-Index - 1" zu. Das stimmt nur, solange jedes Gelenk einen eigenen Motor hat
     (Dex3). Die Revo2 hat je Hand 5 mitgekoppelte Gelenke ohne Motor, danach ist alles verschoben: Die Drehmomente
     fuer den rechten Arm (Gelenke 34-40, Motoren 22-28) landeten auf den Motoren 33-39, also auf den Haenden
     (geprueft 2026-09-14). Hier wird jede Zuordnung aus dem Modell gelesen.
  2. Die echten BrainCo-Haende nutzen andere DDS-Topics und Nachrichten als Dex3 (g1sim/brainco.py). Die Sim
     spricht dieselben, damit der Code fuer Sim und Roboter gleich bleibt.

Nutzung, vor UnitreeG1(...).connect():
    from g1sim.brainco_env import use_brainco_sim
    use_brainco_sim()
"""
import importlib.util
import pathlib
import sys
import threading
from functools import partial

import gymnasium as gym
import mujoco
import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_

from g1sim import HF_REPO, MANIFEST, SIM_DIR, brainco


class _NullPublisher:
    """Ersetzt die Dex3-Publisher der Sim - diese Sim hat keine Dex3-Haende."""

    def Write(self, *args, **kwargs):
        return True


class BraincoBridgeMixin:
    """DDS-Bruecke: zusaetzlich rt/brainco/{left,right}/{cmd,state} wie an den echten Haenden."""

    def __init__(self, config):
        super().__init__(config)
        self.left_hand_state_puber = _NullPublisher()
        self.right_hand_state_puber = _NullPublisher()

        self._brainco_lock = threading.Lock()
        self._brainco_target = {side: None for side in brainco.SIDES}
        self._brainco_state = {}
        self._brainco_state_pub = {}
        self._brainco_cmd_sub = {}
        for side in brainco.SIDES:
            pub = ChannelPublisher(brainco.TOPIC_STATE.format(side=side), MotorStates_)
            pub.Init()
            self._brainco_state_pub[side] = pub
            self._brainco_state[side] = MotorStates_(
                states=[unitree_go_msg_dds__MotorState_() for _ in brainco.MOTORS]
            )
            sub = ChannelSubscriber(brainco.TOPIC_CMD.format(side=side), MotorCmds_)
            sub.Init(partial(self._on_brainco_cmd, side), 1)
            self._brainco_cmd_sub[side] = sub

    def _on_brainco_cmd(self, side, msg):
        n = len(brainco.MOTORS)
        if len(msg.cmds) < n:
            return  # unvollstaendiger Befehl wird verworfen
        q = np.array([c.q for c in msg.cmds[:n]], dtype=float)
        with self._brainco_lock:
            self._brainco_target[side] = q

    def brainco_target(self, side):
        """Letztes normiertes Ziel (6 Werte, 0 = offen) oder None, solange kein Befehl kam."""
        with self._brainco_lock:
            q = self._brainco_target[side]
        return None if q is None else q.copy()

    def publish_brainco_state(self, side, q_norm, dq_norm):
        msg = self._brainco_state[side]
        for state, q, dq in zip(msg.states, q_norm, dq_norm):
            state.q = float(q)
            state.dq = float(dq)
        self._brainco_state_pub[side].Write(msg)


class BraincoEnvMixin:
    """Umgebung: Motor-Zuordnung aus dem Modell, Revo2 per PD-Regler, BrainCo-Zustand ueber DDS."""

    def init_scene(self):
        super().init_scene()
        # UnitreeG1.connect() startet den Sim-Thread und ruft danach im Haupt-Thread reset() auf. Ohne Sperre liest ein
        # Schritt die Daten zwischen mj_resetData und mj_forward: Quaternionen null, das Gummiband wirft, der Sim-Thread
        # stirbt still und die Sim-Zeit bleibt bei 0 (beobachtet 2026-09-14, zufallsabhaengig).
        self._step_lock = threading.RLock()
        m = self.mj_model
        act_of_joint = {int(j): a for a, j in enumerate(m.actuator_trnid[:, 0])}

        # body_joint_index: Gelenk-IDs in G1-Motorreihenfolge 0..28, von der Original-Sim per Namensregel ermittelt
        body = self.body_joint_index
        self.body_act = np.array([act_of_joint[int(j)] for j in body])
        self.body_qadr = m.jnt_qposadr[body]
        self.body_vadr = m.jnt_dofadr[body]

        self.hand = {}
        for side in brainco.SIDES:
            joints = [m.joint(name).id for name in brainco.joint_names(side)]
            self.hand[side] = {
                "act": np.array([act_of_joint[j] for j in joints]),
                "qadr": m.jnt_qposadr[joints],
                "vadr": m.jnt_dofadr[joints],
            }
        n_mapped = len(self.body_act) + sum(len(h["act"]) for h in self.hand.values())
        if n_mapped != m.nu:
            raise RuntimeError(f"Modell hat {m.nu} Aktuatoren, zugeordnet {n_mapped} - kein BrainCo-Modell?")

        self.torques = np.zeros(m.nu)
        self.torque_limit = m.actuator_ctrlrange[:, 1].copy()  # alles <motor>: ctrlrange = Drehmomentgrenze
        self.hand_target = {side: np.zeros(len(brainco.MOTORS)) for side in brainco.SIDES}  # Start offen

    def compute_body_torques(self):
        """PD aus rt/lowcmd wie in der Original-Sim, aber mit Gelenkadressen aus dem Modell."""
        tau = np.zeros(len(self.body_act))
        bridge = self.unitree_bridge
        if bridge is None or not bridge.low_cmd:
            return tau
        if bridge.use_sensor:
            raise NotImplementedError("USE_SENSOR wird von der BrainCo-Sim nicht unterstuetzt")
        cmd = bridge.low_cmd.motor_cmd
        q = self.mj_data.qpos[self.body_qadr]
        dq = self.mj_data.qvel[self.body_vadr]
        for i in range(len(tau)):
            c = cmd[i]
            tau[i] = c.tau + c.kp * (c.q - q[i]) + c.kd * (c.dq - dq[i])
        return tau

    def compute_hand_torques(self):
        """PD je Handmotor auf das letzte BrainCo-Ziel. Rueckgabe {side: 6 Drehmomente}."""
        tau = {}
        for side, h in self.hand.items():
            target = self.unitree_bridge.brainco_target(side) if self.unitree_bridge is not None else None
            if target is not None:
                self.hand_target[side] = target
            q = self.mj_data.qpos[h["qadr"]]
            dq = self.mj_data.qvel[h["vadr"]]
            tau[side] = brainco.HAND_KP * (brainco.to_rad(self.hand_target[side]) - q) - brainco.HAND_KD * dq
        return tau

    def prepare_obs(self):
        d = self.mj_data
        obs = {
            "floating_base_pose": d.qpos[:7],
            "floating_base_vel": d.qvel[:6],
            "floating_base_acc": d.qacc[:6],
            "secondary_imu_quat": d.xquat[self.torso_index],
            "secondary_imu_vel": d.cvel[self.torso_index],
            "body_q": d.qpos[self.body_qadr],
            "body_dq": d.qvel[self.body_vadr],
            "body_ddq": d.qacc[self.body_vadr],
            "body_tau_est": d.actuator_force[self.body_act],
            "time": d.time,
        }
        for side, h in self.hand.items():
            obs[f"{side}_hand_q"] = d.qpos[h["qadr"]]
            obs[f"{side}_hand_dq"] = d.qvel[h["vadr"]]
            obs[f"{side}_hand_tau_est"] = d.actuator_force[h["act"]]
        return obs

    def reset(self):
        with self._step_lock:
            super().reset()

    def sim_step(self):
        if self.config["FREE_BASE"]:
            raise NotImplementedError("FREE_BASE wird von der BrainCo-Sim nicht unterstuetzt")
        with self._step_lock:
            bridge = self.unitree_bridge
            self.obs = self.prepare_obs()
            bridge.PublishLowState(self.obs)
            for side in brainco.SIDES:
                bridge.publish_brainco_state(
                    side,
                    brainco.to_norm(self.obs[f"{side}_hand_q"]),
                    self.obs[f"{side}_hand_dq"] / brainco.SIM_CLOSED_RAD,
                )
            if bridge.joystick:
                bridge.PublishWirelessController()
            if self.config["ENABLE_ELASTIC_BAND"]:
                self._apply_elastic_band()

            ctrl = np.zeros(self.mj_model.nu)
            ctrl[self.body_act] = self.compute_body_torques()
            for side, tau in self.compute_hand_torques().items():
                ctrl[self.hand[side]["act"]] = tau
            self.torques = np.clip(ctrl, -self.torque_limit, self.torque_limit)
            self.mj_data.ctrl[:] = self.torques
            mujoco.mj_step(self.mj_model, self.mj_data)

    def _apply_elastic_band(self):
        """Gummiband-Kraft wie in der Original-Sim."""
        link = self.band_attached_link
        if not self.elastic_band.enable:
            self.mj_data.xfrc_applied[link] = 0.0
            return
        vel = np.zeros(6)  # Weltkoordinaten, [Winkel, linear]
        mujoco.mj_objectVelocity(self.mj_model, self.mj_data, mujoco.mjtObj.mjOBJ_BODY, link, vel, 0)
        pose = np.concatenate([self.mj_data.xpos[link], self.mj_data.xquat[link], vel[3:], vel[:3]])
        self.mj_data.xfrc_applied[link] = self.elastic_band.Advance(pose)


def install(base_sim):
    """BrainCo-Klassen in das geladene Modul sim.base_sim einsetzen. Vor BaseSimulator(...) aufrufen."""
    if getattr(base_sim, "BRAINCO_INSTALLED", False):
        return
    base_sim.DefaultEnv = type("BraincoEnv", (BraincoEnvMixin, base_sim.DefaultEnv), {})
    base_sim.UnitreeSdk2Bridge = type("BraincoBridge", (BraincoBridgeMixin, base_sim.UnitreeSdk2Bridge), {})
    base_sim.BRAINCO_INSTALLED = True


def use_brainco_sim(sim_dir=SIM_DIR, scene=None, cameras=True):
    """LeRobots UnitreeG1 auf die BrainCo-Sim umlenken. Vor robot.connect() aufrufen.

    scene: andere Szene unter sim_dir statt ROBOT_SCENE aus config.yaml, z. B. rooms.SCENE_FILE (Lernpfad L2).
    cameras=False: die Sim rendert und verschickt keine Kamerabilder (kein Renderer im Sim-Thread, kein ZMQ-Port 5555).
      Fuer reine Lauf-Skripte: Rendern zwei Threads gleichzeitig mit OpenGL, kann der Prozess auf diesem Mac unkillbar im
      Grafiktreiber haengen bleiben (beobachtet 2026-10-02 in scripts/62_rooms_check.py).

    UnitreeG1.connect() importiert make_env erst beim Aufruf aus lerobot.envs und laedt fest
    "lerobot/unitree-g1-mujoco". Ersetzt wird deshalb nur lerobot.envs.make_env.
    """
    sim_dir = pathlib.Path(sim_dir)
    if not (sim_dir / MANIFEST).is_file():
        raise FileNotFoundError(f"{sim_dir} ist nicht gebaut - zuerst scripts/51_build_brainco_sim.py ausfuehren")

    import lerobot.envs

    def make_env(repo_id, n_envs=1, use_async_envs=False, **kwargs):
        if repo_id != HF_REPO:
            raise ValueError(f"Die BrainCo-Sim ersetzt nur {HF_REPO}, nicht {repo_id}")
        spec = importlib.util.spec_from_file_location("g1_brainco_sim_env", sim_dir / "env.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)  # importiert dabei sim.base_sim aus sim_dir
        base_sim = sys.modules["sim.base_sim"]
        if pathlib.Path(base_sim.__file__).resolve().parent.parent != sim_dir.resolve():
            raise RuntimeError(f"sim.base_sim kommt aus {base_sim.__file__}, erwartet {sim_dir}")
        install(base_sim)
        if scene is not None:
            create = module.SimulatorFactory.create_simulator

            def create_with_scene(config, **kw):
                return create(config={**config, "ROBOT_SCENE": scene}, **kw)

            module.SimulatorFactory.create_simulator = staticmethod(create_with_scene)
        env = module.make_env(n_envs=n_envs, use_async_envs=use_async_envs, publish_images=cameras)
        return {"hub_env": {0: gym.vector.SyncVectorEnv([lambda: env])}}

    lerobot.envs.make_env = make_env
