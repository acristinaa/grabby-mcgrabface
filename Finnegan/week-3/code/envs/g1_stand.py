"""G1 lernt stehen zu bleiben - eine Gymnasium-Umgebung.

Aufgabe: aus einer zufaellig gestoerten Lage aufrecht bleiben, auch wenn geschubst wird.
Bewusst KLEIN gehalten, damit es auf einem Mac in ~30 min lernt.

Beobachtung (nur was ein echter Roboter auch messen kann - wichtig fuer Sim2Real!):
    projizierte Schwerkraft (3)   <- aus der IMU
    Winkelgeschwindigkeit  (3)    <- aus der IMU
    Gelenkwinkel          (29)    <- Encoder
    Gelenkgeschwindigkeit (29)    <- Encoder
    letzte Aktion         (29)
    = 93 Zahlen

Aktion: 29 Zielwinkel als Abweichung von der Standpose, in [-1, 1] skaliert.
"""
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import mujoco
import mujoco_menagerie as mm

CTRL_HZ      = 50
EPISODE_SEC  = 8.0
ACTION_SCALE = 0.22      # rad Auslenkung um die Standpose (Lauf 1: 0.35 = zu wild)
FALL_HEIGHT  = 0.45      # darunter = umgefallen
FALL_TILT    = -0.55     # projizierte Schwerkraft z darueber = zu schief


class G1StandEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, push=True, seed=None):
        self.model = mm.load("unitree_g1")
        self.data = mujoco.MjData(self.model)
        self.substeps = int(round((1.0 / CTRL_HZ) / self.model.opt.timestep))
        self.max_steps = int(EPISODE_SEC * CTRL_HZ)
        self.push = push
        self.njnt = self.model.nu                      # 29 angesteuerte Gelenke

        self.default_qpos = self.model.key_qpos[0].copy()
        self.default_joints = self.default_qpos[7:].copy()

        obs_dim = 3 + 3 + self.njnt * 3
        self.observation_space = spaces.Box(-np.inf, np.inf, (obs_dim,), np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, (self.njnt,), np.float32)
        self._rng = np.random.default_rng(seed)
        self._last_action = np.zeros(self.njnt, np.float32)

    # ---------- Hilfen ----------
    def _projected_gravity(self):
        """Schwerkraftrichtung im Koerperframe. [0,0,-1] = perfekt aufrecht."""
        quat = self.data.qpos[3:7]
        mat = np.zeros(9)
        mujoco.mju_quat2Mat(mat, quat)
        return mat.reshape(3, 3).T @ np.array([0.0, 0.0, -1.0])

    def _obs(self):
        return np.concatenate([
            self._projected_gravity(),
            self.data.qvel[3:6],                 # Winkelgeschw. im Koerperframe
            self.data.qpos[7:] - self.default_joints,
            self.data.qvel[6:] * 0.05,           # skaliert, sonst dominiert es
            self._last_action,
        ]).astype(np.float32)

    # ---------- Gym-API ----------
    def reset(self, seed=None, options=None):
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self.default_qpos
        # zufaellige Stoerung: Startpose + Anfangsgeschwindigkeit
        self.data.qpos[7:] += self._rng.uniform(-0.12, 0.12, self.njnt)
        self.data.qpos[2] += self._rng.uniform(-0.02, 0.02)
        self.data.qvel[:6] = self._rng.uniform(-0.35, 0.35, 6)
        self._last_action[:] = 0.0
        self._step = 0
        mujoco.mj_forward(self.model, self.data)
        return self._obs(), {}

    def step(self, action):
        action = np.clip(action, -1.0, 1.0).astype(np.float32)
        target = self.default_joints + action * ACTION_SCALE

        # gelegentlicher Schubs - erzwingt echtes Balancieren statt Stillstehen
        if self.push and self._step > 0 and self._step % 60 == 0:
            self.data.qvel[:3] += self._rng.uniform(-0.55, 0.55, 3)

        for _ in range(self.substeps):
            self.data.ctrl[:] = target
            mujoco.mj_step(self.model, self.data)

        self._step += 1
        pg = self._projected_gravity()
        height = float(self.data.qpos[2])

        fallen = (height < FALL_HEIGHT) or (pg[2] > FALL_TILT)
        truncated = self._step >= self.max_steps

        # ----- Belohnung -----
        r_alive   = 1.0
        r_upright = 1.5 * float(np.exp(-4.0 * abs(pg[2] + 1.0)))
        r_height  = 1.0 * float(np.exp(-20.0 * abs(height - self.default_qpos[2])))
        r_posture = 0.5 * float(np.exp(-2.0 * np.mean(np.abs(self.data.qpos[7:] - self.default_joints))))
        p_effort  = 0.02 * float(np.mean(np.square(action)))
        p_rate    = 0.05 * float(np.mean(np.square(action - self._last_action)))
        p_vel     = 0.001 * float(np.mean(np.square(self.data.qvel[6:])))
        reward = r_alive + r_upright + r_height + r_posture - p_effort - p_rate - p_vel
        if fallen:
            reward -= 10.0

        self._last_action = action
        info = {"height": height, "upright": float(-pg[2]), "steps": self._step}
        return self._obs(), float(reward), bool(fallen), bool(truncated), info


def make_env(seed=0, push=True):
    def _f():
        env = G1StandEnv(push=push, seed=seed)
        return env
    return _f
