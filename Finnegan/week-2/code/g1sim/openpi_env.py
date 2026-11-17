"""openpi-Client-Seite der Greifszene (Stufe 2): TableEnv als openpi-Environment und Episoden-Speicher.

Muster: openpi examples/aloha_sim (env.py, saver.py). Laeuft in der Sim-Umgebung (~/dev/g1-ext/.venv) mit openpi-client.
Die Schleife selbst ist openpis Runtime: Beobachtung -> Policy (Websocket, Aktions-Chunks) -> apply_action.
Beobachtung im Eingangsformat von openpi_g1/policy.py (episode.policy_observation), Aktion 26-D absolut.
"""
import pathlib

import numpy as np
from openpi_client.runtime import environment as _environment
from openpi_client.runtime import subscriber as _subscriber

from g1sim import action_space, episode, table_env

DEFAULT_TASK = "pick up the red bottle"


class TableEnvironment(_environment.Environment):
    """Eine Episode = max_steps Regeltakte (1/30 s) ab reset()."""

    def __init__(self, env, task=DEFAULT_TASK, max_steps=300, randomize=False):
        self.env, self.task, self.max_steps, self.randomize = env, task, int(max_steps), randomize
        self.steps = 0
        self._obs = None

    def reset(self):
        self._obs = self.env.reset(randomize=self.randomize)
        self.steps = 0

    def is_episode_complete(self):
        return self.steps >= self.max_steps

    def get_observation(self):
        if self._obs is None:
            raise RuntimeError("zuerst reset()")
        return episode.policy_observation(self._obs, self.task)

    def apply_action(self, action):
        actions = np.asarray(action["actions"], dtype=np.float32)
        if actions.shape != (action_space.DIM,) or not np.isfinite(actions).all():
            raise ValueError(f"Aktion: erwartet {action_space.DIM} endliche Werte, bekommen {actions.shape}")
        self.env.apply(actions)
        self.steps += 1
        self._obs = self.env.observation()


class EpisodeSaver(_subscriber.Subscriber):
    """Schreibt jede Episode als Rohaufnahme out_dir/episode_NNN.npz: Beobachtung vor der Aktion + Aktion."""

    def __init__(self, out_dir, task=DEFAULT_TASK, fps=table_env.CONTROL_HZ, success=lambda: False):
        self.out_dir, self.task, self.fps, self.success = pathlib.Path(out_dir), task, fps, success
        self.recorder = None
        self.saved = []

    def on_episode_start(self):
        self.recorder = episode.EpisodeRecorder(self.task, self.fps)

    def on_step(self, observation, action):
        frame = {key: observation[policy_key] for key, policy_key in episode.POLICY_KEYS.items()}
        self.recorder.add(frame, action["actions"])

    def on_episode_end(self):
        path = self.out_dir / f"episode_{len(self.saved):03d}.npz"
        self.saved.append(self.recorder.save(path, success=bool(self.success())))
