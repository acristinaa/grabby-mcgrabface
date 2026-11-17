"""G1 lernt stehen. PPO auf CPU, mehrere Kerne parallel.
START:  python scripts/30_train_stand.py [gesamt_schritte]

Fortschritt anschauen:  tail -f runs/stand/progress.csv
"""
import sys, pathlib, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor
from stable_baselines3.common.callbacks import BaseCallback
from envs.g1_stand import make_env

TOTAL   = int(sys.argv[1]) if len(sys.argv) > 1 else 5_000_000
N_ENVS  = 10
RUN     = pathlib.Path("runs/stand"); RUN.mkdir(parents=True, exist_ok=True)


class Curve(BaseCallback):
    """Schreibt die Lernkurve mit - das ist das, was du sehen willst."""
    def __init__(self):
        super().__init__()
        self.rows = []
        self.t0 = time.time()

    def _on_step(self): return True

    def _on_rollout_end(self):
        buf = self.model.ep_info_buffer
        if not buf: return
        rew = float(np.mean([e["r"] for e in buf]))
        ln  = float(np.mean([e["l"] for e in buf]))
        el  = time.time() - self.t0
        self.rows.append((self.num_timesteps, rew, ln, el))
        pct = 100.0 * ln / 400.0
        print(f"{self.num_timesteps:>9,} Schritte | Belohnung {rew:>8.1f} | "
              f"Episode {ln:>5.1f}/400 ({pct:>5.1f}% durchgehalten) | "
              f"{self.num_timesteps/max(el,1e-9):>7,.0f} Schritte/s", flush=True)
        np.savetxt(RUN / "progress.csv", np.array(self.rows),
                   delimiter=",", header="steps,reward,ep_len,elapsed_s", comments="")


if __name__ == "__main__":
    venv = SubprocVecEnv([make_env(seed=i) for i in range(N_ENVS)])
    venv = VecMonitor(venv, str(RUN / "monitor"))

    model = PPO(
        "MlpPolicy", venv,
        learning_rate=3e-4, n_steps=512, batch_size=1024, n_epochs=5,
        gamma=0.99, gae_lambda=0.95, clip_range=0.2, ent_coef=0.002,
        policy_kwargs=dict(net_arch=[256, 128], log_std_init=-1.6),
        device="cpu", verbose=0,
    )
    print(f"Trainiere {TOTAL:,} Schritte auf {N_ENVS} Prozessen...")
    print("Ziel: Episodenlaenge 400/400 = faellt nicht mehr um.\n")
    model.learn(total_timesteps=TOTAL, callback=Curve())
    model.save(RUN / "policy")
    print(f"\nGespeichert: {RUN/'policy'}.zip")
