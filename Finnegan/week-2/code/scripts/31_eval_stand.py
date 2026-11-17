"""Wie gut ist die gelernte Politik wirklich? Vergleich gegen 'nichts tun'.
START:  python scripts/31_eval_stand.py
"""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import numpy as np
from stable_baselines3 import PPO
from envs.g1_stand import G1StandEnv

N_EPISODES = 20
MAX_STEPS = 400


def run(policy, label):
    env = G1StandEnv(seed=1234)
    lens, rews, falls = [], [], 0
    for ep in range(N_EPISODES):
        obs, _ = env.reset(seed=10_000 + ep)
        total, n = 0.0, 0
        while True:
            a = (np.zeros(env.njnt, np.float32) if policy is None
                 else policy.predict(obs, deterministic=True)[0])
            obs, r, term, trunc, _ = env.step(a)
            total += r; n += 1
            if term or trunc:
                falls += int(term); break
        lens.append(n); rews.append(total)
    surv = 100.0 * (N_EPISODES - falls) / N_EPISODES
    print(f"{label:<22} Episode {np.mean(lens):>6.1f}/{MAX_STEPS}  "
          f"Belohnung {np.mean(rews):>8.1f}  ueberlebt {surv:>5.1f}% "
          f"({N_EPISODES-falls}/{N_EPISODES})")
    return np.mean(lens), surv


print(f"Je {N_EPISODES} Episoden, gleiche Startbedingungen fuer beide.\n")
base_len, base_surv = run(None, "nichts tun")

p = pathlib.Path("runs/stand/policy.zip")
if p.exists():
    model = PPO.load(p, device="cpu")
    learn_len, learn_surv = run(model, "gelernte Politik")
    print(f"\nVerbesserung: Episodenlaenge {base_len:.0f} -> {learn_len:.0f} "
          f"({learn_len/max(base_len,1):.1f}x) | ueberlebt {base_surv:.0f}% -> {learn_surv:.0f}%")
else:
    print(f"\n(noch keine trainierte Politik unter {p})")
