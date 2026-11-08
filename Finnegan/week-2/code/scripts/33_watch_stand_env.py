"""Die TRAININGS-Umgebung live anschauen (die, in der gerade gelernt wird).
START:  mjpython scripts/33_watch_stand_env.py

Zeigt den G1 auf dem Boden mit denselben Stoerungen wie im Training.
Wenn schon eine Politik trainiert wurde, wird sie benutzt - sonst 'nichts tun'.
"""
import sys, pathlib, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import numpy as np, mujoco, mujoco.viewer
from envs.g1_stand import G1StandEnv

pol = None
p = pathlib.Path("runs/stand/policy.zip")
if p.exists():
    from stable_baselines3 import PPO
    pol = PPO.load(p, device="cpu")
    print(">>> benutze GELERNTE Politik")
else:
    print(">>> noch keine Politik trainiert -> 'nichts tun' (Roboter faellt um, das ist korrekt)")

env = G1StandEnv(seed=0)
obs, _ = env.reset(seed=3)
print("Fenster oeffnet sich. Leertaste = Pause. Strg+Ziehen = schubsen.")
print("Faellt er um, startet die Episode automatisch neu.\n")

with mujoco.viewer.launch_passive(env.model, env.data) as v:
    ep, steps = 0, 0
    while v.is_running():
        t0 = time.time()
        a = np.zeros(env.njnt, np.float32) if pol is None else pol.predict(obs, deterministic=True)[0]
        obs, _, term, trunc, info = env.step(a)
        steps += 1
        v.sync()
        if term or trunc:
            ep += 1
            print(f"Episode {ep}: {steps} Schritte -> {'GEFALLEN' if term else 'durchgehalten'}")
            obs, _ = env.reset(seed=3 + ep); steps = 0
        rest = (1.0 / 50) - (time.time() - t0)
        if rest > 0:
            time.sleep(rest)
