"""Video der gelernten Politik - nebeneinander mit 'nichts tun'.
START:  python scripts/32_record_stand.py
"""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import numpy as np, mujoco, imageio.v2 as imageio
from stable_baselines3 import PPO
from envs.g1_stand import G1StandEnv

OUT = pathlib.Path("docs/videos"); OUT.mkdir(parents=True, exist_ok=True)
FPS = 30


def rollout_frames(policy, seed, H=480, W=640):
    env = G1StandEnv(seed=seed)
    obs, _ = env.reset(seed=seed)
    r = mujoco.Renderer(env.model, height=H, width=W)
    frames = []
    for _ in range(400):
        a = (np.zeros(env.njnt, np.float32) if policy is None
             else policy.predict(obs, deterministic=True)[0])
        obs, _, term, trunc, _ = env.step(a)
        r.update_scene(env.data)
        frames.append(r.render())
        if term or trunc:
            break
    # letztes Bild einfrieren, damit beide Videos gleich lang sind
    while len(frames) < 400:
        frames.append(frames[-1])
    return frames


p = pathlib.Path("runs/stand/policy.zip")
model = PPO.load(p, device="cpu") if p.exists() else None
SEED = 10_003

print("rendere 'nichts tun' ...")
a_frames = rollout_frames(None, SEED)
print("rendere gelernte Politik ...")
b_frames = rollout_frames(model, SEED)

side = [np.concatenate([a, b], axis=1) for a, b in zip(a_frames, b_frames)]
out = OUT / "stand_vergleich.mp4"
imageio.mimsave(out, side, fps=FPS, quality=8)
print(f"gespeichert: {out}   (links: nichts tun | rechts: gelernt)")
