"""Wie schnell simuliert DIESER Mac wirklich? Ehrliche Grundlage fuer Trainings-Rechnungen.

Wichtig: gemessen wird die LOCOMOTION-Szene (freie Basis, Bodenkontakt der Fuesse).
Die Tischszene mit fixer Basis ist deutlich billiger und waere kein fairer Vergleich.
"""
import time
import mujoco
import mujoco_menagerie as mm

CTRL_HZ = 50          # Policy-Takt, wie in den Playground-Rezepten


def bench(label, model, seconds=4.0):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for _ in range(200):
        mujoco.mj_step(model, data)

    n, t0 = 0, time.perf_counter()
    while time.perf_counter() - t0 < seconds:
        for _ in range(500):
            mujoco.mj_step(model, data)
        n += 500
    dt = time.perf_counter() - t0

    phys_sps = n / dt
    substeps = int(round((1.0 / CTRL_HZ) / model.opt.timestep))
    env_sps = phys_sps / substeps
    print(f"\n{label}")
    print(f"  nq={model.nq} nv={model.nv} nu={model.nu}  dt={model.opt.timestep*1000:.1f}ms")
    print(f"  Physikschritte/s : {phys_sps:>10,.0f}")
    print(f"  Substeps je Aktion: {substeps} (bei {CTRL_HZ} Hz Regelung)")
    print(f"  ENV-Schritte/s   : {env_sps:>10,.0f}   <- das zaehlt beim RL")
    return env_sps


print("=" * 70)
print("MuJoCo-Geschwindigkeit auf diesem Mac (1 Kern, 1 Roboter, CPU)")
print("=" * 70)

g1_loco = mm.load("unitree_g1")                                    # freie Basis + Boden
env_loco = bench("G1 LOCOMOTION (freie Basis, Bodenkontakt)", g1_loco)

g1_table = mujoco.MjModel.from_xml_path("models/g1_brainco/scene_table.xml")
bench("G1 TISCHSZENE (fixe Basis) - nur zum Vergleich", g1_table)

print("\n" + "=" * 70)
print("Was ein Lauf-Lernlauf kostet")
print("=" * 70)
ENV_STEPS = 150_000_000        # typische Groessenordnung fuer Humanoid-Locomotion (PPO)
CORES = 10                     # M4 Pro: realistisch nutzbare Kerne

sec_1core = ENV_STEPS / env_loco
print(f"Bedarf: ~{ENV_STEPS:,} Env-Schritte (Humanoid-PPO, Groessenordnung)")
print(f"  1 Kern      : {sec_1core/3600:>8,.1f} h   = {sec_1core/86400:>6,.1f} Tage")
print(f"  {CORES} Kerne     : {sec_1core/CORES/3600:>8,.1f} h   = {sec_1core/CORES/86400:>6,.1f} Tage")
print(f"  1 GPU (MJX) : ~0.5-2 h   (4096 Umgebungen parallel)")
print("\nUND: die Zahl oben ist NUR Physik. Policy-Forward, Backprop und der")
print("PPO-Overhead kommen obendrauf - auf CPU realistisch nochmal Faktor 2-4.")
