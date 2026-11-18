"""LeRobot-G1-Sim mit BrainCo-Revo2-Haenden (Stufe 0.3).

Laeuft in der Umgebung ~/dev/g1-ext/.venv (LeRobot, unitree_sdk2py, CycloneDDS), nicht in der Projekt-venv.

  build.py        baut die Sim unter SIM_DIR (Modell + config.yaml)
  brainco.py      Handmotoren, Normierung, DDS-Topics
  brainco_env.py  Umgebung + DDS-Bruecke fuer die Revo2, Umlenkung von LeRobots make_env
"""
import pathlib

HF_REPO = "lerobot/unitree-g1-mujoco"
HF_REVISION = "a38dc8617f0fca51b38e9354dc58ee35ad850fb5"  # Stand, gegen den 0.2 und 0.3 geprueft wurden

SIM_DIR = pathlib.Path.home() / "dev" / "g1-ext" / "g1_brainco_sim"
SCENE_FILE = "assets/scene_29dof_brainco.xml"
MANIFEST = "BUILD.json"  # nur ein Ordner mit dieser Datei darf beim Neubau geloescht werden
