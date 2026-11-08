"""Stufe 0.3b - Bau-Rezept: LeRobot-G1-Sim mit BrainCo-Revo2-Haenden.

LeRobot liefert die G1-Sim (lerobot/unitree-g1-mujoco) mit Unitree-Dex3-Haenden aus. Gebaut wird eine Kopie unter
SIM_DIR (~/dev/g1-ext/g1_brainco_sim):

  - Modell: LeRobots scene_29dof.xml (g1_29dof_no_hand.xml) + Revo2 an beiden Handgelenken (dashboard/scene.py)
    + Anbauteile des echten G1: AGX-Rucksack und schwenkbares D455-Kopfmodul (g1sim/payload.py)
  - Hand-Aktuatoren als <motor>: Die Sim rechnet fuer jeden Motor selbst das Drehmoment und schreibt ctrl komplett.
    Die Positionsregler der Herstellermodelle (<position kp=30 kv=10>) wuerden dabei ueberschrieben.
    Drehmomentgrenze = actuatorfrcrange der Herstellermodelle.
  - config.yaml: neues Modell, keine Dex3-Haende, kein MuJoCo-Fenster

Warum ausserhalb des Repos: Die LeRobot-Sim hat auf Hugging Face keine Lizenzangabe, der BrainCo-Download auch nicht.
Im Repo liegt nur das Rezept. Den Python-Code der Kopie aendert niemand; die BrainCo-Anpassungen kommen zur Laufzeit
aus g1sim/brainco_env.py.
"""
import datetime
import filecmp
import json
import pathlib
import shutil
import subprocess

import mujoco
import yaml
from huggingface_hub import snapshot_download

from dashboard import scene
from g1sim import HF_REPO, HF_REVISION, MANIFEST, SCENE_FILE, SIM_DIR, payload

REPO = pathlib.Path(__file__).resolve().parent.parent
HAND_PREFIXES = tuple(scene.PREFIX.values())


def snapshot():
    """Lokaler Pfad der LeRobot-Sim in der geprueften Version (laedt nur, wenn sie nicht im Cache liegt)."""
    return pathlib.Path(snapshot_download(HF_REPO, revision=HF_REVISION))


def _hand_to_motors(side, hand):
    for act in hand.actuators:
        lo, hi = hand.joint(act.target).actfrcrange
        act.set_to_motor()
        act.ctrlrange = [lo, hi]


def build_spec(snap):
    """MjSpec: LeRobot-G1 ohne Haende + Revo2 als Drehmoment-Motoren + Rucksack und Kopfmodul. Noch nicht kompiliert."""
    spec = mujoco.MjSpec.from_file(str(snap / "assets" / "scene_29dof.xml"))
    # Ein Mesh-Ordner fuer beide: Die Revo2-Modelle verweisen auf "meshes/<datei>", die G1 hat meshdir="meshes"
    spec.meshdir = ""
    for mesh in spec.meshes:
        mesh.file = f"meshes/{mesh.file}"
    scene.attach_revo2(spec, prepare_hand=_hand_to_motors)
    for side in ("left", "right"):
        mesh = spec.mesh(f"{side}_rubber_hand")
        if mesh is not None:
            spec.delete(mesh)
    payload.attach(spec)
    return spec


def _git_commit():
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=REPO, capture_output=True,
                              text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def write_sim(spec, snap, out=SIM_DIR):
    """Sim-Kopie mit neuem Modell und config.yaml schreiben. spec muss kompiliert sein.

    Rueckgabe: (von der Platte geladenes MjModel, Manifest).
    """
    out = pathlib.Path(out)
    if out.exists():
        if not (out / MANIFEST).is_file():
            raise FileExistsError(f"{out} existiert, ist aber keine gebaute BrainCo-Sim - wird nicht geloescht")
        shutil.rmtree(out)
    shutil.copytree(snap, out, ignore=shutil.ignore_patterns("__pycache__", ".cache"))

    meshes = out / "assets" / "meshes"
    for src in (*scene.asset_mesh_dirs(), payload.MESH_DIR):
        for stl in src.glob("*.STL"):
            dst = meshes / stl.name
            if dst.exists() and not filecmp.cmp(stl, dst, shallow=False):
                raise FileExistsError(f"Mesh-Namenskonflikt G1 <-> Hand/Adapter/Anbauteile: {stl.name}")
            shutil.copy2(stl, dst)

    scene_path = out / SCENE_FILE
    scene_path.write_text(spec.to_xml())
    model = mujoco.MjModel.from_xml_path(str(scene_path))

    body = [i for i in range(model.nu) if not model.actuator(i).name.startswith(HAND_PREFIXES)]
    cfg = yaml.safe_load((snap / "config.yaml").read_text())
    cfg.update(
        ROBOT_SCENE=SCENE_FILE,
        ENABLE_ONSCREEN=False,
        NUM_HAND_MOTORS=0,  # Dex3 - die Revo2 steuert g1sim/brainco_env.py
        NUM_HAND_JOINTS=0,
        motor_effort_limit_list=[float(model.actuator_ctrlrange[i, 1]) for i in body],
    )
    header = ("# Erzeugt von scripts/51_build_brainco_sim.py - nicht von Hand aendern.\n"
              "# Revo2-Haende und DDS-Topics rt/brainco/*: g1sim/brainco_env.py im Repo UnitreeG1-Robot.\n")
    (out / "config.yaml").write_text(header + yaml.safe_dump(cfg, sort_keys=False))

    manifest = {
        "built_at": datetime.datetime.now().isoformat(timespec="seconds"),
        "repo_commit": _git_commit(),
        "hf_repo": HF_REPO,
        "hf_revision": HF_REVISION,
        "scene": SCENE_FILE,
        "nq": model.nq,
        "nv": model.nv,
        "nu": model.nu,
    }
    (out / MANIFEST).write_text(json.dumps(manifest, indent=2))
    return model, manifest
