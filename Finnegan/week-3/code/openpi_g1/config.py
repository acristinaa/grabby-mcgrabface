"""openpi-Trainingskonfiguration pi05_g1_brainco_lora: π₀.₅-Basis + LoRA auf G1-+-BrainCo-Demos (Stufe 2).

openpi fuehrt seine Konfigurationen als feste Liste (openpi/training/config.py, _CONFIGS). Statt openpi zu veraendern
(Commit 215abfb bleibt unangetastet), traegt register() unsere Konfiguration zur Laufzeit dort ein; openpi_g1/run.py ruft
das vor openpis eigenen Skripten auf. Muster: pi0_libero_low_mem_finetune (LoRA) und pi05_libero (π₀.₅).

Festgelegt:
  - action_horizon 30 = 1 s bei 30 Hz (wie openpi: LIBERO 10 bei 10 Hz, DROID 15 bei 15 Hz, ALOHA 50 bei 50 Hz)
  - Arme als Delta zum aktuellen Zustand, Haende absolut (action_space.DELTA_MASK, wie die Greifer bei ALOHA)
  - LoRA auf Gemma 2B (Bild + Sprache) und Gemma 300M (Aktions-Experte), uebrige Gewichte eingefroren, kein EMA
  - Datensatz lokal unter $HF_LEROBOT_HOME/REPO_ID (LeRobot v2.1), Aufgabentext aus dem Datensatz
  - Norm-Statistik und Checkpoints unter WORK_DIR (Standard ~/dev/g1-ext/openpi_g1, ueberschreibbar mit G1_OPENPI_WORK)
Noch offen, in Stufe 2.2 auf der GPU messen: batch_size und num_train_steps (hier openpis Standardwerte).
"""
import dataclasses
import os
import pathlib

from openpi import transforms as _transforms
from openpi.models import pi0_config
from openpi.training import config as _config
from openpi.training import weight_loaders

from g1sim import action_space, episode
from openpi_g1 import policy

CONFIG_NAME = "pi05_g1_brainco_lora"
REPO_ID = "g1_brainco/red_bottle_sim"
ACTION_HORIZON = 30
BASE_WEIGHTS = "gs://openpi-assets/checkpoints/pi05_base/params"
WORK_DIR = pathlib.Path(os.environ.get("G1_OPENPI_WORK", pathlib.Path.home() / "dev" / "g1-ext" / "openpi_g1")).expanduser()


@dataclasses.dataclass(frozen=True)
class LeRobotG1BraincoDataConfig(_config.DataConfigFactory):
    """Umrechnungen fuer unseren Datensatz (Training) und fuer den Policy-Server (Inferenz)."""

    # Wird eingesetzt, wenn eine Beobachtung keinen Aufgabentext mitbringt
    default_prompt: str | None = None

    def create(self, assets_dirs: pathlib.Path, model_config) -> _config.DataConfig:
        # Nur fuer den Datensatz: Spaltennamen (openpi_g1/convert.py) -> Namen, die auch die Sim schickt
        repack = _transforms.Group(inputs=[_transforms.RepackTransform({
            **{episode.POLICY_KEYS[key]: key for key in (*episode.IMAGE_KEYS, "state")},
            "actions": "actions",
            "prompt": "prompt",
        })])
        data_transforms = _transforms.Group(inputs=[policy.G1BraincoInputs()], outputs=[policy.G1BraincoOutputs()])
        data_transforms = data_transforms.push(
            inputs=[_transforms.DeltaActions(action_space.DELTA_MASK)],
            outputs=[_transforms.AbsoluteActions(action_space.DELTA_MASK)],
        )
        return dataclasses.replace(
            self.create_base_config(assets_dirs, model_config),
            repack_transforms=repack,
            data_transforms=data_transforms,
            model_transforms=_config.ModelTransformFactory(default_prompt=self.default_prompt)(model_config),
        )


MODEL = pi0_config.Pi0Config(
    pi05=True,
    action_horizon=ACTION_HORIZON,
    paligemma_variant="gemma_2b_lora",
    action_expert_variant="gemma_300m_lora",
)

TRAIN_CONFIG = _config.TrainConfig(
    name=CONFIG_NAME,
    model=MODEL,
    data=LeRobotG1BraincoDataConfig(repo_id=REPO_ID, base_config=_config.DataConfig(prompt_from_task=True)),
    weight_loader=weight_loaders.CheckpointWeightLoader(BASE_WEIGHTS),
    freeze_filter=MODEL.get_freeze_filter(),
    ema_decay=None,
    assets_base_dir=str(WORK_DIR / "assets"),
    checkpoint_base_dir=str(WORK_DIR / "checkpoints"),
    policy_metadata={
        "robot": "g1_brainco",
        "action_names": list(action_space.NAMES),
        "reset_pose": action_space.READY_POSE.tolist(),
        "action_horizon": ACTION_HORIZON,
    },
)


def register():
    """Konfiguration in openpis Liste eintragen (mehrfacher Aufruf schadet nicht). Rueckgabe: TrainConfig."""
    if CONFIG_NAME not in _config._CONFIGS_DICT:
        _config._CONFIGS.append(TRAIN_CONFIG)
        _config._CONFIGS_DICT[CONFIG_NAME] = TRAIN_CONFIG
    return _config.get_config(CONFIG_NAME)
