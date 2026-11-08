"""Umrechnung zwischen G1 + BrainCo und dem Eingang von π₀.₅ (openpi) - dieselbe im Training und im Policy-Server.

Muster: openpi src/openpi/policies/libero_policy.py. Laeuft in der openpi-Umgebung.
Eingang je Regeltakt (Sim: g1sim/episode.policy_observation; Datensatz: nach dem RepackTransform in openpi_g1/config.py):
  observation/head_image   (H, W, 3) uint8, Kopfkamera D455 16:10 (openpi bringt sie mit Rand auf 224 x 224)
  observation/state   (26,) Layout g1sim/action_space.py
  prompt              Aufgabentext
  actions             (action_horizon, 26) absolute Sollwerte - nur im Training
π₀.₅ hat drei Bildeingaenge (Kopf, linkes und rechtes Handgelenk). Der echte G1 hat nur die Kopfkamera (Entscheidung D3): Die
beiden Handgelenk-Eingaenge bekommen ein schwarzes Bild mit Maske False ("gibt es nicht") - wie openpi es fuer fehlende Kameras
bei pi0/pi0.5 macht (src/openpi/policies/libero_policy.py).
Das Modell rechnet intern mit 32 Werten; openpi fuellt Zustand und Aktionen auf, G1BraincoOutputs schneidet wieder auf 26.
"""
import dataclasses

import einops
import numpy as np
from openpi import transforms

from g1sim import action_space, episode

# Unsere Kameras -> Bildeingaenge von π₀.₅
CAMERA_SLOTS = {"head_image": "base_0_rgb"}
EMPTY_SLOTS = ("left_wrist_0_rgb", "right_wrist_0_rgb")  # keine Kamera am echten G1 -> schwarz + Maske False
assert tuple(CAMERA_SLOTS) == episode.IMAGE_KEYS


def make_example(prompt="pick up the red bottle", image_shape=(200, 320), seed=0):
    """Zufaellige Beobachtung im Eingangsformat (Tests, Selbsttest in run.py). Zustand nahe der Startpose."""
    rng = np.random.default_rng(seed)
    frame = {key: rng.integers(256, size=(*image_shape, 3), dtype=np.uint8) for key in episode.IMAGE_KEYS}
    frame["state"] = action_space.READY_POSE + rng.normal(0.0, 0.05, action_space.DIM).astype(np.float32)
    return episode.policy_observation(frame, prompt)


def _parse_image(image):
    """LeRobot liefert Bilder als float32 (C, H, W) in 0..1, die Sim als uint8 (H, W, C). Ergebnis: uint8 (H, W, C)."""
    image = np.asarray(image)
    if np.issubdtype(image.dtype, np.floating):
        image = np.round(255 * image).astype(np.uint8)  # runden statt abschneiden: 254/255 * 255 ergibt sonst 253
    if image.shape[0] == 3:
        image = einops.rearrange(image, "c h w -> h w c")
    return image


@dataclasses.dataclass(frozen=True)
class G1BraincoInputs(transforms.DataTransformFn):
    """Beobachtung (+ Aktionen im Training) -> Modelleingang."""

    def __call__(self, data: dict) -> dict:
        state = np.asarray(data[episode.POLICY_KEYS["state"]], dtype=np.float32)
        if state.shape[-1] != action_space.DIM:
            raise ValueError(f"Zustand: erwartet {action_space.DIM} Werte, bekommen {state.shape}")
        images = {slot: _parse_image(data[episode.POLICY_KEYS[cam]]) for cam, slot in CAMERA_SLOTS.items()}
        masks = {slot: np.True_ for slot in CAMERA_SLOTS.values()}
        for slot in EMPTY_SLOTS:
            images[slot] = np.zeros_like(images["base_0_rgb"])
            masks[slot] = np.False_
        inputs = {"state": state, "image": images, "image_mask": masks}
        if "actions" in data:
            inputs["actions"] = np.asarray(data["actions"], dtype=np.float32)
        if "prompt" in data:
            inputs["prompt"] = data["prompt"]
        return inputs


@dataclasses.dataclass(frozen=True)
class G1BraincoOutputs(transforms.DataTransformFn):
    """Modellausgabe (auf 32 Werte aufgefuellt) -> 26-D-Aktionen. Nur im Policy-Server."""

    def __call__(self, data: dict) -> dict:
        return {"actions": np.asarray(data["actions"][..., : action_space.DIM])}
