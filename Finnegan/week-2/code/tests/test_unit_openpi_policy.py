"""Unit: Umrechnung G1 + BrainCo <-> π₀.₅ (openpi_g1/policy.py) - Kameras, Zustand, Delta-Aktionen, Rundlauf.

Laeuft nur in der openpi-Umgebung (~/dev/g1-ext/openpi/.venv), sonst uebersprungen. Ohne Netz, ohne Modell.
"""
import numpy as np
import pytest

pytest.importorskip("openpi")

from openpi import transforms  # noqa: E402

from g1sim import action_space, episode  # noqa: E402
from openpi_g1 import policy  # noqa: E402

pytestmark = pytest.mark.unit


def example_with_marked_cameras():
    """Die Kopfkamera bekommt ein einfarbiges Bild (Wert 10) - so faellt jede Verwechslung auf."""
    ex = policy.make_example()
    ex[episode.POLICY_KEYS["head_image"]] = np.full((200, 320, 3), 10, dtype=np.uint8)
    return ex


def test_kopfkamera_auf_base_handgelenk_schwarz_und_maskiert():
    # Der echte G1 hat nur die Kopfkamera (D3): Handgelenk-Eingaenge schwarz mit Maske False, wie openpi bei pi0.5
    out = policy.G1BraincoInputs()(example_with_marked_cameras())
    assert {slot: int(img.max()) for slot, img in out["image"].items()} == {
        "base_0_rgb": 10, "left_wrist_0_rgb": 0, "right_wrist_0_rgb": 0}
    assert all(img.shape == (200, 320, 3) for img in out["image"].values())
    assert {slot: bool(m) for slot, m in out["image_mask"].items()} == {
        "base_0_rgb": True, "left_wrist_0_rgb": False, "right_wrist_0_rgb": False}
    assert out["state"].shape == (action_space.DIM,) and out["state"].dtype == np.float32
    assert out["prompt"] == "pick up the red bottle"


def test_datensatzbilder_float_chw_werden_verlustfrei_uint8_hwc():
    img = np.random.default_rng(0).integers(256, size=(224, 224, 3), dtype=np.uint8)
    ex = policy.make_example()
    ex[episode.POLICY_KEYS["head_image"]] = img.transpose(2, 0, 1).astype(np.float32) / 255.0  # so liefert LeRobot
    out = policy.G1BraincoInputs()(ex)
    np.testing.assert_array_equal(out["image"]["base_0_rgb"], img)


def test_falscher_zustand_wird_abgelehnt():
    ex = policy.make_example()
    ex[episode.POLICY_KEYS["state"]] = np.zeros(29, dtype=np.float32)
    with pytest.raises(ValueError):
        policy.G1BraincoInputs()(ex)


def test_arme_als_delta_haende_absolut_und_zurueck():
    rng = np.random.default_rng(1)
    ex = policy.make_example()
    actions = rng.normal(size=(30, action_space.DIM)).astype(np.float32)
    ex["actions"] = actions.copy()
    state = ex[episode.POLICY_KEYS["state"]]

    data = transforms.DeltaActions(action_space.DELTA_MASK)(policy.G1BraincoInputs()(ex))
    np.testing.assert_allclose(data["actions"][:, :14], actions[:, :14] - state[:14], atol=1e-6)
    np.testing.assert_array_equal(data["actions"][:, episode.HANDS], actions[:, episode.HANDS])  # hands absolute
    np.testing.assert_allclose(data["actions"][:, 26], actions[:, 26] - state[26], atol=1e-6)  # waist yaw as a delta

    # Way back as in the policy server: the model works with 32 values, AbsoluteActions, then cut to 27
    padded = {"state": transforms.pad_to_dim(data["state"], 32), "actions": transforms.pad_to_dim(data["actions"], 32)}
    back = policy.G1BraincoOutputs()(transforms.AbsoluteActions(action_space.DELTA_MASK)(padded))
    assert back["actions"].shape == (30, action_space.DIM)
    np.testing.assert_allclose(back["actions"], actions, atol=1e-5)
