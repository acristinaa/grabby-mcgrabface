"""Handover snapshot in the episode npz (scripts/69_volleyball_demo_factory.py): round trip without pickle, verified write."""
import numpy as np

from g1sim import action_space, episode


def snapshot():
    rng = np.random.default_rng([100, 3, 1])
    return {"state": np.linspace(0, 1, 50), "head_eq_data": np.arange(22.0).reshape(2, 11), "timestep": 1 / 300,
            "variation": {"stance": {"x_m": -0.03, "y_m": 0.1, "yaw_deg": 4.2}, "physics": {"gain_scale": [1.0] * 29}},
            "body_target": np.zeros(29), "hand_target": {"left": np.full(6, 0.1), "right": np.full(6, 0.2)},
            "waist_yaw_target": 0.61, "noise_rng": rng.bit_generator.state}


def test_handover_round_trip(tmp_path):
    rec = episode.EpisodeRecorder("put the volleyball in the basket", 30)
    for _ in range(3):
        rec.add({"head_image": np.zeros((200, 320, 3), np.uint8), "state": np.zeros(action_space.DIM)}, np.zeros(action_space.DIM))
    path = rec.save(tmp_path / "ep.npz", success=True, extra=episode.handover_arrays(snapshot()))
    ep = episode.load(path)  # allow_pickle=False
    assert episode.validate(ep) == []
    back, original = episode.handover_snapshot(ep), snapshot()
    np.testing.assert_array_equal(back["state"], original["state"])
    np.testing.assert_array_equal(back["head_eq_data"], original["head_eq_data"])
    assert back["noise_rng"] == original["noise_rng"]  # 128-bit ints survive JSON exactly
    assert back["variation"] == original["variation"]
    assert back["waist_yaw_target"] == original["waist_yaw_target"] and back["timestep"] == original["timestep"]
    np.testing.assert_array_equal(back["hand_target"]["right"], original["hand_target"]["right"])
    restored = np.random.default_rng()
    restored.bit_generator.state = back["noise_rng"]
    assert restored.normal() == np.random.default_rng([100, 3, 1]).normal()


def test_write_verified_leaves_no_part_file(tmp_path):
    path = episode.write_verified(tmp_path / "a.bin", b"x" * 1000)
    assert path.read_bytes() == b"x" * 1000
    assert [p.name for p in tmp_path.iterdir()] == ["a.bin"]
