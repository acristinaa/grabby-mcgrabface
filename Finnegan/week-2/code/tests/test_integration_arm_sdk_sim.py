"""Track C: 85 alone - the LeRobot G1 sim obeys rt/arm_sdk like Unitree's controller is assumed to (blend, latch, refusals).

Each test starts its own 85 (a DDS sim on lo0) and a foreign rt/arm_sdk writer from tests/helpers/sim_host.py, both as separate
processes, and stops them. Only one DDS run at a time on this Mac (PLAN.md section 0); about 1 min per sim start.
    DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib ~/dev/g1-ext/.venv/bin/python -m pytest -m integration \
        tests/test_integration_arm_sdk_sim.py -v
"""
import subprocess
import sys

import numpy as np
import pytest

pytest.importorskip("unitree_sdk2py")
pytest.importorskip("lerobot")

from conftest import REPO  # noqa: E402

sys.path.insert(0, str(REPO / "tests" / "helpers"))
import sim_host  # noqa: E402

pytestmark = pytest.mark.integration

READY17 = sim_host.POSES["ready"]           # the ready pose 86 rises to (17-vector, motors 12..28)
ARMS = slice(3, 17)                         # motors 15-28 inside a 17-vector
BLEND_WEIGHT = 0.5                          # half way: the blended command differs from both inputs
BLEND_S, LATCH_SEND_S = 4.0, 3.0            # publisher seconds at weight 0.5, then at weight 1
SIM_S = 16.0                                # 85 run: both publishers (~9 s with process starts) + ~5 s latched
SETTLE_S = 1.0                              # seconds the arm needs to reach a new command (kp 50-80, PLAN.md 2.3)
OWN_DRIFT_MAX = 0.01                        # rad; GR00T's own arm targets must not move while we blend
TRACK_TOL = 0.15                            # rad between the sim arm and the command it obeys (gravity sag at kp 40-80)
NOT_MOVED_TOL = 0.1                         # rad; with --no-blend the arm must stay where GR00T holds it


@pytest.fixture(scope="module")
def blend_run(tmp_path_factory):
    """One 85 run: a foreign writer at weight 0.5 for 4 s, then at weight 1 for 3 s, then silence (latch)."""
    report = tmp_path_factory.mktemp("sim85") / "blend.json"
    proc = sim_host.start_85(report, SIM_S)
    try:
        half = sim_host.run_arm_sdk_publisher(BLEND_WEIGHT, BLEND_S)
        full = sim_host.run_arm_sdk_publisher(1.0, LATCH_SEND_S)
        assert half.returncode == 0 and full.returncode == 0, half.stderr + full.stderr
        return sim_host.finish_85(proc, report, timeout_s=SIM_S + 60)
    finally:
        sim_host.stop_process(proc)


def measured_arms_at(report: dict, t: float) -> np.ndarray:
    """Measured arm angles (motors 15-28) of the last 50 Hz sample before wall time t."""
    rows = [m for m in report["measured"] if m["t"] <= t]
    return np.asarray(rows[-1]["q"])[15:29]


def test_85_blends_weighted_arm_sdk_into_a_lowcmd_copy(blend_run):
    """Proves: the copy sent to the sim holds w * arm_sdk + (1 - w) * GR00T for motors 12-28, GR00T's own message never
    changes, and the sim arm follows the blended command.
    Catches: an in-place blend that drifts GR00T's own arm targets towards ours (review m6), or a blend the sim ignores."""
    samples = [s for s in blend_run["blend_samples"] if s["weight"] == BLEND_WEIGHT]
    first_own = np.asarray(blend_run["blend_samples"][0]["own_q"])

    own = np.array([s["own_q"] for s in samples])
    sent = np.array([s["sent_q"] for s in samples])

    assert len(samples) > 10, "85 recorded no blended message at weight 0.5 - did the publisher reach the sim?"
    np.testing.assert_allclose(sent, BLEND_WEIGHT * READY17 + (1 - BLEND_WEIGHT) * own, atol=1e-5)
    assert np.abs(own[:, ARMS] - first_own[ARMS]).max() < OWN_DRIFT_MAX, "GR00T's own arm targets drifted (in-place blend?)"
    settled = samples[-1]
    assert np.abs(measured_arms_at(blend_run, settled["t"]) - np.asarray(settled["sent_q"])[ARMS]).max() < TRACK_TOL


def test_85_latches_last_message_after_sender_stops(blend_run):
    """Proves: after the last rt/arm_sdk message 85 keeps blending it at weight 1 and the arm stays at that target.
    Catches: an 85 that drops back to GR00T's arms when messages stop (the real controller is assumed to latch, PLAN.md 1)."""
    last_received = blend_run["arm_sdk"][-1]["t"]
    after = [s for s in blend_run["blend_samples"] if s["t"] > last_received + SETTLE_S]

    final_arms = measured_arms_at(blend_run, after[-1]["t"])

    assert len(after) > 10, "85 ran too short after the sender stopped to show the latch"
    assert all(s["weight"] == 1.0 for s in after)
    assert np.abs(final_arms - READY17[ARMS]).max() < TRACK_TOL


def test_85_refuses_a_non_loopback_iface(tmp_path):
    """Proves: 85 refuses (exit 2) any interface that is not loopback, before DDS or the sim start, and writes no report.
    Catches: a sim that would publish a fake rt/lowstate on a real network where the robot or another reader listens."""
    report = tmp_path / "refused.json"

    proc = subprocess.run([sys.executable, str(sim_host.SIM85), "--iface", "en0", "--seconds", "1", "--report", str(report)],
                          cwd=REPO, env=sim_host.dds_env(), capture_output=True, text=True, timeout=60)

    assert proc.returncode == 2, proc.stdout + proc.stderr
    assert "not a loopback interface" in proc.stderr
    assert not report.exists()


def test_85_no_blend_counts_samples_without_obeying(tmp_path):
    """Proves: with --no-blend 85 counts every rt/arm_sdk sample but sends GR00T's command unchanged, so the arm stays.
    Catches: a --no-blend that still obeys - then "84 sends nothing" could not be checked without moving the sim arm."""
    report = tmp_path / "no_blend.json"
    proc = sim_host.start_85(report, 10.0, "--no-blend")
    try:
        publisher = sim_host.run_arm_sdk_publisher(1.0, 4.0)
        result = sim_host.finish_85(proc, report, timeout_s=70)
    finally:
        sim_host.stop_process(proc)

    sent = np.array([s["sent_q"] for s in result["blend_samples"]])
    own = np.array([s["own_q"] for s in result["blend_samples"]])
    arms = np.array([m["q"][15:29] for m in result["measured"]])

    assert publisher.returncode == 0, publisher.stderr
    assert result["arm_sdk_samples"] > 100
    np.testing.assert_array_equal(sent, own)
    assert np.abs(arms - arms[0]).max() < NOT_MOVED_TOL
