"""Track C e2e: the robot-day chain 84/86 -> 85 (the G1 sim obeying rt/arm_sdk), incl. fault injection, replay and the 40k policy.

Each test starts what it needs (a fresh 85, 84 for the sim factory pose, 81, 86) and stops it; 85 latches, so no test shares a
sim. 86 always logs to a temp folder (--out), never runs/real (m6-4). One DDS run at a time on this Mac (PLAN.md section 0).
    DYLD_LIBRARY_PATH=$HOME/dev/g1-ext/cyclonedds/install/lib ~/dev/g1-ext/.venv/bin/python -m pytest -m e2e \
        tests/test_e2e_real_adapter_sim.py -v
    G1_E2E_40K=1 ...same... -k vla_40k        # integration phase: the 40k checkpoint on MPS
G2 with 86 from the operators' Python 3.10 env (integration, 6.3): start 85 and 81 --replay as in test G2, then
    (source ~/dev/g1-ext/robot-day-test/robot-env.sh && python scripts/86_real_run.py replay --iface lo0 --yes-sim \
        --server 127.0.0.1:<port> --episode replay_demo --speed 0.5 --max-seconds 40 --no-camera --out /tmp/g2_py310)
"""
import json
import os
import signal
import subprocess
import sys
import time

import numpy as np
import pytest

pytest.importorskip("unitree_sdk2py")
pytest.importorskip("lerobot")

from conftest import REPO  # noqa: E402

sys.path.insert(0, str(REPO / "tests" / "helpers"))
import sim_host  # noqa: E402
from g1sim import real_safety  # noqa: E402

pytestmark = pytest.mark.e2e

REPLAY_DEMO = REPO / "tests" / "data" / "replay_demo.npz"
CKPT = os.path.expanduser("~/dev/g1-ext/smolvla-runs/run1/checkpoints/040000/pretrained_model")
SV = os.path.expanduser("~/dev/g1-ext/smolvla-test/.venv/bin/python")   # SmolVLA env: 81 in checkpoint mode needs torch
FAKE_CAMERA = f"python tests/helpers/fake_camera.py --npz {REPO / 'tests' / 'data' / 'selftest_episode.npz'}"
SIM_S = 120.0               # 85 runs longer than any 86 command below; the test stops it
RAMP_STILL_MAX = 0.03       # rad: motors 12-14 move less than this while the weight ramps in (frozen q0, N1)
WAIST_HOLD_TOL = 0.02       # rad: waist roll/pitch stay at q0 while w > 0 (rule 4)
MAX_GAP_S = 0.060           # rule 15: largest gap between arm_sdk messages
TRACK_MEDIAN_MAX, LEAD_MAX = 0.05, 0.15   # rad at w = 1 in the policy phase (G2, review n2)
RELEASE_SPEED_MAX = 0.35    # rad/s: release moves at 0.3 rad/s (rule 13) + float slack
FACTORY_TOL = 0.03          # rad: release ends this close to the recorded factory pose
CONTENT_TOL = 1e-4          # rad: a received message equals a sent one (DDS carries float32)
HOLD_BEFORE_SIGNAL_S = 22.0 # wall seconds after 86 starts: past preflight, ramp and the waypoint way in (~10 s in 85)


@pytest.fixture
def sim(tmp_path):
    """A fresh 85 plus today's sim factory pose recorded from it; yields (85 process, report path)."""
    report = tmp_path / "sim85.json"
    proc = sim_host.start_85(report, SIM_S)
    try:
        recorded = sim_host.record_sim_factory_pose()
        assert recorded.returncode == 0, recorded.stdout[-2000:]
        yield proc, report
    finally:
        sim_host.stop_process(proc)


def sim_report(sim) -> dict:
    """Stop 85 (it writes its report on SIGTERM) and read the report."""
    proc, report = sim
    sim_host.stop_process(proc)
    return json.loads(report.read_text())


def run_86(mode: str, out, *args, timeout_s: float = 240.0) -> dict:
    return sim_host.finish_86(sim_host.start_86(mode, out, *args), timeout_s)


def signal_86(mode: str, out, signals: int, *args) -> dict:
    """Start 86, wait until it holds the ready pose, send SIGINT `signals` times (0.3 s apart), collect the result."""
    proc = sim_host.start_86(mode, out, *args)
    time.sleep(HOLD_BEFORE_SIGNAL_S)
    for _ in range(signals):
        proc.send_signal(signal.SIGINT)
        time.sleep(0.3)
    return sim_host.finish_86(proc, timeout_s=120)


def weights(report: dict) -> np.ndarray:
    return np.array([m["weight"] for m in report["arm_sdk"]])


def start_81_checkpoint() -> tuple:
    """(process, port) of 81 serving the 40k checkpoint on MPS (SmolVLA env; integration phase only)."""
    server = subprocess.Popen([SV, str(REPO / "scripts" / "81_smolvla_server.py"), "--checkpoint", CKPT, "--device", "mps",
                               "--port", "0"], cwd=REPO, env=sim_host.dds_env(), stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True)
    return server, next(int(line.split()[1]) for line in server.stdout if line.startswith("LISTENING "))


def test_ready_ramps_without_motion_and_holds_waist(sim, tmp_path):
    """Proves (G1): `86 ready` ramps the weight 0 -> 1 in steps <= 0.02 without moving the waist, holds 13/14 at q0, never
    sends gains above the sim gains, ends at weight 0 and keeps every message gap under 60 ms.
    Catches: a ramp that re-reads q0 (the arm sags), a jump in weight, or a stalled 50 Hz loop."""
    result = run_86("ready", tmp_path, "--hold", "3")
    report = sim_report(sim)
    log, w = result["log"], weights(report)
    ramp = log["sent_phase"] == "ramp_in"
    held = log["sent_weight"] > 0

    assert result["code"] == 0, result["stdout"][-2000:]
    assert w[0] <= real_safety.MAX_WEIGHT_STEP and w[-1] == 0.0 and np.abs(np.diff(w)).max() <= real_safety.MAX_WEIGHT_STEP
    assert np.ptp(log["measured_q"][ramp][:, 0:3], axis=0).max() < RAMP_STILL_MAX
    assert np.abs(log["measured_q"][held][:, 1:3] - log["sent_q"][held][:, 1:3]).max() < WAIST_HOLD_TOL
    assert all(np.all(np.array(m["kp"]) <= real_safety.SIM_GAINS_KP + 1e-4) for m in report["arm_sdk"])
    assert all(np.all(np.array(m["kd"]) <= real_safety.SIM_GAINS_KD + 1e-4) for m in report["arm_sdk"])
    assert report["max_gap_s"] < MAX_GAP_S


def received_in_sent_order(report: dict, log: dict) -> bool:
    """Every message 85 received is one 86 logged as sent, in order (two pointers; DDS may drop, never invent)."""
    sent_q, sent_w, j = log["sent_q"], log["sent_weight"], 0
    for message in report["arm_sdk"]:
        q, w = np.array(message["q"]), message["weight"]
        while j < len(sent_q) and not (np.abs(sent_q[j] - q).max() < CONTENT_TOL and abs(sent_w[j] - w) < CONTENT_TOL):
            j += 1
        if j == len(sent_q):
            return False
    return True


def demo_rows_matched(policy_actions: np.ndarray, demo: np.ndarray) -> list:
    """For each logged policy action, the demo row index whose segment to the next row it lies on (interpolation)."""
    matched = []
    for action in policy_actions[~np.isnan(policy_actions).any(axis=1)]:
        a, b = demo[:-1], demo[1:]
        f = np.clip(np.einsum("ij,ij->i", action - a, b - a) / np.maximum(np.einsum("ij,ij->i", b - a, b - a), 1e-12), 0, 1)
        gap = np.abs(a + (b - a) * f[:, None] - action).max(axis=1)
        matched.append(int(np.argmin(gap)) if gap.min() < CONTENT_TOL else -1)
    return matched


def test_replay_tracks_demo_inside_envelope(sim, tmp_path):
    """Proves (G2): `86 replay` against 81 --replay plays the demo's actions at advancing ticks inside the envelope, the sim
    arm tracks at w = 1, every received message was sent by 86, the gate's files are the files at exit, and the exit goes
    through the ready pose.
    Catches: a replay stuck at tick 0 (m6-5), an exit straight from a policy pose down to q0 (RV6-1), a lazy import."""
    server, port = sim_host.start_81_replay(REPLAY_DEMO)
    try:
        result = run_86("replay", tmp_path, "--server", f"127.0.0.1:{port}", "--episode", REPLAY_DEMO.stem, "--speed", "0.5",
                        "--max-seconds", "40", "--no-camera")
    finally:
        sim_host.stop_process(server)
    report = sim_report(sim)
    log, side = result["log"], result["sidecar"]
    policy = (log["sent_phase"] == "policy") & (log["sent_weight"] == 1.0)
    lead = np.abs(log["sent_q"][policy] - log["measured_q"][policy])
    lo17 = real_safety.sdk_targets(real_safety.ENVELOPE_LO, np.full(17, -np.inf))
    hi17 = real_safety.sdk_targets(real_safety.ENVELOPE_HI, np.full(17, np.inf))
    rows = demo_rows_matched(log["policy_actions"], np.load(REPLAY_DEMO)["actions"])
    phases = [p for i, p in enumerate(log["sent_phase"]) if i == 0 or p != log["sent_phase"][i - 1]]

    assert result["code"] == 0, result["stdout"][-2000:]
    assert np.median(lead) < TRACK_MEDIAN_MAX and lead.max() < LEAD_MAX
    assert np.all(log["sent_q"][policy] >= lo17 - 1e-6) and np.all(log["sent_q"][policy] <= hi17 + 1e-6)
    assert report["pelvis_z_min"] > sim_host.FALL_HEIGHT
    assert side["loaded_files_at_exit"] == side["gate"]["loaded_files"]
    assert received_in_sent_order(report, log)
    assert -1 not in rows and rows == sorted(rows) and rows[-1] - rows[0] > 100, rows[:5] + rows[-5:]
    assert phases[phases.index("policy") + 1] == "to_ready"


def test_release_after_kill_moves_slowly_to_factory_pose(sim, tmp_path):
    """Proves (K): after a SIGKILL while 86 holds the arms (85 latches them), `86 ready` is refused for the factory gap and
    `86 release` starts at weight 1 on the measured angles, moves at <= 0.35 rad/s and ends at the factory pose at weight 0.
    Catches: a release that ramps out straight from where the arm hangs (swing), or a start that ignores the latched arms."""
    proc = sim_host.start_86("ready", tmp_path, "--hold", "30")
    time.sleep(HOLD_BEFORE_SIGNAL_S)
    proc.kill()                                  # author test in sim only; operators never kill (4.3 (f))
    proc.wait()
    refused = run_86("ready", tmp_path)
    released = run_86("release", tmp_path)
    log = released["log"]
    factory = np.array(json.loads((REPO / "runs" / "factory_pose_sim.json").read_text())["q"])
    speed = np.abs(np.diff(log["sent_q"], axis=0)).max(axis=1) * real_safety.SEND_HZ

    assert refused["code"] == 2 and "factory pose" in refused["stdout"], refused["stdout"][-1500:]
    assert released["code"] == 0, released["stdout"][-2000:]
    assert log["sent_weight"][0] == 1.0 and np.abs(log["sent_q"][0] - log["measured_q"][0]).max() < 0.02
    assert speed.max() < RELEASE_SPEED_MAX
    assert np.abs(log["measured_q"][-1] - factory).max() < FACTORY_TOL and log["sent_weight"][-1] == 0.0


def test_camera_stop_gives_soft_exit_and_opens_hands(sim, tmp_path):
    """Proves (F1): when the camera stream stops during `86 run`, 86 leaves softly (exit 3 camera_stale) and the last hand
    command is 0 (hands opened).
    Catches: a VLA that keeps acting on a frozen picture, or an exit that leaves the fingers closed."""
    if os.environ.get("G1_E2E_40K") != "1":
        pytest.skip("camera staleness stops only shadow/run, which need the 40k checkpoint server: set G1_E2E_40K=1")
    server, port = start_81_checkpoint()
    try:
        result = run_86("run", tmp_path, "--server", f"127.0.0.1:{port}", "--speed", "0.5", "--max-seconds", "60",
                        "--camera-cmd", f"{FAKE_CAMERA} --stop-after 20")
    finally:
        sim_host.stop_process(server)
    report = sim_report(sim)

    assert result["code"] == 3 and "camera_stale" in result["stdout"], result["stdout"][-2000:]
    assert report["hand_cmd"]["left"]["last"] == [0.0] * 6 and report["hand_cmd"]["right"]["last"] == [0.0] * 6


def test_policy_server_death_gives_soft_exit(sim, tmp_path):
    """Proves (F2): when the policy server dies during `86 replay`, 86 leaves softly (exit 3 policy_stale) and ends at w 0.
    Catches: a runner that waits forever for a chunk while holding the arms at weight 1."""
    server, port = sim_host.start_81_replay(REPLAY_DEMO)
    proc = sim_host.start_86("replay", tmp_path, "--server", f"127.0.0.1:{port}", "--episode", REPLAY_DEMO.stem,
                             "--speed", "0.5", "--max-seconds", "60", "--no-camera")
    time.sleep(HOLD_BEFORE_SIGNAL_S + 5)
    server.kill()
    server.wait()
    result = sim_host.finish_86(proc, timeout_s=120)
    report = sim_report(sim)

    assert result["code"] == 3 and ("policy_stale" in result["stdout"] or "policy_error" in result["stdout"]), \
        result["stdout"][-2000:]
    assert weights(report)[-1] == 0.0


def test_ctrl_c_once_soft_twice_freeze_thrice_hand_back(sim, tmp_path):
    """Proves (F6): one Ctrl+C during hold gives the slow way home (exit 3); a second one freezes the arms (exit 4
    second_signal, ruling F4) and a third ramps out on that frozen target; both runs' last arm_sdk message (weight 0)
    reaches the sim before 86 ends.
    Catches: a Ctrl+C that kills 86 mid-message, leaving the arms latched at weight 1, or a freeze that moves."""
    once = signal_86("ready", tmp_path, 1, "--hold", "30")
    thrice = signal_86("ready", tmp_path, 3, "--hold", "30")
    w = weights(sim_report(sim))
    frozen = thrice["log"]["sent_q"][thrice["log"]["sent_phase"] == "fault_hold"]
    ramp_out = thrice["log"]["sent_q"][thrice["log"]["sent_phase"] == "ramp_out"]
    handed_back = int(np.sum((w[1:] == 0.0) & (w[:-1] > 0.0)))   # weight reached 0 at the end of a run

    assert once["code"] == 3 and thrice["code"] == 4, once["stdout"][-1000:] + thrice["stdout"][-1000:]
    assert "second_signal" in thrice["stdout"] and len(frozen) > 0
    assert handed_back == 2 and w[-1] == 0.0
    assert np.abs(frozen - frozen[0]).max() == 0.0 and np.abs(ramp_out - frozen[0]).max() == 0.0


def test_lowstate_pause_gives_fault_exit(tmp_path):
    """Proves (F7): a 1 s gap in rt/lowstate while 86 holds the arms gives a fault exit (exit 4 lowstate_stale): 86 holds
    still while it is stale, recovers home once it is back (ruling F1), and its weight-0 ramp-out still reaches the sim.
    Catches: a runner that keeps commanding on stale joint angles (rule 10)."""
    report_path = tmp_path / "sim85.json"
    proc = sim_host.start_85(report_path, SIM_S, "--pause-lowstate-at", "10", "--pause-for", "1")
    try:
        assert sim_host.record_sim_factory_pose().returncode == 0
        result = run_86("ready", tmp_path, "--hold", "30")
    finally:
        sim_host.stop_process(proc)
    report = json.loads(report_path.read_text())

    assert result["code"] == 4 and "lowstate_stale" in result["stdout"], result["stdout"][-2000:]
    assert report["lowstate_pause"] is not None and weights(report)[-1] == 0.0


def test_foreign_arm_sdk_writer_refuses_start(sim, tmp_path):
    """Proves (F8): while another program sends rt/arm_sdk, 86 refuses before creating its own writer (exit 2) and the sim
    receives nothing from 86 (the factory pose was recorded before the foreign writer started, R3-9).
    Catches: 86 fighting an owner's arm tool for the arms."""
    foreign = subprocess.Popen([sys.executable, sim_host.__file__, "arm-sdk", "--weight", "0.3", "--seconds", "15",
                                "--pose", "zero"], cwd=REPO, env=sim_host.dds_env())
    try:
        time.sleep(2.0)
        result = run_86("ready", tmp_path)
    finally:
        foreign.wait(timeout=60)
    report = sim_report(sim)

    assert result["code"] == 2, result["stdout"][-2000:]
    assert all(abs(m["weight"] - 0.3) < CONTENT_TOL for m in report["arm_sdk"])


def test_84_sends_nothing_to_the_sim(tmp_path):
    """Proves: a full read-only 84 run against the sim puts 0 samples on rt/arm_sdk and owns 0 DDS writers.
    Catches: a publisher sneaking into the read-only bring-up the operators run first on Monday."""
    report_path = tmp_path / "sim85.json"
    proc = sim_host.start_85(report_path, SIM_S, "--no-blend")
    try:
        result = sim_host.run_script([REPO / "scripts" / "84_real_bringup.py", "--iface", "lo0", "--sim", "--yes-sim",
                                      "--steps", "probe,joints,hand-state,camera,record", "--seconds", "5",
                                      "--camera-cmd", FAKE_CAMERA], timeout_s=180)
    finally:
        sim_host.stop_process(proc)
    report = json.loads(report_path.read_text())

    assert "OWN WRITERS: 0 (must be 0)" in result.stdout, result.stdout[-2000:]
    assert report["arm_sdk_samples"] == 0


def test_vla_40k_shadow_then_live_runs_clean(sim, tmp_path):
    """Proves: the 40k checkpoint served by 81 on MPS runs 30 s in shadow (exit 0) and 60 s live at speed 0.5 (exit 0, or
    exit 3 envelope_streak, which is a policy finding, not a code failure).
    Catches: a chain that only works with the replay server (image, state or chunk format of the real policy)."""
    if os.environ.get("G1_E2E_40K") != "1":
        pytest.skip("integration phase only: set G1_E2E_40K=1 (loads the 40k checkpoint on MPS)")
    server, port = start_81_checkpoint()
    try:
        shadow = run_86("shadow", tmp_path, "--server", f"127.0.0.1:{port}", "--max-seconds", "30", "--camera-cmd", FAKE_CAMERA)
        live = run_86("run", tmp_path, "--server", f"127.0.0.1:{port}", "--speed", "0.5", "--max-seconds", "60",
                      "--camera-cmd", FAKE_CAMERA)
    finally:
        sim_host.stop_process(server)

    assert shadow["code"] == 0, shadow["stdout"][-2000:]
    assert live["code"] == 0 or (live["code"] == 3 and "envelope_streak" in live["stdout"]), live["stdout"][-2000:]
