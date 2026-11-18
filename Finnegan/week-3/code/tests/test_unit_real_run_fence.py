"""Unit: the virtual fence and the chunk crossfade inside the 86 runner, the fence refusals and the motion gate's data file
(FENCE-SMOOTH.md 1.1, 1.5, 2.3; tests U4 shadow, U6, U7 hands, U8 replay/run, U11, U12 refusals).

The runner is driven with the fakes of tests/helpers/fake_real_run.py; the fence is test_unit_fence.WallFence (the real
g1sim.fence decision code on a one-wall geometry: distance = WALL - left shoulder pitch). No DDS, no MuJoCo, seconds.
"""
import json
import pathlib
import types

import numpy as np
import pytest

pytest.importorskip("unitree_sdk2py")

from g1sim import fence, motion_gate, real_preflight as pre, real_safety as rs  # noqa: E402
from g1sim.real_run_policy import PolicyPhase  # noqa: E402
from helpers.fake_real_run import (ClockLimit, FakeArm, FakeClock, FakeFrames, FakeHand, FakePolicy,  # noqa: E402
                                   demo_actions, last_index, make_io, read_sidecar, run, sent, signals_at)
from test_unit_fence import ARM, WALL, WallFence  # noqa: E402
from test_unit_motion_gate import git, script_86, unlocked  # noqa: E402,F401  (script_86 is a fixture)

pytestmark = pytest.mark.unit

ROWS = 300                                   # policy actions served by the fake replay server


def pushing_actions() -> np.ndarray:
    """Replay actions whose left shoulder pitch rises from the ready pose (-0.25) to the envelope top (-0.15), through the
    wall at -0.20, while both hand targets rise from 0.31 to 0.6 (values the ready pose never has)."""
    actions = demo_actions(ROWS)
    actions[:, 0] = np.linspace(-0.25, -0.15, ROWS)
    actions[:, 14:26] = np.linspace(0.31, 0.6, ROWS)[:, None]
    return actions


def test_fence_hold_keeps_the_hands_and_a_streak_ends_softly_through_the_ready_pose(tmp_path, capsys):
    """Proves (U6): while the fence holds the arm, the hand target is not updated either (it stays at the target of the
    last sent message); after fence_streak_ticks held messages the run ends with a soft exit (code 3, fence_streak) that
    goes up to the ready pose first; the operator sees one FENCE line per streak and the sidecar has the fence summary.
    Catches: fingers that keep closing next to the stand while the arm is held, a streak that never ends the run, or a
    streak handled as a fault (arms frozen at the table) [mutants 13, 16]. The fake clock stops a run that never ends after
    120 fake seconds (code review m3: a fault hold fails the test at once instead of hanging the suite)."""
    clock = FakeClock(limit_s=120.0)
    hands = {"left": FakeHand(), "right": FakeHand()}
    io = make_io(clock, FakeArm(clock), hands, policy=FakePolicy(pushing_actions()), fence=WallFence())

    result = run("replay", io, tmp_path, speed=0.5, max_seconds=20.0, episode="replay_demo", no_fence=False)

    phases, q = list(sent(result, "sent_phase")), sent(result, "sent_q")
    first_hand = phases.index("to_ready")                # hand k went out with arm message first_hand + k
    policy = [i for i, p in enumerate(phases) if p == "policy"]
    held = [i for i in policy[1:] if np.array_equal(q[i], q[i - 1])]
    hand_at = {i: hands["left"].sent[i - first_hand][0] for i in policy}
    side = read_sidecar(result)
    assert (result.exit_code, result.reason) == (3, "fence_streak")
    assert len(held) >= rs.fence_streak_ticks(0.5) - 1 and q[held[0]][ARM] < WALL
    assert all(np.array_equal(hand_at[i], hand_at[i - 1]) for i in held)
    assert hand_at[held[0]][0] > 0.31, "the hands never moved with the policy: the test proves nothing"
    assert phases[last_index(phases, "policy") + 1] == "to_ready"
    assert capsys.readouterr().out.count("FENCE: an arm") == 1
    assert side["fence"]["holds"] >= len(held) and side["events"]["fence_hold"]["count"] >= len(held)


class CageFence(WallFence):
    """The robot's own torso all round the policy pose (review fence-smooth O2): CAGE_M away there, 5 cm closer per rad the
    arms move off it. The policy can reach the pose (every step towards it moves away from the torso), but every way-out
    route from there - lift or straight to the ready pose - comes below the 2 cm self margin."""
    CAGE_M = 0.025

    def __init__(self, centre_arms14):
        super().__init__()
        self.centre, self.pairs = np.asarray(centre_arms14, dtype=float), {"torso": None}

    def pose_values(self, q17, hands12, pelvis_rp=None):
        q = np.array(q17, dtype=float)
        self.poses.append((q, np.array(hands12, dtype=float), pelvis_rp))
        return {"torso": self.CAGE_M - 0.05 * float(np.linalg.norm(q[3:17] - self.centre))}


POLICY_PITCH = -0.6                          # both shoulder pitches of the policy pose (the ready pose has -0.25)
SIGNAL_IN_HOLD_S = 25.0                      # s after the start: in the fault hold after the 3 s policy phase (16-19 s)


def caged_run(tmp_path, actions):
    """A 3 s `run` whose policy goes to the CageFence centre; one Ctrl+C at SIGNAL_IN_HOLD_S."""
    clock = FakeClock(limit_s=120.0)
    io = make_io(clock, FakeArm(clock), frames=FakeFrames(), policy=FakePolicy(actions, mode="smolvla"),
                 fence=CageFence(demo_actions(1, POLICY_PITCH)[0, 0:14]), signal_count=signals_at(clock, SIGNAL_IN_HOLD_S))
    return run("run", io, tmp_path, speed=0.5, max_seconds=3.0)


def after_policy(result) -> tuple:
    phases, q, w = list(sent(result, "sent_phase")), sent(result, "sent_q"), sent(result, "sent_weight")
    end = last_index(phases, "policy")
    return phases[end + 1:], q[end:], w[end + 1:]


def test_soft_exit_holds_when_every_route_home_would_push_into_the_body(tmp_path, capsys):
    """Proves (review fence-smooth O2): at the end of a policy phase whose last pose has no way-out route that keeps off
    the robot's body (lift and straight both come below the self margin), 86 does not move: fault exit_route_into_body,
    the arms held still at weight 1 on the last pose (no to_ready, no to_start), the operator sees why, and only the next
    Ctrl+C hands the arms back there (ramp-out). Nothing drops the arms before that Ctrl+C.
    Catches: the old soft exit, which drives the lift route into the torso unchecked, or a refused route driven anyway."""
    result = caged_run(tmp_path, demo_actions(shoulder_pitch=POLICY_PITCH))

    phases, q, weights = after_policy(result)
    hold = [i for i, p in enumerate(phases) if p == "fault_hold"]
    out = capsys.readouterr().out
    assert (result.exit_code, result.reason) == (rs.EXIT_CODES["fault"], "exit_route_into_body")
    assert "to_ready" not in phases and "to_start" not in phases and phases[0] == "fault_hold"
    assert len(hold) > rs.SEND_HZ and np.allclose(q[1:len(hold) + 1], q[0]) and np.all(weights[hold] == 1.0)
    assert phases[len(hold):] == ["ramp_out"] * (len(phases) - len(hold)) and weights[-1] == 0.0
    assert "exit route lift refused: torso" in out and "exit route straight refused: torso" in out
    assert read_sidecar(result)["events"]["exit_route_into_body"]["count"] == 1


def test_fault_recover_from_the_policy_phase_holds_when_its_route_would_push_into_the_body(tmp_path):
    """Proves (review fence-smooth O2): a fault recover that starts from a policy pose (here a NaN chunk after the arm
    reached the CageFence centre) checks its way out against the body as the soft exit does: no route passes, so it holds
    the measured pose at weight 1 instead of recovering, until the operator's Ctrl+C ramps out there.
    Catches: a recover that still drives arm_paths.home's lift route unchecked (to_factory messages into the torso)."""
    actions = demo_actions(shoulder_pitch=POLICY_PITCH)
    actions[60:, 3] = np.nan                     # left elbow; the arm is at the policy pose from row ~35 (0.35 rad)

    result = caged_run(tmp_path, actions)

    phases, q, weights = after_policy(result)
    assert (result.exit_code, result.reason) == (rs.EXIT_CODES["fault"], "invalid_target")
    assert np.allclose(q[0][3:17], CageFence(demo_actions(1, POLICY_PITCH)[0, 0:14]).centre)
    assert "to_factory" not in phases and set(phases) == {"fault_hold", "ramp_out"}
    assert np.allclose(q[1:phases.index("ramp_out") + 1], q[0]) and weights[-1] == 0.0
    assert read_sidecar(result)["events"]["exit_route_into_body"]["count"] == 1


def test_fake_clock_limit_ends_an_endless_fault_hold(tmp_path, monkeypatch):
    """Proves (code review m3): the U6 test's clock limit turns an endless fault hold of the fakes (what a streak handled as
    a fault runs into; here a NaN policy target with its fault action set to HOLD) into an immediate ClockLimit, not a
    300 s hang.
    Catches: a clock limit that the runner's `except Exception` swallows, which would bring the hang back."""
    monkeypatch.setitem(rs.FAULT_ACTIONS, "invalid_target", rs.HOLD)
    actions = demo_actions(ROWS)
    actions[:, 0] = np.nan
    clock = FakeClock(limit_s=120.0)
    io = make_io(clock, FakeArm(clock), policy=FakePolicy(actions), fence=WallFence(wall=0.5))

    with pytest.raises(ClockLimit):
        run("replay", io, tmp_path, speed=0.5, max_seconds=20.0, episode="replay_demo", no_fence=False)


def test_policy_phase_hands_the_hand_command_to_the_filter(tmp_path, monkeypatch):
    """Proves (code review m5): at the hand-over to the policy phase the runner gives the filter the hand target that is out
    (12 finite values) together with the command, so the fence's first "hold still" value uses those fingers.
    Catches: reset_last(command) alone, which leaves F(hold) of the first tick at the policy's candidate hands."""
    calls = []
    original = rs.SafetyFilter.reset_last
    monkeypatch.setattr(rs.SafetyFilter, "reset_last",
                        lambda self, command17, hands12=None: (calls.append(hands12), original(self, command17, hands12)))
    clock = FakeClock()
    io = make_io(clock, FakeArm(clock), policy=FakePolicy(demo_actions()), fence=WallFence(wall=0.5))

    result = run("replay", io, tmp_path, speed=0.5, max_seconds=1.0, episode="replay_demo", no_fence=False)

    assert result.exit_code == 0
    assert any(h is not None and len(h) == 12 and np.all(np.isfinite(h)) for h in calls), calls


def test_without_hand_state_the_fence_uses_the_commanded_hands_and_never_holds_for_it(tmp_path):
    """Proves (U7, review M3): with --no-hands and with a hand service that sends nothing, the fence gets the commanded
    (finite) hand target in every check and never holds because of the hands.
    Catches: a fence fed with measured fingers, which would hold every message of a --no-hands run until fence_streak."""
    for no_hands, hands in ((True, {}), (False, {"left": FakeHand(present=False), "right": FakeHand(present=False)})):
        clock = FakeClock()
        far = WallFence(wall=0.5)                         # the arm never gets close
        io = make_io(clock, FakeArm(clock), hands, policy=FakePolicy(demo_actions()), fence=far)

        result = run("replay", io, tmp_path, speed=0.5, max_seconds=2.0, episode="replay_demo", no_hands=no_hands,
                     no_fence=False)

        assert result.exit_code == 0 and "fence_hold" not in read_sidecar(result)["events"]
        assert far.poses and all(np.all(np.isfinite(hands12)) for _, hands12, _ in far.poses)


def test_shadow_counts_the_policy_command_only_and_never_holds(tmp_path, capsys):
    """Proves (U4, review M4): in shadow the fence looks at the policy's command only (cmd, no `pred`: the arm holds the ready
    pose, so its tracking error says nothing about the policy pose), counts shadow_fence_cmd and holds nothing; the summary
    line calls it information.
    Catches: a shadow fence fed with `pred` (a meaningless number the operators would act on) or one that holds the arms."""
    class CmdOnlyRecorder(WallFence):
        def values(self, cand17, hands12, measured17, last17, pelvis_rp=None, cmd_only=False):
            self.cmd_only_calls = getattr(self, "cmd_only_calls", []) + [cmd_only]
            return super().values(cand17, hands12, measured17, last17, pelvis_rp, cmd_only)

    clock = FakeClock()
    actions = demo_actions()
    actions[:, 0] = -0.19                                 # through the wall: below the margin on cmd alone
    recorder = CmdOnlyRecorder()
    io = make_io(clock, FakeArm(clock), frames=FakeFrames(), policy=FakePolicy(actions, mode="smolvla"), fence=recorder)

    result = run("shadow", io, tmp_path, max_seconds=2.0)

    side = read_sidecar(result)
    assert result.exit_code == 0 and recorder.cmd_only_calls and all(recorder.cmd_only_calls)
    assert side["events"]["shadow_fence_cmd"]["count"] > 0 and "fence_hold" not in side["events"]
    assert "cmd only, blind to arm sag, information" in capsys.readouterr().out


@pytest.mark.parametrize("mode, blend", [("replay", 0), ("run", rs.BLEND_ACTIONS), ("shadow", 0)])
def test_only_run_fades_between_chunks(mode, blend):
    """Proves (U8, review B3): the policy phase's scheduler fades over BLEND_ACTIONS only in `run`; replay and shadow switch
    chunks at once.
    Catches: a fade in replay, which moves logged actions off the demo polyline and fails the replay gate [mutant 17]."""
    runner = types.SimpleNamespace(mode=mode, args=types.SimpleNamespace(speed=0.5, done_check=True), command=np.zeros(17), q0=np.zeros(17),
                                   io=types.SimpleNamespace(make_worker=lambda: None),
                                   new_filter=lambda q0, fenced=True: None)

    phase = PolicyPhase(runner, send_policy=mode != "shadow")

    assert phase.scheduler.blend == blend


# --- refusals ---------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("iface, layout, no_fence, refused", [
    ("en5", None, False, "fence needs the table layout"),
    ("en5", None, True, "--no-fence is for loopback tests only"),
    ("lo0", None, False, "--no-fence"),                   # loopback: no silent default either (review N3)
    ("en5", {"stance": {}}, False, None),
    ("lo0", None, True, None),
])
def test_replay_and_run_need_a_layout_or_loopback_no_fence(iface, layout, no_fence, refused):
    """Proves (U12, R-F7, review N3): replay and run need the layout for the fence; --no-fence is refused off loopback;
    on loopback a run without layout must say --no-fence; ready/release/shadow need nothing.
    Catches: a robot run without a fence because nobody typed the layout, or --no-fence on the robot."""
    for mode in ("replay", "run"):
        problem = pre.fence_option_problem(mode, layout, no_fence, iface == "lo0")
        assert (problem is None) if refused is None else (refused in problem), (mode, problem)
    assert all(pre.fence_option_problem(m, None, False, False) is None for m in ("ready", "release", "shadow"))


def test_runner_refuses_replay_when_no_fence_was_built(tmp_path):
    """Proves (U12, fail closed): a replay whose layout was given but whose fence is missing is refused before any writer.
    Catches: a caller that forgot to build the fence and silently runs the policy unfenced."""
    clock = FakeClock()
    arm = FakeArm(clock)
    io = make_io(clock, arm, policy=FakePolicy(demo_actions()))

    result = run("replay", io, tmp_path, speed=0.5, max_seconds=2.0, episode="replay_demo", no_fence=False,
                 fence_layout=fence.layout_from_sheet(0.0))

    assert result.exit_code == rs.EXIT_CODES["refused"] and "no fence was built" in result.reason and arm.messages == []


def test_86_refuses_replay_off_loopback_when_mujoco_is_missing(script_86, monkeypatch):  # noqa: F811 (fixture)
    """Proves (U12, review B1): `86 replay --iface en5` with the layout but without mujoco in the robot env exits 2 with the
    setup line, before the gate and before any robot connection.
    Catches: an 86 that crashes on import, or one that runs the policy without a fence when mujoco is missing."""
    module, reached_robot = script_86
    monkeypatch.setattr(fence, "mujoco", None)
    printed = []
    monkeypatch.setattr("builtins.print", lambda *a, **k: printed.append(" ".join(map(str, a))))

    code = module.main(["replay", "--iface", "en5", "--server", "127.0.0.1:1", "--episode", "replay_demo", "--no-camera",
                        "--speed", "0.5", "--fence-setback-cm", "0"])

    assert code == rs.EXIT_CODES["refused"] and reached_robot == []
    assert any("REFUSED fence: mujoco is not installed" in line for line in printed), printed


def test_86_reads_the_layout_once(script_86, tmp_path):  # noqa: F811 (fixture)
    """Proves (R-F7): --fence-layout FILE gives the layout keys of the file; the sheet options and --fence-layout together,
    or --no-fence with a layout, are refused by the parser.
    Catches: two layouts silently merged, or --no-fence overriding an entered layout."""
    module, _ = script_86
    layout_file = tmp_path / "layout.json"
    layout_file.write_text(json.dumps({**fence.layout_from_sheet(3.0), "source": {"x": 1}}))
    base = ["replay", "--iface", "lo0", "--server", "127.0.0.1:1", "--episode", "e", "--no-camera"]

    args = module.parse_args(base + ["--fence-layout", str(layout_file)])
    sheet = module.parse_args(base + ["--fence-setback-cm", "3"])

    assert args.layout == fence.layout_from_sheet(3.0) == sheet.layout
    for extra in (["--fence-layout", str(layout_file), "--fence-setback-cm", "3"],
                  ["--no-fence", "--fence-setback-cm", "3"]):
        with pytest.raises(SystemExit):
            module.parse_args(base + extra)


def test_86_refuses_a_layout_file_without_the_stand_offset(script_86, tmp_path, capsys):  # noqa: F811 (fixture)
    """Proves (code review m4): --fence-layout FILE without objects.volleyball (the stand offset) is refused by the parser
    with a line naming it, before anything is built or sent.
    Catches: a file that passes the parser and ends in a KeyError traceback inside Fence.nominal."""
    module, reached_robot = script_86
    base = ["replay", "--iface", "lo0", "--server", "127.0.0.1:1", "--episode", "e", "--no-camera"]
    for content in ({"stance": fence.layout_from_sheet(0.0)["stance"]}, {"objects": {"volleyball": {"dx_m": 0.0}}}, []):
        layout_file = tmp_path / "layout.json"
        layout_file.write_text(json.dumps(content))

        with pytest.raises(SystemExit):
            module.parse_args(base + ["--fence-layout", str(layout_file)])

        assert "has no objects.volleyball" in capsys.readouterr().err, content
    assert reached_robot == []


# --- the gate checks the model file (U11) -----------------------------------------------------------------------------

def test_gate_checks_the_fence_model_file(repo_with_model):
    """Proves (U11, review B1): a loaded module's DATA_FILES are in the gate's file list, and an edited fence model file
    locks the gate and is named.
    Catches: a swapped or edited collision model that moves the fence lines without anyone reviewing it [mutant 21]."""
    path, modules = repo_with_model
    files = motion_gate.loaded_repo_files(path, modules)
    clean = motion_gate.gate_problems(path, files)
    (path / fence.DATA_FILES[0]).write_bytes(b"edited")

    problems = motion_gate.gate_problems(path, files)

    assert files == [pathlib.Path("g1sim/data/fence_model.xml.gz"), pathlib.Path("g1sim/fence.py")]
    assert clean == [] and len(problems) == 1 and fence.DATA_FILES[0] in problems[0]


@pytest.fixture
def repo_with_model(tmp_path):
    """A throw-away repo with g1sim/fence.py and its model file committed and unlocked; (path, fake sys.modules)."""
    (tmp_path / "g1sim" / "data").mkdir(parents=True)
    (tmp_path / "docs").mkdir()
    (tmp_path / "g1sim" / "fence.py").write_text("DATA_FILES = ('g1sim/data/fence_model.xml.gz',)\n")
    (tmp_path / fence.DATA_FILES[0]).write_bytes(b"model")
    git(tmp_path, "init", "-q")
    git(tmp_path, "add", ".")
    git(tmp_path, "commit", "-q", "-m", "fence")
    path = unlocked((tmp_path, git(tmp_path, "rev-parse", "HEAD")))
    modules = {"g1sim.fence": types.SimpleNamespace(__file__=str(path / "g1sim" / "fence.py"),
                                                     DATA_FILES=fence.DATA_FILES)}
    return path, modules
