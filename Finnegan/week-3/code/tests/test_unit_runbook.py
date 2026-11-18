"""Robot-day handover kit (Tracks E1, E2): the runbook's commands, its "never" box, Part C's order and on-site S1 check,
the loopback helper's refusals, 81's cuda choice.

The operators run only what the runbook says, so a flag that does not exist or a missing "never" line costs the robot day.
The loopback helper publishes a fake rt/lowstate; on the robot network that would be read as the robot itself.
Runbook tests need no DDS; the helper tests skip without unitree_sdk2py (project .venv, PLAN.md m6-3).
"""
import ast
import importlib.util
import itertools
import pathlib
import re
import shlex
import subprocess
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parent.parent
RUNBOOK = REPO / "docs" / "ROBOT_DAY_RUNBOOK.md"
LOOPBACK_PUB = REPO / "tests" / "helpers" / "loopback_lowstate_pub.py"
SELFTEST = REPO / "scripts" / "89_robot_day_selftest.py"
BRINGUP_STEPS_SOURCE = REPO / "g1sim" / "real_bringup.py"   # STEPS = (...) of 84, PLAN.md 3.1
INLINE_COMMAND_RE = re.compile(r"`([^`]*scripts/[^`]*)`")    # a command written inside a sentence, e.g. `python scripts/89_... --skip-policy`
SCRIPT_RE = re.compile(r"(?:^|/)(scripts/[\w.-]+\.(?:py|sh)|tests/helpers/[\w.-]+\.py)$")
SEPARATORS = {"|", "||", "&&", ";", ">", ">>", "2>&1", "&"}
# ruling S1 (SAFE-PATHS.md): 87 on the M0 record itself, waypoint paths, waist 0.6, the table on its marks (setback 0)
S1_COMMAND = ("scripts/87_arm_path_clearance.py --waypoint-paths --factory-pose runs/factory_pose_robot.json "
              "--report runs/safe_paths_onsite.json --demo tests/data/replay_demo.npz --waist-limits 0.6 --setbacks 0.0")


def runbook_commands() -> list:
    """Every command line in the runbook: code-block lines (backslash continuations joined) and inline `...scripts/...` commands."""
    return commands_in(RUNBOOK.read_text())


def commands_in(text: str) -> list:
    """The command lines of one piece of runbook text, in the order the operators meet them."""
    commands, block, pending = [], False, ""
    for line in text.splitlines():
        if line.strip().startswith("```"):
            block = not block
            continue
        if not block:
            commands += INLINE_COMMAND_RE.findall(line)
            continue
        pending += line.rstrip()
        if pending.endswith("\\"):
            pending = pending[:-1] + " "
            continue
        commands.append(pending.strip())
        pending = ""
    return [command for command in commands if command and not command.startswith("#")]


def flags_by_script() -> dict:
    """{script path: set of --flags the runbook passes to it}. A quoted inner command is one token and is not split."""
    used = {}
    for command in runbook_commands():
        script = None
        for raw in shlex.split(command.split(" #")[0]):
            token = raw.strip("()")  # "(source x && python scripts/89_...py)" subshell brackets
            match = SCRIPT_RE.search(token)
            if match:
                script = match.group(1)
                used.setdefault(script, set())
            elif token in SEPARATORS:
                script = None
            elif script and token.startswith("--"):
                used[script].add(token.split("=")[0])
            if raw.endswith(")"):
                script = None
    return used


def declared_flags(path: pathlib.Path) -> set:
    """--flags a script accepts: add_argument strings for Python, `--flag)` case labels for the setup shell script."""
    text = path.read_text()
    if path.suffix == ".sh":
        return set(re.findall(r"(--[a-z0-9-]+)[|)]", text))
    flags = set()
    for node in ast.walk(ast.parse(text)):
        if isinstance(node, ast.Call) and getattr(node.func, "attr", "") == "add_argument":
            flags |= {arg.value for arg in node.args if isinstance(arg, ast.Constant) and str(arg.value).startswith("--")}
    return flags


@pytest.mark.parametrize("script", sorted(flags_by_script()))
def test_runbook_commands_match_script_flags(script):
    """Proves: every --flag the runbook passes to a script exists in that script's argparse (or the setup script's options).
    Catches: a runbook command that dies with "unrecognized arguments" in front of the robot, e.g. after a flag rename."""
    path = REPO / script
    if not path.exists():
        pytest.skip(f"{script} not built yet (built in parallel by another track; checked again at integration)")

    unknown = flags_by_script()[script] - declared_flags(path)

    assert not unknown, f"runbook passes {sorted(unknown)} to {script}, which does not accept them"


def bringup_steps() -> set:
    """The step names 84 accepts, read from g1sim/real_bringup.py's STEPS tuple without importing it (no DDS needed)."""
    for node in ast.parse(BRINGUP_STEPS_SOURCE.read_text()).body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "STEPS":
            return set(ast.literal_eval(node.value))
    raise AssertionError(f"no STEPS = (...) in {BRINGUP_STEPS_SOURCE}")


def test_runbook_84_steps_exist():
    """Proves: every name the runbook passes to 84 --steps is one of 84's steps.
    Catches: a runbook line like `--steps hand` (the step is `hand-state`) that 84 rejects at the robot."""
    if not BRINGUP_STEPS_SOURCE.exists():
        pytest.skip("g1sim/real_bringup.py not built yet (Track A; checked again at integration)")
    used = set()
    for command in runbook_commands():
        for match in re.finditer(r"84_real_bringup\.py.*?--steps[ =]([\w,-]+)", command):
            used |= set(match.group(1).split(","))

    unknown = used - bringup_steps()

    assert used, "the runbook passes no --steps to 84 at all - did the parser break?"
    assert not unknown, f"runbook passes unknown 84 steps {sorted(unknown)}"


def test_never_box_lists_72_moving_commands():
    """Proves: the runbook's "never" box names the 72 commands that move the robot, kill -9 and editing files.
    Catches: a runbook edit that drops `72 arm` (it leaves waist roll/pitch at kp 0, F-72a) or `72 hand --bewegen`."""
    text = RUNBOOK.read_text()
    box = re.search(r"^#+ .*NEVER.*?$(.*?)^#+ ", text, flags=re.MULTILINE | re.DOTALL)

    assert box, "no section headed 'NEVER' in the runbook"
    for needle in ("72_real_check.py arm", "72_real_check.py hand --bewegen", "kill -9", "Editing any file"):
        assert needle in box.group(1), f"'{needle}' missing from the never box"


def runbook_section(heading_start: str) -> str:
    """The runbook text from the `## <heading_start>` line up to the next `## ` heading."""
    match = re.search(rf"^## {re.escape(heading_start)}.*?$(.*?)(?=^## |\Z)", RUNBOOK.read_text(), flags=re.MULTILINE | re.DOTALL)
    assert match, f"no '## {heading_start}' section in the runbook"
    return match.group(1)


def test_runbook_part_c_starts_with_factory_pose_record():
    """Proves: Part C's first script command is 84's M0 factory-pose record, and its 86 motion commands come after it.
    Catches: a Part C that lets 86 run before M0, so `86 release` and every ramp-out aim at a missing or stale factory pose (RV6-2)."""
    scripts = [command for command in commands_in(runbook_section("Part C")) if "scripts/" in command]

    first_86 = next((i for i, command in enumerate(scripts) if "86_real_run.py" in command), None)

    assert scripts, "Part C has no commands (still the push-1 stub?)"
    assert "84_real_bringup.py" in scripts[0] and "--record-factory-pose" in scripts[0], f"first Part C command: {scripts[0]}"
    assert first_86 is not None and first_86 > 0, "Part C has no 86 command after M0"


def test_runbook_part_c_runs_s1_on_the_m0_pose_before_the_table():
    """Proves: Part C runs 87 on the M0 record itself (ruling S1) after M0 and before the first table step (an 86 line with
    the table on its marks, --fence-setback-cm 0), and every such table line uses --waist-max 0.6 (ruling S2).
    Catches: a Part C that goes back to the grid lookup (a grid row does not stand in for the real pose, SAFE-PATHS.md S1),
    drops the on-site check, or sends a table step at a waist setting whose replay hits the stand or basket."""
    scripts = [command for command in commands_in(runbook_section("Part C")) if "scripts/" in command]

    s1 = next((i for i, command in enumerate(scripts) if S1_COMMAND in command), None)
    table = [i for i, command in enumerate(scripts) if "86_real_run.py" in command and "--fence-setback-cm 0" in command]

    assert s1 is not None and s1 > 0, "Part C has no S1 command after M0"
    assert table and min(table) > s1, "a table step comes before the S1 check (or there is none)"
    assert all("--waist-max 0.6" in scripts[i] for i in table), [scripts[i] for i in table if "--waist-max 0.6" not in scripts[i]]


@pytest.fixture
def pub():
    """The loopback helper as a module (needs the SDK: skipped in the project .venv)."""
    pytest.importorskip("unitree_sdk2py")
    spec = importlib.util.spec_from_file_location("loopback_lowstate_pub", LOOPBACK_PUB)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_loopback_pub_refuses_robot_network(pub, monkeypatch):
    """Proves: with a 192.168.123.x address present the helper exits 2 before DDS is even started.
    Catches: a helper that forgets the robot-network check and puts a fake rt/lowstate next to the real robot (R4-4)."""
    started = []
    monkeypatch.setattr(pub, "robot_net_present", lambda: True)
    monkeypatch.setattr(pub, "ChannelFactoryInitialize", lambda *args: started.append(args))

    code = pub.main(["--iface", "lo0", "--seconds", "1"])

    assert code == pub.EXIT_REFUSED
    assert started == [], "DDS was initialised although the robot network was present"


def test_loopback_pub_refuses_non_loopback_iface(pub):
    """Proves: an interface that is not loopback (en0, the Wi-Fi or robot cable) is refused even with no robot address present.
    Catches: a helper that only checks for 192.168.123.x and publishes a fake rt/lowstate on a real network adapter."""
    problem = pub.start_problem("en0", robot_net=False)

    assert problem is not None and "not a loopback interface" in problem


def test_bound_interfaces_reads_selected_line(pub):
    """Proves: the helper reads the interface name CycloneDDS reports in its "selected interfaces:" trace line.
    Catches: a parser that returns [] or the wrong name, so the bound-interface refusal blocks lo0 or lets en0 through."""
    trace = ("1792339319.913000 [0]   14815344: config: Domain/General/Interfaces/NetworkInterface[@name]/1: lo0\n"
             "1792339319.913554 [0]   14815344: selected interfaces: lo0 (index 1 priority 2)\n")   # real /tmp/cdds.LOG line

    names = pub.bound_interfaces(trace)

    assert names == ["lo0"]


def test_loopback_pub_refuses_when_dds_bound_another_iface(pub, monkeypatch, tmp_path):
    """Proves: when CycloneDDS reports it bound en0 although lo0 was asked for, the helper exits 2 and publishes nothing.
    Catches: a helper that trusts --iface without reading the trace line and sends fake rt/lowstate out of a real adapter."""
    trace = tmp_path / "cdds.LOG"
    trace.write_text("1792339319.913554 [0]   14815344: selected interfaces: en0 (index 6 priority 0)\n")
    published = []
    monkeypatch.setattr(pub, "CYCLONE_TRACE_LOG", trace)
    monkeypatch.setattr(pub, "robot_net_present", lambda: False)
    monkeypatch.setattr(pub, "ChannelFactoryInitialize", lambda *args: None)
    monkeypatch.setattr(pub, "lowstate_publish_fn", lambda: published.append("lowstate"))

    code = pub.main(["--iface", "lo0", "--seconds", "1"])

    assert code == pub.EXIT_REFUSED
    assert published == [], "a publisher was created although DDS bound en0"


class FakeClock:
    """A clock the test moves forward by hand: sleep(dt) adds dt (+ an optional overshoot; a tuple of overshoots is used in turn,
    as sleep jitter), so a 10 s run finishes instantly."""

    def __init__(self, sleep_overshoot: "float | tuple" = 0.0) -> None:
        self.now = 0.0
        self.overshoots = itertools.cycle(sleep_overshoot if isinstance(sleep_overshoot, tuple) else (sleep_overshoot,))

    def time(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds + next(self.overshoots)


def test_loopback_pub_stops_when_robot_network_appears(pub):
    """Proves: the helper re-checks the robot address while publishing and stops within about a second when it appears.
    Catches: a start-only check that keeps publishing after the robot cable is plugged in mid-run (review r5-2)."""
    clock = FakeClock()
    sent = []
    net_answers = [False, True]   # 1st check at t = 0: no robot; 2nd check at t = 1 s: the cable appeared

    def robot_net_now() -> bool:
        return net_answers.pop(0)

    reason = pub.run_loop(seconds=10.0, hz=200.0, publish_lowstate=lambda: sent.append(clock.now), publish_hands=None,
                          net_check=robot_net_now, clock=clock.time, sleep=clock.sleep)

    assert reason == "robot_network"
    assert max(sent) <= pub.NET_CHECK_S + 0.01, f"published until t = {max(sent):.2f} s after the cable appeared at 1 s"


def test_loopback_pub_keeps_rate_when_publish_is_slow(pub):
    """Proves: with a slow publish (CRC + write 1.5 ms) and sleep overshooting by 0.6-1.8 ms the helper still sends about hz
    LowStates and HAND_HZ hand states per second.
    Catches: a fixed sleep(1/hz) after each publish - measured 112 Hz on the M4 instead of 200 - which a slower laptop pushes
    below 89's 100 Hz loopback limit, a false "CHECK dds: FAIL" on a healthy setup; and a hand schedule restarted from "now"
    (next_hands = now + 1/HAND_HZ), which jitter pushes to every third tick (67 Hz here, 78 Hz measured) instead of 100 Hz."""
    clock = FakeClock(sleep_overshoot=(0.0006, 0.0012, 0.0018))   # time.sleep(5 ms) took 6.19 ms on the M4 (E1 code review
    sent, hands = [], []                                          # round 2); jitter around that mean

    def slow_publish() -> None:
        sent.append(clock.now)
        clock.now += 0.0015                     # one CRC + write took about 1.33 ms on the M4 (same measurement)

    pub.run_loop(seconds=10.0, hz=200.0, publish_lowstate=slow_publish, publish_hands=lambda: hands.append(clock.now),
                 net_check=lambda: False, clock=clock.time, sleep=clock.sleep)

    rate, hand_rate = len(sent) / 10.0, len(hands) / 10.0
    assert rate >= 0.95 * 200.0, f"only {rate:.0f} Hz published, asked for 200 Hz"
    assert hand_rate >= 0.95 * pub.HAND_HZ, f"only {hand_rate:.0f} Hz hand state published, asked for {pub.HAND_HZ:.0f} Hz"


def test_selftest_policy_fails_without_backbone_in_prefix(tmp_path, monkeypatch):
    """Proves: 89's policy check FAILs before starting 81 when the prefix's HF cache lacks SmolVLM2, naming the setup command.
    Catches: a self-test that passes on Finnegan's Mac from his old cache, or fails with a vague "81 exit 1" that sends the
    operators to "drop this laptop" instead of re-running setup (R3-1)."""
    pytest.importorskip("unitree_sdk2py")
    pytest.importorskip("cv2")
    from g1sim import real_io
    monkeypatch.setattr(real_io, "install_publisher_guard", lambda: None)   # keep the guard out of this test process
    spec = importlib.util.spec_from_file_location("robot_day_selftest", SELFTEST)
    selftest = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(selftest)
    (tmp_path / "policy-env" / "bin").mkdir(parents=True)
    (tmp_path / "policy-env" / "bin" / "python").write_text("")       # policy env stub
    (tmp_path / selftest.CHECKPOINT).mkdir(parents=True)                # model present
    (tmp_path / selftest.HF_CACHE).mkdir(parents=True)                  # cache folder present, backbone missing
    started = []

    def fake_run(command, **kwargs) -> subprocess.CompletedProcess:
        started.append(command)
        return subprocess.CompletedProcess(command, returncode=1)   # what 81 does when the backbone is not in the cache

    monkeypatch.setattr(selftest.subprocess, "run", fake_run)
    monkeypatch.setattr(selftest, "REPO", tmp_path)                     # 81's log file goes to tmp_path, not the repo's runs/

    status, detail = selftest.check_policy("cpu", tmp_path)

    assert status == "FAIL"
    assert "setup_robot_laptop.sh --policy-env" in detail, detail
    assert started == [], "81 was started although the backbone is missing from the prefix"


def test_81_accepts_cuda_device(tmp_path):
    """Proves: 81 accepts --device cuda and goes on to its checkpoint check (the Ubuntu + NVIDIA policy laptop, ruling R11).
    Catches: an argparse that still says "invalid choice: 'cuda'", so 81 never starts on a Linux GPU laptop."""
    proc = subprocess.run([sys.executable, str(REPO / "scripts" / "81_smolvla_server.py"), "--checkpoint",
                           str(tmp_path / "missing"), "--device", "cuda"], cwd=REPO, capture_output=True, text=True, timeout=60)

    assert "invalid choice" not in proc.stderr, proc.stderr
    assert "checkpoint not found" in proc.stderr, proc.stderr
