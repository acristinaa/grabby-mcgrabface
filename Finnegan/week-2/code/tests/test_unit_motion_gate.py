"""Unit: the motion gate of 86 (g1sim/motion_gate.py, PLAN.md 4.3) against a throw-away git repo in a temp folder.

The fixture builds a tiny repo with one "loaded" code file, commits it, and (when asked) pins that commit in
docs/ROBOT_DAY_MOTION.json the way Finnegan's unlock recipe does. The last tests drive 86's own gate call site
(scripts/86_real_run.py main) with the gate, the prompts and the robot connection faked. No DDS, no robot.
"""
import datetime
import importlib.util
import json
import pathlib
import subprocess
import types

import pytest

pytest.importorskip("unitree_sdk2py")

from g1sim import motion_gate, real_io, real_safety as rs  # noqa: E402

pytestmark = pytest.mark.unit

CODE = pathlib.Path("g1sim/real_run.py")      # stands in for a file 86 loaded
SCRIPT_86 = pathlib.Path(__file__).resolve().parent.parent / "scripts" / "86_real_run.py"
BOTH_OPERATORS = ("acristinaa", "saadsheikh-tech", "START")


def git(repo, *args) -> str:
    return subprocess.run(["git", "-C", str(repo), "-c", "user.name=test", "-c", "user.email=test@example.com", *args],
                          capture_output=True, text=True, check=True).stdout.strip()


def write_unlock(repo, pinned, **changes) -> None:
    """Write Finnegan's unlock for commit `pinned`, today; `changes` replace single fields (e.g. unlocked_by)."""
    unlock = {"motion_allowed": True, "commit": pinned, "unlocked_by": "finneganadam",
              "date": datetime.date.today().isoformat(), **changes}
    (repo / motion_gate.MOTION_FILE).write_text(json.dumps(unlock) + "\n")


@pytest.fixture
def repo(tmp_path):
    """A repo with one committed code file; returns (path, commit of that file)."""
    git(tmp_path, "init", "-q")
    (tmp_path / "g1sim").mkdir()
    (tmp_path / "docs").mkdir()
    (tmp_path / CODE).write_text("SPEED = 0.3\n")
    git(tmp_path, "add", ".")
    git(tmp_path, "commit", "-q", "-m", "motion code")
    return tmp_path, git(tmp_path, "rev-parse", "HEAD")


def unlocked(repo_and_commit, **changes):
    """Pin the commit and commit the unlock file (Finnegan's recipe, PLAN.md 4.3)."""
    path, commit = repo_and_commit
    write_unlock(path, commit, **changes)
    git(path, "add", motion_gate.MOTION_FILE)
    git(path, "commit", "-q", "-m", "unlock")
    return path


def answers(*typed):
    """ask() that types the given answers in order."""
    queue = list(typed)
    return lambda prompt: queue.pop(0)


def test_gate_locked_without_motion_json(repo):
    """Proves: without docs/ROBOT_DAY_MOTION.json the gate is locked with the message the runbook shows.
    Catches: a gate that treats a missing unlock file as "no problem" and lets motion run."""
    path, _ = repo

    problems = motion_gate.gate_problems(path, [CODE])

    assert problems == ["docs/ROBOT_DAY_MOTION.json not found - only Finnegan can unlock"]


def test_gate_unlocked_for_pinned_unchanged_code(repo):
    """Proves: a correct unlock for today over unchanged code gives no problem (the gate is not simply always locked).
    Catches: a gate that can never open, so Monday's reviewed motion code could not run at all."""
    path = unlocked(repo)

    assert motion_gate.gate_problems(path, [CODE]) == []


def test_gate_refuses_file_changed_since_pinned_commit(repo):
    """Proves: a loaded file edited after the pinned commit locks the gate and names the file.
    Catches: unreviewed code (an operator's edit, a stale checkout) moving the robot [mutation]."""
    path = unlocked(repo)
    (path / CODE).write_text("SPEED = 3.0\n")

    problems = motion_gate.gate_problems(path, [CODE])

    assert len(problems) == 1 and str(CODE) in problems[0] and "differs" in problems[0]


def test_gate_refuses_file_missing_from_pinned_commit(repo):
    """Proves: a loaded file that is not in the pinned commit locks the gate (git diff alone cannot see it, lens 4).
    Catches: a new module added after the review that runs without ever being reviewed."""
    path = unlocked(repo)
    extra = pathlib.Path("g1sim/new_helper.py")
    (path / extra).write_text("X = 1\n")

    problems = motion_gate.gate_problems(path, [CODE, extra])

    assert len(problems) == 1 and "not in the unlocked commit" in problems[0]


def test_loaded_repo_files_lists_exactly_the_repo_code_and_the_86_script(tmp_path):
    """Proves: gate (b)'s file list holds the in-repo g1sim module and the __main__ script 86, and nothing from
    site-packages (even a venv inside the repo), outside the repo or without a file.
    Catches: a list that comes back empty or drops scripts/86_real_run.py, so gate (b) would check nothing and an edited
    or stale file could move the robot (B2 review r3 #1) [mutation, two: return []; keep only g1sim/]."""
    modules = {
        "g1sim.real_run": types.SimpleNamespace(__file__=str(tmp_path / "g1sim" / "real_run.py")),
        "__main__": types.SimpleNamespace(__file__=str(tmp_path / "scripts" / "86_real_run.py")),
        "numpy": types.SimpleNamespace(__file__=str(tmp_path / ".venv/lib/python3.10/site-packages/numpy/__init__.py")),
        "json": types.SimpleNamespace(__file__=str(tmp_path.parent / "python3.10" / "json" / "__init__.py")),
        "sys": types.SimpleNamespace(),                    # built-in modules have no __file__
    }

    files = motion_gate.loaded_repo_files(tmp_path, modules)

    assert files == [pathlib.Path("g1sim/real_run.py"), pathlib.Path("scripts/86_real_run.py")]


def test_gate_refuses_untracked_motion_json(repo):
    """Proves: a hand-made, untracked unlock file with commit = HEAD and today's date locks the gate as "not committed".
    Catches: the "one helpful edit" unlock - someone other than Finnegan writing the JSON by hand (R3-3) [mutation]."""
    path, commit = repo
    write_unlock(path, commit)

    problems = motion_gate.gate_problems(path, [CODE])

    assert problems == ["docs/ROBOT_DAY_MOTION.json is not committed - only Finnegan can unlock"]


def test_gate_reads_motion_json_from_head_not_worktree(repo):
    """Proves: the gate judges the committed unlock (motion_allowed false) and flags the working copy edited to true.
    Catches: a gate that reads the working-tree file, so editing one word would unlock motion (R3-3) [mutation, two:
    read the working tree; drop the "changed after its commit" check]."""
    _, commit = repo
    path = unlocked(repo, motion_allowed=False)
    write_unlock(path, commit, motion_allowed=True)        # the "one helpful edit" in the working copy

    problems = motion_gate.gate_problems(path, [CODE])

    assert any("does not allow motion" in p for p in problems)
    assert any("was changed after its commit" in p for p in problems)


def test_gate_locked_when_unlock_date_is_not_today(repo):
    """Proves: an unlock dated yesterday locks the gate with "the unlock is for <date>" (ruling R13).
    Catches: an unlock that silently carries over into a later session without a review [mutation]."""
    yesterday = (datetime.date.today() - datetime.timedelta(days=1)).isoformat()
    path = unlocked(repo, date=yesterday)

    problems = motion_gate.gate_problems(path, [CODE])

    assert problems == [f"the unlock is for {yesterday}; only Finnegan can unlock today"]


def test_gate_needs_two_distinct_operators():
    """Proves: the same operator twice is refused; the two robot-day operators plus START pass.
    Catches: one person confirming alone (the plan needs two people, one with the hand on Ctrl+C)."""
    same = motion_gate.confirm_operators(answers("acristinaa", "acristinaa", "START"))
    both = motion_gate.confirm_operators(answers("acristinaa", "saadsheikh-tech", "START"))

    assert same == "two different operators must confirm"
    assert both is None


def test_gate_needs_hanging_confirmation():
    """Proves: only the exact word HANGING confirms that the robot is secured; anything else is a refusal.
    Catches: a motion step started with a "y" typed by habit while the robot is not in the gantry."""
    assert motion_gate.confirm_hanging(answers("yes")) is not None
    assert motion_gate.confirm_hanging(answers("HANGING")) is None


def test_sim_bypass_refused_off_loopback(monkeypatch):
    """Proves: --yes-sim with the robot interface (en5) is refused even when no robot address is seen.
    Catches: a sim bypass that skips the unlock, both names and HANGING on a real robot interface [mutation]."""
    monkeypatch.setattr(real_io, "robot_net_present", lambda: False)

    problem = motion_gate.sim_bypass_problem("en5", yes_sim=True)

    assert problem is not None and "loopback" in problem


def test_sim_bypass_refused_with_robot_network_present(monkeypatch):
    """Proves: --yes-sim on loopback is refused while a 192.168.123.x address exists.
    Catches: a "sim" run that skips the gate while the robot cable is plugged in [mutation]."""
    monkeypatch.setattr(real_io, "robot_net_present", lambda: True)

    problem = motion_gate.sim_bypass_problem("lo0", yes_sim=True)

    assert problem is not None and "192.168.123" in problem


@pytest.mark.parametrize("changes, expected", [
    pytest.param(lambda commit: {"unlocked_by": "acristinaa"}, "was not unlocked by finneganadam", id="unlocked_by_other"),
    pytest.param(lambda commit: {"commit": commit[:12]}, "has no full commit id", id="short_commit_id"),
    pytest.param(lambda commit: {"commit": "0" * 40}, "is not part of this checkout", id="commit_not_in_checkout"),
])
def test_gate_refuses_wrong_unlock_content(repo, changes, expected):
    """Proves: an unlock by someone else, with a short commit id, or pinning a commit outside this checkout locks the gate.
    Catches: a gate that skips one of these content checks, so a branch name, a short id or a stranger's unlock could
    pin unreviewed code [mutation, one per case]."""
    path = unlocked(repo, **changes(repo[1]))

    problems = motion_gate.gate_problems(path, [CODE])

    assert any(expected in p for p in problems), problems


@pytest.mark.parametrize("typed, expected", [
    pytest.param(("acristinaa", "a-visitor", "START"), "operator names must be", id="unknown_name"),
    pytest.param(("acristinaa", "saadsheikh-tech", "start"), "START was not typed", id="no_exact_start"),
])
def test_gate_refuses_unknown_operator_or_missing_start(typed, expected):
    """Proves: a name outside the two robot-day operators, or anything but the exact word START, is a refusal.
    Catches: a confirmation that any two names or a lazy "start" can pass [mutation, one per case]."""
    problem = motion_gate.confirm_operators(answers(*typed))

    assert problem is not None and problem.startswith(expected)


# --- 86's own gate call site (scripts/86_real_run.py) ---------------------------------------------------------------

@pytest.fixture
def script_86(tmp_path, monkeypatch):
    """86 loaded as a module, its logs in tmp_path; returns (module, list of build_io calls). build_io is the first step
    that touches the robot network, so a call there means the gate let the run through."""
    spec = importlib.util.spec_from_file_location("script_86_real_run", SCRIPT_86)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(real_io, "RUN_LOG_DIR", tmp_path)
    reached_robot = []

    def fake_build_io(args, signals):
        reached_robot.append(args.mode)
        raise OSError("fake: the test stops here, no robot")
    monkeypatch.setattr(module, "build_io", fake_build_io)
    return module, reached_robot


def test_86_locked_gate_refuses_before_any_robot_connection(script_86, monkeypatch):
    """Proves: with the gate LOCKED, `86 ready --iface en5` exits 2 without asking anyone and without touching the robot.
    Catches: an 86 call site that ignores gate_problems, so motion runs without Finnegan's unlock [mutation]."""
    module, reached_robot = script_86
    monkeypatch.setattr(motion_gate, "gate_problems", lambda repo, loaded: ["docs/ROBOT_DAY_MOTION.json not found"])
    monkeypatch.setattr("builtins.input", lambda prompt: pytest.fail("86 asked a question although the gate is locked"))

    code = module.main(["ready", "--iface", "en5"])

    assert code == rs.EXIT_CODES["refused"] and reached_robot == []


@pytest.mark.parametrize("typed, reaches_robot", [
    # same name twice, then HANGING at once: only the name check can stop this run
    pytest.param(("acristinaa", "acristinaa", "HANGING"), False, id="same_operator_twice"),
    pytest.param((*BOTH_OPERATORS, "yes"), False, id="yes_instead_of_HANGING"),
    pytest.param((*BOTH_OPERATORS, "HANGING"), True, id="all_confirmed"),
])
def test_86_unlocked_gate_needs_two_operators_and_hanging(script_86, monkeypatch, typed, reaches_robot):
    """Proves: with the gate UNLOCKED, 86 reaches the robot only after two different operators, START and HANGING.
    Catches: an 86 call site that skips the operator check or never asks HANGING, so a robot that is not secured in
    the gantry moves [mutation, two: drop each check]."""
    module, reached_robot = script_86
    queue = list(typed)
    monkeypatch.setattr(motion_gate, "gate_problems", lambda repo, loaded: [])
    monkeypatch.setattr("builtins.input", lambda prompt: queue.pop(0))

    code = module.main(["ready", "--iface", "en5"])

    assert code == rs.EXIT_CODES["refused"]
    assert reached_robot == (["ready"] if reaches_robot else [])


def test_86_yes_sim_on_robot_interface_refused(script_86, monkeypatch):
    """Proves: `86 ready --iface en5 --yes-sim` exits 2 without touching the robot.
    Catches: an 86 call site that drops sim_bypass_problem, so --yes-sim would skip the unlock, both names and HANGING
    on the real robot [mutation]."""
    module, reached_robot = script_86
    monkeypatch.setattr(real_io, "robot_net_present", lambda: True)

    code = module.main(["ready", "--iface", "en5", "--yes-sim"])

    assert code == rs.EXIT_CODES["refused"] and reached_robot == []
