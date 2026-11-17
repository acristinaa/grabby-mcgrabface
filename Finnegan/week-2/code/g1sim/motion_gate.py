"""Real G1: the motion gate - decides whether 86 may send anything at all (PLAN.md 4.3).

Why it exists: on the robot day the operators did not write this code and the author is not there. So 86 only moves the robot
when (a) Finnegan unlocked exactly this code today in docs/ROBOT_DAY_MOTION.json, (b) every repo file 86 loaded is unchanged
since that reviewed commit, (c) both operators confirm by name and (d) the robot hangs in the gantry with the owner at the
remote. The sim bypass skips (a)-(d) only on loopback with --yes-sim and no robot network. This guards against accidents
(a stale checkout, an edited file, a forgotten gantry), not against someone who wants to misuse it (said in the runbook).
stdlib only besides g1sim.real_io (for the robot-network check); runs no DDS.
"""
import datetime
import json
import pathlib
import re
import subprocess
import sys

from g1sim import real_io

MOTION_FILE = "docs/ROBOT_DAY_MOTION.json"          # written only by Finnegan after the final gate (PLAN.md 4.3)
OPERATORS = ("acristinaa", "saadsheikh-tech")       # GitHub names of the two robot-day operators (PLAN.md situation)
UNLOCKED_BY = "finneganadam"                        # the only person who may unlock (PLAN.md 4.3 (a))
START_WORD = "START"                                # typed by the operators to confirm (c)
HANGING_WORD = "HANGING"                            # typed for every motion command to confirm (d)
COMMIT_PATTERN = re.compile(r"^[0-9a-f]{40}$")      # a full git commit id, so no branch name or short id can be pinned
UNLOCK_HINT = "only Finnegan can unlock"


def loaded_repo_files(repo, modules=sys.modules) -> list:
    """Every loaded module file inside the repo, plus the __main__ script, as repo-relative paths (sorted). These are the
    files whose code is running right now, so these are the files that must match the reviewed commit."""
    repo = pathlib.Path(repo).resolve()
    files = set()
    for module in list(modules.values()):
        path = getattr(module, "__file__", None)
        if not path:
            continue
        path = pathlib.Path(path).resolve()
        if repo in path.parents and "site-packages" not in path.parts:
            files.add(path.relative_to(repo))
    return sorted(files)


def gate_problems(repo, loaded_files) -> list:
    """Problems with (a) the unlock file and (b) the loaded files; an empty list means motion is unlocked."""
    repo = pathlib.Path(repo)
    problems, commit = _unlock_problems(repo)
    if commit is None:
        return problems
    for path in loaded_files:
        if not _git_ok(repo, "cat-file", "-e", f"{commit}:{pathlib.Path(path).as_posix()}"):
            problems.append(f"{path} is not in the unlocked commit {commit[:8]} - pull again; if it stays, call Finnegan")
        elif not _git_ok(repo, "diff", "--quiet", commit, "--", str(path)):
            problems.append(f"{path} differs from the unlocked commit {commit[:8]} - never edit files; "
                            f"run `git status` and tell Finnegan")
    return problems


def _unlock_problems(repo: pathlib.Path) -> tuple:
    """(problems, pinned commit or None) for check (a). The unlock is read from the COMMITTED file (`git show HEAD:`),
    never from the working tree: only a commit by Finnegan can unlock, not a hand-written or edited file (R3-3)."""
    if not _git_ok(repo, "cat-file", "-e", f"HEAD:{MOTION_FILE}"):
        if (repo / MOTION_FILE).exists():
            return [f"{MOTION_FILE} is not committed - {UNLOCK_HINT}"], None
        return [f"{MOTION_FILE} not found - {UNLOCK_HINT}"], None
    try:
        unlock = json.loads(_git_output(repo, "show", f"HEAD:{MOTION_FILE}") or "")
    except json.JSONDecodeError as error:
        return [f"{MOTION_FILE} is not readable JSON ({error}) - {UNLOCK_HINT}"], None
    problems = _unlock_content_problems(unlock)
    if not _git_ok(repo, "diff", "--quiet", "HEAD", "--", MOTION_FILE):
        problems.append(f"{MOTION_FILE} was changed after its commit - never edit files; {UNLOCK_HINT}")
    commit = str(unlock.get("commit", ""))
    if not COMMIT_PATTERN.match(commit):
        problems.append(f"{MOTION_FILE} has no full commit id - {UNLOCK_HINT}")
        return problems, None
    if not _git_ok(repo, "merge-base", "--is-ancestor", commit, "HEAD"):
        problems.append(f"the unlocked commit {commit[:8]} is not part of this checkout - run `git pull`")
        return problems, None
    return problems, (None if problems else commit)


def _unlock_content_problems(unlock: dict) -> list:
    problems = []
    if unlock.get("motion_allowed") is not True:
        problems.append(f"{MOTION_FILE} does not allow motion - {UNLOCK_HINT}")
    if unlock.get("unlocked_by") != UNLOCKED_BY:
        problems.append(f"{MOTION_FILE} was not unlocked by {UNLOCKED_BY} - {UNLOCK_HINT}")
    today = datetime.date.today().isoformat()
    if unlock.get("date") != today:  # ruling R13: an unlock never carries over to a later session
        problems.append(f"the unlock is for {unlock.get('date')}; {UNLOCK_HINT} today")
    return problems


def _git_ok(repo: pathlib.Path, *args) -> bool:
    """True when the git command exits 0 (the checks above are all yes/no questions to git)."""
    try:
        return subprocess.run(["git", "-C", str(repo), *args], capture_output=True).returncode == 0
    except OSError:
        return False  # no git at all: every check fails, so motion stays locked


def _git_output(repo: pathlib.Path, *args) -> str | None:
    """stdout of a git command, or None when it fails (then the caller reports the file as unreadable: locked)."""
    try:
        result = subprocess.run(["git", "-C", str(repo), *args], capture_output=True, text=True)
    except OSError:
        return None
    return result.stdout if result.returncode == 0 else None


def confirm_operators(ask=input) -> str | None:
    """Check (c): two distinct operators type their GitHub names, then START. Returns the problem, or None."""
    first = ask(f"Operator 1, type your GitHub name ({' or '.join(OPERATORS)}): ").strip()
    second = ask("Operator 2, type your GitHub name: ").strip()
    if first not in OPERATORS or second not in OPERATORS:
        return f"operator names must be {' and '.join(OPERATORS)}"
    if first == second:
        return "two different operators must confirm"
    if ask(f"Both operators agree to start motion? Type {START_WORD}: ").strip() != START_WORD:
        return f"{START_WORD} was not typed"
    return None


def confirm_hanging(ask=input) -> str | None:
    """Check (d): the robot stands on its own feet in the owner's arm_sdk mode, the harness is on the gantry with only a few
    cm of slack, and the owner holds the remote (PLAN.md 4.3 (d), R2-5). Returns the problem, or None."""
    answer = ask("Robot stands on its feet, harness on the gantry with only a few cm of slack, owner holds the remote, "
                 f"1.5 m clear around the arms? Type {HANGING_WORD}: ").strip()
    return None if answer == HANGING_WORD else f"{HANGING_WORD} was not typed - secure the robot first"


def sim_bypass_problem(iface: str, yes_sim: bool) -> str | None:
    """None when the run may skip checks (a)-(d) because it can only reach the loopback sim; else why it may not."""
    if not yes_sim:
        return "--yes-sim not given"
    if not real_io.is_loopback(iface):
        return f"--yes-sim needs a loopback interface ({' or '.join(real_io.LOOPBACK)}), got {iface}"
    if real_io.robot_net_present():
        return "--yes-sim refused: a 192.168.123.x address is present (robot cable plugged in)"
    return None
