"""Unit: the read-only bring-up can never publish (PLAN.md 4.1 rule 1): static ban, import check, start-up guard.

(a) the files Track A/E ship contain no publisher or mode-switch call (ast + text); (b) `84 --dry-run` loads no motion
module; (c) the guard makes creating a writer raise, and 84 installs it before anything else.
"""
import ast
import importlib
import pathlib
import re

import pytest

pytest.importorskip("unitree_sdk2py")
pytest.importorskip("cv2")
from conftest import probe_result, run_python  # noqa: E402
from g1sim import real_io  # noqa: E402

pytestmark = pytest.mark.unit

REPO = pathlib.Path(__file__).resolve().parent.parent
BRINGUP = REPO / "scripts" / "84_real_bringup.py"
SELFTEST = REPO / "scripts" / "89_robot_day_selftest.py"   # Track E1; checked once it exists
READ_ONLY_FILES = [REPO / "g1sim" / name for name in ("real_io.py", "real_factory_pose.py", "real_camera.py", "real_bringup.py")]
READ_ONLY_FILES += [BRINGUP, REPO / "scripts" / "71_real_probe.py", REPO / "tests" / "helpers" / "fake_camera.py",
                    REPO / "tests" / "helpers" / "real_fakes.py"]
NEW_FILES = [p for p in READ_ONLY_FILES if p.name != "71_real_probe.py"]   # Track A's files (71 is older German code)
BANNED = {"ChannelPublisher", "DataWriter", "Publisher", "Damp", "ZeroTorque", "ReleaseMode", "MotionSwitcher",
          "run_g1_server", "LocoClient"}
BANNED_TEXT = re.compile(r"\b(?:" + "|".join(sorted(BANNED)) + r")\s*\(|\.Write\(")
# Every name a DDS writer can be built from (PLAN.md 2.6), written out here so a shortened guard list is caught
WRITER_BINDINGS = (("unitree_sdk2py.core.channel", "ChannelPublisher"), ("unitree_sdk2py.core.channel", "DataWriter"),
                   ("cyclonedds.pub", "DataWriter"))
MOTION_MODULES = ("g1sim.real_send", "g1sim.real_run", "g1sim.motion_gate", "g1sim.brainco_env", "g1sim.teleop", "lerobot")
DRY_RUN = """
import json, runpy, sys
script = sys.argv[1]
sys.argv = [script, "--dry-run"]
try:
    runpy.run_path(script, run_name="__main__")
except SystemExit as stop:
    code = stop.code
from g1sim import real_io
print("RESULT " + json.dumps({"exit": code, "modules": sorted(sys.modules), "guard": real_io.publisher_guard_installed()}))
"""


def banned_uses(path: pathlib.Path) -> list:
    """Every name, attribute, import or call text in the file that could create a writer or switch the robot mode."""
    source = path.read_text()
    found = []
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Name) and node.id in BANNED:
            found.append(f"name {node.id} (line {node.lineno})")
        elif isinstance(node, ast.Attribute) and (node.attr in BANNED or node.attr == "Write"):
            found.append(f"attribute .{node.attr} (line {node.lineno})")
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            found += [f"import {a.name} (line {node.lineno})" for a in node.names if a.name.split(".")[-1] in BANNED]
    found += [f"text {m.group(0)!r}" for m in BANNED_TEXT.finditer(source)]
    return found


@pytest.fixture(scope="module")
def dry_run():
    return probe_result(run_python(["-c", DRY_RUN, BRINGUP], timeout=60))


def test_bringup_files_contain_no_publisher():
    """Proves: no read-only file (real_io, real_camera, 84, 71, 89 and helpers) names a writer, Damp, LocoClient or .Write(.
    Catches: a debug line that publishes to the robot from a script the operators believe is read-only."""
    files = READ_ONLY_FILES + ([SELFTEST] if SELFTEST.exists() else [])

    found = {path.name: banned_uses(path) for path in files}

    assert all(not uses for uses in found.values()), found


def test_bringup_dry_run_imports_no_motion_modules(dry_run):
    """Proves: `84 --dry-run` exits 0 without loading real_send, real_run, motion_gate, brainco_env, teleop or lerobot.
    Catches: a read-only script that pulls in motion code by import, where one wrong call could move the arms."""
    loaded = [m for m in dry_run["modules"] if any(m == bad or m.startswith(bad + ".") for bad in MOTION_MODULES)]

    assert dry_run["exit"] == 0 and loaded == []


def test_bringup_installs_publisher_guard_before_any_step(dry_run):
    """Proves: after 84 has run (even only --dry-run) every writer class is replaced by the guard.
    Catches: a 84 whose main() forgets install_publisher_guard(), leaving the static ban as the only protection."""
    assert dry_run["guard"] is True


def test_publisher_guard_blocks_channel_publisher(monkeypatch):
    """Proves: with the guard installed, ChannelPublisher and both DataWriter bindings raise PublisherBlocked.
    Catches: a guard that patches only one binding - channel.py imports DataWriter by name, so both must be replaced.
    The real classes and ChannelFactoryInitialize are replaced by fakes first, so a broken guard creates a fake writer,
    never a real DDS writer (R3-11)."""
    created = []

    class FakeWriter:
        def __init__(self, *args, **kwargs):
            created.append(args)

    channel = importlib.import_module("unitree_sdk2py.core.channel")
    for module in (channel, real_io):
        monkeypatch.setattr(module, "ChannelFactoryInitialize", lambda *a, **k: pytest.fail("DDS must not start here"))
    for module_name, attribute in WRITER_BINDINGS:   # restored after the test by monkeypatch
        monkeypatch.setattr(importlib.import_module(module_name), attribute, FakeWriter)

    real_io.install_publisher_guard()

    for module_name, attribute in WRITER_BINDINGS:
        with pytest.raises(real_io.PublisherBlocked):
            getattr(importlib.import_module(module_name), attribute)("rt/arm_sdk", None)
    assert created == []


def test_new_files_parse_as_python_3_10():
    """Proves: Track A's files parse as Python 3.10, the robot env's version (cyclonedds 0.10.2 has no newer wheels).
    Catches: 3.11+ syntax that works on this Mac's 3.12 but fails on the operators' laptop on Monday."""
    for path in NEW_FILES:
        ast.parse(path.read_text(), filename=str(path), feature_version=(3, 10))  # raises SyntaxError on newer syntax
