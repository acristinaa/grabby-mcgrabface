"""Unit: lab view files (g1sim/labview.py) and the lab view server (scripts/65_labview.py).

No MuJoCo and no rendering: the Reporter runs without a renderer, the server only reads files.
"""
import http.client
import importlib.util
import json
import pathlib
import threading

import pytest

from g1sim import labview

pytestmark = pytest.mark.unit

REPO = pathlib.Path(__file__).resolve().parent.parent


@pytest.fixture(scope="module")
def server_module():
    spec = importlib.util.spec_from_file_location("labview_server", REPO / "scripts" / "65_labview.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def old_54_report(results):
    """Same shape as a real scripts/54 report (runs/d2_54_seed0.json), shortened."""
    checks = list(labview.REASON_LABELS)[:5]
    rows, failures = [], {}
    for first_failed in results:
        row_checks = {c: first_failed is None or i < checks.index(first_failed) for i, c in enumerate(checks)}
        rows.append({"checks": row_checks, "passed": first_failed is None, "reach_error_m": 0.001, "sim_time_s": 20.5})
        if first_failed:
            failures[first_failed] = failures.get(first_failed, 0) + 1
    passed = sum(r is None for r in results)
    return {"trials": len(results), "passed_trials": passed, "success_rate": passed / len(results), "randomize": True,
            "varied_groups": ["objects", "stand"], "seed": 0, "failures_by_first_failed_check": failures,
            "results": rows, "passed": passed == len(results)}


def test_reporter_without_renderer_writes_status_and_record(tmp_path):
    t = {"sim": 0.0}
    rep = labview.Reporter(script="54", task="grasp_lift", object="bottle", trials=3, argv=["--trials", "3"],
                           sim_time_getter=lambda: t["sim"], root=tmp_path)
    status = json.loads(rep.status_path.read_text())
    assert status["state"] == "running" and status["trials_planned"] == 3 and status["pid"] > 0
    assert [p["key"] for p in status["phase_order"]] == list(labview.PHASE_LABELS)

    outcomes = [(True, None, True), (False, "flasche_in_reichweite", False), (False, "hand_an_greifpose", True)]
    for i, (passed, reason, reachable) in enumerate(outcomes):
        rep.trial_start(i)
        for phase in ("freimachen", "anfahren", "neue_phase"):
            t["sim"] += 0.5
            rep.on_tick(phase)
        status = json.loads(rep.status_path.read_text())
        assert status["trial"] == i and status["phase"] == "neue_phase" and status["phase_label"] == "neue_phase"
        assert status["phase_index"] == len(labview.PHASE_LABELS)  # unknown phase appended at the end, shown raw
        rep.trial_end(passed, reason, {"reachable": reachable})

    status = json.loads(rep.status_path.read_text())
    assert status["counts"] == {"passed": 1, "done": 3, "reachable": 2}
    assert status["last_failure"] == {"trial": 2, "reason": "hand_an_greifpose", "label": "Hand missed the grasp pose"}
    assert not rep.frame_path.exists()

    report = tmp_path / "report.json"
    report.write_text("{}")
    path = rep.finish(report_path=report)
    record = json.loads(path.read_text())
    assert path == tmp_path / "records" / f"{rep.run_id}.json"
    assert rep.run_id.endswith("_54_bottle")
    assert record["schema"] == "g1.run_record/1" and record["state"] == "finished"
    assert record["counts"] == {"passed": 1, "reachable": 2, "attempted": 3}
    assert record["success_rate"] == pytest.approx(1 / 3)
    assert record["reachable_success_rate"] == pytest.approx(1 / 2)
    assert record["failures"] == {"flasche_in_reichweite": 1, "hand_an_greifpose": 1}
    assert [(r["i"], r["passed"], r["reason"]) for r in record["trials"]] == [
        (0, True, None), (1, False, "flasche_in_reichweite"), (2, False, "hand_an_greifpose")]
    assert all({"sim_s", "wall_s"} <= set(r) for r in record["trials"])
    assert record["report"] == str(report.resolve())
    assert record["started_at"][-6] in "+-" and record["finished_at"][-6] in "+-"  # ISO with time zone
    assert json.loads(rep.status_path.read_text())["state"] == "finished"


def test_phase_keys_are_unique_across_tasks():
    """phase_label() takes the first table that knows a key: a shared key would show the wrong task's label (the walk's
    "settle" was shown as "Hold still and let the ball settle", 2026-10-08)."""
    tables = (labview.PHASE_LABELS, labview.PICK_PLACE_PHASE_LABELS, labview.WALK_PHASE_LABELS)
    keys = [k for table in tables for k in table]
    assert len(keys) == len(set(keys))
    for table in tables:
        for key, label in table.items():
            assert labview.phase_label(key) == label


def test_reporter_with_frame_getter_writes_that_image(tmp_path):
    """A run that renders anyway (68: the live view) hands its JPEG over instead of rendering a second time."""
    rep = labview.Reporter(script="68", task="walk_to_table", object="table", trials=1, argv=[],
                           frame_getter=lambda: b"\xff\xd8 jpeg", root=tmp_path)
    rep.trial_start(0)
    rep.on_tick("walk")
    assert rep.frame_path.read_bytes() == b"\xff\xd8 jpeg"
    status = json.loads(rep.status_path.read_text())
    assert status["phase_label"] == labview.WALK_PHASE_LABELS["walk"]
    assert [p["key"] for p in status["phase_order"]] == list(labview.WALK_PHASE_LABELS)
    rep.trial_end(False, "dock_within_tolerance")
    assert json.loads(rep.status_path.read_text())["last_failure"]["label"].startswith("Arrived, but")


def test_labels_unknown_keys_are_shown_raw():
    assert labview.phase_label("schliessen") == "Close fingers"
    assert labview.phase_label("ablegen") == "ablegen"
    assert labview.reason_label("flasche_in_reichweite").startswith("Start out of reach")
    assert labview.reason_label("etwas_neues") == "etwas_neues"


def test_old_54_report_adapter(server_module, tmp_path):
    path = tmp_path / "d2_54_seed0.json"
    path.write_text(json.dumps(old_54_report(
        [None, "flasche_in_reichweite", "flasche_in_reichweite", None, "flasche_5_cm_angehoben_5_s_gehalten"])))
    row = server_module.old_report_row(path)
    assert (row["script"], row["object"], row["task"]) == ("54", "bottle", "grasp_lift")
    assert (row["trials"], row["passed"], row["reachable"]) == (5, 2, 3)
    assert row["success_rate"] == pytest.approx(0.4) and row["reachable_success_rate"] == pytest.approx(2 / 3)
    assert [(r["reason"], r["count"]) for r in row["reasons"]] == [
        ("flasche_in_reichweite", 2), ("flasche_5_cm_angehoben_5_s_gehalten", 1)]
    assert row["reasons"][0]["label"] == labview.reason_label("flasche_in_reichweite")
    assert row["date"][-6] in "+-"

    other = tmp_path / "walk.json"
    other.write_text(json.dumps({"goals": [], "passed": True}))
    assert server_module.old_report_row(other) is None
    broken = tmp_path / "broken.json"
    broken.write_text("{not json")
    assert server_module.old_report_row(broken) is None


@pytest.fixture
def server(server_module, tmp_path):
    root, runs = tmp_path / "labview", tmp_path / "runs"
    runs.mkdir()
    (runs / "old.json").write_text(json.dumps(old_54_report([None, "flasche_in_reichweite"])))
    rep = labview.Reporter(script="54", task="grasp_lift", object="bottle", trials=1, argv=[], root=root)
    rep.trial_start(0)
    rep.trial_end(True, None)
    rep.finish()
    (tmp_path / "x.mjpg").write_text("secret")
    srv = server_module.make_server(port=0, root=root, runs_dir=runs, tries=1)
    thread = threading.Thread(target=srv.serve_forever, daemon=True)
    thread.start()
    yield srv.server_address[1], rep
    srv.shutdown()
    srv.server_close()


def get(port, path):
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    conn.request("GET", path)
    resp = conn.getresponse()
    body = resp.read()
    conn.close()
    return resp.status, resp.getheader("Content-Type"), body


def test_server_endpoints(server):
    port, rep = server
    status, ctype, body = get(port, "/")
    assert status == 200 and ctype.startswith("text/html")
    assert b"Live" in body and b"History" in body and b"Training metrics appear here" in body

    status, ctype, body = get(port, "/api/live")
    assert status == 200 and ctype.startswith("application/json")
    runs = json.loads(body)["runs"]
    assert [r["run_id"] for r in runs] == [rep.run_id] and runs[0]["state"] == "finished"

    status, ctype, body = get(port, "/api/runs")
    assert status == 200 and ctype.startswith("application/json")
    rows = json.loads(body)
    assert {(r["source"], r["trials"], r["passed"], r["reachable"]) for r in rows} == {
        (f"{rep.run_id}.json", 1, 1, 1), ("old.json", 2, 1, 1)}


@pytest.mark.parametrize("path", ["/live/../x.mjpg", "/live/..%2Fx.mjpg", "/live/%2e%2e/x.mjpg", "/live/a/b.mjpg",
                                  "/live/x.jpg"])
def test_server_refuses_path_traversal(server, path):
    port, _ = server
    status, _, body = get(port, path)
    assert status in (400, 404) and b"secret" not in body


def test_server_no_frame_is_404(server):
    port, rep = server
    assert get(port, f"/live/{rep.run_id}.mjpg")[0] == 404
