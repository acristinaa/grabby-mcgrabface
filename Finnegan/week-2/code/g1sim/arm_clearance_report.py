"""87's report summary and the merge of partial runs: rows -> the numbers runbook Part C's M0 lookup is built from. Pure.

Why it exists: the operators never read 87's rows, only one lookup line per factory elbow (rule 16). Computing that summary
from the rows alone (no MuJoCo) lets `87 --merge` rebuild it from runs split by waist limit or setback (the runtime fallback,
PLAN.md 3.5) and lets a test check the merge without a sim. Keys of setbacks and waist limits are numbers as text ("0.15").
"""
from g1sim.arm_clearance import (BLEND_NOTE, CLEARANCE_MIN_M, GRID_BOUNDS, MAX_SAMPLE_STEP_M, OBJECTS, WAIST_LIMITS)

TRAINING_REACH_MAX_M = 0.52     # the ball must stay within 0.52 m of the pelvis, else it is out of the policy's reach
                                # (table_scene.SCENES["volleyball"]["reach_m"], R2-3)
MERGE_MUST_MATCH = ("CLEARANCE_MIN_M", "MAX_SAMPLE_STEP_M", "GRID_BOUNDS", "elbows", "shoulder_pitches", "demo", "arms_up_first")


def key(value: float) -> str:
    """Report key of a setback or waist limit: the number as text, 2 decimals."""
    return f"{value:.2f}"


def _clear_for_every_pitch(rows: list, elbow: float, setback: float, pitches: list) -> bool:
    found = [r for r in rows if r["elbow"] == elbow and r["setback_m"] == setback]
    return len(found) == len(pitches) and all(r["min_clearance_m"] >= CLEARANCE_MIN_M for r in found)


def setback_needed(ready_rows: list, elbows: list, pitches: list, setbacks: list) -> dict:
    """{elbow key: smallest setback with >= 5 cm on the ready path for every shoulder pitch, or None}."""
    return {f"{e:.1f}": next((s for s in sorted(setbacks) if _clear_for_every_pitch(ready_rows, e, s, pitches)), None)
            for e in elbows}


def max_elbow_clear_at_nominal(ready_rows: list, elbows: list, pitches: list) -> float | None:
    """Largest elbow whose ready paths keep >= 5 cm at setback 0 for every shoulder pitch, or None."""
    clear = [e for e in elbows if _clear_for_every_pitch(ready_rows, e, 0.0, pitches)]
    return max(clear) if clear else None


def _largest(flags: dict, ok) -> float | None:
    passing = [float(k) for k, v in flags.items() if ok(v)]
    return max(passing) if passing else None


def table_steps(setback: float | None, keep_layout_max: float | None) -> str:
    """"nominal" | "setback" | "skip": where the table goes for the M4/M6 steps of one factory elbow (rule 16 (c))."""
    if setback is None:
        return "skip"
    if setback == 0.0:
        return "nominal"
    return "setback" if keep_layout_max is not None and setback <= keep_layout_max else "skip"


def _exit_tables(policy_rows: list) -> tuple:
    """(replay_exits_clear, frames_inside), both [setback key][waist key], from the to_ready rows (one per start pose)."""
    clear, inside = {}, {}
    for row in policy_rows:
        if row["exit"] != "to_ready":
            continue
        s, w = key(row["setback_m"]), key(row["waist_limit"])
        clear.setdefault(s, {}).setdefault(w, True)
        clear[s][w] = clear[s][w] and row["passed"]
        counts = inside.setdefault(s, {}).setdefault(w, {name: 0 for name in OBJECTS})
        for name in OBJECTS:
            counts[name] += row["start_clearance_m"][name] < 0
    return clear, inside


def _fault_exits_clear(policy_rows: list, elbow: float, setback: float | None, waist_limits: list) -> dict:
    """{waist key: every to_hanging exit for this elbow (all pitches, worst ordering) passed at this setback}."""
    result = {}
    for limit in waist_limits:
        rows = [r for r in policy_rows if r["exit"] == "to_hanging" and r["elbow"] == elbow
                and r["setback_m"] == setback and r["waist_limit"] == limit]
        result[key(limit)] = bool(rows) and all(r["passed"] for r in rows)
    return result


def _exit_verdict(rows: list) -> dict:
    """{clear: every row passed, min_m: smallest clearance to any object along any of the rows}."""
    return {"clear": bool(rows) and all(r["passed"] for r in rows),
            "min_m": min(min(r["min_clearance_m"].values()) for r in rows) if rows else None}


def _demo_path(to_ready_rows: list) -> dict:
    """{object: {min_m, frame}}: the closest the demo path itself (start poses, 2 cm apart) comes to each object."""
    result = {}
    for name in OBJECTS:
        closest = min(to_ready_rows, key=lambda r: r["start_clearance_m"][name])
        result[name] = {"min_m": closest["start_clearance_m"][name], "frame": closest["frame"]}
    return result


def arms_up_first_summary(ready_rows: list, policy_rows: list, settings: dict) -> dict:
    """The arms-up-first variant (87 --arms-up-first) at the nominal table, per exit:
    waists[L] = the demo path (ready -> grasp -> basket) and the soft exit's first leg (to the ready pose);
    elbows[e] = ready -> factory pose (the soft exit's second leg, and `86 release` from the ready pose: block 1 at setback 0,
    worst shoulder pitch) and per L the fault exits (worst ordering) and `86 release` from every demo pose."""
    nominal = [r for r in policy_rows if r["setback_m"] == 0.0]
    waists = {}
    for limit in settings["waist_limits"]:
        to_ready = [r for r in nominal if r["exit"] == "to_ready" and r["waist_limit"] == limit]
        inside = {name: sum(r["start_clearance_m"][name] < 0 for r in to_ready) for name in OBJECTS}
        waists[key(limit)] = {"demo_path": _demo_path(to_ready), "frames_inside": inside, "to_ready": _exit_verdict(to_ready)}
    elbows = []
    for e in settings["elbows"]:
        ready = [r["min_clearance_m"] for r in ready_rows if r["elbow"] == e and r["setback_m"] == 0.0]
        per_waist = {key(limit): {kind: _exit_verdict([r for r in nominal if r["exit"] == exit_name and r["elbow"] == e
                                                       and r["waist_limit"] == limit])
                                  for kind, exit_name in (("fault", "to_hanging"), ("release", "release"))}
                     for limit in settings["waist_limits"]}
        elbows.append({"elbow": e, "ready_to_factory": {"clear": min(ready) >= CLEARANCE_MIN_M, "min_m": min(ready)},
                       "waists": per_waist})
    return {"waists": waists, "elbows": elbows}


def summarize(ready_rows: list, policy_rows: list | None, settings: dict, stats: dict) -> dict:
    """The report summary (PLAN.md 3.5) from the rows, the grid settings and the sampling stats."""
    elbows, pitches = settings["elbows"], settings["shoulder_pitches"]
    needed = setback_needed(ready_rows, elbows, pitches, settings["setbacks"])
    keep_max = _largest(settings["stand_on_table"], bool)
    policy_max = _largest(settings["ball_distance_m"], lambda d: d <= TRAINING_REACH_MAX_M)
    clear, inside = _exit_tables(policy_rows or [])
    elbow_rows = []
    for e in elbows:
        setback = needed[f"{e:.1f}"]
        elbow_rows.append({"elbow": e, "table_steps": table_steps(setback, keep_max), "setback_m": setback,
                           "fault_exits_clear": _fault_exits_clear(policy_rows or [], e, setback, settings["waist_limits"])})
    summary = {"CLEARANCE_MIN_M": CLEARANCE_MIN_M, "MAX_SAMPLE_STEP_M": MAX_SAMPLE_STEP_M,
               "max_sample_step_m": stats["max_sample_step_m"], "max_start_step_m": stats["max_start_step_m"],
               "samples": stats["samples"], "start_poses": stats["start_poses"], "GRID_BOUNDS": GRID_BOUNDS,
               "WAIST_LIMITS": list(settings["waist_limits"]) or list(WAIST_LIMITS), "BLEND_NOTE": BLEND_NOTE,
               "max_elbow_clear_at_nominal": max_elbow_clear_at_nominal(ready_rows, elbows, pitches),
               "setback_needed_m": needed, "setback_max_keep_layout_m": keep_max, "setback_max_policy_m": policy_max,
               "replay_exits_clear": clear, "frames_inside": inside, "elbows": elbow_rows}
    if settings.get("arms_up_first") and policy_rows:
        summary["arms_up_first"] = arms_up_first_summary(ready_rows, policy_rows, settings)
    return summary


def merge_reports(parts: list) -> dict:
    """One report from partial 87 runs (split by waist limit or setback). Refuses parts with different settings that must
    match, because their rows would not describe the same check; the summary is recomputed from the merged rows."""
    first = parts[0]["settings"]
    for part in parts[1:]:
        for name in MERGE_MUST_MATCH:
            if part["settings"].get(name) != first.get(name):
                raise ValueError(f"87 --merge: parts differ in {name} ({first.get(name)} vs {part['settings'].get(name)}) - "
                                 "re-run the parts with the same settings")
    settings = dict(first)
    for name in ("setbacks", "waist_limits"):
        settings[name] = sorted({v for part in parts for v in part["settings"][name]})
    for name in ("stand_on_table", "ball_distance_m"):
        settings[name] = {k: v for part in parts for k, v in part["settings"][name].items()}
    ready_rows = _unique(row for part in parts for row in part["ready_paths"]["rows"])
    with_policy = [part for part in parts if part["policy_exits"] is not None]
    policy_rows = _unique(row for part in with_policy for row in part["policy_exits"]["rows"]) if with_policy else None
    stats = {"samples": {}, "start_poses": {}, "max_sample_step_m": 0.0, "max_start_step_m": 0.0, "elapsed_s": 0.0}
    for part in parts:
        for block, n in part["stats"]["samples"].items():
            stats["samples"][block] = stats["samples"].get(block, 0) + n
        stats["start_poses"].update(part["stats"]["start_poses"])
        for name in ("max_sample_step_m", "max_start_step_m"):
            stats[name] = max(stats[name], part["stats"][name])
        stats["elapsed_s"] += part["stats"]["elapsed_s"]
    return {"settings": settings, "stats": stats, "ready_paths": {"rows": ready_rows},
            "policy_exits": None if policy_rows is None else {"rows": policy_rows},
            "summary": summarize(ready_rows, policy_rows, settings, stats)}


def _unique(rows) -> list:
    """Rows without exact duplicates (two parts may both have computed the setback-0 rows)."""
    seen, result = set(), []
    for row in rows:
        marker = repr(sorted(row.items()))
        if marker not in seen:
            seen.add(marker)
            result.append(row)
    return result
