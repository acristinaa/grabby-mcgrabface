"""Clearance of the sim teacher: how close every arm part from the elbow down passes the table, the ball stand and the basket.

Why it exists: the VLA student copies the teacher's path with a closed-loop error of ~1-1.5 cm (run1 held-out eval,
.planning/level1-harden/FINGER-PLAN.md section 2). A teacher that never touches but passes 0.7 cm above the table teaches
a path the student then hits. Contact checks only see touches, not near misses, so this meter measures the distance itself.
Per control tick (call ClearanceMeter.update from the teacher's on_tick): mujoco.mj_geomDistance from every collision geom
of both elbow subtrees (elbow, wrists, palm, fingers) to every geom of the forbidden bodies (ball_grasp.FORBIDDEN).
Legs, torso and head are left out on purpose: the knee stands 4.1 cm from the table by design of the stop radius
(g1sim/navigation.py), which is not a reach problem. Used by 67 --clearance.
"""
import collections

import mujoco
import numpy as np

from g1sim import ball_grasp

# m: mj_geomDistance reports this when two geoms are farther apart; the probe range of FINGER-PLAN.md 1.2
DISTMAX_M = 0.15
SIDES = ("left", "right")
TOP_CLOSEST = 5  # how many "where was it closest" places summarize() lists


def elbow_side(model, body):
    """"left" / "right" if body is in the subtree of that elbow link, else None (walks up model.body_parentid)."""
    roots = {model.body(f"{side}_elbow_link").id: side for side in SIDES}
    while body != 0:
        if body in roots:
            return roots[body]
        body = model.body_parentid[body]
    return None


def arm_parts(model):
    """[(geom, side, part)] for every collision geom from the elbows down. part = ball_grasp.finger_of() for hand bodies
    (palm, thumb, index, middle, ring, pinky), "arm" for the elbow and wrist links. Collision geom = contype or conaffinity,
    as TableEnv.robot_geoms."""
    parts = []
    for geom in range(model.ngeom):
        body = model.geom_bodyid[geom]
        side = elbow_side(model, body)
        if side and (model.geom_contype[geom] or model.geom_conaffinity[geom]):
            parts.append((geom, side, ball_grasp.finger_of(model.body(body).name) or "arm"))
    return parts


def forbidden_geoms(model):
    """{body name: [collision geoms]} for every body of ball_grasp.FORBIDDEN that this scene has (the walk world has no
    distractors). Collision geoms only: the table's visual slab and legs (table_scene.TABLE_STYLES) cannot be touched."""
    names = {model.body(b).name for b in range(model.nbody)}
    return {name: [g for g in range(model.ngeom) if model.geom_bodyid[g] == model.body(name).id
                   and (model.geom_contype[g] or model.geom_conaffinity[g])]
            for name in ball_grasp.FORBIDDEN if name in names}


class ClearanceMeter:
    """Smallest distance per (side, part, forbidden body) over one teacher run, plus the single closest moment."""

    def __init__(self, env):
        self.model, self.data = env.model, env.data
        self.parts = arm_parts(self.model)
        self.targets = forbidden_geoms(self.model)
        self.min_m = {}           # "side|part|body" -> m
        self.min_by_phase_m = {}  # phase -> m
        self.closest = None

    def update(self, phase):
        """Measure the current sim state; call after every control tick."""
        for geom, side, part in self.parts:
            for body, geoms in self.targets.items():
                d = min(mujoco.mj_geomDistance(self.model, self.data, geom, g, DISTMAX_M, None) for g in geoms)
                self._record(f"{side}|{part}|{body}", d, phase)

    def _record(self, key, d, phase):
        self.min_m[key] = min(d, self.min_m.get(key, DISTMAX_M))
        self.min_by_phase_m[phase] = min(d, self.min_by_phase_m.get(phase, DISTMAX_M))
        if self.closest is None or d < self.closest["d_m"]:
            side, part, body = key.split("|")
            self.closest = {"side": side, "part": part, "body": body, "d_m": d, "phase": phase,
                            "t_s": float(self.data.time)}

    def result(self):
        """The run's "clearance" block (m, rounded to 0.1 mm)."""
        closest = dict(self.closest, d_m=round(self.closest["d_m"], 4), t_s=round(self.closest["t_s"], 3))
        return {"min_m": {k: round(d, 4) for k, d in self.min_m.items()}, "closest": closest,
                "min_by_phase_m": {p: round(d, 4) for p, d in self.min_by_phase_m.items()}}


def summarize(trials):
    """Pooled clearance over the passed trials with a "clearance" block (failed runs are not paths a demo would teach).
    p5 = 5th percentile of the per-trial minimum (numpy default method, as the probe)."""
    blocks = [t["clearance"] for t in trials if t.get("passed") and "clearance" in t]
    if not blocks:
        return {"n": 0}
    closest = [b["closest"]["d_m"] for b in blocks]
    pinky = [min(d for k, d in b["min_m"].items() if k.split("|")[1] == "pinky") for b in blocks]
    places = collections.Counter("|".join(b["closest"][k] for k in ("side", "part", "body", "phase")) for b in blocks)
    return {"n": len(blocks),
            "p5_m": round(float(np.percentile(closest, 5)), 4), "min_m": round(min(closest), 4),
            "pinky_p5_m": round(float(np.percentile(pinky, 5)), 4), "pinky_min_m": round(min(pinky), 4),
            "closest_counts": dict(places.most_common(TOP_CLOSEST))}
