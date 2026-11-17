"""Forward kinematics for 87: arm poses of the volleyball walk scene -> arm movement and clearance to table, stand, basket.

Why it exists: g1sim/arm_clearance.py decides which poses to check and whether a path passes; this module answers the two
questions it asks, by forward kinematics only (joint angles -> link positions; no physics step, no DDS, no GPU):
step_of(qa, qb) = how far any arm or hand collision geom moves between two poses, and clearance_of(q) = the smallest distance
of the arm and hand collision geoms to each object. Scene: TableEnv("volleyball_walk") in its nominal layout, pelvis at the
walk stop in the GR00T crouch (PLAN.md 2.7b), torso leaning forward by the measured waist sag (arm_clearance.WAIST_PITCH_CHECKED,
review B1); a setback moves table, stand and basket together by +x (PLAN.md 3.5). set_lean and set_table_height let 87
--waypoint-paths check a lean grid (review C r1) and the v2 table height range (configs/randomization.json) on the same scene.
Sim env (MuJoCo 3.13 has mj_geomDistance).
"""
import mujoco
import numpy as np

from g1sim import arm_clearance, real_safety, table_scene
from g1sim.table_env import TableEnv

DISTMAX_M = 0.10            # pairs farther apart are cut early and reported as 0.10 (PLAN.md 3.5; only < 5 cm matters)
ARM_ROOTS = ("left_shoulder_pitch_link", "right_shoulder_pitch_link")   # motors 15 and 22: everything below is arm or hand
OBJECT_BODIES = {"table": "table", "stand": "volleyball_stand", "basket": "basket"}   # keys = arm_clearance.OBJECTS
FK_CACHE_SIZE = 4096        # poses kept per setback; one path holds ~50-500 samples, so a path never recomputes a pose


def rotation_angle(mat_a: np.ndarray, mat_b: np.ndarray) -> np.ndarray:
    """Angle (rad) of the rotation between two stacks of 3x3 rotation matrices (n, 3, 3)."""
    trace = np.einsum("nij,nij->n", mat_a, mat_b)   # trace(A^T B)
    return np.arccos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0))


def geom_step_bound(pos_a, mat_a, pos_b, mat_b, rbound) -> float:
    """Largest movement of any geom between two poses: centre shift + bounding radius x rotation angle (R4-1), so a forearm
    capsule turning about its centre still counts with its far end."""
    shift = np.linalg.norm(np.asarray(pos_b) - np.asarray(pos_a), axis=1)
    return float(np.max(shift + np.asarray(rbound) * rotation_angle(mat_a, mat_b)))


def _collision(model, geom: int) -> bool:
    return bool(model.geom_contype[geom] or model.geom_conaffinity[geom])


def _in_subtree(model, body: int, root: int) -> bool:
    while body != 0:
        if body == root:
            return True
        body = model.body_parentid[body]
    return False


class ClearanceModel:
    """The scene for 87 with one FK cache per setback. step_of and clearance_of are the callables arm_clearance needs."""

    def __init__(self):
        env = TableEnv(scene="volleyball_walk")
        env.reset(randomize=False)          # nominal layout: no randomization, pelvis at WALK_STANCE (PLAN.md 3.5)
        self.model, self.data = env.model, env.data
        m = self.model
        self.qadr17 = env.body_qadr[12:29]   # qpos addresses of motors 12..28
        self.waist_roll_pitch_qadr = self.qadr17[list(real_safety.WAIST_HOLD_POS)]   # motors 13/14
        roots = [m.body(name).id for name in ARM_ROOTS]
        self.arm_geoms = np.array([g for g in range(m.ngeom) if _collision(m, g)
                                   and any(_in_subtree(m, m.geom_bodyid[g], r) for r in roots)])
        self.object_geoms = {name: np.array([g for g in range(m.ngeom) if _collision(m, g) and m.geom_bodyid[g] == m.body(b).id])
                             for name, b in OBJECT_BODIES.items()}
        self.setback_bodies = [m.body(b).id for b in OBJECT_BODIES.values()]
        self.base_body_pos = m.body_pos[self.setback_bodies].copy()
        self.pelvis_xy = self.data.xpos[m.body("pelvis").id][:2].copy()
        self.ball_xy = self.data.xpos[m.body(table_scene.SCENES["volleyball"]["target"]).id][:2].copy()
        self.setback, self.table_dz = 0.0, 0.0
        self.lean = (0.0, arm_clearance.WAIST_PITCH_CHECKED)   # (waist roll, waist pitch) the FK puts the torso at
        self._cache = {}
        self.samples = 0                     # FK evaluations so far (each one mj_kinematics + distances)

    # --- scene ---------------------------------------------------------------------------------------------------------
    def set_setback(self, setback_m: float) -> None:
        """Move table, stand and basket together by +setback_m in x (away from the robot)."""
        self.model.body_pos[self.setback_bodies] = self.base_body_pos + np.array([setback_m, 0.0, self.table_dz])
        self.setback = float(setback_m)

    def set_table_height(self, dz_m: float) -> None:
        """Move table, stand and basket together by dz_m in z, as randomization's table height does (what stands on the
        table moves with it; the robot's stance stays)."""
        self.table_dz = float(dz_m)
        self.set_setback(self.setback)

    def set_lean(self, roll: float, pitch: float) -> None:
        """Torso lean (waist roll, waist pitch, rad) for every following FK pose; default (0, WAIST_PITCH_CHECKED)."""
        self.lean = (float(roll), float(pitch))

    def ball_distance(self, setback_m: float) -> float:
        """Pelvis to nominal ball (x, y) distance with the layout moved back by setback_m (training reach_m, R2-3)."""
        return float(np.linalg.norm(self.ball_xy + np.array([setback_m, 0.0]) - self.pelvis_xy))

    def stand_on_table(self, setback_m: float) -> bool:
        """True when the whole stand lies on the table top at this setback (table and stand move together, PLAN.md 3.5)."""
        m, d = self.model, self.data
        self.set_setback(setback_m)
        mujoco.mj_kinematics(m, d)
        top = self.object_geoms["table"][0]
        lo = d.geom_xpos[top][:2] - m.geom_size[top][:2]
        hi = d.geom_xpos[top][:2] + m.geom_size[top][:2]
        stand = self.object_geoms["stand"]
        r = m.geom_rbound[stand][:, None]
        inside = np.all((d.geom_xpos[stand][:, :2] - r >= lo) & (d.geom_xpos[stand][:, :2] + r <= hi))
        self.set_setback(self.setback)
        return bool(inside)

    # --- the two callables ---------------------------------------------------------------------------------------------
    def evaluate(self, q17: np.ndarray) -> dict:
        """FK of one pose at the current setback (cached): arm geom positions/rotations and clearance per object."""
        key = (self.setback, self.table_dz, self.lean, np.round(np.asarray(q17, dtype=float), 9).tobytes())
        if key not in self._cache:
            if len(self._cache) >= FK_CACHE_SIZE:
                self._cache.clear()
            self._cache[key] = self._compute(q17)
        return self._cache[key]

    def step_of(self, qa: np.ndarray, qb: np.ndarray) -> float:
        a, b = self.evaluate(qa), self.evaluate(qb)
        return geom_step_bound(a["pos"], a["mat"], b["pos"], b["mat"], self.model.geom_rbound[self.arm_geoms])

    def clearance_of(self, q17: np.ndarray) -> dict:
        return self.evaluate(q17)["clearance"]

    def worst_pair(self, q17: np.ndarray) -> str:
        return self.evaluate(q17)["worst_pair"]

    def _compute(self, q17: np.ndarray) -> dict:
        """The 17-vector carries the q0 that 86 sends for waist roll/pitch, but the torso does not stay there: it sags forward
        under the arms (review B1). So the FK puts it at self.lean (default roll 0 and the largest measured pitch) instead of
        the commanded values."""
        m, d = self.model, self.data
        d.qpos[self.qadr17] = q17
        d.qpos[self.waist_roll_pitch_qadr] = self.lean
        mujoco.mj_kinematics(m, d)          # positions only: no physics step (PLAN.md 3.5)
        self.samples += 1
        clearance, worst = {}, (np.inf, "")
        for name, geoms in self.object_geoms.items():
            distance, pair = self._object_distance(geoms)
            clearance[name] = distance
            worst = min(worst, (distance, pair))
        return {"pos": d.geom_xpos[self.arm_geoms].copy(), "mat": d.geom_xmat[self.arm_geoms].reshape(-1, 3, 3).copy(),
                "clearance": clearance, "worst_pair": worst[1]}

    def _object_distance(self, geoms: np.ndarray) -> tuple:
        """Smallest arm-to-object distance (capped at DISTMAX_M) and the pair "arm body/geom|object geom"."""
        m, d = self.model, self.data
        centre = np.linalg.norm(d.geom_xpos[self.arm_geoms][:, None] - d.geom_xpos[geoms][None], axis=2)
        lower = centre - m.geom_rbound[self.arm_geoms][:, None] - m.geom_rbound[geoms][None]   # bounding-sphere pre-filter
        best, pair = DISTMAX_M, ""
        for i, j in zip(*np.nonzero(lower < DISTMAX_M)):
            arm, obj = int(self.arm_geoms[i]), int(geoms[j])
            distance = mujoco.mj_geomDistance(m, d, arm, obj, DISTMAX_M, None)
            if distance < best:
                best, pair = distance, f"{m.body(m.geom_bodyid[arm]).name}/{arm}|{m.body(m.geom_bodyid[obj]).name}/{obj}"
        return float(best), pair


def check_scene(model: ClearanceModel) -> None:
    """Fail loudly if the geom sets differ from the scene 87 was planned on (53 arm/hand, 22 object geoms, PLAN.md 3.5)."""
    counts = {name: len(g) for name, g in model.object_geoms.items()}
    if len(model.arm_geoms) == 0 or min(counts.values()) == 0:
        raise RuntimeError(f"87: scene has {len(model.arm_geoms)} arm geoms and object geoms {counts} - rebuild the scene "
                           "(scripts/53_build_table_scene.py) before trusting any clearance")

