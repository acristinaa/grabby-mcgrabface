"""Virtual fence of the policy phase: how close the arms would come to table, stand, basket and the robot's own body.

Why it exists: in the policy phase (86 replay/run, 82 --fence) nothing else keeps a learned arm away from the table, the stand,
the basket or the robot itself; the lead monitor only fires after a contact has blocked the arm for 0.26 s (FENCE-SMOOTH.md 0).
Before a command is sent, the fence puts the robot model into that pose (forward kinematics: joint angles -> link positions,
no physics step) and measures the smallest distance of the moving parts to each obstacle group. The SafetyFilter
(g1sim/real_safety.py) then holds the last command when the new one comes below a group's margin AND closer than holding
still ("never closer than now", FENCE-SMOOTH.md 1.5).
Two poses are checked and the smaller distance counts: `cmd` (the command) and `pred` (the command shifted by the arm's
present tracking error: the PD arm sags, so the real arm sits below its command). Hands always at their commanded target
(BrainCo fingers lag and then arrive). Near an obstacle the pose halfway from the last command is checked too (review N1).
Two ways to build it: Fence.from_env(env) in the sim (82: the eval env's own model and layout) and Fence.nominal(layout) on
the robot laptop (86: the collision-hull model file g1sim/data/fence_model.xml.gz + the layout the operators measured).
numpy + mujoco (pinned MUJOCO_VERSION); importable without mujoco, so 86 can refuse with a clear line instead of crashing.
"""
import collections
import gzip
import pathlib

import numpy as np

from g1sim import brainco

try:
    import mujoco
except ImportError:          # robot env before mujoco was pinned: building a Fence refuses (fail closed), importing does not
    mujoco = None

MUJOCO_VERSION = "3.13.0"    # mj_geomDistance results depend on the version; the model file was built and checked with it
MODEL_FILE = pathlib.Path(__file__).resolve().parent / "data" / "fence_model.xml.gz"
DATA_FILES = ("g1sim/data/fence_model.xml.gz",)   # repo files the motion gate checks besides the modules (motion_gate.py)
DISTMAX_M = 0.10             # distances are capped here (arm_clearance_fk.DISTMAX_M): only a few cm matter

# Margins (m) per obstacle group. Rule (FENCE-SMOOTH.md 1.2): the shipped model's training demos' p5 minus 0.5 cm, at most
# 2.0 cm, at least 0.5 cm, rounded to 1 mm (review N6: basket 0.88 -> 0.9). v1 demos p5: table 1.82, stand 2.98, basket 1.38 cm.
MARGIN_M = {"table": 0.013, "stand": 0.020, "basket": 0.009}   # ruling pending (Finnegan) R-F1
SELF_MARGIN_M = 0.020        # ruling pending (Finnegan) R-F1: torso, head, pelvis/legs, other arm (demos keep >= 4.0 cm)
DISTRACTOR_MARGIN_M = 0.020  # ruling pending (Finnegan): sim distractors are not calibrated (review m3); 82 E1 checks them
MIDPOINT_BELOW_M = 0.03      # review N1: below this (any scene group) the halfway pose last -> cand is checked too
ROUTE_TOLERANCE_M = 1e-4    # route_refused: a route that keeps a group's start distance to 0.1 mm counts as "not closer"
PELVIS_TILT_FROM_IMU = True # ruling pending (Finnegan) R-F10: pelvis roll/pitch from the LowState IMU (False: sim crouch tilt)

# Copies of sim constants, so this module imports without table_scene (which needs mujoco and the scene builder);
# tests/test_integration_fence.py pins them equal to g1sim/table_scene.py and g1sim/ball_grasp.py.
SCENE_BODIES = {"table": "table", "stand": "volleyball_stand", "basket": "basket"}   # group -> body (ball_grasp.FORBIDDEN)
DISTRACTORS = ("distractor_box", "distractor_can", "distractor_red_cylinder", "distractor_purple_cylinder",
               "distractor_yellow_block")                                           # table_scene.VOLLEYBALL_DISTRACTORS
HOLDER = "volleyball_stand"  # moves with the ball offset (table_scene.SCENES["volleyball_walk"]["holders"])
STATIC = ("volleyball_stand", "basket")   # stand on the table: move with its height (table_scene.SCENES[...]["static"])
WALK_STANCE = {"x_m": -0.04546501768451561, "y_m": 0.0, "yaw_deg": 0.0}           # table_scene.WALK_STANCE
GROOT_STANCE = {"pelvis_z": 0.7505, "roll_deg": 0.05, "pitch_deg": -0.96,          # table_scene.GROOT_STANCE
                "legs": (-0.377, 0.0, 0.0, 0.709, -0.317, 0.0, -0.402, 0.0, 0.0, 0.697, -0.281, 0.0)}
SELF_BODIES = {"torso": ("torso_link", "backpack_link"), "head": ("head_servo_link", "d455_link"),
               "pelvis_legs": ("pelvis", "left_hip_pitch_link", "left_hip_roll_link", "left_hip_yaw_link", "left_knee_link",
                               "right_hip_pitch_link", "right_hip_roll_link", "right_hip_yaw_link", "right_knee_link")}
SIDES = ("left", "right")
PRED_POS = np.r_[0, 3:17]    # 17-vector positions that get the tracking error in `pred`: waist yaw and both arms
WAIST_RP = slice(1, 3)       # waist roll/pitch: never commanded by the policy, so the fence takes the measured angles

FenceCheck = collections.namedtuple("FenceCheck", "hold values held low")


class FenceUnavailable(RuntimeError):
    """The fence cannot be built here (no mujoco, wrong version, model file missing, layout missing); 86 refuses."""


def margin(group: str) -> float:
    """Margin (m) of an obstacle group."""
    if group in MARGIN_M:
        return MARGIN_M[group]
    return SELF_MARGIN_M if group in (*SELF_BODIES, "other_arm") else DISTRACTOR_MARGIN_M


def refused_groups(values: dict, held: dict) -> list:
    """The never-closer rule (FENCE-SMOOTH.md 1.5, review M2): groups where the candidate is below its margin AND closer than
    holding still. Empty = send. A candidate below the margin that moves away (or along) is sent, so an arm that overshot
    into the margin can leave it; only pushing further in is refused."""
    return [g for g, v in values.items() if v < margin(g) and v < held[g]]


def low_groups(values: dict) -> list:
    """Groups below their margin."""
    return [g for g, v in values.items() if v < margin(g)]


def require_mujoco() -> None:
    """Raise FenceUnavailable unless the pinned MuJoCo is importable (a different version measures other distances)."""
    if mujoco is None:
        raise FenceUnavailable(f"mujoco is not installed in this env - run scripts/setup_robot_laptop.sh --robot-env "
                               f"(it pins mujoco=={MUJOCO_VERSION})")
    if mujoco.__version__ != MUJOCO_VERSION:
        raise FenceUnavailable(f"mujoco {mujoco.__version__} is installed, the fence needs {MUJOCO_VERSION} - "
                               f"re-run scripts/setup_robot_laptop.sh --robot-env")


def layout_from_sheet(setback_cm: float, table_dz_cm: float = 0.0, stand_dx_cm: float = 0.0, stand_dy_cm: float = 0.0,
                      basket_dx_cm: float | None = 0.0, basket_dy_cm: float = 0.0, robot_yaw_deg: float = 0.0) -> dict:
    """Runbook sheet numbers -> layout (the keys of a sim variation). Table frame: x away from the robot, y to its left;
    offsets from the nominal marks (FENCE-SMOOTH.md 1.3). setback: the table edge this much further away than the marks
    (the robot stance moves back, which is the same geometry as table + stand + basket moving away). basket_dx_cm None =
    no basket on the table. robot_yaw_deg: the robot turned on its marks (default 0 = feet on the marks)."""
    layout = {"stance": {"x_m": WALK_STANCE["x_m"] - setback_cm / 100, "y_m": WALK_STANCE["y_m"],
                         "yaw_deg": WALK_STANCE["yaw_deg"] + robot_yaw_deg},
              "objects": {"volleyball": {"dx_m": stand_dx_cm / 100, "dy_m": stand_dy_cm / 100}},
              "table": {"dz_m": table_dz_cm / 100}}
    if basket_dx_cm is not None:
        layout["basket"] = {"dx_m": basket_dx_cm / 100, "dy_m": basket_dy_cm / 100}
    return layout


class Fence:
    """Fence values of a robot pose in one MuJoCo model. Use from_env (sim) or nominal (robot laptop)."""

    def __init__(self, model, include_distractors: bool = True, groups_from=None):
        require_mujoco()
        m = self.model = model
        self.data = mujoco.MjData(m)
        body_acts = [a for a in range(m.nu) if not m.actuator(a).name.startswith(tuple(brainco.PREFIX.values()))]
        motor_qadr = m.jnt_qposadr[[int(m.actuator_trnid[a, 0]) for a in body_acts]]   # G1 motors 0..28
        self.qadr17, self.legs_qadr = motor_qadr[12:29], motor_qadr[0:12]
        self.hand_qadr = np.concatenate([m.jnt_qposadr[[m.joint(n).id for n in brainco.joint_names(s)]] for s in SIDES])
        # finger joints coupled to an actuated one (proximal = c0 + c1 * distal): FK sets the distal joint from it
        self.couplings = [(m.jnt_qposadr[m.eq_obj1id[i]], m.jnt_qposadr[m.eq_obj2id[i]], m.eq_data[i][0], m.eq_data[i][1])
                          for i in range(m.neq) if m.eq_type[i] == mujoco.mjtEq.mjEQ_JOINT and m.eq_obj2id[i] >= 0]
        self.base_qadr = m.jnt_qposadr[m.body_jntadr[m.body("pelvis").id]]
        self.base = self.data.qpos.copy()
        self.original_body_pos = m.body_pos.copy()
        self.pelvis_yaw_deg = WALK_STANCE["yaw_deg"]
        self.live = None                     # from_env: the env's MjData; its qpos is the base of every pose
        self.pairs = self._groups(include_distractors)

    # --- construction -------------------------------------------------------------------------------------------------
    @classmethod
    def from_env(cls, env) -> "Fence":
        """82: the eval env's own model (its layout follows every reset) and a private MjData; pelvis, legs and objects are
        copied from env.data at every check."""
        fence = cls(env.model)
        fence.live = env.data
        return fence

    @classmethod
    def nominal(cls, layout: dict | None, model_file: pathlib.Path = MODEL_FILE) -> "Fence":
        """86: the collision-hull model file with `layout` applied (layout_from_sheet, or a sim variation). No distractors:
        on the robot day nothing but table, stand and basket is entered."""
        require_mujoco()
        if layout is None:
            raise FenceUnavailable("no layout - enter the sheet numbers (--fence-setback-cm ...) or --fence-layout FILE")
        if not pathlib.Path(model_file).exists():
            raise FenceUnavailable(f"{model_file} is missing - pull again (it is in the repo)")
        model = mujoco.MjModel.from_xml_string(gzip.decompress(pathlib.Path(model_file).read_bytes()).decode())
        fence = cls(model, include_distractors=False)
        fence.set_layout(layout)
        return fence

    def set_layout(self, layout: dict) -> None:
        """Place table, stand, basket and the pelvis as TableEnv._set_up / _pose_base do for a variation (I4 pins it)."""
        m, pos = self.model, self.original_body_pos.copy()
        dz = layout.get("table", {"dz_m": 0.0})["dz_m"]
        ball = layout["objects"]["volleyball"]
        pos[m.body("table").id, 2] += dz
        pos[m.body(HOLDER).id] += [ball["dx_m"], ball["dy_m"], 0.0]
        if "basket" in layout:
            pos[m.body("basket").id] += [layout["basket"]["dx_m"], layout["basket"]["dy_m"], 0.0]
        elif "basket" in self.pairs:
            del self.pairs["basket"]         # --fence-no-basket: nothing there to keep away from
        for name in STATIC:
            pos[m.body(name).id, 2] += dz
        m.body_pos[:] = pos
        stance = layout.get("stance", WALK_STANCE)   # a nominal sim variation has none: the scene's stance
        a = self.base_qadr
        self.base[a:a + 3] = [stance["x_m"], stance["y_m"], GROOT_STANCE["pelvis_z"]]
        self.pelvis_yaw_deg = stance["yaw_deg"]
        self._set_tilt(self.base, GROOT_STANCE["roll_deg"], GROOT_STANCE["pitch_deg"])
        self.base[self.legs_qadr] = GROOT_STANCE["legs"]

    def set_base(self, qpos) -> None:
        """Use qpos for everything the pose does not set (pelvis, legs, objects); make_fence_model.py's comparison."""
        self.base[:] = qpos

    def _set_tilt(self, qpos, roll_deg: float, pitch_deg: float) -> None:
        a = self.base_qadr
        mujoco.mju_euler2Quat(qpos[a + 3:a + 7], np.radians([roll_deg, pitch_deg, self.pelvis_yaw_deg]), "XYZ")

    def _groups(self, include_distractors: bool) -> dict:
        """group -> (moving geoms, obstacle geoms). Scene groups: everything below both shoulder_pitch_links (87's set,
        review M1: the upper arm reaches the basket). Self groups: below both elbow_links (the upper arm touches the torso by
        construction). other_arm: each forearm/hand against the whole other arm below its shoulder_yaw_link."""
        m = self.model

        def collision_geoms(test):
            return np.array([g for g in range(m.ngeom) if (m.geom_contype[g] or m.geom_conaffinity[g]) and test(g)], int)

        def below(body_name):
            root = m.body(body_name).id
            return lambda g: _in_subtree(m, m.geom_bodyid[g], root)

        def bodies(names):
            ids = {m.body(n).id for n in names}
            return lambda g: m.geom_bodyid[g] in ids

        arms = collision_geoms(lambda g: any(below(f"{s}_shoulder_pitch_link")(g) for s in SIDES))
        forearms = {s: collision_geoms(below(f"{s}_elbow_link")) for s in SIDES}
        both_forearms = np.concatenate([forearms["left"], forearms["right"]])
        scene = dict(SCENE_BODIES)
        if include_distractors:
            scene.update({name: name for name in DISTRACTORS if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name) >= 0})
        pairs = {group: [(arms, collision_geoms(bodies([body])))] for group, body in scene.items()}
        pairs.update({group: [(both_forearms, collision_geoms(bodies(names)))] for group, names in SELF_BODIES.items()})
        upper = {s: collision_geoms(below(f"{s}_shoulder_yaw_link")) for s in SIDES}
        pairs["other_arm"] = [(forearms["left"], upper["right"]), (forearms["right"], upper["left"])]
        return pairs

    # --- values -------------------------------------------------------------------------------------------------------
    def pose_values(self, q17, hands12, pelvis_rp=None) -> dict:
        """Smallest distance (m, capped at DISTMAX_M, < 0 = overlap) per group with motors 12..28 at q17 and the hands at
        hands12 (normalized BrainCo targets, left 6 then right 6). pelvis_rp: (roll, pitch) rad of the pelvis, or None for
        the base pose's own tilt."""
        m, d = self.model, self.data
        d.qpos[:] = self.live.qpos if self.live is not None else self.base
        if pelvis_rp is not None:
            self._set_tilt(d.qpos, *np.degrees(pelvis_rp))
        d.qpos[self.qadr17] = q17
        hands = np.asarray(hands12, dtype=float)
        d.qpos[self.hand_qadr] = np.concatenate([brainco.to_rad(hands[:6]), brainco.to_rad(hands[6:])])   # left, right
        for proximal, distal, c0, c1 in self.couplings:
            d.qpos[distal] = (d.qpos[proximal] - c0) / c1
        mujoco.mj_kinematics(m, d)
        return {group: min(self._distance(a, b) for a, b in pairs) for group, pairs in self.pairs.items()}

    def values(self, cand17, hands12, measured17, last17, pelvis_rp=None, cmd_only: bool = False) -> dict:
        """Fence value per group of sending cand17: min over `cmd` and `pred` (FENCE-SMOOTH.md 1.1), plus the halfway pose
        from last17 when a scene group is below MIDPOINT_BELOW_M (review N1: the rate limit allows up to ~2.3 cm of geom
        movement per tick, more than twice the smallest margin, so the straight segment between two checked ends can dip
        below both). cmd_only: the shadow check (the arm holds the ready pose there, so `pred` would describe that pose)."""
        cand, measured, last = (np.asarray(v, dtype=float) for v in (cand17, measured17, last17))
        poses = [_with_waist(cand, measured)]
        if not cmd_only:
            poses.append(_predicted(cand, measured, last))
        result = _min_values(self.pose_values(q, hands12, pelvis_rp) for q in poses)
        near = min((result[g] for g in self._scene_groups()), default=DISTMAX_M) < MIDPOINT_BELOW_M
        if near and not cmd_only and not np.array_equal(cand, last):   # holding still has no segment
            mid = (last + cand) / 2
            result = _min_values([result, self.pose_values(_with_waist(mid, measured), hands12, pelvis_rp),
                                  self.pose_values(_predicted(mid, measured, last), hands12, pelvis_rp)])
        return result

    def held_values(self, last17, hands12_last, measured17, pelvis_rp=None) -> dict:
        """The fence value of holding still: cmd = last17 and pred = the measured arm (last + measured - last), hands at
        their last sent target (review N2)."""
        return self.values(last17, hands12_last, measured17, last17, pelvis_rp)

    def measured_values(self, measured17, hands12, pelvis_rp=None) -> dict:
        """The fence value of the measured arm alone (overshoot after a hold)."""
        measured = np.asarray(measured17, dtype=float)
        return self.pose_values(measured, hands12, pelvis_rp)

    def check(self, cand17, hands12, measured17, last17, hands12_last, pelvis_rp=None) -> FenceCheck:
        """The 86 / 82 decision: hold = some group below its margin and closer than holding still (refused_groups)."""
        values = self.values(cand17, hands12, measured17, last17, pelvis_rp)
        low = low_groups(values)
        if not low:
            return FenceCheck(False, values, None, [])
        held = self.held_values(last17, hands12_last, measured17, pelvis_rp)
        return FenceCheck(bool(refused_groups({g: values[g] for g in low}, held)), values, held, low)

    def route_refused(self, poses, hands12, measured17, pelvis_rp=None) -> dict:
        """Review fence-smooth O2: the groups a scripted route brings below their margin AND closer than at its start - the
        policy phase's never-closer rule (refused_groups) along the whole route, the robot's own body included. poses: the
        50 Hz targets 86 will send, the first one where the arm starts. Waist roll/pitch at the measured angles, hands at
        hands12 for the whole route, as in values(). {group: (start m, smallest m)}; empty = the route may be driven. A
        non-finite value (NaN angle or IMU) refuses: nothing says where the route goes."""
        measured = np.asarray(measured17, dtype=float)
        values = [self.pose_values(_with_waist(np.asarray(q, dtype=float), measured), hands12, pelvis_rp) for q in poses]
        start, smallest = values[0], _min_values(values)
        if not all(np.isfinite(v) for v in (*start.values(), *smallest.values())):
            return {"no usable angle": (float("nan"), float("nan"))}
        no_closer = {g: v - ROUTE_TOLERANCE_M for g, v in start.items()}
        return {g: (start[g], smallest[g]) for g in refused_groups(smallest, no_closer)}

    def _scene_groups(self) -> tuple:
        return tuple(g for g in self.pairs if g not in SELF_BODIES and g != "other_arm")

    def _distance(self, moving: np.ndarray, obstacles: np.ndarray) -> float:
        """Smallest mj_geomDistance between two geom sets, after a bounding-sphere pre-filter (arm_clearance_fk.py)."""
        m, d = self.model, self.data
        if len(moving) == 0 or len(obstacles) == 0:
            return DISTMAX_M
        centre = np.linalg.norm(d.geom_xpos[moving][:, None] - d.geom_xpos[obstacles][None], axis=2)
        lower = centre - m.geom_rbound[moving][:, None] - m.geom_rbound[obstacles][None]
        best = DISTMAX_M
        for i, j in zip(*np.nonzero(lower < best)):
            best = min(best, mujoco.mj_geomDistance(m, d, int(moving[i]), int(obstacles[j]), DISTMAX_M, None))
        return float(best)


def _in_subtree(model, body: int, root: int) -> bool:
    while body != 0:
        if body == root:
            return True
        body = model.body_parentid[body]
    return False


def _with_waist(q17: np.ndarray, measured17: np.ndarray) -> np.ndarray:
    """q17 with waist roll/pitch at the measured angles (86 sends q0 there; the torso sits where it is)."""
    pose = q17.copy()
    pose[WAIST_RP] = measured17[WAIST_RP]
    return pose


def _predicted(q17: np.ndarray, measured17: np.ndarray, last17: np.ndarray) -> np.ndarray:
    """`pred` = q17 + (measured - last sent) on waist yaw and arms: where the arm will sit if it keeps its present error."""
    pose = _with_waist(q17, measured17)
    pose[PRED_POS] = q17[PRED_POS] + (measured17[PRED_POS] - last17[PRED_POS])
    return pose


def _min_values(dicts) -> dict:
    dicts = list(dicts)
    return {g: min(d[g] for d in dicts) for g in dicts[0]}
