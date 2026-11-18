"""Integration: the virtual fence's geometry - the shipped collision model, the layout, the obstacle groups, the replay
demo in its own layout and 82 --fence in the sim (FENCE-SMOOTH.md 1.1-1.6; tests I1-I7 and the constant pins).

Sim env only (MuJoCo 3.13, the version the fence pins): ~/dev/g1-ext/.venv. Forward kinematics only, except I7 (a few
seconds of sim physics). The poses below were found by a random search over arm angles around the ready pose
(/tmp/fence/build/find_poses.py, find_upper.py, 2026-11-08) and are kept as numbers so the test does not search.
"""
import json
import pathlib

import numpy as np
import pytest

from conftest import needs_g1ext

pytestmark = [pytest.mark.integration, needs_g1ext]

mujoco = pytest.importorskip("mujoco")
if mujoco.__version__ != "3.13.0":
    pytest.skip("the fence pins MuJoCo 3.13.0 (sim env ~/dev/g1-ext/.venv)", allow_module_level=True)

from g1sim import action_space, fence  # noqa: E402

DATA = pathlib.Path(__file__).resolve().parent / "data"
READY = action_space.BALL_READY_POSE
HANDS = READY[14:26]
WAIST_RP = [0.0, 0.04]          # waist roll/pitch the sim torso settles at under the arms (measured 0.036-0.047 rad)
READY17 = np.concatenate([[READY[26]], WAIST_RP, READY[0:14]])
INTO = {   # 17-vector poses (motors 12..28) with one part inside a group, nominal layout
    "table": [-0.558, 0.0, 0.04, -0.903, 0.702, 0.492, 0.073, -0.849, 1.468, -0.162, -0.794, -0.426, 1.184, 0.294, -0.999,
              0.156, 0.527],
    "stand": [-0.236, 0.0, 0.04, -0.222, 1.131, -0.854, 0.277, -0.452, 0.416, 0.786, -0.468, 0.069, -1.134, -0.192, 0.092,
              0.191, 0.692],
    "torso": [0.564, 0.0, 0.04, -0.362, -0.828, -0.233, -1.512, -0.57, 1.201, -0.527, -0.286, 1.104, 1.108, -0.261, 0.099,
              0.065, -0.814],     # the left hand inside the torso, head and right arm
}
INTO["other_arm"] = INTO["head"] = INTO["torso"]
# right upper arm (shoulder links) inside the basket while the forearm and hand stay > 2 cm away; table raised 10 cm and
# 5 cm closer, so the basket rim reaches shoulder height
UPPER_ARM_INTO_BASKET = ([0.004, 0.0, 0.04, -0.25, 0.05, -0.0, -0.8, -0.0, 0.6, -0.0, -1.18, -0.074, 1.284, -0.815, -0.636,
                          0.797, 0.96], fence.layout_from_sheet(-5.0, table_dz_cm=10.0))


@pytest.fixture(scope="module", autouse=True)
def scene():
    """The walk scene XML (other tests rebuild the sim folder; test_integration_vla_eval.py does the same)."""
    from g1sim import MANIFEST, SIM_DIR, table_scene
    if not (SIM_DIR / MANIFEST).is_file():
        pytest.skip("walking model missing - run scripts/51_build_brainco_sim.py first")
    table_scene.write(scene="volleyball_walk")


@pytest.fixture(scope="module")
def nominal():
    return fence.Fence.nominal(fence.layout_from_sheet(0.0))


def make_env(seed=0, config=None):
    from g1sim import randomization, table_env
    return table_env.TableEnv(scene="volleyball_walk", seed=seed,
                              variation_config=randomization.load(config or randomization.CONFIG))


def q17(action27, waist_rp=WAIST_RP):
    return np.concatenate([[action27[26]], waist_rp, action27[0:14]])


def test_fence_constants_equal_the_sim_scene():
    """Proves: the copies in g1sim/fence.py (kept so 86 can import it without the scene builder) equal the sim's own values:
    forbidden bodies, distractors, the stand holder, static bodies, the walk stance and the GR00T crouch.
    Catches: a scene change that moves the sim but not the robot's fence."""
    from g1sim import ball_grasp, table_scene
    scene = table_scene.SCENES["volleyball_walk"]

    assert set(ball_grasp.FORBIDDEN) == set(fence.SCENE_BODIES.values()) | set(fence.DISTRACTORS)
    assert tuple(table_scene.VOLLEYBALL_DISTRACTORS) == fence.DISTRACTORS
    assert scene["holders"] == {"volleyball": fence.HOLDER} and tuple(scene["static"]) == fence.STATIC
    assert table_scene.WALK_STANCE == fence.WALK_STANCE
    assert {k: table_scene.GROOT_STANCE[k] for k in fence.GROOT_STANCE} == fence.GROOT_STANCE


def test_each_group_is_wired_to_its_obstacle(nominal):
    """Proves (I1): the ready pose is outside every margin; a hand in the table, in the stand, in the torso, head and other
    arm, and the upper arm alone in the basket each give a negative value for exactly that group.
    Catches: a dropped group, a group wired to the wrong body, a strict/non-strict margin slip, or scene groups that only
    look at forearm and hand (the upper arm reaches the basket, review M1) [mutants 5, 9, 10, 19]."""
    ready = nominal.pose_values(READY17, HANDS)
    pose, layout = UPPER_ARM_INTO_BASKET
    upper = fence.Fence.nominal(layout).pose_values(np.array(pose), HANDS)

    assert all(value >= fence.margin(group) for group, value in ready.items()), ready
    for group, pose17 in INTO.items():
        assert nominal.pose_values(np.array(pose17), HANDS)[group] < 0.0, group
    assert upper["basket"] < -0.005


def test_pred_catches_the_sagging_arm(nominal):
    """Proves (I2): a command 5-7 cm above the table whose arm sags by the measured error to ~0.5 cm is refused; the same
    command without the sag is sent.
    Catches: a cmd-only fence (the commanded pose never comes near the table, the real arm does) or a flipped sag sign
    [mutants 3, 4]."""
    toward = np.array(INTO["table"])
    path = [READY17 + t * (toward - READY17) for t in np.linspace(0.0, 1.0, 201)]
    table = np.array([nominal.pose_values(q, HANDS)["table"] for q in path])
    cmd_i = int(np.flatnonzero((table > 0.05) & (table < 0.07))[0])
    sag_i = int(np.flatnonzero(table < 0.005)[0])
    last, cand = path[cmd_i - 1], path[cmd_i]
    sag = path[sag_i] - path[cmd_i]

    sagging = nominal.check(cand, HANDS, last + sag, last, HANDS)
    tracking = nominal.check(cand, HANDS, last, last, HANDS)

    assert sagging.hold and sagging.values["table"] < fence.margin("table")
    assert not tracking.hold


def test_layout_moves_the_obstacles(nominal):
    """Proves (I3): the sheet numbers move what the fence sees - setback +5 cm puts the pelvis 5 cm back and moves the stand
    away from a hand near it, the table 2 cm higher brings a hand above it closer, the stand offset and robot yaw change the
    stand value, and no basket removes the basket group.
    Catches: a sheet number that is parsed but never reaches the model [mutant 8]."""
    toward = np.array(INTO["stand"])
    stand_pose = next(q for q in (READY17 + t * (toward - READY17) for t in np.linspace(0.0, 1.0, 201))
                      if nominal.pose_values(q, HANDS)["stand"] < 0.04)                 # a hand ~4 cm before the stand
    toward = np.array(INTO["table"])
    table_pose = next(q for q in (READY17 + t * (toward - READY17) for t in np.linspace(0.0, 1.0, 201))
                      if nominal.pose_values(q, HANDS)["table"] < 0.05)                 # a hand ~5 cm above the table
    base_stand, base_table = nominal.pose_values(stand_pose, HANDS)["stand"], nominal.pose_values(table_pose, HANDS)["table"]

    def value(pose, group, **sheet):
        return fence.Fence.nominal(fence.layout_from_sheet(sheet.pop("setback_cm", 0.0), **sheet)).pose_values(
            pose, HANDS).get(group)

    assert value(stand_pose, "stand", setback_cm=5.0) > base_stand + 0.01       # 1-Lipschitz: at most the 5 cm moved
    moved = fence.Fence.nominal(fence.layout_from_sheet(5.0))
    moved.pose_values(stand_pose, HANDS)
    assert moved.data.xpos[moved.model.body("pelvis").id][0] == pytest.approx(fence.WALK_STANCE["x_m"] - 0.05, abs=1e-9)
    assert base_table - value(table_pose, "table", table_dz_cm=2.0) > 0.01
    assert abs(value(stand_pose, "stand", stand_dx_cm=3.0) - base_stand) > 0.005
    assert abs(value(stand_pose, "stand", robot_yaw_deg=5.0) - base_stand) > 0.005
    assert value(stand_pose, "basket", basket_dx_cm=None) is None


@pytest.mark.parametrize("config", ["randomization_v1.json", "randomization.json"])
def test_nominal_model_and_layout_equal_the_sim(config):
    """Proves (I4): Fence.nominal (the shipped hull model file + layout code) puts table, stand, basket and pelvis where
    TableEnv puts them for 10 random variations (scene geoms within 0.01 mm), and its fence values equal the sim's own
    within 1 mm on 5 poses each. The remaining <= 0.8 mm is the sim's settling under the pelvis weld (legs 0.005 rad,
    pelvis 0.04 deg), which the nominal crouch does not model.
    Catches: a wrong hull file, a frame mix-up in set_layout, or the two scene builds drifting apart."""
    env = make_env(seed=7, config=pathlib.Path(__file__).resolve().parent.parent / "configs" / config)
    rng = np.random.default_rng(3)
    worst_geom = worst_value = 0.0
    for _ in range(10):
        env.reset(randomize=True)
        live = fence.Fence.from_env(env)
        layout = {k: env.variation[k] for k in ("stance", "objects", "basket", "table") if k in env.variation}
        hull = fence.Fence.nominal(layout)
        waist_rp = env.data.qpos[live.qadr17[1:3]]
        for _ in range(5):
            pose = q17(READY, waist_rp)
            pose[3:17] += rng.uniform(-0.5, 0.5, 14)
            a, b = live.pose_values(pose, HANDS), hull.pose_values(pose, HANDS)
            worst_value = max(worst_value, max(abs(a[g] - b[g]) for g in b))
        for group in fence.SCENE_BODIES.values():
            ids_env = [g for g in range(env.model.ngeom) if env.model.geom_bodyid[g] == env.model.body(group).id
                       and (env.model.geom_contype[g] or env.model.geom_conaffinity[g])]
            ids_hull = [g for g in range(hull.model.ngeom) if hull.model.geom_bodyid[g] == hull.model.body(group).id]
            worst_geom = max(worst_geom, np.abs(live.data.geom_xpos[ids_env] - hull.data.geom_xpos[ids_hull]).max())
    env.close()

    assert worst_geom < 1e-5 and worst_value < 1e-3, (worst_geom, worst_value)


def test_ball_is_never_an_obstacle(nominal):
    """Proves (I5): the obstacle groups are exactly the sim's forbidden bodies present in the model plus the self groups -
    the volleyball (the thing to grasp) is not one, so a hand on the ball is never held.
    Catches: a fence that refuses the grasp itself [mutant 11]."""
    env = make_env()
    env.reset(randomize=False)
    live = fence.Fence.from_env(env)
    present = {g for g, body in {**fence.SCENE_BODIES, **{d: d for d in fence.DISTRACTORS}}.items()
               if mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, body) >= 0}
    env.close()

    expected = present | set(fence.SELF_BODIES) | {"other_arm"}
    assert set(live.pairs) == expected and "volleyball" not in live.pairs and "volleyball" not in nominal.pairs


def replay_through_fence(layout, waist_rp):
    """tests/data/replay_demo.npz through the 86 decision as 86 replay would make it (cmd = the demo action, measured = the
    recorded state, last = the previous action). Returns (refused ticks, smallest value per group in cm)."""
    demo = np.load(DATA / "replay_demo.npz")
    actions, states = demo["actions"].astype(float), demo["state"].astype(float)
    hull = fence.Fence.nominal(layout)
    refused, smallest, last = 0, {}, actions[0]
    for action, state in zip(actions, states):
        result = hull.check(q17(action, waist_rp), action[14:26], q17(state, waist_rp), q17(last, waist_rp), last[14:26])
        refused += result.hold
        for group, value in result.values.items():
            smallest[group] = min(smallest.get(group, 1.0), value)
        last = action
    return refused, {g: round(v * 100, 2) for g, v in smallest.items()}


@pytest.fixture(scope="module")
def demo_waist_rp():
    """Waist roll/pitch the sim torso settled at when the demo was recorded (the recorded states carry none): the demo's
    first reset regenerated exactly as tests/helpers/make_replay_demo_layout.py does (seed 101, v1 config), physics included."""
    env = make_env(seed=101, config=pathlib.Path(__file__).resolve().parent.parent / "configs" / "randomization_v1.json")
    env.reset(randomize=True)
    waist_rp = env.data.qpos[fence.Fence.from_env(env).qadr17[1:3]].copy()
    env.close()
    return waist_rp


def test_replay_demo_in_its_own_layout_is_never_refused(demo_waist_rp):
    """Proves (I6): the recorded replay demo in its own layout (tests/data/replay_demo_layout.json) passes the fence with 0
    refused ticks; its closest approaches match the planner's sim measurement (table 1.97, stand 3.27, basket 4.18 cm) within
    1 mm (the hull model's crouch vs the sim's settled crouch, see I4).
    Catches: margins too large for the demos the model learned from, or a layout file that is ignored."""
    layout = json.loads((DATA / "replay_demo_layout.json").read_text())

    refused, smallest = replay_through_fence(layout, demo_waist_rp)

    assert refused == 0
    assert smallest["table"] == pytest.approx(1.97, abs=0.1) and smallest["stand"] == pytest.approx(3.27, abs=0.1)
    assert smallest["basket"] == pytest.approx(4.18, abs=0.1)


def test_robot_day_replay_needs_the_stand_where_the_demo_had_it(demo_waist_rp):
    """Proves (I6b, R-F8): in the nominal layout with the stand 6.7 cm further away and 1.7 cm to the right (where the demo
    had it relative to the robot) the replay is never refused; with the stand on its nominal mark it comes within 1.7 cm of
    the stand and the fence refuses there.
    Catches: a runbook placement that makes the operators' go/no-go replay fail on the robot day."""
    placed, _ = replay_through_fence(fence.layout_from_sheet(0.0, stand_dx_cm=6.7, stand_dy_cm=-1.7), demo_waist_rp)
    unplaced, smallest = replay_through_fence(fence.layout_from_sheet(0.0), demo_waist_rp)

    assert placed == 0
    assert unplaced > 0 and smallest["stand"] < fence.margin("stand") * 100


class PushClient:
    """82 policy client whose chunks drive the left hand from the ready pose into the table (INTO["table"])."""

    def __init__(self):
        self.k = 0

    def reset(self, episode=None, seed=0):
        self.k = 0

    def infer(self, obs, tick):
        target = READY.copy()
        target[0:14], target[26] = np.array(INTO["table"])[3:17], INTO["table"][0]
        t = np.clip((tick + np.arange(50)) / 60.0, 0.0, 1.0)[:, None]
        return {"actions": READY + t * (target - READY), "server_ms": 0.0, "round_trip_ms": 0.0}


@pytest.mark.parametrize("fenced", [True, False])
def test_82_fence_stops_a_hand_driven_into_the_table(fenced):
    """Proves (I7): in 82 (the sim, which has the table) a policy that drives a hand into the table is held before the
    contact and the trial ends as fence_exit with no forbidden contact; without --fence the same policy touches the table.
    (The push passes over the basket on its way down, so the unfenced contact can be the basket rim or the table.)
    Catches: a fence that is not wired into 82's apply, or one that only acts after the contact (m4)."""
    from g1sim import ball_grasp, vla_eval
    env = make_env()
    env.reset(randomize=False)
    tracker = vla_eval.EventTracker(env.object_position(ball_grasp.BALL), timeout_ticks=200)

    row = vla_eval.run_policy_trial(env, PushClient(), "task", tracker, 25, None, 0, fence=fenced)
    env.close()

    if fenced:
        assert row["reason"] == "fence_exit" and row["forbidden_pair"] is None and row["fence_holds"] > 0
    else:
        assert row["reason"] == "no_forbidden_contacts" and row["forbidden_pair"][1] in ball_grasp.FORBIDDEN


# --- review fence-smooth O2: the soft exit from a pose the fence held near the robot's own body --------------------------
# Last commands the self fence held near the torso / head (reviewer probe /tmp/fence/review3/probe_exit.py, hull model,
# nominal layout, ready-pose hands, 2026-11-09), kept as numbers so the test does not search.
HELD_NEAR_BODY = {
    "left fingertip 2.1 cm from the torso": [-0.106, 0.0, 0.04, -0.729, 0.173, -0.014, -0.77, 0.181, 0.06, -0.262, -0.15,
                                             -0.094, -0.525, -0.713, 0.017, 0.271, -0.368],
    "right palm 2.1 cm from the torso": [-0.114, 0.0, 0.04, -0.335, -0.21, 0.229, -0.42, -0.38, 0.755, -0.052, -0.63,
                                         -0.299, 0.1, -0.9, -0.167, 0.22, 0.247],
    "right forearm 2.2 cm from the head": [0.009, 0.0, 0.04, -0.407, 0.182, 0.127, -0.683, -0.281, 0.63, -0.183, -1.13,
                                           -0.127, 0.1, -0.676, 0.259, 0.654, 0.114],
}
# held near the head, but the lift route keeps 2.26 cm from the body (above the 2 cm self margin)
HELD_LIFT_CLEAR = [0.03, 0.0, 0.04, -0.35, -0.05, -0.1, -0.7, -0.1, 0.5, -0.1, -0.35, -0.15, 0.071, -0.7, 0.1, 0.5, 0.1]
Q0 = np.concatenate([[0.0], WAIST_RP, np.zeros(14)])
SELF_GROUPS = (*fence.SELF_BODIES, "other_arm")


def exit_runner(fence_, command17):
    """The parts of an 86 Runner that real_run_exits.soft_exit_path reads, after a policy phase that ended at command17
    (the arm tracks it: measured = command)."""
    import types
    command = np.array(command17, dtype=float)
    return types.SimpleNamespace(arm_path={"phase": "policy"}, command=command, measured=command.copy(), q0=Q0,
                                 ready17=READY17, io=types.SimpleNamespace(fence=fence_), pelvis_rp=None,
                                 hand_command=lambda: (HANDS[:6], HANDS[6:]), say=lambda text: None,
                                 note_event=lambda name: None)


def body_along(fence_, waypoints, ready_index):
    """({self group: smallest distance}, {self group: distance at the start}) along the 50 Hz targets up to the ready pose
    (waypoints[ready_index])."""
    from g1sim import arm_paths
    up = waypoints[:ready_index + 1]
    values = [fence_.pose_values(q, HANDS) for q in [up[0]] + arm_paths.path_ticks(up)]
    return {g: min(v[g] for v in values) for g in SELF_GROUPS}, {g: values[0][g] for g in SELF_GROUPS}


@pytest.mark.parametrize("where", HELD_NEAR_BODY)
def test_soft_exit_from_a_pose_held_near_the_body_keeps_off_the_body(nominal, where):
    """Proves (review fence-smooth O2): from a last command the fence held 2.1-2.2 cm from the torso or head, the old way
    out (lift, waist home, ready) goes more than 2 cm INTO the torso or head, and the soft exit 86 now drives
    (soft_exit_path) keeps every self group - torso, head, pelvis/legs, other arm - at its margin or no closer than at the
    start, all the way to the ready pose where the hands open.
    Catches: a soft exit that still drives the lift route unchecked (the old code), or a route check that leaves out the
    robot's own body."""
    from g1sim import arm_paths, real_preflight as pre, real_run_exits
    start = np.array(HELD_NEAR_BODY[where])
    old_route = arm_paths.way_out(start, Q0, READY17, pre.LIMITS_LO, pre.LIMITS_HI)
    old_min, start_values = body_along(nominal, old_route, arm_paths.WAY_OUT_READY)
    assert min(old_min.values()) < -0.02, "the old route no longer goes into the body: the test proves nothing"

    plan, waypoints, ready = real_run_exits.soft_exit_path(exit_runner(nominal, start))

    new_min, _ = body_along(nominal, waypoints, ready)
    assert plan == arm_paths.WAY_OUT and np.allclose(waypoints[ready], READY17)
    assert all(new_min[g] >= min(fence.margin(g), start_values[g]) - 1e-4 for g in SELF_GROUPS), new_min


def test_soft_exit_keeps_the_87_lift_route_where_it_stays_off_the_body(nominal):
    """Proves (review fence-smooth O2): from the ready pose and from a held pose whose lift route stays >= 2 cm from the
    body, the soft exit is still the lift route (lift, waist home, ready) that 87's lift_home rows check against table,
    stand and basket.
    Catches: a fix that always takes the straight route, which 87 does not check against the table."""
    from g1sim import arm_paths, real_preflight as pre, real_run_exits
    for start in (READY17, np.array(HELD_LIFT_CLEAR)):
        _, waypoints, ready = real_run_exits.soft_exit_path(exit_runner(nominal, start))
        lift_route = arm_paths.way_out(start, Q0, READY17, pre.LIMITS_LO, pre.LIMITS_HI)
        assert ready == arm_paths.WAY_OUT_READY and len(waypoints) == len(lift_route)
        assert all(np.allclose(a, b) for a, b in zip(waypoints, lift_route))
