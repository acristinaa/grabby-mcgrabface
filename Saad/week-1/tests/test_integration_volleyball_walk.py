"""Integration: free-floating volleyball world for the Level 1 walk-up (SCENES["volleyball_walk"], TableEnv free base).

Standing posture (measured GR00T crouch) on the floor, pelvis weld, stance placement, FK clearance to the table, handover
snapshot/restore and the per-reset camera-noise seed.
Needs the built walking model (scripts/51_build_brainco_sim.py); the test writes the scene itself.
"""
import mujoco
import numpy as np
import pytest

from conftest import needs_g1ext

pytestmark = [pytest.mark.integration, needs_g1ext]

SCENE = "volleyball_walk"


@pytest.fixture(scope="module")
def env():
    from g1sim import MANIFEST, SIM_DIR, table_env, table_scene

    if not (SIM_DIR / MANIFEST).is_file():
        pytest.skip("walking model missing - run scripts/51_build_brainco_sim.py first")
    table_scene.write(scene=SCENE)
    e = table_env.TableEnv(scene=SCENE)
    yield e
    e.close()


def noisy_variation(env, stance=None):
    """Nominal variation with camera noise on (nominal has none), optionally at a stance."""
    from g1sim import randomization

    v = randomization.nominal(env.objects)
    v["image"].update(rgb_noise_std=3.0, depth_noise_std_m=0.002)
    if stance is not None:
        v["stance"] = stance
    return v


def pelvis_pose(env):
    a = env.base_qadr
    return env.data.qpos[a:a + 3].copy(), env.data.qpos[a + 3:a + 7].copy()


def yaw_deg(quat):
    w, x, y, z = quat
    return float(np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))


def foot_gaps(env):
    """{side: smallest distance (m) of the foot contact spheres to the floor}. The left spheres sit in child bodies."""
    m, d = env.model, env.data
    mujoco.mj_kinematics(m, d)
    floor = m.geom("floor").id
    gaps = {}
    for side in ("left", "right"):
        ankle = m.body(f"{side}_ankle_roll_link").id
        feet = [g for g in env.robot_geoms if ankle in (m.geom_bodyid[g], m.body_parentid[m.geom_bodyid[g]])]
        assert len(feet) == 4, (side, feet)
        gaps[side] = min(mujoco.mj_geomDistance(m, d, g, floor, 0.1, None) for g in feet)
    return gaps


def test_feet_on_floor_and_weld_holds(env):
    from g1sim import action_space, table_env

    env.reset()
    for side, gap in foot_gaps(env).items():
        assert abs(gap) <= 0.003, (side, gap)
    pos, quat = pelvis_pose(env)
    for _ in range(3 * table_env.CONTROL_HZ):  # 3 s
        env.apply(action_space.READY_POSE)
    pos2, quat2 = pelvis_pose(env)
    turn = np.zeros(3)
    mujoco.mju_subQuat(turn, quat2, quat)
    assert np.linalg.norm(pos2 - pos) <= 0.002
    assert np.degrees(np.linalg.norm(turn)) <= 0.2
    assert env.robot_contacts() == []


def test_stance_is_applied_not_pulled_back(env):
    from g1sim import action_space, table_env

    stance = {"x_m": -0.05, "y_m": -0.05, "yaw_deg": 8.0}
    env.reset(variation=noisy_variation(env, stance))
    for _ in range(table_env.CONTROL_HZ):  # 1 s on top of the reset's settle
        env.apply(action_space.READY_POSE)
    pos, quat = pelvis_pose(env)
    assert np.allclose(pos[:2], [-0.05, -0.05], atol=0.001)
    assert yaw_deg(quat) == pytest.approx(8.0, abs=0.1)


def test_stance_clearance_matches_probe(env):
    from g1sim import table_scene

    env.reset()
    assert env.stance_clearance(table_scene.WALK_STANCE) >= 0.03  # nominal R_STOP stop, pelvis 0.236 m from the edge
    edge = table_scene.SCENES[SCENE]["table"][0] - table_scene.SCENES[SCENE]["table"][1][0]
    assert env.stance_clearance({"x_m": edge - 0.17, "y_m": 0.0, "yaw_deg": 0.0}) < 0  # knee in the table


def test_basket_offset_moves_basket(env):
    from g1sim import table_scene

    v = noisy_variation(env)
    v["basket"] = {"dx_m": 0.02, "dy_m": -0.01}
    env.reset(variation=v)
    basket = env.data.xpos[env.model.body("basket").id]
    assert np.allclose(basket[:2], np.add(table_scene.BASKET_CENTER, [0.02, -0.01]))
    env.reset()
    assert np.allclose(env.data.xpos[env.model.body("basket").id][:2], table_scene.BASKET_CENTER)


def run_ticks(env, n):
    """n ticks with the waist turning, one head image per tick. Returns (qpos per tick, images)."""
    from g1sim import action_space

    action = action_space.READY_POSE.copy()
    qpos, images = [], []
    for i in range(n):
        action[action_space.SLICES["waist"]] = 0.01 * (i + 1)
        env.apply(action)
        qpos.append(env.data.qpos.copy())
        images.append(env.render("head_camera"))
    return np.array(qpos), np.array(images)


def test_snapshot_restore_is_bit_exact(env):
    from g1sim import action_space

    env.reset(variation=noisy_variation(env, {"x_m": -0.04, "y_m": 0.01, "yaw_deg": 3.0}))
    for _ in range(5):
        env.apply(action_space.READY_POSE)
        env.render("head_camera")
    snapshot = env.snapshot_handover()
    qpos_a, images_a = run_ticks(env, 30)
    env.reset()  # somewhere else in between
    env.render("head_camera")
    env.restore_handover(snapshot)
    qpos_b, images_b = run_ticks(env, 30)
    assert np.array_equal(qpos_a, qpos_b)
    assert np.array_equal(images_a, images_b)
    assert images_a.std() > 0 and not np.array_equal(images_a[0], images_a[1])  # noise is on and changes per image


def test_restore_in_fresh_env(env):
    """As in a new process (eval --from-handover): state bit-exact, images within 1 grey level (second GL context)."""
    from g1sim import action_space, table_env

    env.reset(variation=noisy_variation(env, {"x_m": -0.04, "y_m": 0.01, "yaw_deg": 3.0}))
    for _ in range(5):
        env.apply(action_space.READY_POSE)
    snapshot = env.snapshot_handover()
    qpos_a, images_a = run_ticks(env, 30)
    fresh = table_env.TableEnv(scene=SCENE)
    fresh.restore_handover(snapshot)
    qpos_b, images_b = run_ticks(fresh, 30)
    fresh.close()
    assert np.array_equal(qpos_a, qpos_b)
    assert np.abs(images_a.astype(int) - images_b.astype(int)).max() <= 1


def test_reset_noise_depends_only_on_seed_and_index():
    from g1sim import table_env

    def second_reset_images(renders_in_between):
        e = table_env.TableEnv(scene=SCENE, seed=7)
        v = noisy_variation(e)
        e.reset(variation=v)
        for _ in range(renders_in_between):
            e.render("head_camera")
        e.reset(variation=v)
        rgb, depth = e.render_depth("head_camera")
        e.close()
        return rgb, depth

    rgb_a, depth_a = second_reset_images(0)
    rgb_b, depth_b = second_reset_images(5)
    assert rgb_a.shape[:2] == depth_a.shape
    assert np.array_equal(rgb_a, rgb_b) and np.array_equal(depth_a, depth_b)
