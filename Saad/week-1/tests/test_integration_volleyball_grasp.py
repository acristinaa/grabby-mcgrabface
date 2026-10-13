"""Integration: two-hand volleyball pick-and-place teacher (g1sim/ball_grasp.py) on the volleyball scene.

One nominal trial must pass; the placement check must reject a ball outside the basket and a moving ball (states set
directly). Also covers the waist yaw target of TableEnv (action index 26), the stand moving with the ball's start offset and
a trial with the robot's stand shifted and turned (random starts).
Needs the built walking model (scripts/51_build_brainco_sim.py); the test writes the scene itself.
"""
import mujoco
import numpy as np
import pytest

from conftest import needs_g1ext

pytestmark = [pytest.mark.integration, needs_g1ext]


@pytest.fixture(scope="module")
def env():
    from g1sim import MANIFEST, SIM_DIR, table_env, table_scene

    if not (SIM_DIR / MANIFEST).is_file():
        pytest.skip("walking model missing - run scripts/51_build_brainco_sim.py first")
    table_scene.write(scene="volleyball")
    e = table_env.TableEnv(scene="volleyball")
    yield e
    e.close()


def place_ball(env, xyz, velocity=(0.0, 0.0, 0.0)):
    """Put the ball at xyz with the given linear velocity (no physics step), arms in the ready pose."""
    from g1sim import ball_grasp

    m, d = env.model, env.data
    joint = m.joint(f"{ball_grasp.BALL}_joint").id
    qadr, dof = m.jnt_qposadr[joint], m.jnt_dofadr[joint]
    d.qpos[qadr:qadr + 7] = [*xyz, 1.0, 0.0, 0.0, 0.0]
    d.qvel[dof:dof + 6] = [*velocity, 0.0, 0.0, 0.0]
    mujoco.mj_forward(m, d)


def basket_floor_point(env):
    from g1sim import ball_grasp, table_scene

    basket = env.data.xpos[env.model.body("basket").id]
    return [basket[0], basket[1], basket[2] + table_scene.BASKET_WALL + ball_grasp.RADIUS]


def test_nominal_trial_passes(env):
    from g1sim import ball_grasp

    env.reset()
    grasp = ball_grasp.TwoHandBallGrasp(env)
    result = grasp.run()
    assert result["passed"], (result["first_failed"], result["checks"], result["forbidden_contacts"])
    assert result["first_failed"] is None
    assert result["lift_height_m"] >= ball_grasp.MIN_LIFT
    # ends where it started: commands the scene's start pose (waist to the front); measured arms sag a little under gravity
    assert np.allclose(grasp.action, env.ready_pose, atol=1e-6)
    assert np.abs(env.state() - env.ready_pose).max() < 0.1


def test_placement_check_accepts_ball_lying_in_basket(env):
    from g1sim import ball_grasp

    env.reset()
    place_ball(env, basket_floor_point(env))
    assert ball_grasp.placement_checks(env) == {"ball_in_basket": True, "ball_on_basket_floor": True,
                                                "ball_at_rest": True, "hands_off_ball": True}
    assert ball_grasp.ball_placed(env)


def test_placement_check_rejects_ball_outside_basket(env):
    from g1sim import ball_grasp, table_scene

    env.reset()
    place_ball(env, [0.55, 0.0, table_scene.TABLE_TOP + ball_grasp.RADIUS])  # lying on the table between stand and basket
    checks = ball_grasp.placement_checks(env)
    assert not checks["ball_in_basket"] and not checks["ball_on_basket_floor"]
    assert not ball_grasp.ball_placed(env)


def test_placement_check_rejects_moving_ball(env):
    from g1sim import ball_grasp

    env.reset()
    place_ball(env, basket_floor_point(env), velocity=(0.03, 0.0, 0.0))
    checks = ball_grasp.placement_checks(env)
    assert checks["ball_in_basket"] and checks["ball_on_basket_floor"]
    assert not checks["ball_at_rest"]
    assert not ball_grasp.ball_placed(env)


def test_waist_yaw_target(env):
    from g1sim import action_space, table_env

    env.reset()
    assert env.waist_yaw_target == 0.0
    action = action_space.READY_POSE.copy()
    action[action_space.SLICES["waist"]] = 0.3
    for _ in range(table_env.CONTROL_HZ):  # 1 s
        env.apply(action)
    assert env.waist_yaw_target == pytest.approx(0.3)
    assert env.state()[26] == pytest.approx(0.3, abs=0.01)
    assert env.data.qpos[env.body_qadr[table_env.WAIST_YAW]] == pytest.approx(0.3, abs=0.01)
    env.reset()
    assert env.waist_yaw_target == 0.0
    assert env.data.qpos[env.body_qadr[table_env.WAIST_YAW]] == pytest.approx(0.0, abs=0.002)


def test_stand_moves_with_ball_offset(env):
    from g1sim import randomization, table_scene

    variation = randomization.nominal(env.objects)
    variation["objects"]["volleyball"].update(dx_m=0.02, dy_m=-0.015)
    env.reset(variation=variation)
    x, y = table_scene.VOLLEYBALL_OBJECTS["volleyball"][4]
    stand = env.data.xpos[env.model.body("volleyball_stand").id]
    assert np.allclose(stand[:2], [x + 0.02, y - 0.015])
    ball = env.object_position("volleyball")
    assert np.allclose(ball[:2], stand[:2], atol=0.002)
    assert ball[2] == pytest.approx(table_scene.object_rest_height("volleyball"), abs=0.003)
    env.reset()  # nominal again: the stand is back at its place
    assert np.allclose(env.data.xpos[env.model.body("volleyball_stand").id][:2], [x, y])


def test_trial_passes_with_the_robot_shifted_and_turned(env):
    """The teacher turns the waist after where ball and basket really are around its waist axis: with the stand 6 cm back,
    4 cm to the side and turned 8 degrees the trial still passes, and the waist drops the ball at a different angle than
    nominal (it turns back to the front afterwards, so the angle is taken at the end of the turn)."""
    from g1sim import ball_grasp, randomization

    variation = randomization.nominal(env.objects)
    variation["stand"] = {"dx_m": -0.06, "dy_m": 0.04, "yaw_deg": 8.0}
    env.reset(variation=variation)
    turned = {}

    def on_tick(phase):
        if phase == "turn":
            turned["yaw"] = env.waist_yaw_target

    result = ball_grasp.TwoHandBallGrasp(env, on_tick=on_tick).run()
    assert result["passed"], (result["first_failed"], result["checks"], result["forbidden_contacts"])
    assert abs(turned["yaw"] - np.radians(ball_grasp.PLACE_WAIST_DEG)) > np.radians(3.0)
    env.reset()
