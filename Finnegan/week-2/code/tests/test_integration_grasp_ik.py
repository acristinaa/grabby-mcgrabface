"""Integration: the IK for the right arm (g1sim/grasp_ik.py) reaches the grasp pose at the bottle, using wrist pitch/yaw.
Our G1 has 29 DoF (confirmed 2026-10-07), so the locked-wrist (23 DoF) case is no longer tested: the rolled D2 grasp cannot
be reached without wrist pitch/yaw. The full grasp sequence is checked by the acceptance script scripts/54_grasp_scripted.py.
"""
import numpy as np
import pytest

from conftest import needs_g1ext

pytestmark = [pytest.mark.integration, needs_g1ext]


@pytest.fixture(scope="module")
def env():
    pytest.importorskip("mink")
    from g1sim import MANIFEST, SIM_DIR, table_env, table_scene

    if not (SIM_DIR / MANIFEST).is_file():
        pytest.skip("Laufmodell fehlt - zuerst scripts/51_build_brainco_sim.py")
    table_scene.write()
    e = table_env.TableEnv()
    e.reset()
    yield e
    e.close()


def test_ik_erreicht_greifpose(env):
    from g1sim import action_space, grasp
    from g1sim.grasp_ik import ArmIK

    ik = ArmIK(env.model)
    qpos = env.data.qpos.copy()
    qpos[ik.arm_qadr] = action_space.READY_ARM
    ik.reset(qpos)
    arm, pos_err, rot_err = ik.solve(grasp.grasp_pose(grasp.bottle_top(env)))

    assert pos_err < 0.01 and np.rad2deg(rot_err) < 10.0
    ranges = env.model.jnt_range[[env.model.joint(n).id for n in action_space.SIM_ARM_JOINTS["right"]]]
    assert np.all((arm >= ranges[:, 0]) & (arm <= ranges[:, 1]))


def ready_ik(env):
    from g1sim import action_space
    from g1sim.grasp_ik import ArmIK

    ik = ArmIK(env.model)
    qpos = env.data.qpos.copy()
    qpos[ik.arm_qadr] = action_space.READY_ARM
    ik.reset(qpos)
    return ik


def arm_trunk_distance(env, ik):
    """Kleinster Abstand Arm <-> Rumpf in der IK-Stellung (m), gemessen wie in der IK selbst."""
    import mujoco

    data = mujoco.MjData(env.model)
    data.qpos[:] = ik.configuration.q
    mujoco.mj_forward(env.model, data)
    limit = ik.limits[-1]
    return min(mujoco.mj_geomDistance(env.model, data, a, b, 0.1, None) for a, b in limit.geom_id_pairs)


def test_ik_haelt_abstand_zum_rumpf(env):
    """Befund 1.5b: Flasche nahe der Koerpermitte -> ohne Grenze steckte der Oberarm 14-21 mm im Rumpf.

    Zugesagt ist "kein Eindringen": Die Grenze wirkt auf Geschwindigkeiten (linear), gemessen bleiben hier 0,4 mm
    der Soll-5-mm uebrig.
    """
    from g1sim import grasp, randomization

    v = randomization.nominal()
    v["objects"]["red_bottle"].update(dx_m=-0.02, dy_m=0.14)  # Flasche bei y = -0,02 m, vor der Koerpermitte
    env.reset(variation=v)
    ik = ready_ik(env)
    ik.solve(grasp.grasp_pose(grasp.bottle_top(env)))
    env.reset()
    assert arm_trunk_distance(env, ik) >= 0.0


def test_ik_springt_nicht_bei_unerreichbarem_ziel(env):
    """Befund 1.5b: Ohne Geschwindigkeitsgrenze sprang die IK in einem Schritt 9,6 cm durch den Rumpf."""
    from g1sim import grasp
    from g1sim.grasp_ik import MAX_JOINT_SPEED

    ik = ready_ik(env)
    far = grasp.shifted(grasp.grasp_pose(grasp.bottle_top(env)), [0.4, 0.3, 0.0])  # 50 cm neben der Flasche
    before = ik.arm_q
    for _ in range(60):  # ein Regeltakt je step()
        after = ik.step(far)
        assert np.abs(after - before).max() <= MAX_JOINT_SPEED * ik.dt + 1e-6
        before = after
    assert ik.error(far)[0] > 0.1  # bleibt unerreichbar, ohne Absturz des Loesers
    assert arm_trunk_distance(env, ik) >= 0.0


def test_freimachen_hand_aus_dem_bild(env):
    """Lernpfad L3a: Vor dem ersten Hinsehen faehrt die rechte Hand um grasp.LOOK_OFFSET aus dem Bild der Kopfkamera.
    Aendert jemand Startpose, Kopfneigung oder Kamera, kann die Hand still wieder die Flasche verdecken (Befund 1.5b)."""
    import mujoco

    from g1sim import action_space, grasp, table_env
    from g1sim.grasp_ik import ArmIK, pose

    m = env.model
    arm_bodies = {m.body("right_shoulder_pitch_link").id}
    for b in range(m.nbody):  # alle Koerper unterhalb der Schulter: Arm, Hand, Finger
        p = b
        while p > 0 and p not in arm_bodies:
            p = m.body_parentid[p]
        if p > 0:
            arm_bodies.add(b)
    arm_geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] in arm_bodies]
    seg = mujoco.Renderer(m, *table_env.IMAGE_SIZES["head_camera"])
    seg.enable_segmentation_rendering()

    def pixels():
        seg.update_scene(env.data, camera="head_camera")
        ids = seg.render()[..., 0]
        return int(np.isin(ids, arm_geoms).sum()), int((ids == m.geom("red_bottle_geom").id).sum())

    env.reset()
    ik = ArmIK(m, "right", False, dt=1.0 / table_env.CONTROL_HZ)
    qpos = env.data.qpos.copy()
    qpos[ik.arm_qadr] = action_space.READY_POSE[grasp.ARM]
    ik.reset(qpos)
    for _ in range(grasp.ticks(1.0)):
        env.apply(action_space.READY_POSE)
    assert pixels()[0] > 1000  # Gegenprobe: in der Startpose ist die Hand im Bild

    ready = ik.hand_pose()
    pelvis_rot = env.data.xmat[m.body("pelvis").id].reshape(3, 3)
    target = pose(ready.rotation().as_matrix(), ready.translation() + pelvis_rot @ grasp.LOOK_OFFSET)
    for _ in range(150):
        arm = ik.step(target)
    action = action_space.READY_POSE.copy()
    action[grasp.ARM] = arm
    for _ in range(grasp.ticks(grasp.FREE_S)):
        env.apply(action)
    hand_px, bottle_px = pixels()
    seg.close()
    env.reset()
    assert hand_px == 0 and bottle_px > 500
