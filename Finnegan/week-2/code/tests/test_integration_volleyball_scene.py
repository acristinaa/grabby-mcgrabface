"""Integration: volleyball scene (Level 1, g1sim/table_scene.py SCENES["volleyball"]) - ball, stand, basket, resting ball.

Also guards the bottle scene: table_scene.write() without a scene must still write exactly the bottle scene.
Needs the built walking model (scripts/51_build_brainco_sim.py); the test writes the scenes itself.
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


def test_ball_is_a_hollow_sphere(env):
    m = env.model
    geom, body = m.geom("volleyball_geom"), m.body("volleyball")
    assert geom.type[0] == mujoco.mjtGeom.mjGEOM_SPHERE
    assert geom.size[0] == pytest.approx(0.105)
    assert geom.condim[0] == 6
    assert np.allclose(geom.rgba, [0.55, 0.55, 0.55, 1.0])  # grey like the real ball
    assert body.mass[0] == pytest.approx(0.27)
    assert np.allclose(body.inertia, 2.0 / 3.0 * 0.27 * 0.105 ** 2)
    assert m.jnt_type[m.joint("volleyball_joint").id] == mujoco.mjtJoint.mjJNT_FREE


def test_stand_and_basket_are_static(env):
    m = env.model
    for name in ("volleyball_stand", "basket"):
        body = m.body(name).id
        assert m.body_jntnum[body] == 0, name
        assert m.body_weldid[body] == 0, name
        assert body in env.scene_bodies, f"{name}: contacts with it must be reported"


def test_table_legs_are_visual_only_and_the_collision_box_is_unchanged(env):
    """Proves: the table's slab and legs (table_scene.TABLE_STYLES) make no contacts and add no mass, and the collision box
    "table_top" is still geom 0 of body "table" with the old size, place, friction and mass - the physics of the solid block.
    Catches: someone giving the legs or the slab collisions or mass, or moving, resizing or reordering the box."""
    from g1sim import table_scene

    m = env.model
    table = m.body("table").id
    geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] == table]
    box = m.geom("table_top")
    centre_x, half = table_scene.SCENES["volleyball"]["table"]
    assert geoms[0] == box.id
    assert np.allclose(m.body_pos[table], [centre_x, 0.0, table_scene.TABLE_TOP / 2]) and np.allclose(box.pos, 0.0)
    assert np.allclose(box.size, [*half, table_scene.TABLE_TOP / 2])
    assert np.allclose(box.friction, [1.0, 0.02, 0.001]) and box.contype[0] and box.conaffinity[0]
    assert m.body_mass[table] == pytest.approx(1000.0 * np.prod(2 * box.size))  # MuJoCo's default density, box only
    visuals = geoms[1:]
    assert {m.geom(g).name for g in visuals} == {"table_slab", *(f"table_leg_{i}" for i in range(4))}
    assert not m.geom_contype[visuals].any() and not m.geom_conaffinity[visuals].any()


def test_ball_rests_on_stand(env):
    from g1sim import action_space, table_env, table_scene

    env.reset()
    start = env.object_position("volleyball")
    for _ in range(table_env.CONTROL_HZ):  # 1 s
        env.apply(action_space.READY_POSE)
    ball = env.object_position("volleyball")
    assert np.linalg.norm(ball - start) < 0.003
    assert ball[2] == pytest.approx(table_scene.object_rest_height("volleyball"), abs=0.003)
    assert env.lift_height() == pytest.approx(0.0, abs=0.003)  # default target object is the ball


def test_randomization_varies_every_group(env):
    """Volleyball scene: randomize=True varies ball, stand, physics, image and the distractors - not only the distractors."""
    from g1sim import randomization

    env.reset(randomize=True)
    nominal = randomization.nominal(env.objects)
    for group in randomization.GROUPS:
        assert env.variation[group] != nominal[group], group
    assert env.variation["image"]["object_rgb"]["volleyball"] is not None  # the target's colour is jittered, stays red
    env.reset()


def test_volleyball_ranges_come_from_the_scenes_block(env):
    """Ball offsets stay within the volleyball override (narrower than the bottle range), stand within the stand group."""
    from g1sim import randomization

    config = randomization.load()
    ball_offset = randomization.for_scene(config, "volleyball")["objects"]["offset_m"]
    assert ball_offset < config["objects"]["offset_m"]
    rng = np.random.default_rng(0)
    for _ in range(200):
        v = randomization.sample(config, rng, "volleyball")
        o = v["objects"]["volleyball"]
        assert max(abs(o["dx_m"]), abs(o["dy_m"])) <= ball_offset
        assert abs(v["stand"]["dx_m"]) <= config["stand"]["forward_m"]


def test_random_starts_keep_the_ball_in_reach(env):
    """reset(randomize=True) redraws starts where stand and ball offset add up to a ball out of the teacher's reach."""
    from g1sim import table_scene

    low, high = table_scene.SCENES["volleyball"]["reach_m"]
    pelvis = env.model.body("pelvis").id
    for _ in range(30):
        env.reset(randomize=True)
        distance = np.linalg.norm(env.object_position("volleyball")[:2] - env.data.xpos[pelvis][:2])
        assert low <= distance <= high
    env.reset()


def test_unknown_scene_override_is_refused(tmp_path):
    import json

    from g1sim import randomization

    for scenes, word in (({"tennis": {}}, "tennis"), ({"volleyball": {"objects": {"enabled": False}}}, "enabled"),
                         ({"volleyball": {"objects": {"radius": 1}}}, "radius")):
        config = json.loads(randomization.CONFIG.read_text())
        config["scenes"] = scenes
        path = tmp_path / "randomization.json"
        path.write_text(json.dumps(config))
        with pytest.raises(ValueError, match=word):
            randomization.load(path)


def test_default_write_is_the_bottle_scene():
    """write() without a scene writes the bottle scene, unchanged: same XML as build_spec of the bottle scene."""
    from g1sim import SIM_DIR, build, table_scene

    table_scene.write()
    spec = table_scene.build_spec(build.snapshot(), "bottle")
    spec.compile()
    assert (SIM_DIR / table_scene.SCENE_FILE).read_text() == spec.to_xml()
    model = mujoco.MjModel.from_xml_path(str(SIM_DIR / table_scene.SCENE_FILE))
    names = {model.body(b).name for b in range(model.nbody)}
    assert set(table_scene.OBJECTS) <= names
    assert not names & {"volleyball", "volleyball_stand", "basket"}
    assert model.body("table").pos[0] == pytest.approx(table_scene.TABLE_CENTER_X)
    assert np.allclose(model.geom("table_top").size[:2], table_scene.TABLE_HALF)


def test_table_height_moves_the_top_and_everything_on_it_and_reset_restores_it(env):
    """Proves: a variation's table offset (objects.table_height_m) lifts the collision block, the visible slab, the stand, the
    basket, the ball and a distractor by the same dz, the legs still reach the floor, the lift is measured from the new rest
    height, and reset(variation=...) back to the nominal start restores the built table exactly.
    Catches: something left at the old height (a stand or a ball floating or sunk into the table, lift_height off by dz) or an
    offset that survives into the next episode."""
    import copy

    from g1sim import randomization, table_scene

    m, d = env.model, env.data
    top, slab, leg = m.geom("table_top").id, m.geom("table_slab").id, m.geom("table_leg_0").id
    built = {field: getattr(m, field).copy() for field in ("body_pos", "geom_pos", "geom_size")}
    nominal = randomization.nominal(env.objects)
    nominal["distractors"] = {"distractor_box": {"x_m": 0.55, "y_m": 0.0, "yaw_deg": 0.0}}
    dz = 0.03
    raised = copy.deepcopy(nominal)
    raised["table"] = {"dz_m": dz}
    heights = {}
    for name, v in (("nominal", nominal), ("raised", raised)):
        env.reset(variation=v)
        heights[name] = {"block": d.geom_xpos[top][2] + m.geom_size[top][2], "slab": d.geom_xpos[slab][2] + m.geom_size[slab][2],
                         "stand": d.xpos[m.body("volleyball_stand").id][2], "basket": d.xpos[m.body("basket").id][2],
                         "ball": env.object_position("volleyball")[2], "box": env.object_position("distractor_box")[2]}
        assert d.geom_xpos[leg][2] - m.geom_size[leg][2] == pytest.approx(0.0, abs=1e-9), name  # legs on the floor
        assert env.lift_height() == pytest.approx(0.0, abs=0.003), name
    assert heights["nominal"]["block"] == pytest.approx(table_scene.TABLE_TOP)
    for part, z in heights["raised"].items():
        assert z - heights["nominal"][part] == pytest.approx(dz, abs=0.002), part
    env.reset(variation=nominal)
    for field, value in built.items():
        assert np.array_equal(getattr(m, field), value), field
    env.reset()


def test_render_look_changes_the_image_and_not_the_physics(env):
    """Proves: image.render_look "natural" (table_scene.RENDER_LOOKS) changes the head image in both table styles while every
    physics step stays bit-identical to "basic", and "basic" is the look of the file as built (lamps off, no look material).
    Catches: a look that touches the physics (friction, margins, contacts of a lamp or a material), or one that silently
    does nothing."""
    import copy

    from g1sim import action_space, randomization, table_scene

    m = env.model
    runs = {}
    for style in table_scene.TABLE_STYLES:
        for look in table_scene.RENDER_LOOKS:
            v = randomization.nominal(env.objects)
            v["image"].update(render_look=look, table_style=style)
            obs = env.reset(variation=copy.deepcopy(v))
            if look == "basic":
                assert not m.light_active[[m.light(name).id for name in table_scene.LOOK_LIGHTS]].any()
                assert m.mat(table_scene.LOOK_MATERIAL).id not in m.geom_matid
            qpos = [env.data.qpos.copy()]
            for _ in range(10):
                env.apply(action_space.BALL_READY_POSE)
                qpos.append(env.data.qpos.copy())
            runs[style, look] = (obs["head_image"], np.array(qpos))
    for style in table_scene.TABLE_STYLES:
        (natural, q_natural), (basic, q_basic) = runs[style, "natural"], runs[style, "basic"]
        assert np.array_equal(q_natural, q_basic), style
        assert np.abs(natural.astype(int) - basic.astype(int)).mean() > 5, style
    env.reset()
