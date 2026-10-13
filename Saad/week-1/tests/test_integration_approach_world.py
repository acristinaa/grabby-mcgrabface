"""Integration: walking world for the walk up to the table (g1sim/approach_world.py) and the live view's markers.

Guarded: the table has the volleyball scene's outline and top height and free space under the top for the feet; place()
moves table and obstacles to an approach.Setup and parks the rest; contacts with the robot are reported, the floor and
furniture standing on it are not. Also: LiveView markers must not inherit garbage from the scene slot they are drawn into
(mjr_render crashed with a bus error in 68, 2026-10-08).
Needs the built walking model (scripts/51_build_brainco_sim.py), no LeRobot.
"""
import math

import mujoco
import numpy as np
import pytest

from g1sim.navigation import Pose2D

pytestmark = pytest.mark.integration


@pytest.fixture(scope="module")
def world():
    from g1sim import MANIFEST, SIM_DIR, approach_world

    if not (SIM_DIR / MANIFEST).is_file():
        pytest.skip("walking model missing - run scripts/51_build_brainco_sim.py first")
    model = mujoco.MjModel.from_xml_path(str(SIM_DIR / approach_world.write()))
    data = mujoco.MjData(model)
    return model, data, approach_world.Furniture(model, data)


def test_table_outline_top_and_legs(world):
    from g1sim import approach

    model, data, furniture = world
    furniture.place(approach.Setup("t", Pose2D(0.0, 0.0, 0.0)))
    mujoco.mj_forward(model, data)
    top = model.geom("approach_table_top").id
    assert np.allclose(model.geom_size[top][:2], approach.TABLE_HALF)
    assert data.geom_xpos[top][2] + model.geom_size[top][2] == pytest.approx(approach.TABLE_TOP)
    legs = [model.geom(f"approach_table_leg{i}").id for i in range(4)]
    for leg in legs:  # legs stand on the floor and end under the top
        assert data.geom_xpos[leg][2] - model.geom_size[leg][2] == pytest.approx(0.0, abs=1e-9)
        assert data.geom_xpos[leg][2] + model.geom_size[leg][2] == pytest.approx(data.geom_xpos[top][2] - model.geom_size[top][2])


def test_place_moves_the_setup_and_parks_the_rest(world):
    from g1sim import approach, approach_world

    model, data, furniture = world
    setup = approach.Setup("t", Pose2D(3.0, -1.0, math.radians(30)),
                           (approach.Placed("obstacle_crate", 1.0, 2.0, math.radians(-45)),))
    furniture.place(setup)
    mujoco.mj_forward(model, data)
    table = model.body(approach_world.TABLE).id
    assert np.allclose(data.xpos[table][:2], [3.0, -1.0])
    assert math.atan2(data.xmat[table][3], data.xmat[table][0]) == pytest.approx(math.radians(30))
    assert np.allclose(data.xpos[model.body("obstacle_crate").id][:2], [1.0, 2.0])
    for name in ("obstacle_cabinet", "obstacle_bin"):
        assert np.linalg.norm(data.xpos[model.body(name).id][:2]) > 50.0
    furniture.park()
    mujoco.mj_forward(model, data)
    assert np.linalg.norm(data.xpos[table][:2]) > 50.0


def test_contacts_with_the_robot_are_reported(world):
    """Table right on top of the standing robot -> contact with a robot body; parked -> none (floor does not count)."""
    from g1sim import approach

    model, data, furniture = world
    mujoco.mj_resetData(model, data)
    furniture.park()
    mujoco.mj_forward(model, data)
    assert furniture.touching() == set()
    furniture.place(approach.Setup("t", Pose2D(0.2, 0.0, 0.0)))  # top slab through the hands and arms
    mujoco.mj_forward(model, data)
    hits = furniture.contacts()
    assert {piece for piece, _, _ in hits} == {"approach_table"}
    assert all(body != "world" for _, _, body in hits)
    furniture.park()


def test_live_view_marker_does_not_inherit_slot_garbage(world):
    """Fill the scene slots with nonsense first: the marker's disc and arrow must come out fully initialized."""
    from g1sim.liveview import LiveView

    model, data, _ = world
    view = LiveView(model, data, serve=False)
    try:
        view.renderer.update_scene(data, camera=view.cam)
        scene = view.renderer.scene
        start = scene.ngeom
        for i in range(start, start + 2):
            scene.geoms[i].matid, scene.geoms[i].dataid, scene.geoms[i].objtype = 9999, 9999, 99
        view._add_marker(1.0, 0.0, 0.5, (0.9, 0.4, 0.0, 0.8))
        assert scene.ngeom == start + 2
        for i in range(start, start + 2):
            assert scene.geoms[i].matid == -1 and scene.geoms[i].dataid == -1
        assert scene.geoms[start + 1].type == mujoco.mjtGeom.mjGEOM_ARROW
        view.renderer.render()
    finally:
        view.close()
