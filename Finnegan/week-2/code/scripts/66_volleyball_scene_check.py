"""Level 1 step 1 - volleyball scene: acceptance check of the scene itself (no grasping yet).

Writes the volleyball scene (g1sim/table_scene.py, SCENES["volleyball"]), resets TableEnv with the arms in READY_POSE and
holds that pose for 3 s. Checks, in this order:
  1. ball_still               ball moved < 3 mm since the reset and is slower than 1 cm/s at the end
  2. ball_on_stand            ball centre within 3 mm of its designed rest point on the stand
  3. no_robot_contact         no robot contact with ball, stand, basket or table during the whole hold
  4. stand_and_basket_static  stand and basket have no joint, are welded to the world and did not move
  5. ball_visible_head_camera head camera at 320 x 200: > 2000 pixels show the ball, none on the image border. Colour-free
                              (the ball is grey like the real one): a segmentation render says which geom each pixel shows
Images: docs/volleyball_scene_head.png (head camera), docs/volleyball_scene_overview.png (free camera). One renderer for
everything: the head-camera renderer of TableEnv, at 640 x 400 (the Locator size of the grasp scripts); the visibility
check scales the ball mask down to 320 x 200, the TableEnv observation size.
Environment ~/dev/g1-ext:
    ~/dev/g1-ext/.venv/bin/python scripts/66_volleyball_scene_check.py [--watch 1]
Lab view (scripts/65_labview.py) shows the run live unless --no-labview.
"""
import argparse
import pathlib
import sys
import warnings

import cv2
import mujoco
import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim import action_space, labview, table_env, table_scene  # noqa: E402

SCENE = "volleyball"
BALL = "volleyball"
HOLD_S = 3.0
MAX_MOVE = 0.003
MAX_SPEED = 0.01
MAX_REST_ERROR = 0.003
MIN_BALL_PIXELS = 2000
HEAD_SIZE = (400, 640)  # (height, width) of the one renderer
CHECK_SIZE = (320, 200)  # (width, height) for the visibility check, as in table_env.IMAGE_SIZES
OVERVIEW = {"lookat": [0.30, 0.0, 0.80], "distance": 2.0, "azimuth": 205.0, "elevation": -32.0}
HEAD_IMAGE = REPO / "docs" / "volleyball_scene_head.png"
OVERVIEW_IMAGE = REPO / "docs" / "volleyball_scene_overview.png"

warnings.filterwarnings("ignore", message=".*parent has")


def overview_camera():
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = OVERVIEW["lookat"]
    cam.distance, cam.azimuth, cam.elevation = OVERVIEW["distance"], OVERVIEW["azimuth"], OVERVIEW["elevation"]
    return cam


def ball_mask(renderer, data, geom):
    """Head-camera pixels that show the geom (segmentation render: object id and type per pixel), at CHECK_SIZE."""
    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera="head_camera")
    seg = renderer.render()
    renderer.disable_segmentation_rendering()
    mask = (seg[:, :, 0] == geom) & (seg[:, :, 1] == mujoco.mjtObj.mjOBJ_GEOM)
    return cv2.resize(mask.astype(np.float32), CHECK_SIZE, interpolation=cv2.INTER_AREA) >= 0.5


def main():
    parser = argparse.ArgumentParser(description="Level 1 step 1 - volleyball scene check")
    parser.add_argument("--watch", type=float, metavar="SPEED",
                        help="lab view: slow down to SPEED x real time (1 = real time) so the run can be followed")
    parser.add_argument("--no-labview", action="store_true", help="write no lab view files (runs/labview/)")
    args = parser.parse_args()

    table_scene.write(scene=SCENE)
    env = table_env.TableEnv(scene=SCENE, image_sizes={"head_camera": HEAD_SIZE})
    m, d = env.model, env.data
    env.reset()  # renders the head camera once: from here on env owns exactly one renderer
    renderer = env._renderers[HEAD_SIZE]
    reporter = None if args.no_labview else labview.Reporter(
        script="66", task="scene_check", object=SCENE, trials=1, argv=sys.argv[1:], renderer_getter=lambda: renderer,
        sim_time_getter=lambda: d.time, watch=args.watch, data=d, track_body=BALL)
    if reporter:
        reporter.trial_start(0)

    static = {name: m.body(name).id for name in table_scene.SCENES[SCENE]["static"]}
    static_start = {name: d.xpos[b].copy() for name, b in static.items()}
    ball_start = env.object_position(BALL)
    contacts = set(env.robot_contacts())
    for _ in range(int(HOLD_S * table_env.CONTROL_HZ)):
        env.apply(action_space.READY_POSE)
        contacts.update(env.robot_contacts())
        if reporter:
            reporter.on_tick("settle")
    if reporter:
        reporter.on_tick("check")

    ball = env.object_position(BALL)
    dof = m.jnt_dofadr[m.joint(f"{BALL}_joint").id]
    moved = float(np.linalg.norm(ball - ball_start))
    speed = float(np.linalg.norm(d.qvel[dof:dof + 3]))
    x, y = table_scene.VOLLEYBALL_OBJECTS[BALL][4]
    rest_error = float(np.linalg.norm(ball - [x, y, table_scene.object_rest_height(BALL)]))
    static_ok = all(m.body_jntnum[b] == 0 and m.body_weldid[b] == 0 and np.array_equal(d.xpos[b], static_start[name])
                    for name, b in static.items())

    head = env.render("head_camera")
    mask = ball_mask(renderer, d, m.geom(f"{BALL}_geom").id)
    visible = int(mask.sum())
    on_border = bool(mask[0].any() or mask[-1].any() or mask[:, 0].any() or mask[:, -1].any())
    renderer.update_scene(d, camera=overview_camera())
    overview = renderer.render()

    checks = {
        "ball_still": moved < MAX_MOVE and speed < MAX_SPEED,
        "ball_on_stand": rest_error < MAX_REST_ERROR,
        "no_robot_contact": not contacts,
        "stand_and_basket_static": static_ok,
        "ball_visible_head_camera": visible > MIN_BALL_PIXELS and not on_border,
    }
    numbers = {
        "ball_still": f"moved {moved * 1000:.2f} mm (< {MAX_MOVE * 1000:.0f}), speed {speed * 100:.3f} cm/s "
                      f"(< {MAX_SPEED * 100:.0f})",
        "ball_on_stand": f"{rest_error * 1000:.2f} mm from the rest point (< {MAX_REST_ERROR * 1000:.0f}), "
                         f"ball at {np.round(ball, 4).tolist()}",
        "no_robot_contact": f"contacts {sorted(contacts)}",
        "stand_and_basket_static": f"bodies {list(static)}, joints {[int(m.body_jntnum[b]) for b in static.values()]}",
        "ball_visible_head_camera": f"{visible} ball pixels at {CHECK_SIZE[0]}x{CHECK_SIZE[1]} (> {MIN_BALL_PIXELS}), "
                                    f"on border: {on_border}",
    }
    for name, ok in checks.items():
        print(f"{'PASS' if ok else 'FAIL'}  {name}: {numbers[name]}")

    HEAD_IMAGE.parent.mkdir(exist_ok=True)
    cv2.imwrite(str(HEAD_IMAGE), head[:, :, ::-1])
    cv2.imwrite(str(OVERVIEW_IMAGE), overview[:, :, ::-1])
    passed = all(checks.values())
    if reporter:
        reporter.trial_end(passed, next((k for k, ok in checks.items() if not ok), None))
        reporter.finish(summary=numbers)
    env.close()
    print(f"saved: {HEAD_IMAGE.relative_to(REPO)}, {OVERVIEW_IMAGE.relative_to(REPO)}")
    print("SCENE CHECK PASSED" if passed else "SCENE CHECK FAILED")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
