"""Image checks of the v2 variation (plan humming-squishing-treasure A2, A3, 2026-11-01): what the head camera sees.

Why it exists: a wider camera pose range (+-0.025 m, +-5 deg per axis) is only useful if the ball, the basket and both palms stay
in the picture at the worst corner of that range; and textures are only useful if they look like different tables and floors
without hiding the ball. Both are judged by eye from one image each, plus a number for the camera.
  --textures          head-camera images of TEXTURE_IMAGES random v2 starts (walk world), labelled with the table and floor
                      material -> docs/videos/dr_textures.png
  --camera-extremes   the teacher runs once per start (CAMERA_STARTS starts, far stances first); at the key moments
                      (MOMENTS) ball centre, basket floor centre and both palms are projected into the head image for every
                      corner of the camera range (2^3 offsets x 2^3 tilts = 64); pass = every point inside the image with
                      >= MIN_MARGIN_PX -> docs/videos/dr_camera_extremes.png (worst start: nominal and worst corner) and a
                      JSON report. The camera pose does not touch the physics, so one teacher run per start covers all 64.
                      --table-dz M puts every checked start's table top M m above table_scene.TABLE_TOP (the lowest and the
                      highest table of objects.table_height_m: the head camera sees the table from closer or farther).
Environment ~/dev/g1-ext:
    ~/dev/g1-ext/.venv/bin/python scripts/88_dr_image_checks.py --textures
    ~/dev/g1-ext/.venv/bin/python scripts/88_dr_image_checks.py --camera-extremes --report runs/l1v2_camera_extremes.json
    ~/dev/g1-ext/.venv/bin/python scripts/88_dr_image_checks.py --camera-extremes --table-dz -0.04 --report runs/l1h2_camera_low.json
"""
import argparse
import itertools
import json
import pathlib
import sys
import warnings

import cv2
import mujoco
import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim import ball_grasp, randomization, table_env, table_scene  # noqa: E402
from g1sim.grasp_ik import hand_frame  # noqa: E402

warnings.filterwarnings("ignore", message=".*parent has")
warnings.filterwarnings("ignore", message=".*encountered in matmul")  # false alarm numpy 2 + Apple Accelerate

SCENE = "volleyball_walk"
CAMERA = "head_camera"
VIDEOS = REPO / "docs" / "videos"
TEXTURE_IMAGES = 12  # plan A3: ~12 variations on the sheet
TEXTURE_SEED = 3
CAMERA_SEED = 0  # eval seed: the starts the held-out eval draws
CAMERA_DRAWS = 60  # starts drawn; the CAMERA_STARTS most extreme are checked
CAMERA_STARTS = 10  # plan A2: >= 10 starts incl. far stances
FAR, SIDE, YAW = 6, 2, 2  # of CAMERA_STARTS: largest arrival back_m, then largest |side_m|, then largest |yaw_deg|
MIN_MARGIN_PX = 10  # plan A2
# moment (last tick of a teacher phase, "handover" = before the first tick) -> points that must be in view then
MOMENTS = {"handover": ("ball", "basket"), "touch": ("ball", "left_palm", "right_palm"),
           "lift": ("ball", "left_palm", "right_palm"), "centre": ("ball", "basket", "left_palm", "right_palm")}
PALM_POINT = (ball_grasp.PALM_X, 0.0, ball_grasp.PALM_Z)  # palm contact point in hand-base coordinates (ball_grasp)


# --- camera pose and projection ----------------------------------------------------------------------------------------
def corners(config):
    """Every corner of the camera range: (offset m xyz, tilt deg xyz), 64 pairs, the nominal pose first."""
    c = config["image"]
    offsets = [np.array(s) * c["camera_offset_m"] for s in itertools.product((-1.0, 1.0), repeat=3)]
    tilts = [np.array(s) * c["camera_tilt_deg"] for s in itertools.product((-1.0, 1.0), repeat=3)]
    return [(np.zeros(3), np.zeros(3))] + [(o, t) for o in offsets for t in tilts]


def set_camera(model, original_pos, original_quat, offset, tilt):
    """Head camera at the original pose moved like randomization.ModelVariation.apply does it."""
    cam = model.camera(CAMERA).id
    model.cam_pos[cam] = original_pos + offset
    mujoco.mju_mulQuat(model.cam_quat[cam], original_quat, randomization._quat(tilt))


def project(model, data, point, size):
    """World point -> (column, row) in the head image of size (height, width), and the margin to the nearest edge (px;
    negative = outside, -inf = behind the camera). MuJoCo cameras look along -z with +y up."""
    cam = model.camera(CAMERA).id
    local = data.cam_xmat[cam].reshape(3, 3).T @ (np.asarray(point) - data.cam_xpos[cam])
    height, width = size
    if local[2] >= 0.0:
        return None, -np.inf
    focal = 0.5 * height / np.tan(np.radians(model.cam_fovy[cam]) / 2.0)
    col, row = 0.5 * width + focal * local[0] / -local[2], 0.5 * height - focal * local[1] / -local[2]
    return (col, row), float(min(col, width - col, row, height - row))


def points(m, d):
    """World positions of the checked points in d (positions computed by mj_kinematics or mj_forward)."""
    basket = d.xpos[m.body("basket").id] + [0.0, 0.0, table_scene.BASKET_WALL]
    out = {"ball": d.xpos[m.body(ball_grasp.BALL).id].copy(), "basket": basket}
    for side in ball_grasp.SIDES:
        body = m.body(hand_frame(side)).id
        out[f"{side}_palm"] = d.xpos[body] + d.xmat[body].reshape(3, 3) @ PALM_POINT
    return out


# --- camera extremes ---------------------------------------------------------------------------------------------------
def pick_starts(env):
    """CAMERA_DRAWS random walk starts (variations), then the FAR / SIDE / YAW most extreme ones."""
    drawn = []
    for _ in range(CAMERA_DRAWS):
        env.reset(randomize=True)
        drawn.append(env.variation)
    chosen = []
    for count, key in ((FAR, lambda v: v["arrival"]["back_m"]), (SIDE, lambda v: abs(v["arrival"]["side_m"])),
                       (YAW, lambda v: abs(v["arrival"]["yaw_deg"]))):
        chosen += [v for v in sorted(drawn, key=key, reverse=True) if v not in chosen][:count]
    return chosen


def teacher_moments(env, variation):
    """Run the teacher from variation; qpos at every moment of MOMENTS (last tick of the phase). Returns (passed, {moment: qpos})."""
    env.reset(variation=variation)
    saved = {"handover": env.data.qpos.copy()}

    def on_tick(phase):
        if phase in MOMENTS:
            saved[phase] = env.data.qpos.copy()

    result = ball_grasp.TwoHandBallGrasp(env, on_tick=on_tick).run()
    return result["passed"], saved


def margins(env, saved, camera_corners, original):
    """Per corner: the smallest margin over all moments and points, and where it was."""
    m, scratch = env.model, mujoco.MjData(env.model)
    size = env.image_sizes[CAMERA]
    rows = []
    for offset, tilt in camera_corners:
        set_camera(m, *original, offset, tilt)
        worst = (np.inf, None)
        for moment, qpos in saved.items():
            scratch.qpos[:] = qpos
            mujoco.mj_kinematics(m, scratch)
            mujoco.mj_camlight(m, scratch)
            where = points(m, scratch)
            for name in MOMENTS[moment]:
                _, margin = project(m, scratch, where[name], size)
                worst = min(worst, (margin, f"{moment}:{name}"), key=lambda w: w[0])
        rows.append({"offset_m": offset.round(4).tolist(), "tilt_deg": tilt.round(2).tolist(),
                     "min_margin_px": round(worst[0], 1), "at": worst[1]})
    set_camera(m, *original, np.zeros(3), np.zeros(3))
    return rows


def render_marked(env, renderer, qpos, corner, original):
    """Head image at qpos with the camera at corner, the checked points of every moment marked (no sensor noise)."""
    m, d = env.model, env.data
    set_camera(m, *original, *corner)
    d.qpos[:] = qpos
    mujoco.mj_forward(m, d)
    renderer.update_scene(d, camera=CAMERA)
    image = renderer.render().copy()
    size = env.image_sizes[CAMERA]
    for name, point in points(m, d).items():
        pixel, _ = project(m, d, point, size)
        if pixel is not None:
            cv2.circle(image, (int(round(pixel[0])), int(round(pixel[1]))), 4, (255, 0, 255), 1)
    set_camera(m, *original, np.zeros(3), np.zeros(3))
    return image


def camera_extremes(args):
    config = randomization.load(args.randomization)
    env = table_env.TableEnv(scene=SCENE, seed=CAMERA_SEED, variation_config=config)
    cam = env.model.camera(CAMERA).id
    original = (env.model.cam_pos[cam].copy(), env.model.cam_quat[cam].copy())
    camera_corners = corners(config)
    starts = []
    for i, variation in enumerate(pick_starts(env)):
        if args.table_dz is not None:
            variation["table"] = {"dz_m": args.table_dz}
        passed, saved = teacher_moments(env, variation)
        rows = margins(env, saved, camera_corners, original)
        worst = min(rows[1:], key=lambda r: r["min_margin_px"])
        starts.append({"start": i, "arrival": variation["arrival"], "teacher_passed": passed, "nominal": rows[0], "worst": worst,
                       "saved": saved, "variation": variation})
        print(f"start {i}: arrival {variation['arrival']}, teacher {'PASS' if passed else 'FAIL'}, nominal min "
              f"{rows[0]['min_margin_px']} px ({rows[0]['at']}), worst corner {worst['min_margin_px']} px ({worst['at']}, "
              f"offset {worst['offset_m']}, tilt {worst['tilt_deg']})", flush=True)
    worst_start = min(starts, key=lambda s: s["worst"]["min_margin_px"])
    min_margin = worst_start["worst"]["min_margin_px"]
    verdict = "PASS" if min_margin >= MIN_MARGIN_PX and all(s["teacher_passed"] for s in starts) else "FAIL"
    sheet_path = camera_sheet(env, worst_start, original)
    env.close()
    report = {"config": str(args.randomization), "seed": CAMERA_SEED, "table_dz_m": args.table_dz, "corners": len(camera_corners) - 1,
              "moments": MOMENTS, "min_margin_required_px": MIN_MARGIN_PX, "min_margin_px": min_margin,
              "worst": {"start": worst_start["start"], **worst_start["worst"]}, "verdict": verdict, "image": str(sheet_path),
              "starts": [{k: v for k, v in s.items() if k not in ("saved", "variation")} for s in starts]}
    if args.report:
        args.report.write_text(json.dumps(report, indent=2))
    print(f"CAMERA EXTREMES {verdict}: min margin {min_margin} px over {len(starts)} starts x {len(camera_corners) - 1} corners "
          f"(need >= {MIN_MARGIN_PX}); worst start {worst_start['start']} at {worst_start['worst']['at']} -> {sheet_path}")
    return 0 if verdict == "PASS" else 1


def camera_sheet(env, start, original):
    """Rows = moments of the worst start; left nominal camera, right the worst corner."""
    env.reset(variation=start["variation"])  # its textures, lights and colours
    renderer = mujoco.Renderer(env.model, *env.image_sizes[CAMERA])
    corner = (np.array(start["worst"]["offset_m"]), np.array(start["worst"]["tilt_deg"]))
    rows = []
    for moment, qpos in start["saved"].items():
        pair = [render_marked(env, renderer, qpos, c, original) for c in ((np.zeros(3), np.zeros(3)), corner)]
        rows.append(np.concatenate([label(pair[0], f"{moment}: nominal"), label(pair[1], f"{moment}: worst corner")], 1))
    renderer.close()
    return save(np.concatenate(rows, 0), VIDEOS / "dr_camera_extremes.png")


# --- textures ----------------------------------------------------------------------------------------------------------
def colour_rule_ok():
    """Every DR_MATERIALS colour keeps r <= g and r <= b (table_scene: the product with a table colour stays not-red)."""
    return all(c[0] <= c[1] and c[0] <= c[2] for _, rgb1, rgb2, _, markrgb, _ in table_scene.DR_MATERIALS.values()
               for c in (rgb1, rgb2, markrgb))


def textures(args):
    env = table_env.TableEnv(scene=SCENE, seed=TEXTURE_SEED, variation_config=randomization.load(args.randomization))
    images = []
    for _ in range(TEXTURE_IMAGES):
        obs = env.reset(randomize=True)
        t = env.variation["image"]["textures"]
        images.append(label(obs["head_image"], f"table {short(t.get('table_top'))} / floor {short(t.get('floor'))}"))
    env.close()
    columns = 4
    grid = np.concatenate([np.concatenate(images[i:i + columns], 1) for i in range(0, len(images), columns)], 0)
    path = save(grid, VIDEOS / "dr_textures.png")
    print(f"colour rule r <= g, b for every material: {colour_rule_ok()}; {len(images)} images -> {path}")
    return 0 if colour_rule_ok() else 1


def short(material):
    return "own" if material is None else material.removeprefix("dr_")


def label(image, text):
    image = image.copy()
    cv2.putText(image, text, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, text, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
    return image


def save(rgb, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), rgb[..., ::-1])
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--textures", action="store_true")
    mode.add_argument("--camera-extremes", action="store_true")
    parser.add_argument("--randomization", type=pathlib.Path, default=randomization.CONFIG)
    parser.add_argument("--report", type=pathlib.Path, help="--camera-extremes: JSON report path")
    parser.add_argument("--table-dz", type=float, help="--camera-extremes: table top this many m above the built one in every start")
    args = parser.parse_args()
    table_scene.write(scene=SCENE)
    return textures(args) if args.textures else camera_extremes(args)


if __name__ == "__main__":
    sys.exit(main())
