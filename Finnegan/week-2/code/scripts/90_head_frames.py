"""Full-resolution head-camera frames with ground-truth labels, for the Level 2 brain track (QR reading, object recognition).

Why: the VLA sees the head camera at 200 x 320 (policy_wire.IMAGE_SHAPE), which is too small for QR codes and for a vision-language
model to name objects. The brain reads one full frame per task, at the D455 colour mode the robot captures (real_camera.CAPTURE_W/H,
1280 x 800). This script renders such frames from random sim starts and writes what is really on the table next to each image,
so look() / read_boxes() can be scored without the robot.

Per frame: frame_<seed>_<i>.jpg + frame_<seed>_<i>.json = {scene, seed, index, image size, objects: [{name, shape, size_m, rgba,
xyz_m}]}. Only bodies on or near the table are listed (parked distractors stay 10 m away, see table_scene).

    ~/dev/g1-ext/.venv/bin/python scripts/90_head_frames.py --scene volleyball --count 20 --seed 0 --out runs/l2_head_frames
"""
import argparse
import json
import pathlib
import sys

import cv2
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from g1sim import randomization, table_env, table_scene  # noqa: E402

FULL_H, FULL_W = 800, 1280        # real_camera.CAPTURE_H/W: the D455 colour mode with the sim's 16:10 field of view
NEAR_TABLE_M = 2.0                # bodies farther than this from the robot are parked, not part of the scene
JPEG_QUALITY = 92                 # high enough that QR edges survive compression, small enough to share a sample set
SHAPES = {0: "plane", 2: "sphere", 3: "capsule", 4: "ellipsoid", 5: "cylinder", 6: "box", 7: "mesh"}  # mjtGeom values


def make_env(scene, seed):
    """TableEnv whose head camera renders at full resolution (the model's offscreen buffer is 640 wide by default)."""
    table_scene.write(scene=scene)
    env = table_env.TableEnv(scene=scene, seed=seed, variation_config=randomization.load(),
                             image_sizes={**table_env.IMAGE_SIZES, "head_camera": (FULL_H, FULL_W)})
    env.model.vis.global_.offwidth, env.model.vis.global_.offheight = FULL_W, FULL_H
    return env


def rounded(values, digits):
    """Plain floats for JSON: MuJoCo's float32 colours would print as 0.5669999718666077 instead of 0.567."""
    return [round(float(v), digits) for v in values]


def object_labels(env):
    """Name, shape, size, colour and world position of every scene object near the robot (sim truth)."""
    m, d = env.model, env.data
    robot = d.xpos[m.body("pelvis").id]
    names = list(env.objects) + list(env.distractors) + ["basket"]
    labels = []
    for name in names:
        try:
            body = m.body(name).id
        except KeyError:
            continue
        xyz = d.xpos[body]
        if np.linalg.norm(xyz[:2] - robot[:2]) > NEAR_TABLE_M:
            continue
        geom = m.body_geomadr[body]
        labels.append({"name": name, "shape": SHAPES.get(int(m.geom_type[geom]), "other"),
                       "size_m": rounded(m.geom_size[geom], 4), "rgba": rounded(m.geom_rgba[geom], 3), "xyz_m": rounded(xyz, 4)})
    return labels


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scene", default="volleyball", choices=sorted(table_scene.SCENES))
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", type=pathlib.Path, default=pathlib.Path("runs/l2_head_frames"))
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    env = make_env(args.scene, args.seed)
    for i in range(args.count):
        env.reset(randomize=True)
        stem = args.out / f"frame_{args.seed}_{i:03d}"
        cv2.imwrite(f"{stem}.jpg", env.render("head_camera")[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        label = {"scene": args.scene, "seed": args.seed, "index": i, "image_hw": [FULL_H, FULL_W], "objects": object_labels(env)}
        pathlib.Path(f"{stem}.json").write_text(json.dumps(label, indent=1))
        print(f"{stem.name}: {', '.join(o['name'] for o in label['objects'])}", flush=True)
    env.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
