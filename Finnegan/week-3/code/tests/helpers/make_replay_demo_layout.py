"""Write tests/data/replay_demo_layout.json: the table layout tests/data/replay_demo.npz was recorded in.

Why: the fence tests and the 86 replay gate (85 e2e) must run the fence in the demo's own layout (review B2: in the nominal
layout the demo passes 1.70 cm from the stand, inside the 2 cm margin). The demo is dataset episode 0 = episode_s101_t0000:
v1 config, seed 101, first reset. Its layout is regenerated here and checked by the head image of frame 0 against the
recorded one (tests/data/selftest_episode.npz); the file is written only when they agree (mean |diff| < MAX_IMAGE_MAD; the
next draw and the nominal layout give 42.9 / 46.4, FENCE-SMOOTH.md 1.3). Also written: where the demo's stand stood relative
to the nominal stand in the robot's frame (R-F8, the runbook's replay placement).
Sim env, once:  ~/dev/g1-ext/.venv/bin/python tests/helpers/make_replay_demo_layout.py
"""
import json
import pathlib
import sys
import warnings

import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from g1sim import randomization, table_env  # noqa: E402

warnings.filterwarnings("ignore", message=".*encountered in matmul")  # false alarm numpy 2 + Apple Accelerate

SEED = 101                   # episode_s101_t0000 (convert map: dataset episode 0)
CONFIG = REPO / "configs" / "randomization_v1.json"   # run1's demos were recorded with v1
MAX_IMAGE_MAD = 10.0         # grey levels; regenerated 3.76, wrong layouts 43-46 (/tmp/fence/b2/demo_layout.log)
OUT = REPO / "tests" / "data" / "replay_demo_layout.json"
KEYS = ("stance", "objects", "basket", "table")   # what fence.Fence.nominal reads


def stand_in_robot_frame(env) -> np.ndarray:
    """(x forward, y left) of the stand centre relative to the pelvis, in the pelvis yaw frame (m)."""
    m, d = env.model, env.data
    pelvis = m.body("pelvis").id
    yaw = np.arctan2(d.xmat[pelvis][3], d.xmat[pelvis][0])
    rel = d.xpos[m.body("volleyball_stand").id][:2] - d.xpos[pelvis][:2]
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([c * rel[0] + s * rel[1], -s * rel[0] + c * rel[1]])


def main() -> int:
    recorded = np.load(REPO / "tests" / "data" / "selftest_episode.npz")["head_image"][0].astype(float)
    env = table_env.TableEnv(scene="volleyball_walk", seed=SEED, variation_config=randomization.load(CONFIG))
    obs = env.reset(randomize=True)
    mad = float(np.abs(obs["head_image"].astype(float) - recorded).mean())
    variation = json.loads(json.dumps(env.variation, default=float))
    demo_stand = stand_in_robot_frame(env)
    env.reset(randomize=False)
    nominal_stand = stand_in_robot_frame(env)
    env.close()
    print(f"head image frame 0: mean |diff| {mad:.2f} (limit {MAX_IMAGE_MAD})")
    if mad >= MAX_IMAGE_MAD:
        print("NOT WRITTEN: the regenerated layout does not match the recorded demo")
        return 1
    offset_cm = np.round((demo_stand - nominal_stand) * 100, 1)
    layout = {key: variation[key] for key in KEYS if key in variation}
    layout["source"] = {"episode": "episode_s101_t0000", "seed": SEED, "config": CONFIG.name, "reset": 1,
                        "head_image_mad": round(mad, 2), "built_by": "tests/helpers/make_replay_demo_layout.py"}
    layout["stand_vs_nominal_robot_frame_cm"] = {"forward": float(offset_cm[0]), "left": float(offset_cm[1])}
    OUT.write_text(json.dumps(layout, indent=2) + "\n")
    print(f"wrote {OUT.relative_to(REPO)}; stand vs nominal in the robot's frame: {offset_cm[0]:+.1f} cm forward, "
          f"{offset_cm[1]:+.1f} cm left")
    return 0


if __name__ == "__main__":
    sys.exit(main())
