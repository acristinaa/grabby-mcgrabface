"""Build g1sim/data/fence_model.xml.gz: the collision-only volleyball_walk scene the 86 fence loads on the robot laptop.

Why: 86's fence (g1sim/fence.py, FENCE-SMOOTH.md 1.1, review B1) needs the robot's and the table scene's collision shapes, but
the robot laptop has no ~/dev/g1-ext/g1_brainco_sim (TableEnv's scene lives there, outside the repo). This file is small enough
for the repo: visual geoms and textures removed, every collision mesh replaced by its convex hull vertices written inline
(MuJoCo collides a mesh by its hull anyway, so distances do not change). Before writing, the hull model's distances are compared
with the full TableEnv model on teacher-like poses (ready pose + random arm poses); the file is written only if they agree
within MAX_DIFF_M. The header comment carries the source scene's sha256 and the MuJoCo version (fence.py refuses another one).
Sim env (MuJoCo 3.13), once, after the scene changed:
    ~/dev/g1-ext/.venv/bin/python tests/helpers/make_fence_model.py
"""
import gzip
import hashlib
import pathlib
import sys
import warnings

import mujoco
import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from g1sim import SIM_DIR, action_space, fence, table_scene  # noqa: E402
from g1sim.table_env import TableEnv  # noqa: E402

warnings.filterwarnings("ignore", message=".*encountered in matmul")  # false alarm numpy 2 + Apple Accelerate

SCENE = "volleyball_walk"
MAX_DIFF_M = 1e-5            # hull vs full model distance; measured 0.0006 mm on 145 teacher poses (/tmp/fence/b1/hull.log)
CHECK_POSES = 60             # random arm/hand poses around the ready pose compared before writing
POSE_SPREAD_RAD = 0.6        # arm joints drawn within +-this of the ready pose (reaches table, stand and basket)
VERTEX_DECIMALS = 6          # hull vertices rounded to 1 um in the XML


def reduced_spec(scene_file: pathlib.Path) -> mujoco.MjSpec:
    """The scene without visual geoms, textures and unused meshes."""
    spec = mujoco.MjSpec.from_file(str(scene_file))
    for geom in list(spec.geoms):
        if geom.contype == 0 and geom.conaffinity == 0:
            spec.delete(geom)
    for texture in list(spec.textures):
        spec.delete(texture)
    for material in list(spec.materials):
        material.textures = [""] * len(material.textures)
    used = {geom.meshname for geom in spec.geoms if geom.meshname}
    for mesh in list(spec.meshes):
        if mesh.name not in used:
            spec.delete(mesh)
    return spec


def hull_spec(scene_file: pathlib.Path) -> mujoco.MjSpec:
    """reduced_spec with every mesh replaced by its convex hull vertices, inline (no mesh file left)."""
    compiled = reduced_spec(scene_file).compile()
    spec = reduced_spec(scene_file)          # fresh: a compiled spec keeps the file's face normals (mesh cache)
    for mesh in spec.meshes:
        i = mujoco.mj_name2id(compiled, mujoco.mjtObj.mjOBJ_MESH, mesh.name)
        graph = compiled.mesh_graphadr[i]
        hull_count = compiled.mesh_graph[graph]
        ids = compiled.mesh_graph[graph + 2 + hull_count: graph + 2 + 2 * hull_count]   # hull vertex ids
        vertices = compiled.mesh_vert[compiled.mesh_vertadr[i] + ids].astype(float)
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, compiled.mesh_quat[i].astype(float))
        # the compiler centred and rotated the mesh: undo it, so the file's frame stays (geom pos/quat refer to it)
        original = vertices @ rotation.reshape(3, 3).T + compiled.mesh_pos[i]
        original = original / np.asarray(mesh.scale if mesh.scale is not None else [1.0, 1.0, 1.0])
        mesh.file = ""
        mesh.uservert = np.round(original, VERTEX_DECIMALS).ravel().tolist()
        mesh.userface = []
        for attr in ("usernormal", "usertexcoord", "userfacenormal", "userfacetexcoord"):
            if hasattr(mesh, attr):
                setattr(mesh, attr, [])
    return spec


def largest_difference(xml: str) -> float:
    """Largest |hull - full| fence value over CHECK_POSES poses in the nominal layout (every group, cmd pose)."""
    env = TableEnv(scene=SCENE)
    env.reset(randomize=False)
    full = fence.Fence.from_env(env)
    hull = fence.Fence(mujoco.MjModel.from_xml_string(xml))
    hull.set_base(env.data.qpos)
    rng = np.random.default_rng(0)
    ready17 = np.concatenate([[0.0], env.data.qpos[env.body_qadr[13:15]], action_space.BALL_READY_POSE[0:14]])
    worst = 0.0
    for _ in range(CHECK_POSES):
        q17 = ready17.copy()
        q17[3:] += rng.uniform(-POSE_SPREAD_RAD, POSE_SPREAD_RAD, 14)
        q17[0] = rng.uniform(-0.3, 0.3)
        hands = rng.uniform(0.0, 1.0, 12)
        full.set_base(env.data.qpos)
        a, b = full.pose_values(q17, hands), hull.pose_values(q17, hands)
        if a.keys() != b.keys():
            raise SystemExit(f"groups differ: full {sorted(a)} hull {sorted(b)}")
        worst = max(worst, max(abs(a[g] - b[g]) for g in a))
    env.close()
    return worst


def main() -> int:
    scene_file = SIM_DIR / table_scene.SCENES[SCENE]["file"]
    if mujoco.__version__ != fence.MUJOCO_VERSION:
        print(f"MuJoCo {mujoco.__version__}, the fence needs {fence.MUJOCO_VERSION} - run in ~/dev/g1-ext/.venv")
        return 1
    xml = hull_spec(scene_file).to_xml()
    worst = largest_difference(xml)
    print(f"hull vs full model: largest fence value difference {worst * 1000:.4f} mm over {CHECK_POSES} poses")
    if worst > MAX_DIFF_M:
        print(f"NOT WRITTEN: above {MAX_DIFF_M * 1000:.3f} mm")
        return 1
    digest = hashlib.sha256(scene_file.read_bytes()).hexdigest()
    header = (f"<!-- g1sim fence model: collision hulls of {scene_file.name} sha256 {digest}, MuJoCo {mujoco.__version__}; "
              f"built by tests/helpers/make_fence_model.py - do not edit -->\n")
    data = gzip.compress((header + xml).encode(), mtime=0)          # mtime 0: the same scene gives the same bytes
    fence.MODEL_FILE.parent.mkdir(exist_ok=True)
    fence.MODEL_FILE.write_bytes(data)
    print(f"wrote {fence.MODEL_FILE.relative_to(REPO)}: {len(data) / 1e6:.2f} MB gzip, {len(xml) / 1e6:.2f} MB XML")
    return 0


if __name__ == "__main__":
    sys.exit(main())
