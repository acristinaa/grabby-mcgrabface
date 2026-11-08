"""Walking world for g1sim/approach.py: the walking model plus a movable table and movable obstacles (MuJoCo side).

Table and obstacles are mocap bodies: they collide like fixed furniture but can be put anywhere between two trials by
writing data.mocap_pos / mocap_quat - the world moves, the walking robot is never teleported. Unused ones stand far away
(PARK). The table is a real table, top slab + four legs, so the toes can go under the top as at a real table; its outline
is the volleyball scene's table (approach.TABLE_HALF, top at approach.TABLE_TOP). The grasp scene (g1sim/table_scene.py)
still uses a solid block - only the walk needs the free space for the feet.
Nothing stands on the table: while walking the arms are at LeRobot's zero pose, the hands 0.30-0.42 m in front of the
pelvis at 0.85-1.0 m height, and at the dock they reach over the table top; a ball or basket there would sit in the hands.
Built on the walking model (scripts/51_build_brainco_sim.py), written next to it as SCENE_FILE.
"""
import math
import os

import mujoco
import numpy as np

from g1sim import MANIFEST, SIM_DIR, approach
from g1sim import SCENE_FILE as WALK_SCENE

SCENE_FILE = "assets/scene_29dof_brainco_approach.xml"
TABLE = "approach_table"
SLAB = 0.04          # m, thickness of the table top
LEG = 0.025          # m, half edge of a square leg
LEG_INSET = 0.05     # m, leg centre from the table corner
TABLE_RGBA = (0.55, 0.42, 0.30, 1.0)
PARK = (60.0, 60.0)  # where unused furniture stands; each piece 3 m further along x


def write(sim_dir=SIM_DIR):
    """Walking model + table + obstacles, all parked, as SCENE_FILE next to the model. Returns SCENE_FILE (relative)."""
    if not (sim_dir / MANIFEST).is_file():
        raise FileNotFoundError(f"{sim_dir} is not built - run scripts/51_build_brainco_sim.py first")
    spec = mujoco.MjSpec.from_file(str(sim_dir / WALK_SCENE))
    (hx, hy), top = approach.TABLE_HALF, approach.TABLE_TOP
    table = _mocap_body(spec, TABLE, 0)
    _box(table, "approach_table_top", [hx, hy, SLAB / 2], [0.0, 0.0, top - SLAB / 2], TABLE_RGBA)
    leg_h = (top - SLAB) / 2
    for i, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
        _box(table, f"approach_table_leg{i}", [LEG, LEG, leg_h],
             [sx * (hx - LEG_INSET), sy * (hy - LEG_INSET), leg_h], TABLE_RGBA)
    for i, (name, ((ox, oy), height, rgba)) in enumerate(approach.OBSTACLES.items(), start=1):
        body = _mocap_body(spec, name, i)
        _box(body, f"{name}_geom", [ox, oy, height / 2], [0.0, 0.0, height / 2], rgba)
    spec.compile()
    path = sim_dir / SCENE_FILE
    tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")  # own file first, then rename (as table_scene.write)
    tmp.write_text(spec.to_xml())
    os.replace(tmp, path)
    return SCENE_FILE


def _mocap_body(spec, name, index):
    body = spec.worldbody.add_body()
    body.name = name
    body.mocap = True
    body.pos = [PARK[0] + 3.0 * index, PARK[1], 0.0]
    return body


def _box(body, name, size, pos, rgba):
    geom = body.add_geom()
    geom.name = name
    geom.type = mujoco.mjtGeom.mjGEOM_BOX
    geom.size = size
    geom.pos = pos
    geom.rgba = rgba
    geom.friction = [1.0, 0.02, 0.001]


class Furniture:
    """Puts the table and the obstacles of an approach.Setup into a running sim, and reports robot contacts with them."""

    def __init__(self, model, data, lock=None):
        """lock: the sim thread's step lock (BraincoEnvMixin._step_lock), so a move never lands in the middle of a step."""
        self.model, self.data, self.lock = model, data, lock
        names = [TABLE, *approach.OBSTACLES]
        self.mocap = {n: int(model.body_mocapid[model.body(n).id]) for n in names}
        self.bodies = {model.body(n).id: n for n in names}
        self.geoms = {g: self.bodies[int(model.geom_bodyid[g])] for g in range(model.ngeom)
                      if int(model.geom_bodyid[g]) in self.bodies}

    def place(self, setup):
        """Table and the setup's obstacles to their poses, all other obstacles to their parking spots."""
        poses = {TABLE: (setup.table.x, setup.table.y, setup.table.yaw)}
        poses.update({o.name: (o.x, o.y, o.yaw) for o in setup.obstacles})
        self._move(poses)

    def park(self):
        self._move({})

    def _move(self, poses):
        with self.lock if self.lock is not None else _nolock():
            for i, (name, mocap) in enumerate(self.mocap.items()):
                x, y, yaw = poses.get(name, (PARK[0] + 3.0 * i, PARK[1], 0.0))
                self.data.mocap_pos[mocap] = [x, y, 0.0]
                self.data.mocap_quat[mocap] = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]

    def touching(self):
        """Names of the furniture pieces a robot part touches right now (floor and furniture among each other do not count)."""
        return {piece for piece, _, _ in self.contacts()}

    def contacts(self):
        """Robot contacts with furniture right now: {(piece, piece geom, robot body)}.
        The sim steps in its own thread: copy the contact pairs in one access, try again next time if that fails."""
        try:
            pairs = np.array(self.data.contact.geom)
        except (IndexError, ValueError):
            return set()
        hits = set()
        for a, b in pairs.reshape(-1, 2):
            for piece, other in ((a, b), (b, a)):
                body = int(self.model.geom_bodyid[other])
                if piece in self.geoms and body != 0 and body not in self.bodies:
                    hits.add((self.geoms[piece], self.model.geom(int(piece)).name, self.model.body(body).name))
        return hits


class _nolock:
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False
