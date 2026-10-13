"""Stufe 1.5b - Zufallsvariation (Domain Randomization) der Greifszene.

Die Bereiche stehen in configs/randomization.json, nicht im Code. Je Versuch:
  sample(config, rng)      zieht eine Variation - ein dict aus einfachen Zahlen (passt ins Ergebnis-JSON und reicht,
                           um denselben Start spaeter wiederherzustellen, z. B. fuer ein Replay)
  ModelVariation.apply(v)  schreibt Standpose, Masse, Reibung, Licht, Farben und Kamerapose ins MjModel
Objektlage, Motorgains, Latenz und Bildrauschen setzt TableEnv selbst (Zustand bzw. Regler, nicht Modell).
Distractors (volleyball scene): sample(..., scene="volleyball") puts 1-3 objects of table_scene.VOLLEYBALL_DISTRACTORS on the
table, clear of the ball, the basket and the arc the ball is carried along; TableEnv places them (state, not model).
Every group works in every scene (scene objects, target and table from table_scene.SCENES). A scene can narrow the ranges in
the config block "scenes" (e.g. "volleyball": {"objects": {"offset_m": 0.03}}): for_scene() lays those over the groups.
"""
import json
import pathlib

import mujoco
import numpy as np

from g1sim import table_scene

CONFIG = pathlib.Path(__file__).resolve().parent.parent / "configs" / "randomization.json"
GROUPS = ("objects", "stand", "physics", "image", "distractors")
KEYS = {
    "objects": {"enabled", "offset_m", "yaw_deg", "table_margin_m", "min_gap_m"},
    "stand": {"enabled", "forward_m", "sideways_m", "yaw_deg"},
    "physics": {"enabled", "mass_scale", "friction_scale", "gain_scale", "latency_s"},
    "image": {"enabled", "light_scale", "light_tilt_deg", "light_tint", "table_rgb", "distractor_rgb", "max_red_ratio",
              "bottle_rgb_jitter", "camera_offset_m", "camera_tilt_deg", "rgb_noise_std", "depth_noise_std_m"},
    "distractors": {"enabled", "count", "yaw_deg", "table_margin_m", "min_gap_m", "keepout_m"},
    "start": {"min_object_clearance_m", "min_table_clearance_m", "max_draws"},
}
BODY_MOTORS = 29
MAX_OBJECT_DRAWS = 1000


def load(path=CONFIG, groups=None):
    """Konfiguration lesen und pruefen: jede Gruppe mit genau den bekannten Schluesseln.

    groups: nur diese Gruppen einschalten (None = wie in der Datei).
    """
    path = pathlib.Path(path)
    config = json.loads(path.read_text())
    config.pop("_info", None)
    scenes = config.pop("scenes", {})
    if set(config) != set(KEYS):
        raise ValueError(f"{path}: Gruppen {sorted(config)}, erwartet {sorted(KEYS)}")
    for group, keys in KEYS.items():
        if set(config[group]) != keys:
            raise ValueError(f"{path}: '{group}' - falsch oder fehlend: {sorted(set(config[group]) ^ keys)}")
    for scene, groups_of_scene in scenes.items():
        if scene not in table_scene.SCENES:
            raise ValueError(f"{path}: scenes - unknown scene '{scene}', possible: {sorted(table_scene.SCENES)}")
        for group, values in groups_of_scene.items():
            unknown = set(values) - (KEYS.get(group, set()) - {"enabled"}) if group in GROUPS else {group}
            if unknown:
                raise ValueError(f"{path}: scenes.{scene}.{group} - unknown {sorted(unknown)} (enabled is not overridable)")
    config["scenes"] = scenes
    if groups is not None:
        unknown = set(groups) - set(GROUPS)
        if unknown:
            raise ValueError(f"unbekannte Gruppen {sorted(unknown)}, moeglich: {GROUPS}")
        for group in GROUPS:
            config[group]["enabled"] = group in groups
    return config


def for_scene(config, scene):
    """The config with the ranges of config["scenes"][scene] laid over its groups (a copy; the scene's own narrower ranges)."""
    overrides = config.get("scenes", {}).get(scene, {})
    return {key: {**value, **overrides.get(key, {})} if isinstance(value, dict) else value for key, value in config.items()}


def enabled(config):
    return [group for group in GROUPS if config[group]["enabled"]]


def nominal(objects=table_scene.OBJECTS):
    """Variation ohne Abweichung = Szene wie gebaut. None bei Farben heisst: Originalfarbe. objects: the scene's objects."""
    zero3 = [0.0, 0.0, 0.0]
    return {
        "objects": {name: {"dx_m": 0.0, "dy_m": 0.0, "yaw_deg": 0.0} for name in objects},
        "stand": {"dx_m": 0.0, "dy_m": 0.0, "yaw_deg": 0.0},
        "physics": {"mass_scale": {name: 1.0 for name in objects}, "friction_scale": 1.0,
                    "gain_scale": [1.0] * BODY_MOTORS, "hand_gain_scale": {"left": 1.0, "right": 1.0},
                    "latency_s": 0.0},
        "image": {"light_scale": 1.0, "headlight_scale": 1.0, "light_tilt_deg": 0.0, "light_azimuth_deg": 0.0,
                  "light_tint": [1.0, 1.0, 1.0], "table_rgb": None, "object_rgb": {name: None for name in objects},
                  "camera_offset_m": {cam: zero3 for cam in table_scene.CAMERAS},
                  "camera_tilt_deg": {cam: zero3 for cam in table_scene.CAMERAS},
                  "rgb_noise_std": 0.0, "depth_noise_std_m": 0.0},
        "distractors": {},  # name -> {x_m, y_m, yaw_deg} on the table; distractors not listed stay parked
    }


def sample(config, rng, scene="bottle"):
    """Eine Variation ziehen; abgeschaltete Gruppen bleiben nominal. scene: key of table_scene.SCENES."""
    setup = table_scene.SCENES[scene]
    objects, target = setup["objects"], setup["target"]
    config = for_scene(config, scene)
    v = nominal(objects)
    if config["objects"]["enabled"]:
        v["objects"] = _sample_objects(config["objects"], rng, scene)
    if config["stand"]["enabled"]:
        c = config["stand"]
        v["stand"] = {"dx_m": _pm(rng, c["forward_m"]), "dy_m": _pm(rng, c["sideways_m"]), "yaw_deg": _pm(rng, c["yaw_deg"])}
    if config["physics"]["enabled"]:
        c = config["physics"]
        v["physics"] = {
            "mass_scale": {name: _between(rng, c["mass_scale"]) for name in objects},
            "friction_scale": _between(rng, c["friction_scale"]),
            "gain_scale": [_between(rng, c["gain_scale"]) for _ in range(BODY_MOTORS)],
            "hand_gain_scale": {side: _between(rng, c["gain_scale"]) for side in ("left", "right")},
            "latency_s": _between(rng, c["latency_s"]),
        }
    if config["image"]["enabled"]:
        c = config["image"]
        v["image"] = {
            "light_scale": _between(rng, c["light_scale"]),
            "headlight_scale": _between(rng, c["light_scale"]),
            "light_tilt_deg": float(rng.uniform(0.0, c["light_tilt_deg"])),
            "light_azimuth_deg": float(rng.uniform(0.0, 360.0)),
            "light_tint": [1.0 + _pm(rng, c["light_tint"]) for _ in range(3)],
            "table_rgb": _not_red(rng, c["table_rgb"], c["max_red_ratio"]),
            "object_rgb": {name: (_jitter(rng, objects[name][2][:3], c["bottle_rgb_jitter"]) if name == target
                                  else _not_red(rng, c["distractor_rgb"], c["max_red_ratio"]))
                           for name in objects},
            "camera_offset_m": {cam: [_pm(rng, c["camera_offset_m"]) for _ in range(3)] for cam in table_scene.CAMERAS},
            "camera_tilt_deg": {cam: [_pm(rng, c["camera_tilt_deg"]) for _ in range(3)] for cam in table_scene.CAMERAS},
            "rgb_noise_std": float(c["rgb_noise_std"]),
            "depth_noise_std_m": float(c["depth_noise_std_m"]),
        }
    # last, so they keep clear of where the ball and the robot actually are (no draw at all in a scene without distractors)
    o, stand = v["objects"].get("volleyball"), v["stand"]
    v["distractors"] = sample_distractors(config["distractors"], rng, scene,
                                          ball_shift=(o["dx_m"], o["dy_m"]) if o else (0.0, 0.0),
                                          robot_shift=(stand["dx_m"], stand["dy_m"]))
    return v


class ModelVariation:
    """Merkt sich die Originalwerte eines MjModel und setzt je Versuch eine Variation darauf.

    apply() geht immer von den Originalwerten aus - sonst wuerde z. B. "Masse x 1,2" von Versuch zu Versuch wachsen.
    Danach muss der Aufrufer mj_setConst rufen (abgeleitete Groessen wie Teilbaum-Massen).
    """

    FIELDS = ("body_pos", "body_quat", "body_mass", "body_inertia", "geom_friction", "geom_rgba", "light_diffuse",
              "light_dir", "cam_pos", "cam_quat")

    def __init__(self, model, objects=table_scene.OBJECTS):
        m = self.model = model
        self.pelvis = m.body("pelvis").id
        self.objects = {name: m.body(name).id for name in objects}
        self.object_geoms = {name: m.geom(f"{name}_geom").id for name in objects}
        self.table_geom = m.geom("table_top").id
        self.cameras = {cam: m.camera(cam).id for cam in table_scene.CAMERAS}
        self.original = {field: getattr(m, field).copy() for field in self.FIELDS}
        self.headlight = (m.vis.headlight.ambient.copy(), m.vis.headlight.diffuse.copy())

    def apply(self, v):
        m, o = self.model, self.original
        for field in self.FIELDS:
            getattr(m, field)[:] = o[field]

        stand = v["stand"]
        m.body_pos[self.pelvis] += [stand["dx_m"], stand["dy_m"], 0.0]
        mujoco.mju_mulQuat(m.body_quat[self.pelvis], _quat([0.0, 0.0, stand["yaw_deg"]]), o["body_quat"][self.pelvis])

        physics = v["physics"]
        for name, body in self.objects.items():
            m.body_mass[body] *= physics["mass_scale"][name]
            m.body_inertia[body] *= physics["mass_scale"][name]
        # Alle Flaechen gleich: MuJoCo nimmt bei zwei Flaechen die groessere Reibung, sonst wirkt eine Absenkung nur halb
        m.geom_friction[:, 0] *= physics["friction_scale"]

        image = v["image"]
        m.light_diffuse[:] *= image["light_scale"] * np.asarray(image["light_tint"])
        azimuth = np.radians(image["light_azimuth_deg"])
        tilt = _quat(image["light_tilt_deg"] * np.array([np.cos(azimuth), np.sin(azimuth), 0.0]))
        for light in range(m.nlight):
            mujoco.mju_rotVecQuat(m.light_dir[light], o["light_dir"][light], tilt)
        m.vis.headlight.ambient[:] = self.headlight[0] * image["headlight_scale"]
        m.vis.headlight.diffuse[:] = self.headlight[1] * image["headlight_scale"]
        if image["table_rgb"] is not None:
            m.geom_rgba[self.table_geom, :3] = image["table_rgb"]
        for name, rgb in image["object_rgb"].items():
            if rgb is not None:
                m.geom_rgba[self.object_geoms[name], :3] = rgb
        for cam, cam_id in self.cameras.items():
            m.cam_pos[cam_id] += image["camera_offset_m"][cam]
            mujoco.mju_mulQuat(m.cam_quat[cam_id], o["cam_quat"][cam_id], _quat(image["camera_tilt_deg"][cam]))


# --- Ziehen -----------------------------------------------------------------------------------------------------------
def _pm(rng, half):
    return float(rng.uniform(-half, half))


def _between(rng, bounds):
    return float(rng.uniform(*bounds))


def _jitter(rng, rgb, half):
    return [float(np.clip(c + rng.uniform(-half, half), 0.0, 1.0)) for c in rgb]


def _not_red(rng, bounds, max_ratio):
    """Zufallsfarbe, die nicht als rot durchgeht - die Flaschensuche (perception.red_mask) soll nur die Flasche finden."""
    for _ in range(MAX_OBJECT_DRAWS):
        r, g, b = rng.uniform(*bounds, 3)
        if r <= max_ratio * g and r <= max_ratio * b:
            return [float(r), float(g), float(b)]
    raise RuntimeError("keine nicht-rote Farbe gefunden - Bereich pruefen")


def footprint(name):
    """Radius der Standflaeche eines Objekts (m); beim Wuerfel die halbe Diagonale, weil er gedreht sein kann."""
    kind, size, *_ = table_scene.ALL_OBJECTS[name]
    return size[0] if kind in ("cylinder", "sphere") else float(np.hypot(size[0], size[1]))


def _sample_objects(c, rng, scene="bottle"):
    """Objekte verschieben und drehen; nur Lagen, in denen alle ganz auf dem Tisch stehen und sich nicht beruehren."""
    centre_x, (half_x, y_max) = table_scene.SCENES[scene]["table"]
    x_min, x_max = centre_x - half_x, centre_x + half_x
    for _ in range(MAX_OBJECT_DRAWS):
        placed, xy = {}, {}
        for name, (*_, (x, y)) in table_scene.SCENES[scene]["objects"].items():
            placed[name] = {"dx_m": _pm(rng, c["offset_m"]), "dy_m": _pm(rng, c["offset_m"]), "yaw_deg": _pm(rng, c["yaw_deg"])}
            xy[name] = np.array([x + placed[name]["dx_m"], y + placed[name]["dy_m"]])
        on_table = all(x_min + footprint(n) + c["table_margin_m"] <= p[0] <= x_max - footprint(n) - c["table_margin_m"]
                       and abs(p[1]) <= y_max - footprint(n) - c["table_margin_m"] for n, p in xy.items())
        names = list(xy)
        apart = all(np.linalg.norm(xy[a] - xy[b]) >= footprint(a) + footprint(b) + c["min_gap_m"]
                    for i, a in enumerate(names) for b in names[i + 1:])
        if on_table and apart:
            return placed
    raise RuntimeError("keine gueltige Objektlage gefunden - Bereiche pruefen")


def sample_distractors(c, rng, scene, ball_shift=(0.0, 0.0), robot_shift=(0.0, 0.0)):
    """Put c["count"] = [min, max] distractors of the scene on the table: {name: {x_m, y_m, yaw_deg}}. {} if the group is off
    or the scene has none. Each one fully on the table, apart from the others, clear of ball, basket and the carry arc.
    ball_shift, robot_shift: start offset (x, y) of the ball and of the robot's stand in this variation."""
    pool = list(table_scene.SCENES[scene].get("distractors", {}))
    if not c["enabled"] or not pool:
        return {}
    low, high = c["count"]
    count = int(rng.integers(low, min(high, len(pool)) + 1))
    names = [pool[i] for i in sorted(rng.choice(len(pool), count, replace=False))]
    for _ in range(MAX_OBJECT_DRAWS):
        placed = {}
        for name in names:
            for _ in range(MAX_OBJECT_DRAWS):
                xy = _on_table(rng, scene, footprint(name) + c["table_margin_m"])
                if (_clear_of_volleyball_setup(xy, footprint(name) + c["keepout_m"], ball_shift, robot_shift)
                        and all(np.hypot(xy[0] - p["x_m"], xy[1] - p["y_m"]) >= footprint(name) + footprint(other) + c["min_gap_m"]
                                for other, p in placed.items())):
                    placed[name] = {"x_m": float(xy[0]), "y_m": float(xy[1]), "yaw_deg": _pm(rng, c["yaw_deg"])}
                    break
            else:
                break  # this one does not fit next to the ones already placed: draw the whole set again
        if len(placed) == count:
            return placed
    raise RuntimeError("keine gueltige Lage fuer die Distractors gefunden - Bereiche pruefen")


def _on_table(rng, scene, inset):
    """Random (x, y) on the table top of the scene, at least `inset` m from every edge."""
    centre_x, (half_x, half_y) = table_scene.SCENES[scene]["table"]
    return np.array([rng.uniform(centre_x - half_x + inset, centre_x + half_x - inset), rng.uniform(-half_y + inset, half_y - inset)])


def _clear_of_volleyball_setup(xy, radius, ball_shift=(0.0, 0.0), robot_shift=(0.0, 0.0)):
    """A circle of `radius` at xy is outside the ball (nominal spot + ball_shift), the basket and the carry arc around the
    waist axis (moved by robot_shift)."""
    _, ball_size, *_, ball_xy = table_scene.VOLLEYBALL_OBJECTS["volleyball"]
    basket = np.abs(xy - table_scene.BASKET_CENTER) - (table_scene.BASKET_INNER_HALF + table_scene.BASKET_WALL)
    return (np.linalg.norm(xy - np.add(ball_xy, ball_shift)) >= ball_size[0] + radius
            and np.linalg.norm(np.maximum(basket, 0.0)) >= radius  # distance to the basket's outer square
            and np.linalg.norm(xy - np.asarray(robot_shift)) >= table_scene.DISTRACTOR_MIN_REACH + radius)


def _quat(rotation_deg):
    """Quaternion zu einem Drehvektor (Grad)."""
    rotvec = np.radians(np.asarray(rotation_deg, float))
    angle = float(np.linalg.norm(rotvec))
    q = np.array([1.0, 0.0, 0.0, 0.0])
    if angle > 0.0:
        mujoco.mju_axisAngle2Quat(q, rotvec / angle, angle)
    return q
