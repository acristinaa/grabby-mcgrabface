"""Stufe 1.5b - Zufallsvariation (Domain Randomization) der Greifszene.

Die Bereiche stehen in configs/randomization.json, nicht im Code. Je Versuch:
  sample(config, rng)      zieht eine Variation - ein dict aus einfachen Zahlen (passt ins Ergebnis-JSON und reicht,
                           um denselben Start spaeter wiederherzustellen, z. B. fuer ein Replay)
  ModelVariation.apply(v)  schreibt Standpose, Masse, Reibung, Licht, Farben, Kamerapose, Texturen, Tischstil
                           (table_style) und Render-Look (render_look) ins MjModel
Objektlage, Motorgains, Latenz und Bildrauschen setzt TableEnv selbst (Zustand bzw. Regler, nicht Modell).
Distractors (volleyball scene): sample(..., scene="volleyball") puts 1-3 objects of table_scene.VOLLEYBALL_DISTRACTORS on the
table, clear of the ball, the basket and the arc the ball is carried along; TableEnv places them (state, not model).
Every group works in every scene (scene objects, target and table from table_scene.SCENES). A scene can narrow the ranges in
the config block "scenes" (e.g. "volleyball": {"objects": {"offset_m": 0.03}}): for_scene() lays those over the groups.
Walk world (scene "volleyball_walk", free base, Level 1 Stage 3): sample_walk() draws a layout (ball with its stand, basket),
the stop pose the walk-up aims for (navigation.table_stop_pose) and the stance = stop pose + an arrival error; ranges in the
config group "walk". TableEnv.reset() then rejects draws whose stop pose or stance brings the robot too close to the table.
"""
import json
import pathlib

import mujoco
import numpy as np

from g1sim import navigation, table_scene

CONFIG = pathlib.Path(__file__).resolve().parent.parent / "configs" / "randomization.json"
GROUPS = ("objects", "stand", "physics", "image", "distractors")
KEYS = {
    "objects": {"enabled", "offset_m", "yaw_deg", "table_margin_m", "min_gap_m", "table_height_m"},
    "stand": {"enabled", "forward_m", "sideways_m", "yaw_deg"},
    "physics": {"enabled", "mass_scale", "friction_scale", "gain_scale", "latency_s"},
    "image": {"enabled", "light_scale", "light_tilt_deg", "light_tint", "table_rgb", "distractor_rgb", "max_red_ratio",
              "bottle_rgb_jitter", "camera_offset_m", "camera_tilt_deg", "rgb_noise_std", "depth_noise_std_m", "textures",
              "texture_surfaces", "table_style", "render_look"},
    "distractors": {"enabled", "count", "yaw_deg", "table_margin_m", "min_gap_m", "keepout_m"},
    "start": {"min_object_clearance_m", "min_table_clearance_m", "max_draws"},
    "walk": {"ball_offset_m", "basket_offset_m", "separation_m", "stand_edge_gap_m", "basket_overhang_m", "side_margin_m",
             "object_gap_m", "max_waist_deg", "min_gap_deg", "arrival_back_m", "arrival_side_m", "arrival_yaw_deg", "stop_clearance_m",
             "stance_clearance_m"},
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
    if config["image"]["table_style"] not in table_scene.TABLE_STYLES:
        raise ValueError(f"{path}: image.table_style '{config['image']['table_style']}', possible: {table_scene.TABLE_STYLES}")
    low, high = config["objects"]["table_height_m"]
    if not -table_scene.TABLE_HEIGHT_MAX_M <= low <= high <= table_scene.TABLE_HEIGHT_MAX_M:
        raise ValueError(f"{path}: objects.table_height_m [{low}, {high}], needs -max <= low <= high <= max with max "
                         f"{table_scene.TABLE_HEIGHT_MAX_M} (table_scene.TABLE_HEIGHT_MAX_M)")
    if config["image"]["render_look"] not in table_scene.RENDER_LOOKS:
        raise ValueError(f"{path}: image.render_look '{config['image']['render_look']}', possible: {table_scene.RENDER_LOOKS}")
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
                  "rgb_noise_std": 0.0, "depth_noise_std_m": 0.0,
                  "textures": {},  # surface geom -> material name of table_scene.DR_MATERIALS; surfaces not listed keep their own
                  "table_style": table_scene.TABLE_STYLES[0],  # look of the table as built (table_scene.TABLE_STYLES)
                  "render_look": table_scene.RENDER_LOOKS[0]},  # lights and shine (table_scene.RENDER_LOOKS)
        "distractors": {},  # name -> {x_m, y_m, yaw_deg} on the table; distractors not listed stay parked
        "table": {"dz_m": 0.0},  # table top this much above table_scene.TABLE_TOP (table_scene.TABLE_HEIGHT_MAX_M)
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
            # drawn last and only when on: with textures off (configs/randomization_v1.json) a seed draws the same starts as before
            "textures": {surface: _texture(rng) for surface in c["texture_surfaces"]} if c["textures"] else {},
            "table_style": c["table_style"],  # no draw: a seed draws the same starts in either style
            "render_look": c["render_look"],  # no draw, as table_style
        }
    # last, so they keep clear of where the ball and the robot actually are (no draw at all in a scene without distractors)
    o, stand = v["objects"].get("volleyball"), v["stand"]
    v["distractors"] = sample_distractors(config["distractors"], rng, scene,
                                          ball_shift=(o["dx_m"], o["dy_m"]) if o else (0.0, 0.0),
                                          robot_shift=(stand["dx_m"], stand["dy_m"]))
    if config["objects"]["enabled"]:
        v["table"] = sample_table(config["objects"], rng)
    return v


def sample_table(c, rng):
    """Table height offset {dz_m} within c["table_height_m"] = [low, high] (m). Asymmetric on purpose: the teacher brushes
    table/stand/basket above +1.4 cm (runs/l1h2_g5_g7_buckets.log), so v2 uses [-0.04, 0.01] (user decision 2026-11-01).
    Drawn last and only when the range is not [0, 0]: with [0, 0] (configs/randomization_v1.json) a seed draws the same
    starts as before the table height existed."""
    if c["table_height_m"] == [0.0, 0.0]:
        return {"dz_m": 0.0}
    return {"dz_m": _between(rng, c["table_height_m"])}


def sample_walk(config, rng, scene="volleyball_walk"):
    """One variation for the walk world: physics and image as in sample(), plus a layout, its stop pose and the stance
    (walk_layout). The groups "objects", "stand" and "distractors" are not used: the layout replaces "objects", the free
    pelvis ignores the stand group's body offset, and the walk world has no distractors yet; of "objects" only the table height
    (table_height_m) is used. Ranges in config["walk"]."""
    c = config["walk"]
    v = sample({**config, **{g: {**config[g], "enabled": False} for g in ("objects", "stand", "distractors")}}, rng, scene)
    objects = for_scene(config, scene)["objects"]
    for _ in range(MAX_OBJECT_DRAWS):
        ball_shift = [_pm(rng, c["ball_offset_m"]), _pm(rng, c["ball_offset_m"])]
        basket_shift = [_pm(rng, c["basket_offset_m"]), _pm(rng, c["basket_offset_m"])]
        arrival = {"back_m": _between(rng, c["arrival_back_m"]), "side_m": _pm(rng, c["arrival_side_m"]),
                   "yaw_deg": _pm(rng, c["arrival_yaw_deg"])}
        layout = walk_layout(ball_shift, basket_shift, arrival)
        if walk_layout_ok(layout, c, scene):
            v["objects"]["volleyball"].update(dx_m=ball_shift[0], dy_m=ball_shift[1])
            v.update({key: layout[key] for key in ("basket", "stop", "arrival", "stance")})
            if objects["enabled"]:  # last, as in sample()
                v["table"] = sample_table(objects, rng)
            return v
    raise RuntimeError("no valid walk layout found - check the ranges in the config group 'walk'")


def walk_layout(ball_shift=(0.0, 0.0), basket_shift=(0.0, 0.0), arrival=None):
    """Ball (its stand) and basket moved by the shifts (x, y in m) from their nominal spots; stop = where the robot should stop
    (navigation.table_stop_pose, radius table_scene.R_STOP); stance = stop + arrival error {back_m, side_m, yaw_deg} in the
    stop pose's frame (back = away from the table, side + = left). All keys of a variation except ball_xy/basket_xy."""
    arrival = arrival or {"back_m": 0.0, "side_m": 0.0, "yaw_deg": 0.0}
    ball_xy = np.add(table_scene.VOLLEYBALL_OBJECTS["volleyball"][4], ball_shift)
    basket_xy = np.add(table_scene.BASKET_CENTER, basket_shift)
    stop = navigation.table_stop_pose(ball_xy, basket_xy, table_scene.R_STOP)
    yaw = np.radians(stop["yaw_deg"])
    forward, left = np.array([np.cos(yaw), np.sin(yaw)]), np.array([-np.sin(yaw), np.cos(yaw)])
    xy = np.array([stop["x_m"], stop["y_m"]]) - arrival["back_m"] * forward + arrival["side_m"] * left
    return {"ball_xy": ball_xy, "basket_xy": basket_xy,
            "basket": {"dx_m": float(basket_shift[0]), "dy_m": float(basket_shift[1])}, "stop": stop,
            "arrival": {key: float(value) for key, value in arrival.items()},
            "stance": {"x_m": float(xy[0]), "y_m": float(xy[1]), "yaw_deg": stop["yaw_deg"] + float(arrival["yaw_deg"])}}


def walk_layout_ok(layout, c, scene="volleyball_walk"):
    """Layout rules of the walk world (c = config["walk"]): separation in range; stand at least stand_edge_gap_m behind the
    front edge and on the table; basket on the table (its front may overhang the edge by basket_overhang_m, as in the
    nominal scene); stand and basket apart; ball and basket each within max_waist_deg of the stance's heading and at least
    min_gap_deg apart as seen from the stance (closer, the right wrist touched the basket in the reach: 4 of 200, sweep 2026-10-10)."""
    centre_x, (half_x, half_y) = table_scene.SCENES[scene]["table"]
    front, back = centre_x - half_x, centre_x + half_x
    ball, basket = layout["ball_xy"], layout["basket_xy"]
    stand_r = table_scene.STAND_INNER_RADIUS + table_scene.STAND_WALL
    basket_half = table_scene.BASKET_INNER_HALF + table_scene.BASKET_WALL
    low, high = c["separation_m"]
    if not low <= float(np.linalg.norm(ball - basket)) <= high:
        return False
    if ball[0] < front + c["stand_edge_gap_m"] or abs(ball[1]) + stand_r > half_y - c["side_margin_m"]:
        return False
    if (basket[0] - basket_half < front - c["basket_overhang_m"] or basket[0] + basket_half > back - c["side_margin_m"]
            or abs(basket[1]) + basket_half > half_y - c["side_margin_m"]):
        return False
    outside = np.maximum(np.abs(ball - basket) - basket_half, 0.0)  # stand centre to the basket's outer square
    if float(np.linalg.norm(outside)) < stand_r + c["object_gap_m"]:
        return False
    stance = layout["stance"]
    bearings = [(np.degrees(np.arctan2(xy[1] - stance["y_m"], xy[0] - stance["x_m"])) - stance["yaw_deg"] + 180.0) % 360.0 - 180.0
                for xy in (ball, basket)]
    return max(abs(b) for b in bearings) <= c["max_waist_deg"] and bearings[0] - bearings[1] >= c["min_gap_deg"]


class ModelVariation:
    """Merkt sich die Originalwerte eines MjModel und setzt je Versuch eine Variation darauf.

    apply() geht immer von den Originalwerten aus - sonst wuerde z. B. "Masse x 1,2" von Versuch zu Versuch wachsen.
    Danach muss der Aufrufer mj_setConst rufen (abgeleitete Groessen wie Teilbaum-Massen).
    """

    FIELDS = ("body_pos", "body_quat", "body_mass", "body_inertia", "geom_friction", "geom_rgba", "geom_matid", "geom_group",
              "geom_size", "geom_pos", "light_diffuse", "light_dir", "light_active", "mat_specular", "mat_shininess",
              "mat_reflectance", "cam_pos", "cam_quat")

    def __init__(self, model, objects=table_scene.OBJECTS):
        m = self.model = model
        self.pelvis = m.body("pelvis").id
        self.objects = {name: m.body(name).id for name in objects}
        self.object_geoms = {name: m.geom(f"{name}_geom").id for name in objects}
        self.table_geom = m.geom("table_top").id
        self.table = table = m.body("table").id
        self.table_visuals = [g for g in range(m.ngeom)
                              if m.geom_bodyid[g] == table and not (m.geom_contype[g] or m.geom_conaffinity[g])]
        self.table_legs = [g for g in self.table_visuals if m.geom(g).name.startswith("table_leg_")]
        self.cameras = {cam: m.camera(cam).id for cam in table_scene.CAMERAS}
        self.original = {field: getattr(m, field).copy() for field in self.FIELDS}
        self.headlight = (m.vis.headlight.ambient.copy(), m.vis.headlight.diffuse.copy())
        self.look_lights = [m.light(name).id for name in table_scene.LOOK_LIGHTS]
        robot = m.body_rootid[self.pelvis]
        # table, stand, basket, objects, distractors: every geom outside the robot and the world body with no material of its own
        self.look_geoms = [g for g in range(m.ngeom) if m.geom_matid[g] < 0 and m.body_rootid[m.geom_bodyid[g]] not in (0, robot)]
        self.look_materials = [m.geom_matid[m.geom("floor").id], *(m.mat(name).id for name in table_scene.DR_MATERIALS)]

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

        # variations saved before 2026-11-01 have no table height: the table as built. Legs still reach the floor.
        dz = v.get("table", {"dz_m": 0.0})["dz_m"]
        m.body_pos[self.table, 2] += dz
        m.geom_size[self.table_legs, 2] += dz / 2
        m.geom_pos[self.table_legs, 2] -= dz / 2

        image = v["image"]
        # variations saved before 2026-11-01 have no render_look: they were recorded with the basic look
        natural = image.get("render_look", "basic") == "natural"
        headlight = self._natural_look() if natural else self.headlight
        m.light_diffuse[:] *= image["light_scale"] * np.asarray(image["light_tint"])
        azimuth = np.radians(image["light_azimuth_deg"])
        tilt = _quat(image["light_tilt_deg"] * np.array([np.cos(azimuth), np.sin(azimuth), 0.0]))
        for light in range(m.nlight):
            mujoco.mju_rotVecQuat(m.light_dir[light], o["light_dir"][light], tilt)
        m.vis.headlight.ambient[:] = headlight[0] * image["headlight_scale"]
        m.vis.headlight.diffuse[:] = headlight[1] * image["headlight_scale"]
        if image["table_rgb"] is not None:
            m.geom_rgba[self._look_geoms("table_top"), :3] = image["table_rgb"]
        for name, rgb in image["object_rgb"].items():
            if rgb is not None:
                m.geom_rgba[self.object_geoms[name], :3] = rgb
        for cam, cam_id in self.cameras.items():
            m.cam_pos[cam_id] += image["camera_offset_m"][cam]
            mujoco.mju_mulQuat(m.cam_quat[cam_id], o["cam_quat"][cam_id], _quat(image["camera_tilt_deg"][cam]))
        for surface, material in image.get("textures", {}).items():  # variations saved before 2026-11-01 have no textures
            if material is not None:
                m.geom_matid[self._look_geoms(surface)] = m.mat(material).id
        # variations saved before 2026-11-01 have no table_style: they were recorded with the block
        if image.get("table_style", "block") == "block":
            m.geom_group[self.table_geom] = table_scene.VISIBLE_GROUP
            m.geom_group[self.table_visuals] = table_scene.HIDDEN_GROUP

    def _natural_look(self):
        """Switch the model to the "natural" table_scene.RENDER_LOOKS (lamps, matte floor, mild shine; textures drawn later
        still replace the material of their surface). Returns the headlight (ambient, diffuse) to scale."""
        m = self.model
        m.light_active[:] = 0
        m.light_active[self.look_lights] = 1
        m.mat_reflectance[self.look_materials[0]] = table_scene.LOOK_FLOOR_REFLECTANCE
        m.mat_specular[self.look_materials] = table_scene.LOOK_SPECULAR
        m.mat_shininess[self.look_materials] = table_scene.LOOK_SHININESS
        m.geom_matid[self.look_geoms] = m.mat(table_scene.LOOK_MATERIAL).id
        return tuple(np.full(3, value) for value in table_scene.LOOK_HEADLIGHT)

    def _look_geoms(self, surface):
        """Geom ids that show the colour and texture drawn for surface: the surface and its visual twin (table_scene.LOOK_TWINS)."""
        twin = table_scene.LOOK_TWINS.get(surface)
        return [self.model.geom(name).id for name in (surface, twin) if name]


# --- Ziehen -----------------------------------------------------------------------------------------------------------
def _pm(rng, half):
    return float(rng.uniform(-half, half))


def _between(rng, bounds):
    return float(rng.uniform(*bounds))


def _jitter(rng, rgb, half):
    return [float(np.clip(c + rng.uniform(-half, half), 0.0, 1.0)) for c in rgb]


def _texture(rng):
    """A material of table_scene.DR_MATERIALS or None (the surface keeps its own look), all equally likely."""
    choices = [None, *table_scene.DR_MATERIALS]
    return choices[int(rng.integers(len(choices)))]


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
