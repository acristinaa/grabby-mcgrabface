"""Greif-Umgebung auf der Tischszene: direkte MuJoCo-Regelung ohne DDS, schneller als Echtzeit.

Eine Umgebung fuer alle, die am Tisch greifen: Demo-Fabrik (Stufe 1.4/2.1), openpi-Anbindung (g1sim/openpi_env.py)
und Auswertung. Interface in the language of the 27-D action space (g1sim/action_space.py):
  observation() -> Kamerabilder (uint8) + state (27-D)
  apply(actions) -> Arm-Sollwinkel und Hand-Sollwerte fuer einen Regeltakt (1/30 s)

Regelung wie am Roboter: PD je Gelenk mit LeRobots G1-Verstaerkungen (config_unitree_g1.py) fuer Beine, Taille und
Arme, Revo2 mit dem PD aus g1sim/brainco.py. Beine und Taille halten die Nullstellung, die Arme starten in
action_space.READY_POSE (volleyball scenes: BALL_READY_POSE, see self.ready_pose). Halten heisst: einen festen Sollwert
weiter schicken - nicht state(), sonst sacken die Arme ab.
Zufallsvariation (Stufe 1.5b): reset(randomize=True) zieht Objektlage, Standpose, Physik und Bild nach
configs/randomization.json (g1sim/randomization.py); die gezogenen Werte stehen danach in self.variation.
scene picks the table scene (table_scene.SCENES, default "bottle"). reset(randomize=True) varies every enabled group in every scene
(ranges narrowed per scene in the config block "scenes"); in the volleyball scene also 1-3 of table_scene.VOLLEYBALL_DISTRACTORS on
the table, the rest parked on the floor behind the robot. A scene with "reach_m" redraws starts whose target is out of reach.
The free-base scene ("volleyball_walk") has no randomization yet: its stance is set by the walk-up, not by the "stand" group.
Waist yaw: action index 26 (absolute target, 0 in READY_POSE); the last commanded value is in self.waist_yaw_target.
Free base (scene "volleyball_walk"): the pelvis keeps its free joint. reset() puts it at the variation's "stance" (else the
scene's) in the measured GR00T crouch (legs and waist roll/pitch held there) and welds it there (hold_pelvis).
snapshot_handover() / restore_handover() save and restore everything needed to continue bit-exactly from a moment.
"""
import copy

import mujoco
import numpy as np

from g1sim import SIM_DIR, action_space, brainco, episode, payload, randomization, table_scene

CONTROL_HZ = 30
SUBSTEPS = 10  # Physik 300 Hz - Regeltakt genau 1/30 s (LeRobot-Laufsim: 250 Hz)

# LeRobot UnitreeG1Config (_GAINS), G1-Motorreihenfolge 0..28
# Bildgroesse je Kamera (Hoehe, Breite). Kopfkamera wie die Farbkamera der D455: 16:10 (1280 x 800 / 4). Mit fovy 65 Grad
# (payload.xml) ergibt das 91 Grad Breite - Datenblatt 90 x 65. Quadratisch waeren es nur 65 Grad Breite: Der Sim-Roboter saehe
# links und rechts je 12 Grad weniger als der echte (Lernpfad L3a). Auf 224 x 224 mit Rand bringt openpi die Bilder selbst
# (ResizeImages = resize_with_pad) - fuer Sim und echten Roboter gleich. Die Handgelenkkameras gibt es am echten G1 nicht
# (Entscheidung D3, 2026-10-02): observation() liefert nur episode.IMAGE_KEYS, render() zeigt sie weiter zum Anschauen.
IMAGE_SIZES = {"head_camera": (200, 320), "left_wrist_camera": (224, 224), "right_wrist_camera": (224, 224)}

KP = np.array([150, 150, 150, 300, 40, 40] * 2 + [250, 250, 250] + ([50, 50, 80, 80] + [40, 40, 40]) * 2, dtype=float)
KD = np.array([2, 2, 2, 4, 2, 2] * 2 + [5, 5, 5] + ([3, 3, 3, 3] + [1.5, 1.5, 1.5]) * 2, dtype=float)
WAIST_YAW = 12  # G1 motor index of waist_yaw_joint (range +-2.618 rad)
LEGS = slice(0, 12)  # G1 motors of both legs
WAIST_ROLL_PITCH = slice(13, 15)
CLEARANCE_RANGE = 0.5  # m, stance_clearance reports at most this


class TableEnv:
    def __init__(self, scene_path=None, image_sizes=None, seed=0, variation_config=None, scene="bottle"):
        """variation_config: aus randomization.load(); None = configs/randomization.json. scene: key of table_scene.SCENES."""
        self.scene_name, self.scene = scene, table_scene.SCENES[scene]
        self.ready_pose = self.scene.get("ready", action_space.READY_POSE)  # 27-D start pose of arms, hands and waist
        self.objects = self.scene["objects"]
        self.distractors = self.scene.get("distractors", {})
        path = scene_path or SIM_DIR / self.scene["file"]
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.model.opt.timestep = 1.0 / (CONTROL_HZ * SUBSTEPS)
        self.data = mujoco.MjData(self.model)
        self.image_sizes = image_sizes or IMAGE_SIZES
        self.seed = seed
        self.rng = np.random.default_rng(seed)
        # Bildrauschen aus eigenem Zufallsstrom: sonst haengt die naechste Startlage davon ab, wie viele Bilder gerendert wurden
        self.noise_rng = np.random.default_rng([seed, 1])
        self.resets = 0  # reset() calls so far; each reset re-seeds noise_rng with its index
        self.variation_config = variation_config or randomization.load()
        self.model_variation = randomization.ModelVariation(self.model, self.objects)
        self._renderers = {}  # je Bildgroesse ein Renderer (ein mujoco.Renderer hat eine feste Groesse)
        m = self.model

        act_of_joint = {int(j): a for a, j in enumerate(m.actuator_trnid[:, 0])}
        body_acts = [a for a in range(m.nu) if not m.actuator(a).name.startswith(tuple(brainco.PREFIX.values()))]
        if len(body_acts) != 29:
            raise RuntimeError(f"{len(body_acts)} Koerpermotoren statt 29")
        body_joints = [int(m.actuator_trnid[a, 0]) for a in body_acts]
        self.body_act = np.array(body_acts)
        self.body_qadr = m.jnt_qposadr[body_joints]
        self.body_vadr = m.jnt_dofadr[body_joints]
        self.arm_slot = {s: np.array(action_space.ARM_MOTOR_INDEX[s]) for s in action_space.SIDES}

        self.hand = {}
        for side in brainco.SIDES:
            joints = [m.joint(n).id for n in brainco.joint_names(side)]
            self.hand[side] = {
                "act": np.array([act_of_joint[j] for j in joints]),
                "qadr": m.jnt_qposadr[joints],
                "vadr": m.jnt_dofadr[joints],
            }
        self.limit = m.actuator_ctrlrange[:, 1].copy()
        self.object_qadr = {name: m.jnt_qposadr[m.joint(f"{name}_joint").id] for name in (*self.objects, *self.distractors)}
        self.scene_bodies = {m.body(name).id: name
                             for name in ("table", *self.scene["static"], *self.objects, *self.distractors)}
        self.robot_geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] != 0 and m.geom_bodyid[g] not in self.scene_bodies
                            and (m.geom_contype[g] or m.geom_conaffinity[g])]
        self.scene_geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] in self.scene_bodies]
        self.head_eq = [m.equality(eq).id for _, _, eq, _ in payload.HEAD.values()]
        self.free_base = bool(self.scene.get("free_base"))
        if self.free_base:
            self.base_qadr = m.jnt_qposadr[m.body_jntadr[m.body("pelvis").id]]
            self.hold_mocap = m.body_mocapid[m.body(table_scene.PELVIS_HOLD).id]
            self.hold_eq = m.equality(table_scene.PELVIS_HOLD).id
        self._scratch = None  # MjData for stance_clearance
        self.variation, self.draws = randomization.nominal(self.objects), 0
        self._set_up(self.variation)

    # --- Ablauf -------------------------------------------------------------------------------------------------
    def reset(self, randomize=False, variation=None):
        """Arms and hands in the scene's start pose (self.ready_pose), objects in their places. Rueckgabe: Beobachtung.

        randomize: neue Variation ziehen. Starts, in denen der Roboter einem Objekt oder dem Tisch zu nah ist, werden
        verworfen und neu gezogen (Anzahl Zuege danach in self.draws).
        variation: genau diese Variation wiederherstellen (ein frueheres self.variation, z. B. fuer ein Replay).
        """
        if randomize and self.free_base:
            raise ValueError(f"scene '{self.scene_name}': no randomization yet, use reset(randomize=False)")
        # camera noise of reset k depends only on (seed, k), not on how many images earlier trials rendered
        self.noise_rng = np.random.default_rng([self.seed, self.resets, 1])
        self.resets += 1
        limits = self.variation_config["start"]
        for draw in range(1, limits["max_draws"] + 1):
            if variation is not None:
                v = variation
            else:
                v = (randomization.sample(self.variation_config, self.rng, self.scene_name) if randomize
                     else randomization.nominal(self.objects))
            self._set_up(v)
            if variation is not None or not randomize or self._start_is_clear(limits):
                break
        else:
            raise RuntimeError(f"{limits['max_draws']} Starts gezogen, keiner mit genug Abstand - Bereiche pruefen")
        self.variation, self.draws = v, draw
        for _ in range(int(0.3 * CONTROL_HZ)):  # Objekte absetzen lassen
            self._substeps()
        self.data.time = 0.0
        return self.observation()

    def _set_up(self, v):
        """Variation v ins Modell und in den Regler schreiben, Startzustand setzen (ohne Physikschritt)."""
        m, d = self.model, self.data
        self.model_variation.apply(v)
        for name, holder in self.scene.get("holders", {}).items():  # a static holder moves with its object's offset
            o = v["objects"][name]
            m.body_pos[m.body(holder).id] += [o["dx_m"], o["dy_m"], 0.0]
        if "basket" in v:  # basket offset (volleyball scenes)
            m.body_pos[m.body("basket").id] += [v["basket"]["dx_m"], v["basket"]["dy_m"], 0.0]
        payload.set_head(m, **table_scene.HEAD_POSE)  # ModelVariation does not restore eq_data
        mujoco.mj_setConst(m, d)  # abgeleitete Groessen (z. B. Teilbaum-Massen) zu den neuen Werten
        mujoco.mj_resetData(m, d)
        for name, (*_, (x, y)) in self.objects.items():
            adr, o = self.object_qadr[name], v["objects"][name]
            yaw = np.radians(o["yaw_deg"])
            d.qpos[adr:adr + 3] = [x + o["dx_m"], y + o["dy_m"], table_scene.object_rest_height(name) + 0.002]
            d.qpos[adr + 3:adr + 7] = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
        on_table = v.get("distractors", {})  # older saved variations have no distractors
        for i, name in enumerate(self.distractors):
            adr = self.object_qadr[name]
            if name in on_table:
                p = on_table[name]
                xyz, yaw = [p["x_m"], p["y_m"], table_scene.object_rest_height(name)], np.radians(p["yaw_deg"])
            else:
                xyz, yaw = [*table_scene.distractor_park(i), table_scene.object_rest_height(name, on=0.0)], 0.0
            d.qpos[adr:adr + 3] = [xyz[0], xyz[1], xyz[2] + 0.002]
            d.qpos[adr + 3:adr + 7] = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
        physics = v["physics"]
        gain = np.asarray(physics["gain_scale"])
        self.kp, self.kd = KP * gain, KD * gain
        self.hand_gain = dict(physics["hand_gain_scale"])
        self.latency = int(round(physics["latency_s"] * CONTROL_HZ * SUBSTEPS))  # in Physikschritten
        if not 0 <= self.latency <= SUBSTEPS:
            raise ValueError(f"Latenz {physics['latency_s']} s: erlaubt 0 .. {1 / CONTROL_HZ:.3f} s (ein Regeltakt)")
        self.noise_std = (v["image"]["rgb_noise_std"], v["image"]["depth_noise_std_m"])
        self.body_target = np.zeros(29)
        self.waist_yaw_target = 0.0
        for side, q in action_space.arm_targets(self.ready_pose).items():
            d.qpos[self.body_qadr[self.arm_slot[side]]] = q  # direkt dort starten, nicht erst hinfahren
            self.body_target[self.arm_slot[side]] = q
        if self.free_base:
            self._pose_base(d.qpos, v.get("stance", self.scene["stance"]))
            stand = table_scene.GROOT_STANCE
            self.body_target[LEGS] = stand["legs"]
            self.body_target[WAIST_ROLL_PITCH] = [stand["waist_roll"], stand["waist_pitch"]]
            self.hold_pelvis(True)
        self.hand_target = action_space.hand_targets(self.ready_pose)
        for side, h in self.hand.items():
            d.qpos[h["qadr"]] = brainco.to_rad(self.hand_target[side])
        mujoco.mj_forward(m, d)

    def _pose_base(self, qpos, stance):
        """Free base: pelvis at stance {"x_m", "y_m", "yaw_deg"}, STAND_Z and the measured roll/pitch; legs and waist
        roll/pitch in the GR00T crouch (table_scene.GROOT_STANCE). Writes into qpos only."""
        s, a = table_scene.GROOT_STANCE, self.base_qadr
        qpos[a:a + 3] = [stance["x_m"], stance["y_m"], table_scene.STAND_Z]
        mujoco.mju_euler2Quat(qpos[a + 3:a + 7], np.radians([s["roll_deg"], s["pitch_deg"], stance["yaw_deg"]]), "XYZ")
        qpos[self.body_qadr[LEGS]] = s["legs"]
        qpos[self.body_qadr[WAIST_ROLL_PITCH]] = [s["waist_roll"], s["waist_pitch"]]

    def _start_is_clear(self, limits):
        """Abstand jedes Roboterteils zu den Objekten und zum Tisch mindestens wie in limits; the target within the scene's
        reach_m of the pelvis (if it has one)."""
        m, d = self.model, self.data
        if "reach_m" in self.scene:
            low, high = self.scene["reach_m"]
            distance = np.linalg.norm(self.object_position(self.scene["target"])[:2] - d.xpos[m.body("pelvis").id][:2])
            if not low <= distance <= high:
                return False
        targets = [(m.geom(f"{name}_geom").id, limits["min_object_clearance_m"]) for name in self.objects]
        targets.append((m.geom("table_top").id, limits["min_table_clearance_m"]))
        static = {m.body(name).id for name in self.scene["static"]}
        targets += [(g, limits["min_object_clearance_m"]) for g in range(m.ngeom) if m.geom_bodyid[g] in static]
        return all(mujoco.mj_geomDistance(m, d, g, target, 2 * limit, None) >= limit
                   for target, limit in targets for g in self.robot_geoms)

    def hold_pelvis(self, on=True):
        """Free base: weld the pelvis at its current pose (on) or release it (off)."""
        d, a = self.data, self.base_qadr
        if on:
            d.mocap_pos[self.hold_mocap] = d.qpos[a:a + 3]
            d.mocap_quat[self.hold_mocap] = d.qpos[a + 3:a + 7]
        d.eq_active[self.hold_eq] = int(on)

    def stance_clearance(self, stance):
        """Smallest distance (m, < 0 = overlap) between the robot and table, stand, objects and basket with the pelvis at
        stance (as in reset), arms and hands in the scene's start pose. Kinematics only, in a scratch MjData; objects where
        they are now."""
        m = self.model
        if self._scratch is None:
            self._scratch = mujoco.MjData(m)
        d = self._scratch
        d.qpos[:] = self.data.qpos
        self._pose_base(d.qpos, stance)
        for side, q in action_space.arm_targets(self.ready_pose).items():
            d.qpos[self.body_qadr[self.arm_slot[side]]] = q
        d.qpos[self.body_qadr[WAIST_YAW]] = self.ready_pose[action_space.SLICES["waist"]][0]
        for side, q in action_space.hand_targets(self.ready_pose).items():
            d.qpos[self.hand[side]["qadr"]] = brainco.to_rad(q)
        mujoco.mj_kinematics(m, d)
        return float(min(mujoco.mj_geomDistance(m, d, g, target, CLEARANCE_RANGE, None)
                         for g in self.robot_geoms for target in self.scene_geoms))

    def snapshot_handover(self):
        """Everything restore_handover() needs to continue bit-exactly from now: MuJoCo integration state (qpos, qvel, act,
        ctrl, mocap, eq_active, warmstart, ...), head servo targets, timestep, variation, controller targets, camera noise."""
        m, d = self.model, self.data
        mujoco.mj_forward(m, d)  # derived values to the current state: the next image equals the first one after a restore
        state = np.empty(mujoco.mj_stateSize(m, mujoco.mjtState.mjSTATE_INTEGRATION))
        mujoco.mj_getState(m, d, state, mujoco.mjtState.mjSTATE_INTEGRATION)
        return {"state": state, "head_eq_data": m.eq_data[self.head_eq].copy(), "timestep": float(m.opt.timestep),
                "variation": copy.deepcopy(self.variation), "body_target": self.body_target.copy(),
                "hand_target": {side: np.array(t, dtype=float) for side, t in self.hand_target.items()},
                "waist_yaw_target": float(self.waist_yaw_target), "noise_rng": self.noise_rng.bit_generator.state}

    def restore_handover(self, snapshot):
        """Back to the moment of snapshot_handover(). In the same TableEnv the run continues bit-exactly (state and images);
        in a fresh TableEnv of the same scene the state is bit-exact, images may differ by 1 grey level (other GL context)."""
        m, d, s = self.model, self.data, snapshot
        self._set_up(s["variation"])
        self.variation = copy.deepcopy(s["variation"])
        m.opt.timestep = s["timestep"]
        m.eq_data[self.head_eq] = s["head_eq_data"]
        mujoco.mj_setState(m, d, np.asarray(s["state"], dtype=float), mujoco.mjtState.mjSTATE_INTEGRATION)
        self.body_target = np.array(s["body_target"], dtype=float)
        self.hand_target = {side: np.array(t, dtype=float) for side, t in s["hand_target"].items()}
        self.waist_yaw_target = float(s["waist_yaw_target"])
        self.noise_rng.bit_generator.state = s["noise_rng"]
        mujoco.mj_forward(m, d)

    def apply(self, actions):
        """Einen Regeltakt ausfuehren. actions: 27-D (arms rad, hands normalized, waist yaw rad), absolute targets.

        Mit Latenz gelten die ersten self.latency Physikschritte noch die alten Sollwerte.
        """
        actions = np.asarray(actions, dtype=float)
        body = self.body_target.copy()
        for side, q in action_space.arm_targets(actions).items():
            body[self.arm_slot[side]] = q
        body[WAIST_YAW] = self.waist_yaw_target = float(actions[action_space.SLICES["waist"]][0])
        hand = action_space.hand_targets(actions)
        self._substeps(self.latency)
        self.body_target, self.hand_target = body, hand
        self._substeps(SUBSTEPS - self.latency)

    def _substeps(self, n=SUBSTEPS):
        d = self.data
        for _ in range(n):
            ctrl = np.zeros(self.model.nu)
            q, dq = d.qpos[self.body_qadr], d.qvel[self.body_vadr]
            ctrl[self.body_act] = self.kp * (self.body_target - q) - self.kd * dq
            for side, h in self.hand.items():
                q, dq = d.qpos[h["qadr"]], d.qvel[h["vadr"]]
                gain = self.hand_gain[side]
                ctrl[h["act"]] = gain * (brainco.HAND_KP * (brainco.to_rad(self.hand_target[side]) - q) - brainco.HAND_KD * dq)
            d.ctrl[:] = np.clip(ctrl, -self.limit, self.limit)
            mujoco.mj_step(self.model, d)

    # --- Beobachtung --------------------------------------------------------------------------------------------
    def state(self):
        d = self.data
        arm = {s: d.qpos[self.body_qadr[self.arm_slot[s]]] for s in action_space.SIDES}
        hand = {s: d.qpos[self.hand[s]["qadr"]] for s in brainco.SIDES}
        return action_space.from_sim(arm, hand, d.qpos[self.body_qadr[WAIST_YAW]])

    def _renderer(self, camera):
        size = self.image_sizes[camera]
        if size not in self._renderers:
            self._renderers[size] = mujoco.Renderer(self.model, height=size[0], width=size[1])
        renderer = self._renderers[size]
        renderer.update_scene(self.data, camera=camera)
        return renderer

    def render(self, camera):
        return self.camera_noise(self._renderer(camera).render())

    def render_depth(self, camera):
        """RGB image (uint8) and depth image (m) of camera through the same renderer, with the variation's sensor noise."""
        renderer = self._renderer(camera)
        rgb = renderer.render()
        renderer.enable_depth_rendering()
        depth = renderer.render()
        renderer.disable_depth_rendering()
        return self.camera_noise(rgb, depth)

    def camera_noise(self, rgb, depth=None):
        """Sensorrauschen der aktuellen Variation auf ein RGB-Bild (uint8) und optional ein Tiefenbild (m) legen."""
        rgb_std, depth_std = self.noise_std
        rgb = rgb.copy()
        if rgb_std > 0:
            rgb = np.clip(rgb + self.noise_rng.normal(0.0, rgb_std, rgb.shape), 0, 255).astype(np.uint8)
        if depth is None:
            return rgb
        if depth_std > 0:
            depth = depth + self.noise_rng.normal(0.0, depth_std, depth.shape).astype(depth.dtype)
        return rgb, depth

    def observation(self):
        obs = {key: self.render(key.replace("_image", "_camera")) for key in episode.IMAGE_KEYS}
        obs["state"] = self.state()
        obs["time"] = float(self.data.time)
        return obs

    def object_position(self, name):
        adr = self.object_qadr[name]
        return self.data.qpos[adr:adr + 3].copy()

    def lift_height(self, name=None):
        """Wie weit das Objekt ueber seiner Ruhehoehe auf dem Tisch ist (m). name: None = target object of the scene."""
        name = name or self.scene["target"]
        return float(self.object_position(name)[2] - table_scene.object_rest_height(name))

    def object_tilt(self, name=None):
        """Neigung des Objekts gegen die Senkrechte (Grad). name: None = target object of the scene."""
        adr = self.object_qadr[name or self.scene["target"]]
        _, qx, qy, _ = self.data.qpos[adr + 3:adr + 7]
        return float(np.degrees(np.arccos(np.clip(1.0 - 2.0 * (qx * qx + qy * qy), -1.0, 1.0))))

    def robot_contacts(self):
        """Aktuelle Kontakte zwischen Roboter und Tisch oder Objekten: sortierte Liste (Roboter-Body, Tisch/Objekt)."""
        m, d = self.model, self.data
        pairs = set()
        for i in range(d.ncon):
            b1, b2 = int(m.geom_bodyid[d.contact[i].geom1]), int(m.geom_bodyid[d.contact[i].geom2])
            for robot, other in ((b1, b2), (b2, b1)):
                if other in self.scene_bodies and robot != 0 and robot not in self.scene_bodies:
                    pairs.add((m.body(robot).name, self.scene_bodies[other]))
        return sorted(pairs)

    def close(self):
        for renderer in self._renderers.values():
            renderer.close()
        self._renderers = {}
