"""Integration: Zufallsvariation der Greifszene (Stufe 1.5b, g1sim/randomization.py + TableEnv.reset).

Abgesichert wird vor allem, was still brechen kann: ModelVariation schreibt direkt ins MjModel. Stellt reset() ohne
Variation nicht alles zurueck, wandern Masse, Reibung, Licht oder Kamerapose unbemerkt von Versuch zu Versuch weiter -
jede spaetere Demo haette dann eine andere "Normal"-Szene.
Braucht das gebaute Laufmodell (scripts/51_build_brainco_sim.py); die Tischszene schreibt der Test selbst.
"""
import copy
import json

import numpy as np
import pytest

from conftest import needs_g1ext

pytestmark = [pytest.mark.integration, needs_g1ext]

SAMPLES = 300


@pytest.fixture(scope="module")
def env():
    from g1sim import MANIFEST, SIM_DIR, table_env, table_scene

    if not (SIM_DIR / MANIFEST).is_file():
        pytest.skip("Laufmodell fehlt - zuerst scripts/51_build_brainco_sim.py")
    table_scene.write()
    e = table_env.TableEnv()
    yield e
    e.close()


@pytest.fixture(scope="module")
def config():
    from g1sim import randomization

    return randomization.load()


def snapshot(env):
    """Alles, was eine Variation veraendert: Modellfelder, abgeleitete Massen, Stirnlicht und Regler."""
    from g1sim import randomization

    m = env.model
    snap = {field: getattr(m, field).copy() for field in randomization.ModelVariation.FIELDS}
    snap.update(body_subtreemass=m.body_subtreemass.copy(), headlight_ambient=m.vis.headlight.ambient.copy(),
                headlight_diffuse=m.vis.headlight.diffuse.copy(), kp=env.kp.copy(), kd=env.kd.copy(),
                hand_gain=np.array(list(env.hand_gain.values())), latency=np.array(env.latency),
                noise_std=np.array(env.noise_std))
    return snap


def test_reset_ohne_variation_stellt_alles_bitgenau_zurueck(env):
    env.reset()
    nominal = snapshot(env)
    for _ in range(5):
        env.reset(randomize=True)
        varied = snapshot(env)
        assert any(not np.array_equal(varied[k], nominal[k]) for k in nominal), "Variation hat nichts veraendert"
    env.reset()
    after = snapshot(env)
    changed = [k for k in nominal if not np.array_equal(after[k], nominal[k])]
    assert changed == [], f"nach reset() nicht zurueckgestellt: {changed}"


def test_variation_laesst_sich_wiederherstellen(env):
    """Ein gespeichertes env.variation stellt genau denselben Start wieder her (Replay, Teleop-Wiederholung)."""
    env.reset(randomize=True)
    v, qpos = copy.deepcopy(env.variation), env.data.qpos.copy()
    env.reset()
    env.reset(variation=v)
    assert np.array_equal(env.data.qpos, qpos)
    assert env.variation == v


def test_gleicher_seed_gleiche_variation(config):
    from g1sim import randomization

    a = randomization.sample(config, np.random.default_rng(7))
    b = randomization.sample(config, np.random.default_rng(7))
    c = randomization.sample(config, np.random.default_rng(8))
    assert json.dumps(a) == json.dumps(b)
    assert json.dumps(a) != json.dumps(c)


def test_abgeschaltete_gruppen_bleiben_nominal():
    from g1sim import randomization

    v = randomization.sample(randomization.load(groups=["stand"]), np.random.default_rng(0))
    nominal = randomization.nominal()
    assert v["stand"] != nominal["stand"]
    assert {g: v[g] for g in ("objects", "physics", "image")} == {g: nominal[g] for g in ("objects", "physics", "image")}


def test_nur_die_flasche_ist_rot(config):
    """Die Flaschensuche (red_mask) darf nur die Flasche finden - auch bei unguenstigster Lichttoenung."""
    from g1sim import randomization
    from g1sim.perception import red_mask

    rng = np.random.default_rng(0)
    worst_tint = np.array([1 + config["image"]["light_tint"], 1 - config["image"]["light_tint"],
                           1 - config["image"]["light_tint"]])
    for _ in range(SAMPLES):
        image = randomization.sample(config, rng)["image"]
        others = [image["table_rgb"]] + [rgb for name, rgb in image["object_rgb"].items() if name != "red_bottle"]
        for rgb in others:
            pixel = np.clip(np.asarray(rgb) * worst_tint * 255, 0, 255).astype(np.uint8).reshape(1, 1, 3)
            assert not red_mask(pixel)[0, 0], rgb
        bottle = (np.asarray(image["object_rgb"]["red_bottle"]) * 255).astype(np.uint8).reshape(1, 1, 3)
        assert red_mask(bottle)[0, 0], image["object_rgb"]["red_bottle"]


def test_objekte_ganz_auf_dem_tisch_und_ohne_beruehrung(config):
    from g1sim import randomization, table_scene

    rng = np.random.default_rng(1)
    x_min = table_scene.TABLE_CENTER_X - table_scene.TABLE_HALF[0]
    x_max = table_scene.TABLE_CENTER_X + table_scene.TABLE_HALF[0]
    for _ in range(SAMPLES):
        objects = randomization.sample(config, rng)["objects"]
        xy = {n: np.array(table_scene.OBJECTS[n][4]) + [o["dx_m"], o["dy_m"]] for n, o in objects.items()}
        for name, (x, y) in xy.items():
            r = randomization.footprint(name)
            assert x_min + r <= x <= x_max - r and abs(y) <= table_scene.TABLE_HALF[1] - r, name
        names = list(xy)
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                assert np.linalg.norm(xy[a] - xy[b]) >= randomization.footprint(a) + randomization.footprint(b)


def test_konfiguration_mit_unbekanntem_schluessel_bricht_ab(tmp_path):
    from g1sim import randomization

    config = json.loads(randomization.CONFIG.read_text())
    config["physics"]["gravity_scale"] = [0.9, 1.1]
    path = tmp_path / "randomization.json"
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match="gravity_scale"):
        randomization.load(path)
