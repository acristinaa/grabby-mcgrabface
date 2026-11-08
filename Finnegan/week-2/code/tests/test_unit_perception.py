"""Unit: shape-based ball and basket finding for the Level 1 walk-up (g1sim/perception.py) on synthetic point clouds.

The real ball is grey, so the ball is found by shape, never by colour: the clouds here carry no colour at all. Each scene is
sampled densely on its surfaces and "seen" from a head camera with a z-buffer (only the nearest point per pixel survives,
so every object shows one side and hides what is behind it) plus 2 mm depth noise, like TableEnv's camera noise.
"""
import numpy as np
import pytest

from g1sim import perception

pytestmark = pytest.mark.unit

# same numbers as the volleyball scene (g1sim/table_scene.py)
TABLE_TOP = 0.74
RADIUS = 0.105
BALL = np.array([0.344, 0.241, 0.9123])  # resting on its stand
BASKET = np.array([0.344, -0.241])
BASKET_HALF = 0.145  # outer
BASKET_WALL = 0.005
RIM_Z = TABLE_TOP + 0.105  # 0.845
NOISE = 0.002
CAMERAS = {"stop": np.array([0.05, 0.0, 1.29]), "1 m back": np.array([-0.95, 0.0, 1.29])}
DENSITY = 5e5  # surface samples per m^2, several per pixel


def sphere(centre, radius, rng):
    n = int(DENSITY * 4 * np.pi * radius ** 2)
    d = rng.normal(size=(n, 3))
    return centre + radius * d / np.linalg.norm(d, axis=1, keepdims=True)


def cylinder(x, y, radius, height, rng, base=TABLE_TOP):
    side, top = int(DENSITY * 2 * np.pi * radius * height), int(DENSITY * np.pi * radius ** 2)
    a = rng.uniform(0, 2 * np.pi, side)
    r, b = radius * np.sqrt(rng.uniform(0, 1, top)), rng.uniform(0, 2 * np.pi, top)
    return np.concatenate([np.c_[x + radius * np.cos(a), y + radius * np.sin(a), base + rng.uniform(0, height, side)],
                           np.c_[x + r * np.cos(b), y + r * np.sin(b), np.full(top, base + height)]])


def box(centre, half, rng):
    centre, half = np.asarray(centre, float), np.asarray(half, float)
    faces = []
    for axis in range(3):
        other = [i for i in range(3) if i != axis]
        n = int(DENSITY * 4 * half[other[0]] * half[other[1]])
        for sign in (-1, 1):
            p = rng.uniform(-1, 1, (n, 3)) * half
            p[:, axis] = sign * half[axis]
            faces.append(centre + p)
    return np.concatenate(faces)


def table(rng):
    return box([0.525, 0.0, TABLE_TOP - 0.01], [0.325, 0.4, 0.01], rng)


def stand(rng):
    return cylinder(BALL[0], BALL[1], 0.056, 0.08, rng)


def basket(rng, hidden=None):
    """Open square basket: floor and four walls (box surfaces); hidden: wall "x-", "x+", "y-" or "y+" left out."""
    inner, height = BASKET_HALF - BASKET_WALL, RIM_Z - TABLE_TOP - BASKET_WALL
    t, z = BASKET_WALL / 2, TABLE_TOP + BASKET_WALL + height / 2
    walls = {"x-": ([-inner - t, 0, z], [t, inner, height / 2]), "x+": ([inner + t, 0, z], [t, inner, height / 2]),
             "y-": ([0, -inner - t, z], [BASKET_HALF, t, height / 2]), "y+": ([0, inner + t, z], [BASKET_HALF, t, height / 2])}
    parts = [box([*BASKET, TABLE_TOP + t], [BASKET_HALF, BASKET_HALF, t], rng)]
    parts += [box(np.add(c, [*BASKET, 0]), h, rng) for name, (c, h) in walls.items() if name != hidden]
    return np.concatenate(parts)


def look(camera, *surfaces, rng):
    """Keep the nearest surface point per camera pixel (0.005 rad, like 320 px over 91 deg); depth noise along the ray."""
    points = np.concatenate(surfaces)
    ray = points - camera
    dist = np.linalg.norm(ray, axis=1)
    unit = ray / dist[:, None]
    pixel = np.floor(np.c_[np.arctan2(unit[:, 1], unit[:, 0]), np.arcsin(unit[:, 2])] / 0.005).astype(int)
    order = np.lexsort((dist, pixel[:, 1], pixel[:, 0]))
    first = np.r_[True, np.any(np.diff(pixel[order], axis=0) != 0, axis=1)]
    seen = order[first]
    return camera + unit[seen] * (dist[seen] + rng.normal(0, NOISE, len(seen)))[:, None]


def find_ball(points):
    return perception.find_ball(points, RADIUS, TABLE_TOP)


def test_points_from_depth():
    # camera at (1, 2, 3) looking straight down (MuJoCo: along -z, +y up in the image), fovy 90 deg: focal = h / 2 = 1 px
    depth = np.full((2, 4), 2.0)
    points = perception.points_from_depth(depth, np.array([1.0, 2.0, 3.0]), np.eye(3).flatten(), 90.0)
    assert points.shape == (8, 3)
    assert points[:, 2] == pytest.approx(1.0)
    assert points[0] == pytest.approx([1.0 + (0.5 - 2.0) * 2.0, 2.0 - (0.5 - 1.0) * 2.0, 1.0])
    v, u = np.array([1, 0]), np.array([3, 2])
    assert perception.points_from_depth(depth, [1.0, 2.0, 3.0], np.eye(3), 90.0, (v, u)) == pytest.approx(
        points[[1 * 4 + 3, 0 * 4 + 2]])


@pytest.mark.parametrize("camera", CAMERAS)
def test_sphere_seen_from_one_side(camera):
    rng = np.random.default_rng(0)
    points = look(CAMERAS[camera], table(rng), sphere(BALL, RADIUS, rng), rng=rng)
    assert np.linalg.norm(find_ball(points) - BALL) <= 0.002


@pytest.mark.parametrize("camera", CAMERAS)
def test_ball_on_its_stand_next_to_the_basket(camera):
    # the ball touches its stand, so both end up in one cluster; the trimmed refit has to drop the stand
    rng = np.random.default_rng(1)
    points = look(CAMERAS[camera], table(rng), stand(rng), sphere(BALL, RADIUS, rng), basket(rng), rng=rng)
    assert np.linalg.norm(find_ball(points) - BALL) <= 0.002


@pytest.mark.parametrize("camera", CAMERAS)
def test_grey_sphere_next_to_cylinders_and_a_box(camera):
    # distractors of similar size; a red and a grey cylinder look the same here, because the cloud has no colour
    rng = np.random.default_rng(2)
    red_cylinder = cylinder(0.45, 0.0, 0.06, 0.2, rng)
    grey_cylinder = cylinder(0.65, 0.25, 0.105, 0.21, rng)
    grey_box = box([0.344, -0.241, TABLE_TOP + 0.1], [0.1, 0.1, 0.1], rng)
    points = look(CAMERAS[camera], table(rng), stand(rng), sphere(BALL, RADIUS, rng), red_cylinder, grey_cylinder,
                  grey_box, rng=rng)
    assert np.linalg.norm(find_ball(points) - BALL) <= 0.002


@pytest.mark.parametrize("camera", CAMERAS)
@pytest.mark.parametrize("thing", ["cylinder same radius", "thin cylinder", "flat puck", "box", "basket"])
def test_no_sphere_no_ball(camera, thing):
    rng = np.random.default_rng(3)
    make = {"cylinder same radius": lambda: cylinder(0.4, 0.1, 0.105, 0.21, rng),
            "thin cylinder": lambda: cylinder(0.4, 0.1, 0.06, 0.2, rng),
            "flat puck": lambda: cylinder(0.4, 0.1, 0.105, 0.08, rng),
            "box": lambda: box([0.4, 0.1, TABLE_TOP + 0.1], [0.1, 0.1, 0.1], rng),
            "basket": lambda: basket(rng)}
    assert find_ball(look(CAMERAS[camera], table(rng), make[thing](), rng=rng)) is None


@pytest.mark.parametrize("camera", CAMERAS)
@pytest.mark.parametrize("radius", [0.07, 0.09, 0.12, 0.14])
def test_sphere_of_another_size_no_ball(camera, radius):
    rng = np.random.default_rng(8)
    other = sphere(np.array([0.45, 0.0, TABLE_TOP + radius]), radius, rng)
    assert find_ball(look(CAMERAS[camera], table(rng), other, rng=rng)) is None


def test_too_few_points_no_ball():
    rng = np.random.default_rng(4)
    assert find_ball(np.empty((0, 3))) is None
    few = sphere(BALL, RADIUS, rng)[:perception.MIN_PIXELS - 1]
    assert find_ball(few) is None


def test_floor_reflection_is_ignored():
    # a mirror image of the ball below the table (shiny floor), noise-free: it would fit best if it were not dropped
    rng = np.random.default_rng(5)
    seen = look(CAMERAS["stop"], table(rng), stand(rng), sphere(BALL, RADIUS, rng), basket(rng), rng=rng)
    mirror = sphere(BALL * [1, 1, -1], RADIUS, rng)
    assert np.linalg.norm(find_ball(np.concatenate([seen, mirror])) - BALL) <= 0.002
    rim_mirror = basket(rng) * [1, 1, -1]
    centre = perception.find_basket(np.concatenate([seen, rim_mirror]), BALL, RIM_Z, RADIUS)
    assert np.linalg.norm(centre - BASKET) <= 0.005


@pytest.mark.parametrize("camera", CAMERAS)
@pytest.mark.parametrize("hidden", [None, "x-", "x+", "y-", "y+"])
def test_basket_ring_with_one_wall_hidden(camera, hidden):
    rng = np.random.default_rng(6)
    points = look(CAMERAS[camera], table(rng), stand(rng), sphere(BALL, RADIUS, rng), basket(rng, hidden), rng=rng)
    for outer_size in (None, 2 * BASKET_HALF):
        centre = perception.find_basket(points, find_ball(points), RIM_Z, RADIUS, outer_size)
        assert np.linalg.norm(centre - BASKET) <= 0.005


def test_basket_missing_or_cut_off():
    rng = np.random.default_rng(7)
    camera = CAMERAS["stop"]
    no_basket = look(camera, table(rng), stand(rng), sphere(BALL, RADIUS, rng), rng=rng)
    assert perception.find_basket(no_basket, BALL, RIM_Z, RADIUS) is None
    # only the half with y > BASKET[1] in view (image edge): wrong size, so None when the size is known
    points = look(camera, table(rng), stand(rng), sphere(BALL, RADIUS, rng), basket(rng), rng=rng)
    half = points[points[:, 1] > BASKET[1]]
    assert perception.find_basket(half, BALL, RIM_Z, RADIUS) is not None
    assert perception.find_basket(half, BALL, RIM_Z, RADIUS, 2 * BASKET_HALF) is None


@pytest.mark.parametrize("camera", CAMERAS)
@pytest.mark.parametrize("where", ["front", "outer side", "towards the ball"])
def test_basket_next_to_a_tall_object(camera, where):
    # distractors (section I) stand >= 0.03 m from the basket and cross the rim height; the square of the known size leaves them out
    rng = np.random.default_rng(9)
    gap = 0.03
    make = {"front": lambda: cylinder(BASKET[0] - BASKET_HALF - gap - 0.04, BASKET[1], 0.04, 0.2, rng),
            "outer side": lambda: cylinder(BASKET[0], BASKET[1] - BASKET_HALF - gap - 0.04, 0.04, 0.2, rng),
            "towards the ball": lambda: box([BASKET[0], BASKET[1] + BASKET_HALF + gap + 0.04, TABLE_TOP + 0.1],
                                            [0.04, 0.04, 0.1], rng)}
    points = look(CAMERAS[camera], table(rng), stand(rng), sphere(BALL, RADIUS, rng), basket(rng), make[where](), rng=rng)
    for ball in (find_ball(points), None):
        centre = perception.find_basket(points, ball, RIM_Z, RADIUS, 2 * BASKET_HALF)
        assert np.linalg.norm(centre - BASKET) <= 0.005
