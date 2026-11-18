"""Stufe 1.4 - rote Flasche im Kamerabild finden: Farbmaske + Tiefenbild -> Deckelmitte in Weltkoordinaten.

Platzhalter fuer das Bildmodell aus Stufe 1.3: Dort liefert das Bildmodell, welche Pixel zur Flasche gehoeren, hier eine
Farbschwelle. Die Geometrie danach bleibt: Jedes Pixel wird mit seiner Tiefe (Entfernung zur Kamera) in einen 3D-Punkt
umgerechnet. Die Kopfkamera schaut von oben auf die Flasche; die Punkte der Deckelflaeche liegen alle auf gleicher Hoehe,
ihr Mittelpunkt liegt auf der Flaschenachse.

Level 1 walk-up (volleyball): find_ball and find_basket work on the 3D points of the whole depth image, by shape only. The
real ball is grey, so colour is never a filter: the ball is the point cluster that fits a sphere of the known radius, the
basket is the ring of points at its rim height.
"""
import mujoco
import numpy as np

CAP_BAND = 0.004  # m; Punkte bis so weit unter dem Deckel zaehlen zum Deckel
MIN_PIXELS = 50   # weniger rote Pixel = Flasche nicht gefunden
TABLE_MARGIN = 0.01  # m; ball points must be this far above the table top (drops the table and the floor reflection)
CLUSTER_VOXEL = 0.01  # m; points in touching voxels of this size belong to the same object
MAX_SPHERE_RESIDUAL = 0.005  # m; RMS distance of the points on the fitted sphere (depth noise is 0.002)
TRIM = (0.03, 0.015, 0.006)  # m; refits on the points this close to the sphere: drops the stand the ball rests on
RADIUS_TOL = 0.01  # m; a sphere fit with free radius may differ this much from the known radius
MIN_ON_SPHERE = 0.6  # share of a cluster that must lie on the sphere (a cylinder or a box touches it only in a band)
MIN_CURVATURE = 3.0  # a plane and an upright cylinder must fit this many times worse than the sphere (a wall strip or a
# band of a can fits a sphere of the same radius well)
RIM_BAND = 0.015  # m; basket rim points lie within rim_z +- RIM_BAND
BALL_KEEP_OUT = 0.05  # m; basket points closer than ball radius + this (horizontally) to the ball belong to ball or stand
RIM_VOXEL = 0.03  # m; cluster size for the rim: seen from 1 m back the thin rim is a dotted line
BASKET_MIN_POINTS = 100
BASKET_SIZE_TOL = 0.02  # m; a rim box further off the known outer size is cut by the image edge or an arm
BASKET_CELL = 0.005  # m; grid of the search for the basket square among the rim points


def red_mask(image):
    """Deutlich rote Pixel eines RGB-Bilds (uint8) als bool-Maske. Der Tisch (braun) ist nur ~1,3x roter als gruen."""
    rgb = image.astype(np.int16)
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    return (r > 80) & (r > 2 * g) & (r > 2 * b)


def points_from_depth(depth, cam_pos, cam_xmat, fovy, pixels=None):
    """3D world points (N, 3) of depth-image pixels. fovy in degrees (model.cam_fovy); pixels: (v, u) index arrays, e.g.
    np.nonzero(mask); None = every pixel."""
    h, w = depth.shape
    v, u = np.indices(depth.shape).reshape(2, -1) if pixels is None else pixels
    focal = (h / 2.0) / np.tan(np.deg2rad(fovy) / 2.0)  # Pixel
    d = depth[v, u]
    # Kamera schaut entlang -z, +y ist oben (MuJoCo)
    in_camera = np.stack([(u + 0.5 - w / 2.0) / focal * d, -(v + 0.5 - h / 2.0) / focal * d, -d], axis=1)
    return cam_pos + in_camera @ np.reshape(cam_xmat, (3, 3)).T


def cluster_labels(points, voxel=CLUSTER_VOXEL):
    """Label per point; points whose voxels touch (26 neighbours) share a label (connected components)."""
    keys = np.floor(points / voxel).astype(np.int64)
    keys -= keys.min(axis=0) - 1
    cells, inverse = np.unique(keys, axis=0, return_inverse=True)
    span = cells.max(axis=0) + 2
    codes = (cells[:, 0] * span[1] + cells[:, 1]) * span[2] + cells[:, 2]  # sorted, because cells are sorted
    first, second = [], []
    for offset in np.array(np.meshgrid([-1, 0, 1], [-1, 0, 1], [-1, 0, 1])).reshape(3, -1).T:
        neighbour = cells + offset
        code = (neighbour[:, 0] * span[1] + neighbour[:, 1]) * span[2] + neighbour[:, 2]
        index = np.minimum(np.searchsorted(codes, code), len(codes) - 1)
        hit = codes[index] == code
        first.append(np.nonzero(hit)[0])
        second.append(index[hit])
    first, second = np.concatenate(first), np.concatenate(second)
    label = np.arange(len(cells))
    while True:  # every cell takes the smallest label among its neighbours until nothing changes
        lowest = np.minimum(label[first], label[second])
        new = label.copy()
        np.minimum.at(new, first, lowest)
        new = new[new]
        if np.array_equal(new, label):
            return label[inverse.reshape(-1)]
        label = new


def fit_sphere(points, radius, iterations=30):
    """Least-squares centre of a sphere with known radius through points (N, 3). Returns (centre, RMS residual in m)."""
    # start: free-radius fit, linear in centre c and k because |p|^2 = 2 p.c + k; good even for one visible side
    centre = np.linalg.lstsq(np.c_[2 * points, np.ones(len(points))], (points ** 2).sum(axis=1), rcond=None)[0][:3]
    for _ in range(iterations):  # Gauss-Newton on |p - c| - radius with the radius fixed
        diff = points - centre
        dist = np.linalg.norm(diff, axis=1)
        step = np.linalg.lstsq(-diff / dist[:, None], radius - dist, rcond=None)[0]
        centre = centre + step
        if np.linalg.norm(step) < 1e-7:
            break
    residual = np.linalg.norm(points - centre, axis=1) - radius
    return centre, float(np.sqrt(np.mean(residual ** 2)))


def find_ball(points, radius, table_top, max_residual=MAX_SPHERE_RESIDUAL):
    """Ball centre (x, y, z) in world coordinates (m), or None. points: (N, 3) from points_from_depth, no colour filter.

    Points above the table are split into clusters; each cluster gets a sphere fit with the known radius, refitted on the
    points close to the sphere (TRIM), because the ball touches its stand. A cluster counts as a ball when at least
    MIN_ON_SPHERE of it lies on the sphere, those points span at least one radius horizontally, a fit with free radius
    lands within RADIUS_TOL of the known one (a ball of another size fails) and the points are clearly curved
    (MIN_CURVATURE: a plane or an upright cylinder fits clearly worse) and their RMS residual is below max_residual. A
    cylinder, a box or the basket fails. Lowest residual wins.
    """
    points = points[points[:, 2] > table_top + TABLE_MARGIN]
    if len(points) < MIN_PIXELS:
        return None
    labels = cluster_labels(points)
    best, best_residual = None, max_residual
    for label in np.unique(labels):
        cluster = points[labels == label]
        if len(cluster) < MIN_PIXELS:
            continue
        centre, residual = fit_sphere(cluster, radius)
        for tol in TRIM:
            on = cluster[np.abs(np.linalg.norm(cluster - centre, axis=1) - radius) < tol]
            if len(on) < MIN_PIXELS:
                break
            centre, residual = fit_sphere(on, radius)
        if len(on) < max(MIN_PIXELS, MIN_ON_SPHERE * len(cluster)) or np.ptp(on[:, :2], axis=0).max() < radius:
            continue
        free = np.linalg.lstsq(np.c_[2 * on, np.ones(len(on))], (on ** 2).sum(axis=1), rcond=None)[0]  # radius not fixed
        if abs(np.sqrt(max(free[3] + free[:3] @ free[:3], 0.0)) - radius) > RADIUS_TOL:
            continue  # a sphere of another size
        plane = np.linalg.svd(on - on.mean(axis=0), compute_uv=False)[-1] / np.sqrt(len(on))  # RMS distance from a plane
        # RMS distance from an upright cylinder of any radius: circle fit in x, y (linear like the sphere start)
        xy = on[:, :2]
        a, b, k = np.linalg.lstsq(np.c_[2 * xy, np.ones(len(xy))], (xy ** 2).sum(axis=1), rcond=None)[0]
        column = np.linalg.norm(xy - [a, b], axis=1) - np.sqrt(max(k + a * a + b * b, 0.0))
        column = np.sqrt(np.mean(column ** 2))
        if min(plane, column) > MIN_CURVATURE * residual and residual <= best_residual:
            best, best_residual = centre, residual
    return best


def in_densest_square(xy, size):
    """Mask of the points (N, 2) inside the table-aligned square of side size + BASKET_SIZE_TOL (m; scalar or (x, y))
    that holds the most of them. Window sums over a BASKET_CELL grid via a summed-area table."""
    cells = np.floor((xy - xy.min(axis=0)) / BASKET_CELL).astype(np.int64)
    n = np.broadcast_to(np.ceil((np.asarray(size) + BASKET_SIZE_TOL) / BASKET_CELL).astype(np.int64), (2,))
    starts = cells.max(axis=0) + 1  # window start cells per axis
    counts = np.zeros(starts + n)
    np.add.at(counts, (cells[:, 0], cells[:, 1]), 1)
    total = np.pad(counts.cumsum(axis=0).cumsum(axis=1), ((1, 0), (1, 0)))
    (sx, sy), (nx, ny) = starts, n
    windows = total[nx:nx + sx, ny:ny + sy] - total[:sx, ny:ny + sy] - total[nx:nx + sx, :sy] + total[:sx, :sy]
    low = np.unravel_index(np.argmax(windows), windows.shape)
    return np.all((cells >= low) & (cells < low + n), axis=1)


def find_basket(points, ball_centre, rim_z, ball_radius, outer_size=None, band=RIM_BAND):
    """Basket centre (x, y) in world coordinates (m), or None below BASKET_MIN_POINTS rim points.

    By geometry (the basket's colour matches the floor): points in the rim band rim_z +- band, without those within
    ball_radius + BALL_KEEP_OUT of the ball (ball and stand) and without scattered background pixels (clusters below
    MIN_PIXELS); centre of their bounding box. The box stays right with one wall hidden, because the two walls across it
    span the full length. ball_centre None = no ball to drop. outer_size: outer width of the (table-aligned) basket in m;
    if given, only the rim points in the square of that size (+ BASKET_SIZE_TOL) holding the most of them count, so
    another object reaching the rim height more than about 0.025 m away is left out, and a box that differs by more than
    BASKET_SIZE_TOL in x or y gives None (only part of the basket is seen). Without outer_size every rim point counts.
    """
    rim = points[np.abs(points[:, 2] - rim_z) <= band]
    if ball_centre is not None:
        rim = rim[np.linalg.norm(rim[:, :2] - np.asarray(ball_centre)[:2], axis=1) > ball_radius + BALL_KEEP_OUT]
    if len(rim):
        _, label, size = np.unique(cluster_labels(rim, RIM_VOXEL), return_inverse=True, return_counts=True)
        rim = rim[size[label.reshape(-1)] >= MIN_PIXELS]
    if outer_size is not None and len(rim):
        rim = rim[in_densest_square(rim[:, :2], outer_size)]
    if len(rim) < BASKET_MIN_POINTS:
        return None
    low, high = rim[:, :2].min(axis=0), rim[:, :2].max(axis=0)
    if outer_size is not None and np.abs(high - low - outer_size).max() > BASKET_SIZE_TOL:
        return None
    return (low + high) / 2.0


class Locator:
    def __init__(self, model, camera="head_camera", width=640, height=400, noise=None):
        """noise: optional f(rgb, depth) -> (rgb, depth), legt Sensorrauschen auf die Bilder (TableEnv.camera_noise).
        Bildformat 16:10 wie die Farbkamera der D455 (1280 x 800 / 2), siehe table_env.IMAGE_SIZES (Lernpfad L3a)."""
        self.camera, self.noise = camera, noise
        self.cam_id = model.camera(camera).id
        self.fovy = model.cam_fovy[self.cam_id]
        self.renderer = mujoco.Renderer(model, height=height, width=width)

    def locate(self, data):
        """Deckelmitte (x, y, z) der roten Flasche in Weltkoordinaten (m), oder None, wenn sie nicht zu sehen ist."""
        self.renderer.update_scene(data, camera=self.camera)
        rgb = self.renderer.render()
        self.renderer.enable_depth_rendering()
        depth = self.renderer.render()
        self.renderer.disable_depth_rendering()
        if self.noise is not None:
            rgb, depth = self.noise(rgb, depth)

        v, u = np.nonzero(red_mask(rgb))
        if len(u) < MIN_PIXELS:
            return None
        points = points_from_depth(depth, data.cam_xpos[self.cam_id], data.cam_xmat[self.cam_id], self.fovy, (v, u))
        top = np.percentile(points[:, 2], 99)  # robust gegen einzelne Kantenpixel
        return points[points[:, 2] > top - CAP_BAND].mean(axis=0)

    def close(self):
        self.renderer.close()
