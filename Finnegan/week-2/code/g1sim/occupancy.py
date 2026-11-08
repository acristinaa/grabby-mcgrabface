"""Belegungskarte, Kostenkarte und A* (Lernpfad L2) - reines numpy, ohne MuJoCo, laeuft in jeder Umgebung.

Kette:
  Hindernisse (Rechtecke) -> Belegungskarte: Boden in RESOLUTION-Kaestchen, jedes frei oder belegt
  -> Wandabstand je Kaestchen (Brushfire: Abstand breitet sich von den Waenden aus wie ein Lauffeuer)
  -> Kostenkarte: naeher als ROBOT_RADIUS = unpassierbar (der Koerper wuerde anstossen), bis SOFT_RADIUS steigende Kosten
  -> A*: guenstigster Weg durch die Kaestchen
So arbeitet auch Nav2 am echten Roboter ("inscribed radius" + "inflation layer"). Die Kosten schieben den Weg von den
Waenden weg, z. B. in die Tuermitte statt haarscharf an die Tuerkante.
Zeilen = y, Spalten = x (wie ein Bild von oben).
"""
import heapq
import math

import numpy as np

RESOLUTION = 0.05     # m je Kaestchen
ROBOT_RADIUS = 0.30   # m: G1 ist 0,45 m breit (Unitree-Datenblatt) + Arme und Haende -> darunter stoesst der Koerper an
SOFT_RADIUS = 0.80    # m: bis zu diesem Wandabstand kostet ein Kaestchen mehr
WALL_COST = 20.0      # Zusatzkosten direkt an der Grenze ROBOT_RADIUS, faellt bis SOFT_RADIUS quadratisch auf 0

NEIGHBORS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


class Grid:
    """Belegungskarte ueber dem Rechteck bounds = ((x_min, x_max), (y_min, y_max)) in Weltkoordinaten."""

    def __init__(self, bounds, resolution=RESOLUTION):
        (x0, x1), (y0, y1) = bounds
        self.origin = np.array([x0, y0], dtype=float)
        self.res = resolution
        self.shape = (int(round((y1 - y0) / resolution)), int(round((x1 - x0) / resolution)))
        self.occupied = np.zeros(self.shape, dtype=bool)

    def to_cell(self, x, y):
        """Weltpunkt -> (Zeile, Spalte), auf den Kartenrand begrenzt."""
        col = int(np.clip((x - self.origin[0]) // self.res, 0, self.shape[1] - 1))
        row = int(np.clip((y - self.origin[1]) // self.res, 0, self.shape[0] - 1))
        return row, col

    def to_world(self, row, col):
        """Mitte eines Kaestchens in Weltkoordinaten."""
        return (float(self.origin[0] + (col + 0.5) * self.res), float(self.origin[1] + (row + 0.5) * self.res))

    def add_box(self, center, half, yaw=0.0):
        """Rechteck (Mitte, halbe Kantenlaengen, Drehung) als belegt eintragen: jedes Kaestchen, dessen Mitte darin liegt."""
        rows, cols = np.indices(self.shape)
        x = self.origin[0] + (cols + 0.5) * self.res - center[0]
        y = self.origin[1] + (rows + 0.5) * self.res - center[1]
        c, s = math.cos(yaw), math.sin(yaw)
        u, v = c * x + s * y, -s * x + c * y  # ins System des Rechtecks drehen
        self.occupied |= (np.abs(u) <= half[0] + 1e-9) & (np.abs(v) <= half[1] + 1e-9)

    def clearance(self):
        """Abstand jedes Kaestchens zur naechsten Wand in m (Brushfire mit Dijkstra, 8 Nachbarn).
        Kaestchen ohne Wand in der Karte bekommen unendlich."""
        dist = np.full(self.shape, np.inf)
        heap = [(0.0, r, c) for r, c in zip(*np.nonzero(self.occupied))]
        for _, r, c in heap:
            dist[r, c] = 0.0
        heapq.heapify(heap)
        while heap:
            d, r, c = heapq.heappop(heap)
            if d > dist[r, c]:
                continue
            for dr, dc in NEIGHBORS:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.shape[0] and 0 <= nc < self.shape[1]:
                    nd = d + self.res * (math.sqrt(2.0) if dr and dc else 1.0)
                    if nd < dist[nr, nc]:
                        dist[nr, nc] = nd
                        heapq.heappush(heap, (nd, nr, nc))
        return dist

    def costmap(self, robot_radius=ROBOT_RADIUS, soft_radius=SOFT_RADIUS, wall_cost=WALL_COST):
        """Kosten je Kaestchen: unendlich = unpassierbar, sonst >= 1 (1 = freie Flaeche weit weg von Waenden)."""
        d = self.clearance()
        closeness = np.clip((soft_radius - d) / (soft_radius - robot_radius), 0.0, 1.0)
        cost = 1.0 + wall_cost * closeness ** 2
        cost[d < robot_radius] = np.inf
        return cost


def astar(cost, start, goal):
    """Guenstigster Weg von start nach goal (je (Zeile, Spalte)) durch die Kostenkarte, 8 Nachbarn.

    A* erweitert immer das Kaestchen mit der kleinsten Summe aus bisherigen Kosten g und geschaetzten Restkosten h.
    h ist die Diagonal-Entfernung ohne Hindernisse; weil jedes Kaestchen mindestens 1 kostet, unterschaetzt h nie -
    deshalb ist der gefundene Weg der guenstigste. Das Startkaestchen darf teuer oder unpassierbar sein (der Roboter
    steht nun einmal dort). Rueckgabe: Liste der Kaestchen vom Start zum Ziel, None ohne Weg.
    """
    if not np.isfinite(cost[goal]):
        return None

    def h(cell):
        dr, dc = abs(cell[0] - goal[0]), abs(cell[1] - goal[1])
        return max(dr, dc) + (math.sqrt(2.0) - 1.0) * min(dr, dc)

    g = {start: 0.0}
    came = {}
    heap = [(h(start), start)]
    while heap:
        _, cell = heapq.heappop(heap)
        if cell == goal:
            path = [cell]
            while cell in came:
                cell = came[cell]
                path.append(cell)
            return path[::-1]
        for dr, dc in NEIGHBORS:
            nxt = (cell[0] + dr, cell[1] + dc)
            if not (0 <= nxt[0] < cost.shape[0] and 0 <= nxt[1] < cost.shape[1]) or not np.isfinite(cost[nxt]):
                continue
            step = (math.sqrt(2.0) if dr and dc else 1.0) * cost[nxt]  # ein Kaestchen betreten: Schrittlaenge x Kosten
            new_g = g[cell] + step
            if new_g < g.get(nxt, math.inf):
                g[nxt] = new_g
                came[nxt] = cell
                heapq.heappush(heap, (new_g + h(nxt), nxt))
    return None


def path_length(points):
    """Laenge eines Wegs aus Weltpunkten in m."""
    pts = np.asarray(points, dtype=float)
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum()) if len(pts) > 1 else 0.0


# --- Smooth path (2026-10-08/09): the raw A* path zig-zags on the grid (a direction change at ~20 % of its steps) and the robot
# wobbled 9-10 cm around it (max 28 cm, 68 seeds 1 and 3). shortcut() pulls it taut, smooth() rounds the corners.
def segment_cost(cost, grid, a, b):
    """Highest cost of the cells on the straight line a -> b (world points), sampled every half cell. inf if blocked."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(2, int(np.ceil(np.linalg.norm(b - a) / (grid.res / 2))) + 1)
    return max(float(cost[grid.to_cell(*p)]) for p in np.linspace(a, b, n))


def shortcut(points, cost, grid, max_cost=5.0):
    """Pull a path taut: from each kept point jump to the furthest later point whose straight line is free and not worse than
    the original stretch it replaces (highest cost <= max(max_cost, highest cost of that stretch)). The default 5 keeps the
    line ~0.58 m from walls and furniture (costmap: 1 + 20 * closeness^2; 10 = ~0.45 m hugged the table corners). Returns the
    kept points (world)."""
    pts = [np.asarray(p, float) for p in points]
    cell_cost = [float(cost[grid.to_cell(*p)]) for p in pts]
    kept, i = [pts[0]], 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1:
            allowed = max(max_cost, max(cell_cost[i:j + 1]))
            if segment_cost(cost, grid, pts[i], pts[j]) <= allowed:
                break
            j -= 1
        kept.append(pts[j])
        i = j
    return [tuple(map(float, p)) for p in kept]


def smooth(points, iterations=3, spacing=RESOLUTION):
    """Round the corners of a polyline (Chaikin: each pass cuts every corner at 1/4 and 3/4), keep both ends, resample
    every `spacing` m."""
    pts = np.asarray(points, float)
    for _ in range(iterations):
        if len(pts) < 3:
            break
        a, b = pts[:-1], pts[1:]
        cut = np.empty((2 * len(a), 2))
        cut[0::2], cut[1::2] = 0.75 * a + 0.25 * b, 0.25 * a + 0.75 * b
        pts = np.vstack([pts[0], cut[1:-1], pts[-1]])
    return resample(pts, spacing)


def resample(points, spacing=RESOLUTION):
    """Points every `spacing` m along a polyline (last point included)."""
    pts = np.asarray(points, float)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    if s[-1] == 0.0:
        return [tuple(map(float, pts[0]))]
    at = np.arange(0.0, s[-1], spacing)
    at = np.append(at[at < s[-1] - 1e-6], s[-1])  # the end once (arange can already hit it, then it came twice)
    return [(float(x), float(y)) for x, y in zip(np.interp(at, s, pts[:, 0]), np.interp(at, s, pts[:, 1]))]


def smooth_path(points, cost, grid, max_cost=5.0, tail=()):
    """shortcut() + smooth(), with fewer smoothing passes if rounding a corner would cut too close to an obstacle (corners are
    cut on the inside of the turn, where the obstacle usually is). Never worse than the A* path it starts from.
    tail: fixed points after the path (e.g. the straight lead-in onto the line into the table): not shortcut, but the corner
    where the path joins them is rounded too; the last tail segment stays on its line (Chaikin keeps the end point and
    cuts that segment only at 1/4 and 3/4 of its length). Without this the join was a corner of up to 136 degrees."""
    tail = [tuple(map(float, p)) for p in tail]
    taut = shortcut(points, cost, grid, max_cost)
    if tail and len(taut) > 1:
        taut = taut[:-1]  # the path's last point is the grid cell at the tail's start: a stub there blocks the rounding
    taut = taut + tail
    limit = max(max_cost, max(float(cost[grid.to_cell(*p)]) for p in list(points) + tail))
    for iterations in (3, 2, 1, 0):
        path = smooth(taut, iterations)
        if max(float(cost[grid.to_cell(*p)]) for p in path) <= limit:
            return path
    return resample(list(points) + tail)
