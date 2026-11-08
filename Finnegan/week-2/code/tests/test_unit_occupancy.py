"""Unit: Belegungskarte, Kostenkarte, A* (g1sim/occupancy.py) und Bahnverfolgung (navigation.PathFollower) - Lernpfad L2.

Ein Fehler hier faellt erst in der minutenlangen Abnahme scripts/62_rooms_check.py auf, z. B. als Weg durch die Wand
oder als Roboter, der an der Tuerkante haengen bleibt.
"""
import math

import numpy as np
import pytest

from g1sim import navigation as nav
from g1sim import occupancy as occ
from g1sim import rooms

pytestmark = pytest.mark.unit


def wall_with_door(door=(0.4, 1.6)):
    """4 x 4 m mit Trennwand bei x = 2 und Tuer bei y in door."""
    g = occ.Grid(((0.0, 4.0), (-2.0, 2.0)))
    g.add_box((2.0, (-2.0 + door[0]) / 2), (0.05, (door[0] + 2.0) / 2))
    g.add_box((2.0, (door[1] + 2.0) / 2), (0.05, (2.0 - door[1]) / 2))
    return g


def test_kaestchen_und_welt():
    g = occ.Grid(((-1.0, 1.0), (0.0, 1.0)))
    assert g.shape == (20, 40)
    row, col = g.to_cell(0.12, 0.33)
    assert g.to_world(row, col) == pytest.approx((0.125, 0.325))


def test_wandabstand_waechst_mit_der_entfernung():
    g = occ.Grid(((0.0, 2.0), (0.0, 2.0)))
    g.add_box((0.025, 1.0), (0.025, 1.0))  # Wand in der ersten Spalte
    clear = g.clearance()
    assert clear[20, 0] == 0.0
    assert clear[20, 10] == pytest.approx(0.5)


def test_kostenkarte_unpassierbar_nah_an_der_wand():
    cost = wall_with_door().costmap()
    g = wall_with_door()
    assert np.isinf(cost[g.to_cell(2.2, -1.0)])           # 0,2 m neben der Wand: der Koerper stiesse an
    assert cost[g.to_cell(3.5, -1.0)] == pytest.approx(1.0)  # 1,5 m weg: freie Flaeche


def test_astar_geht_durch_die_tuermitte():
    g = wall_with_door()
    cost = g.costmap()
    cells = occ.astar(cost, g.to_cell(0.5, -1.5), g.to_cell(3.5, -1.5))
    pts = [g.to_world(*c) for c in cells]
    at_wall = [y for x, y in pts if abs(x - 2.0) < 0.03]
    assert at_wall and all(abs(y - 1.0) <= 0.1 for y in at_wall)  # Tuer 0,4..1,6 -> Mitte 1,0
    assert all(np.isfinite(cost[c]) for c in cells[1:])


def test_astar_ohne_tuer_kein_weg():
    g = wall_with_door(door=(1.0, 1.0))
    assert occ.astar(g.costmap(), g.to_cell(0.5, 0.0), g.to_cell(3.5, 0.0)) is None


def test_astar_leere_karte_gerade():
    g = occ.Grid(((0.0, 3.0), (0.0, 1.0)))
    cells = occ.astar(g.costmap(), g.to_cell(0.1, 0.5), g.to_cell(2.9, 0.5))
    assert occ.path_length([g.to_world(*c) for c in cells]) == pytest.approx(2.8, abs=0.06)


def test_raumszene_start_frei_und_tuer_offen():
    g = rooms.grid()
    cost = g.costmap()
    assert np.isfinite(cost[g.to_cell(0.0, 0.0)])
    assert occ.astar(cost, g.to_cell(0.0, 0.0), g.to_cell(5.5, 1.0)) is not None
    assert rooms.room_of(0.0, 0.0) == "A" and rooms.room_of(5.5, 1.0) == "B"


def test_bahnverfolgung_peilt_voraus_und_am_ende_das_ziel():
    path = [(0.05 * k, 0.0) for k in range(41)]  # 2 m gerade nach +x
    goal = nav.Pose2D(2.0, 0.0, math.pi / 2)
    f = nav.PathFollower(path, goal)
    ahead = f.target(nav.Pose2D(0.0, 0.0, 0.0))
    assert ahead.x == pytest.approx(nav.LOOKAHEAD, abs=0.06) and ahead.yaw == pytest.approx(0.0)
    assert f.target(nav.Pose2D(1.8, 0.0, 0.0)) == goal
