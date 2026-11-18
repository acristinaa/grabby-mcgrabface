"""Unit: Folgeregler g1sim/navigation.py (Lernpfad L1) - Koordinaten, Vorzeichen, Reglerverhalten.

Ein Vorzeichenfehler hier laesst den Roboter still in die falsche Richtung laufen; die Abnahme scripts/61_goto_check.py
merkt es erst nach Minuten Sim.
"""
import math

import numpy as np
import pytest

from g1sim import navigation as nav

pytestmark = pytest.mark.unit


def quat_yaw(yaw):
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])  # MuJoCo (w, x, y, z), Drehung um z


@pytest.mark.parametrize("yaw", [0.0, 0.7, -2.5, math.pi - 1e-6])
def test_yaw_aus_quaternion(yaw):
    assert nav.yaw_from_quat(quat_yaw(yaw)) == pytest.approx(yaw, abs=1e-9)


def test_wrap_nimmt_den_kurzen_weg():
    assert nav.wrap(math.radians(350)) == pytest.approx(math.radians(-10))
    assert nav.wrap(math.radians(-190)) == pytest.approx(math.radians(170))


def test_koerpersystem():
    # Roboter schaut nach Welt-y: ein Ziel 1 m in Welt-y liegt geradeaus, eines in Welt-x rechts (y negativ)
    assert nav.to_body(0.0, 1.0, math.pi / 2) == pytest.approx((1.0, 0.0))
    assert nav.to_body(1.0, 0.0, math.pi / 2) == pytest.approx((0.0, -1.0))


def test_joystick_vorzeichen():
    # GR00T: cmd = [ly, -lx, -rx] -> nach links gehen und links drehen heisst lx, rx negativ
    assert nav.to_remote(0.3, 0.2, 0.5) == {"remote.ly": 0.3, "remote.lx": -0.2, "remote.rx": -0.5}


def run(ctrl, pose, goal, ticks=60):
    for _ in range(ticks):
        cmd = ctrl.step(pose, goal)
    return cmd


def test_ziel_voraus_geht_vorwaerts():
    vx, vy, omega = run(nav.GotoController(), nav.Pose2D(0, 0, 0), nav.Pose2D(3, 0, 0))
    assert vx == pytest.approx(nav.GotoParams().max_vx) and vy == 0 and omega == pytest.approx(0)


def test_ziel_dahinter_erst_drehen():
    vx, _, omega = run(nav.GotoController(), nav.Pose2D(0, 0, 0), nav.Pose2D(-3, 0.01, 0))
    assert vx == pytest.approx(0) and omega == pytest.approx(nav.GotoParams().max_omega)


def test_angekommen_steht():
    ctrl = nav.GotoController()
    run(ctrl, nav.Pose2D(0, 0, 0), nav.Pose2D(3, 0, 0))  # in Fahrt
    assert run(ctrl, nav.Pose2D(0, 0, 0), nav.Pose2D(0.05, 0, 0.05)) == (0.0, 0.0, 0.0)


def test_totzone_wird_ueberbrueckt():
    # 11 cm vor dem Ziel waere der P-Befehl 0,11 - knapp; 10,5 cm mit kleinem k laege unter der GR00T-Totzone
    ctrl = nav.GotoController(nav.GotoParams(k_lin=0.3))
    cmd = run(ctrl, nav.Pose2D(0, 0, 0), nav.Pose2D(0.105, 0, 0))
    assert np.linalg.norm(cmd) >= nav.GROOT_DEADBAND


def test_rampe_begrenzt_den_sprung():
    ctrl = nav.GotoController()
    vx, _, _ = ctrl.step(nav.Pose2D(0, 0, 0), nav.Pose2D(3, 0, 0))
    assert vx == pytest.approx(nav.GotoParams().max_accel / nav.CONTROL_HZ)


def test_table_stop_pose_nominal_volleyball_layout():
    """Ball left, basket right, both 0.458 m from the pelvis; facing the middle between them (Level 1 Stage 3)."""
    ball, basket = (0.344, 0.241), (0.344, -0.241)
    stop = nav.table_stop_pose(ball, basket, 0.458)
    assert stop["yaw_deg"] == pytest.approx(0.0, abs=1e-9)
    assert stop["y_m"] == pytest.approx(0.0, abs=1e-9)
    assert stop["x_m"] == pytest.approx(0.344 - math.sqrt(0.458 ** 2 - 0.241 ** 2))
    for xy in (ball, basket):
        assert math.hypot(xy[0] - stop["x_m"], xy[1] - stop["y_m"]) == pytest.approx(0.458)


def test_table_stop_pose_turns_with_the_layout():
    """Ball-basket line turned by 10 degrees -> the robot turns by 10 degrees, ball still on its left."""
    t = math.radians(10.0)
    half = 0.25 * np.array([-math.sin(t), math.cos(t)])
    ball, basket = np.array([0.4, 0.05]) + half, np.array([0.4, 0.05]) - half
    stop = nav.table_stop_pose(ball, basket, 0.46)
    assert stop["yaw_deg"] == pytest.approx(10.0)
    left = ball - [stop["x_m"], stop["y_m"]]
    assert math.degrees(math.atan2(left[1], left[0])) - stop["yaw_deg"] > 0  # ball on the left


def test_table_stop_pose_rejects_too_wide_layout():
    with pytest.raises(ValueError):
        nav.table_stop_pose((0.3, 0.5), (0.3, -0.5), 0.458)
