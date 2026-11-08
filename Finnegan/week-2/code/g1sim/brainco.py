"""BrainCo Revo2: Motorreihenfolge, Normierung und DDS-Topics.

Gemeinsame Quelle fuer Sim-Bau, Sim-Bruecke und Tests. Wandert in Stufe 1.2 ins 26-D-Aktionsraum-Modul.

Echte Haende (xr_teleoperate, geprueft 2026-09-14): 6 Motoren je Hand, q normiert 0..1, 0 = offen, 1 = zu,
Reihenfolge Daumen, Daumen-Aux, Zeige, Mittel, Ring, Klein. Reale Wege 1,52 / 1,05 / 1,47 rad.
"""
import numpy as np

# Kein mujoco-Import hier: action_space.py (und damit openpi_g1) nutzt dieses Modul auch in der openpi-Umgebung.
PREFIX = {"left": "lh_", "right": "rh_"}  # wie dashboard.scene.PREFIX

SIDES = ("left", "right")

# Aktuierte Gelenke in der Reihenfolge des echten BrainCo-DDS-Dienstes
MOTORS = ("thumb_metacarpal", "thumb_proximal", "index_proximal", "middle_proximal", "ring_proximal", "pinky_proximal")

# "Ganz zu" in der Sim = ctrlrange-Maximum der Herstellermodelle (rad), gleiche Reihenfolge wie MOTORS
SIM_CLOSED_RAD = np.array([1.51, 1.04, 1.46, 1.46, 1.46, 1.46])

TOPIC_CMD = "rt/brainco/{side}/cmd"      # unitree_go MotorCmds_
TOPIC_STATE = "rt/brainco/{side}/state"  # unitree_go MotorStates_

# PD-Regler, mit dem die Sim die handeigene Positionsregelung nachbildet. Die Sim rechnet explizit mit 4 ms
# Schrittweite; ein Fingergelenk hat ~0,01 kg m^2 Traegheit (armature), dazu Gelenkdaempfung 0,5 aus dem
# Herstellermodell. Sprungantwort: scripts/51_build_brainco_sim.py.
HAND_KP = 20.0  # Nm/rad
HAND_KD = 0.3   # Nm s/rad


def joint_names(side):
    """MuJoCo-Namen der 6 aktuierten Gelenke einer Hand, Reihenfolge wie MOTORS."""
    return [f"{PREFIX[side]}{side}_{m}_joint" for m in MOTORS]


def to_rad(q_norm):
    """Normiert (0 = offen, 1 = zu) -> Sim-Gelenkwinkel. Werte ausserhalb 0..1 werden begrenzt."""
    return np.clip(np.asarray(q_norm, dtype=float), 0.0, 1.0) * SIM_CLOSED_RAD


def to_norm(q_rad):
    """Sim-Gelenkwinkel -> normiert. Nicht begrenzt, damit Ueberschwingen sichtbar bleibt."""
    return np.asarray(q_rad, dtype=float) / SIM_CLOSED_RAD


def tips_in_hand_frame(model, data, side):
    """Positionen der 5 Endglieder im Koordinatensystem der Handbasis (m) - unabhaengig davon, wie Arm und Koerper stehen."""
    from dashboard.scene import distal_body_ids, hand_base_id

    base = hand_base_id(model, side)
    rot = data.xmat[base].reshape(3, 3)
    return (data.xpos[distal_body_ids(model, side)] - data.xpos[base]) @ rot
