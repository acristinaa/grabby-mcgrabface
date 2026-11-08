"""Natural arm swing while walking (2026-10-09, user's request): each arm swings opposite to its leg, like a person.

When the right foot is in front, the left arm is forward and the right arm back, and the other way round. The signal is the
feet themselves: (x of the right foot - x of the left foot) in the pelvis frame. In sim it comes from the body positions; on
the real G1 the same number comes from the leg joint angles (forward kinematics), so sim and real use the same input.
Measured on GR00T (2026-10-09, arms down): one step cycle takes ~0.75 s; the feet are +-6.5 cm apart at 0.15 m/s, +-9 cm at
0.25 m/s, +-12.5 cm at 0.35 m/s; standing, they stay ~4 cm apart -> the slow average is taken out (high-pass), otherwise one
arm would stay forward while the robot stands.
Pure numpy, no MuJoCo: step(foot_gap, dt) -> (left, right) offsets for (shoulder pitch, elbow) in rad.
Shoulder pitch: negative = arm forward (checked by forward kinematics: +0.2 rad moved the hands 8 cm back).
"""
import dataclasses
import math

import numpy as np


@dataclasses.dataclass(frozen=True)
class SwingParams:
    """A slight, slow nudge forward and back (user, 2026-10-09). First version: +-15-23 deg, snapping with every 0.75 s step
    cycle - it looked hectic and the path deviation rose to 11-15 cm. GR00T's step rhythm cannot be slowed, so the swing is
    kept small and smoothed instead."""
    gain: float = 1.0            # rad of shoulder pitch per m of foot gap: +-9 cm at 0.25 m/s -> +-0.09 rad (~5 deg)
    max_pitch: float = 0.12      # rad (~7 deg) either way
    elbow_forward: float = 0.0   # no extra elbow bend
    centre_s: float = 1.5        # s, time constant of the slow average of the foot gap that is taken out (~2 step cycles)
    smooth_s: float = 0.15       # s, low-pass: the arm flows instead of snapping (at the 0.75 s cycle ~0.1 s behind the feet)
    fade_s: float = 0.8          # s, how slowly the swing fades in when walking starts and out when it stops
    max_rate: float = 0.5        # rad/s (~29 deg/s): the arm never moves faster than this, so it cannot snap


class ArmSwing:
    """step(foot_gap, dt, walking) once per control tick. foot_gap = right foot x - left foot x (m, pelvis frame, forward +).
    walking: True while the robot is told to walk (the swing fades in), False while it stands (fades out)."""

    def __init__(self, params=SwingParams()):
        self.p = params
        self.centre = None
        self.gap = 0.0
        self.amount = 0.0  # 0 standing .. 1 walking
        self.swing = 0.0

    def step(self, foot_gap, dt, walking=True):
        p = self.p
        if self.centre is None:
            self.centre = self.gap = float(foot_gap)
        self.centre += (foot_gap - self.centre) * min(1.0, dt / p.centre_s)
        self.gap += (foot_gap - self.gap) * min(1.0, dt / p.smooth_s)
        self.amount += ((1.0 if walking else 0.0) - self.amount) * min(1.0, dt / p.fade_s)
        target = self.amount * float(np.clip(p.gain * (self.gap - self.centre), -p.max_pitch, p.max_pitch))
        self.swing += float(np.clip(target - self.swing, -p.max_rate * dt, p.max_rate * dt))
        swing = self.swing
        # right foot forward (swing > 0): left arm forward (pitch -), right arm back (pitch +)
        left_pitch, right_pitch = -swing, swing
        return ((left_pitch, p.elbow_forward * max(0.0, -left_pitch)),
                (right_pitch, p.elbow_forward * max(0.0, -right_pitch)))

    def apply(self, arms, offsets, pitch_index=(0, 7), elbow_index=(3, 10)):
        """27-D action vector with the swing offsets added (left arm 0-6, right arm 7-13, as action_space)."""
        out = np.array(arms, dtype=np.float32, copy=True)
        for (pitch, elbow), pi, ei in zip(offsets, pitch_index, elbow_index):
            out[pi] += pitch
            out[ei] += elbow
        return out


def swing_deg(offsets):
    """Left shoulder pitch offset in degrees (+ = left arm forward), for the live view."""
    return -math.degrees(offsets[0][0])
