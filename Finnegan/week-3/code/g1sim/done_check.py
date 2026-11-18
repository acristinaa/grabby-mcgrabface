"""Done check of the policy phase: has the policy finished one grasp-and-place attempt? Decided from its own hand commands.

Why it exists: the VLA has no notion of "task done". Its demos end right after the place, so after a place it starts
reaching for the stand again (seen in the 82 videos, 2026-11-16). On the robot only the operator's Ctrl-C or --max-seconds
ended the policy phase. This check ends it as soon as one attempt is over, through the same soft exit as --max-seconds.
It only ever ends the phase earlier; it never commands anything.

Signal: the teacher's hand schedule, which the policy copies. Fingers 1-2 (thumb pair, slots 0-1 of each hand) go
ready 0.10 -> open 0.00 (reach) -> grip 0.30 (lift, carry) -> open 0.00 (release over the basket) -> ready 0.10 (relaxed).
DONE = a grip, then a release, then the relaxed hands held RELAX_S. Times are policy time (actions at FPS), so the check
behaves the same at --speed 0.25 and 0.5. Calibrated on 50 run3 trials in the sim (runs/states_run3, 2026-11-16).
Pure numpy, no MuJoCo, no DDS.
"""
import numpy as np

FPS = 30                        # policy actions per second (the dataset's fps)
THUMBS = [0, 1, 6, 7]           # fingers 1-2 of both hands in the 12 hand values (left 6, right 6)
GRIP_MIN, GRIP_S = 0.25, 0.5    # grip: thumb mean >= 0.25 for 0.5 s (teacher grip 0.30, ready 0.10)
OPEN_MAX, OPEN_S = 0.05, 0.3    # release: every hand value <= 0.05 for 0.3 s (teacher open 0.00)
RELAX_LO, RELAX_HI = 0.06, 0.20 # relaxed: every hand value in this band (teacher ready: thumbs 0.10, other fingers 0.15)
RELAX_S = 1.0                   # relaxed for 1 s after the release -> DONE
STAGES = ("wait_grip", "gripping", "released", "done")


class DoneCheck:
    """Feed one policy action's 12 hand values per action (update); `done` turns True once, after grip -> release -> relax."""

    def __init__(self):
        self.stage, self.run, self.events = "wait_grip", 0, []

    @property
    def done(self) -> bool:
        return self.stage == "done"

    def update(self, hands12, index: int) -> bool:
        """hands12: the action's hand values (left 6, right 6). index: the action's number (for the event log). True once
        DONE is reached (and stays True)."""
        if self.done:
            return True
        h = np.asarray(hands12, dtype=float)
        if self.stage == "wait_grip":
            self._advance(h[THUMBS].mean() >= GRIP_MIN, GRIP_S, "gripping", index)
        elif self.stage == "gripping":
            self._advance(h.max() <= OPEN_MAX, OPEN_S, "released", index)
        elif self.stage == "released":
            self._advance(bool(((h >= RELAX_LO) & (h <= RELAX_HI)).all()), RELAX_S, "done", index)
        return self.done

    def _advance(self, condition: bool, seconds: float, next_stage: str, index: int) -> None:
        self.run = self.run + 1 if condition else 0
        if self.run >= round(seconds * FPS):
            self.stage, self.run = next_stage, 0
            self.events.append((next_stage, index))
