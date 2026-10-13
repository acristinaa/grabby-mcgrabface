"""Lab view, sim side: a running script reports its progress as files, scripts/65_labview.py shows them in the browser.

Per run (run_id like 20261007-153012_54_bottle) under runs/labview/:
  live/<run_id>.status.json  current trial, phase, running counts, last failure (rewritten on every phase change, at most
                             every 0.5 s otherwise)
  live/<run_id>.jpg          latest live image (only with renderer_getter), at most 10 images per wall second, 20 with watch
  records/<run_id>.json      run record v1, written by finish(): counts, rates, failures by reason, one row per trial
  videos/<run_id>.mp4        replay video of the run (scripts that pass video_path to finish(); 67 does it with --watch)
All files are written atomically (temp file + os.replace), so the server never reads half a file.

No new MuJoCo renderer is created here: the live image borrows a renderer the run already owns (for the grasp scripts the
head-camera renderer of the Locator, which sets its own camera before every look). This module imports MuJoCo only when it
renders, so the server and the unit tests work without it.
watch=S slows the run down to S times real time (1.0 = real time), so a person can follow the phases.
A failing render or file write never stops the run: it is reported once and then skipped.
"""
import collections
import datetime
import json
import os
import pathlib
import subprocess
import time

REPO = pathlib.Path(__file__).resolve().parent.parent
ROOT = REPO / "runs" / "labview"
VIDEO_DIR = ROOT / "videos"
SCHEMA = "g1.run_record/1"

STATUS_INTERVAL_S = 0.5
FPS, WATCH_FPS = 10, 20
JPEG_QUALITY = 80
# Free camera that follows the target object, same view as the 54 video (distance m, azimuth and elevation in degrees)
CAMERA = {"distance": 1.1, "azimuth": 225.0, "elevation": -20.0}

# Phase keys come from the scripts (g1sim/grasp.py, German). Unknown keys are shown raw.
PHASE_LABELS = {
    "freimachen": "Clear the camera view",
    "anfahren": "Look, then move above the object",
    "absenken": "Lower beside the object",
    "heranfahren": "Look again, move to the grasp point",
    "daumen_drehen": "Turn thumb",
    "schliessen": "Close fingers",
    "anheben": "Lift",
    "halten": "Hold 5 s",
}
# Two-hand volleyball pick-and-place (g1sim/ball_grasp.py, scripts/67_volleyball_pick_place.py), English keys. A separate
# table so the bottle runs keep their own phase order.
PICK_PLACE_PHASE_LABELS = {
    "face_ball": "Turn the waist towards the ball",
    "reach": "Reach straight to beside the ball, hands opening",
    "touch": "Put both palms on the ball",
    "squeeze": "Squeeze with the palms, curl all five fingers",
    "lift": "Lift the ball",
    "turn": "Turn the waist to the basket",
    "release": "Open the hands over the basket",
    "retract": "Pull the hands back to the start pose",
    "face_front": "Turn the waist back to the front",
    "settle": "Hold still and let the ball settle",
}
# Walk up to the table (scripts/68_walk_to_table.py), English keys
WALK_PHASE_LABELS = {
    "stand_up": "Stand on the elastic band, then let go",
    "plan": "New table setup, plan a path (A*)",
    "walk": "Walk with the arms down, along the path to the checkpoint",
    "align": "Checkpoint: line up with the table, facing it",
    "raise_arms": "Stand still, raise the arms to the ready pose",
    "walk_in": "Walk straight into the table (no side steps)",
    "stand_still": "Stand still, measure; go again if more than 4 cm off",
}
PHASE_ORDERS = {"grasp_lift": list(PHASE_LABELS), "pick_place": list(PICK_PLACE_PHASE_LABELS),
                "walk_to_table": list(WALK_PHASE_LABELS)}

# Failure reason = first failed check in scripts/54_grasp_scripted.py evaluate() (German keys). Unknown keys are shown raw.
_REASONS = {  # key: (label, short form for tables)
    "flasche_in_reichweite": ("Start out of reach: the arm cannot get there (a start problem, not a grasp error)",
                              "out of reach"),
    "kopfkamera_findet_flasche": ("Camera position estimate more than 1.5 cm off", "camera estimate off"),
    "hand_an_greifpose": ("Hand missed the grasp pose", "missed grasp pose"),
    "kein_kontakt_tisch_tasse_wuerfel": ("Robot touched the table or another object", "touched table/objects"),
    "flasche_5_cm_angehoben_5_s_gehalten": ("Not lifted 5 cm, or dropped while holding", "not lifted / dropped"),
    "ablauf_ohne_fehler": ("Script error", "script error"),
    # scripts/67_volleyball_pick_place.py (g1sim/ball_grasp.py run())
    "no_forbidden_contacts": ("Robot touched the table, the stand or the basket", "touched table/stand/basket"),
    "ball_lifted": ("Ball not lifted 6 cm", "not lifted"),
    "ball_carried_to_basket": ("Ball dropped or sank while the waist turned", "dropped while turning"),
    "ball_in_basket": ("Ball not inside the basket", "missed basket"),
    "ball_on_basket_floor": ("Ball not lying on the basket floor", "not on basket floor"),
    "ball_at_rest": ("Ball still moving during the 3 s check", "ball moving"),
    "hands_off_ball": ("Robot still touching the ball during the 3 s check", "still touching ball"),
    "run_without_error": ("Script error", "script error"),
    # scripts/68_walk_to_table.py
    "no_fall": ("Robot fell", "fell"),
    "no_furniture_contact": ("Robot touched the table or an obstacle", "touched furniture"),
    "arrived_in_time": ("Did not arrive within the time limit", "timeout"),
    "dock_within_tolerance": ("Arrived, but more than 10 cm / 10 degrees off the grasp pose", "dock error"),
}
REASON_LABELS = {k: v[0] for k, v in _REASONS.items()}
REASON_SHORT = {k: v[1] for k, v in _REASONS.items()}
REACH_REASONS = ("flasche_in_reichweite",)  # a start problem: these trials do not count as reachable


def phase_label(phase):
    """Phase keys must be unique over the three tables (the first table that knows a key wins)."""
    return PHASE_LABELS.get(phase) or PICK_PLACE_PHASE_LABELS.get(phase) or WALK_PHASE_LABELS.get(phase, phase)


def reason_label(reason):
    return REASON_LABELS.get(reason, reason)


def now_iso():
    return datetime.datetime.now().astimezone().isoformat(timespec="seconds")


def git_state(repo=REPO):
    """{"commit": short hash, "dirty": bool} of the repo, or None if git is not available. Untracked files do not count."""
    try:
        commit = subprocess.run(["git", "-C", str(repo), "rev-parse", "--short", "HEAD"], capture_output=True, text=True,
                                timeout=5, check=True).stdout.strip()
        status = subprocess.run(["git", "-C", str(repo), "status", "--porcelain", "--untracked-files=no"],
                                capture_output=True, text=True, timeout=5, check=True).stdout
        return {"commit": commit, "dirty": bool(status.strip())}
    except (OSError, subprocess.SubprocessError):
        return None


def write_atomic(path, data):
    """Write bytes or text to path via a temp file + os.replace (readers see the old or the new file, never half)."""
    tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    tmp.write_bytes(data if isinstance(data, bytes) else data.encode())
    os.replace(tmp, path)


def encode_jpeg(rgb, quality=JPEG_QUALITY):
    """RGB uint8 image -> JPEG bytes (OpenCV if installed, else Pillow)."""
    try:
        import cv2
    except ImportError:
        import io

        from PIL import Image
        buf = io.BytesIO()
        Image.fromarray(rgb).save(buf, format="JPEG", quality=quality)
        return buf.getvalue()
    ok, jpeg = cv2.imencode(".jpg", rgb[..., ::-1], [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return jpeg.tobytes()


class Reporter:
    """Progress of one run as lab-view files. Call trial_start / on_tick / trial_end per trial, finish once at the end.

    renderer_getter: () -> mujoco.Renderer the run already owns (or None); with data (MjData) and track_body (body name)
    it renders the live image from a free camera that follows that body. sim_time_getter: () -> sim time of the trial (s).
    camera: {"distance", "azimuth", "elevation"} of that camera (default CAMERA, close up for grasping).
    frame_getter: () -> JPEG bytes of an image the run renders anyway (e.g. g1sim/liveview.py) - used instead of rendering
    a second time, which halved the control rate of the walking loop (scripts/68_walk_to_table.py).
    """

    def __init__(self, script, task, object, trials, argv, run_id=None, renderer_getter=None, sim_time_getter=None,
                 watch=None, root=ROOT, data=None, track_body=None, camera=None, frame_getter=None):
        started = datetime.datetime.now().astimezone()
        self.run_id = run_id or f"{started:%Y%m%d-%H%M%S}_{script}_{object}"
        self.root = pathlib.Path(root)
        self.live_dir, self.records_dir = self.root / "live", self.root / "records"
        self.status_path = self.live_dir / f"{self.run_id}.status.json"
        self.frame_path = self.live_dir / f"{self.run_id}.jpg"
        self.record_path = self.records_dir / f"{self.run_id}.json"
        self.renderer_getter, self.sim_time_getter, self.frame_getter = renderer_getter, sim_time_getter, frame_getter
        self.data, self.track_body = data, track_body
        self.watch = watch
        self.header = {
            "run_id": self.run_id, "script": script, "task": task, "object": object, "argv": list(argv),
            "trials_planned": trials, "started_at": started.isoformat(timespec="seconds"), "git": git_state(),
            "pid": os.getpid(), "watch": watch,
        }
        self.phase_order = list(PHASE_ORDERS.get(task, []))
        self.t0 = time.monotonic()
        self.trials, self.failures = [], collections.Counter()
        self.trial, self.phase, self.last_failure = None, None, None
        self.trial_wall0 = self.t0
        self.next_status = self.next_frame = 0.0
        self.camera, self.camera_setup = None, camera or CAMERA
        self.warned = set()
        for d in (self.live_dir, self.records_dir):
            d.mkdir(parents=True, exist_ok=True)
        self._write_status("running")

    # --- per trial --------------------------------------------------------------------------------------------------
    def trial_start(self, i, info=None):
        self.trial, self.phase, self.info = i, None, info
        self.trial_wall0 = time.monotonic()
        self._write_status("running")

    def on_tick(self, phase):
        """Call after every control tick with the current phase key."""
        now = time.monotonic()
        if phase != self.phase:
            self.phase = phase
            if phase not in self.phase_order:
                self.phase_order.append(phase)
            self._write_status("running")
        elif now >= self.next_status:
            self._write_status("running")
        if (self.renderer_getter is not None or self.frame_getter is not None) and now >= self.next_frame:
            self.next_frame = now + 1.0 / (WATCH_FPS if self.watch else FPS)
            self._write_frame()
        if self.watch:
            # Real-time pacing: wall time since the trial start >= sim time / watch
            wait = self._sim_time() / self.watch - (time.monotonic() - self.trial_wall0)
            if wait > 0:
                time.sleep(wait)

    def trial_end(self, passed, reason, details=None):
        """reason: first failed check (None if passed). details: optional {"reachable": bool, "error": message, ...}."""
        details = details or {}
        reachable = details.get("reachable", reason not in REACH_REASONS)
        row = {"i": self.trial, "passed": bool(passed), "reason": None if passed else reason, "reachable": bool(reachable),
               "sim_s": round(self._sim_time(), 3), "wall_s": round(time.monotonic() - self.trial_wall0, 3)}
        if details.get("error"):
            row["error"] = details["error"]
        self.trials.append(row)
        if not passed:
            self.failures[reason] += 1
            label = reason_label(reason) + (f": {details['error']}" if details.get("error") else "")
            self.last_failure = {"trial": self.trial, "reason": reason, "label": label}
        self._write_status("running")

    def finish(self, summary=None, report_path=None, video_path=None):
        """Write the run record and mark the run finished. Returns the record path (or None if writing failed).
        video_path: replay video of the run (mp4 inside the repo), shown with a play button in the lab view."""
        done = len(self.trials)
        passed = sum(t["passed"] for t in self.trials)
        reachable = sum(t["reachable"] for t in self.trials)
        report = None
        if report_path is not None:
            report_path = pathlib.Path(report_path).resolve()
            report = str(report_path.relative_to(REPO)) if report_path.is_relative_to(REPO) else str(report_path)
        video = None
        if video_path is not None and pathlib.Path(video_path).is_file():
            video = str(pathlib.Path(video_path).resolve().relative_to(REPO))
        record = {
            "schema": SCHEMA, **{k: v for k, v in self.header.items() if k not in ("pid", "watch")},
            "finished_at": now_iso(), "wall_s": round(time.monotonic() - self.t0, 2), "state": "finished",
            "watch": self.watch, "trials_done": done,
            "counts": {"passed": passed, "reachable": reachable, "attempted": done},
            "success_rate": passed / done if done else None,
            "reachable_success_rate": passed / reachable if reachable else None,
            "failures": dict(self.failures), "report": report, "summary": summary, "video": video,
            "trials": self.trials,
        }
        try:
            write_atomic(self.record_path, json.dumps(record, indent=2, ensure_ascii=False))
        except OSError as error:
            self._warn("record", f"run record not written: {error}")
            return None
        finally:
            self._write_status("finished")
        return self.record_path

    # --- files ------------------------------------------------------------------------------------------------------
    def _sim_time(self):
        if self.sim_time_getter is None:
            return 0.0
        return float(self.sim_time_getter())

    def _counts(self):
        return {"passed": sum(t["passed"] for t in self.trials), "done": len(self.trials),
                "reachable": sum(t["reachable"] for t in self.trials)}

    def _write_status(self, state):
        self.next_status = time.monotonic() + STATUS_INTERVAL_S
        order = self.phase_order
        status = {
            **self.header, "state": state, "trial": self.trial, "phase": self.phase,
            "phase_label": phase_label(self.phase) if self.phase else None,
            "phase_index": order.index(self.phase) if self.phase in order else None,
            "phase_order": [{"key": p, "label": phase_label(p)} for p in order],
            "sim_t": round(self._sim_time(), 2), "counts": self._counts(), "failures": dict(self.failures),
            "last_failure": self.last_failure, "has_frame": self.frame_path.exists(), "updated_at": time.time(),
        }
        try:
            write_atomic(self.status_path, json.dumps(status, ensure_ascii=False))
        except OSError as error:
            self._warn("status", f"status file not written: {error}")

    def _write_frame(self):
        if self.frame_getter is not None:
            try:
                jpeg = self.frame_getter()
                if jpeg:
                    write_atomic(self.frame_path, jpeg)
            except OSError as error:
                self._warn("frame", f"live image skipped: {error}")
            return
        try:
            renderer = self.renderer_getter()
            if renderer is None or self.data is None:
                return
            if self.camera is None:
                import mujoco
                self.camera = mujoco.MjvCamera()
                self.camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                self.camera.trackbodyid = renderer.model.body(self.track_body).id if self.track_body else 0
                setup = self.camera_setup
                self.camera.distance, self.camera.azimuth, self.camera.elevation = (
                    setup["distance"], setup["azimuth"], setup["elevation"])
            renderer.update_scene(self.data, camera=self.camera)
            write_atomic(self.frame_path, encode_jpeg(renderer.render()))
        except Exception as error:  # the live image is optional, the run must go on
            self._warn("frame", f"live image skipped: {type(error).__name__}: {error}")

    def _warn(self, kind, message):
        if kind not in self.warned:
            self.warned.add(kind)
            print(f"Lab view: {message} (reported once)")
