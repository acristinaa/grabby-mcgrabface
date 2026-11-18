"""Level 1 Step 4 - closed-loop eval of a policy in the walk world: event checks, reference replay, replay gate and metrics.

Pure code for scripts/82_vla_eval.py and its tests. The policy is reached through a client object that 82 passes in
(g1sim/policy_wire.py's PolicyClient, or a test double with reset(episode, seed) and infer(obs, tick)); this module never imports
policy_wire, so its unit tests run without it. MuJoCo, table_env and ball_grasp are imported only inside the functions that take
an env: the unit tests run in the project .venv, which has neither mink nor huggingface_hub.

Tick convention: tick = number of actions applied since the handover. The request at tick t sends the observation before action
t; event ticks and the latch tick count the actions applied when the event is seen (1-based). Requests at 0, exec, 2 exec, ...
Events, checked after every env.apply (rest = ball position at the handover, as the teacher's rest):
  lifted   ball centre at least MIN_LIFT above rest
  carried  lifted and the ball centre over the basket's outer footprint (FOOTPRINT_HALF per axis)
  placing  all 4 ball_grasp.placement_checks true on this tick; a false tick resets the streak
  latched  the streak reaches HOLD_TICKS (3 s) with no forbidden contact before: success, the trial stops
A robot contact with a ball_grasp.FORBIDDEN body ends the trial (no_forbidden_contacts). At the timeout the reason is the first
failed check in the teacher's order (ball_grasp.py run()): ball_lifted, ball_carried_to_basket, the first false placement check
of the last tick, placement_not_held_3s. The eval's ball_carried_to_basket adds the footprint condition to the teacher's height.
Two diagnostics, not gates: stalled (a failed trial with a 2 s window of nearly constant actions) and no_progress (failed by
timeout, no new event and the ball still over the last 5 s).
"""
import collections
import math
import pathlib
import time

import numpy as np

from g1sim import action_space, episode

SCENE = "volleyball_walk"
HZ = 30  # table_env.CONTROL_HZ
MAX_EXEC_STEPS = 50  # chunk size of the wire contract (policy_wire.CHUNK)
STALL_WINDOW = 60  # ticks (2 s)
STALL_RANGE = 0.01  # smallest 60-tick range in 43 teacher demos: 0.038 (probe_window_range); re-calibrate on the first run1 eval
PROGRESS_WINDOW = 150  # ticks (5 s)
PROGRESS_MIN_M = 0.02
HOLD_TICKS = 90  # 3 s at 30 Hz
HANDS = slice(14, 26)
EVENTS = ("handover", "lifted", "carried", "placing", "latched")
TRAIN_LOW = 0.6  # plan E verdict (R3)
GAP = 0.2
# Observation checks of the replay gate (plan2_obs_probe: tick-0 image max grey diff 0, later request ticks MAD 3.03-3.39)
OBS_IMAGE_MAXDIFF_TICK0 = 2
OBS_IMAGE_MAD_MAX = 4.0
# Copies of sim constants so this module imports without MuJoCo; tests/test_integration_vla_eval.py pins them equal.
MIN_LIFT = 0.06  # ball_grasp.MIN_LIFT
FOOTPRINT_HALF = 0.14 + 0.005  # table_scene.BASKET_INNER_HALF + table_scene.BASKET_WALL (same float, 0.145 m)
PLACEMENT_CHECKS = ("ball_in_basket", "ball_on_basket_floor", "ball_at_rest", "hands_off_ball")  # ball_grasp.placement_checks
OBS_FIELDS = ("obs_state_err_max", "obs_image_maxdiff_tick0", "obs_image_mad_max", "obs_prompt_ok_all", "obs_checked")
FENCE_STREAK_EXTRA_ACTIONS = 15  # 0.5 s at 30 Hz: a fence streak of exec_steps + 15 refused actions ends the trial (86's
                                 # one chunk + 0.5 s, FENCE-SMOOTH.md 1.6; ruling pending (Finnegan) R-F2)
SMOOTH_MOVING_RAD_S = 0.05      # a tick counts as moving when some arm/waist joint's commanded speed is above this (2.1)
SMOOTH_JOINTS = np.r_[0:14, 26]  # arms and waist yaw in the 27-D action


class ContractError(ValueError):
    """The policy's reply breaks the wire contract (shape); 82 aborts the run."""


def wilson(k, n, z=1.959964):
    """Wilson score interval (low, high) of k successes in n trials; (None, None) for n = 0."""
    if n == 0:
        return None, None
    p, z2 = k / n, z * z
    denom = 1.0 + z2 / n
    centre = (p + z2 / (2 * n)) / denom
    half = z * math.sqrt(p * (1.0 - p) / n + z2 / (4.0 * n * n)) / denom
    return max(0.0, centre - half), min(1.0, centre + half)


class EventTracker:
    """Events of one trial from per-tick measurements (module docstring). update() once per tick until done; then passed,
    reason, furthest_event, ticks (event -> tick or None), forbidden (pair or None) and no_progress hold the outcome."""

    def __init__(self, rest_xyz, hz=HZ, timeout_ticks=45 * HZ):
        self.rest = np.array(rest_xyz, dtype=float)
        self.hz, self.timeout_ticks = hz, int(timeout_ticks)
        self.ticks = dict.fromkeys(EVENTS)
        self.ticks["handover"] = 0
        self.tick, self.streak = 0, 0
        self.placement, self.forbidden = None, None
        self.done = self.passed = self.no_progress = False
        self.reason = None
        self.track = collections.deque([self.rest.copy()], maxlen=PROGRESS_WINDOW + 1)  # ball positions, ticks t-150..t

    def update(self, tick, ball_xyz, basket_xyz, placement, forbidden_pair=None):
        """Measurements after action `tick` (1-based). placement: {check: bool} of ball_grasp.placement_checks. Returns done."""
        if self.done:
            raise RuntimeError("update() after the trial ended")
        ball = np.array(ball_xyz, dtype=float)
        self.tick = tick
        self.track.append(ball)
        if ball[2] - self.rest[2] >= MIN_LIFT:
            self._event("lifted", tick)
            if np.all(np.abs(ball[:2] - np.asarray(basket_xyz, dtype=float)[:2]) <= FOOTPRINT_HALF):
                self._event("carried", tick)
        self.placement = dict(placement)
        placing = all(self.placement[check] for check in PLACEMENT_CHECKS)
        self.streak = self.streak + 1 if placing else 0
        if placing:
            self._event("placing", tick)
        if forbidden_pair is not None:
            self.forbidden = tuple(forbidden_pair)
            self.end("no_forbidden_contacts")
        elif self.streak >= HOLD_TICKS and self.ticks["lifted"] is not None:  # a ball pushed in without a lift is no success
            self.ticks["latched"] = tick
            self.done, self.passed = True, True
        elif tick >= self.timeout_ticks:
            self.end()
        return self.done

    def end(self, reason=None):
        """End the trial now as failed: with reason (e.g. policy_output_invalid), or None = by the timeout order (also used
        when the teacher's program ends without a latch)."""
        self.done, self.passed = True, False
        self.reason = reason or self._timeout_reason()
        self.no_progress = reason is None and self._no_progress()

    @property
    def phase(self):
        """Lab view phase key now."""
        if self.streak > 0:
            return "vla_placing"
        if self.ticks["carried"] is not None:
            return "vla_carried"
        if self.ticks["lifted"] is not None:
            return "vla_lifted"
        return "vla_start"

    @property
    def furthest_event(self):
        return max((e for e in EVENTS if self.ticks[e] is not None), key=EVENTS.index)

    def result(self):
        """Row fields of the trial."""
        latch = self.ticks["latched"]
        return {"passed": self.passed, "reason": self.reason, "furthest_event": self.furthest_event,
                "events": dict(self.ticks),
                "events_s": {e: None if t is None else round(t / self.hz, 3) for e, t in self.ticks.items()},
                "latch_tick": latch, "time_to_latch_s": None if latch is None else round(latch / self.hz, 3),
                "forbidden_pair": None if self.forbidden is None else list(self.forbidden), "no_progress": self.no_progress,
                "ticks": self.tick, "sim_s": round(self.tick / self.hz, 3)}

    def _event(self, name, tick):
        if self.ticks[name] is None:
            self.ticks[name] = tick

    def _timeout_reason(self):
        if self.ticks["lifted"] is None:
            return "ball_lifted"
        if self.ticks["carried"] is None:
            return "ball_carried_to_basket"
        for check in PLACEMENT_CHECKS:
            if self.placement is not None and not self.placement[check]:
                return check
        return "placement_not_held_3s"

    def _no_progress(self):
        """No new event and the ball moved less than PROGRESS_MIN_M over the last PROGRESS_WINDOW ticks."""
        if len(self.track) <= PROGRESS_WINDOW:
            return False
        if max(t for t in self.ticks.values() if t is not None) > self.tick - PROGRESS_WINDOW:
            return False
        moved = np.linalg.norm(np.asarray(self.track) - self.track[0], axis=1).max()
        return bool(moved < PROGRESS_MIN_M)


def measure(env):
    """EventTracker.update keyword arguments from the sim now."""
    from g1sim import ball_grasp
    forbidden = next(((robot, other) for robot, other in env.robot_contacts() if other in ball_grasp.FORBIDDEN), None)
    return {"ball_xyz": env.object_position(ball_grasp.BALL), "basket_xyz": env.data.xpos[env.model.body("basket").id].copy(),
            "placement": ball_grasp.placement_checks(env), "forbidden_pair": forbidden}


def stalled(actions, passed):
    """(stalled, start tick): a failed trial is stalled if some STALL_WINDOW-tick window of its executed actions has a range
    (max over the 27 dims of max - min) below STALL_RANGE. A success is never stalled (the replay holds its last action for
    fewer than STALL_WINDOW ticks before the latch, a policy may hold longer)."""
    actions = np.asarray(actions, dtype=float)
    if passed or len(actions) < STALL_WINDOW:
        return False, None
    windows = np.lib.stride_tricks.sliding_window_view(actions, STALL_WINDOW, axis=0)  # (n - 59, 27, 60)
    spread = np.ptp(windows, axis=-1).max(axis=-1)
    low = np.flatnonzero(spread < STALL_RANGE)
    return (True, int(low[0])) if len(low) else (False, None)


def check_exec_steps(exec_steps):
    if isinstance(exec_steps, bool) or not isinstance(exec_steps, int) or not 1 <= exec_steps <= MAX_EXEC_STEPS:
        raise ValueError(f"exec_steps {exec_steps!r}: must be an int in 1..{MAX_EXEC_STEPS}")


def request_ticks(total, exec_steps):
    """Ticks at which the client asks for a chunk within `total` ticks."""
    check_exec_steps(exec_steps)
    return list(range(0, total, exec_steps))


def clip_hands(chunk):
    """Hand channels 14..25 clipped to [0, 1] (R5); arms and waist unchanged. Returns (copy, number of clipped values)."""
    chunk = np.array(chunk, copy=True)
    hands = chunk[..., HANDS]
    count = int(np.count_nonzero((hands < 0.0) | (hands > 1.0)))
    chunk[..., HANDS] = np.clip(hands, 0.0, 1.0)
    return chunk, count


def pick_evenly(paths, n):
    """n items evenly spaced over the list (first and last included); all of them if n >= len."""
    paths = list(paths)
    if n >= len(paths):
        return paths
    if n <= 0:
        return []
    return [paths[i] for i in sorted({int(round(x)) for x in np.linspace(0, len(paths) - 1, n)})]


def list_demos(path):
    """An npz file -> [it]; a folder -> its *.npz sorted, without macOS ._* files and *.part temp files."""
    path = pathlib.Path(path)
    if path.is_file():
        return [path]
    if not path.is_dir():
        raise FileNotFoundError(f"{path}: no such file or folder")
    return sorted(p for p in path.iterdir()
                  if p.suffix == ".npz" and not p.name.startswith("._") and not p.name.endswith(".part"))


def no_walk_handovers(env, n):
    """69's handover, step for step (69:214, :127, :130): per trial one env.reset(randomize=True), the snapshot, then the first
    observation. Yields {"trial", "snapshot", "obs", "error"}; a reset that draws no valid start yields error = its message and
    snapshot = obs = None (reason handover_failed). Trial k of a seed is the k-th draw of that seed."""
    for trial in range(n):
        try:
            env.reset(randomize=True)
        except RuntimeError as error:
            yield {"trial": trial, "snapshot": None, "obs": None, "error": str(error)}
            continue
        snapshot = env.snapshot_handover()
        yield {"trial": trial, "snapshot": snapshot, "obs": env.observation(), "error": None}


def restore(env, ep):
    """Restore a recorded episode's handover into env. Returns the first observation (the recorded frame 0's moment)."""
    env.restore_handover(episode.handover_snapshot(ep))
    return env.observation()


def _stats(values):
    if not values:
        return None
    v = np.asarray(values, dtype=float)
    mean, p95 = float(v.mean()), float(np.percentile(v, 95))
    return {"mean": round(mean, 1), "p95": round(p95, 1), "max": round(float(v.max()), 1),
            "mean_ticks": round(mean * HZ / 1000, 2), "p95_ticks": round(p95 * HZ / 1000, 2)}


def _obs_fields(checks):
    """Aggregate the replay server's per-request checks; all None against a server that sends none (SmolVLA)."""
    with_prompt = [c for c in checks if "obs_prompt_ok" in c]
    if not with_prompt:
        return dict.fromkeys(OBS_FIELDS)
    recorded = [c for c in checks if c.get("recorded_tick") is not None]
    tick0 = [int(c["obs_image_maxdiff"]) for c in recorded if c["recorded_tick"] == 0]
    return {"obs_state_err_max": max((float(c["obs_state_err"]) for c in recorded), default=None),
            "obs_image_maxdiff_tick0": tick0[0] if tick0 else None,
            "obs_image_mad_max": max((float(c["obs_image_mad"]) for c in recorded), default=None),
            "obs_prompt_ok_all": all(bool(c["obs_prompt_ok"]) for c in with_prompt), "obs_checked": len(recorded)}


def _row(tracker, executed, wall0, **fields):
    stall, stall_start = stalled(executed, tracker.passed)
    return {**tracker.result(), "stalled": stall, "stall_start": stall_start,
            "wall_s": round(time.perf_counter() - wall0, 2), **fields}


class FenceGuard:
    """82 --fence: 86's virtual fence on the sim's own scene (g1sim/fence.py, FENCE-SMOOTH.md 1.6). Before each env.apply the
    action is checked like 86 does (cmd and pred = cmd + measured - last applied, hands at their commanded target); a
    refused action applies the last applied action again; exec_steps + FENCE_STREAK_EXTRA_ACTIONS refused actions in a row
    end the trial as fence_exit (a failure, not a forbidden contact)."""

    def __init__(self, env, exec_steps):
        from g1sim import fence
        self.env, self.fence = env, fence.Fence.from_env(env)
        self.streak_limit = exec_steps + FENCE_STREAK_EXTRA_ACTIONS
        self.last = _env_command(env)
        self.holds = self.streak = self.alternations = 0
        self.first_tick, self.exit, self.held_before = None, False, False
        self.smallest = {}

    def filter(self, action, tick):
        """The action to apply for `action` at `tick` (0-based): itself, or the last applied one when the fence holds."""
        action = np.asarray(action, dtype=float)
        measured = self.env.data.qpos[self.fence.qadr17]
        result = self.fence.check(_q17(action, measured), action[HANDS], measured, _q17(self.last, measured),
                                  self.last[HANDS])
        for group, value in result.values.items():
            self.smallest[group] = min(self.smallest.get(group, np.inf), value)
        self.alternations += result.hold != self.held_before and tick > 0
        self.held_before = result.hold
        if not result.hold:
            self.streak, self.last = 0, action
            return action
        self.holds += 1
        self.streak += 1
        self.first_tick = tick if self.first_tick is None else self.first_tick
        self.exit = self.exit or self.streak >= self.streak_limit
        return self.last

    def row(self):
        return {"fence_holds": self.holds, "fence_first_tick": self.first_tick, "fence_exit": self.exit,
                "fence_alternations": self.alternations,
                "fence_worst_cm": {g: round(float(v) * 100, 2) for g, v in sorted(self.smallest.items())}}


def _q17(action27, measured17):
    """27-D action -> motors 12..28 (waist roll/pitch: the measured ones; the policy never commands them)."""
    return np.concatenate([[action27[26]], measured17[1:3], action27[0:14]])


def _env_command(env):
    """The command the env applies now, as a 27-D action (the start of a trial: the handover's targets)."""
    action = np.zeros(action_space.DIM)
    for side in action_space.SIDES:
        action[action_space.SLICES[f"{side}_arm"]] = env.body_target[env.arm_slot[side]]
        action[action_space.SLICES[f"{side}_hand"]] = env.hand_target[side]
    action[26] = env.body_target[12]
    return action


def smoothness(applied):
    """p99 |acceleration| (rad/s^2) of the applied arm/waist commands over moving ticks, worst joint, at 30 Hz - the 82
    counterpart of FENCE-SMOOTH.md 2.1 (86 measures at 50 Hz after its scheduler). None for fewer than 3 actions."""
    q = np.asarray(applied, dtype=float)[:, SMOOTH_JOINTS] if len(applied) >= 3 else None
    if q is None:
        return None
    v = np.diff(q, axis=0) * HZ
    a = np.abs(np.diff(v, axis=0)) * HZ
    moving = np.abs(v[1:]).max(axis=1) > SMOOTH_MOVING_RAD_S
    return round(float(np.percentile(a[moving], 99, axis=0).max()), 2) if moving.any() else 0.0


def run_policy_trial(env, client, prompt, tracker, exec_steps, episode_stem, seed, recorded_states=None, on_tick=None,
                     first_obs=None, fence=False, blend=0):
    """One trial from the handover env is at: a fresh env.observation() at every request tick (first_obs at tick 0 if given:
    the observation the handover already rendered - a second render would move the camera noise), chunk[:exec_steps] executed
    with one env.apply each, hands clipped. recorded_states: the episode's (T, 27) states, for replay_state_error.
    on_tick(phase) after every tick. fence: 86's virtual fence before every apply (FenceGuard). blend: for this many actions
    after a new chunk, fade linearly from the old chunk (86 `run`, real_timing.BLEND_ACTIONS). Returns the row; raises
    ContractError on a wrong chunk shape."""
    check_exec_steps(exec_steps)
    wall0 = time.perf_counter()
    client.reset(episode=episode_stem, seed=seed)
    guard = FenceGuard(env, exec_steps) if fence else None
    executed, applied, server_ms, round_trip_ms, checks = [], [], [], [], []
    clips, state_err = 0, None if recorded_states is None else 0.0
    chunk, start, tick = None, 0, 0
    old, old_start = None, 0
    while not tracker.done:
        if recorded_states is not None and tick < len(recorded_states):
            state_err = max(state_err, float(np.abs(env.state().astype(np.float32) - recorded_states[tick]).max()))
        if tick % exec_steps == 0:
            obs = first_obs if tick == 0 and first_obs is not None else env.observation()
            reply = client.infer(episode.policy_observation(obs, prompt), tick)
            raw = np.asarray(reply["actions"])
            if raw.ndim != 2 or raw.shape[0] < exec_steps or raw.shape[1] != action_space.DIM:
                raise ContractError(f"tick {tick}: actions shape {raw.shape}, expected (>= {exec_steps}, {action_space.DIM})")
            server_ms.append(float(reply["server_ms"]))
            round_trip_ms.append(float(reply["round_trip_ms"]))
            checks.append(reply.get("checks") or {})
            if not np.all(np.isfinite(raw)):
                tracker.end("policy_output_invalid")
                break
            old, old_start = chunk, start
            chunk, clipped = clip_hands(raw)
            clips += clipped
            start = tick
        action = chunk[tick - start]
        if blend and old is not None and tick - start < blend and tick - old_start < len(old):
            w = (tick - start) / blend
            action = (1 - w) * old[tick - old_start] + w * action
        sent = guard.filter(action, tick) if guard else action
        env.apply(sent)
        executed.append(action)
        applied.append(sent)
        tick += 1
        tracker.update(tick, **measure(env))
        if guard and guard.exit and not tracker.done:
            tracker.end("fence_exit")
        if on_tick:
            on_tick(tracker.phase)
    server, rtt = _stats(server_ms), _stats(round_trip_ms)
    return _row(tracker, executed, wall0, reached_handover=True, exec_steps=exec_steps, seed=seed, requests=len(server_ms),
                server_ms=server and {k: server[k] for k in ("mean", "max")},
                round_trip_ms=rtt and {k: rtt[k] for k in ("mean", "max")},
                latency_ms_all={"server": [round(v, 1) for v in server_ms], "round_trip": [round(v, 1) for v in round_trip_ms]},
                hand_clips=clips, replay_state_error=state_err, blend=blend, smooth_p99_acc=smoothness(applied),
                **(guard.row() if guard else {}), **_obs_fields(checks))


def run_teacher_trial(env, tracker, on_tick=None, fence=False, exec_steps=25):
    """The sim teacher (ball_grasp.TwoHandBallGrasp) from the handover env is at, measured by the same tracker. Returns the row
    with the teacher's own verdict next to the event verdict ("agree"). fence: every env.apply of the teacher goes through
    86's fence (FenceGuard; E1: the teacher must never be refused). The teacher module itself is not changed: env.apply is
    wrapped on this env instance for the trial."""
    import mink

    from g1sim import ball_grasp
    wall0 = time.perf_counter()
    executed = []
    guard = FenceGuard(env, exec_steps) if fence else None

    def tick(phase):
        if tracker.done:
            return
        executed.append(grasp.action.copy())
        tracker.update(len(executed), **measure(env))
        if guard and guard.exit and not tracker.done:
            tracker.end("fence_exit")
        if on_tick:
            on_tick(tracker.phase)

    if guard:
        apply = env.apply
        env.apply = lambda action: apply(guard.filter(action, len(executed)))
    grasp = ball_grasp.TwoHandBallGrasp(env, on_tick=tick)
    try:
        result = grasp.run()
    except (RuntimeError, mink.exceptions.NoSolutionFound) as error:
        result = {"passed": False, "first_failed": "run_without_error", "error": f"{type(error).__name__}: {error}"}
    finally:
        if guard:
            del env.apply             # the class method again
    if not tracker.done:  # the program ended before a latch, a forbidden contact or the timeout
        tracker.end()
    teacher = {"passed": bool(result["passed"]), "first_failed": result["first_failed"], "checks": result.get("checks"),
               "error": result.get("error")}
    return _row(tracker, executed, wall0, reached_handover=True, requests=0, hand_clips=0, teacher=teacher,
                agree=teacher["passed"] == tracker.passed, **(guard.row() if guard else {}))


def reference_replay(ep, timeout_ticks):
    """Reference for the replay gate, independent of the wire and of chunking: ep's handover restored in a fresh TableEnv,
    actions[min(t, T - 1)] applied per tick through the same EventTracker. Returns events, passed, reason, latch_tick, frames."""
    from g1sim import ball_grasp, randomization, table_env
    env = table_env.TableEnv(scene=SCENE, variation_config=randomization.load())
    try:
        env.restore_handover(episode.handover_snapshot(ep))
        tracker = EventTracker(env.object_position(ball_grasp.BALL), timeout_ticks=timeout_ticks)
        actions = ep["actions"]
        tick = 0
        while not tracker.done:
            env.apply(actions[min(tick, len(actions) - 1)])
            tick += 1
            tracker.update(tick, **measure(env))
    finally:
        env.close()
    return {"events": dict(tracker.ticks), "passed": tracker.passed, "reason": tracker.reason,
            "latch_tick": tracker.ticks["latched"], "frames": len(actions)}


def replay_gate_problems(row, reference):
    """The Step 4 done criterion for one replayed trial against its reference_replay. Returns problems, each starting with the
    row field it is about; [] = pass."""
    problems = []

    def need(ok, field, detail):
        if not ok:
            problems.append(f"{field}: {detail}")

    need(row["passed"], "passed", f"not latched ({row['reason']})")
    need(row["latch_tick"] == reference["latch_tick"], "latch_tick", f"{row['latch_tick']}, reference {reference['latch_tick']}")
    for event in EVENTS[1:4]:
        need(row["events"][event] == reference["events"][event], f"events.{event}",
             f"{row['events'][event]}, reference {reference['events'][event]}")
    need(row["replay_state_error"] == 0.0, "replay_state_error", f"{row['replay_state_error']}, must be 0.0")
    need(row["obs_state_err_max"] == 0.0, "obs_state_err_max", f"{row['obs_state_err_max']}, must be 0.0")
    value = row["obs_image_maxdiff_tick0"]
    need(value is not None and value <= OBS_IMAGE_MAXDIFF_TICK0, "obs_image_maxdiff_tick0",
         f"{value}, must be <= {OBS_IMAGE_MAXDIFF_TICK0}")
    value = row["obs_image_mad_max"]
    need(value is not None and value <= OBS_IMAGE_MAD_MAX, "obs_image_mad_max", f"{value}, must be <= {OBS_IMAGE_MAD_MAX}")
    need(row["obs_prompt_ok_all"] is True, "obs_prompt_ok_all", f"{row['obs_prompt_ok_all']}")
    latch, exec_steps = row["latch_tick"], row["exec_steps"]
    if latch is not None:
        checked = len(range(0, min(latch, reference["frames"]), exec_steps))  # requests at ticks with a recorded frame
        need(row["obs_checked"] == checked, "obs_checked", f"{row['obs_checked']}, expected {checked}")
        need(row["requests"] == math.ceil(latch / exec_steps), "requests",
             f"{row['requests']}, expected ceil({latch} / {exec_steps}) = {math.ceil(latch / exec_steps)}")
    need(not row["stalled"], "stalled", f"from tick {row['stall_start']}")
    return problems


def _rate(k, n):
    low, high = wilson(k, n)
    return {"k": k, "n": n, "rate": k / n if n else None, "ci95": [low, high]}


def summarize(rows):
    """Section summary: success rates with Wilson 95 % intervals (vla_success over trials that reached the handover,
    end_to_end over all), failures by reason, furthest events, diagnostics, latency, time to latch, hand clips; replay_gate
    and teacher_agreement when the rows have them."""
    reached = [r for r in rows if r.get("reached_handover")]
    passes = sum(bool(r["passed"]) for r in rows)
    latched = [r["time_to_latch_s"] for r in rows if r["passed"]]
    summary = {
        "trials": len(rows), "reached_handover": len(reached), "passed": passes,
        "vla_success": _rate(passes, len(reached)), "end_to_end": _rate(passes, len(rows)),
        "failures": dict(collections.Counter(r["reason"] for r in rows if not r["passed"])),
        "furthest_event": {e: sum(r.get("furthest_event") == e for r in rows) for e in EVENTS},
        "stalled": sum(bool(r.get("stalled")) for r in rows), "no_progress": sum(bool(r.get("no_progress")) for r in rows),
        "latency_ms": {kind: _stats([v for r in rows for v in r.get("latency_ms_all", {}).get(kind, [])])
                       for kind in ("server", "round_trip")},
        "time_to_latch_s_mean": round(float(np.mean(latched)), 3) if latched else None,
        "hand_clips": sum(r.get("hand_clips", 0) for r in rows),
    }
    fenced = [r for r in rows if "fence_holds" in r]
    if fenced:
        smallest = {}
        for r in fenced:
            for group, value in r["fence_worst_cm"].items():
                smallest[group] = min(smallest.get(group, value), value)
        summary["fence"] = {"trials": len(fenced), "holds": sum(r["fence_holds"] for r in fenced),
                            "trials_with_holds": sum(r["fence_holds"] > 0 for r in fenced),
                            "fence_exits": sum(bool(r["fence_exit"]) for r in fenced),
                            "alternations": sum(r["fence_alternations"] for r in fenced), "smallest_cm": smallest}
    gated = [r for r in rows if "gate_problems" in r]
    if gated:
        summary["replay_gate"] = {"passed": sum(not r["gate_problems"] for r in gated), "trials": len(gated),
                                  "problems": [f"{r['id']}: {p}" for r in gated for p in r["gate_problems"]]}
    taught = [r for r in rows if "teacher" in r]
    if taught:
        summary["teacher_agreement"] = {"agree": sum(r["agree"] for r in taught), "trials": len(taught),
                                        "teacher_passed": sum(r["teacher"]["passed"] for r in taught)}
    return summary


def verdict(train_summary, heldout_summary):
    """Plan E verdict (R3) from the vla_success rates of the training-variation and the held-out section. Rounded comparisons,
    so 0.8 vs 0.6 is "consistent", not a float-error "gap"."""
    train, heldout = train_summary["vla_success"], heldout_summary["vla_success"]
    out = {"train": train["rate"], "train_ci95": train["ci95"], "heldout": heldout["rate"], "heldout_ci95": heldout["ci95"]}
    if train["rate"] is None or heldout["rate"] is None:
        out["verdict"] = "no verdict: a section has no trial that reached the handover"
    elif round(train["rate"], 6) < TRAIN_LOW:
        out["verdict"] = "undertrained: resume training, do not add demos"
    elif round(train["rate"] - heldout["rate"], 6) > GAP:
        out["verdict"] = "generalization gap: more demos or narrower ranges"
    else:
        out["verdict"] = "consistent"
    return out
