"""Real G1: the policy phase of 86 (replay, run, shadow) - observation, chunk requests, chunk playback, episode frames.

Why it exists: the policy answers late and at its own pace (30 Hz chunks over the network) while the arms need a command
every 20 ms. This loop never waits for the server: requests run in a worker thread, the ChunkScheduler plays whatever chunk
is current, and every input that goes stale ends the run through the runner's exits (PLAN.md 4.2 rules 10, 11, 14).
In `shadow` the policy's actions are filtered and logged but never sent; the arms hold the ready pose (PLAN.md 7 M5).
"""
import threading
from typing import TYPE_CHECKING

import numpy as np

from g1sim import action_space, policy_wire, real_io, real_safety as rs
from g1sim.real_run_exits import ExitRequest

if TYPE_CHECKING:                                   # for the type hints only: real_run imports this module
    from g1sim.real_run import Runner

SHADOW_SPEED = 1.0              # shadow plays the policy in real time: nothing moves, so no slow factor is needed
HAND_SLOTS = slice(14, 26)      # both hands in the 27-D state (action_space.SLICES)
NO_ACTION = np.full(action_space.DIM, np.nan, dtype=np.float32)   # policy_actions row before the first chunk


def reset_policy(runner: "Runner") -> None:
    """Tell the server which demo to replay (or reset the checkpoint's noise seed). Done BEFORE the ramp-in: this blocking
    round trip in the middle of the run would leave the arm without a message (85's gate: < 60 ms between messages)."""
    episode = runner.args.episode if runner.mode == "replay" else None
    try:
        runner.io.policy.reset(episode=episode, seed=policy_wire.noise_seed(episode) if episode else 0)
    except (OSError, policy_wire.PolicyServerError) as error:
        runner.say(f"policy server problem at reset: {error}")
        raise ExitRequest("soft", "policy_error") from None


class ThreadedWorker:
    """Runs one policy request at a time in a background thread, so the 50 Hz loop never blocks on the network."""

    def __init__(self):
        self._lock = threading.Lock()
        self._result = None

    def submit(self, job) -> None:
        """Start job() in a daemon thread; its result (or error) is picked up by poll()."""
        threading.Thread(target=self._run, args=(job,), daemon=True).start()

    def _run(self, job) -> None:
        try:
            result = ("ok", job())
        except Exception as error:  # socket timeout, server error, broken connection: all end the run softly
            result = ("error", f"{type(error).__name__}: {error}")
        with self._lock:
            self._result = result

    def poll(self):
        """None while the request runs, else ("ok", reply) or ("error", message), once."""
        with self._lock:
            result, self._result = self._result, None
        return result


class PolicyPhase:
    """One policy phase. `runner` is the g1sim.real_run.Runner that owns the sending, the filter and the log."""

    def __init__(self, runner: "Runner", send_policy: bool):
        self.r, self.send_policy = runner, send_policy
        self.phase = "policy" if send_policy else "shadow"
        self.scheduler = rs.ChunkScheduler(runner.args.speed if send_policy else SHADOW_SPEED)
        self.worker = runner.io.make_worker()
        self.requested_at, self.pending_since = None, None
        self.frames_logged = 0
        self.hold17 = runner.command.copy()      # held before the first chunk and after one ran out
        self.shadow_filter = None if send_policy else runner.new_filter(runner.q0)
        self.shadow_ticks, self.shadow_clipped = 0, 0

    def run(self) -> None:
        """Play the policy until max_seconds (raises ExitRequest('soft', 'done')) or until something stops it."""
        r = self.r
        if self.send_policy:
            r.filter.reset_last(r.command)
        start = r.io.clock()
        try:
            while r.io.clock() - start < r.args.max_seconds:
                self._step()
        finally:
            self._print_shadow_summary()
        raise ExitRequest("soft", "done")

    def _step(self) -> None:
        r = self.r
        now = r.io.clock()
        obs, ages = self._observe()
        self._collect_chunk(now)
        reason = rs.stale_reason(*ages, self.scheduler.seconds_since_ran_out(), r.mode)
        if reason:
            raise ExitRequest(rs.exit_kind(reason), reason)
        self._request_if_due(obs, now)
        action = self.scheduler.target()
        target17 = self._target(action)
        r.tick(self.phase, 1.0, target17)
        self._log_frame(obs, action)
        self.scheduler.advance(1.0 / rs.SEND_HZ)

    def _target(self, action) -> np.ndarray:
        """The 17-vector to send this tick, and the hand command (run/replay); shadow always holds the ready pose."""
        r = self.r
        if action is None:
            return self.hold17
        if not self.send_policy:
            self._shadow_evaluate(action)
            return self.hold17
        left, right = rs.hand_targets(action)
        r.command_hands(left, right, rs.HAND_SPEED)
        self.hold17 = rs.sdk_targets(action, r.q0)
        return self.hold17

    def _observe(self) -> tuple:
        """(observation for the server, (lowstate_age, camera_age, hand_age) for stale_reason)."""
        r = self.r
        lowstate, lowstate_age = r.io.lowstate.latest()
        if r.io.frames is None:  # replay --no-camera: 81's replay ignores the image
            frame, camera_age = np.zeros(policy_wire.IMAGE_SHAPE, dtype=np.uint8), None
        else:
            frame, camera_age = r.io.frames.latest()
            if frame is None:
                frame = np.zeros(policy_wire.IMAGE_SHAPE, dtype=np.uint8)
        if r.mode == "replay" and camera_age is not None and camera_age > rs.STALE_S["camera"]:
            r.note_event("camera_stale_warning")
        hand_msgs, hand_age = self._hand_states()
        state = real_io.state27(lowstate, *hand_msgs)   # lowstate is never None here: preflight saw a message
        if np.isnan(state[HAND_SLOTS]).any():
            state[HAND_SLOTS] = np.concatenate(r.hand_command())   # m6-6: check_observation refuses NaN
            r.note_event("hand_state_synthetic")
        obs = {policy_wire.IMAGE_KEY: frame, policy_wire.STATE_KEY: state, "prompt": policy_wire.TASK}
        return obs, (lowstate_age, camera_age, hand_age)

    def _hand_states(self) -> tuple:
        """((left msg | None, right msg | None), oldest age or None with --no-hands)."""
        readers = self.r.io.hands
        if not readers:
            return (None, None), None
        latest = [readers[side].latest() for side in action_space.SIDES]
        return tuple(msg for msg, _ in latest), max(age for _, age in latest)

    def _collect_chunk(self, now: float) -> None:
        result = self.worker.poll()
        if result is None:
            if self.pending_since is not None and now - self.pending_since > rs.POLICY_TIMEOUT_S:
                raise ExitRequest("soft", "policy_stale")
            return
        self.pending_since = None
        status, reply = result
        if status == "error":
            self.r.say(f"policy server problem: {reply}")
            raise ExitRequest("soft", "policy_error")
        try:
            self.scheduler.add_chunk(reply["actions"], self.requested_at)
        except rs.InvalidTarget:
            raise ExitRequest("fault", "invalid_target") from None
        except rs.ChunkTooLate as error:
            self.r.say(str(error))
            raise ExitRequest("soft", "chunk_too_late") from None
        except ValueError as error:  # after InvalidTarget, which is a ValueError too: a NaN target stays a fault
            self.r.say(f"policy server problem: {error}")
            raise ExitRequest("soft", "policy_error") from None   # a chunk of the wrong shape (rule 10)

    def _request_if_due(self, obs: dict, now: float) -> None:
        if not self.scheduler.needs_request():
            return
        problems = policy_wire.check_observation(obs)
        if problems:
            self.r.say(f"observation not valid: {'; '.join(problems)}")
            raise ExitRequest("soft", "observation_invalid")
        self.requested_at = self.scheduler.mark_requested()
        tick, client = int(self.requested_at), self.r.io.policy   # m6-5: the replay server advances with this tick
        self.worker.submit(lambda: client.infer(obs, tick))
        self.pending_since = now

    def _log_frame(self, obs: dict, action) -> None:
        """One episode frame per policy action (fps 30 in the dataset's sense, PLAN.md 3.4)."""
        if int(self.scheduler.position) < self.frames_logged:
            return
        r = self.r
        raw = NO_ACTION if action is None else np.asarray(action, dtype=np.float32)
        r.log.add_frame(obs[policy_wire.IMAGE_KEY], obs[policy_wire.STATE_KEY], r.command27(), raw)
        self.frames_logged = int(self.scheduler.position) + 1

    def _shadow_evaluate(self, action) -> None:
        """Shadow only: run the policy action through its own filter (envelope, limits) and count the clips (M5 abort rule)."""
        r = self.r
        _, events = self.shadow_filter.step(rs.sdk_targets(action, r.q0), r.measured, "policy", 0.0)
        self.shadow_ticks += 1
        self.shadow_clipped += "envelope_clip" in events
        for event in events:
            r.note_event(f"shadow_{event}")

    def _print_shadow_summary(self) -> None:
        if not self.send_policy and self.shadow_ticks:
            share = 100.0 * self.shadow_clipped / self.shadow_ticks
            self.r.say(f"shadow: {self.shadow_clipped} of {self.shadow_ticks} policy ticks envelope-clipped ({share:.1f} %; "
                       f"runbook M5 aborts above 5 %)")
