"""Unit: 86 moves between the factory pose and the ready pose only on the waypoints of g1sim/arm_paths.py - the same lines
87 checks against the table - and `86 recover` goes home on them after a crash (SAFE-PATHS.md S4).

Every test drives the real runner with the fakes of tests/helpers/fake_real_run.py. No DDS, no robot, seconds. That these
lines keep 5 cm from the table is the FK test in tests/test_integration_arm_paths.py.
"""
import numpy as np
import pytest

pytest.importorskip("unitree_sdk2py")

from g1sim import action_space, arm_clearance, arm_paths, real_io, real_preflight as pre, real_safety as rs  # noqa: E402
from g1sim.real_run_exits import HANDS_OPEN_S  # noqa: E402
from helpers.fake_real_run import (FACTORY, FakeArm, FakeClock, FakeFrames, FakePolicy, demo_actions,  # noqa: E402
                                   factory_record, last_index, make_io, run, sent, signals_at)

pytestmark = pytest.mark.unit

READY17 = rs.sdk_targets(action_space.BALL_READY_POSE, FACTORY)
WAIST_ROLL_PITCH = list(rs.WAIST_HOLD_POS)      # motors 13/14: q0 in 86, ignored by 87 (its FK sets the torso lean)
KILL_AT_S = 7.5                                 # s after the start: about the middle of way-in leg 2 for the fake FACTORY
HAND_BACK_SIGNALS_S = (20.0, 21.0, 24.0)        # in the hold: Ctrl+C soft exit, Ctrl+C freeze, Ctrl+C hand back (F4)
RIGHT_WRIST_ROLL = arm_paths.RIGHT_ARM.start + 4     # 17-vector position of motor 26; 0 on all of way-in leg 2
OFF_LEG_RAD = 0.30                              # moved by the owner's remote: more than LEAD_LIMIT (0.25) off the leg
KILL_IN_HOLD_S = 20.0                           # s after the start of `ready --hold 10`: in the hold (it starts at ~15 s)
KILL_IN_POLICY_S = 17.5                         # s after the start of `run --max-seconds 3`: in the policy phase (16-19 s)
QUICK_HAND_BACK_S = (20.0, 20.2, 20.4)          # 3 Ctrl+C in the hold: freeze and hand back right at the start of exit leg 1
ELBOWS = [arm_paths.LEFT_ARM.start + arm_paths.ELBOW, arm_paths.RIGHT_ARM.start + arm_paths.ELBOW]
OWNER_MOVED_RAD = 0.15                          # both elbows off the factory pose: not home (> 0.10), not on the log (review B1-r)


class KilledArm(FakeArm):
    """A fake arm whose run is killed (SIGKILL, power, cable) at `kill_at`: the send raises SystemExit, which no exit
    path catches, so the sidecar keeps the arm path logged last and the arm stays where it was (Unitree latches)."""

    def __init__(self, clock, kill_at: float):
        super().__init__(clock)
        self.kill_at = kill_at

    def send(self, weight, q17, kp17, kd17) -> None:
        if self.clock() >= self.kill_at:
            raise SystemExit("killed")
        super().send(weight, q17, kp17, kd17)


def kill_run(clock, tmp_path, mode: str, kill_after_s: float, io_fields=None, **args) -> KilledArm:
    """One 86 run killed `kill_after_s` after its start (sidecar: exit_code null, the arm path logged last)."""
    killed = KilledArm(clock, clock() + kill_after_s)
    with pytest.raises(SystemExit):
        run(mode, make_io(clock, killed, **(io_fields or {})), tmp_path, **args)
    return killed


def owner_moved() -> np.ndarray:
    """The factory pose with both elbows moved OWNER_MOVED_RAD, as the owner's remote may leave the arms."""
    moved = FACTORY.copy()
    moved[ELBOWS] += OWNER_MOVED_RAD
    return moved


def test_86_sends_exactly_the_waypoint_lines_87_checks(tmp_path):
    """Proves: `ready` sends exactly arm_paths.way_in on the way up and the way in backwards on the way down, from the
    ready pose 87 uses (equal except 13/14, which 87's FK ignores). So 87's way_in rows describe what 86 sends.
    Catches: a straight joint line put back into move_to_ready or the soft exit (it sweeps the low hands through the table
    edge, 87), or 86 and 87 drifting apart on the ready pose [mutation]."""
    clock = FakeClock()

    result = run("ready", make_io(clock, FakeArm(clock)), tmp_path, hold=0.5)

    phases, q = sent(result, "sent_phase"), sent(result, "sent_q")
    assert result.exit_code == 0
    assert np.allclose(q[phases == "to_ready"], arm_paths.path_ticks(arm_paths.way_in(FACTORY, READY17)), atol=1e-9)
    back = arm_paths.recover(READY17, arm_paths.WAY_IN, arm_paths.way_in(FACTORY, READY17), 3)
    assert np.allclose(q[phases == "to_start"], arm_paths.path_ticks(back), atol=1e-9)
    assert np.array_equal(np.delete(arm_clearance.READY17, WAIST_ROLL_PITCH), np.delete(READY17, WAIST_ROLL_PITCH))


def test_soft_exit_from_the_policy_phase_is_the_way_out(tmp_path):
    """Proves: the end of a policy phase sends exactly arm_paths.way_out from the last policy command (lift, waist home,
    ready pose, the hands-open pause there, then the way in backwards to q0) - the lift_home rows 87 checks, then its
    way_in rows backwards.
    Catches: the old straight lines policy pose -> ready -> q0 (through stand, basket and table edge, 87) [mutation]."""
    clock = FakeClock()
    policy = FakePolicy(demo_actions(shoulder_pitch=-0.6), mode="smolvla")

    result = run("run", make_io(clock, FakeArm(clock), frames=FakeFrames(), policy=policy), tmp_path, speed=0.5,
                 max_seconds=3.0)

    phases, q = list(sent(result, "sent_phase")), sent(result, "sent_q")
    last_policy = last_index(phases, "policy")
    path = q[last_policy + 1:phases.index("ramp_out")]
    way_out = arm_paths.way_out(q[last_policy], FACTORY, READY17, pre.LIMITS_LO, pre.LIMITS_HI)
    up = len(arm_paths.path_ticks(way_out[:arm_paths.WAY_OUT_READY + 1]))
    pause = round(HANDS_OPEN_S * rs.SEND_HZ)
    assert result.exit_code == 0
    assert np.allclose(np.delete(path, range(up, up + pause), axis=0), arm_paths.path_ticks(way_out), atol=1e-9)


def test_recover_after_a_kill_on_the_way_in_goes_back_along_it_to_q0_before_handing_back(tmp_path):
    """Proves: a run killed in the middle of way-in leg 2 leaves that leg in its sidecar; `86 recover` then starts at
    weight 1 on the measured angles, goes back along legs 2 and 1 (arm_paths.recover) to that run's q0, and only there
    ramps out to weight 0 (SAFE-PATHS.md S4).
    Catches: a recover that ends anywhere but the factory pose (e.g. "on to the ready pose": Unitree's controller then
    sweeps the arms down through the table, review safe-paths round 2 n2), the straight release line from the crash pose,
    or a sidecar that loses the leg [mutation]."""
    clock = FakeClock()
    killed = KilledArm(clock, clock() + KILL_AT_S)
    with pytest.raises(SystemExit):
        run("ready", make_io(clock, killed), tmp_path, hold=0.5)
    logged = real_io.last_arm_path(tmp_path)
    latched = FakeArm(clock, killed.q)

    result = run("recover", make_io(clock, latched, last_arm_path=logged, factory_record=factory_record()), tmp_path)

    phases, q, weight = sent(result, "sent_phase"), sent(result, "sent_q"), sent(result, "sent_weight")
    plan = [np.asarray(w) for w in logged["waypoints"]]
    assert (logged["plan"], logged["leg"]) == (arm_paths.WAY_IN, 2), "the kill did not hit way-in leg 2"
    assert result.exit_code == 0
    assert phases[0] == "release" and weight[0] == 1.0 and np.allclose(q[0], killed.q, atol=1e-9)
    expected = arm_paths.path_ticks([killed.q, plan[1], plan[0]])
    assert np.allclose(q[phases == "to_factory"], expected, atol=1e-9), "not back along the way in"
    assert np.allclose(q[phases == "ramp_out"], FACTORY, atol=1e-9) and weight[-1] == 0.0


def test_recover_refuses_without_a_logged_arm_path(tmp_path):
    """Proves: `86 recover` with no arm path from our last run refuses before any writer exists (exit 2) and sends nothing.
    Catches: a recover that guesses the way home - from a pose low on the way in, the way out goes 7-12 cm into the table
    (review safe-paths B1) [mutation]."""
    clock = FakeClock()
    arm = FakeArm(clock)

    result = run("recover", make_io(clock, arm, last_arm_path=None), tmp_path)

    assert result.exit_code == 2 and "logged no arm path" in result.reason
    assert arm.messages == []


def test_recover_refuses_when_the_arms_are_not_where_the_log_says(tmp_path):
    """Proves: `86 recover` refuses before any writer exists (exit 2, nothing sent) when the arms are not where the last
    sidecar says - (a) back where that run took them over (a kill in the hold, then the owner's remote put them home):
    the text names `86 release`; (b) moved more than LEAD_LIMIT off the logged way-in leg 2 (owner's remote, or a kill
    Unitree did not latch).
    Catches: a recover that trusts the log and drives home from wherever the arms are - from the factory pose that is
    the way out, -7 to -12 cm into the table from a hold/policy log, and -15 cm from other logs (review safe-paths-code
    B1) [mutation: either check removed; the hold/shadow check moved above the home check, review B1-r m1]."""
    clock = FakeClock()
    kill_run(clock, tmp_path, "ready", KILL_IN_HOLD_S, hold=10.0)
    home_arm = FakeArm(clock)                           # the owner's remote put the arms back at the factory pose

    home = run("recover", make_io(clock, home_arm, last_arm_path=real_io.last_arm_path(tmp_path)), tmp_path)

    killed = kill_run(clock, tmp_path, "ready", KILL_AT_S, hold=0.5)
    moved = killed.q.copy()
    moved[RIGHT_WRIST_ROLL] += OFF_LEG_RAD
    moved_arm = FakeArm(clock, moved)

    off_leg = run("recover", make_io(clock, moved_arm, last_arm_path=real_io.last_arm_path(tmp_path),
                                     factory_record=factory_record()), tmp_path)

    assert home.exit_code == 2 and "where our last run took them over" in home.reason and "86 release" in home.reason
    assert off_leg.exit_code == 2 and "from where our last run logged them" in off_leg.reason
    assert home_arm.messages == [] and moved_arm.messages == []


def test_recover_from_a_hold_log_refuses_off_the_ready_pose_and_goes_home_from_it(tmp_path):
    """Proves: after a kill in the hold (sidecar phase hold, exit_code null), `86 recover` refuses with nothing sent when
    the arms are more than LEAD_LIMIT from the logged ready pose (owner moved them), and still goes home (exit 0, ends at
    the factory pose) when they are latched at it.
    Catches: the free-phase hole of review safe-paths-code B1-r (a): the old code drove the way out from the moved pose,
    9.9 cm into the table in the review's repro [old code; mutation: hold/shadow check removed]."""
    clock = FakeClock()
    killed = kill_run(clock, tmp_path, "ready", KILL_IN_HOLD_S, hold=10.0)
    logged = real_io.last_arm_path(tmp_path)
    moved_arm = FakeArm(clock, owner_moved())

    moved = run("recover", make_io(clock, moved_arm, last_arm_path=logged, factory_record=factory_record()), tmp_path)

    latched_arm = FakeArm(clock, killed.q)
    latched = run("recover", make_io(clock, latched_arm, last_arm_path=logged, factory_record=factory_record()),
                  tmp_path)

    assert logged["phase"] == "hold", "the kill did not hit the hold"
    assert moved.exit_code == 2 and "the ready pose of its hold phase" in moved.reason
    assert moved_arm.messages == []
    assert latched.exit_code == 0 and np.allclose(sent(latched, "sent_q")[-1], FACTORY, atol=1e-9)


def test_recover_refuses_a_policy_log(tmp_path):
    """Proves: after a kill in the policy phase, `86 recover` refuses with nothing sent and names the next step (owner's
    remote, or table back and `86 release`): the policy phase logs no pose to check the arms against
    (RECOVER_FROM_POLICY_LOG False, ruling pending).
    Catches: review safe-paths-code B1-r (b): the old code drove the way out from wherever the arms were after a policy
    kill [old code; mutation: policy refusal removed]."""
    clock = FakeClock()
    policy = FakePolicy(demo_actions(shoulder_pitch=-0.6), mode="smolvla")
    killed = kill_run(clock, tmp_path, "run", KILL_IN_POLICY_S, dict(frames=FakeFrames(), policy=policy), speed=0.5,
                      max_seconds=3.0)
    logged = real_io.last_arm_path(tmp_path)
    latched_arm = FakeArm(clock, killed.q)

    result = run("recover", make_io(clock, latched_arm, last_arm_path=logged, factory_record=factory_record()), tmp_path)

    assert logged["phase"] == "policy", "the kill did not hit the policy phase"
    assert result.exit_code == 2 and "policy phase" in result.reason and "86 release" in result.reason
    assert latched_arm.messages == []
    assert pre.RECOVER_FROM_POLICY_LOG is False


def test_recover_refuses_after_our_last_run_handed_the_arms_back(tmp_path):
    """Proves: when our last run finished with weight 0 (here: 3 quick Ctrl+C, freeze and hand back at the start of exit
    leg 1), `86 recover` refuses with nothing sent even if the arms sit on the logged leg: they are Unitree's now
    (real_io.arms_may_be_held_by_us False) and no log says where they are.
    Catches: review safe-paths-code B1-r (c): the old code trusted the stale log and drove its way home from there
    [old code; mutation: hand-back check removed]."""
    clock = FakeClock()
    run("ready", make_io(clock, FakeArm(clock), signal_count=signals_at(clock, *QUICK_HAND_BACK_S)), tmp_path, hold=10.0)
    on_leg_arm = FakeArm(clock, READY17)                # the owner's remote left the arms at the start of the logged leg

    result = run("recover", make_io(clock, on_leg_arm, last_arm_path=real_io.last_arm_path(tmp_path),
                                    factory_record=factory_record()), tmp_path)

    assert not real_io.arms_may_be_held_by_us(tmp_path)
    assert result.exit_code == 2 and "already handed the arms back" in result.reason
    assert on_leg_arm.messages == []
