# Review: FENCE-SMOOTH (virtual fence + chunk-boundary blend)

## Plan review round 1 (2026-11-08, independent reviewer; did not write the plan)

Reviewed `.planning/real-robot/FENCE-SMOOTH.md` against `g1sim/real_safety.py`, `real_timing.py`, `real_run.py`,
`real_run_policy.py`, `arm_clearance_fk.py`, `vla_eval.py`, `ball_grasp.py`, `scripts/setup_robot_laptop.sh`,
`tests/test_e2e_real_adapter_sim.py`. Numbers re-measured on the Mac (one sim process at a time, no VLA model, no 85, no test
suite). Raw output: `/tmp/fence/review/` (`freeze20.log`, `freeze20_stats.txt`, `demo_nominal_walk2.txt`,
`blend_rerun.log`, `blend_replay_rows.txt`, `teacher_a_p99.txt`).

**Verdict 1 - is the plan right?** Partly. The idea is right: FK fence on min(cmd, pred), hold the last sent command, soft
exit on a streak, placed after the rate limit and before the lead monitor, scripted phases untouched, blend in the
scheduler. Gaps in the design: the margin is calibrated with zero layout error, the upper arm is not fenced, the hand part of
`pred` misses "sagged arm + closed fingers", and the shadow fence value is meaningless.
**Verdict 2 - buildable as written?** No. The robot laptop cannot load the fence model, and the blend and the nominal fence
each make the existing replay gate (E2) fail.

**GO / NO-GO: NO-GO.** Three blockers (B1-B3), all fixable in one plan revision. The concept stays.

### What I re-measured (matches the plan)

| item | plan | re-measured | how |
|---|---|---|---|
| teacher fence minimum (pred as specified: cmd + (measured before send - last sent)) | table 4.46, stand 3.70, basket 3.60, torso 4.04, other arm 5.76 cm (50 trials) | table 4.47, stand 3.78, basket 4.37, torso 4.04, other arm 5.85 cm; head, legs >= 10 (20 trials, seed 0, 20/20 passed) | `freeze20_stats.txt` |
| FK + 7 groups, one call | 0.416 ms mean, p99 0.85 | 0.37 ms mean, p50 0.24, p99 0.74, max 1.9 ms (17,520 calls); 2 calls/tick = ~0.75 ms of 20 ms | `freeze20.log` |
| nominal scene load | - | 0.6 s (ClearanceModel) | one-off |
| p99 \|a\| at 0.5: VLA lat 0/3, + blend 5 | 21.8 / 29.2 -> 12.4 / 12.6 | 21.76 / 29.23 -> 12.38 / 12.56 (identical) | `blend_rerun.log` |
| teacher p99 \|a\| at 0.5 | 8.84 | 8.84 pooled; per trial median 8.85, **max 15.85** | `teacher_a_p99.txt` |
| streak N | 109 / 192 ticks | same | arithmetic |

Small note: the planner's script computed `pred` with the measured state one tick after the action was applied (one tick
later than the spec). With the spec formula the minima move by <= 0.8 cm upward (basket) and <= 0.1 cm elsewhere, so the
margins stay valid.

### Blockers

**B1 - the robot laptop cannot build `Fence.nominal`.** 86 today loads no MuJoCo (imports: numpy, cyclonedds, unitree_sdk2py,
opencv). The robot env is Python 3.10 with `ROBOT_PINS="cyclonedds==0.10.2 numpy==2.2.6 opencv-python-headless..."`
(`scripts/setup_robot_laptop.sh:24`): no `mujoco`. `TableEnv("volleyball_walk")` loads its XML from
`SIM_DIR = ~/dev/g1-ext/g1_brainco_sim` (`g1sim/__init__.py:14`), which is not on the robot laptop and is outside the repo, so
the motion gate does not hash it either. The first attempt probed an `.mjb` export (`/tmp/fence/probe/mjb_small.py`,
109 MB) but the plan dropped it. Needed in the plan: a collision-only fence model file shipped with the repo or the release
(small: robot collision geoms + table/stand/basket, no textures), its hash in the motion gate, `mujoco==<the Mac's 3.13.0>`
pinned in the robot env (mj_geomDistance results depend on the version), a check in `setup_robot_laptop.sh` and 89, and 86
refusing `run`/`replay` off loopback when the model does not load (fail closed). Without this, B1-B5 can only run on the Mac.

**B2 - the 2 cm margin assumes the fence knows the layout exactly; the replay gate and I6 fail as written.** The recorded demo
`tests/data/replay_demo.npz` (a successful demo, layout unknown: the file stores no variation) through the prototype fence
in the **nominal** layout: stand **1.61 cm**, 43 actions below 2 cm, longest run **42 actions = 2.8 s = 140 ticks at
`--speed 0.5` > N = 109** -> `fence_streak` soft exit -> E2 (`result["code"] == 0`) fails and I6 ("0 holds") fails.
Sensitivity: moving the stand 2 cm changes that minimum to -0.27 / 1.50 / 2.00 / 3.55 cm (dy +2 / dx -2 / dx +2 / dy -2),
i.e. about 1 cm of fence value per cm of layout error. v2 trains with ball+stand offsets up to 10 cm, so the policy will
reach wherever the stand really is. On the robot day the fence's scene comes from tape measurements and the nominal pelvis
(feet on marks; pelvis tilt from the IMU is not used, only waist 13/14). A 1-2 cm layout error either stops good grasps
(stand side) or leaves ~0 cm real clearance (table side). Needed in the plan:
- tests: I6/E2 run the fence with the demo's own layout (store the variation with the test demo, or record a new test demo
  with a known layout), or E2 runs without the fence and a separate E2b checks the fence path;
- a stated error budget (tape setback/offsets, table height via B3, stance on the marks, pelvis tilt) and what margin is
  left per group; consider pelvis roll/pitch from LowState IMU in `pred`, and per-group margins (the table position is the
  best known, the stand the worst);
- an operator check before live that shows the fence values with the entered layout (see M4: today's shadow cannot).

**B3 - the blend breaks the replay gate.** `test_replay_tracks_demo_inside_envelope` asserts every logged policy action lies
within `CONTENT_TOL = 1e-4` rad of the demo polyline (`demo_rows_matched`, `-1 not in rows`). Through ChunkScheduler with
blend 5 on `replay_demo.npz` at speed 0.5: **4-6 of 2,417 logged actions are off the polyline** (max 0.0004 / 0.0028 /
0.0097 rad at latency 0 / 1.7 / 3); blend 0 gives 0. The plan's "max difference 0.024 rad, E2 must still pass" checks the
wrong thing. Fix: no blend in `replay` (replay chunks are slices of one demo, the blend buys nothing there), or log the
unblended action and say why. Then U8 must also prove "replay mode -> no blend".

### Major

**M1 - the upper arm is not fenced.** Moving parts start at `*_elbow_link`. The teacher's shoulder links (shoulder pitch /
roll / yaw geoms) come to **5.08 cm** of the basket (median 7.9 cm); the forbidden-contact judge counts every robot geom,
and 87 checks everything below `*_shoulder_pitch_link` (`arm_clearance_fk.py:19`). A waist-yaw swing or a wide shoulder roll
can put the upper arm into the basket unseen. Add the shoulder-link geoms to the moving parts for the scene groups (not for
torso: they touch it by construction), and add the group to I1.

**M2 - a hold does not stop the arm, and once inside the margin the arm cannot get out.** Freeze probe (sim, teacher at speed
1, 1,219 probes: hold the accepted command for 1 s, record the smallest measured distance): overshoot past the accepted fence
value stand p50 0.0 / p99 **1.08** / max **1.40 cm** (touch), table max 0.49, basket max 0.62 cm. So at a 2.0 cm margin a hold
can still end ~0.6 cm away in sim (less at speed 0.5, but the real layout error comes on top, B2). Also: after an overshoot the
held pose itself is below the margin, every rate-limited step (<= 0.02 rad/joint) is checked against "< margin", and a step
rarely gains 1 cm, so the arm stays held until the streak exit. The plan's reason for a 2.2 s streak ("the policy gets a chance
to steer away") is then mostly not true. Options: hold only if the command is below the margin **and** closer than the current
pred pose ("never closer than now", the same rule 87's `exit_passes` uses), or keep the rule and say plainly that a hold almost
always ends in a soft exit (then a shorter streak loses nothing). Put the overshoot numbers into 1.4.

**M3 - hands in `pred`.** `pred` shifts the hands by their measured error, `cmd` has the closed hand target but no arm sag.
Neither checks "sagged arm + fingers at their target" (BrainCo fingers lag the target by up to a second, then arrive). Use the
commanded hand target in both poses (only the arm/waist get the tracking error). Second issue: U7 "NaN measured -> hold"
must not cover hands: hand state can be NaN or absent (`--no-hands`, `hand_state_synthetic` in `real_run_policy.py:130`); with
fail-closed hands every such run would end in `fence_streak`. Say: arm/waist NaN -> hold; hand state missing -> the commanded
target.

**M4 - the shadow fence number is meaningless.** In shadow the arm holds the ready pose while the shadow filter's `last` follows
the policy, so `pred = cmd + (measured_ready - shadow_last)` is about the ready pose and never near anything; the shadow share
counts only `cmd` violations, which the teacher never comes close to (teacher cmd: table 10, stand 9.8 cm). R-F5's 2 % abort rule
would be fed by the wrong number. Shadow needs a sag model (e.g. `pred = cmd + sag offset` from a table of measured arm error
vs pose), or the plan must drop R-F5 and say shadow checks `cmd` only.

### Minor

- m1 `SafetyFilter.step(target17, measured17, phase, weight)` has no hand arguments; B2/B3 must change its signature and
  `Runner.tick` must pass hand target + hand state. Say so in B2/B3 and U3.
- m2 Layout entry has no stand/basket yaw (v2 trains yaw up to 10 deg; a rotated rectangular basket moves its rim by cm).
- m3 Calibration measured only table/stand/basket; `from_env` adds the FORBIDDEN distractors. E1 (teacher `--fence`, 100) covers
  it; keep E1 mandatory before any VLA comparison.
- m4 E3 "no contact in 85": the 85 sim (LeRobot G1 sim) has no table, so that part is empty. Keep the hold/exit assertions,
  move the contact claim to 82/I7.
- m5 Smoothness bar: the teacher's p99 is pooled over 100 trials with long still phases; one teacher trial reaches 15.85
  rad/s^2 (bar 17.7). The VLA number is one trial. Compare per trial or over moving ticks only, and keep S3 (run3 chunks).
- m6 A hold is a stop from up to 1 rad/s to 0 in one tick (up to 50 rad/s^2), the resume is rate-capped - bounded like today,
  but add holds to the 82 `--fence --blend` smoothness numbers.
- m7 The adjacent envelope finding (18/100 teacher trials above `ENVELOPE_HI` wrist pitch) is consistent with the smoothness
  replay (teacher replay at 0.25: 6,961 envelope clips over 100 trials). Not checked further (out of scope).

### Checked and fine

- Fence stage after the rate limit and before the lead monitor (`real_safety.py:287-290`): the fence sees what would be sent,
  the lead monitor sees the held command. Hold = `self.last` keeps `step()`'s `self.last = cmd` consistent.
- `fence_streak` -> soft exit through `_react` like `envelope_streak`: no path drops the arms; an FK exception becomes
  `internal_error` -> HOLD (weight 1), not a ramp-out.
- Segment argument (1.3) holds with endpoints checked every tick; U9 pins it.
- 50 Hz budget: ~0.75 ms per tick for two FK calls on the M4; E4 must repeat it on the robot laptop.
- Hands held on `fence_hold`: `Runner.send_hands` re-sends the last hand target, which passed the fence on its tick.

### What the next revision must contain for GO

B1 (model on the robot laptop + gate + setup/89), B2 (layout-aware tests + error budget + operator check), B3 (no blend in
replay or unblended log), M1-M4 decided (fix or say plainly why not).

Mac use for this review: one sim process (20 teacher trials + 1,219 freeze probes, 91 s, one core), FK-only and numpy
scripts (a few seconds to ~1 min each), never in parallel; no VLA model, no 85, no test suite.

## Plan review round 2 (2026-11-08, independent reviewer; did not write the plan)

Reviewed `.planning/real-robot/FENCE-SMOOTH.md` revision 2 against round 1, the code (`motion_gate.py`, `randomization.py`,
`table_env.py`, `real_run_policy.py`, `real_timing.py`, `setup_robot_laptop.sh`, `tests/test_e2e_real_adapter_sim.py`) and the
planner's raw data. Re-derived on the Mac: one FK-only process (1.2 s, `/tmp/fence/review2/self_hull.log`), small numpy
reads of `/tmp/fence/b2/`, one PyPI query. No sim batch, no VLA model, no 85, no test suite.

**Verdict 1 - is the plan right?** Yes, within what a layout-from-tape fence can do. It now says plainly that it cannot
promise "no touch" on the real robot (layout error ~1.5 cm RSS + overshoot up to 1.4 cm > margins); that is a ruling for
Finnegan (R-F1), not a design error.
**Verdict 2 - buildable as written?** Yes, with the small items below folded in by the builder.

**GO.** All round-1 blockers and majors are fixed; the new items are one decision to write down (N1) and minor clarifications.

### Round-1 findings re-derived

| finding | status | evidence (re-derived) |
|---|---|---|
| B1 model on the robot laptop | fixed | Mac mujoco 3.13.0; PyPI 3.13.0 cp310 wheels: macOS arm64, Linux x86_64/aarch64, Windows - no Intel-Mac wheel (R-F9 correct). Hull model `/tmp/fence/b1/fence_model.xml` vs full TableEnv on 90 teacher poses: **self groups too** (torso, other arm, head) and upper arm vs stand differ by <= 0.0009 mm; geom sets identical (`review2/self_hull.log`; the planner had checked only table/stand/basket). `gate_problems` does `git diff` per path, so adding `fence.DATA_FILES` to the list works for a data file. Fail closed off loopback + 89 step present. |
| B2 layout | fixed | `demo_layout.log`: replay demo = s101 t0, state[0] equals the regenerated reset exactly (0.0), head image 3.76 vs 42.9/46.4. Own layout: table 1.97 / stand 3.27 / basket 4.18 cm -> above the new margins, 0 refusals (I6). R-F8 placement gives stand 3.54 (`replay_stand_moved.log`). Per-group margins recounted from `demos50_fence.json`: p5 table 1.82, stand 2.98, basket 1.38 -> 1.3 / 2.0 / 0.88 (plan rounds to 0.9); exactly 1 of 50 demos below its margin (ep 264, basket). Error budget RSS ~1.5 cm checks out. Operator check = replay with real `pred`. |
| B3 blend in replay | fixed | blend 0 by default, only `run` passes 5; `PolicyPhase` already knows `runner.mode` (`real_run_policy.py:27,67`); U8 covers it. |
| M1 upper arm | fixed | scene moving parts = below `*_shoulder_pitch_link`; I1 has "upper arm into basket". |
| M2 hold / no escape | fixed | "never closer than holding still" + streak counts only held ticks; N = ceil((25/(30*0.5)+0.5)*50) = 109 (192 at 0.25) re-computed. |
| M3 hands in pred | fixed | commanded hand target in both poses; hand NaN / `--no-hands` never holds (U7). |
| M4 shadow | fixed | shadow = `cmd` only, labelled informational; R-F5 dropped. |
| m1-m7 | fixed / flagged | m2: re-checked `randomization.py` - the walk world shifts ball+stand and basket by x/y only, no object yaw; the 10 deg "stand" yaw in the config is the robot stance, now a layout entry. m3 E1 mandatory, m4 contact claim moved to I7, m5 per-trial bar 17.13 (`m5/per_trial.log`), m6 S2, m7 flagged. |

### New findings (round 2)

**N1 (major, decide in the plan, not at build time) - the segment check.** 1.4 says the rate cap allows up to ~2.3 cm
geom movement per tick and the no-contact bound needs s < 2 x smallest margin = 1.8 cm. So U9 as written is expected to
fail by the plan's own numbers, and "the build adds a midpoint check" leaves open what the check does. Write it down now,
for example: on every policy tick where any scene group is < 3 cm, also check the midpoint pose (mean of last and cand, same pred offset); the midpoint counts in F(cand)
(below margin and closer than hold -> hold). Cost: one more FK (~0.4 ms) only near objects; add it to the 1.5 budget and to
E4. Then U9 becomes "midpoint value <= min of endpoints is caught", not a pass/fail on the rate cap.

**N2 (minor) - which hand target is in F(hold).** On a hold the hands keep the last sent target, so F(hold) must use the last
sent hand target and F(cand) the new one. 1.5 step 3 says only "hands commanded". Say which, and test it in U10.

**N3 (minor) - loopback default.** "Refuse off loopback without a fence" and "`--no-fence` for loopback tests only" leaves open
what 86 does on loopback with no layout and no `--no-fence`. Recommend: error (no silent default) and add `--no-fence` to the
existing e2e commands (fault injection, G2, 40k) - then `tests/test_e2e_real_adapter_sim.py` belongs in the build table.

**N4 (minor) - operator check threshold.** "A value far below that in the replay = stop" needs a number, e.g. any hold, or
any group > 1 cm below the expected value (table 2.31, stand 3.54, basket 3.02), and the runbook (B7) must print it.

**N5 (minor) - chatter near a surface.** Any sent tick resets the streak, so a policy that alternates push (held) and slide
(sent) never reaches the soft exit and the arm stops and starts at 50 Hz near the object. Safe for distance, but it is a
jerk source. S2 measures holds; also report hold/send alternations per trial in 82 `--fence`, and if frequent, count held
ticks in a sliding window (e.g. N held of the last 2N) instead.

**N6 (minor) - margin rounding.** The rule gives basket 0.88 cm, the plan uses 0.9; say "rounded to 1 mm" or use 0.88.

### Not a finding, stated for Finnegan

With the estimated layout error and sim overshoot, a push ends between ~2 cm before and ~1-2 cm into the object (1.3).
The goal "never hits table/stand/basket" is therefore not guaranteed by the fence; it lowers the chance and the force.
The alternative (2.0 cm everywhere) is in R-F1.

Mac use for this review: one FK-only process (1.2 s, one core), numpy reads of existing files, one PyPI JSON query; no
sim batch, no VLA model, no 85, no test suite, no background process left.

## Code review round 1 (2026-11-08, independent reviewer; did not write the code)

Read: `git diff` of the 17 changed files and every new file (`g1sim/fence.py`, the three fence test files, the two
helpers, `tests/data/replay_demo_layout.json`). Raw output of everything below: `/tmp/fence/review3/`.

**Verdict 1 - did the code do what the plan said: YES.** B0-B8 are built as in FENCE-SMOOTH.md rev 2 (fence after the rate
limit, before the lead monitor, policy phase only; min over cmd and pred, hands commanded, waist roll/pitch measured, IMU
tilt; never-closer rule; streak = one chunk + 0.5 s -> soft exit; hands not sent on a hold; shadow cmd-only, no hold; blend
only in `run`; fail closed off loopback; model file in the gate; 82 `--fence`/`--blend`; 89 step; setup pin). The E2
deviation is reported in 6.3, not hidden.

**Verdict 2 - was the plan right: MOSTLY.** Two things the plan got wrong (O1, O2 below), both rulings, not bugs in this
diff.

**Overall: OPEN.** No behaviour bug found in the new code. Open because E2 is a red test in the suite and needs Finnegan's
ruling (O1), and O2 needs a decision before live runs at the table.

### What I ran (Mac, one process at a time, no VLA model)

| check | result |
|---|---|
| `scripts/run_tests.sh` (once, 4 min 55 s) | project 318 passed, sim env 578 passed, openpi 10 passed, exit 0 |
| `82 --no-walkup --teacher --fence --seed 0 --trials 50` (once, 93 s) | 50/50 passed, **0 holds, 0 fence exits**; smallest table 4.43, stand 3.70, basket 3.60, torso 4.04, other arm 5.76 cm - same as the implementer's E1 |
| Probe A, real `SafetyFilter` + real `Fence.nominal` (hull model), FK only (90 s) | 34 target poses inside the demo envelope and joint limits, each putting a **fingertip, palm or forearm 1.0-2.0 cm into** table, stand, basket, torso, head, pelvis/legs or the other arm (left and right); driven from the ready pose, perfect tracking and with a 0.05 rad arm sag: **no sent command inside any group, none more than 3 mm below its margin, the arm never inside**; every one ends in `fence_streak` after 109 held ticks. 8 part/group pairs not reachable inside the envelope (e.g. pelvis/legs with the forearm) |
| Probe B, the real 86 `Runner` with the test fakes and the real fence (table/palm, stand/fingertip, basket/fingertip, torso/palm) | exit 3 `fence_streak`, one `FENCE:` line, held messages repeat the last command, nothing sent inside, then `to_ready -> to_start -> ramp_out`; weight stays 1 until the final 2 s ramp-out (**nothing drops the arms**) |
| Fence time per tick (Mac M4) | no-hold path ~1.1-1.5 ms p99; while holding 1.5-4.4 ms p99 (probe A), 4.23 ms in 85 - inside the 20 ms tick |
| Mutation check (26 mutants, my own list incl. removing each new check; sources restored, checked with `cmp`) | **26/26 caught**; "streak exit as fault" only by a test hang (300 s timeout), see m3 |
| 85 e2e, E2 + E3 only (once, 1 min 56 s) | E3 passed; **E2 failed again** exactly as in 6.3: `holds 301 of 712 ... SOFT EXIT fence_streak` |

### Open (rulings)

**O1 - E2 red: the plan's expectation "85 replay with the fence: 0 holds" is wrong (reproduced).** In 85 the torso leans
forward more than in the training sim and the replayed arm really passes within mm of where basket and stand would be; the
fence holds it, as it should. The code is right; the test and the plan expectation are not. The real risk is for Friday: if
the real robot sags like 85, the M4 replay will be held near the basket and the runbook's fence go/no-go stops the day before
M6. Needs Finnegan's choice from 6.3 (a)/(b)/(c). Until then `test_replay_tracks_demo_inside_envelope` stays red in
`run_tests.sh all`. My suggestion: (c) first - read the real waist pitch at the ready pose in M2 (84 shows it) and decide on
site; change E2 to assert the fence numbers instead of exit 0, so the suite is green and still catches a broken fence.

**O2 - the soft-exit route is not checked against the robot's own body (old code, but the plan says the scripted paths are
safe).** After a fence streak 86 takes `arm_paths.way_out`: lift (shoulder pitch -0.1, elbow -0.5 rad) -> waist home ->
ready. 87 checks that route only against table, stand and basket. From held poses near the torso or head (the self fence
held them at 2.1-2.6 cm) the FK of that route goes **2-6 cm into the torso or head**, mostly on the lift leg (the elbow bend
brings the hand up toward the chest/face). Example: left fingertip held 2.11 cm from the torso -> lift leg -5.97 cm into the
torso. Not caused by this diff: the same route runs after `envelope_streak`, `done` or Ctrl+C from such a pose, and without
the fence the hand would have gone into the body anyway. The demos keep >= 4 cm from the torso, so it needs an off-demo
policy. On the robot the lead monitor would fault after ~0.26 s of pushing. Options: let 87 also check the exit route
against the self groups from the fence model; or in 86, before a soft exit from the policy phase, compute the route with the
fence and, if it goes closer than its start to a self group, hold and fault (operator) instead. Probe log
`/tmp/fence/review3/probe_exit.log`.

### Minor (no behaviour change needed for safety)

- **m1 - FENCE SUMMARY "smallest" counts refused commands.** `FenceLog.tick` takes the values of every checked command,
  including held ones that were never sent. Probe B: summary "stand 0.46 cm", while no sent command came closer than
  2.49 cm. For the go/no-go this only errs on the safe side (a refused command also means holds > 0), but the line reads as
  "the arm came this close". Report sent and refused minima separately. The 6.3 numbers (-0.39 / -0.96 cm) are refused
  commands; the implementer's offline check (measured arm 0.14 / 1.63 cm) is the right number.
- **m2 - 89 CHECK fence times only the no-hold path.** The replay demo has no holds, so 89 times 2 FK per message. A hold
  tick costs 6-7 FK (values + midpoint + held + overshoot watch): 4.2-4.4 ms on the M4 vs 1.34 ms in 89. On a slower robot
  laptop 89 can pass while a holding tick takes ~3x longer. Time a held tick too (e.g. a pushed copy of the demo).
- **m3 - "fence_streak as fault" is caught only by a hang.** The test runs into the endless fault hold of the fakes instead of
  failing fast; give the runner test a clock limit.
- **m4 - `--fence-layout FILE` without "objects"** ends in a KeyError traceback (before DDS, nothing sent). Catch it and
  refuse with a line.
- **m5 - first policy tick:** F(hold) uses the candidate's hand target (`last_hands` is still None), not the ready-pose hands
  that are really out. Negligible (one tick, hands open).

### Checked and fine

Policy-only scope (fence off in every scripted phase); held command = `self.last`, lead monitor on the held command; hands not
commanded on a hold and resumed after; NaN arm/waist angle or NaN IMU holds, NaN hand state never does (`--no-hands` works);
shadow uses cmd only and never holds; blend only in `run` (replay polyline intact), blend math (old->new over 5 actions, no
fade when the old chunk ran out); 86 refuses replay/run off loopback without layout, with `--no-fence`, without mujoco
3.13.0 or without the model file, before the gate and before DDS; gate lists `g1sim/data/fence_model.xml.gz`; IMU rpy order
matches the sim's `mju_euler2Quat(..., "XYZ")` (fixed axes = Unitree roll/pitch/yaw); teacher never fenced (E1);
`ball_grasp.py`, training scripts, dataset code and `docs/ROBOT_DAY_MOTION.json` untouched.

Mac use for this review: about 16 min of single-process CPU in total (tests 5 min, 82 1.5 min, FK probes 2.5 min, mutants
5 min, 85 two tests 2 min), never two sim processes at once, no VLA model, no background process left (checked with `ps`).

## Code review round 2 (2026-11-08, independent reviewer; did not write the code or the fixes)

Re-checked every round-1 item against the code after fix round 1. The fix diff (vs the round-1 backup
`/tmp/fence/review3/backup/`) touches only `real_safety.py` (`reset_last(..., hands12)`, `FenceLog.refused_smallest`),
`real_run_policy.py` (hand-over call, summary line), `scripts/86_real_run.py` (layout check), `tests/helpers/fake_real_run.py`
(`FakeClock(limit_s)`) and tests; `fence.py`, `_apply_fence`, `real_timing.py`, `vla_eval.py`, `real_run.py` unchanged since
round 1. Raw output: `/tmp/fence/review4/`.

**Verdict 1 - did the fixes do what round 1 asked: YES.** **Verdict 2 - were the fixes right: YES.** No regression found.

**Overall: OPEN** - no behaviour bug; still open only for the two rulings O1 (E2 red in `run_tests.sh all`) and O2 (soft-exit
route vs torso/head), both unchanged and both Finnegan's call.

### What I ran (Mac, one process at a time, no VLA model)

| check | result |
|---|---|
| `scripts/run_tests.sh` (once, ~5 min, one core) | project 320 passed, sim env 583 passed, openpi 10 passed, exit 0 (`run_tests.log`) |
| Probe B again: real 86 `Runner` + real `Fence.nominal`, pushes into table/stand/basket/torso (5 s) | identical behaviour to round 1: exit 3 `fence_streak`, 109 holds, **nothing sent inside or below margin**, `policy -> to_ready -> to_start -> ramp_out`, weight 1 until the final ramp-out. Summary now: stand case `smallest 2.02 | refused 0.46` (round 1 printed 0.46 as "smallest"); basket case `smallest 0.91 | refused -0.18` (margin 0.9) (`runner_probe.log`) |
| 86 with a partial layout file (basket without dy_m), en5, no DDS | refused before gate and DDS, but as a KeyError traceback (see n1) (`m4_partial.log`) |
| leftover processes | none (`ps`) |

Not re-run (no code on their path changed): probe A, 82 teacher `--fence`, 85 e2e, mutants (the fix round's own 5 mutants
are in `/tmp/fence/fix1/mutants.log`, all caught).

### Round-1 items

| item | status | evidence |
|---|---|---|
| m1 summary "smallest" counted refused commands | fixed | held ticks go to `refused_smallest`, sent to `smallest`; probe B sent minima are all >= the group margin; runbook M4 text says what each number means; unit test pins it |
| m3 streak-as-fault caught only by a hang | fixed | `FakeClock(limit_s)` raises `ClockLimit` (BaseException, so the runner's `except Exception` cannot swallow it); fix-round mutant fails in 0.5 s |
| m4 layout file without objects | fixed for the case named | parser refuses with a line when `objects.volleyball.dx_m/dy_m` is missing; see n1 for the rest |
| m5 first-tick F(hold) hands | fixed | `PolicyPhase.run` passes `hand_command()` (open hands if never commanded) to `reset_last`; only the policy-phase call passes hands, the fault-recover `reset_last(target)` keeps the old ones (fence off there) |
| m2 89 times only the no-hold path | open (minor, not in the fix list) | unchanged |
| O1, O2 | open (rulings) | unchanged |

### New minors (no safety effect)

- **n1 - m4 only covers the stand key.** A layout file with `basket` but no `dy_m`, `table` without `dz_m`, or `stance` without
  `yaw_deg` still ends in a KeyError traceback in `Fence.nominal` (`build_fence` catches only `FenceUnavailable`). It fails
  closed before the gate and DDS, so it is cosmetic: catch KeyError/TypeError in `build_fence` and refuse with a line.
- **n2 - the FENCE SUMMARY line shows scene groups only.** When the holds come from a self group (probe B torso case), the
  line reads `refused (not sent) smallest cm: basket 10.00, stand 10.00, table 10.00` - it looks like nothing was refused
  near anything. The `FENCE:` line before it names the torso, and the sidecar has all groups. Fine for the M4 go/no-go (H > 0
  stops anyway); add self groups to the refused part if it confuses the operator.

Mac use for this review: about 5.5 min single-process CPU (tests 5 min, runner probe 5 s, one 86 parse), never two sim
processes at once, no VLA model, no background process left.

## O2 code review round 1 (2026-11-10, independent reviewer; did not write the fix)

Scope: the uncommitted O2 diff on b097514/63cbed6 - `Fence.route_refused`, `arm_paths.way_out(route=)` / `ready_index`,
`real_run_exits.safe_way_out` (soft exit from policy, fault recover from policy/hold/shadow), `FAULT_ACTIONS
exit_route_into_body: HOLD`, 6 new tests, FENCE-SMOOTH.md 6.5, runbook lines. Raw output: `/tmp/fence-o2/review1/`.

**Verdict 1 - did the code do what was intended: YES.** **Verdict 2 - was the approach right: YES** (the fence's own geometry
and never-closer rule along the 50 Hz targets, first passing route, else hold; no new geometry to keep in sync).

**Overall: GO** - no behaviour bug found. Minors below; m3 and m4 are rulings for Finnegan.

### What I ran (Mac, one process at a time, no VLA model, no 81)

| check | result |
|---|---|
| Probe A (`probe_o2.py`, real SafetyFilter + real `Fence.nominal`, 70 random targets 0.3-3 cm inside table/stand/basket/torso/head/pelvis_legs/other_arm, perfect tracking + 0.04 rad random sag; 3.5 min) | 70/70 end in `fence_streak`, nothing sent inside. Soft exit from the held command: lift 35, straight 34, hold 1. Recover from the sagged measured pose: lift 38, straight 32. **0 routes below min(self margin, start)** for torso/head/pelvis_legs/other_arm; 0 below the fence scene rule; **0 below 87's rule min(5 cm, start)** for table/stand/basket; ready pose and way-in-backwards tail identical in all 140. The old lift route breaks the self rule in 35/70 (down to -5.1 cm) (`probe_o2.log`) |
| Probe B: normal exits unchanged - every replay_demo state and action frame (726 + 726, demo's own layout) and all 1450 run1_45k chunk actions as the stop pose | **2902/2902 lift route**, i.e. identical waypoints to the old code (`probe_o2_demo_layout.log`) |
| Probe C (`runner_probe.py`): real 86 `Runner` + fakes + real fence, 8 replays driven into torso/head/other arm | 7 x `fence_streak` + 1 x `done`; all take `straight`; every sent to_ready message keeps the self groups >= min(margin, start) (lowest 2.02 cm); weight 1 until the final ramp-out, last weight 0, ends at q0; phases policy -> to_ready -> to_start -> ramp_out (`runner_probe.log`) |
| Mutants (`mutplug.py`, source-level, no repo file touched): no check / check without self groups / recover unchecked / refused route driven anyway / start = route minimum | all 5 caught (5, 5, 1, 2, 5 of the 6 new tests red); unmutated 6 passed (`mutants.log`) |
| `scripts/run_tests.sh` (once, ~4.5 min) | project 320 passed, sim env 589 passed, openpi 10 passed, exit 0 (`run_tests.log`) |
| 85 e2e | not run: my review changes no code on its path (the implementer's `/tmp/fence-o2/e2e/e3.log`: 1 passed) |
| leftover processes | none (`ps_end.txt`) |

### Minor (no NO-GO)

- **m1 - the new hold holds the measured angles, not the last command.** `exit_route_into_body` is not in
  `HOLD_LAST_COMMAND`, so `fault_targets` takes the measured pose. After a fence stop the arm sits at last command + sag;
  a target at the measured pose lets it sag once more (an unchecked pose, ~one tracking error toward gravity, near the
  body). The reason says nothing wrong about the arm (like the F5 reasons), so holding the last command - the pose the fence
  checked, cmd and pred - is the better still target. One-word fix: add it to `HOLD_LAST_COMMAND`. The unit test cannot see
  this (fake: measured = command); its docstring says "on the last pose".
- **m2 - the route check takes up to 252 ms on the M4 (median 58, p99 113), not 35-115 ms** as FENCE-SMOOTH.md 6.5 says:
  both routes checked, long straight route. No arm message goes out in that gap; `Runner.wait` restarts the schedule (no
  burst), LowState age is from the DDS callback so it does not go stale, and the exit clock starts after the check. I assume
  the G1 keeps the last arm_sdk target during a 0.25 s gap; I have not confirmed it. Fix the doc number; let 89 time it on
  the robot laptop.
- **m3 - `86 recover` (after a crash or kill) still drives the unchecked lift route from a logged policy/hold/shadow pose**
  (`real_run_modes.recover` -> `arm_paths.home`; that mode has no fence). This is the same O2 problem on a rare path. It is
  not listed under "Not covered" in 6.5. Ruling for Finnegan: build the fence there too (the sidecar has the layout), or a
  runbook line.
- **m4 - the 0.1 mm tolerance can turn a recover after a contact at an obstacle into a hold.** From a pose already inside
  a hull (example: replay_demo states 114-119 in the nominal layout, stand -1.06 ... -1.46 cm), the lift dips 0.1-0.5 mm
  deeper before it rises, so both routes are refused and 86 holds instead of recovering (F2). 87's `exit_passes` would fail
  these too, so it fits the rules; for the operator it means remote or Ctrl+C at the stand instead of a slow way home.
  Ruling: keep it, or allow a few mm more on scene groups when the start is already below 0.
- **m5 - test gap:** no test pins the straight route against table/stand/basket (Probe A covers it: 0 of 66 straight
  routes broke the 87 rule). It also falls under "Not covered" in 6.5.

### Checked and fine

`route_refused` = `refused_groups` along the route (below margin AND closer than start - 0.1 mm), every group, NaN refuses;
hands at the last command, waist roll/pitch measured, IMU tilt when R-F10; route check on exactly `[start] + path_ticks` up to
`ready_index` (lift 3, straight 1), same lines `soft_exit_plan` / `move_through` send (the recover's half speed samples the same
line); a check exception refuses -> hold; no fence -> old lift route (loopback `--no-fence` unchanged); `soft_exit` returns
the fault before `start_exit_clock`, `fault_exit` holds at weight 1 and only a signal ramps out; recover from a waypoint plan
(`arm_paths.home` -> `recover`) unchanged; a fault on the straight route goes on along the logged straight waypoints; replay
blend/polyline untouched; `ball_grasp.py`, training code and `docs/ROBOT_DAY_MOTION.json` untouched. The motion lock must be
re-unlocked by Finnegan for the changed modules.

Mac use for this review: about 12 min single-process CPU (probes 6.5 min, mutants 15 s, tests 4.5 min), never two sim
processes at once, no VLA model, no background process left.
