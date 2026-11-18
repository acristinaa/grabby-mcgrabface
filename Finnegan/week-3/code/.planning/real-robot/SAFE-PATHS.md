# Safe arm paths at the table: waypoints instead of straight lines (2026-11-04)

**Question.** 86 moves between the factory pose (hands hanging, below the 0.74 m table top) and the ready pose on straight
joint lines, so the hands swing forward while they are low. Every exit that ends in the factory pose (soft exit leg 2, fault
ramp-out, `86 release`) goes 5-16 cm into table or basket (`table-clearance-variant.md`). Are there waypoint paths that keep
every arm/hand geom >= 5 cm from table, stand and basket?

**Answer: partly.** In the sim kinematics, yes for the path itself: the waypoints add no contact for any factory pose
that is clear in every case. Where a way in is not clear, the factory pose itself is closer than 5 cm in its worst case;
the only rows with a clear start that still fail are elbow 0.0 / 0.1 at shoulder pitch -0.1 (forearm forward over a lowered
table, into the basket), poses that are not clear in their worst case anyway. What is clear, over 45 scene cases (torso
lean pitch -0.05 / 0.0 / 0.041 / 0.0835 / 0.126 rad x roll +-0.02 x table height -4 / 0 / +1 cm):

- **Way in** (factory -> ready): clear for every factory pose that is itself >= 5 cm from the table in every case.
- **Recover from the middle of the way in** (back along the legs, fix round, review B1): clear wherever the way in is clear.
- **Way out** (policy pose -> factory pose): clear at waist limit 0.6 for those same factory poses (5265 of 5265 start rows).
- **Not clear:** factory poses whose hanging fingers are within 5 cm of the table front in some case (elbow < 1.5 at
  shoulder pitch -0.1, < 1.2 at 0.1, < 0.8 at 0.3); and the waist columns 0.0 / 0.15 / 0.35. With those waist columns the
  replayed demo itself puts the hands into stand or basket before any exit starts.
- **Fix round 2026-11-04** (review `.planning/reviews/safe-paths-design.md`, B1-B4): recover depends on phase and leg (B1),
  the lean grid now starts at the upright end (B2; elbow 1.1 at shoulder pitch 0.1 drops out: 4.0 cm at pitch -0.05), the
  real M0 pose is checked on site instead of read from a grid row (B3, S1), and the 4 missing fault reasons are ruling F5 (B4).
- **Built into 86 (S4, 2026-11-04, not yet reviewed):** way in, soft exit, fault hold/recover and the new `86 recover` use
  these paths; the fault behaviours F1-F5 are the proposed defaults in `real_safety.FAULT_ACTIONS` and wait for rulings.

So `feasible` is **partly**, not yes.

## The waypoints and why (`g1sim/arm_paths.py`, one definition for 86 and 87)

Right arm, rad, order shoulder pitch / roll / yaw, elbow, wrist roll / pitch / yaw. The left arm is the mirror (roll and yaw
change sign). Signs found by FK: shoulder pitch + = back, roll - = out, yaw - = forearm out. Elbow 0 = forearm forward,
+ = towards straight.

| Name | Values | What the arm does | Why |
|---|---|---|---|
| `SWING_BACK_RIGHT` (partial) | pitch 0.5, roll 0.0, yaw -0.8; elbow and wrists stay as found | upper arm swings 29 deg back and turns out, hand hangs | the hand moves away from the table front; turning out makes the next elbow bend lift the hand sideways, not forward |
| `SIDE_UP_RIGHT_ARM` | 0.3, -0.6, -1.4, -0.8, 0.0, 0.6, 0.0 | hands up beside the body, forearms out and up, elbow/wrist already as in the ready pose | the hand rises above table top + margin while it is still behind the front edge (x < 0.20 m) |
| ready (`BALL_READY_POSE`) | -0.25, -0.05, 0.0, -0.8, 0.0, 0.6, 0.0 | unchanged | last leg = shoulders only: hands swing forward at chest height, above stand and basket |
| `LIFT_DELTA_RIGHT` (relative) | pitch -0.1, elbow -1.0, wrist pitch -0.5, each clipped 0.05 rad inside the joint limits | from any pose, hands rise and fold in | from the ball or the basket, the first motion goes up; turning the waist first sweeps the hands across stand/basket |

Paths (each leg is one `real_safety.move` line at 0.3 rad/s, the line 87 checks):
- **Way in** `way_in(factory, ready)`: factory -> swing back -> side up -> ready.
- **Way out** `way_out(current, factory, ready, lo, hi)`: current -> lift -> same pose with waist yaw home -> ready ->
  side up -> swing back -> factory. The second half is the way in backwards.
- **Recover** (`arm_paths.recover`, fix round 2026-11-04, review B1) depends on where the arm is, then 86's 2 s ramp-out at
  the factory pose. Unitree's controller is then asked to hold where the arm already is, so it has nothing to move near the
  table. This holds only if the M0 record matches what the controller holds; 86's 0.10 rad `factory_pose_gap` check covers that.

  | Where the fault strikes | Recover path | Why it is checked |
  |---|---|---|
  | `policy`, `hold`, `shadow` (no waypoint plan running) | the way out from the measured pose | lift_home rows + way-in rows |
  | way in, leg k | **back** along the legs already travelled: leg k's start, ..., the factory pose | recover rows (new): the same lines as the way in, backwards |
  | way out, leg k | **on** along the rest of the way out | legs 1-3: parts of lift_home lines; legs 4-6: the way in backwards, the same lines as a recover from the way in |

  The old prescription (the way out from the measured pose, whatever the phase) goes into the table from way-in leg 1 or early
  leg 2: -6.9 to -11.5 cm, worst of 27 cases, every start pose >= 9 cm clear (review probe P4,
  `/tmp/safepaths/review/probe_P4.txt`). This needs 86 to log the plan name and leg number it is on (S4).
- **Soft exit from `to_ready` / `hold` / `shadow`** (not from the policy phase): go back along the way-in legs. That is the
  same set of lines, so the way-in rows cover it. From the middle of a way-in leg it is `arm_paths.recover` (recover rows).

Rejected in the search (`runs/safe_paths_probes/`):
- One side waypoint without the swing back: the factory -> side-up leg still sweeps forward, -0.3 to +2.3 cm (probe4/5).
- The leg orders "lift -> arms to ready -> waist home" and "lift -> ready": 132 to 515 failing rows (probe11).
- Smaller or larger lifts: 1 to 232 failing rows at waist 0.6 (probe10/13).

## 87 evidence

Run: `87 --waypoint-paths` (new flag; code in `g1sim/arm_paths_clearance.py`). Each path is densified so that no arm point
moves more than 2 cm between samples (`max_sample_step` 2.00 cm, asserted). Clearance is capped at 10 cm.

```
~/dev/g1-ext/.venv/bin/python scripts/87_arm_path_clearance.py --waypoint-paths --report runs/safe_paths_waist0.6.json \
    --demo tests/data/replay_demo.npz --waist-limits 0.6 --setbacks 0.0,0.02,0.04,0.07,0.10,0.15,0.20,0.30   # 1069 s
# same with --waist-limits 0.35 / 0.15 / 0.0 and --setbacks 0.0 (about 290 s each, the four run in parallel)
```
Logs: `runs/safe_paths_waist{0.6,0.35,0.15,0.0}.{json,log}` (fix round re-run, 45 cases). Table `runs/safe_paths_tables.txt`,
made by the scratch extractor `/tmp/safepaths/fix/tables.py` (not in the repo, review m4).

### Way in (and the way out's second half), worst of 45 cases, per factory pose h(elbow, shoulder pitch)

"Clear at" = the smallest table setback where the way in and every recover from it pass in all 45 cases. The ball stays in
the policy's reach up to 0.07 m, and the stand stays on the table up to 0.15 m. This grid is a preview; S1 decides on the
real pose.

| Shoulder pitch | Elbows clear at the nominal table | Clear with the table 2-7 cm back (ball still in reach) | Need 10-30 cm back (policy out of reach) |
|---|---|---|---|
| -0.1 | 1.5, 1.6 (min 5.4 / 8.1 cm) | 1.4 (4 cm), 1.2, 1.3 (7 cm) | 0.0-1.1 |
| 0.1 | 1.2-1.6 (min 6.3 cm at 1.2, 8.8 at 1.3, else 8.9) | 1.1 (2 cm), 1.0 (4 cm), 0.8, 0.9 (7 cm) | 0.0-0.7 |
| 0.3 | 0.8-1.6 (min 5.5 cm at 0.8, 7.6 at 0.9, else 8.9) | 0.7 (2 cm), 0.6 (4 cm), 0.5 (7 cm) | 0.0-0.4 |

- Against the 27-case run, the poses whose closest point is the factory pose lost up to 1.9 cm (h(1.5, -0.1) 7.3 -> 5.4): the worst case is now lean pitch
  -0.05 (upright), pair `rh_right_thumb_tip|table`. That is review B2.
- For the factory poses that clear, the way in's closest point is either the factory pose itself or the last leg into the
  ready pose, never the waypoints in between:
  - at the factory pose: worst 5.4 cm (h(1.5, -0.1)), pair `rh_right_thumb_tip|table`;
  - on the last leg: never below 8.9 cm, pair `right_wrist_yaw_link|basket`.
- 11499 of the 11522 rows whose factory pose is >= 5 cm pass. The 23 failures are elbow 0.0 (20) and 0.1 (3) at shoulder
  pitch -0.1, setbacks 0-0.20 m, -2.5 to +4.9 cm, all basket: the forearm points forward and the hand is just above a
  lowered table. In their worst case these factory poses are about 2 cm inside the table (-2.1 cm), so no lookup passes them.
- Where the factory pose is inside or near the table (elbow 0.0-1.4), the swing-back leg can go up to 6.9 cm deeper than
  the pose. No path can start clear from such a pose anyway.
- Duration at 0.3 rad/s: 10.0-15.4 s (today's straight line for elbow 1.6: about 8 s).

### Recover from the middle of the way in (fix round, review B1)

A fault at 10 / 30 / 50 / 70 / 90 % of each way-in leg, then `arm_paths.recover` back to the factory pose, 15 per factory
pose, 45 cases, 8 setbacks (275400 rows). Pass: exit rule (never below min(5 cm, start)).
- Wherever the way in passes: 172470 of 172485 pass, worst margin +0.36 cm (h(1.5, -0.1): the factory pose at 5.36 cm is
  the end of the recover).
- The 15 others are one way-in row: h(0.6, 0.3) at setback 0.02, lean 0.041, roll -0.02, table -4 cm, where the factory pose
  is at 4.9996 cm. The way-in row reads it as passed only because its pass test uses minima rounded to 0.1 mm; the recover
  row uses the raw value. The "clear at" table above counts both, so it lists elbow 0.6 at pitch 0.3 at 4 cm, not 2 cm.
- The old prescription (the way out from that pose) on the same kind of start pose: -8.6 cm (h(1.6, 0.1), leg 1 at 10 %,
  default scene; `/tmp/safepaths/fix/probe_t1.txt`); the review's worst of 27 cases: -11.5 cm.

### Way out first half: lift -> waist home -> ready, from 87's start poses (exit rule: never below min(5 cm, start))

| Waist limit | Rows pass | Worst margin above the rule | Worst pair / case | Why it fails |
|---|---|---|---|---|
| 0.6 | **5265 / 5265** | **+0.06 cm** (passes 5.06 cm from the basket) | `right_elbow_link|basket`, frames 84-87, lean 0.041 | - |
| 0.35 | 4611 / 4635 | -0.3 cm | `lh_left_pinky|stand`, frames 109-115 | all 24 failing rows start **inside** the stand (18 start poses inside it at nominal) |
| 0.15 | 3399 / 3465 | -0.7 cm | `right_wrist_yaw_link|basket`, frames 106-152 | all 66 failing rows start inside stand/basket (14 / 16 start poses inside) |
| 0.0 (hold) | 2519 / 2655 | -1.5 cm | `rh_right_ring_distal|basket`, frames 119-302 | 112 of 136 start inside the basket, the rest within 0.33 cm of it (17 start poses inside) |

The two upright lean cases (-0.05, 0.0) add no failing lift_home row except 4 at waist 0.0: the policy-side paths get
tighter as the torso leans forward, the factory pose as it stands up.

- Below 0.6 the waist clip turns the demo into a replay that reaches for the ball without turning, so the hands land in the
  stand or the basket. That is a problem of the replay, not of the exit.
- For comparison, today's straight soft-exit leg 1 (policy pose -> ready): 207 of 356 start poses pass (-1.4 to -2.7 cm).
  Today's fault ramp-out to the factory pose: 0 of 18156 rows pass (-15.7 to -16.2 cm, `runs/arm_path_clearance_arms_up_first.json`,
  lean 0.126, nominal table). These are the real-scene fault-exit rows from review C P4.
- Duration: lift_home up to 8.4 s. Whole way out up to 8.4 + 15.4 = 23.8 s, plus the 2 s hands-open pause, plus the 2 s ramp-out.
  All legs run at 0.3 rad/s, within 86's caps (arms 1.0 rad/s, waist 0.3 rad/s).

### Per path and case, the paths that clear (waist 0.6, nominal table)

| Path | Cases | Min clearance | Worst pair |
|---|---|---|---|
| way in, factory h(1.6, 0.1) | 45 | 8.9 cm | `right_wrist_yaw_link|basket` (last leg) |
| way in, factory h(1.2, 0.1) | 45 | 6.3 cm (the factory pose) | `rh_right_thumb_tip|table` |
| way in, factory h(1.1, 0.1) | 45 | **4.0 cm, NOT clear** (the factory pose at lean -0.05; 5.6 cm in the 27-case run) | `rh_right_thumb_tip|table` |
| way in, factory h(0.8, 0.3) | 45 | 5.5 cm (the factory pose) | `rh_right_thumb_tip|table` |
| soft exit from the policy phase = way out | 45 x 117 start poses | rule margin +0.06 cm, then the way-in rows above | `right_elbow_link|basket` |
| recover from the way in (fault or `86 release` with a logged leg) | 45 x 15 per factory pose | rule margin >= +0.36 cm where the way in is clear | the factory pose |
| recover from the hold / shadow phase (`86 recover` only within `LEAD_LIMIT` of the logged ready pose, review B1-r) | same as the way out | same | same |
| recover from the policy phase | **`86 recover` refuses** (the phase logs no pose, ruling S6); a fault in the run itself still takes the way out | same | same |
| recover from the way out | parts of the lift_home lines, then the way-in rows | same | same |

**Own-body clearance** (probe, not in 87, `runs/safe_paths_self_clearance.txt`). The lift_home paths come no closer to the
robot than the demo poses: 4.07 vs 4.09 cm, `left_elbow_link|torso_link`. The legs after the swing back keep >= 4.15 cm.
The grid factory poses (shoulder roll 0) already overlap the hip collision geoms by 0.4-4.4 cm. That comes from the roll-0
grid; a real hanging pose does not intersect its own hips.

**Tests** (`tests/test_integration_arm_paths.py`, 7 tests, Proves/Catches). Each was seen failing on a mutant
(`runs/safe_paths_probes/mutation.log`, fix round: `/tmp/safepaths/fix/mutant_*.log`):
- SIDE_UP set to the ready pose;
- no lift;
- lift without the joint-limit clip;
- FK cache key without lean/height;
- recover on the way in replaced by the way out (B1);
- lean grid starting at 0.041 again (B2);
- `--factory-pose` without the lean refusal (B2/B3).

## What the arms do per fault reason - RULINGS for Finnegan

Rule 12 today: every fault ramps out at once. The factory controller then pulls the arms home along its own path, through
the table (-15.7 cm). Proposed instead:

| Reason | Proposed behaviour | Why | Ruling needed |
|---|---|---|---|
| `lowstate_stale` (no feedback > 0.1 s) | keep sending the last command at weight 1 (nothing moves). If LowState returns within a few seconds and the measured pose is within `LEAD_LIMIT` of the command -> recover (by phase and leg, table above). If not -> keep holding and tell the operator: owner's remote / e-stop. | We do not know where the arm is, so no planned path is safe. Holding still is. A ramp-out drops the arms through the table. | **F1:** hold instead of ramp-out? How long to wait? What happens if feedback never returns? (Today's assumption: Unitree latches our last message if our messages stop - not published.) |
| `contact_fault` (lead > 0.25 rad for 0.26 s) | command := measured angles at weight 1 (stops pushing at once, like today's `fault_targets`), short hold, then recover from the measured pose (by phase and leg) at half speed (0.15 rad/s) with the lead monitor on. A second contact fault -> hold, owner's remote. | The arm is blocked by something. Recover moves it back the way it came (way in) or up and away (policy phase). A ramp-out would drag it along the factory path. | **F2:** automatic recover after the hold, or only after the operator confirms? (Rule 12 says no exit depends on the terminal.) |
| `invalid_target` (NaN/Inf, never sent) | recover from the measured pose by phase and leg (scripted path, no policy) | LowState is fine and the arm sits at the last valid command, so the planned path is known. | **F3:** treat it like a soft exit through the recover path |
| `second_signal` (2nd Ctrl+C) | freeze: hold at weight 1. A 3rd signal = ramp out now (the operator accepts the factory path). | The operator wants it to stop. "Stop" means hold still, not "drop the arms onto the table". | **F4:** freeze, or recover at once? Rule 12's "a third signal does not skip the ramp" changes. |
| `internal_error` (any exception in `_drive` or `soft_exit`), `exit_path_overrun` (exit clock past planned + 1 s, `real_run.py:206`), `weight_step` (`:204`), `signal_in_release` (`:233`) | today: ramp out at once, and the factory controller takes the arms 15.7-16.2 cm into the table from a policy pose. Proposed: hold at weight 1 on the last command (nothing moves), tell the operator; the operator decides (owner's remote / e-stop, or `86 release` = recover by the logged phase and leg). | These 4 reasons were missing from F1-F4 (review B4). None of them says where the arm is wrong; dropping the arms is the one known collision. `exit_path_overrun` gets more likely with these paths (up to 23.8 s, F2's half speed doubles a leg), so 86's exit clock must be planned from `arm_paths.path_seconds` at the speed actually used. | **F5:** hold + operator for these 4? An `internal_error` may come from the send loop itself: if 86 cannot send, it cannot hold either (then Unitree's latch decides, as for F1). |

Further rulings:
- **S1 (layout gate, changed in the fix round, review B2/B3):** the grid table above is a preview only. Table steps run
  only if, on site after M0, 87 on the recorded pose itself says "way in clear" and "recover from the way in clear" at the
  waist column and table position used:
  ```
  ~/dev/g1-ext/.venv/bin/python scripts/87_arm_path_clearance.py --waypoint-paths --factory-pose runs/factory_pose_robot.json \
      --report runs/safe_paths_onsite.json --demo tests/data/replay_demo.npz --waist-limits 0.6 --setbacks 0.0   # ~40-130 s
  ```
  Why: a grid row does not stand in for the poses next to it. Three poses the grid lookup reads as clear (row h(1.1, 0.1),
  5.6 cm) are not, on the same 45 cases (`/tmp/safepaths/fix/onsite_*.log`): wrist pitch -0.15 (inside the 0.15 rad
  tolerance) 2.8 cm, waist yaw +0.15 2.2 cm, elbow 1.1 at shoulder pitch 0.0 (between rows) 0.1 cm. h(1.6, 0.1) reads the
  same on site as in the grid (8.9 cm, clear). The run refuses a recorded q0 waist pitch outside -0.05..0.041 rad or roll
  outside +-0.02 ("skip table steps"). It needs the sim env on the laptop that runs it. If it cannot run: skip table steps.
  Otherwise use arms-up-first with the table >= 30 cm back, and pull the table back before any recover/release.
- **S5 (rule 16 (a) lower bound, new, review B2):** rule 16 (a) gets a lower bound for the M0 q0 waist pitch, -0.05 rad
  (`arm_paths_clearance.WAIST_PITCH_Q0_MIN`); below it, skip the table steps. Rule 16 (a)'s sentence "a smaller or negative
  q0 pitch only adds clearance" is wrong for the factory pose, where the arms hang and the torso sits at about q0:
  h(1.1, 0.1) is 4.00 cm at -0.05, 4.88 at 0.0, 5.65 at 0.041 (review probe P3b). PLAN.md is not changed until you rule.
- **S2 (waist column):** only waist 0.6 clears at the table. 0.0 / 0.15 / 0.35 make the replay itself collide. That
  conflicts with the robot default `--waist-max 0.15` and with rule 16 (b) "never above the M3 limit".
- **S3 (thin margin):** the way out passes at 5.06 cm (right elbow past the basket rim). Accept it, or move the basket a few cm
  right on the real table?
- **S4 (86 build):** put these paths into 86 (`move_to_ready`, `soft_exit`, `fault_exit`, `release`) as a reviewed step with
  an 85 sim gate. Until then the runbook's "skip table steps" stands. Needed for B1/B4: 86 writes the plan name
  (`arm_paths.WAY_IN` / `WAY_OUT`) and leg number to the sidecar at every leg start (flushed, so it survives a crash);
  recover and `86 release` call `arm_paths.recover` with it; without a logged leg near the table, `86 release` refuses and
  the runbook sends the operators to the owner's remote. 86's exit deadline comes from `arm_paths.path_seconds` at the speed
  actually used (0.3 or F2's 0.15 rad/s).
- **S6 (`86 recover` after a policy-phase log, new, review safe-paths-code B1-r (b)):** the policy phase logs no arm pose,
  so `86 recover` cannot check that the arms are still where the run left them (a kill, then the owner's remote: 5-14 cm
  into the table in the review probe). Default built: refuse (`real_preflight.RECOVER_FROM_POLICY_LOG = False`); the
  operators then use the owner's remote, or pull the table back and run `86 release`. Alternative for you to rule: 86 logs
  the last sent command to the sidecar about once a second, and recover accepts within `LEAD_LIMIT` plus the policy speed cap
  times that interval (not built).

## Still not clear

- 86 uses the paths since the S4 build (2026-11-04); until its review is GO and the rulings are made, the runbook's
  "skip table steps" stands.
- `86 recover` refuses, with nothing sent, unless the log still describes the arms (review safe-paths-code B1, B1-r,
  `real_preflight.arm_path_problem`): no logged arm path; our last run already handed the arms back (weight 0); the arms are
  where that run took them over (then: `86 release`); a hold/shadow log and the arms more than `LEAD_LIMIT` from its ready
  pose; a policy log (S6); a waypoint log and the arms more than `LEAD_LIMIT` off its leg. Not built: logging the policy
  pose (S6 alternative), so after a policy-phase crash only the owner's remote or table back + `86 release` remain.
- The way out is checked only from demo poses of one episode at the nominal layout. Not checked: policy (M6) poses and the
  2-7 cm setback rows (lift_home was computed at setback 0 only). A fault in the middle of the way in now has recover rows;
  they start on the planned line, not at a measured pose up to `LEAD_LIMIT` off it (review m3).
- The factory grid has shoulder roll/yaw 0; the M0 bounds line allows +-0.15 rad (rule 16 (a)). The real elbow and pitch are
  unknown until M0; the on-site `--factory-pose` run (S1) checks the real ones.
- Not modelled: the ball, own-body collision (only probed), the real torso sag (sim number), and tracking lag of a 0.3 rad/s
  move (assumed inside the 5 cm). The table is the sim's solid collision block, so a hand under a real table top reads as
  "inside".
- The old 87 blocks still use a single lean of 0.126 rad. Review C r1 is handled only in `--waypoint-paths` (now from
  -0.05 rad, review B2). Review C m6 and m8 and the minors m1-m8 of `.planning/reviews/safe-paths-design.md` are untouched.
